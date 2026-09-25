#include "ImageMatcher.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include "Utils.h"


// pts1, pts2 are matched pixel points (same length), already filtered by inliers if possible
// K is 3x3 double, R is 3x3 double from recoverPose (rotation from cam1 -> cam2)

// Projects a 3D ray x through intrinsics K onto the image plane (pinhole projection).
static inline cv::Point2f projectPoint(const cv::Mat& K, const cv::Vec3d& x)
{
    double X = x[0], Y = x[1], Z = x[2];
    double u = (K.at<double>(0,0) * (X/Z)) + K.at<double>(0,2);
    double v = (K.at<double>(1,1) * (Y/Z)) + K.at<double>(1,2);
    return cv::Point2f((float)u, (float)v);
}

// Estimates the pixel-space translation residual left over after "undoing" the known
// rotation R: for each inlier match, unprojects pts1[i], rotates it by R, reprojects
// it back to pixels, and measures how far that rotation-only prediction is from the
// observed pts2[i]. Returns the median (robust to outliers) residual over both axes;
// (0,0) if there are fewer than 8 usable points. Used by getAlignmentDisplacement()
// as a cheap alternative to a full essential-matrix decomposition.
cv::Point2f rotationCompensatedResidual(
    const std::vector<cv::Point2f>& pts1,
    const std::vector<cv::Point2f>& pts2,
    const cv::Mat& K,
    const cv::Mat& R,
    const cv::Mat& inlierMask // optional: CV_8U mask (Nx1) from findEssentialMat/recoverPose
)
{
    cv::Mat Kinv = K.inv();

    std::vector<float> rx, ry;
    rx.reserve(pts1.size());
    ry.reserve(pts1.size());

    for (size_t i = 0; i < pts1.size(); ++i)
    {
        if (!inlierMask.empty() && inlierMask.at<uchar>((int)i) == 0)
            continue;

        // p1 -> normalized ray
        cv::Vec3d p1(pts1[i].x, pts1[i].y, 1.0);
        cv::Vec3d x = cv::Vec3d(
            Kinv.at<double>(0,0)*p1[0] + Kinv.at<double>(0,1)*p1[1] + Kinv.at<double>(0,2)*p1[2],
            Kinv.at<double>(1,0)*p1[0] + Kinv.at<double>(1,1)*p1[1] + Kinv.at<double>(1,2)*p1[2],
            Kinv.at<double>(2,0)*p1[0] + Kinv.at<double>(2,1)*p1[1] + Kinv.at<double>(2,2)*p1[2]
        );

        // rotate ray
        cv::Vec3d xr(
            R.at<double>(0,0)*x[0] + R.at<double>(0,1)*x[1] + R.at<double>(0,2)*x[2],
            R.at<double>(1,0)*x[0] + R.at<double>(1,1)*x[1] + R.at<double>(1,2)*x[2],
            R.at<double>(2,0)*x[0] + R.at<double>(2,1)*x[1] + R.at<double>(2,2)*x[2]
        );

        // project back to pixels (rotation-only prediction)
        cv::Point2f p_rot = projectPoint(K, xr);

        // residual (what rotation can't explain)
        cv::Point2f r = pts2[i] - p_rot;

        rx.push_back(r.x);
        ry.push_back(r.y);
    }

    if (rx.size() < 8) return cv::Point2f(0,0);

    auto median = [](std::vector<float>& v)->float {
        size_t n = v.size()/2;
        std::nth_element(v.begin(), v.begin()+n, v.end());
        return v[n];
    };

    float dx = median(rx);
    float dy = median(ry);
    return cv::Point2f(dx, dy);
}


// Estimates a scalar "zoom" (approach/recede) signal as the median ratio of each
// inlier match's distance from the image center in pts2 vs. pts1, returned as
// log(ratio) so it's symmetric around 0. KNOWN LIMITATION: despite the name/K,R
// params, this does NOT actually rotate pts1 before comparing (see the inlined
// /*p_rot*/ note below) — it only compares raw radial distances, so a pure rotation
// (no zoom) can still produce a nonzero result. Returns 0 if fewer than 8 usable
// points.
float rotationCompensatedZoom(
    const std::vector<cv::Point2f>& pts1,
    const std::vector<cv::Point2f>& pts2,
    const cv::Mat& K,
    const cv::Mat& R,
    const cv::Mat& inlierMask
){
    cv::Mat Kinv = K.inv();
    cv::Point2f c((float)K.at<double>(0,2), (float)K.at<double>(1,2));

    std::vector<float> ratios;
    for (size_t i=0;i<pts1.size();++i){
        if (!inlierMask.empty() && inlierMask.at<uchar>((int)i)==0) continue;

        // predict p_rot from pts1 exactly like above...
        // (call the same code and get p_rot)

        // r1 = distance to center in image1, r2 = distance to center in image2 after rotation compensation
        float r1 = cv::norm(pts1[i] - c);
        float r2 = cv::norm(pts2[i] - /*p_rot*/ c); // <-- better: compare to predicted rotation position
        if (r1 > 1e-3f) ratios.push_back(r2 / r1);
    }
    if (ratios.size() < 8) return 0.f;
    std::nth_element(ratios.begin(), ratios.begin()+ratios.size()/2, ratios.end());
    float s = ratios[ratios.size()/2];
    return std::log(std::max(s, 1e-6f));
}


// Loads the target image and converts it to grayscale. Note: this does NOT populate
// targetKeypoints/targetDescriptors (SIFT features of the target) — nothing in the
// current codebase does. That's harmless for the live pipeline, since
// getAlignment(matches, frame) (the only entry point main.cpp calls) takes
// pre-computed SuperPoint/LightGlue matches and never touches targetKeypoints. But it
// means the SIFT-based paths that DO read targetKeypoints/targetDescriptors —
// getAlignmentDisplacement(), getAlignmentDisplacementRansac(), getAlignmentDirection(),
// getAlignmentOld() — are effectively broken (they'll always match against an empty
// target) unless something is added to detect/compute target features first.
ImageMatcher::ImageMatcher(Logger& logger, const std::string& targetImagePath, const cv::Mat& K_cv): logger(logger)
{
    cameraMatrix = K_cv.clone();
    cv::Mat K_64;
    K_cv.convertTo(K_64, CV_64F);
    K << K_64.at<double>(0,0), K_64.at<double>(0,1), K_64.at<double>(0,2),
        K_64.at<double>(1,0), K_64.at<double>(1,1), K_64.at<double>(1,2),
        K_64.at<double>(2,0), K_64.at<double>(2,1), K_64.at<double>(2,2);
    cv::Mat target = cv::imread(targetImagePath, cv::IMREAD_COLOR);
    if (target.empty()) {
        throw std::runtime_error("Could not load target image");
    }
    cv::cvtColor(target, targetImageGray, cv::COLOR_BGR2GRAY);
    {
        std::ostringstream ss;
        ss << "Target image size: " << targetImageGray.cols << "x" << targetImageGray.rows;
        logger.log("ImageMatcher", ss.str());
    }
}

// Plain SIFT detect+compute, no spatial filtering.
void ImageMatcher::detectAndCompute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
    sift->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
}
// Unused placeholder — see header comment.
void ImageMatcher::detectAndComputeLKFlow(){

}

// SIFT detect, then bucket keypoints into a gridX x gridY grid and keep only the
// maxPerCell strongest (by response) per cell before computing descriptors. This
// keeps keypoints from clustering entirely in one high-texture region of the image.
void ImageMatcher::detectAndComputegrid(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                                    cv::Mat& descriptors, int gridX, int gridY, int maxPerCell)
{
    // 1. Detect all keypoints
    std::vector<cv::KeyPoint> allKeypoints;
    sift->detect(image, allKeypoints);

    // 2. Divide image into grid
    int cellW = image.cols / gridX;
    int cellH = image.rows / gridY;
    std::vector<std::vector<cv::KeyPoint>> grid(gridX * gridY);

    for (auto& kp : allKeypoints)
    {
        int ix = std::min(int(kp.pt.x / cellW), gridX - 1);
        int iy = std::min(int(kp.pt.y / cellH), gridY - 1);
        grid[iy * gridX + ix].push_back(kp);
    }

    // 3. Select top keypoints per cell
    keypoints.clear();
    for (auto& cell : grid)
    {
        // Sort by response (strength)
        std::sort(cell.begin(), cell.end(),
                  [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                      return a.response > b.response;
                  });
        for (int i = 0; i < std::min(maxPerCell, (int)cell.size()); i++)
            keypoints.push_back(cell[i]);
    }

    // 4. Compute descriptors for selected keypoints
    sift->compute(image, keypoints, descriptors);
    {
        std::ostringstream ss;
        ss << "All Keypoints detected: " << keypoints.size();
        logger.log("ImageMatcher", ss.str());
    }
}


// SIFT-based alignment: matches inputImage against the target and returns a raw 2D
// pixel-space displacement (average match motion vector) plus a crude in/out "zMotion"
// heuristic (based on whether matches move toward or away from the image center). Not
// called by the live pipeline — see the KNOWN LIMITATION note on the constructor above.
cv::Point3f ImageMatcher::getAlignmentDisplacement(const cv::Mat& inputImage) {
    cv::Mat inputGray;
    cv::cvtColor(inputImage, inputGray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> inputKeypoints;
    cv::Mat inputDescriptors;
    detectAndComputegrid(inputGray, inputKeypoints, inputDescriptors);

    std::vector<cv::DMatch> goodMatches;
    {
        std::ostringstream ss;
        ss << "Target image keypoint size: " << targetKeypoints.size();
        logger.log("ImageMatcher", ss.str());
    }
    {
        std::ostringstream ss;
        ss << "Input image keypoint size: " << inputKeypoints.size();
        logger.log("ImageMatcher", ss.str());
    }
    goodMatches = goodMatcher(inputDescriptors);
    // goodMatches = gridFilterMatches(matches, inputKeypoints);
    {
        std::ostringstream ss;
        ss << "Found good matches of size: " << goodMatches.size();
        logger.log("ImageMatcher", ss.str());
    }
    if (goodMatches.empty()) return cv::Point3f(0,0,0);
    
    cv::Point2f direction2D(0,0);
    float zMotion = 0; // inward/outward
    cv::Point2f center(inputGray.cols/2.0f, inputGray.rows/2.0f);

    for (const auto& m : goodMatches) {
        const cv::KeyPoint& kpInput = inputKeypoints[m.queryIdx];
        const cv::KeyPoint& kpTarget = targetKeypoints[m.trainIdx];
        inputMatches.push_back(kpInput.pt);
        targetMatches.push_back(kpTarget.pt);

        // 2D translation
        direction2D += (kpTarget.pt - kpInput.pt);

        // Inward/outward: dot product with vector from center
        cv::Point2f vecToCenter = kpInput.pt - center;
        cv::Point2f motionVec = kpTarget.pt - kpInput.pt;
        float dot = vecToCenter.dot(motionVec);

        // If dot > 0 → moving outward, dot < 0 → moving inward
        zMotion += (dot > 0) ? -1.0f : 1.0f;
    }

    direction2D.x /= goodMatches.size();
    direction2D.y /= goodMatches.size();
    zMotion /= goodMatches.size(); // average tendency
    return cv::Point3f(direction2D.x, direction2D.y, zMotion);
}

// KNN-matches inputDescriptors against targetDescriptors (k=2) and keeps only matches
// that pass Lowe's ratio test (best match clearly better than second-best), which
// filters out ambiguous matches in repetitive/low-texture regions.
std::vector<cv::DMatch> ImageMatcher::goodMatcher(const cv::Mat& inputDescriptors) {
    // KNN match to find the two best matches for each descriptor
    std::vector<std::vector<cv::DMatch>> matchesAB;
    matcherFlann.knnMatch(inputDescriptors, targetDescriptors, matchesAB, 2);
    {
        std::ostringstream ss;
        ss << "Total matches found: " << matchesAB.size();
        logger.log("ImageMatcher", ss.str());
    }
    // Apply Lowe's ratio test
    const float ratio = 0.8f;

    std::vector<cv::DMatch> goodAB;

    for (const auto& m : matchesAB){
        if (m.size() == 2 && m[0].distance < ratio * m[1].distance)
            goodAB.push_back(m[0]);
    }
    {
        std::ostringstream ss;
        ss << "Good matches after ratio test: " << goodAB.size();
        logger.log("ImageMatcher", ss.str());
    }
    return goodAB;
}


// Buckets matches into a gridCols x gridRows grid (by their query-image position) and
// keeps only the maxPerCell best (lowest descriptor distance) matches per cell, so
// the surviving matches are spread across the image instead of clustering wherever
// SIFT found the most texture.
std::vector<cv::DMatch> ImageMatcher::gridFilterMatches(const std::vector<cv::DMatch>& matches,
                                                        const std::vector<cv::KeyPoint>& queryKps,
                                                        int gridCols, int gridRows, int maxPerCell)
{
    float cellW = width  / gridCols;
    float cellH = height / gridRows;

    // grid of matches per cell
    std::vector<std::vector<cv::DMatch>> grid(gridCols * gridRows);

    for (const auto& m : matches) {
        cv::Point2f pt = queryKps[m.queryIdx].pt;
        int col = std::clamp((int)(pt.x / cellW), 0, gridCols - 1);
        int row = std::clamp((int)(pt.y / cellH), 0, gridRows - 1);
        grid[row * gridCols + col].push_back(m);
    }

    // Sort each cell by distance and keep top N
    std::vector<cv::DMatch> filtered;
    for (auto& cell : grid) {
        std::sort(cell.begin(), cell.end(),
            [](const cv::DMatch& a, const cv::DMatch& b) {
                return a.distance < b.distance;
            });
        for (int i = 0; i < std::min(maxPerCell, (int)cell.size()); i++)
            filtered.push_back(cell[i]);
    }

    return filtered;
}


// Computes the essential matrix from the member inputMatches/targetMatches (RANSAC)
// and recovers the winning (bestR, bestT) pose from it via cv::recoverPose.
void ImageMatcher::findAnddecomposeEssentialMat(cv::Mat& bestR, cv::Mat& bestT ){


    cv::Mat inliersE;
    cv::Mat EssentialMat = cv::findEssentialMat(
                                inputMatches,     // std::vector<cv::Point2f> from current frame
                                targetMatches,    // std::vector<cv::Point2f> from target image
                                cameraMatrix,    // 3x3 intrinsic matrix
                                cv::RANSAC,      // method
                                0.999,           // confidence
                                0.1,              // reprojection threshold in pixels
                                inliersE
                            );
    // Now compute translation-like residual in pixels:
    cv::Point2f dxy = rotationCompensatedResidual(inputMatches, targetMatches, cameraMatrix, bestR, inliersE);
    float dz = rotationCompensatedZoom(inputMatches, targetMatches, cameraMatrix, bestR, inliersE);
    dxyz.x = dxy.x;
    dxyz.y = dxy.y;
    dxyz.z = dz;
}


// SIFT-based alignment (not called by the live pipeline — see the KNOWN LIMITATION
// note on the constructor above). If inputImage is given, re-detects/matches SIFT
// features against the target; if it's empty, reuses whatever inputMatches/
// targetMatches are already set (e.g. by a previous call). Then:
//   1. Fits both a homography H and an essential matrix in parallel and compares
//      their inlier counts (tau_H) to gauge whether the motion looks like a pure
//      rotation (homography explains it well) or has real translation.
//   2. Estimates the focus-of-expansion (FOE); a high residual also implies
//      rotation-only motion (no clear expansion/contraction center).
//   3. NOTE: the homography/"pure rotation" branch below is permanently disabled via
//      "&& 0" — only the essential-matrix + recoverPose() branch ever runs. Whether
//      that's deliberate (the homography path was found unreliable) or leftover from
//      debugging is worth confirming before relying on tau_H/rotationOnlyFlag again.
std::tuple<cv::Mat, cv::Point3f, cv::Point2f, float, bool> ImageMatcher::getAlignmentDirection( const cv::Mat& inputImage, bool rotationOnly){

    if(inputImage.empty() && inputMatches.size() == 0) return {cv::Mat::eye(3, 3, CV_32F), cv::Point3f(0,0,0), cv::Point2f(0,0), std::numeric_limits<float>::infinity(), false};

    if (!inputImage.empty()){

        cv::cvtColor(inputImage, inputImageGray, cv::COLOR_BGR2GRAY);
        std::vector<cv::KeyPoint> inputKeypoints;
        cv::Mat inputDescriptors;

        std::vector<cv::Point2f>().swap(inputMatches);
        std::vector<cv::Point2f>().swap(targetMatches);
        cv::Mat inputDesc;
        detectAndComputegrid(inputImageGray, inputKeypoints, inputDesc);
        inputDesc.convertTo(inputDescriptors, CV_32F);
        inputDesc.release();
        std::vector<cv::DMatch> goodMatches;
        auto matches = goodMatcher(inputDescriptors);
        goodMatches = gridFilterMatches(matches, inputKeypoints);

        if (goodMatches.empty()) return {cv::Mat::eye(3, 3, CV_32F), cv::Point3f(0,0,0), cv::Point2f(0,0), std::numeric_limits<float>::infinity(), false};
        for (const auto& m : goodMatches) {
            const cv::KeyPoint& kpInput = inputKeypoints[m.queryIdx];
            const cv::KeyPoint& kpTarget = targetKeypoints[m.trainIdx];

            inputMatches.push_back(kpInput.pt);
            targetMatches.push_back(kpTarget.pt);
        }

        // Setup termination criteria (Max 30 iterations or 0.01 epsilon)
        cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01);

        // Refine the points
        // winSize is the search window (5x5 or 11x11 is standard)
        cv::cornerSubPix(inputImageGray, inputMatches, cv::Size(5, 5), cv::Size(-1, -1), criteria);
        cv::cornerSubPix(targetImageGray, targetMatches, cv::Size(5, 5), cv::Size(-1, -1), criteria);
    }

    cv::Mat rotationMatrix = cv::Mat::eye(3, 3, CV_32F);
    cv::Point3f world_direction = cv::Point3f(0,0,0);
    float meanError = std::numeric_limits<float>::infinity();
    cv::Point2f flow = cv::Point2f( std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
    // inputImageGray.copyTo(oldImageGray);

    if(inputMatches.size() < 50){
        logger.log("ImageMatcher", "Not enough Matches found (atleast 50)");
        return {rotationMatrix, world_direction, flow, meanError, false};
    }
    bool rotationOnlyFlag = false;
    cv::Mat maskH;
    cv::Mat H = cv::findHomography(inputMatches, targetMatches, cv::RANSAC, 5.0, maskH, 2000, 0.999);
    int hInliers = H.empty() ? 0 : cv::countNonZero(maskH);

    cv::Mat maskE;
    cv::Mat EssentialMat = cv::findEssentialMat(inputMatches, targetMatches, cameraMatrix, cv::RANSAC, 0.999, 2.0, 2000, maskE);
    int eInliers = EssentialMat.empty() ? 0 : cv::countNonZero(maskE);

    float tau_H = hInliers / (hInliers + eInliers + 1e-6f);

    FOEResult foe = computeFOE(inputMatches, targetMatches);
    // bool foeInFrame = (foe.point.x >= 0 && foe.point.x <= imgWidth && foe.point.y >= 0 && foe.point.y <= imgHeight);
    if (foe.residual > 10) rotationOnlyFlag = true;
    if (rotationOnly) rotationOnlyFlag = true;

    {
        std::ostringstream ss;
        ss << "FOE: (" << foe.point.x << ", " << foe.point.y << "), residual: " << foe.residual
           << ", rotation only: " << rotationOnlyFlag;
        logger.log("ImageMatcher", ss.str());
    }
    {
        std::ostringstream ss;
        ss << "Homography inliers: " << hInliers << ", Essential inliers: " << eInliers
           << ", tau_H: " << tau_H;
        logger.log("ImageMatcher", ss.str());
    }

    // H = [ s*cosθ  -s*sinθ  tx ]
    //     [ s*sinθ   s*cosθ  ty ]

    if ( (!H.empty() && ((tau_H > 0.7) || rotationOnlyFlag)) && 0) {
        cv::Mat bestR = computeRotation(H, hInliers, maskH);
        cv::transpose(bestR, rotationMatrix);
    }
    else{

        cv::Mat bestR, bestT, bestT64, inliersE;
        int inlierCount = cv::recoverPose(EssentialMat, inputMatches, targetMatches, cameraMatrix, bestR, bestT64, inliersE);
        std::vector<cv::Point2f> inliers1, inliers2;
        for(int i = 0; i < inliersE.rows; i++) {
            if(inliersE.at<uchar>(i)) {
                inliers1.push_back(inputMatches[i]);
                inliers2.push_back(targetMatches[i]);
            }
        }
        meanError = getReprojectionError(inliers1, inliers2, bestR, bestT64);
        flow = getOpticalFlow(inliers1, inliers2);
        float flow_norm = std::sqrt(flow.x*flow.x + flow.y*flow.y);
        rotationMatrix = bestR.clone();
        cv::transpose(bestR, rotationMatrix);
        // bestT64.convertTo(bestT, CV_32F);
        cv::Mat bestT_ = - bestR.t() * bestT64;
        bestT_.convertTo(bestT, CV_32F);
        world_direction = cv::Point3f(bestT.at<float>(0,0)*flow_norm, bestT.at<float>(1,0)*flow_norm, bestT.at<float>(2,0)*flow_norm);

        if ((std::abs(bestT.at<float>(2,0)) > std::abs(bestT.at<float>(1,0)) + std::abs(bestT.at<float>(0,0))) && flow_norm < 10) world_direction.z *= 10;
        meanError = flow_norm;
    }
    {
        std::ostringstream ss;
        ss << "Estimated translation magnitude (mean error): " << meanError;
        logger.log("ImageMatcher", ss.str());
    }
  
    return {rotationMatrix, world_direction, flow, meanError, true};
}

// Mean per-axis displacement (pts2 - pts1) over all correspondences.
cv::Point2f ImageMatcher::getOpticalFlow(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2) {
    cv::Point2f flow(0, 0);
    int count = 0;
    for (size_t i = 0; i < pts1.size(); ++i) {
            flow += cv::Point2f(pts2[i].x - pts1[i].x, pts2[i].y - pts1[i].y);
            count++;
    }
    if (count > 0) {
        flow.x /= count;
        flow.y /= count;
    }
    return flow;
    
}


// Triangulates each pts1<->pts2 correspondence under the rigid transform (Rf, tf)
// and returns the mean symmetric reprojection error (pixels) across both views,
// skipping points that fail the cheirality test (behind either camera) or land at
// infinity. Returns a large penalty (1e6) if no points pass the depth checks.
float ImageMatcher::getReprojectionError(const std::vector<cv::Point2f>& pts1,
                                         const std::vector<cv::Point2f>& pts2,
                                         const cv::Mat& Rf, const cv::Mat& tf)
{
    if (pts1.empty() || pts1.size() != pts2.size()) return 0.0f;

    // Enforce float precision consistently
    cv::Mat R, t, K;
    Rf.convertTo(R, CV_32F);
    tf.convertTo(t, CV_32F);
    cameraMatrix.convertTo(K, CV_32F);

    // 1. Build Projection Matrices
    // P1 = K * [I | 0]
    cv::Mat P1 = cv::Mat::zeros(3, 4, CV_32F);
    // cv::Mat::eye(3, 3, CV_32F).copyTo(P1(cv::Rect(0, 0, 3, 3)));
    P1(cv::Rect(0, 0, 3, 3)) = cv::Mat::eye(3, 3, CV_32F);
    P1 = K * P1;

    // P2 = K * [R | t]
    cv::Mat P2 = cv::Mat::zeros(3, 4, CV_32F);
    R.copyTo(P2(cv::Rect(0, 0, 3, 3)));
    t.copyTo(P2(cv::Rect(3, 0, 1, 3)));
    P2 = K * P2;

    // 2. Triangulate 3D Points
    cv::Mat pts4D;
    cv::triangulatePoints(P1, P2, pts1, pts2, pts4D);

    float totalError = 0.0f;
    int validCount = 0;

    for (int i = 0; i < pts4D.cols; i++) {
        float w = pts4D.at<float>(3, i);
        if (std::abs(w) < 1e-6f) continue; // Guard: skip points at infinity

        // Convert from Homogeneous [X, Y, Z, W] to 3D Cartesian X1 = [x, y, z] in Cam 1 frame
        cv::Mat X1 = (cv::Mat_<float>(3,1) << pts4D.at<float>(0, i) / w, 
                                               pts4D.at<float>(1, i) / w, 
                                               pts4D.at<float>(2, i) / w);

        // Guard: Cheirality test (3D point must be in front of Camera 1)
        if (X1.at<float>(2) <= 0.0f) continue;

        // Transform point to Camera 2 frame: X2 = R*X1 + t
        cv::Mat X2 = R * X1 + t;
        
        // Guard: Cheirality test (3D point must be in front of Camera 2)
        if (X2.at<float>(2) <= 0.0f) continue;

        // 3. Project to Image 1 Plane: x1 = K * X1
        cv::Mat x1_hom = K * X1;
        cv::Point2f proj1(x1_hom.at<float>(0) / x1_hom.at<float>(2),
                           x1_hom.at<float>(1) / x1_hom.at<float>(2));

        // 4. Project to Image 2 Plane: x2 = K * X2
        cv::Mat x2_hom = K * X2;
        cv::Point2f proj2(x2_hom.at<float>(0) / x2_hom.at<float>(2),
                           x2_hom.at<float>(1) / x2_hom.at<float>(2));

        // 5. Symmetric Reprojection Error (L2 Euclidean distance)
        float err1 = cv::norm(proj1 - pts1[i]);
        float err2 = cv::norm(proj2 - pts2[i]);

        // Mean error across both views for this point
        totalError += (err1 + err2) * 0.5f;
        validCount++;
    }

    if (validCount == 0) return 1e6f; // High penalty error if no valid points pass depth check

    return totalError / validCount;
}

// Builds a 4x4 homogeneous transform [R | t; 0 0 0 1].
cv::Mat ImageMatcher::formTransf(const cv::Mat& R, const cv::Mat& t) {
    cv::Mat T = cv::Mat::eye(4, 4, CV_32F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    t.copyTo(T(cv::Rect(3, 0, 1, 3)));
    return T;
}

// Triangulates inputMatches/targetMatches under (Rotation, translation) and counts
// how many resulting 3D points have positive depth (Z) in both camera views — a
// relative-scale/cheirality sanity signal (higher = more consistent geometry).
int ImageMatcher::sumZCalRelativeScale(const cv::Mat& Rotation, const cv::Mat& translation) {
        // Form transformation matrix
        cv::Mat T = ImageMatcher::formTransf(Rotation, translation);
        // Projection matrices
        cv::Mat P0 = cameraMatrix * cv::Mat::eye(3, 4, CV_32F);
        cv::Mat P1 = cameraMatrix * T(cv::Rect(0, 0, 4, 3));

        // Convert points to cv::Mat (2xN)
        cv::Mat pts1(2, inputMatches.size(), CV_32F);
        cv::Mat pts2(2, targetMatches.size(), CV_32F);
        for (size_t i = 0; i < inputMatches.size(); i++) {
            pts1.at<float>(0, i) = inputMatches[i].x;
            pts1.at<float>(1, i) = inputMatches[i].y;
            pts2.at<float>(0, i) = targetMatches[i].x;
            pts2.at<float>(1, i) = targetMatches[i].y;
        }

        // Triangulate points
        cv::Mat hom_Q1;
        cv::triangulatePoints(P0, P1, pts1, pts2, hom_Q1);
        // Transform into cam2
        cv::Mat hom_Q2 = T * hom_Q1;

        // Extract 3xN points
        cv::Mat Q1 = hom_Q1.rowRange(0, 3).clone();
        cv::Mat Q2 = hom_Q2.rowRange(0, 3).clone();

        // Divide each column by its w (row 3 of hom_Q1)
        cv::Mat w1 = hom_Q1.row(3);  // 1 x N
        cv::Mat w2 = hom_Q2.row(3);  // 1 x N

        cv::Mat w1_rep, w2_rep;
        cv::repeat(w1, 3, 1, w1_rep);  // now 3 x N
        cv::repeat(w2, 3, 1, w2_rep);

        Q1 = Q1 / w1_rep;  // element-wise division
        Q2 = Q2 / w2_rep;

        // Count how many points have positive Z in both views
        int positiveZ = 0;
        for (int i = 0; i < Q1.cols; i++) {
            if (Q1.at<double>(2, i) > 0 && Q2.at<double>(2, i) > 0)
                positiveZ++;
        }

        return positiveZ;
    }



// SIFT-based alignment using a robust RANSAC affine fit instead of the essential
// matrix (not called by the live pipeline — see the KNOWN LIMITATION note on the
// constructor above). Fits a partial affine (rotation + uniform scale + translation)
// between matched SIFT points, then decomposes it into a 2D center-shift (x, y) and
// a log-scale "zoom" proxy (z) — cheaper than a full 3D pose recovery, at the cost of
// only being valid for roughly planar/rotation-dominated scenes.
cv::Point3f ImageMatcher::getAlignmentDisplacementRansac(const cv::Mat& inputImage)
{
    cv::cvtColor(inputImage, inputImageGray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> inputKeypoints;
    cv::Mat inputDescriptors;
    cv::Mat inputDesc;
    detectAndCompute(inputImageGray, inputKeypoints, inputDesc);
    inputDesc.convertTo(inputDescriptors,   CV_32F);

    std::vector<cv::DMatch> goodMatches;
    auto matches = goodMatcher(inputDescriptors);
    goodMatches = gridFilterMatches(matches, inputKeypoints);
    if (goodMatches.empty()) return cv::Point3f(0,0,0);
    float zMotion = 0; // inward/outward
    cv::Point2f center(inputImageGray.cols/2.0f, inputImageGray.rows/2.0f);
    for (const auto& m : goodMatches) {
        const cv::KeyPoint& kpInput = inputKeypoints[m.queryIdx];
        const cv::KeyPoint& kpTarget = targetKeypoints[m.trainIdx];
        inputMatches.push_back(kpInput.pt);
        targetMatches.push_back(kpTarget.pt);

        // Inward/outward: dot product with vector from center
        cv::Point2f vecToCenter = kpInput.pt - center;
        cv::Point2f motionVec = kpTarget.pt - kpInput.pt;
        float dot = vecToCenter.dot(motionVec);

        // If dot > 0 → moving outward, dot < 0 → moving inward
        zMotion += (dot > 0) ? -1.0f : 1.0f;
    }

    // Setup termination criteria (Max 30 iterations or 0.01 epsilon)
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01);

    // Refine the points
    // winSize is the search window (5x5 or 11x11 is standard)
    cv::cornerSubPix(inputImageGray, inputMatches, cv::Size(5, 5), cv::Size(-1, -1), criteria);
    cv::cornerSubPix(targetImageGray, targetMatches, cv::Size(5, 5), cv::Size(-1, -1), criteria);

    zMotion /= goodMatches.size(); // average tendency
    if (std::abs(zMotion) < 0.2) zMotion = 0;

    // Robust affine (rotation + uniform scale + translation), rejects outliers
    cv::Mat inliers;
    cv::Mat A = cv::estimateAffinePartial2D(inputMatches, targetMatches, inliers,
                                            cv::RANSAC,
                                            2.0,     // reprojection threshold in pixels (tune: 2-5 px)
                                            2000,    // max iterations
                                            0.99,    // confidence
                                            10       // refine iterations
                                            );

    if (A.empty()) return cv::Point3f(0, 0, 0);

    // Count inliers (optional safety)
    int inlierCount = 0;
    for (int i = 0; i < inliers.rows; ++i) inlierCount += (inliers.at<uchar>(i) != 0);
    if (inlierCount < 8) return cv::Point3f(0, 0, 0);

    // A is 2x3:
    // [ a b tx ]
    // [ c d ty ]
    double a = A.at<double>(0,0);
    double b = A.at<double>(0,1);
    double c = A.at<double>(1,0);
    double d = A.at<double>(1,1);
    double tx = A.at<double>(0,2);
    double ty = A.at<double>(1,2);

    // For estimateAffinePartial2D, scale is (roughly) uniform:
    // scale = sqrt(a^2 + c^2)  (also ~= sqrt(b^2 + d^2))
    double s = std::sqrt(a*a + c*c);

    // "z error" as zoom proxy:
    // s > 1 means target looks bigger than input (zoom-in / closer)
    // Use log(s) so it's symmetric: log(1.1)=+0.095, log(0.9)=-0.105
    double z = std::log(std::max(s, 1e-6));

    // rotation (radians)
    double theta = std::atan2(c, a);

    // center displacement (pixels)
    double cx = center.x, cy = center.y;
    double cxp = a*cx + b*cy + tx;
    double cyp = c*cx + d*cy + ty;

    double dxc = cxp - cx;
    double dyc = cyp - cy;

    // Return center shift, and zoom proxy
    return cv::Point3f((float)dxc, (float)dyc, (float)zMotion);
}

// Decomposes homography H into up to 4 candidate (R, t, normal) solutions
// (cv::decomposeHomographyMat), discards the ones inconsistent with the visible
// inlier points, and among the survivors picks the rotation whose roll component
// (rvec.z) is closest to zero — a heuristic for picking the "upright" solution when
// the scene is treated as a pure rotation. Only reachable via the permanently
// disabled "&& 0" branches in getAlignmentDirection()/getAlignment(); not currently
// exercised by the live pipeline.
cv::Mat ImageMatcher::computeRotation(cv::Mat& H, int& hInliers, cv::Mat& inlierMask)
{
    // ── PASS 1: Pure rotation via Homography ──────────────────────────


    float inlierRatio = (float)hInliers / inputMatches.size();

    // Decompose homography
    std::vector<cv::Mat> Rs, ts, normals;
    cv::decomposeHomographyMat(H, cameraMatrix, Rs, ts, normals);

    // Filter valid solutions
    std::vector<cv::Point2f> inlierInput, inlierTarget;
    for (int i = 0; i < inlierMask.rows; i++) {
        if (inlierMask.at<uchar>(i)) {
            inlierInput.push_back(inputMatches[i]);
            inlierTarget.push_back(targetMatches[i]);
        }
    }

    std::vector<int> validSolutions;
    cv::filterHomographyDecompByVisibleRefpoints(Rs, normals, inlierInput, inlierTarget, validSolutions);

    {
        std::ostringstream ss;
        ss << "Valid homography solutions: " << validSolutions.size();
        logger.log("ImageMatcher", ss.str());
    }
    if (!validSolutions.empty()) {
        // Pick solution with roll closest to 0
        cv::Mat bestR;
        double minRoll = 1e9;
        for (int idx : validSolutions) {
            if (normals[idx].at<double>(2) <= 0) continue;
            if (ts[idx].at<double>(2) <= 0) continue;
            // cv::Point3f angles = rotmatToYPRDeg_XYZ(Rs[idx]);
            cv::Mat rvec;
            cv::Rodrigues(Rs[idx], rvec);
            double angle = cv::norm(rvec) * 180.0 / CV_PI;
            // if (angle > 45.0) continue;  

            if (std::abs(rvec.at<double>(2)) < minRoll) {
                minRoll = std::abs(rvec.at<double>(2));
                bestR = Rs[idx];
            }
        }
        {
            std::ostringstream ss;
            ss << "shape of bestR: " << bestR.size();
            logger.log("ImageMatcher", ss.str());
        }
        cv::Point3f delta_cv = rotmatToYPRDeg_XYZ(bestR);
        {
            std::ostringstream ss;
            ss << "[PASS 1 - pure rotation] inliers: " << hInliers
               << " ratio: " << inlierRatio
               << " roll: " << delta_cv.z
               << " yaw: " << delta_cv.x
               << " pitch: " << delta_cv.y;
            logger.log("ImageMatcher", ss.str());
        }

        return bestR;
    }

    // ── PASS 2: Affine fallback (rotation + translation) ─────────────
    cv::Mat inliers;
    cv::Mat A = cv::estimateAffinePartial2D(
        inputMatches, targetMatches, inliers,
        cv::RANSAC, 2.0, 2000, 0.99, 10
    );

    if (A.empty()) return cv::Mat::eye(3, 3, CV_32F);

    int aInliers = 0;
    for (int i = 0; i < inliers.rows; ++i) aInliers += (inliers.at<uchar>(i) != 0);
    if (aInliers < 8) return cv::Mat::eye(3, 3, CV_32F);

    double a  = A.at<double>(0, 0);
    double b  = A.at<double>(0, 1);
    double tx = A.at<double>(0, 2);
    double ty = A.at<double>(1, 2);

    // Extract rotation angle from affine (partial affine = rotation + scale + translation)
    double angleRad = std::atan2(b, a);
    double angleDeg = angleRad * 180.0 / CV_PI;

    {
        std::ostringstream ss;
        ss << "[PASS 2 - affine fallback] inliers: " << aInliers
           << " angle: " << angleDeg
           << " tx: " << tx
           << " ty: " << ty;
        logger.log("ImageMatcher", ss.str());
    }

    // tx/ty in pixels → convert to approximate degrees using focal length
    double fx = cameraMatrix.at<float>(0, 0);
    double fy = cameraMatrix.at<float>(1, 1);
    double yaw   = std::atan2(tx, fx);
    double pitch = std::atan2(ty, fy);

    cv::Mat R = getRotationMatrixXYZ(0.0, pitch, yaw); // roll=0, pitch, yaw
    return R;
}

// Builds R = Rz(yaw) * Ry(pitch) * Rx(roll) (all angles in radians) — the inverse
// operation of rotmatToYPRDeg_XYZ() in Utils.cpp.
cv::Mat ImageMatcher::getRotationMatrixXYZ(double roll, double pitch, double yaw) {
    // Rotation matrices around x, y, z axes
    cv::Mat Rx = (cv::Mat_<double>(3, 3) << 
        1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll));

    cv::Mat Ry = (cv::Mat_<double>(3, 3) << 
        cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch));

    cv::Mat Rz = (cv::Mat_<double>(3, 3) << 
        cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1);

    // Order: R = Rz * Ry * Rx
    cv::Mat R = Rz * Ry * Rx;
    return R;
}

// Estimates the focus-of-expansion (FOE): the point every flow vector (pts2 - pts1)
// would pass through if extended as a line, found via weighted least squares over
// all the per-point flow lines. A large residual (mean distance from the fitted
// point back to each flow line) means the flow vectors don't converge/diverge from a
// single point — i.e. the motion looks like a rotation rather than a translation
// toward/away from some point in the scene. Used by getAlignmentDirection() to flag
// rotation-only frames.
FOEResult ImageMatcher::computeFOE(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2)
{
    assert(pts1.size() == pts2.size() && pts1.size() >= 2);

    // Build least squares system A^T A f = A^T b
    double AtA00 = 0, AtA01 = 0, AtA11 = 0;
    double Atb0  = 0, Atb1  = 0;

    for (size_t i = 0; i < pts1.size(); ++i) {
        float u = pts2[i].x - pts1[i].x;  // flow x
        float v = pts2[i].y - pts1[i].y;  // flow y

        // Line: v*x - u*y = v*x1 - u*y1
        float a = v;
        float b = -u;
        float c = v * pts1[i].x - u * pts1[i].y;

        float w = std::sqrt(u*u + v*v);  // weight by flow magnitude
        if (w < 1e-6f) continue;

        AtA00 += w * a * a;
        AtA01 += w * a * b;
        AtA11 += w * b * b;
        Atb0  += w * a * c;
        Atb1  += w * b * c;
    }

    // Solve 2x2 system
    double det = AtA00 * AtA11 - AtA01 * AtA01;
    if (std::abs(det) < 1e-10)
        return {cv::Point2f(0, 0), std::numeric_limits<float>::max()};

    float fx = (float)((AtA11 * Atb0 - AtA01 * Atb1) / det);
    float fy = (float)((AtA00 * Atb1 - AtA01 * Atb0) / det);

    // Compute residual — mean distance from each line to FOE
    double residual = 0;
    int count = 0;
    for (size_t i = 0; i < pts1.size(); ++i) {
        float u = pts2[i].x - pts1[i].x;
        float v = pts2[i].y - pts1[i].y;
        float norm = std::sqrt(u*u + v*v);
        if (norm < 1e-6f) continue;

        // Distance from point (fx, fy) to line through pts1[i] with direction (u,v)
        float dx = fx - pts1[i].x;
        float dy = fy - pts1[i].y;
        float cross = std::abs(dx * v - dy * u) / norm;
        residual += cross;
        ++count;
    }

    return {cv::Point2f(fx, fy), count > 0 ? (float)(residual / count) : 0.f};
}

// Estimates rotation directly from a homography (H = K*R*K_inv => R = K_inv*H*K),
// re-orthogonalized via SVD since the raw decomposition is rarely a perfect rotation
// matrix. Simpler/cheaper alternative to computeRotation()'s full decomposeHomographyMat
// approach, but doesn't disambiguate between the homography's multiple valid rotation
// solutions. Currently unused.
cv::Mat ImageMatcher::solvePureRotation() {


    // 1. Find the Homography matrix
    cv::Mat H = cv::findHomography(inputMatches, targetMatches, cv::RANSAC);

    // 2. Decompose Homography to Rotation
    // Since H = K * R * K_inv, we solve for R:
    // R = K_inv * H * K
    cv::Mat K;
    cameraMatrix.convertTo(K, CV_64F); 
    cv::Mat K_inv = K.inv();
    // cv::Mat K_inv = cameraMatrix.inv();
    cv::Mat R = K_inv * H * K;

    // 3. Optional: Extract Euler Angles from R
    // Note: R might not be perfectly orthogonal due to noise
    // Using SVD to force it to be a valid rotation matrix:
    cv::SVD svd(R);
    R = svd.u * svd.vt;

    return R;
}


// Legacy variant of getAlignmentDirection() that takes point correspondences as
// arguments instead of recomputing SIFT matches internally — same homography-vs-
// essential-matrix logic (including the same permanently-disabled "&& 0" pure-
// rotation branch), just parameterized differently. Not called anywhere in the
// current codebase; kept for reference alongside getAlignmentDirection().
std::tuple<cv::Mat, cv::Point3f, cv::Point2f, float, bool> ImageMatcher::getAlignmentOld( const std::vector<cv::Point2f> newInputMatches, const std::vector<cv::Point2f> newTargetMatches, bool rotationOnly){

    cv::Mat rotationMatrix = cv::Mat::eye(3, 3, CV_32F);
    cv::Point3f world_direction = cv::Point3f(0,0,0);
    float meanError = std::numeric_limits<float>::infinity();
    cv::Point2f flow = cv::Point2f( std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
    // inputImageGray.copyTo(oldImageGray);

    if(newTargetMatches.size() < 50){
        logger.log("ImageMatcher", "Not enough Matches found (atleast 50)");
        return {rotationMatrix, world_direction, flow, meanError, false};
    }
    bool rotationOnlyFlag = rotationOnly;
    cv::Mat maskH;
    cv::Mat H = cv::findHomography(newInputMatches, newTargetMatches, cv::RANSAC, 5.0, maskH, 2000, 0.999);
    int hInliers = H.empty() ? 0 : cv::countNonZero(maskH);

    cv::Mat maskE;
    cv::Mat EssentialMat = cv::findEssentialMat(newInputMatches, newTargetMatches, cameraMatrix, cv::RANSAC, 0.999, 2.0, 2000, maskE);
    int eInliers = EssentialMat.empty() ? 0 : cv::countNonZero(maskE);

    float tau_H = hInliers / (hInliers + eInliers + 1e-6f);

    FOEResult foe = computeFOE(newInputMatches, newTargetMatches);
    // bool foeInFrame = (foe.point.x >= 0 && foe.point.x <= imgWidth && foe.point.y >= 0 && foe.point.y <= imgHeight);
    if (foe.residual > 10) rotationOnlyFlag = true;
    if (rotationOnly) rotationOnlyFlag = true;

    {
        std::ostringstream ss;
        ss << "FOE: (" << foe.point.x << ", " << foe.point.y << "), residual: " << foe.residual
           << ", rotation only: " << rotationOnlyFlag;
        logger.log("ImageMatcher", ss.str());
    }
    {
        std::ostringstream ss;
        ss << "Homography inliers: " << hInliers << ", Essential inliers: " << eInliers
           << ", tau_H: " << tau_H;
        logger.log("ImageMatcher", ss.str());
    }

    // H = [ s*cosθ  -s*sinθ  tx ]
    //     [ s*sinθ   s*cosθ  ty ]

    if ( (!H.empty() && ((tau_H > 0.7) || rotationOnlyFlag)) && 0) {
        cv::Mat bestR = computeRotation(H, hInliers, maskH);
        cv::transpose(bestR, rotationMatrix);
    }
    else{

        cv::Mat bestR, bestT, bestT64, inliersE;
        int inlierCount = cv::recoverPose(EssentialMat, newInputMatches, newTargetMatches, cameraMatrix, bestR, bestT64, inliersE);
        std::vector<cv::Point2f> inliers1, inliers2;
        for(int i = 0; i < inliersE.rows; i++) {
            if(inliersE.at<uchar>(i)) {
                inliers1.push_back(newInputMatches[i]);
                inliers2.push_back(newTargetMatches[i]);
            }
        }
        meanError = getReprojectionError(inliers1, inliers2, bestR, bestT64);
        flow = getOpticalFlow(inliers1, inliers2);
        float flow_norm = std::sqrt(flow.x*flow.x + flow.y*flow.y);
        rotationMatrix = bestR.clone();
        cv::transpose(bestR, rotationMatrix);
        // bestT64.convertTo(bestT, CV_32F);
        cv::Mat bestT_ = - bestR.t() * bestT64;
        bestT_.convertTo(bestT, CV_32F);
        world_direction = cv::Point3f(bestT.at<float>(0,0)*flow_norm, bestT.at<float>(1,0)*flow_norm, bestT.at<float>(2,0)*flow_norm);

        if ((std::abs(bestT.at<float>(2,0)) > std::abs(bestT.at<float>(1,0)) + std::abs(bestT.at<float>(0,0))) && flow_norm < 10) world_direction.z *= 10;
        meanError = flow_norm;
    }
    {
        std::ostringstream ss;
        ss << "Estimated translation magnitude (mean error): " << meanError;
        logger.log("ImageMatcher", ss.str());
    }
  
    return {rotationMatrix, world_direction, flow, meanError, true};
}

// THE LIVE ENTRY POINT: this is the only ImageMatcher method main.cpp actually calls,
// once per frame. High-level flow:
//   1. Get point correspondences (inputMatches = current frame, targetMatches =
//      target image) one of two ways:
//        a) matchedPoints.newMatches (the common case): use the fresh SuperPoint/
//           LightGlue matches read from shared memory this frame, converting each
//           match's reported 2D covariance (packed as (xx, xy, yy) in cov.{x,y,z})
//           into a proper 2x2 covariance matrix.
//        b) matchedPoints.newMatches == false but a frame is available: the external
//           Python matcher hasn't produced a new result yet, so fall back to Lucas-
//           Kanade optical-flow tracking of the *previous* frame's matched points
//           (targetMatches_) into the current frame (track_features()), and estimate
//           per-point covariance from local image gradients instead
//           (compute_point_covariances()). Either path bails out (returns
//           success=false) if fewer than 9 usable correspondences survive.
//   2. Convert every 2D match + its pixel covariance into a MatchData: unproject to
//      a normalized 3D bearing vector via K_inv, and propagate the 2D covariance to
//      3D via the unscented transform (see UnscentedTransform::propagate2DTo3D).
//   3. Hand the batch of MatchData to the PNEC solver (RelativePoseEstimatorOld —
//      see pnecOptimizer.hpp) to get the rotation R and translation direction t_dir.
//      R and t_init/t_dir are member variables, so each call seeds the solver with
//      the previous frame's result for faster convergence.
//   4. Score the result with the Sampson pixel error (getSampsonPixelError) as a
//      confidence/quality metric returned to main.cpp; errors under 1px are inflated
//      x100 so downstream thresholds (main.cpp's transError-based logic) treat a
//      near-zero fit as "very good" rather than being swamped by numerical noise.
std::tuple<cv::Mat, cv::Point3f, float, bool> ImageMatcher::getAlignment( const Matches& matchedPoints, const cv::Mat& frame){

    pnec::RelativePoseEstimatorOld estimator(logger, K);

    if(!matchedPoints.newMatches && !frame.empty()){
        TrackingResult trackingResult = track_features(frame, targetImageGray, targetMatches_);
        int valid_count = std::count(trackingResult.status.begin(), trackingResult.status.end(), 1);
        if(valid_count > 8){
            inputMatches.clear();
            targetMatches.clear();
            for (int i = 0; i < targetMatches_.size(); ++i){
                inputMatches.push_back(trackingResult.tracked_pts[i]);
                targetMatches.push_back(targetMatches_[i]);
            }
        }
        else return {cv::Mat::eye(3, 3, CV_64F), cv::Point3f(0, 0, 0), std::numeric_limits<float>::max(), false};
        covariances = compute_point_covariances(frame, inputMatches, 5);
    }
    else{
        inputMatches = matchedPoints.kpts0;
        targetMatches = matchedPoints.kpts1;
        targetMatches_ = matchedPoints.kpts1;
        if(matchedPoints.n > 8){
            for(auto cov : matchedPoints.covariances){
                cv::Matx22f H(cov.x, cov.y, cov.y, cov.z);

                // Inverse Hessian = 2D Covariance Matrix Sigma_2D
                covariances.push_back(H.inv(cv::DECOMP_SVD));
            }
        }
        else return {cv::Mat::eye(3, 3, CV_64F), cv::Point3f(0, 0, 0), std::numeric_limits<float>::max(), false};
    }
    std::vector<pnec::MatchData> matches;
    Eigen::Matrix3d K_inv = K.inverse();
    pnec::UnscentedTransform ut(logger);

    for (size_t i = 0; i < inputMatches.size(); i++) {
        pnec::MatchData m;
        Eigen::Vector3d h1(inputMatches[i].x, inputMatches[i].y, 1.0);
        Eigen::Vector3d h2(targetMatches[i].x, targetMatches[i].y, 1.0);
        m.bearing1 = (K_inv * h1).normalized();
        m.bearing2 = (K_inv * h2).normalized();
        Eigen::Matrix2d cov2d;
        cov2d << covariances[i](0,0), covariances[i](0,1),
                covariances[i](1,0), covariances[i](1,1);
        m.cov3d = ut.propagate2DTo3D(cov2d, K);
        matches.push_back(m);
    }

    double totalError_ = 0.0d;

    estimator.estimate(matches, R, t_dir, t_init, totalError_);
    t_init = t_dir;
    cv::Mat R_cv;
    cv::Point3f t_cv;
    R_cv = matrix3dToMat(R);
    t_cv = cv::Point3f(t_dir.x(), t_dir.y(), t_dir.z());

    cv::Mat t_mat = cv::Mat(t_cv);
    float totalError;
    totalError = ImageMatcher::getSampsonPixelError(inputMatches,targetMatches, R_cv, t_mat);
    if (totalError < 1) totalError *= 100;

    return {R_cv, t_cv, totalError, true};
}

// Lucas-Kanade optical-flow fallback used when the external SuperPoint/LightGlue
// matcher hasn't produced a new match set for this frame (see getAlignment(), which
// calls this as track_features(currentFrame, targetImageGray, targetMatches_)).
// NOTE the (img_prev, img_next) parameter names are misleading for that call site:
// pts_next are the previously-matched points in the *target* image (img_next), and
// this tracks them into img_prev (the *current* frame) — result.tracked_pts ends up
// holding their estimated positions in img_prev/the current frame, which is what
// getAlignment() uses as the new inputMatches. It then tracks those estimated points
// back into img_next and drops any point whose round-trip distance from its original
// pts_next position exceeds max_bidirectional_error, to reject points that drifted
// onto the wrong feature.
TrackingResult ImageMatcher::track_features(const cv::Mat& img_prev, const cv::Mat& img_next,
                                           const std::vector<cv::Point2f>& pts_next,
                                           float max_bidirectional_error)
{
    TrackingResult result;

    if (pts_next.empty()) {
        logger.log("ImageMatcher", "Warning: pts_next is empty!");
        return result;
    }

    // Convert both to grayscale
    cv::Mat gray_prev = img_prev, gray_next = img_next;
    if (gray_prev.channels() > 1) cv::cvtColor(gray_prev, gray_prev, cv::COLOR_BGR2GRAY);
    if (gray_next.channels() > 1) cv::cvtColor(gray_next, gray_next, cv::COLOR_BGR2GRAY);

    // LK Optical Flow parameters
    cv::Size win_size(31, 31); // Slightly larger window for higher motion tolerance
    int max_level = 3;
    cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    // 1. Backward Tracking (img_next -> img_prev)
    std::vector<cv::Point2f> pts_prev_estimated;
    std::vector<uchar> status_bwd;
    std::vector<float> err_bwd;

    cv::calcOpticalFlowPyrLK(
        gray_next, gray_prev,
        pts_next, pts_prev_estimated,
        status_bwd, err_bwd,
        win_size, max_level, criteria
    );

    // 2. Forward Tracking (img_prev -> img_next) for validation
    std::vector<cv::Point2f> pts_next_back;
    std::vector<uchar> status_fwd;
    std::vector<float> err_fwd;

    cv::calcOpticalFlowPyrLK(
        gray_prev, gray_next,
        pts_prev_estimated, pts_next_back,
        status_fwd, err_fwd,
        win_size, max_level, criteria
    );

    // Pre-allocate tracking result
    size_t num_pts = pts_next.size();
    result.tracked_pts.resize(num_pts);
    result.status.assign(num_pts, 0);

    int valid_count = 0;

    for (size_t i = 0; i < num_pts; ++i) {
        // Save matched point in img_prev
        result.tracked_pts[i] = pts_prev_estimated[i];

        // if (!status_bwd[i] || !status_fwd[i]) continue;

        // // Round-trip distance error check (distance between original pt and round-trip pt in img_next)
        float fb_dist = cv::norm(pts_next[i] - pts_next_back[i]);

        if (fb_dist <= max_bidirectional_error) {
            result.status[i] = 1;
            valid_count++;
        }
    }

    {
        std::ostringstream ss;
        ss << "Input points: " << num_pts << " | Valid tracked points: " << valid_count;
        logger.log("ImageMatcher", ss.str());
    }
    return result;
}

/**
 * @brief Computes the 2x2 inverse Hessian (covariance matrix) for each tracked keypoint,
 * from the local structure tensor of image gradients in a window_size x window_size
 * patch around it (a flat/low-texture patch -> a near-singular Hessian -> a large,
 * unreliable covariance). Used by getAlignment() in the optical-flow fallback path,
 * as a substitute for the per-match covariances SuperPoint/LightGlue would otherwise
 * report.
 */
std::vector<cv::Matx22f> ImageMatcher::compute_point_covariances(const cv::Mat& img_gray,
                                const std::vector<cv::Point2f>& points, int window_size) 
{
    std::vector<cv::Matx22f> covariances(points.size());

    // Compute image gradients
    cv::Mat gx, gy;
    cv::Sobel(img_gray, gx, CV_32F, 1, 0, 3);
    cv::Sobel(img_gray, gy, CV_32F, 0, 1, 3);

    int half_win = window_size / 2;

    for (size_t k = 0; k < points.size(); ++k) {
        int cx = cvRound(points[k].x);
        int cy = cvRound(points[k].y);

        float Ixx = 0.0f, Iyy = 0.0f, Ixy = 0.0f;

        for (int y = cy - half_win; y <= cy + half_win; ++y) {
            for (int x = cx - half_win; x <= cx + half_win; ++x) {
                if (x >= 0 && x < img_gray.cols && y >= 0 && y < img_gray.rows) {
                    float ix = gx.at<float>(y, x);
                    float iy = gy.at<float>(y, x);
                    Ixx += ix * ix;
                    Iyy += iy * iy;
                    Ixy += ix * iy;
                }
            }
        }

        // Structure Tensor H
        cv::Matx22f H(Ixx, Ixy, Ixy, Iyy);
        
        // Inverse Hessian = 2D Covariance Matrix Sigma_2D
        covariances[k] = H.inv(cv::DECOMP_SVD);
    }

    return covariances;
}


// Mean Sampson distance (pixels) between pts1/pts2 under the fundamental matrix
// F = K^-T * [t]_x * R * K^-1. This is what getAlignment() uses as its returned
// confidence/quality metric (see the totalError output).
float ImageMatcher::getSampsonPixelError(const std::vector<cv::Point2f>& pts1,
                                         const std::vector<cv::Point2f>& pts2,
                                         const cv::Mat& R_in, const cv::Mat& t_in)
{
    if (pts1.empty() || pts1.size() != pts2.size()) return 0.0f;

    // Convert inputs to double precision
    cv::Mat R, t, K;
    R_in.convertTo(R, CV_64F);
    t_in.convertTo(t, CV_64F);
    cameraMatrix.convertTo(K, CV_64F);

    // Normalize translation vector
    cv::Mat t_norm = t / cv::norm(t);

    // 1. Compute Skew-Symmetric Matrix [t]_x
    cv::Mat t_skew = (cv::Mat_<double>(3, 3) <<
            0.0, -t_norm.at<double>(2),  t_norm.at<double>(1),
         t_norm.at<double>(2),     0.0, -t_norm.at<double>(0),
        -t_norm.at<double>(1),  t_norm.at<double>(0),     0.0);

    // 2. Essential Matrix E = [t]_x * R
    cv::Mat E = t_skew * R;

    // 3. Fundamental Matrix F = K^(-T) * E * K^(-1)
    cv::Mat K_inv = K.inv();
    cv::Mat F = K_inv.t() * E * K_inv;

    double totalError = 0.0;
    int validCount = 0;

    for (size_t i = 0; i < pts1.size(); i++) {
        cv::Mat x1 = (cv::Mat_<double>(3, 1) << pts1[i].x, pts1[i].y, 1.0);
        cv::Mat x2 = (cv::Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, 1.0);

        // Compute Epipolar Lines: l2 = F * x1, l1 = F^T * x2
        cv::Mat F_x2  = F * x2;
        cv::Mat Ft_x1 = F.t() * x1;

        // Algebraic error: x1^T * F * x2
        double num = x1.dot(F_x2);
        double num_sq = num * num;

        // Geometric weighting denominator (sum of squared components of epipolar lines)
        double den = F_x2.at<double>(0) * F_x2.at<double>(0) +
                     F_x2.at<double>(1) * F_x2.at<double>(1) +
                     Ft_x1.at<double>(0) * Ft_x1.at<double>(0) +
                     Ft_x1.at<double>(1) * Ft_x1.at<double>(1);

        if (den < 1e-8) continue; // Prevent division by zero

        // Squared Sampson distance in pixels^2
        double sampson_sq = num_sq / den;

        // Take square root to get mean pixel distance from epipolar line
        totalError += std::sqrt(sampson_sq);
        validCount++;
    }

    if (validCount == 0) return 1e6f;

    // Returns average pixel distance to epipolar line (e.g. 0.85 px)
    return static_cast<float>(totalError / validCount); 
}

// Alternative to getSampsonPixelError(): mean symmetric epipolar distance (sum of the
// point-to-line distance in each image, rather than the first-order Sampson
// approximation) under the fundamental matrix built from (Rf, tf). Not currently
// called by the live pipeline.
float ImageMatcher::getSymmetricEpipolarDistance(const std::vector<cv::Point2f>& pts1,
                                                 const std::vector<cv::Point2f>& pts2,
                                                 const cv::Mat& Rf, const cv::Mat& tf)
{
    if (pts1.empty() || pts1.size() != pts2.size()) return 0.0f;

    // 1. Convert to double precision and normalize translation
    cv::Mat R, t, K;
    Rf.convertTo(R, CV_64F);
    tf.convertTo(t, CV_64F);
    cameraMatrix.convertTo(K, CV_64F);

    double t_norm = cv::norm(t);
    if (t_norm < 1e-8) return 1e6f; // Pure rotation / degenerate translation
    t = t / t_norm;

    // 2. Skew-Symmetric matrix [t]_x
    cv::Mat t_skew = (cv::Mat_<double>(3, 3) <<
            0.0, -t.at<double>(2),  t.at<double>(1),
         t.at<double>(2),     0.0, -t.at<double>(0),
        -t.at<double>(1),  t.at<double>(0),     0.0);

    // 3. Essential Matrix E = [t]_x * R
    cv::Mat E = t_skew * R;

    // 4. Fundamental Matrix F = K^(-T) * E * K^(-1)
    cv::Mat K_inv = K.inv();
    cv::Mat F = K_inv.t() * E * K_inv;
    cv::Mat Ft = F.t();

    double total_distance = 0.0;
    int valid_count = 0;

    for (size_t i = 0; i < pts1.size(); i++) {
        // Homogeneous pixel coordinates
        cv::Mat x1 = (cv::Mat_<double>(3, 1) << pts1[i].x, pts1[i].y, 1.0);
        cv::Mat x2 = (cv::Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, 1.0);

        // Epipolar line in Image 2: l2 = F * x1 = [a2, b2, c2]^T
        cv::Mat l2 = F * x1;
        double a2 = l2.at<double>(0);
        double b2 = l2.at<double>(1);
        double c2 = l2.at<double>(2);

        // Epipolar line in Image 1: l1 = F^T * x2 = [a1, b1, c1]^T
        cv::Mat l1 = Ft * x2;
        double a1 = l1.at<double>(0);
        double b1 = l1.at<double>(1);
        double c1 = l1.at<double>(2);

        // Perpendicular distance denominators
        double den2 = std::sqrt(a2 * a2 + b2 * b2);
        double den1 = std::sqrt(a1 * a1 + b1 * b1);

        if (den1 < 1e-8 || den2 < 1e-8) continue; // Guard against numerical instability

        // Distance from point 2 to epipolar line 2: |a2*u2 + b2*v2 + c2| / sqrt(a2^2 + b2^2)
        double d2 = std::abs(a2 * pts2[i].x + b2 * pts2[i].y + c2) / den2;

        // Distance from point 1 to epipolar line 1: |a1*u1 + b1*v1 + c1| / sqrt(a1^2 + b1^1)
        double d1 = std::abs(a1 * pts1[i].x + b1 * pts1[i].y + c1) / den1;

        // Symmetric distance (average of d1 and d2)
        total_distance += 0.5 * (d1 + d2);
        valid_count++;
    }

    if (valid_count == 0) return 1e6f;

    // Returns average distance in pixels (e.g. 0.75 px)
    return static_cast<float>(total_distance / valid_count);
}
