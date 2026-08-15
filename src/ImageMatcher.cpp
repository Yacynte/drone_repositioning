#include "ImageMatcher.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>



// pts1, pts2 are matched pixel points (same length), already filtered by inliers if possible
// K is 3x3 double, R is 3x3 double from recoverPose (rotation from cam1 -> cam2)

static inline cv::Point2f projectPoint(const cv::Mat& K, const cv::Vec3d& x)
{
    double X = x[0], Y = x[1], Z = x[2];
    double u = (K.at<double>(0,0) * (X/Z)) + K.at<double>(0,2);
    double v = (K.at<double>(1,1) * (Y/Z)) + K.at<double>(1,2);
    return cv::Point2f((float)u, (float)v);
}

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


// ImageMatcher::ImageMatcher1(const std::string& targetImagePath) {
//     // Load target image
//     cv::Mat target = cv::imread(targetImagePath, cv::IMREAD_COLOR);
//     if (target.empty()) {
//         throw std::runtime_error("Could not load target image");
//     }
//     cv::cvtColor(target, targetImageGray, cv::COLOR_BGR2GRAY);

//     // Initialize SIFT detector and BFMatcher
//     // sift = cv::SIFT::create();
//     sift = cv::SIFT::create(5000,    // nfeatures    — default 0 (unlimited, but keeps best), set explicitly
//                             3,       // nOctaveLayers — default 3, increase for more features
//                             0.05,    // contrastThreshold — default 0.04, LOWER = more features
//                             10,      // edgeThreshold — default 10, HIGHER = more features  
//                             1.6      // sigma — default 1.6, leave this
//                             );
//     // matcher = cv::BFMatcher::create(cv::NORM_L2);

//     matcherFlann = cv::FlannBasedMatcher(cv::makePtr<cv::flann::KDTreeIndexParams>(5),
//                                         cv::makePtr<cv::flann::SearchParams>(75));

//     height = target.rows;
//     width = target.cols;
//     cameraMatrix = (cv::Mat_<float>(3,3) << 
//                     width / 2.0f, 0,            width / 2.0f,
//                     0,            width / 2.0f, height / 2.0f,
//                     0,            0,            1.0f);
//     // Detect and compute features for target
//     cv::Mat targetDesc;
//     detectAndComputegrid(targetImageGray, targetKeypoints, targetDesc);
//     // detectAndCompute(targetImageGray, targetKeypoints, targetDesc);
//     targetDesc.convertTo(targetDescriptors, CV_32F);
    
//     std::cout << "Target keypoints size: " << targetKeypoints.size() << std::endl;
// }

ImageMatcher::ImageMatcher(const std::string& targetImagePath, const cv::Mat& K_cv) {
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
    std::cout << "Target image size: " << targetImageGray.cols << "x" << targetImageGray.rows << std::endl;

}

void ImageMatcher::detectAndCompute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
    sift->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
}
void ImageMatcher::detectAndComputeLKFlow(){

}

void ImageMatcher::detectAndComputegrid(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                                    cv::Mat& descriptors, int gridX, int gridY, int maxPerCell)
{
    // 1. Detect all keypoints
    std::vector<cv::KeyPoint> allKeypoints;
    sift->detect(image, allKeypoints);

    // 2. Divide image into grid
    int cellW = image.cols / gridX;
    int cellH = image.rows / gridY;
    // std::cout << "All Keypoints detected: " << allKeypoints.size() << std::endl;
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
    // cv::Mat ImageDesc;
    sift->compute(image, keypoints, descriptors);
    // ImageDesc.convertTo(descriptors, CV_32F);
    std::cout << "All Keypoints detected: " << keypoints.size() << std::endl;
}


cv::Point3f ImageMatcher::getAlignmentDisplacement(const cv::Mat& inputImage) {
    cv::Mat inputGray;
    cv::cvtColor(inputImage, inputGray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> inputKeypoints;
    cv::Mat inputDescriptors;
    detectAndComputegrid(inputGray, inputKeypoints, inputDescriptors);

    // std::vector<cv::DMatch> matches;
    // matcher->match(inputDescriptors, targetDescriptors, matches);

    std::vector<cv::DMatch> goodMatches;
    std::cout << "Target image keypoint size: " << targetKeypoints.size() << std::endl;
    std::cout << "Input image keypoint size: " << inputKeypoints.size() << std::endl;
    goodMatches = goodMatcher(inputDescriptors);
    // goodMatches = gridFilterMatches(matches, inputKeypoints);
    std::cout << "Found good matches of size: " << goodMatches.size() << std::endl;
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
    // double d_pixels = std::sqrt(direction2D.x*direction2D.x + direction2D.y*direction2D.y);
    // float dist_z = cv::norm(direction2D);
    // zMotion *= dist_z; // scale by overall motion magnitude
    return cv::Point3f(direction2D.x, direction2D.y, zMotion);
    // return zMotion;
}

std::vector<cv::DMatch> ImageMatcher::goodMatcher(const cv::Mat& inputDescriptors) {
    // KNN match to find the two best matches for each descriptor
    std::vector<std::vector<cv::DMatch>> matchesAB;
    // matcher->knnMatch(inputDescriptors, targetDescriptors, matchesAB, 2);
    matcherFlann.knnMatch(inputDescriptors, targetDescriptors, matchesAB, 2);
    // std::vector<std::vector<cv::DMatch>> matchesBA;
    // matcherFlann.knnMatch(targetDescriptors, inputDescriptors, matchesBA, 2);
    // matcher->knnMatch(targetDescriptors, inputDescriptors, matchesBA, 2);
    std::cout << "Total matches found: " << matchesAB.size() << std::endl;
    // std::cout << "Target descriptors size: " << targetDescriptors.rows << std::endl;
    // std::cout << "Input descriptors size: " << inputDescriptors.rows << std::endl;  
    // Apply Lowe's ratio test and cross-check
    const float ratio = 0.8f;

    std::vector<cv::DMatch> goodAB, goodBA;

    for (const auto& m : matchesAB){
        if (m.size() == 2 && m[0].distance < ratio * m[1].distance)
            goodAB.push_back(m[0]);
    }
    std::cout << "Good matches after ratio test: " << goodAB.size() << std::endl;
    return goodAB;
    // for (const auto& m : matchesBA){
    //     if (m.size() == 2 && m[0].distance < ratio * m[1].distance)
    //         goodBA.push_back(m[0]);
    // }
    // // Cross-check: keep only matches that are mutual best matches
    // std::vector<cv::DMatch> crossCheckedMatches;
    // for (const auto& mAB : goodAB)
    // {
    //     for (const auto& mBA : goodBA)
    //     {
    //         if (mAB.queryIdx == mBA.trainIdx &&
    //             mAB.trainIdx == mBA.queryIdx)
    //         {
    //             crossCheckedMatches.push_back(mAB);
    //             break;
    //         }
    //     }
    // }
    // return crossCheckedMatches;
}


std::vector<cv::DMatch> ImageMatcher::gridFilterMatches(const std::vector<cv::DMatch>& matches, 
                                                        const std::vector<cv::KeyPoint>& queryKps, 
                                                        int gridCols, int gridRows, int maxPerCell)
{
    float cellW = width  / gridCols;
    float cellH = height / gridRows;

    // std::cout<< "width: "<< width << " height: " << height <<std::endl;
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
    // int inlierCount = cv::recoverPose(EssentialMat, inputMatches, targetMatches, cameraMatrix, bestR, bestT, inliersE);
    // Now compute translation-like residual in pixels:
    cv::Point2f dxy = rotationCompensatedResidual(inputMatches, targetMatches, cameraMatrix, bestR, inliersE);
    float dz = rotationCompensatedZoom(inputMatches, targetMatches, cameraMatrix, bestR, inliersE);
    dxyz.x = dxy.x;
    dxyz.y = dxy.y;
    dxyz.z = dz;
}


std::tuple<cv::Mat, cv::Point3f, cv::Point2f, float, bool> ImageMatcher::getAlignmentDirection( const cv::Mat& inputImage, bool rotationOnly){
    
    // cv::imwrite("inputImageGray.png", inputImage);
    if(inputImage.empty() && inputMatches.size() == 0) return {cv::Mat::eye(3, 3, CV_32F), cv::Point3f(0,0,0), cv::Point2f(0,0), std::numeric_limits<float>::infinity(), false};

    if (!inputImage.empty()){

        cv::Mat inputGray;
        cv::cvtColor(inputImage, inputImageGray, cv::COLOR_BGR2GRAY);
        // std::cout << "Converted to Gray" << std::endl;
        std::vector<cv::KeyPoint> inputKeypoints;
        cv::Mat inputDescriptors;

        std::vector<cv::Point2f>().swap(inputMatches);
        std::vector<cv::Point2f>().swap(targetMatches);
        cv::Mat inputDesc;
        // detectAndCompute(inputImageGray, inputKeypoints, inputDesc);
        detectAndComputegrid(inputImageGray, inputKeypoints, inputDesc);
        inputDesc.convertTo(inputDescriptors, CV_32F);
        inputDesc.release();
        std::vector<cv::DMatch> goodMatches;
        // goodMatches = goodMatcher(inputDescriptors);
        auto matches = goodMatcher(inputDescriptors);
        goodMatches = gridFilterMatches(matches, inputKeypoints);

        // std::cout << "Matched Descriptors" << std::endl;
        if (goodMatches.empty()) return {cv::Mat::eye(3, 3, CV_32F), cv::Point3f(0,0,0), cv::Point2f(0,0), std::numeric_limits<float>::infinity(), false};
        for (const auto& m : goodMatches) {
            const cv::KeyPoint& kpInput = inputKeypoints[m.queryIdx];
            const cv::KeyPoint& kpTarget = targetKeypoints[m.trainIdx];

            inputMatches.push_back(kpInput.pt);
            targetMatches.push_back(kpTarget.pt);
            // oldMatches.target_indices.push_back(m.trainIdx);
        }
        // oldMatches.prev_pts = inputMatches;
        // matches_length = targetMatches.size();
        // std::cout << " Sift features and Flann matcher " << std::endl;
        

        // Setup termination criteria (Max 30 iterations or 0.01 epsilon)
        cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01);

        // Refine the points
        // winSize is the search window (5x5 or 11x11 is standard)
        cv::cornerSubPix(inputImageGray, inputMatches, cv::Size(5, 5), cv::Size(-1, -1), criteria);
        cv::cornerSubPix(targetImageGray, targetMatches, cv::Size(5, 5), cv::Size(-1, -1), criteria);
        // std::cout << "Prepared Matches" << std::endl;
        
    }

    // std::cout <<"Matches, target: " << targetMatches.size() << ", input: " << inputMatches.size() << std::endl;
    cv::Mat rotationMatrix = cv::Mat::eye(3, 3, CV_32F);
    cv::Point3f world_direction = cv::Point3f(0,0,0);
    float meanError = std::numeric_limits<float>::infinity();
    cv::Point2f flow = cv::Point2f( std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
    // inputImageGray.copyTo(oldImageGray);

    if(inputMatches.size() < 50){
        std::cout << " Not enough Matches found (atleast 50)" << std::endl;
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

    std::cout << "FOE: (" << foe.point.x << ", " << foe.point.y << "), residual: " << foe.residual << ", rotation only: " << rotationOnlyFlag << std::endl;

    std::cout << "Homography inliers: " << hInliers << ", Essential inliers: " << eInliers << ", tau_H: " << tau_H << std::endl;

    // H = [ s*cosθ  -s*sinθ  tx ]
    //     [ s*sinθ   s*cosθ  ty ]

    double tx = H.at<double>(0,2);
    double ty = H.at<double>(1,2);
    // meanError = std::sqrt(tx*tx + ty*ty);

    // std::cout << "Estimated translation magnitude (mean error): " << meanError << std::endl;
    
    if ( (!H.empty() && ((tau_H > 0.7) || rotationOnlyFlag)) && 0) {
        // rotationMatrix = solvePureRotation();
        cv::Mat bestR = computeRotation(H, hInliers, maskH);
        cv::transpose(bestR, rotationMatrix);
    }
    else{

        cv::Mat bestR, bestT, bestT64, inliersE;
        int inlierCount = cv::recoverPose(EssentialMat, inputMatches, targetMatches, cameraMatrix, bestR, bestT64, inliersE);
        // std::cout << " Recoverpose inliers " << inlierCount << std::endl;
        std::vector<cv::Point2f> inliers1, inliers2;
        for(int i = 0; i < inliersE.rows; i++) {
            if(inliersE.at<uchar>(i)) {
                inliers1.push_back(inputMatches[i]);
                inliers2.push_back(targetMatches[i]);
            }
        }
        // std::cout << " Before ReprojectionError " << std::endl;
        meanError = getReprojectionError(inliers1, inliers2, bestR, bestT64);
        // std::cout << " After ReprojectionError " << std::endl;
        flow = getOpticalFlow(inliers1, inliers2);
        float flow_norm = std::sqrt(flow.x*flow.x + flow.y*flow.y);
        // error3d.x = flow.x;
        // error3d.y = flow.y;
        // error3d.z = 
        rotationMatrix = bestR.clone();
        cv::transpose(bestR, rotationMatrix);
        // bestT64.convertTo(bestT, CV_32F);
        cv::Mat bestT_ = - bestR.t() * bestT64;
        bestT_.convertTo(bestT, CV_32F);
        // cv::Mat t_inv, t_inv64;
        // t_inv = bestT * meanError;
        // t_inv64 = -bestT64 * bestR.t() * meanError;
        // t_inv64.convertTo(t_inv, CV_32F);
        // world_direction = cv::Point3f(t_inv.at<float>(0,0), t_inv.at<float>(1,0), t_inv.at<float>(2,0));
        world_direction = cv::Point3f(bestT.at<float>(0,0)*flow_norm, bestT.at<float>(1,0)*flow_norm, bestT.at<float>(2,0)*flow_norm);

        if ((std::abs(bestT.at<float>(2,0)) > std::abs(bestT.at<float>(1,0)) + std::abs(bestT.at<float>(0,0))) && flow_norm < 10) world_direction.z *= 10;
        meanError = flow_norm;
        // std::cout << "Estimated flow direction: " << flow << std::endl;
        // std::cout << "Estimated translation matrix: " << std::endl << bestT << std::endl;
        // std::cout << "world direction: " << world_direction << std::endl;
    }
    std::cout << "Estimated translation magnitude (mean error): " << meanError << std::endl;
  
    return {rotationMatrix, world_direction, flow, meanError, true};
}

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

cv::Mat ImageMatcher::formTransf(const cv::Mat& R, const cv::Mat& t) {
    cv::Mat T = cv::Mat::eye(4, 4, CV_32F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    t.copyTo(T(cv::Rect(3, 0, 1, 3)));
    return T;
}

int ImageMatcher::sumZCalRelativeScale(const cv::Mat& Rotation, const cv::Mat& translation) {
        // Form transformation matrix
        cv::Mat T = ImageMatcher::formTransf(Rotation, translation);
        // std::cout << "In relative Scale" << std::endl;
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
        // std::cout << "Triangulation successful" << std::endl;

        // Transform into cam2
        cv::Mat hom_Q2 = T * hom_Q1;

        // Un-homogenize
        // cv::Mat Q1 = hom_Q1.rowRange(0, 3).clone();
        // Q1 = Q1.mul(1.0 / hom_Q1.row(3).clone().t());
        // cv::Mat Q2 = hom_Q2.rowRange(0, 3).clone();
        // Q2 = Q2.mul(1.0 / hom_Q2.row(3).clone().t());

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



cv::Point3f ImageMatcher::getAlignmentDisplacementRansac(const cv::Mat& inputImage)
{
    // std::cout << "target gray size: " << targetImageGray.size() << std::endl;
    // std::cout << "input image size: " << inputImage.size() << std::endl;
    
    cv::cvtColor(inputImage, inputImageGray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> inputKeypoints;
    cv::Mat inputDescriptors;
    // detectAndComputegrid(inputImageGray, inputKeypoints, inputDescriptors);
    cv::Mat inputDesc;
    detectAndCompute(inputImageGray, inputKeypoints, inputDesc);
    inputDesc.convertTo(inputDescriptors,   CV_32F);
    // std::vector<cv::DMatch> matches;
    // matcher->match(inputDescriptors, targetDescriptors, matches);

    std::vector<cv::DMatch> goodMatches;
    // std::cout << "Target image keypoint size: " << targetKeypoints.size() << std::endl;
    // std::cout << "Input image keypoint size: " << inputKeypoints.size() << std::endl;
    auto matches = goodMatcher(inputDescriptors);
    // std::cout << "Found matches of size: " << matches.size() << std::endl;
    goodMatches = gridFilterMatches(matches, inputKeypoints);
    // std::cout << "Found good matches of size: " << goodMatches.size() << std::endl;
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
    // std::cout << "Prepared Matches" << std::endl;

    zMotion /= goodMatches.size(); // average tendency
    if (std::abs(zMotion) < 0.2) zMotion = 0;
    // if(std::abs(zMotion) < 0.35 * goodMatches.size()) zMotion = 0; 

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

    // Return (x,y) in pixels and z as dimensionless zoom error
    // return cv::Point3f((float)tx, (float)ty, (float)zMotion);
    // rotation (radians)
    double theta = std::atan2(c, a);

    // center displacement (pixels)
    // cv::Point2f center(inputGray.cols * 0.5f, inputGray.rows * 0.5f);
    double cx = center.x, cy = center.y;
    double cxp = a*cx + b*cy + tx;
    double cyp = c*cx + d*cy + ty;

    double dxc = cxp - cx;
    double dyc = cyp - cy;

    // Return center shift, and zoom proxy
    return cv::Point3f((float)dxc, (float)dyc, (float)zMotion);
}

cv::Mat ImageMatcher::computeRotation(cv::Mat& H, int& hInliers, cv::Mat& inlierMask)
{
    // ── PASS 1: Pure rotation via Homography ──────────────────────────
    
    float inlierRatio = (float)hInliers / inputMatches.size();

    // std::cout << "[PASS 1 - pure rotation] inliers: " << hInliers
    //           << " ratio: " << inlierRatio
    //           << std::endl;

    // && inlierRatio > 0.2f
    // if (!H.empty() && hInliers >= 20) {
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

    std::cout << "Valid homography solutions: " << validSolutions.size() << std::endl;
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
        std::cout << "shape of bestR: " << bestR.size() << std::endl;
        cv::Point3f delta_cv = rotmatToYPRDeg_XYZ(bestR);
        std::cout << "[PASS 1 - pure rotation] inliers: " << hInliers
                    << " ratio: " << inlierRatio
                    << " roll: " << delta_cv.z
                    << " yaw: " << delta_cv.x
                    << " pitch: " << delta_cv.y << std::endl;

        // return cv::Point3f(-delta_cv.y, -delta_cv.x, 0.0f); // remap to Unreal, roll=0
        // return cv::Point3f(delta_cv.x, delta_cv.y, 0.0f);
        return bestR;
    }
    // }

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

    std::cout << "[PASS 2 - affine fallback] inliers: " << aInliers
              << " angle: " << angleDeg
              << " tx: "    << tx
              << " ty: "    << ty << std::endl;

    // tx/ty in pixels → convert to approximate degrees using focal length
    double fx = cameraMatrix.at<float>(0, 0);
    double fy = cameraMatrix.at<float>(1, 1);
    double yaw   = std::atan2(tx, fx);
    double pitch = std::atan2(ty, fy);

    cv::Mat R = getRotationMatrixXYZ(0.0, pitch, yaw); // roll=0, pitch, yaw
    return R;
}

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

    // std::cout << "Rotation Matrix:\n" << R << std::endl;
    return R;
}


std::tuple<cv::Mat, cv::Point3f, cv::Point2f, float, bool> ImageMatcher::getAlignmentOld( const std::vector<cv::Point2f> newInputMatches, const std::vector<cv::Point2f> newTargetMatches, bool rotationOnly){
    
    // std::cout <<"Matches, target: " << targetMatches.size() << ", input: " << inputMatches.size() << std::endl;
    cv::Mat rotationMatrix = cv::Mat::eye(3, 3, CV_32F);
    cv::Point3f world_direction = cv::Point3f(0,0,0);
    float meanError = std::numeric_limits<float>::infinity();
    cv::Point2f flow = cv::Point2f( std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
    // inputImageGray.copyTo(oldImageGray);

    if(newTargetMatches.size() < 50){
        std::cout << " Not enough Matches found (atleast 50)" << std::endl;
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

    std::cout << "FOE: (" << foe.point.x << ", " << foe.point.y << "), residual: " << foe.residual << ", rotation only: " << rotationOnlyFlag << std::endl;

    std::cout << "Homography inliers: " << hInliers << ", Essential inliers: " << eInliers << ", tau_H: " << tau_H << std::endl;

    // H = [ s*cosθ  -s*sinθ  tx ]
    //     [ s*sinθ   s*cosθ  ty ]

    // double tx = H.at<double>(0,2);
    // double ty = H.at<double>(1,2);
    // meanError = std::sqrt(tx*tx + ty*ty);

    // std::cout << "Estimated translation magnitude (mean error): " << meanError << std::endl;
    
    if ( (!H.empty() && ((tau_H > 0.7) || rotationOnlyFlag)) && 0) {
        // rotationMatrix = solvePureRotation();
        cv::Mat bestR = computeRotation(H, hInliers, maskH);
        cv::transpose(bestR, rotationMatrix);
    }
    else{

        cv::Mat bestR, bestT, bestT64, inliersE;
        int inlierCount = cv::recoverPose(EssentialMat, newInputMatches, newTargetMatches, cameraMatrix, bestR, bestT64, inliersE);
        // std::cout << " Recoverpose inliers " << inlierCount << std::endl;
        std::vector<cv::Point2f> inliers1, inliers2;
        for(int i = 0; i < inliersE.rows; i++) {
            if(inliersE.at<uchar>(i)) {
                inliers1.push_back(newInputMatches[i]);
                inliers2.push_back(newTargetMatches[i]);
            }
        }
        // std::cout << " Before ReprojectionError " << std::endl;
        meanError = getReprojectionError(inliers1, inliers2, bestR, bestT64);
        // std::cout << " After ReprojectionError " << std::endl;
        flow = getOpticalFlow(inliers1, inliers2);
        float flow_norm = std::sqrt(flow.x*flow.x + flow.y*flow.y);
        // error3d.x = flow.x;
        // error3d.y = flow.y;
        // error3d.z = 
        rotationMatrix = bestR.clone();
        cv::transpose(bestR, rotationMatrix);
        // bestT64.convertTo(bestT, CV_32F);
        cv::Mat bestT_ = - bestR.t() * bestT64;
        bestT_.convertTo(bestT, CV_32F);
        // cv::Mat t_inv, t_inv64;
        // t_inv = bestT * meanError;
        // t_inv64 = -bestT64 * bestR.t() * meanError;
        // t_inv64.convertTo(t_inv, CV_32F);
        // world_direction = cv::Point3f(t_inv.at<float>(0,0), t_inv.at<float>(1,0), t_inv.at<float>(2,0));
        world_direction = cv::Point3f(bestT.at<float>(0,0)*flow_norm, bestT.at<float>(1,0)*flow_norm, bestT.at<float>(2,0)*flow_norm);

        if ((std::abs(bestT.at<float>(2,0)) > std::abs(bestT.at<float>(1,0)) + std::abs(bestT.at<float>(0,0))) && flow_norm < 10) world_direction.z *= 10;
        meanError = flow_norm;
        // std::cout << "Estimated flow direction: " << flow << std::endl;
        // std::cout << "Estimated translation matrix: " << std::endl << bestT << std::endl;
        // std::cout << "world direction: " << world_direction << std::endl;
    }
    std::cout << "Estimated translation magnitude (mean error): " << meanError << std::endl;
  
    return {rotationMatrix, world_direction, flow, meanError, true};
}

std::tuple<cv::Mat, cv::Point3f, float, bool> ImageMatcher::getAlignment( const Matches& matchedPoints, const cv::Mat& frame){

    pnec::RelativePoseEstimatorOld estimator(K);

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
        for(auto cov : matchedPoints.covariances){
            cv::Matx22f H(cov.x, cov.y, cov.y, cov.z);
        
            // Inverse Hessian = 2D Covariance Matrix Sigma_2D
            covariances.push_back(H.inv(cv::DECOMP_SVD));
        }
        // covariances = matchedPoints.covariances;
    }
    std::vector<pnec::MatchData> matches;
    Eigen::Matrix3d K_inv = K.inverse();
    pnec::UnscentedTransform ut;

    // std::cout << "Camera Intrinsics K:\n" << K << std::endl;
    // std::cout << "Camera Intrinsics K_inv:\n" << K_inv << std::endl;
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

        // std::cout << "Match " << i << ": input(" << inputMatches[i].x << ", " << inputMatches[i].y 
        //           << "), target(" << targetMatches[i].x << ", " << targetMatches[i].y 
        //           << "), cov2d: [" << covariances[i](0,0) << ", " << covariances[i](0,1) 
        //           << "; " << covariances[i](1,0) << ", " << covariances[i](1,1) 
        //           << "]" << std::endl;
    }
    
    cv::Mat Rf, tf, inliersE, maskE;
    if (t_init.isZero()) {
        cv::Mat EssentialMat = cv::findEssentialMat(inputMatches, targetMatches, cameraMatrix, cv::RANSAC, 0.999, 2.0, 2000, maskE);
        int inlierCount = cv::recoverPose(EssentialMat, inputMatches, targetMatches, cameraMatrix, Rf, tf, inliersE);
        cv::Mat t64;
        tf.convertTo(t64, CV_64F);
        t_init = Eigen::Vector3d(t64.at<double>(0), t64.at<double>(1), t64.at<double>(2));
    }
    
    double totalError_ = 0.0d;
    // R = cv::matToMatrix3d(Rf);
    
    estimator.estimate(matches, R, t_dir, t_init, totalError_);

    // To get cam1_to_cam2:
    Eigen::Matrix3d R_12 = R.transpose();
    Eigen::Vector3d t_12 = -R_12 * t_dir;  
    t_init = t_dir;
    cv::Mat R_cv;
    cv::Point3f t_cv;
    R_cv = matrix3dToMat(R);
    t_cv = cv::Point3f(t_dir.x(), t_dir.y(), t_dir.z());

    cv::Mat t_mat = cv::Mat(t_cv);
    float totalError;
    // if (cv::norm(t_cv) < 1e-6) totalError = 0;
    // else 
    totalError = ImageMatcher::getSampsonPixelError(inputMatches,targetMatches, R_cv, t_mat);
    if (totalError < 1) totalError *= 100;

    return {R_cv, t_cv, totalError, true};
}

TrackingResult ImageMatcher::track_features(const cv::Mat& img_prev, const cv::Mat& img_next,
                                           const std::vector<cv::Point2f>& pts_next, 
                                           float max_bidirectional_error)
{
    TrackingResult result;

    if (pts_next.empty()) {
        std::cout << "Warning: pts_next is empty!" << std::endl;
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

    std::cout << "Input points: " << num_pts << " | Valid tracked points: " << valid_count << std::endl;
    return result;
}

/**
 * @brief Computes the 2x2 inverse Hessian (covariance matrix) for each tracked keypoint.
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