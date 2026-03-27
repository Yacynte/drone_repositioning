#include "ImageMatcher.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include "Utils.h"


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


ImageMatcher::ImageMatcher(const std::string& targetImagePath) {
    // Load target image
    cv::Mat target = cv::imread(targetImagePath, cv::IMREAD_COLOR);
    if (target.empty()) {
        throw std::runtime_error("Could not load target image");
    }
    cv::cvtColor(target, targetImageGray, cv::COLOR_BGR2GRAY);

    // Initialize SIFT detector and BFMatcher
    // sift = cv::SIFT::create();
    sift = cv::SIFT::create(5000,    // nfeatures    — default 0 (unlimited, but keeps best), set explicitly
                            3,       // nOctaveLayers — default 3, increase for more features
                            0.05,    // contrastThreshold — default 0.04, LOWER = more features
                            10,      // edgeThreshold — default 10, HIGHER = more features  
                            1.6      // sigma — default 1.6, leave this
                            );
    // matcher = cv::BFMatcher::create(cv::NORM_L2);

    matcherFlann = cv::FlannBasedMatcher(cv::makePtr<cv::flann::KDTreeIndexParams>(5),
                                        cv::makePtr<cv::flann::SearchParams>(75));

    height = target.rows;
    width = target.cols;
    cameraMatrix = (cv::Mat_<float>(3,3) << 
                    width / 2.0f, 0,            width / 2.0f,
                    0,            width / 2.0f, height / 2.0f,
                    0,            0,            1.0f);
    // Detect and compute features for target
    cv::Mat targetDesc;
    // detectAndComputegrid(targetImageGray, targetKeypoints, targetDescriptors);
    detectAndCompute(targetImageGray, targetKeypoints, targetDesc);
    targetDesc.convertTo(targetDescriptors, CV_32F);
    
    std::cout << "Target keypoints size: " << targetKeypoints.size() << std::endl;
}

void ImageMatcher::detectAndCompute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
    sift->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
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
    std::cout << "All Keypoints detected: " << allKeypoints.size() << std::endl;
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
    std::vector<std::vector<cv::DMatch>> matchesBA;
    matcherFlann.knnMatch(targetDescriptors, inputDescriptors, matchesBA, 2);
    // matcher->knnMatch(targetDescriptors, inputDescriptors, matchesBA, 2);

    // Apply Lowe's ratio test and cross-check
    const float ratio = 0.75f;

    std::vector<cv::DMatch> goodAB, goodBA;

    for (const auto& m : matchesAB)
        if (m.size() == 2 && m[0].distance < ratio * m[1].distance)
            goodAB.push_back(m[0]);

    return goodAB;
    for (const auto& m : matchesBA)
        if (m.size() == 2 && m[0].distance < ratio * m[1].distance)
            goodBA.push_back(m[0]);

    // Cross-check: keep only matches that are mutual best matches
    std::vector<cv::DMatch> crossCheckedMatches;
    for (const auto& mAB : goodAB)
    {
        for (const auto& mBA : goodBA)
        {
            if (mAB.queryIdx == mBA.trainIdx &&
                mAB.trainIdx == mBA.queryIdx)
            {
                crossCheckedMatches.push_back(mAB);
                break;
            }
        }
    }
    return crossCheckedMatches;
}


std::vector<cv::DMatch> ImageMatcher::gridFilterMatches(const std::vector<cv::DMatch>& matches, 
                                                        const std::vector<cv::KeyPoint>& queryKps, 
                                                        int gridCols, int gridRows, int maxPerCell)
{
    float cellW = width  / gridCols;
    float cellH = height / gridRows;

    std::cout<< "width: "<< width << " height: " << height <<std::endl;
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
    int inlierCount = cv::recoverPose(EssentialMat, inputMatches, targetMatches, cameraMatrix, bestR, bestT, inliersE);
    // Now compute translation-like residual in pixels:
    cv::Point2f dxy = rotationCompensatedResidual(inputMatches, targetMatches, cameraMatrix, bestR, inliersE);
    float dz = rotationCompensatedZoom(inputMatches, targetMatches, cameraMatrix, bestR, inliersE);
    dxyz.x = dxy.x;
    dxyz.y = dxy.y;
    dxyz.z = dz;
}


std::tuple<cv::Mat, cv::Point3f> ImageMatcher::getAlignmentDirection( const cv::Mat& inputImage){
    if (!inputImage.empty()){
        cv::Mat inputGray;
        cv::cvtColor(inputImage, inputImageGray, cv::COLOR_BGR2GRAY);
        // std::cout << "Converted to Gray" << std::endl;
        std::vector<cv::KeyPoint> inputKeypoints;
        cv::Mat inputDescriptors;
        // detectAndComputegrid(inputImageGray, inputKeypoints, inputDescriptors);
        // std::cout << "Computed Keypoints and Descriptors" << std::endl;
        cv::Mat inputDesc;
        detectAndCompute(inputImageGray, inputKeypoints, inputDesc);
        inputDesc.convertTo(inputDescriptors,   CV_32F);
        std::vector<cv::DMatch> goodMatches;
        // goodMatches = goodMatcher(inputDescriptors);
        auto matches = goodMatcher(inputDescriptors);
        goodMatches = gridFilterMatches(matches, inputKeypoints);

        // std::cout << "Matched Descriptors" << std::endl;
        if (goodMatches.empty()) return {cv::Mat::eye(3, 3, CV_32F), cv::Point3f(0,0,0)};
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
        // std::cout << "Prepared Matches" << std::endl;
    }

    std::cout << "Matches, target: " << targetMatches.size() << ", input: " << inputMatches.size() << std::endl;
    // 1. Run Homography
    cv::Mat maskH;
    cv::Mat HomographyMat = cv::findHomography(inputMatches, targetMatches, cv::RANSAC, 2.0, maskH);
    float ratioH = (float)cv::countNonZero(maskH) / inputMatches.size();
    std::cout << " Homography ratio " << ratioH << std::endl;
    // 2. Run Essential
    cv::Mat maskE;
    cv::Mat EssentialMat = cv::findEssentialMat(inputMatches, targetMatches, cameraMatrix, cv::RANSAC, 0.999, 2.0, maskE);
    float ratioE = (float)cv::countNonZero(maskE) / inputMatches.size();
    std::cout << " Essential ratio " << ratioE << std::endl;

    cv::Mat rotationMatrix = cv::Mat::eye(3, 3, CV_32F);
    cv::Point3f world_direction = cv::Point3f(0,0,0);
    // 3. The Logic
    if (ratioH > 0.98 && ratioE < 0.90) {
        // This is almost certainly Pure Rotation.
        // Use the Homography-to-Rotation math.
        std::vector<cv::Mat> Rs, ts, normals;
        int nSolutions = cv::decomposeHomographyMat(HomographyMat, cameraMatrix, Rs, ts, normals);
        std::cout << " Homography nSolutions " << nSolutions << std::endl;
        // Filter to physically valid solutions
        // Need inlier points only for filtering
        std::vector<cv::Point2f> inlierInput, inlierTarget;
        for (int i = 0; i < maskH.rows; i++) {
            if (maskH.at<uchar>(i)) {
                inlierInput.push_back(inputMatches[i]);
                inlierTarget.push_back(targetMatches[i]);
            }
        }
        std::cout << " Homography nMatches " << inlierInput.size() << std::endl;
        // Filter using visible points
        std::vector<int> validSolutions;
        cv::filterHomographyDecompByVisibleRefpoints(
            Rs, normals,
            inlierInput,   // points from view 1
            inlierTarget,  // points from view 2
            validSolutions
        );

        // validSolutions now contains indices of physically plausible solutions
        std::cout << "Rotation only " << std::endl;
        auto bestR = Rs[validSolutions[0]];
        cv::transpose(bestR, rotationMatrix);
    } else {
        cv::Mat bestR, bestT, inliersE;
        // This is General Motion (even if it's a flat scene).
        // Use recoverPose.
        std::cout << " in RecoverPose " << std::endl;
        int inlierCount = cv::recoverPose(EssentialMat, inputMatches, targetMatches, cameraMatrix, bestR, bestT, inliersE);
        std::cout << " Recoverpose inliers " << inlierCount << std::endl;
        std::vector<cv::Point2f> inliers1, inliers2;
        for(int i = 0; i < inliersE.rows; i++) {
            if(inliersE.at<uchar>(i)) {
                inliers1.push_back(inputMatches[i]);
                inliers2.push_back(targetMatches[i]);
            }
        }
        std::cout << " Before ReprojectionError " << std::endl;
        double meanError = getReprojectionError(inliers1, inliers2, bestR, bestT);
        std::cout << " ReprojectionError " << meanError << std::endl;
        // if (meanError < 4){
        //     rotationMatrix = bestR;
        //     world_direction = cv::Point3f( bestT.at<double>(0,0), bestT.at<double>(1,0), bestT.at<double>(2,0));
        // }
        // rotationMatrix = bestR;
        cv::Mat t_inv;
        cv::transpose(bestR, rotationMatrix);
        // t_inv = -rotationMatrix * bestT;
        // rotationMatrix = bestR;
        t_inv = -bestT;
        t_inv *= meanError ; // scale translation

        std::vector<double> flows;
        double avg_disp = 0;
        for (size_t i = 0; i < inputMatches.size(); i++) {
            avg_disp += cv::norm(inputMatches[i] - targetMatches[i]);
            double d = cv::norm(inputMatches[i] - targetMatches[i]);
            flows.push_back(d);
        }
        avg_disp /= inputMatches.size();

        double var = 0;
        for (double f : flows) var += (f - avg_disp) * (f - avg_disp);
        var /= flows.size();

        if (avg_disp < 2.0 && var < 1.5) world_direction = cv::Point3f(0,0,0); // or keep previous
        else world_direction = cv::Point3f( t_inv.at<double>(0,0), t_inv.at<double>(1,0), t_inv.at<double>(2,0));
        std::cout << "Rotation and translation " << std::endl;
    }
    std::vector<cv::Point2f>().swap(inputMatches);
    std::vector<cv::Point2f>().swap(targetMatches);
    return {rotationMatrix, world_direction};
}


double ImageMatcher::getReprojectionError(const std::vector<cv::Point2f>& pts1, 
                             const std::vector<cv::Point2f>& pts2, 
                             cv::Mat& Rf, cv::Mat& tf) 
{    
    cv::Mat R, t;
    Rf.convertTo(R, CV_32F);
    tf.convertTo(t, CV_32F);
    // 1. Create Projection Matrices
    // P1 = K * [I | 0]
    cv::Mat P1 = cv::Mat::zeros(3, 4, CV_32F);
    // cv::Mat::eye(3, 3, CV_64F).copyTo(P1(cv::Rect(0, 0, 3, 3)));
    P1(cv::Rect(0, 0, 3, 3)) = cv::Mat::eye(3, 3, CV_32F);
    P1 = cameraMatrix * P1;

    // P2 = K * [R | t]
    cv::Mat P2 = cv::Mat::zeros(3, 4, CV_32F);
    R.copyTo(P2(cv::Rect(0, 0, 3, 3)));
    t.copyTo(P2(cv::Rect(3, 0, 1, 3)));
    P2 = cameraMatrix * P2;

    // 2. Triangulate Points to 4D (Homogeneous coordinates)
    cv::Mat pts4D;
    cv::triangulatePoints(P1, P2, pts1, pts2, pts4D);

    double totalError = 0;
    int count = pts4D.cols;
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < count; i++) {
        // 3. Convert from Homogeneous to 3D Cartesian [x, y, z]
        float w = pts4D.at<float>(3, i);
        cv::Mat X = (cv::Mat_<float>(3,1) << pts4D.at<float>(0, i) / w, 
                                               pts4D.at<float>(1, i) / w, 
                                               pts4D.at<float>(2, i) / w);
        objectPoints.push_back(cv::Point3f(pts4D.at<float>(0, i)/w, 
                                      pts4D.at<float>(1, i)/w, 
                                      pts4D.at<float>(2, i)/w));
        // 4. Project 3D point back to Image 2 plane
        // x = K * (R*X + t)
        cv::Mat x_hom = cameraMatrix * (R * X + t);
        cv::Point2f projected_pt(
            x_hom.at<float>(0) / x_hom.at<float>(2),
            x_hom.at<float>(1) / x_hom.at<float>(2)
        );

        // 5. Calculate Euclidean distance (L2 Norm)
        float error = cv::norm(projected_pt - pts2[i]);
        totalError += error;
    }

    // 6. Refine R and t using solvePnP
    // solvePnP outputs rotation as a 3x1 vector (Rodrigues), so we convert
    cv::Mat rvec, tvec;
    cv::Rodrigues(Rf, rvec); 
    tvec = tf.clone();

    // The "Refining" Magic: useExtrinsicGuess = true
    cv::solvePnP(objectPoints, pts2, cameraMatrix, cv::noArray(), 
                rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);

    // 7. Convert rvec back to Matrix
    cv::Mat refinedR;
    cv::Rodrigues(rvec, refinedR);
    // tvec is now the refined translation
    // tf = tvec;
    // Rf = refinedR;
    return totalError / count;
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

cv::Point3f ImageMatcher::computeRotation()
{
    // ── PASS 1: Pure rotation via Homography ──────────────────────────
    cv::Mat inlierMask;
    cv::Mat H = cv::findHomography(inputMatches, targetMatches, cv::RANSAC, 2.0, inlierMask);

    int hInliers = H.empty() ? 0 : cv::countNonZero(inlierMask);
    float inlierRatio = (float)hInliers / inputMatches.size();

    if (!H.empty() && inlierRatio > 0.5f && hInliers >= 20) {
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

        if (!validSolutions.empty()) {
            // Pick solution with roll closest to 0
            cv::Mat bestR;
            double minRoll = 1e9;
            for (int idx : validSolutions) {
                cv::Point3f angles = rotmatToYPRDeg_ZYX(Rs[idx]);
                if (std::abs(angles.z) < minRoll) {
                    minRoll = std::abs(angles.z);
                    bestR = Rs[idx];
                }
            }

            cv::Point3f delta_cv = rotmatToYPRDeg_ZYX(bestR);
            std::cout << "[PASS 1 - pure rotation] inliers: " << hInliers
                      << " ratio: " << inlierRatio
                      << " yaw: " << delta_cv.x
                      << " pitch: " << delta_cv.y << std::endl;

            // return cv::Point3f(-delta_cv.y, -delta_cv.x, 0.0f); // remap to Unreal, roll=0
            return cv::Point3f(delta_cv.x, delta_cv.y, 0.0f);
        }
    }

    // ── PASS 2: Affine fallback (rotation + translation) ─────────────
    cv::Mat inliers;
    cv::Mat A = cv::estimateAffinePartial2D(
        inputMatches, targetMatches, inliers,
        cv::RANSAC, 2.0, 2000, 0.99, 10
    );

    if (A.empty()) return cv::Point3f(0, 0, 0);

    int aInliers = 0;
    for (int i = 0; i < inliers.rows; ++i) aInliers += (inliers.at<uchar>(i) != 0);
    if (aInliers < 8) return cv::Point3f(0, 0, 0);

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
    double yawDeg   = std::atan2(tx, fx) * 180.0 / CV_PI;
    double pitchDeg = std::atan2(ty, fy) * 180.0 / CV_PI;

    // return cv::Point3f((float)-pitchDeg, (float)-yawDeg, 0.0f); // remap to Unreal
    return cv::Point3f((float)yawDeg, (float)pitchDeg, 0.0f);
}