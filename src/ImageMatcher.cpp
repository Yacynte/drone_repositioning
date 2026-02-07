#include "ImageMatcher.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

ImageMatcher::ImageMatcher(const std::string& targetImagePath) {
    // Load target image
    cv::Mat target = cv::imread(targetImagePath, cv::IMREAD_COLOR);
    if (target.empty()) {
        throw std::runtime_error("Could not load target image");
    }
    cv::cvtColor(target, targetImageGray, cv::COLOR_BGR2GRAY);

    // Initialize SIFT detector and BFMatcher
    sift = cv::SIFT::create();
    matcher = cv::BFMatcher::create(cv::NORM_L2);
    float height = target.size().height;
    float width = target.size().height;
    cameraMatrix = (cv::Mat_<float>(3,3) <<
                    width,   0,      width / 2.0f,
                    0,       width,  height / 2.0f,
                    0,       0,      1.0f
                    );
    // Detect and compute features for target
    detectAndComputegrid(targetImageGray, targetKeypoints, targetDescriptors);
    std::cout << "Target keypoints size: " << targetKeypoints.size() << std::endl;
}

void ImageMatcher::detectAndCompute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
    sift->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
}

void ImageMatcher::detectAndComputegrid(const cv::Mat& image,
                                    std::vector<cv::KeyPoint>& keypoints,
                                    cv::Mat& descriptors,
                                    int gridX,
                                    int gridY,
                                    int maxPerCell)
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


float ImageMatcher::getAlignmentDisplacement(const cv::Mat& inputImage) {
    cv::Mat inputGray;
    cv::cvtColor(inputImage, inputGray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> inputKeypoints;
    cv::Mat inputDescriptors;
    detectAndComputegrid(inputGray, inputKeypoints, inputDescriptors);

    // std::vector<cv::DMatch> matches;
    // matcher->match(inputDescriptors, targetDescriptors, matches);

    std::vector<cv::DMatch> goodMatches;
    std::cout << "Input image keypoint size: " << inputKeypoints.size() << std::endl;
    goodMatches = goodMatcher(inputDescriptors);
    std::cout << "Found good matches of size: " << goodMatches.size() << std::endl;
    if (goodMatches.empty()) return 0; // cv::Point3f(0,0,0);
    
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
        zMotion += (dot > 0) ? 1.0f : -1.0f;
    }

    direction2D.x /= goodMatches.size();
    direction2D.y /= goodMatches.size();
    zMotion /= goodMatches.size(); // average tendency
    // double d_pixels = std::sqrt(direction2D.x*direction2D.x + direction2D.y*direction2D.y);
    float dist_z = cv::norm(direction2D);
    // zMotion *= dist_z; // scale by overall motion magnitude
    // return cv::Point3f(direction2D.x, direction2D.y, zMotion);
    return zMotion;
}

std::vector<cv::DMatch> ImageMatcher::goodMatcher(const cv::Mat& inputDescriptors) {
    // KNN match to find the two best matches for each descriptor
    std::vector<std::vector<cv::DMatch>> matchesAB;
    matcher->knnMatch(inputDescriptors, targetDescriptors, matchesAB, 2);
    std::vector<std::vector<cv::DMatch>> matchesBA;
    matcher->knnMatch(targetDescriptors, inputDescriptors, matchesBA, 2);

    // Apply Lowe's ratio test and cross-check
    const float ratio = 0.75f;

    std::vector<cv::DMatch> goodAB, goodBA;

    for (const auto& m : matchesAB)
        if (m.size() == 2 && m[0].distance < ratio * m[1].distance)
            goodAB.push_back(m[0]);

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

void ImageMatcher::findAnddecomposeEssentialMat(cv::Mat& bestR, cv::Mat& bestT){
    
    cv::Mat EssentialMat = cv::findEssentialMat(
                                inputMatches,     // std::vector<cv::Point2f> from current frame
                                targetMatches,    // std::vector<cv::Point2f> from target image
                                cameraMatrix,    // 3x3 intrinsic matrix
                                cv::RANSAC,      // method
                                0.999,           // confidence
                                0.1              // reprojection threshold in pixels
                            );
    // std::cout << "Computed Essential Matrix" << std::endl;
    cv::Mat R1, R2, t;
    cv::decomposeEssentialMat(EssentialMat, R1, R2, t);
    // std::cout << "decomposed Essential Matrix" << std::endl;
    // Build candidate pairs
    std::vector<std::pair<cv::Mat, cv::Mat>> candidates = {
        {R1,  t},
        {R1, -t},
        {R2,  t},
        {R2, -t}
    };

    int bestCount = -1;
    // cv::Mat bestR;
    for (const auto& [R, tCandidate] : candidates) {
        int posZ = sumZCalRelativeScale(R, tCandidate);
        if (posZ > bestCount) {
            bestCount = posZ;
            bestR = R.clone();
            bestT = tCandidate.clone();
        }
    }
}


std::pair<cv::Mat, cv::Point3f> ImageMatcher::getAlignmentDirection(const cv::Mat& inputImage){
    if (!inputImage.empty()){
        cv::Mat inputGray;
        cv::cvtColor(inputImage, inputGray, cv::COLOR_BGR2GRAY);
        // std::cout << "Converted to Gray" << std::endl;
        std::vector<cv::KeyPoint> inputKeypoints;
        cv::Mat inputDescriptors;
        detectAndCompute(inputGray, inputKeypoints, inputDescriptors);
        // std::cout << "Computed Keypoints and Descriptors" << std::endl;
        std::vector<cv::DMatch> goodMatches;
        goodMatches = goodMatcher(inputDescriptors);
        // matcher->match(inputDescriptors, targetDescriptors, matches);
        // std::vector<cv::Point2f> inputMatches, targetMatches;
        // std::cout << "Matched Descriptors" << std::endl;
        if (goodMatches.empty()) return {cv::Mat::eye(3, 3, CV_32F), cv::Point3f(0,0,0)};
        for (const auto& m : goodMatches) {
            const cv::KeyPoint& kpInput = inputKeypoints[m.queryIdx];
            const cv::KeyPoint& kpTarget = targetKeypoints[m.trainIdx];

            inputMatches.push_back(kpInput.pt);
            targetMatches.push_back(kpTarget.pt);
        }
        // std::cout << "Prepared Matches" << std::endl;
    }
    cv::Mat translation, rotationMatrix;
    // std::cout << "Find translation " << std::endl;
    findAnddecomposeEssentialMat(rotationMatrix, translation);
    //
    // findAnddecomposeEssentialMat(rotationMatrix, translation);
    // std::cout << "Translation Vector " << std::endl;
    double translation_norm = cv::norm(translation);
    // double translation_norm = translation.at<double>(2,0);
    cv::Mat direction = (translation_norm > 1e-12) ? translation/translation_norm : cv::Mat::zeros(3,1, CV_64F);
    // std::cout << "Rotation Matrix: "<< rotationMatrix << std::endl;
    cv::Mat world_transformation = rotationMatrix * direction;
    // std::cout << "World rotation matrix" << rotationMatrix << std::endl;
    // std::cout << "World Direction Vector " << direction << std::endl;
    // std::cout << "World Transformation Matrix " << world_transformation << std::endl;
    cv::Point3f world_direction = cv::Point3f( direction.at<double>(0,0), direction.at<double>(1,0), direction.at<double>(2,0));

    // cv::Mat world_rotation_mat = world_transformation(cv::Rect(0, 0, 3, 3)).clone();
    // std::cout << "World Rotation Matrix " << rotationMatrix << std::endl;
    // cv::Mat world_rotation_vec;
    // cv::Rodrigues(rotationMatrix, world_rotation_vec);

    // cv::Vec3d axis = world_rotation_vec / cv::norm(world_rotation_vec);
    // double angle   = cv::norm(world_rotation_vec);
    // double roll, pitch, yaw;

    // pitch = std::asin(-rotationMatrix.at<double>(2,0));

    // if (std::abs(std::cos(pitch)) > 1e-6) {
    //     roll = std::atan2(rotationMatrix.at<double>(2,1), rotationMatrix.at<double>(2,2));
    //     yaw  = std::atan2(rotationMatrix.at<double>(1,0), rotationMatrix.at<double>(0,0));
    // } else {
    //     // gimbal lock
    //     roll = std::atan2(-rotationMatrix.at<double>(1,2), rotationMatrix.at<double>(1,1));
    //     yaw = 0.0;
    // }

    // // const double rad2deg = 180.0 / CV_PI;
    // // roll  *= rad2deg;
    // // pitch *= rad2deg;
    // // yaw   *= rad2deg;

    // double roll_ue  = -pitch;
    // double pitch_ue =  yaw;
    // double yaw_ue   = -roll;

    // cv::Point3f Motion;
    // Motion.x =  std::cos(pitch_ue) * std::cos(yaw_ue);
    // Motion.y =  std::cos(pitch_ue) * std::sin(yaw_ue);
    // Motion.z =  std::sin(pitch_ue);

    // std::cout << "World Rotation Vector Matrix " << std::endl;
    // cv::Point3f world_rotation = cv::Point3f( Motion.x, Motion.y, std::sin(pitch));
    // std::cout << " return World Rotation Vector " << std::endl;
    
    
    return {rotationMatrix, world_direction};
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
    cv::Mat inputGray;
    cv::cvtColor(inputImage, inputGray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> inputKeypoints;
    cv::Mat inputDescriptors;
    detectAndComputegrid(inputGray, inputKeypoints, inputDescriptors);

    // std::vector<cv::DMatch> matches;
    // matcher->match(inputDescriptors, targetDescriptors, matches);

    std::vector<cv::DMatch> goodMatches;
    // std::cout << "Input image keypoint size: " << inputKeypoints.size() << std::endl;
    goodMatches = goodMatcher(inputDescriptors);
    // std::cout << "Found good matches of size: " << goodMatches.size() << std::endl;
    if (goodMatches.empty()) return cv::Point3f(0,0,0);

    for (const auto& m : goodMatches) {
        const cv::KeyPoint& kpInput = inputKeypoints[m.queryIdx];
        const cv::KeyPoint& kpTarget = targetKeypoints[m.trainIdx];
        inputMatches.push_back(kpInput.pt);
        targetMatches.push_back(kpTarget.pt);
    }

    // Robust affine (rotation + uniform scale + translation), rejects outliers
    cv::Mat inliers;
    cv::Mat A = cv::estimateAffinePartial2D(
        inputMatches, targetMatches, inliers,
        cv::RANSAC,
        3.0,     // reprojection threshold in pixels (tune: 2-5 px)
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
    // return cv::Point3f((float)tx, (float)ty, (float)z);
    // rotation (radians)
    double theta = std::atan2(c, a);

    // center displacement (pixels)
    cv::Point2f center(inputGray.cols * 0.5f, inputGray.rows * 0.5f);
    double cx = center.x, cy = center.y;
    double cxp = a*cx + b*cy + tx;
    double cyp = c*cx + d*cy + ty;

    double dxc = cxp - cx;
    double dyc = cyp - cy;

    // Return center shift, and zoom proxy
    return cv::Point3f((float)dxc, (float)dyc, (float)z);
}
