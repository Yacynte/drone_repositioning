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
    detectAndCompute(targetImageGray, targetKeypoints, targetDescriptors);
}

void ImageMatcher::detectAndCompute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
    sift->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
}

double ImageMatcher::getAlignmentDisplacement(const cv::Mat& inputImage) {
    cv::Mat inputGray;
    cv::cvtColor(inputImage, inputGray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> inputKeypoints;
    cv::Mat inputDescriptors;
    detectAndCompute(inputGray, inputKeypoints, inputDescriptors);

    std::vector<cv::DMatch> matches;
    matcher->match(inputDescriptors, targetDescriptors, matches);

    if (matches.empty()) return 0;

    cv::Point2f direction2D(0,0);
    float zMotion = 0; // inward/outward
    cv::Point2f center(inputGray.cols/2.0f, inputGray.rows/2.0f);

    for (const auto& m : matches) {
        const cv::KeyPoint& kpInput = inputKeypoints[m.queryIdx];
        const cv::KeyPoint& kpTarget = targetKeypoints[m.trainIdx];
        inputMatches.push_back(kpInput.pt);
        targetMatches.push_back(kpTarget.pt);
        // 2D translation
        direction2D += (kpTarget.pt - kpInput.pt);

        // Inward/outward: dot product with vector from center
        cv::Point2f vecToCenter = kpInput.pt - center;
        cv::Point2f motionVec = kpTarget.pt - kpInput.pt;
        // float dot = vecToCenter.dot(motionVec);

        // // If dot > 0 → moving outward, dot < 0 → moving inward
        // zMotion += (dot > 0) ? 1.0f : -1.0f;
    }

    direction2D.x /= matches.size();
    direction2D.y /= matches.size();
    // zMotion /= matches.size(); // average tendency
    double d_pixels = std::sqrt(direction2D.x*direction2D.x + direction2D.y*direction2D.y);

    return d_pixels;
}

cv::Point3f ImageMatcher::getAlignmentDirection(const cv::Mat& inputImage){
    if (!inputImage.empty()){
        cv::Mat inputGray;
        cv::cvtColor(inputImage, inputGray, cv::COLOR_BGR2GRAY);

        std::vector<cv::KeyPoint> inputKeypoints;
        cv::Mat inputDescriptors;
        detectAndCompute(inputGray, inputKeypoints, inputDescriptors);

        std::vector<cv::DMatch> matches;
        matcher->match(inputDescriptors, targetDescriptors, matches);
        // std::vector<cv::Point2f> inputMatches, targetMatches;

        if (matches.empty()) return cv::Point3f(0,0,0);
        for (const auto& m : matches) {
            const cv::KeyPoint& kpInput = inputKeypoints[m.queryIdx];
            const cv::KeyPoint& kpTarget = targetKeypoints[m.trainIdx];

            inputMatches.push_back(kpInput.pt);
            targetMatches.push_back(kpTarget.pt);
        }
    }
    cv::Mat translation, rotationMatrix;
    ImageMatcher::findAnddecomposeEssentialMatrix(rotationMatrix, translation);
    // double translation_norm = cv::norm(translation);
    double translation_norm = translation.at<double>(2,0);
    cv::Mat direction = (translation_norm > 1e-12) ? translation/translation_norm : cv::Mat::zeros(3,1, CV_64F);
    // std::cout << "Rotation Matrix: "<< rotationMatrix << std::endl;
    cv::Mat world_direction = rotationMatrix * direction;

    return cv::Point3f( world_direction.at<double>(0,0), world_direction.at<double>(1,0), 
                        world_direction.at<double>(2,0));
}

void ImageMatcher::findAnddecomposeEssentialMatrix(cv::Mat& bestR, cv::Mat& bestT){
    
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
