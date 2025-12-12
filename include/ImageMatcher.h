#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <string>

class ImageMatcher {
public:
    ImageMatcher(const std::string& targetImagePath);

    // Returns a direction vector (dx, dy) to align input image with target
    double getAlignmentDisplacement(const cv::Mat& inputImage);
    std::pair<cv::Point3f, cv::Point3f> getAlignmentDirection(const cv::Mat& inputImage = cv::Mat());


private:
    cv::Mat targetImageGray;
    std::vector<cv::KeyPoint> targetKeypoints;
    cv::Mat targetDescriptors;
    cv::Mat cameraMatrix;
    std::vector<cv::Point2f> inputMatches, targetMatches;
    cv::Ptr<cv::SIFT> sift;
    cv::Ptr<cv::BFMatcher> matcher;

    void detectAndCompute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
    void findAnddecomposeEssentialMatrix(cv::Mat& bestR, cv::Mat& bestT);
    cv::Mat formTransf(const cv::Mat& R, const cv::Mat& t);
    int sumZCalRelativeScale(const cv::Mat& R, const cv::Mat& t);
};

template <typename T>
T clamp(T value, T low, T high) {
    return std::max(low, std::min(value, high));
}