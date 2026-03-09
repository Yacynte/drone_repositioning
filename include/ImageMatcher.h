#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <string>

class ImageMatcher {
public:
    cv::Point3f dxyz = cv::Point3f(0,0,0);
    ImageMatcher(const std::string& targetImagePath);

    // Returns a direction vector (dx, dy) to align input image with target
    cv::Point3f getAlignmentDisplacement(const cv::Mat& inputImage);
    cv::Point3f getAlignmentDisplacementRansac(const cv::Mat& inputImage);
    std::pair<cv::Mat, cv::Point3f> getAlignmentDirection( const cv::Mat& inputImage = cv::Mat());


private:
    cv::Mat targetImageGray;
    std::vector<cv::KeyPoint> targetKeypoints;
    cv::Mat targetDescriptors;
    cv::Mat cameraMatrix;
    std::vector<cv::Point2f> inputMatches, targetMatches;
    cv::Ptr<cv::SIFT> sift;
    cv::Ptr<cv::BFMatcher> matcher;

    std::vector<cv::DMatch> goodMatcher(const cv::Mat& inputDescriptors);
    void detectAndCompute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
    void detectAndComputegrid(const cv::Mat& image,
                                    std::vector<cv::KeyPoint>& keypoints,
                                    cv::Mat& descriptors,
                                    int gridX = 5,
                                    int gridY = 4,
                                    int maxPerCell = 50);
    void findAnddecomposeEssentialMat(cv::Mat& bestR, cv::Mat& bestT);
    cv::Mat formTransf(const cv::Mat& R, const cv::Mat& t);
    int sumZCalRelativeScale(const cv::Mat& R, const cv::Mat& t);
};

template <typename T>
T clamp(T value, T low, T high) {
    return std::max(low, std::min(value, high));
}