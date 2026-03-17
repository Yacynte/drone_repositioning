#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
// #include <string>


class ImageMatcher {
public:
    cv::Point3f dxyz = cv::Point3f(0,0,0);
    ImageMatcher(const std::string& targetImagePath);

    // Returns a direction vector (dx, dy) to align input image with target
    cv::Point3f getAlignmentDisplacement(const cv::Mat& inputImage);
    cv::Point3f getAlignmentDisplacementRansac(const cv::Mat& inputImage);
    std::tuple<cv::Mat, cv::Point3f> getAlignmentDirection( const cv::Mat& inputImage = cv::Mat());


private:
    cv::Mat targetImageGray;
    cv::Mat inputImageGray;
    std::vector<cv::KeyPoint> targetKeypoints;
    cv::Mat targetDescriptors;
    cv::Mat cameraMatrix;
    std::vector<cv::Point2f> inputMatches, targetMatches;
    cv::Ptr<cv::SIFT> sift;
    // cv::Ptr<cv::BFMatcher> matcher;
    cv::FlannBasedMatcher matcherFlann;
    float width, height;
    // cv::TermCriteria criteria;
    cv::Point3f computeRotation();
    std::vector<cv::DMatch> goodMatcher(const cv::Mat& inputDescriptors);
    std::vector<cv::DMatch> gridFilterMatches(const std::vector<cv::DMatch>& matches,
                                                        const std::vector<cv::KeyPoint>& queryKps,
                                                        int gridCols = 5, int gridRows = 4, int maxPerCell = 150);
    void detectAndCompute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
    void detectAndComputegrid(const cv::Mat& image,
                                    std::vector<cv::KeyPoint>& keypoints,
                                    cv::Mat& descriptors,
                                    int gridX = 5,
                                    int gridY = 4,
                                    int maxPerCell = 50);
    double getReprojectionError(const std::vector<cv::Point2f>& pts1, 
                             const std::vector<cv::Point2f>& pts2, 
                             cv::Mat& R, cv::Mat& t);

    void findAnddecomposeEssentialMat(cv::Mat& bestR, cv::Mat& bestT);
    cv::Mat formTransf(const cv::Mat& R, const cv::Mat& t);
    int sumZCalRelativeScale(const cv::Mat& R, const cv::Mat& t);
};

template <typename T>
T clamp(T value, T low, T high) {
    return std::max(low, std::min(value, high));
}