#pragma once
// #include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "Utils.h"
#include "pnecOptimizer.hpp"
#include "Logger.h"
// #include <string>
struct TrackingState {
    std::vector<cv::Point2f> prev_pts;     // Points in Frame (t-1)
    std::vector<int> target_indices;       // Maps prev_pts[i] to target_kps[target_indices[i]]
};

struct FOEResult {
    cv::Point2f point;
    float residual;
};

struct TrackingResult {
        std::vector<cv::Point2f> tracked_pts;
        std::vector<uint8_t> status;      // 1 if tracked successfully, 0 otherwise
        std::vector<float> err;           // Tracking error metric
    };

class ImageMatcher {
public:
    explicit  ImageMatcher(Logger& logger, const std::string& targetImagePath, const cv::Mat& K);
    cv::Point3f dxyz = cv::Point3f(0,0,0);
    // ImageMatcher1(const std::string& targetImagePath);
    
    // Returns a direction vector (dx, dy) to align input image with target
    cv::Point3f getAlignmentDisplacement(const cv::Mat& inputImage);
    cv::Point3f getAlignmentDisplacementRansac(const cv::Mat& inputImage);
    std::tuple<cv::Mat, cv::Point3f, cv::Point2f, float, bool> getAlignmentDirection( const cv::Mat& inputImage = cv::Mat(), bool rotationOnly = false);
    // std::tuple<cv::Mat, cv::Point3f, cv::Point2f, float, bool> getAlignment( const std::vector<cv::Point2f> newTargetMatches, bool rotationOnly = false);
    std::tuple<cv::Mat, cv::Point3f, cv::Point2f, float, bool> getAlignmentOld( const std::vector<cv::Point2f> newInputMatches, const std::vector<cv::Point2f> newTargetMatches, bool rotationOnly = false);
    std::tuple<cv::Mat, cv::Point3f, float, bool> getAlignment( const Matches& matchedPoints, const cv::Mat& frame = cv::Mat() );

private:
    Logger& logger;
    // std::ostringstream ss;
    std::vector<cv::Matx22f> covariances;
    cv::Mat targetImageGray;
    cv::Mat inputImageGray;
    cv::Mat oldImageGray;
    TrackingState oldMatches;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_dir = Eigen::Vector3d::Zero();
    Eigen::Vector3d t_init = Eigen::Vector3d::Zero();
    bool need_sift_refresh;
    std::vector<cv::KeyPoint> targetKeypoints;
    cv::Mat targetDescriptors;
    cv::Mat cameraMatrix;
    Eigen::Matrix3d K;
    int matches_length;
    std::vector<cv::Point2f> inputMatches, targetMatches, targetMatches_;
    cv::Ptr<cv::SIFT> sift;
    // cv::Ptr<cv::BFMatcher> matcher;
    cv::FlannBasedMatcher matcherFlann;
    float width, height;

    // cv::TermCriteria criteria;
    TrackingResult track_features(const cv::Mat& img_prev, const cv::Mat& img_next,
            const std::vector<cv::Point2f>& pts_next, float max_bidirectional_error = 5.0f);
    std::vector<cv::DMatch> goodMatcher(const cv::Mat& inputDescriptors);
    std::vector<cv::DMatch> gridFilterMatches(const std::vector<cv::DMatch>& matches,
                                                        const std::vector<cv::KeyPoint>& queryKps,
                                                        int gridCols = 4, int gridRows = 3, int maxPerCell = 250);
    void detectAndCompute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
    void detectAndComputeLKFlow();
    
    void detectAndComputegrid(const cv::Mat& image,
                                    std::vector<cv::KeyPoint>& keypoints,
                                    cv::Mat& descriptors,
                                    int gridX = 4,
                                    int gridY = 3,
                                    int maxPerCell = 500);
    float getReprojectionError(const std::vector<cv::Point2f>& pts1, 
                                const std::vector<cv::Point2f>& pts2, 
                                const cv::Mat& Rf, const cv::Mat& tf);

    float getSampsonPixelError(const std::vector<cv::Point2f>& pts1,
                                         const std::vector<cv::Point2f>& pts2,
                                         const cv::Mat& R_in, const cv::Mat& t_in);

    float getSymmetricEpipolarDistance(const std::vector<cv::Point2f>& pts1,
                                        const std::vector<cv::Point2f>& pts2,
                                        const cv::Mat& Rf, const cv::Mat& tf);

    void findAnddecomposeEssentialMat(cv::Mat& bestR, cv::Mat& bestT);
    cv::Mat formTransf(const cv::Mat& R, const cv::Mat& t);
    int sumZCalRelativeScale(const cv::Mat& R, const cv::Mat& t);
    cv::Mat computeRotation(cv::Mat& H, int& hInliers, cv::Mat& inlierMask);
    cv::Mat getRotation();
    cv::Mat solvePureRotation();
    cv::Mat getRotationMatrixXYZ(double roll, double pitch, double yaw);
    FOEResult computeFOE(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2);
    cv::Point2f getOpticalFlow(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2);
    std::vector<cv::Matx22f> compute_point_covariances(const cv::Mat& img_gray, const std::vector<cv::Point2f>& points, int window_size = 5);
};

template <typename T>
T clamp(T value, T low, T high) {
    return std::max(low, std::min(value, high));
}