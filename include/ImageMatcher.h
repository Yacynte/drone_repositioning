#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "Utils.h"
#include "pnecOptimizer.hpp"
#include "Logger.h"

// Optical-flow tracking state carried between consecutive frames.
struct TrackingState {
    std::vector<cv::Point2f> prev_pts;     // Points in Frame (t-1)
    std::vector<int> target_indices;       // Maps prev_pts[i] to target_kps[target_indices[i]]
};

// Focus-of-expansion estimate: the image point motion vectors radiate from/to,
// used as a proxy for the drone's direction of travel.
struct FOEResult {
    cv::Point2f point;
    float residual;
};

// Result of Lucas-Kanade optical-flow tracking of a set of points into a new frame.
struct TrackingResult {
        std::vector<cv::Point2f> tracked_pts;
        std::vector<uint8_t> status;      // 1 if tracked successfully, 0 otherwise
        std::vector<float> err;           // Tracking error metric
    };

// Estimates the camera's pose (rotation + translation direction) relative to a
// fixed target image by matching SIFT features between the target and each new
// video frame. This is the core visual-alignment algorithm used by main.cpp to
// drive the repositioning loop.
class ImageMatcher {
public:
    // Loads and caches the target image (SIFT keypoints/descriptors) that all
    // subsequent frames are aligned against. K is the 3x3 camera intrinsics matrix.
    explicit  ImageMatcher(Logger& logger, const std::string& targetImagePath, const cv::Mat& K);
    cv::Point3f dxyz = cv::Point3f(0,0,0);

    // Returns a direction vector (dx, dy) to align input image with target
    cv::Point3f getAlignmentDisplacement(const cv::Mat& inputImage);
    // Same as getAlignmentDisplacement but uses estimateAffinePartial2D (RANSAC) for
    // the pixel-space displacement/zoom estimate instead of the SIFT-only heuristic.
    cv::Point3f getAlignmentDisplacementRansac(const cv::Mat& inputImage);
    // Detects/matches features in inputImage against the cached target, then
    // recovers relative rotation and translation direction (essential-matrix based).
    // Returns {rotationMatrix, translationDirection, opticalFlow, reprojectionError, success}.
    std::tuple<cv::Mat, cv::Point3f, cv::Point2f, float, bool> getAlignmentDirection( const cv::Mat& inputImage = cv::Mat(), bool rotationOnly = false);
    // Legacy variant of getAlignment() that recomputes SIFT matches itself instead of
    // taking externally-supplied (SuperPoint/LightGlue) matches. Kept for reference;
    // not called anywhere in the current pipeline.
    std::tuple<cv::Mat, cv::Point3f, cv::Point2f, float, bool> getAlignmentOld( const std::vector<cv::Point2f> newInputMatches, const std::vector<cv::Point2f> newTargetMatches, bool rotationOnly = false);
    // Main entry point used by main.cpp: given point correspondences already produced
    // by the external SuperPoint/LightGlue matcher (matchedPoints) and the current
    // frame, estimates {rotationMatrix, translationDirection, meanError, success} via
    // the PNEC (probabilistic normal epipolar constraint) optimizer.
    std::tuple<cv::Mat, cv::Point3f, float, bool> getAlignment( const Matches& matchedPoints, const cv::Mat& frame = cv::Mat() );

private:
    Logger& logger;
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
    cv::FlannBasedMatcher matcherFlann;
    float width, height;

    // Tracks pts_next from img_prev into img_next with Lucas-Kanade optical flow and
    // filters out tracks whose backward (img_next -> img_prev) reprojection error
    // exceeds max_bidirectional_error.
    TrackingResult track_features(const cv::Mat& img_prev, const cv::Mat& img_next,
            const std::vector<cv::Point2f>& pts_next, float max_bidirectional_error = 5.0f);
    // KNN-matches inputDescriptors against the cached target descriptors and applies
    // Lowe's ratio test to keep only unambiguous matches.
    std::vector<cv::DMatch> goodMatcher(const cv::Mat& inputDescriptors);
    // Buckets matches into a gridCols x gridRows grid over the query image and keeps
    // only the strongest maxPerCell matches per cell, to spread matches spatially
    // instead of letting them cluster in one high-texture region.
    std::vector<cv::DMatch> gridFilterMatches(const std::vector<cv::DMatch>& matches,
                                                        const std::vector<cv::KeyPoint>& queryKps,
                                                        int gridCols = 4, int gridRows = 3, int maxPerCell = 250);
    // Plain SIFT detect+compute over the whole image (no spatial filtering).
    void detectAndCompute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
    // Unused placeholder for a future Lucas-Kanade-based detect/compute path.
    void detectAndComputeLKFlow();

    // SIFT detect+compute with the same per-cell strongest-keypoint spatial filtering
    // as gridFilterMatches, applied at detection time (keeps at most maxPerCell
    // keypoints per gridX x gridY cell) so features are spread across the image.
    void detectAndComputegrid(const cv::Mat& image,
                                    std::vector<cv::KeyPoint>& keypoints,
                                    cv::Mat& descriptors,
                                    int gridX = 4,
                                    int gridY = 3,
                                    int maxPerCell = 500);
    // Mean reprojection error (pixels) of pts1 -> pts2 under the rigid transform (Rf, tf).
    float getReprojectionError(const std::vector<cv::Point2f>& pts1,
                                const std::vector<cv::Point2f>& pts2,
                                const cv::Mat& Rf, const cv::Mat& tf);

    // Mean Sampson distance (first-order approximation of geometric epipolar error,
    // in pixels) between corresponding points under the essential matrix built from
    // (R_in, t_in) and the camera intrinsics.
    float getSampsonPixelError(const std::vector<cv::Point2f>& pts1,
                                         const std::vector<cv::Point2f>& pts2,
                                         const cv::Mat& R_in, const cv::Mat& t_in);

    // Mean symmetric epipolar distance (pixels) between corresponding points under
    // the essential matrix built from (Rf, tf).
    float getSymmetricEpipolarDistance(const std::vector<cv::Point2f>& pts1,
                                        const std::vector<cv::Point2f>& pts2,
                                        const cv::Mat& Rf, const cv::Mat& tf);

    // Computes the essential matrix from the current inputMatches/targetMatches and
    // recovers the best-scoring (R, t) pose via cv::recoverPose.
    void findAnddecomposeEssentialMat(cv::Mat& bestR, cv::Mat& bestT);
    // Builds a 4x4 homogeneous transform matrix from a 3x3 rotation and a translation.
    cv::Mat formTransf(const cv::Mat& R, const cv::Mat& t);
    // Triangulates matches under (R, t) and sums the z-component (depth) of the
    // triangulated points as a relative-scale proxy.
    int sumZCalRelativeScale(const cv::Mat& R, const cv::Mat& t);
    // Decomposes a homography H into candidate rotations and picks the one consistent
    // with the visible inlier points ("pure rotation" pass — currently disabled, see
    // the "&& 0" guard around its call sites in ImageMatcher.cpp).
    cv::Mat computeRotation(cv::Mat& H, int& hInliers, cv::Mat& inlierMask);
    cv::Mat solvePureRotation();
    // Builds a rotation matrix from roll/pitch/yaw (radians), intrinsic X-Y-Z order.
    cv::Mat getRotationMatrixXYZ(double roll, double pitch, double yaw);
    // Estimates the focus-of-expansion point (where flow vectors converge/diverge)
    // and its residual fit error from a set of point correspondences.
    FOEResult computeFOE(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2);
    // Mean flow vector (pts2 - pts1) over all correspondences.
    cv::Point2f getOpticalFlow(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2);
    // Estimates a 2x2 pixel-space covariance per point from local image gradients
    // within a window_size x window_size patch, used to weight matches in the PNEC
    // optimizer (see pnecOptimizer.hpp / getAlignment()).
    std::vector<cv::Matx22f> compute_point_covariances(const cv::Mat& img_gray, const std::vector<cv::Point2f>& points, int window_size = 5);
};

template <typename T>
T clamp(T value, T low, T high) {
    return std::max(low, std::min(value, high));
}