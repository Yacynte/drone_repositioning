#pragma once
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <string>
#include <iomanip>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unistd.h>
#include <sys/types.h>
// #include <fmt/core.h>
// #include "AlgoLogger.hpp"

bool hasNaN(const cv::Point3f& p);

void launch_python( const std::string& target);

void launch_python_posix( const std::string& target);

cv::Point3f activation(cv::Point3f x, float k = 1.0f, std::string function = "sigmoid");


using Clock = std::chrono::steady_clock;
struct YPRDeg {
    double yaw;   // Z
    double pitch; // Y
    double roll;  // X
};

struct Matches {
    int n = 0;
    std::vector<cv::Point2f> kpts0;
    std::vector<cv::Point2f> kpts1;
    std::vector<float> scores;
    std::vector<cv::Point3f> covariances;
    bool newMatches = false;
};

cv::Point3f ConvertCVToUE(const cv::Point3f& p);

cv::Point3f ConvertCVToUERot(const cv::Point3f& r);

cv::Point3f ConvertCVToDrone(const cv::Point3f& r);

double wrapDeg(double a);

double tanhRate(double errDeg, double maxRateDegS, double k);

// ZYX (yaw-pitch-roll) extraction
cv::Point3f rotmatToYPRDeg_ZYX(const cv::Mat& R);
cv::Point3f rotmatToYPRDeg_XYZ(const cv::Mat& R);
cv::Point3f rotmatToRPYDeg_XYZ(const cv::Mat& R);
cv::Point3f rotmatRQ_YZX(const cv::Mat& R);

std::unordered_map<std::string,std::string> parseFlags(int argc, char** argv);

std::string getStr(const std::unordered_map<std::string,std::string>& kv,
                          const std::string& key,
                          const std::string& def);

int getInt(const std::unordered_map<std::string,std::string>& kv,
                  const std::string& key,
                  int def);


// Convert a double to a string where the decimal point is replaced with '_'.
// Example: 11829.695398 -> "11829_695398"
std::string timeToUnderscoreString(int precision = 6);
std::string timeToString();

cv::Mat matrix3dToMat(const Eigen::Matrix3d& R);
cv::Mat vector3dToMat(const Eigen::Vector3d& v);