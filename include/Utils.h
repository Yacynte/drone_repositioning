#pragma once
#include <limits>
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
#include <thread>
#include "Logger.h"

// Grab-bag of small helpers shared across the C++ app: NaN checks, coordinate-frame
// conversions, rotation-matrix <-> Euler-angle extraction, CLI flag parsing, and
// timestamp/path utilities. See Utils.cpp for implementations.

// True if any component of p is NaN.
bool hasNaN(const cv::Point3f& p);

// Launches "python3 src/matches.py --target <target> &" via std::system(). Superseded
// by launch_python_posix() (used by main.cpp); kept for reference.
void launch_python( const std::string& target);

// The external SuperPoint/LightGlue matcher process (src_py/matches_onnx.py) started
// by launch_python_posix(): its pid and a thread that copies every line Python writes
// to stdout/stderr into the app log as "[python] <line>". stop() (also run by the
// destructor) waits for Python to exit, sends SIGTERM after timeoutSec, and joins the
// thread, so the logger is never used after main() returns. Declare it after the
// Logger it writes to.
class PythonProcess {
public:
    PythonProcess() = default;
    PythonProcess(pid_t pid, std::thread reader, Logger* logger)
        : pid_(pid), reader_(std::move(reader)), logger_(logger) {}
    PythonProcess(PythonProcess&&) = default;
    PythonProcess& operator=(PythonProcess&&) = default;
    ~PythonProcess() { stop(); }
    void stop(double timeoutSec = 3.0);
private:
    pid_t pid_ = -1;
    std::thread reader_;
    Logger* logger_ = nullptr;
};

// Forks and execs "python3 -u <onnx_matches> --target <target>" without blocking the
// caller, with Python's stdout and stderr piped into logger (see PythonProcess).
PythonProcess launch_python_posix(const std::string& onnx_matches, const std::string& target,
                                  Logger& logger);

// Squashes each component of x through the given activation ("sigmoid", "tanh", or
// "gaussian"; unrecognized names pass x through unchanged) with gain k, used to turn
// raw position/rotation error into a bounded command velocity.
cv::Point3f activation(cv::Point3f x, float k = 1.0f, std::string function = "sigmoid");


using Clock = std::chrono::steady_clock;
struct YPRDeg {
    double yaw;   // Z
    double pitch; // Y
    double roll;  // X
};

// One frame's worth of point correspondences between the target image and the
// current camera frame, as produced by the external SuperPoint/LightGlue process and
// read over shared memory by SPSGReader (see Matches.hpp).
struct Matches {
    int n = 0;
    std::vector<cv::Point2f> kpts0;        // target-image keypoints
    std::vector<cv::Point2f> kpts1;        // current-frame keypoints
    std::vector<float> scores;             // per-match confidence
    std::vector<cv::Point3f> covariances;  // per-match 2D covariance (packed xx, xy, yy)
    bool newMatches = false;               // false if this frame's matches were already consumed
    // Timing (wall clock, seconds since epoch; NaN if unknown), see Matches.hpp layout:
    uint32_t matchId = 0;      // Python's match-set counter
    uint32_t camFrameId = 0;   // RtspReader frame_id of the image Python matched
    double pyFrameTs = std::numeric_limits<double>::quiet_NaN();  // Python got that frame
    double pyDoneTs  = std::numeric_limits<double>::quiet_NaN();  // Python finished writing
    double recvTs    = std::numeric_limits<double>::quiet_NaN();  // C++ read the match set
    int skipped = 0;           // match sets Python wrote that C++ never read (overwritten)
};

// Remaps an OpenCV-convention point (x right, y down, z forward) into Unreal Engine's
// convention (x forward, y right, z up).
cv::Point3f ConvertCVToUE(const cv::Point3f& p);

// Remaps an OpenCV-convention rotation (roll/pitch/yaw) into Unreal Engine's
// pitch/yaw/roll ordering.
cv::Point3f ConvertCVToUERot(const cv::Point3f& r);

// Remaps an OpenCV-convention rotation into the drone controller's roll/pitch/yaw
// ordering.
cv::Point3f ConvertCVToDrone(const cv::Point3f& r);

// Wraps an angle in degrees into (-180, 180].
double wrapDeg(double a);

// tanh-shaped rate command: maxRateDegS * tanh(k * errDeg). k is a gain in 1/deg.
double tanhRate(double errDeg, double maxRateDegS, double k);

// Extracts yaw-pitch-roll Euler angles (degrees) from a 3x3 rotation matrix, ZYX
// (intrinsic) convention.
cv::Point3f rotmatToYPRDeg_ZYX(const cv::Mat& R);
// Extracts yaw-pitch-roll Euler angles (degrees) from a 3x3 rotation matrix, XYZ
// (intrinsic) convention. This is the convention used by the live pipeline (see
// main.cpp).
cv::Point3f rotmatToYPRDeg_XYZ(const cv::Mat& R);
// Extracts roll-pitch-yaw Euler angles (degrees) from a 3x3 rotation matrix, XYZ
// (intrinsic, R = Rx*Ry*Rz) convention.
cv::Point3f rotmatToRPYDeg_XYZ(const cv::Mat& R);
// Extracts Euler angles (degrees) from a 3x3 rotation matrix via RQ decomposition
// (cv::RQDecomp3x3), returned as (roll, yaw, pitch) — Y-Z-X ordering.
cv::Point3f rotmatRQ_YZX(const cv::Mat& R);

// Parses argv into a "--flag value" map. Flags without a following value are ignored.
std::unordered_map<std::string,std::string> parseFlags(int argc, char** argv);

// Looks up key in kv, returning def if absent.
std::string getStr(const std::unordered_map<std::string,std::string>& kv,
                          const std::string& key,
                          const std::string& def);

// Looks up key in kv and parses it as an int, returning def if absent or unparsable.
int getInt(const std::unordered_map<std::string,std::string>& kv,
                  const std::string& key,
                  int def);

// Formats the current wall-clock time as a fractional Unix timestamp with '.'
// replaced by '_' (e.g. 11829.695398 -> "11829_695398"). Unused by the live pipeline
// (which uses the no-arg overload below); kept for reference.
std::string timeToString(int precision = 6);
// Formats the current local time as "YYYY_MM_DD_HH_MM_SS", used to name log files
// (see main.cpp) and per-frame saved images (see RstpReader.cpp).
std::string timeToUnderscoreString();
// Expands a leading "~/" in path to $HOME; returns path unchanged otherwise (or if
// $HOME is unset).
std::string expandUser(const std::string& path);

// Copies a 3x3 Eigen rotation matrix into a CV_64F cv::Mat.
cv::Mat matrix3dToMat(const Eigen::Matrix3d& R);
// Copies an Eigen 3-vector into a 3x1 CV_32F cv::Mat.
cv::Mat vector3dToMat(const Eigen::Vector3d& v);