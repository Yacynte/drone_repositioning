#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>

struct YPRDeg {
    double yaw;   // Z
    double pitch; // Y
    double roll;  // X
};

static double wrapDeg(double a)
{
    a = std::fmod(a + 180.0, 360.0);
    if (a < 0) a += 360.0;
    return a - 180.0;
}

static double tanhRate(double errDeg, double maxRateDegS, double k)
{
    // k is “gain” in 1/deg (try 0.03 to 0.10)
    return maxRateDegS * std::tanh(k * errDeg);
}

// ZYX (yaw-pitch-roll) extraction
static YPRDeg rotmatToYPRDeg_ZYX(const cv::Mat& R)
{
    CV_Assert(R.rows == 3 && R.cols == 3);

    const double r00 = R.at<double>(0,0), r01 = R.at<double>(0,1), r02 = R.at<double>(0,2);
    const double r10 = R.at<double>(1,0), r11 = R.at<double>(1,1), r12 = R.at<double>(1,2);
    const double r20 = R.at<double>(2,0), r21 = R.at<double>(2,1), r22 = R.at<double>(2,2);

    double yaw, pitch, roll;

    // pitch = asin(-r20)
    pitch = std::asin(std::clamp(-r20, -1.0, 1.0));

    const double cp = std::cos(pitch);

    if (std::abs(cp) > 1e-8) {
        // roll = atan2(r21, r22)
        roll = std::atan2(r21, r22);
        // yaw  = atan2(r10, r00)
        yaw  = std::atan2(r10, r00);
    } else {
        // Gimbal lock: roll and yaw coupled
        roll = 0.0;
        yaw  = std::atan2(-r01, r11);
    }

    const double rad2deg = 180.0 / CV_PI;
    return { yaw * rad2deg, pitch * rad2deg, roll * rad2deg };
}
