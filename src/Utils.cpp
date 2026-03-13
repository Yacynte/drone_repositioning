#include "Utils.h"
// #include "AlgoLogger.hpp"

bool hasNaN(const cv::Point3f& p) {
    return std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z);
}


cv::Point3f activation(cv::Point3f x, float k , std::string function ) {
    if (function == "sigmoid") {
        cv::Point3f sig;
        // float k = 1.0f;  // increase → steeper (toward tanh), decrease → gentler
        sig.x = 2.0f / (1.0f + std::exp(-k * x.x)) - 1.0f;
        sig.y = 2.0f / (1.0f + std::exp(-k * x.y)) - 1.0f;
        sig.z = 2.0f / (1.0f + std::exp(-k * x.z)) - 1.0f;
        return sig;
    }
    else if (function == "tanh") {
        return cv::Point3f(
            std::tanh(x.x),
            std::tanh(x.y),
            std::tanh(x.z)
        );
    }
    else if (function == "gaussian") {
        return cv::Point3f(
            1.0f - std::exp(-x.x),
            1.0f - std::exp(-x.y),
            1.0f - std::exp(-x.z)
        );
    }
    return x; // cv::Point3f(0, 0, 0); // fallback
}


cv::Point3f ConvertCVToUE(const cv::Point3f& p)
{
    return cv::Point3f(
        p.z,      // forward
        p.x,      // right
        -p.y      // up
    );
}

cv::Point3f ConvertCVToUERot(const cv::Point3f& r)
{
    return cv::Point3f(
        r.y,      // Pitch (Y axis)
        -r.x,      // Yaw   (Z axis)
        r.z      // Roll  (X axis)
    );
}

double wrapDeg(double a)
{
    a = std::fmod(a + 180.0, 360.0);
    if (a < 0) a += 360.0;
    return a - 180.0;
}

double tanhRate(double errDeg, double maxRateDegS, double k)
{
    // k is “gain” in 1/deg (try 0.03 to 0.10)
    return maxRateDegS * std::tanh(k * errDeg);
}

// ZYX (yaw-pitch-roll) extraction
cv::Point3f rotmatToYPRDeg_ZYX(const cv::Mat& R)
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
    // return { yaw * rad2deg, pitch * rad2deg, roll * rad2deg };
    // return cv::Point3f((float)(roll * rad2deg), (float)(pitch * rad2deg), (float)(yaw * rad2deg));
    return cv::Point3f((float)(yaw * rad2deg), (float)(pitch * rad2deg), (float)(roll * rad2deg));
}


std::unordered_map<std::string,std::string> parseFlags(int argc, char** argv)
{
    std::unordered_map<std::string,std::string> kv;
    for(int i = 1; i + 1 < argc; ++i)
    {
        std::string key = argv[i];
        if(key.rfind("--",0) == 0)
        {
            kv[key] = argv[i+1];
            ++i;
        }
    }
    return kv;
}

std::string getStr(const std::unordered_map<std::string,std::string>& kv,
                          const std::string& key,
                          const std::string& def)
{
    auto it = kv.find(key);
    return (it != kv.end()) ? it->second : def;
}

int getInt(const std::unordered_map<std::string,std::string>& kv,
                  const std::string& key,
                  int def)
{
    auto it = kv.find(key);
    if(it == kv.end()) return def;
    try { return std::stoi(it->second); }
    catch(...) { return def; }
}


// Convert a double to a string where the decimal point is replaced with '_'.
// Example: 11829.695398 -> "11829_695398"
std::string timeToUnderscoreString(int precision) {
    const auto t_now = Clock::now().time_since_epoch();
    double value = std::chrono::duration<double>(t_now).count();
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    std::string s = oss.str();
    // Trim trailing zeros after the decimal point
    auto pos = s.find('.');
    if (pos != std::string::npos) {
        while (!s.empty() && s.back() == '0') s.pop_back();
        if (!s.empty() && s.back() == '.') s.pop_back();
    }
    std::replace(s.begin(), s.end(), '.', '_');
    return s;
}