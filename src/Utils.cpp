#include "Utils.h"
// #include "AlgoLogger.hpp"

bool hasNaN(const cv::Point3f& p) {
    return std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z);
}


void launch_python(const std::string& target) {
    std::string cmd = "python3 src/matches.py"
                    //   " --camera " + camera +
                      " --target " + target + " &";  // & = background
    std::system(cmd.c_str());
}

void launch_python_posix(const std::string& target) {
    pid_t pid = fork();
    if (pid == 0) {
        // Child process
        execlp("python3", "python3", "src/matches_onnx.py", "--target", target.c_str(), nullptr);
        _exit(1); // Exits child if exec fails
    }
    // Parent C++ code continues immediately
}

cv::Mat vector3dToMat(const Eigen::Vector3d& v) {
    cv::Mat m(3, 1, CV_64F);
    m.at<double>(0) = v(0);
    m.at<double>(1) = v(1);
    m.at<double>(2) = v(2);
    cv::Mat m_float;
    m.convertTo(m_float, CV_32F);
    return m_float;
}

cv::Mat matrix3dToMat(const Eigen::Matrix3d& R) {
    cv::Mat m(3, 3, CV_64F);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m.at<double>(i, j) = R(i, j);
    // cv::Mat m_float;
    // m.convertTo(m_float, CV_32F);
    return m;
}

Eigen::Matrix3d matToMatrix3d(const cv::Mat& R) {
    Eigen::Matrix3d m;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m(i, j) = R.at<double>(i, j);

    return m;
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
        r.y,    // Unreal Pitch ← OpenCV pitch (negate: Y points down in CV, up in UE)
        r.x,    // Unreal Yaw   ← OpenCV yaw   (negate: Z forward flips handedness)
        r.z     // Unreal Roll  ← OpenCV roll   (same axis)
    );
}

cv::Point3f ConvertCVToDrone(const cv::Point3f& r)
{
    return cv::Point3f(
        r.x,    // Unreal roll  ← OpenCV pitch (negate: Y points down in CV, up in UE)
        r.y,    // Unreal pitch ← OpenCV yaw   (negate: Z forward flips handedness)
        r.z     // Unreal yaw   ← OpenCV roll   (same axis)
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

// XYZ (roll-pitch-yaw) extraction
cv::Point3f rotmatToYPRDeg_XYZ(const cv::Mat& R)
{
    CV_Assert( R.rows == 3 && R.cols == 3);
    const double r00 = R.at<double>(0,0), r01 = R.at<double>(0,1), r02 = R.at<double>(0,2);
    const double r10 = R.at<double>(1,0), r11 = R.at<double>(1,1), r12 = R.at<double>(1,2);
    const double r20 = R.at<double>(2,0), r21 = R.at<double>(2,1), r22 = R.at<double>(2,2);

    double yaw, pitch, roll;

    // pitch = asin(-r20)
    yaw = std::asin(std::clamp(-r20, -1.0, 1.0));

    const double cp = std::cos(yaw);

    if (std::abs(cp) > 1e-8) {
        // roll = atan2(r21, r22)
        pitch = std::atan2(r21, r22);
        // yaw  = atan2(r10, r00)
        roll  = std::atan2(r10, r00);
    } else {
        // Gimbal lock: roll and yaw coupled
        roll = 0.0;
        pitch  = std::atan2(-r01, r11);
    }

    const double rad2deg = 180.0 / CV_PI;
    // return { yaw * rad2deg, pitch * rad2deg, roll * rad2deg };
    return cv::Point3f((float)(roll * rad2deg), (float)(pitch * rad2deg), (float)(yaw * rad2deg));
    // return cv::Point3f((float)(yaw * rad2deg), (float)(pitch * rad2deg), (float)(roll * rad2deg));
}

cv::Point3f rotmatToRPYDeg_XYZ(const cv::Mat& R)
{
    CV_Assert(R.rows == 3 && R.cols == 3);
    const double r00 = R.at<double>(0,0), r01 = R.at<double>(0,1), r02 = R.at<double>(0,2);
    const double r10 = R.at<double>(1,0), r11 = R.at<double>(1,1), r12 = R.at<double>(1,2);
    const double r20 = R.at<double>(2,0), r21 = R.at<double>(2,1), r22 = R.at<double>(2,2);

    double roll, pitch, yaw;

    // XYZ: R = Rx * Ry * Rz
    pitch = std::asin(std::clamp(r02, -1.0, 1.0));  // pitch from r02, not -r20

    if (std::abs(std::cos(pitch)) > 1e-8) {
        roll = std::atan2(-r12, r22);  // ✓ XYZ roll
        yaw  = std::atan2(-r01, r00);  // ✓ XYZ yaw
    } else {
        // gimbal lock (pitch = ±90°)
        roll = std::atan2(r21, r11);
        yaw  = 0.0;
    }

    const double rad2deg = 180.0 / CV_PI;
    return cv::Point3f(
        (float)(roll  * rad2deg),  // x = roll
        (float)(pitch * rad2deg),  // y = pitch
        (float)(yaw   * rad2deg)   // z = yaw
    );
}

// ZYX (yaw-pitch-roll) extraction
cv::Point3f rotmatToYPRDeg_ZYX(const cv::Mat& R)
{
    CV_Assert(R.rows == 3 && R.cols == 3);

    // cv::Mat Q, R;
    // cv::RQDecomposition(rotationMatrix, Q, R); // R is now a clean rotation matrix
    // Now extract angles from R

    const float r00 = R.at<float>(0,0), r01 = R.at<float>(0,1), r02 = R.at<float>(0,2);
    const float r10 = R.at<float>(1,0), r11 = R.at<float>(1,1), r12 = R.at<float>(1,2);
    const float r20 = R.at<float>(2,0), r21 = R.at<float>(2,1), r22 = R.at<float>(2,2);

    float roll, pitch, yaw;

    // Ensure the values are sane before computing
    // float val = std::pow(r21, 2) + std::pow(r22, 2);
    // // Ensure we don't sqrt a negative number
    // float sqrt_val = std::sqrt(std::max(0.0f, val));

    // pitch = std::atan2(-r20, sqrt_val);

    pitch = std::asin(std::clamp(-r20, -1.0f, 1.0f));

    float cp = std::cos(pitch);

    if (std::abs(cp) > 1e-6){
        roll = std::atan2(r21, r22);
        yaw = std::atan2(r10, r00);

    }
    else {
        // Gimbal lock: roll and yaw coupled
        roll = 0.0;
        yaw  = std::atan2(-r01, r11);
    }

    const float rad2deg = 180.0 / CV_PI;
    // return { yaw * rad2deg, pitch * rad2deg, roll * rad2deg };
    return cv::Point3f(roll * rad2deg, pitch * rad2deg, yaw * rad2deg);
    // return cv::Point3f((float)(yaw * rad2deg), (float)(pitch * rad2deg), (float)(roll * rad2deg));
}

cv::Point3f rotmatRQ_YZX(const cv::Mat& R){

    //  Prepare output matrices for RQ Decomposition
    cv::Mat mtxR = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat mtxQ = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat Qx, Qy, Qz;

    // Execute RQ Decomposition
    // Pass the rotation matrix R as the input source
    cv::Vec3d eulerAngles = cv::RQDecomp3x3(R, mtxR, mtxQ, Qx, Qy, Qz);

    // 4. Output the extracted Euler Angles
    // std::cout << "Pitch (X-axis rotation) in degrees: " << eulerAngles[0] << std::endl;
    // std::cout << "Yaw   (Y-axis rotation) in degrees: " << eulerAngles[1] << std::endl;
    // std::cout << "Roll  (Z-axis rotation) in degrees: " << eulerAngles[2] << std::endl;

    return cv::Point3f(eulerAngles[2], eulerAngles[0], eulerAngles[1]);
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
    const auto t_now = std::chrono::system_clock::now().time_since_epoch();
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


// std::string timeToString() {
//     auto now = std::chrono::system_clock::now();
//     // {:%Y%m%d_%H%M%S} formats the time point directly
//     std::string timestamp = fmt::format("{:%Y%m%d_%H%M%S}", now);
// }