#include "ImageMatcher.h"
#include "MetadataClient.h"
#include "RtspReader.h"
#include "AlgoLogger.hpp"
#include "Utils.h"

#include <opencv2/opencv.hpp>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

// Helper constants used for velocity scaling and smoothing.
namespace {
constexpr float kRotationGain = 0.1f;
constexpr float kTranslationGain = 0.2f;
constexpr float kVelocitySmoothing = 0.5f;
constexpr float kMaxVelocity = 50.0f;
constexpr float kMaxRotationRate = 10.0f;
constexpr int kReprojectWindowSize = 10;

// Open either a local camera or remote stream depending on the selected mode.
bool openCapture(const std::string& mode,
                 const std::string& streamUrl,
                 int cameraIndex,
                 cv::VideoCapture& cap) {
    if (mode == "live") {
        if (!cap.open(cameraIndex, cv::CAP_V4L2)) {
            std::cerr << "Warning: cv::CAP_V4L2 failed, trying default backend..." << std::endl;
            return cap.open(cameraIndex);
        }
        return true;
    }

    if (mode == "stream") {
        while (!cap.open(streamUrl, cv::CAP_FFMPEG)) {
            std::cout << "Waiting for stream to be available: " << streamUrl << std::endl;
            usleep(1000 * 1000);
        }
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        return true;
    }

    return false;
}

// Compute the mean of a vector of values, returning 0 if the vector is empty.
double meanValue(const std::vector<float>& values) {
    if (values.empty()) {
        return 0.0;
    }
    return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
}
} // namespace

int main(int argc, char** argv) {
    // Parse command-line flags for mode, network, and camera settings.
    const auto flags = parseFlags(argc, argv);

    const std::string mode = getStr(flags, "--mode", "stream");
    const std::string streamUrl = getStr(flags, "--rtsp", "rtsp://10.116.88.38:8554/mystream");
    const int cameraIndex = getInt(flags, "--camera", 0);
    const bool unrealTest = (getInt(flags, "--unreal", 0) != 0);

    const std::string cmdIp = getStr(flags, "--cmd_ip", "0.0.0.0");
    const int cmdPort = getInt(flags, "--cmd_port", 9020);
    const std::string msgIp = getStr(flags, "--relay_ip", "0.0.0.0");
    const int msgPort = getInt(flags, "--relay_port", 9010);

    const std::string logPath = getStr(flags, "--log", "../logs/");
    const std::string targetImagePath = getStr(flags, "--target", "../target.png");
    const int imgHeight = getInt(flags, "--imgHeight", 640);
    const int imgWidth = getInt(flags, "--imgWidth", 480);

    std::cout << "[mode] " << mode << "\n";
    std::cout << "[net] cmd=" << cmdIp << ":" << cmdPort
              << " msg=" << msgIp << ":" << msgPort << "\n";

    // Ensure log directory exists before creating the logger.
    const std::filesystem::path logDirectory = std::filesystem::path(logPath);
    if (!logDirectory.empty() && !std::filesystem::exists(logDirectory)) {
        std::filesystem::create_directories(logDirectory);
    }

    // Create a CSV logger for runtime data and command history.
    const std::string logFile = logPath + "AlgoLog_" + timeToUnderscoreString() + ".csv";
    AlgoLogger logger(logFile, /*write_header=*/true, /*flush_every_n=*/30);

    // Start the metadata command and relay connections.
    MetadataTcpClient client;
    if (!client.StartConnectionHandler(cmdIp, cmdPort, "command_handler")) {
        std::cerr << "Failed to start command handler. Ensure port " << cmdPort << " is available." << std::endl;
        return -1;
    }
    client.startReceiver();

    if (!client.StartConnectionHandler(msgIp, msgPort, "metadata_server")) {
        std::cerr << "Failed to start metadata server connection handler. Ensure port " << msgPort << " is available." << std::endl;
        return -1;
    }
    std::cout << "Connected to metadata server" << std::endl;

    // Open the selected video source and set the requested resolution.
    cv::VideoCapture cap;
    if (!openCapture(mode, streamUrl, cameraIndex, cap)) {
        std::cerr << "Error: Cannot open input source for mode '" << mode << "'." << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, imgWidth);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, imgHeight);
    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open camera/stream" << std::endl;
        return -1;
    }
    std::cout << "Camera/Stream opened" << std::endl;

    // Create the RTSP reader and start it, using Unreal mode if configured.
    RtspReader reader(streamUrl, imgWidth, imgHeight, unrealTest);
    std::cout << "Unreal test: " << unrealTest << std::endl;
    if (unrealTest) {
        reader.start();
    } else {
        reader.start(&cap);
    }

    // Load the target image matcher used for alignment estimation.
    ImageMatcher matcher(targetImagePath);

    cv::Mat frame;
    // std::vector<int> directionHistory;
    std::vector<float> reprojectErrors;
    // cv::Point3f lastCmdVx(0.0f, 0.0f, 0.0f);
    // cv::Point3f lastAngleRate(0.0f, 0.0f, 0.0f);
    bool hasStarted = false;
    bool complete = false;
    // double oldMeanError = std::numeric_limits<double>::infinity();

    // Main repositioning loop: wait for start, process frames, and send commands.
    while (true) {
        if (client.stopRepositioning.load() || complete) {
            std::cout << "Received command to stop repositioning or arrived at target" << std::endl;
            break;
        }

        if (!client.startRepositioning.load() && !hasStarted) {
            std::cout << "Waiting to start the repositioning system" << std::endl;
            usleep(1000 * 1000);
            continue;
        }

        if (client.pauseRepositioning.load()) {
            std::cout << "Received command to pause repositioning" << std::endl;
            while (!client.resumeRepositioning.load()) {
                usleep(1000 * 100);
            }
            std::cout << "Resuming repositioning" << std::endl;
        }

        hasStarted = true;
        if (!reader.getFrame(frame)) {
            // Skip processing when no new frame is available.
            continue;
        }

        // Compute alignment direction and error metrics from the current frame.
        const double imgTs = AlgoLogger::nowWallSec();
        const auto [rotationMatrix, directionCv] = matcher.getAlignmentDirection(frame);
        const float rotError = std::acos((cv::trace(rotationMatrix)[0] - 1.0f) / 2.0f) * 180.0f / CV_PI;
        const float transError = cv::norm(directionCv);

        // Maintain a short history of translation errors for smoothing.
        if (reprojectErrors.size() >= kReprojectWindowSize) {
            reprojectErrors.erase(reprojectErrors.begin());
        }
        reprojectErrors.push_back(transError);
        const double meanTransError = meanValue(reprojectErrors);

        // Convert estimated rotation and translation to the appropriate coordinate frame.
        const cv::Point3f rotationVec = rotmatToYPRDeg_ZYX(rotationMatrix);
        const cv::Point3f rotation = unrealTest ? ConvertCVToUERot(rotationVec) : ConvertCVToDrone(rotationVec);
        const cv::Point3f translation = unrealTest ? ConvertCVToUE(directionCv) : directionCv;

        // Compute command velocities, then apply smoothing for stability.
        cv::Point3f angleRateCmd = kMaxRotationRate * activation(rotation, kRotationGain);
        cv::Point3f cmdVx = kMaxVelocity * activation(translation, kTranslationGain);
        if (translation.x == 0.0f) {
            cmdVx.x = 0.0f;
        }

        // const int meanPreviousDirection = (directionHistory.size() < 8)
        //     ? -2
        //     : std::round(std::accumulate(directionHistory.begin(), directionHistory.end(), 0.0) / directionHistory.size());
        // if (meanPreviousDirection == 0) {
        //     cmdVx.x = 0.0f;
        // }

        // cmdVx = kVelocitySmoothing * cmdVx + (1.0f - kVelocitySmoothing) * lastCmdVx;
        // angleRateCmd = kVelocitySmoothing * angleRateCmd + (1.0f - kVelocitySmoothing) * lastAngleRate;

        std::cout << "rotation: x=" << rotation.x << " y=" << rotation.y << " z=" << rotation.z << std::endl;

        std::string dataToSend;
        if (client.respositionFunc(angleRateCmd, cmdVx, rotError, meanTransError, translation, dataToSend)) {
            std::cout << "Arrived at target" << std::endl;
            complete = true;
        }

        // if (meanTransError <= oldMeanError) {
        //     oldMeanError = meanTransError;
        // }

        // Log the command and error values for later analysis.
        const auto parsed = AlgoLogger::parseCommand6(dataToSend);
        logger.log(imgTs, AlgoLogger::nowWallSec(), meanTransError, parsed);

        // lastCmdVx = cmdVx;
        // lastAngleRate = angleRateCmd;
        // directionHistory.push_back((translation.x >= 0.0f) ? 1 : -1);
        // if (directionHistory.size() > 10) {
        //     directionHistory.erase(directionHistory.begin());
        // }
    }

    reader.stop();
    client.stopReceiver();
    return 0;
}
