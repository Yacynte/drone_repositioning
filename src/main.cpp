#include "ImageMatcher.h"
#include "MetadataClient.h"
#include "RtspReader.h"
#include "AlgoLogger.hpp"
#include "Utils.h"
#include "Matches.hpp"

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
constexpr float kMaxRotationRate = 5.0f;
constexpr int kReprojectWindowSize = 10;

// Open either a local camera or remote stream depending on the selected mode.
bool openCapture(const std::string& mode,
                 const std::string& streamUrl,
                 int cameraIndex,
                 cv::VideoCapture& cap) {
    if (mode == "live") {
        if (!cap.open(cameraIndex, cv::CAP_V4L2)) {
            std::cerr << "Warning: cv::CAP_V4L2 failed, trying default backend..." << std::endl;
            if (!cap.open(cameraIndex)) return false;
        }
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        return true;
    }

    if (mode == "stream" && !(streamUrl.find("tcp") != std::string::npos)) {
        while (!cap.open(streamUrl, cv::CAP_FFMPEG)) {
            std::cout << "Waiting for stream to be available: " << streamUrl << std::endl;
            usleep(1000 * 1000);
        }
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        return true;
    }
    if (mode == "stream" && streamUrl.find("tcp") != std::string::npos) {
        return true; // The RTSP reader will handle the connection.
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
    const char* memoryName = getStr(flags, "--memory", "/sp_sg_matches").c_str();
    const int cameraIndex = getInt(flags, "--camera", 0);
    const bool unrealTest = (getInt(flags, "--unreal", 0) != 0);

    const std::string cmdIp = getStr(flags, "--cmd_ip", "0.0.0.0");
    const int cmdPort = getInt(flags, "--cmd_port", 9020);
    const std::string msgIp = getStr(flags, "--relay_ip", "0.0.0.0");
    const int msgPort = getInt(flags, "--relay_port", 9010);

    const std::string logPath = getStr(flags, "--log", "../data/");
    const std::string targetImagePath = getStr(flags, "--target", "../target.png");
    const int imgHeight = getInt(flags, "--imgHeight", 640);
    const int imgWidth = getInt(flags, "--imgWidth", 480);

    // bool hardStop = false;

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
    if (!client.StartConnectionHandlerUDP(cmdIp, cmdPort, "command_handler")) {
        std::cerr << "Failed to start command handler. Ensure port " << cmdPort << " is available." << std::endl;
        return -1;
    }
    client.startReceiver();

    // if (!client.StartConnectionHandler(msgIp, msgPort, "metadata_server")) {
    //     std::cerr << "Failed to start metadata server connection handler. Ensure port " << msgPort << " is available." << std::endl;
    //     return -1;
    // }
    // std::cout << "Connected to metadata server" << std::endl;

    // Open the selected video source and set the requested resolution.
    cv::VideoCapture cap;
    if (!openCapture(mode, streamUrl, cameraIndex, cap)) {
        std::cerr << "Error: Cannot open input source for mode '" << mode << "'." << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, imgWidth);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, imgHeight);
    if (!cap.isOpened() && !(streamUrl.find("tcp") != std::string::npos)) {
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
    std::string cam = std::to_string(cameraIndex);
    if (mode == "stream" ) {
       cam = streamUrl;
    }
    // std::cout << "To launch Python process for matches.py "<< std::endl;
    // launch_python(targetImagePath);
    launch_python_posix(targetImagePath);
    std::cout << "Launched Python process for matches.py with target image: " << targetImagePath << std::endl;
    // give Python time to init models and create shm
    std::this_thread::sleep_for(std::chrono::seconds(5));

    SPSGReader matchesReader;
    std::cout << "Matches reader initialized with shared memory: " << memoryName << std::endl;
        
    cv::Mat cameraMatrix = (cv::Mat_<float>(3,3) << 
                    imgWidth / 2.0f, 0,            imgWidth / 2.0f,
                    0,            imgWidth / 2.0f, imgHeight / 2.0f,
                    0,            0,            1.0f);
    // Load the target image matcher used for alignment estimation.
    ImageMatcher matcher(targetImagePath, cameraMatrix);
    std::cout << "Image matcher initialized with target image: " << targetImagePath << std::endl;
    // cv::Mat frame;
    // std::vector<int> directionHistory;
    std::vector<float> reprojectErrors;
    // cv::Point3f lastCmdVx(0.0f, 0.0f, 0.0f);
    // cv::Point3f lastAngleRate(0.0f, 0.0f, 0.0f);
    bool hasStarted = false;
    bool complete = false;
    bool atTarget = false;
    double arrivalTime = 0.0;
    // double oldMeanError = std::numeric_limits<double>::infinity();

    // Main repositioning loop: wait for start, process frames, and send commands.
    while (true) {
        if (client.stopRepositioning.load() || complete) {
            std::cout << "Received command to stop repositioning or arrived at target" << std::endl;
            std::stringstream ss;
            ss << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << "-1" << "\n";
            std::string data_to_send = ss.str();
            if (!client.SendMetadata(data_to_send)){
                std::cout << "Could not send Stop command \n";
            }
            // reader.sendStopSignal();
            break;
        }

        if (!client.startRepositioning.load() && !hasStarted) {
            std::cout << "Waiting to start the repositioning system" << std::endl;
            usleep(1000 * 1000);
            continue;
        }

        // if (client.pauseRepositioning.load()) {
        //     std::cout << "Received command to pause repositioning" << std::endl;
        //     while (!client.resumeRepositioning.load()) {
        //         usleep(1000 * 100);
        //     }
        //     std::cout << "Resuming repositioning" << std::endl;
        // }

        while (client.pauseRepositioning.load()){
            std::cout << "Received command to pause repositioning" << std::endl;
            usleep(1000 * 100);
            if (client.resumeRepositioning.load() || client.stopRepositioning.load()) {
                std::cout << "Resuming repositioning" << std::endl;
                break;
            }
        }

        hasStarted = true;
        cv::Mat frame;
        if (!reader.getFrame(frame)) {
            std::cout << "Waiting to receive image \n";
            // Skip processing when no new frame is available.
            continue;
        }

        auto result = matchesReader.read();
        if (!result.has_value()) {
                std::cout << "Python shut down, exiting.\n";
                break;
            }
        Matches matches = result.value();

        if (!matches.newMatches){
            // std::cout << "Waiting for new matches....\n";
            continue;
        }

        // Compute alignment direction and error metrics from the current frame.
        const double imgTs = AlgoLogger::nowWallSec();
        // const auto [rotationMatrix, directionCv, flow, transError, success] = matcher.getAlignmentDirection(frame, client.rotationOnly.load());
        const auto [rotationMatrix, direction, transError_, success] = matcher.getAlignment(matches, frame);
        // const float rotError = std::acos((cv::trace(rotationMatrix)[0] - 1.0f) / 2.0f) * 180.0f / CV_PI;
     
        // Convert estimated rotation and translation to the appropriate coordinate frame.
        const cv::Point3f rotationVec = rotmatToYPRDeg_XYZ(rotationMatrix);
        bool hasNaNRot = std::isnan(rotationVec.x) || std::isnan(rotationVec.y) || std::isnan(rotationVec.z);
        bool hasNanTrans = std::isnan(direction.x) || std::isnan(direction.y) || std::isnan(direction.z) || std::isnan(transError_);
        // std::cout << "rotation vec: x=" << rotationVec.x << " y=" << rotationVec.y << " z=" << rotationVec.z << std::endl;
        // std::cout << "direction: x=" << direction.x << " y=" << direction.y << " z=" << direction.z << std::endl;
        
        if(hasNaNRot) {
            std::cerr << "Warning: NaN detected in rotation or translation vector, skipping this frame." << std::endl;
            continue;
        }
        float transError = transError_;
        if (hasNanTrans) transError = 0;
        if (!success) {
            std::stringstream ss;
            ss << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << "0" << "\n";
            std::string data_to_send = ss.str();
            if (!client.SendMetadata(data_to_send)){
                std::cout << "Could not send Stop command \n";
            }
            std::cerr << "Warning: GetAlignmentDirection failed." << std::endl;
            continue;
        }
        
        cv::Point3f directionCv(direction.x * transError, direction.y * transError, direction.z * transError);
        // const cv::Point3f rotationVec = rotmatRQ_YZX(rotationMatrix);
        const cv::Point3f rotation = rotationVec; // Convert to degrees if needed);
        const cv::Point3f translation = unrealTest ? ConvertCVToUE(directionCv) : directionCv;
        // const cv::Point3f px_error = unrealTest ? ConvertCVToUE(trans_vec) : trans_vec;
        
        // Compute command velocities, then apply smoothing for stability.
        cv::Point3f angleRateCmd = kMaxRotationRate * activation(rotation, kRotationGain);
        cv::Point3f cmdVx = kMaxVelocity * activation(translation, kTranslationGain);
        

        // const int meanPreviousDirection = (directionHistory.size() < 8)
        //     ? -2
        //     : std::round(std::accumulate(directionHistory.begin(), directionHistory.end(), 0.0) / directionHistory.size());
        // if (meanPreviousDirection == 0) {
        //     cmdVx.x = 0.0f;
        // }

        // cmdVx = kVelocitySmoothing * cmdVx + (1.0f - kVelocitySmoothing) * lastCmdVx;
        // angleRateCmd = kVelocitySmoothing * angleRateCmd + (1.0f - kVelocitySmoothing) * lastAngleRate;

        std::cout << "rotation: x=" << rotation.x << " y=" << rotation.y << " z=" << rotation.z << std::endl;
        std::cout << "translation: x=" << translation.x << " y=" << translation.y << " z=" << translation.z << std::endl;
        std::cout << "transError: " << transError << std::endl;
        
        
        std::string dataToSend;
        if (client.respositionFunc(angleRateCmd, cmdVx, rotation, translation, dataToSend, unrealTest) ) {
            std::stringstream ss;
            ss << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << "0" << "\n";
            std::string data_to_send = ss.str();
            if (!client.SendMetadata(data_to_send)){
                std::cout << "Could not send Stop command \n";
            }
            std::cout << "Arrived at target" << std::endl;
            atTarget = true;
            if (arrivalTime == 0.0) arrivalTime = AlgoLogger::nowWallSec();
        }
        else {
            atTarget = false;
            arrivalTime = 0.0;
        }

        double currentTime = AlgoLogger::nowWallSec();
        
        if (atTarget && ((currentTime - arrivalTime > 2.0) ) && arrivalTime > 0.0) {
            std::cout << "Maintained target position for 3 seconds, stopping repositioning" << std::endl;
            std::string dataToSend;
            std::stringstream ss;
            ss << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << "-1" << "\n";
            std::string data_to_send = ss.str();
            if (!client.SendMetadata(data_to_send)){
                std::cout << "Could not send Stop command \n";
            }
            complete = true;
        }

        // Log the command and error values for later analysis.
        const auto parsed = AlgoLogger::parseCommand6(dataToSend);
        auto unit_vect = translation / transError;
        logger.log(currentTime, transError, rotation, unit_vect, parsed);

    }

    matchesReader.stop();
    client.stopReceiver();
    return 0;
}
