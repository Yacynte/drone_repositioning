// ImageMatcher: the drone-repositioning entry point.
//
// Overall flow of main():
//   1. Parse CLI flags, set up logging (Logger for text logs, AlgoLogger for CSV).
//   2. Start the MetadataTcpClient command receiver (listens for start/stop/pause
//      commands over UDP — see MetadataClient.h/.cpp).
//   3. Open the video source (openCapture(): local camera, RTSP via OpenCV/ffmpeg, or
//      a TCP feed from Unreal Engine) and start RtspReader, which publishes frames
//      both in-process and into shared memory.
//   4. Launch the external Python SuperPoint/LightGlue matcher process
//      (launch_python_posix(), see src_py/matches_onnx.py) and open the SPSGReader
//      shared-memory link to read its match results (see Matches.hpp).
//   5. Construct the ImageMatcher against the target image.
//   6. Run the main loop: wait for a start command, then each iteration reads a
//      frame + the latest matches, calls ImageMatcher::getAlignment() to estimate
//      relative rotation/translation, converts that into command velocities, sends
//      them via MetadataTcpClient::respositionFunc(), and logs everything to CSV via
//      AlgoLogger. Stops on a stop command, a timeout, or reaching+holding the target.
#include "ImageMatcher.h"
#include "MetadataClient.h"
#include "RtspReader.h"
#include "AlgoLogger.hpp"
#include "Utils.h"
#include "Matches.hpp"
#include "Logger.h"

#include <opencv2/opencv.hpp>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

// Helper constants used for velocity scaling.
namespace {
constexpr float kRotationGain = 0.1f;
constexpr float kTranslationGain = 0.2f;
constexpr float kMaxVelocity = 50.0f;
constexpr float kMaxRotationRate = 5.0f;

// Open either a local camera or remote stream depending on the selected mode.
// mode == "live": opens the fixed V4L2 device below and blocks (up to t seconds)
//   warming it up until WARMUP_GOOD_FRAMES consecutive good frames are read.
// mode == "stream" with a TCP url: does nothing here — RtspReader handles that
//   connection itself once started.
// mode == "stream" otherwise: opens streamUrl via OpenCV's FFmpeg backend, retrying
//   until it succeeds (this call does NOT time out).
bool openCapture(const std::string& mode,
                 const std::string& streamUrl,
                 int t,
                 cv::VideoCapture& cap,
                 Logger& appLogger) {
    if (mode == "live") {
        // NOTE: hardcoded to this specific USB camera's by-id device path. If the
        // camera hardware changes, this needs to change too (or be made a CLI flag).
        std::string device = "/dev/v4l/by-id/usb-UltraSemi_USB3_Video_20210623-video-index0";

        auto reconnect = [&]() -> bool {
            appLogger.log("main", "error: [CAM] Reconnecting to camera...");
            cap.release();

            cap.open(device, cv::CAP_V4L2);
            if (!cap.isOpened()) {
                appLogger.log("main", "error: [CAM] Reconnect failed");
                return false;
            }

            // Re-apply your capture settings
            cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
            cap.set(cv::CAP_PROP_FRAME_WIDTH,  1920);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
            cap.set(cv::CAP_PROP_FPS,          30);
            cap.set(cv::CAP_PROP_BUFFERSIZE,   4);

            // Flush stale frames
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            cv::Mat tmp;
            for (int i = 0; i < 10; ++i) cap.read(tmp);

            appLogger.log("main", "[CAM] Reconnected");
            return true;
        };
        if (!std::filesystem::exists(device)){
            {
                std::ostringstream ss;
                ss << "[Camera] file " << device << " does not exist";
                appLogger.log("main", ss.str());
            }
        }

        cap.release();

        {
            std::ostringstream ss;
            ss << "[Camera] Trying " << device;
            appLogger.log("main", ss.str());
        }

        if(reconnect()) {
            {
                std::ostringstream ss;
                ss << "[Camera] Opened " << device;
                appLogger.log("main", ss.str());
            }
            // Same as time.sleep(1.0)
            appLogger.log("main", "Warming up sensor...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        cv::Mat frame;

        constexpr int WARMUP_GOOD_FRAMES = 10;
        constexpr int MAX_FAILS = 100;

        int goodFrames = 0;
        int failCount = 0;

        const auto startTime = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(t);

        while (goodFrames < WARMUP_GOOD_FRAMES) {
            // Check timeout first
            const auto elapsed = std::chrono::steady_clock::now() - startTime;

            if (elapsed >= timeout) {
                appLogger.log(
                    "main",
                    "[CAM] Warmup timeout reached"
                );
                return false;
            }

            bool success = false;

            try {
                success = cap.read(frame);
            }
            catch (const cv::Exception& e) {
                std::ostringstream ss;
                ss << "[CAM] read exception: " << e.what();
                appLogger.log("main", ss.str());
            }

            // ---------------------------------------------------------
            // Bad frame
            // ---------------------------------------------------------
            if (!success || frame.empty()) {
                ++failCount;

                {
                    std::ostringstream ss;
                    ss << "[CAM] Bad frame ("
                    << failCount << "/" << MAX_FAILS << ")";
                    appLogger.log("main", ss.str());
                }

                // Too many consecutive failures
                if (failCount >= MAX_FAILS) {
                    appLogger.log(
                        "main",
                        "[CAM] Maximum consecutive failures reached"
                    );

                    return false;
                }

                // Try to reconnect
                if (!reconnect()) {
                    appLogger.log(
                        "main",
                        "[CAM] Reconnect failed, retrying..."
                    );

                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(100)
                    );
                }

                continue;
            }

            // ---------------------------------------------------------
            // Good frame
            // ---------------------------------------------------------
            ++goodFrames;
            failCount = 0;

            {
                std::ostringstream ss;
                ss << "[CAM] Warmup frame "
                << goodFrames << "/"
                << WARMUP_GOOD_FRAMES;
                appLogger.log("main", ss.str());
            }
        }

        appLogger.log(
            "main",
            "[CAM] Camera warmup completed successfully"
        );

        return true;
    }

    if (mode == "stream" && !(streamUrl.find("tcp") != std::string::npos)) {
        setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp", 1);
        // Retry loop until the stream is successfully opened
        while (!cap.isOpened()) {
            {
                std::ostringstream ss;
                ss << "Waiting for stream to be available: " << streamUrl;
                appLogger.log("main", ss.str());
            }
            
            // Explicitly call open with CAP_FFMPEG on every retry iteration
            cap.open(streamUrl, cv::CAP_FFMPEG);

            if (!cap.isOpened()) {
                usleep(1000 * 1000); // Wait 1 second before trying again
            }
        }
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
    const std::string onnx_matches = expandUser(getStr(flags, "--onnx_matches", "~/drone_repositioning/src_py/matches_onnx.py"));
    const std::string memoryName = getStr(flags, "--memory", "/sp_sg_matches");
    const int cameraIndex = getInt(flags, "--camera", 0);
    const bool unrealTest = (getInt(flags, "--unreal", 0) != 0);

    const std::string cmdIp = getStr(flags, "--cmd_ip", "0.0.0.0");
    const int cmdPort = getInt(flags, "--cmd_port", 9020);
    const std::string msgIp = getStr(flags, "--relay_ip", "0.0.0.0");
    const int msgPort = getInt(flags, "--relay_port", 9010);

    const std::string logPath = expandUser(getStr(flags, "--log", "~/drone_repositioning/data/"));
    std::string targetImagePath = getStr(flags, "--target", "../target.png");
    const int imgHeight = getInt(flags, "--imgHeight", 1080);
    const int imgWidth = getInt(flags, "--imgWidth", 1920);
    const int timer = getInt(flags, "--timer", 180);
    const int targetImageIndex = getInt(flags, "--targetIndex", -1);

    // Ensure log directory exists before creating the logger.
    const std::filesystem::path logDirectory = std::filesystem::path(logPath);
    if (!logDirectory.empty() && !std::filesystem::exists(logDirectory)) {
        std::filesystem::create_directories(logDirectory);
    }
    const std::filesystem::path logImages = std::filesystem::path(logPath + "images/");
    if (!logImages.empty() && !std::filesystem::exists(logImages)) {
        std::filesystem::create_directories(logImages);
    }
    // Create a CSV logger for runtime data and command history.
    const std::string logData = logPath + "dataLog_" + timeToUnderscoreString() + ".csv";
    AlgoLogger algoLogger(logData, /*write_header=*/true, /*flush_every_n=*/30);

    const std::string logFile = logPath + "log_" + timeToUnderscoreString() + ".txt";
    Logger appLogger(logFile);
    appLogger.log("main", std::string("[mode] ") + mode);
    {
        std::ostringstream ss;
        ss << "[net] cmd=" << cmdIp << ":" << cmdPort
           << " msg=" << msgIp << ":" << msgPort;
        appLogger.log("main", ss.str());
    }
    // Start the metadata command and relay connections.
    MetadataTcpClient client(appLogger);
    if (!client.StartConnectionHandlerUDP(cmdIp, cmdPort, "command_handler")) {
        {
            std::ostringstream ss;
            ss << "error: Failed to start command handler. Ensure port " << cmdPort << " is available.";
            appLogger.log("main", ss.str());
        }
        return -1;
    }
    client.startReceiver();

    // Open the selected video source and set the requested resolution.
    cv::VideoCapture cap;
    if (!openCapture(mode, streamUrl, timer, cap, appLogger)) {
        {
            std::ostringstream ss;
            ss << "error: Cannot open input source for mode '" << mode << "'.";
            appLogger.log("main", ss.str());
        }
        return -1;
    }

    if (!cap.isOpened() && !(streamUrl.find("tcp") != std::string::npos)) {
        appLogger.log("main", "error: Cannot open camera/stream");
        return -1;
    }
    appLogger.log("main", "Camera/Stream opened");

    // Create the RTSP reader and start it, using Unreal mode if configured.
    RtspReader reader(appLogger, logImages, streamUrl, imgWidth, imgHeight, unrealTest);
    {
        std::ostringstream ss;
        ss << "Unreal test: " << unrealTest;
        appLogger.log("main", ss.str());
    }
    if (unrealTest) {
        reader.start();
    } else {
        reader.start(&cap);
    }
    std::string cam = std::to_string(cameraIndex);
    if (mode == "stream" ) {
       cam = streamUrl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
    if (targetImageIndex != -1){
        // Construct the filename using the index: e.g., "camera0.png"    
        targetImagePath =  expandUser("~/drone_repositioning/targets/") + "targetImage" + std::to_string(targetImageIndex) + ".jpg";
    }

    launch_python_posix(onnx_matches, targetImagePath);
    {
        std::ostringstream ss;
        ss << "Launched Python process for matches with target image: " << targetImagePath;
        appLogger.log("main", ss.str());
    }
    // give Python time to init models and create shm
    std::this_thread::sleep_for(std::chrono::seconds(5));

    SPSGReader matchesReader(appLogger);
    {
        std::ostringstream ss;
        ss << "Matches reader initialized with shared memory: " << memoryName;
        appLogger.log("main", ss.str());
    }
        
    cv::Mat cameraMatrix = (cv::Mat_<float>(3,3) << 
                    imgWidth / 2.0f, 0,            imgWidth / 2.0f,
                    0,            imgWidth / 2.0f, imgHeight / 2.0f,
                    0,            0,            1.0f);
    // Load the target image matcher used for alignment estimation.
    ImageMatcher matcher(appLogger, targetImagePath, cameraMatrix);
    {
        std::ostringstream ss;
        ss << "Image matcher initialized with target image: " << targetImagePath;
        appLogger.log("main", ss.str());
    }
    bool hasStarted = false;
    bool complete = false;
    bool atTarget = false;
    double arrivalTime = 0.0;

    // Main repositioning loop: wait for start, process frames, and send commands.
    double start_time = AlgoLogger::nowWallSec();
    while (!complete) {
        double currentTime = AlgoLogger::nowWallSec();
        if (client.stopRepositioning.load()) {
            appLogger.log("main", "Received command to stop repositioning");
            std::stringstream ss;
            // reader.sendStopSignal();
            break;
        }
        if (currentTime - start_time > timer){
            if (hasStarted){
                appLogger.log("main", "Drone Repositioning Timed Out");
                std::stringstream ss;
                // reader.sendStopSignal();
                complete = true;
                break;
            }
            else {start_time = AlgoLogger::nowWallSec();}
        }
        

        if (!client.startRepositioning.load() && !hasStarted) {
            appLogger.log("main", "Waiting to start the repositioning system");
            usleep(1000 * 1000);
            continue;
        }

        while (client.pauseRepositioning.load()){
            appLogger.log("main", "Received command to pause repositioning");
            usleep(1000 * 100);
            if (client.resumeRepositioning.load() || client.stopRepositioning.load()) {
                appLogger.log("main", "Resuming repositioning");
                break;
            }
        }

        hasStarted = true;
        auto [frame, success_frame] = reader.getFrame();
        if (!success_frame) {
            // Skip processing when no new frame is available.
            continue;
        }

        auto result = matchesReader.read();
        if (!result.has_value()) {
                appLogger.log("main", "Python shut down, exiting.");
                break;
            }
        Matches matches = result.value();

        if (!matches.newMatches){
            continue;
        }

        // Compute alignment direction and error metrics from the current frame.
        const auto [rotationMatrix, direction, transError_, success] = matcher.getAlignment(matches, frame);

        // Convert estimated rotation and translation to the appropriate coordinate frame.
        const cv::Point3f rotationVec = rotmatToYPRDeg_XYZ(rotationMatrix);
        bool hasNaNRot = std::isnan(rotationVec.x) || std::isnan(rotationVec.y) || std::isnan(rotationVec.z);
        bool hasNanTrans = std::isnan(direction.x) || std::isnan(direction.y) || std::isnan(direction.z) || std::isnan(transError_);

        if(hasNaNRot) {
            appLogger.log("main", "error: NaN detected in rotation or translation vector, skipping this frame.");
            continue;
        }
        float transError = transError_;
        if (hasNanTrans) transError = 0;
        if (!success) {
            std::stringstream ss;
            ss << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << "0" << "\n";
            std::string data_to_send = ss.str();
            if (!client.SendMetadata(data_to_send)){
                appLogger.log("main", "error: Could not send Stop command");
            }
            appLogger.log("main", "error: GetAlignmentDirection failed.");
            continue;
        }
        
        cv::Point3f directionCv(direction.x * transError, direction.y * transError, direction.z * transError);
        const cv::Point3f rotation = rotationVec;
        const cv::Point3f translation = unrealTest ? ConvertCVToUE(directionCv) : directionCv;

        // Compute command velocities.
        cv::Point3f angleRateCmd = kMaxRotationRate * activation(rotation, kRotationGain);
        cv::Point3f cmdVx = kMaxVelocity * activation(translation, kTranslationGain);

        {
            std::ostringstream ss;
            ss << "rotation: x=" << rotation.x << " y=" << rotation.y << " z=" << rotation.z;
            appLogger.log("main", ss.str());
        }
        {
            std::ostringstream ss;
            ss << "translation: x=" << translation.x << " y=" << translation.y << " z=" << translation.z;
            appLogger.log("main", ss.str());
        }
        {
            std::ostringstream ss;
            ss << "transError: " << transError;
            appLogger.log("main", ss.str());
        }
        
        
        std::string dataToSend;
        if (client.respositionFunc(angleRateCmd, cmdVx, rotation, translation, dataToSend, unrealTest) ) {
            std::stringstream ss;
            ss << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << "0" << "\n";
            std::string data_to_send = ss.str();
            if (!client.SendMetadata(data_to_send)){
                appLogger.log("main", "error: Could not send Stop command");
            }
            appLogger.log("main", "Arrived at target");
            atTarget = true;
            if (arrivalTime == 0.0) arrivalTime = AlgoLogger::nowWallSec();
        }
        else {
            atTarget = false;
            arrivalTime = 0.0;
        }

        currentTime = AlgoLogger::nowWallSec();
        
        if (atTarget && ((currentTime - arrivalTime > 1.0) ) && arrivalTime > 0.0) {
            appLogger.log("main", "Maintained target position for 2 seconds, stopping repositioning");
            std::stringstream ss;
            ss << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << "-1" << "\n";
            std::string data_to_send = ss.str();
            if (!client.SendMetadata(data_to_send)){
                appLogger.log("main", "error: Could not send Stop command");
            }
            complete = true;
        }

        // Log the command and error values for later analysis.
        const auto parsed = AlgoLogger::parseCommand6(dataToSend);
        auto unit_vect = translation / transError;
        algoLogger.log(currentTime, transError, rotation, unit_vect, parsed);

    }
    matchesReader.stop();
    reader.stop();
        
    client.stopReceiver();
    return 0;
}
