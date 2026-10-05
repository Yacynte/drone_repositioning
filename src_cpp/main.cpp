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
#include <algorithm>
#include <chrono>
#include <deque>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

// Helper constants used for velocity scaling.
namespace {
constexpr float kRotationGain = 0.1f;
constexpr float kTranslationGain = 0.15f;
constexpr float kMaxVelocity = 30.0f;
constexpr float kMaxRotationRate = 5.0f;

// ── Translation gate: when to move, and when translation has arrived ─────────────
// Replaces the old per-axis 3 px deadband in respositionFunc, which (a) counted frames
// where PNEC set t = 0 for lack of signal as "arrived" (runs stopped ~1 m short) and
// (b) judged every frame on its own, so a t whose sign flipped frame to frame made the
// drone dither instead of settle (logData4_9, 2026-10-05).
// Windows are in seconds, not frames: the Release build runs ~16 frames/s, where the
// old 5-frame windows covered only 0.3 s (logData4_11).
constexpr double kArriveWindowSec   = 1.0;   // arrival median of parallaxPx over this span
constexpr int    kArriveMinSamples  = 5;     // ...with at least this many frames in it
constexpr float  kArriveNoiseFactor = 1.2f;  // arrival threshold = factor * noise level...
constexpr float  kArriveMinPx       = 1.0f;  // ...clamped to [min, max] px
constexpr float  kArriveMaxPx       = 3.0f;
constexpr double kDirMaxAgeSec      = 1.0;   // a measured direction stays usable this long
constexpr int    kDirMinSamples     = 3;     // directions needed before moving
constexpr float  kMinConsistency    = 0.6f;  // |mean of unit directions| needed to move
// Resolution limit: no usable direction for kLimitNoDirSec, parallax within a few noise
// levels and no longer decreasing = as close as the measurements can tell. Without this
// the drone would hold until the timeout between "too weak for a direction" and
// "aligned". It was 2.5x noise with a 5-frame window and fired at 2.1-2.9 px while
// parallax was still falling (logData4_11 runs 3, 4).
constexpr float  kLimitNoiseFactor  = 1.5f;
constexpr double kLimitNoDirSec     = 1.0;
constexpr float  kLimitMinDecrease  = 0.05f; // recent median must be >= (1 - this) * previous
// Outlier frames: parallax above this factor * window median is dropped (single-frame
// 10-20 px spikes commanded 20-25 cm/s in logData4_11 run 5). After
// kMaxOutlierStreak consecutive drops the jump is taken as real and accepted.
constexpr float  kOutlierFactor     = 3.0f;
constexpr int    kMaxOutlierStreak  = 3;

struct TranslationGate {
    struct Result {
        cv::Point3f error{0, 0, 0};   // filtered direction * current parallaxPx (0 = no move)
        bool aligned = false;          // translation arrived
        float consistency = 0, medianPx = 0, thresholdPx = 0;
        const char* mode = "hold";   // move | hold | aligned | aligned-limit | outlier
    };

    // unitDir: this frame's t direction in command axes, zero when PNEC found no
    // translation signal; parallaxPx is measured either way; noisePx from the estimator;
    // nowSec: frame time in seconds (any monotonic clock).
    Result update(const cv::Point3f& unitDir, float parallaxPx, float noisePx, double nowSec) {
        // Outlier check against the median of the last arrival window.
        const auto recent = window(nowSec - kArriveWindowSec, nowSec + 1.0);
        if (static_cast<int>(recent.size()) >= kArriveMinSamples
            && parallaxPx > kOutlierFactor * median(recent)
            && outlierStreak_ < kMaxOutlierStreak) {
            ++outlierStreak_;
            Result r = last_;
            r.mode = "outlier";
            return r;
        }
        outlierStreak_ = 0;

        parallax_.push_back({parallaxPx, nowSec});
        while (!parallax_.empty() && nowSec - parallax_.front().t > 2 * kArriveWindowSec)
            parallax_.pop_front();
        if (cv::norm(unitDir) > 1e-6) { dirs_.push_back({unitDir, nowSec}); lastDirSec_ = nowSec; }
        if (lastDirSec_ < 0) lastDirSec_ = nowSec;   // start counting from the first frame
        while (!dirs_.empty() && nowSec - dirs_.front().t > kDirMaxAgeSec) dirs_.pop_front();

        Result r;
        const float noise = std::isfinite(noisePx) ? kArriveNoiseFactor * noisePx : kArriveMaxPx;
        r.thresholdPx = std::clamp(noise, kArriveMinPx, kArriveMaxPx);
        const auto cur = window(nowSec - kArriveWindowSec, nowSec + 1.0);
        r.medianPx = median(cur);
        // The window must actually span the arrival time, not just hold enough frames.
        const bool fullWindow = static_cast<int>(cur.size()) >= kArriveMinSamples
            && nowSec - parallax_.front().t >= 0.8 * kArriveWindowSec;

        // Arrived: parallax itself (measured with or without a direction) has stayed
        // below the noise-based threshold for the whole window.
        if (fullWindow && r.medianPx < r.thresholdPx) {
            r.aligned = true;
            r.mode = "aligned";
            return last_ = r;
        }
        // Resolution limit, only once parallax has stopped falling.
        const float limitPx = std::isfinite(noisePx)
            ? std::min(kLimitNoiseFactor * noisePx, kArriveMaxPx) : r.thresholdPx;
        const auto prev = window(nowSec - 2 * kArriveWindowSec, nowSec - kArriveWindowSec);
        const bool stalled = static_cast<int>(prev.size()) >= kArriveMinSamples
            && r.medianPx >= (1.0f - kLimitMinDecrease) * median(prev);
        if (fullWindow && stalled && nowSec - lastDirSec_ >= kLimitNoDirSec && r.medianPx < limitPx) {
            r.aligned = true;
            r.mode = "aligned-limit";
            return last_ = r;
        }
        // Move only along a direction the recent frames agree on; random sign flips
        // average out (low consistency) and the drone holds instead of dithering.
        cv::Point3f sum(0, 0, 0);
        for (const auto& d : dirs_) sum += d.dir;
        if (!dirs_.empty()) r.consistency = static_cast<float>(cv::norm(sum)) / dirs_.size();
        if (static_cast<int>(dirs_.size()) >= kDirMinSamples && r.consistency >= kMinConsistency) {
            r.error = sum * (parallaxPx / static_cast<float>(cv::norm(sum)));
            r.mode = "move";
        }
        return last_ = r;
    }

private:
    struct DirSample { cv::Point3f dir; double t; };
    struct PxSample { float px; double t; };
    std::deque<DirSample> dirs_;
    std::deque<PxSample> parallax_;
    Result last_;
    double lastDirSec_ = -1;
    int outlierStreak_ = 0;

    // Parallax samples with time in [from, to).
    std::vector<float> window(double from, double to) const {
        std::vector<float> v;
        for (const auto& s : parallax_) if (s.t >= from && s.t < to) v.push_back(s.px);
        return v;
    }
    static float median(std::vector<float> v) {
        if (v.empty()) return 0.0f;
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        return v[v.size() / 2];
    }
};

// ── Gate for the image-based translation error (--trans-source flow | ecc) ───────
// The error vector from ImageMatcher's flow fit (option 1) or ECC (option 2) is
// signed and goes to zero at the target. Like rotation, each axis settles on its own:
// a settled axis gets a zero command, so leftover noise on it does not keep the drone
// moving or skew the others. Arrival needs every axis settled AND the net error
// (vector length) below settlePx.
constexpr double kFitSmoothSec   = 0.3;    // command = mean error over this span
constexpr double kFitSettleSec   = 1.0;    // settle test span...
constexpr int    kFitMinSamples  = 5;      // ...with at least this many frames
constexpr float  kResumeFactor   = 2.0f;   // a settled axis resumes above this * axis threshold
// Option 3, overshoot detector per axis: that axis's error changing sign between
// significant samples at least kOscMinFlips times within kOscWindowSec, while its
// median stays within kOscMaxFactor * axis threshold, settles the axis.
constexpr double kOscWindowSec   = 2.0;
constexpr int    kOscMinFlips    = 2;
constexpr float  kOscMaxFactor   = 2.0f;

struct FitGate {
    struct Result {
        cv::Point3f error{0, 0, 0};     // per-axis command error, px (0 on settled axes)
        bool aligned = false;
        cv::Point3f medianAbs{0, 0, 0}; // per-axis median |error| over the settle span, px
        float medianNorm = 0;           // median |error vector| over the settle span, px
        bool settled[3] = {false, false, false};
        int flips[3] = {0, 0, 0};
        const char* mode = "move";      // move | settled | invalid
    };

    // settlePx: net (vector length) arrival threshold. axisPx: per-axis settle
    // threshold; <= settlePx / sqrt(3) guarantees all-axes-settled implies the net test.
    FitGate(float settlePx, float axisPx, bool useOscillation)
        : settlePx_(settlePx), axisPx_(axisPx), useOsc_(useOscillation) {}

    Result update(bool valid, const cv::Point3f& error, double nowSec) {
        Result r;
        for (int i = 0; i < 3; ++i) r.settled[i] = settled_[i];
        if (!valid) { r.mode = "invalid"; return r; }
        samples_.push_back({error, nowSec});
        while (!samples_.empty() && nowSec - samples_.front().t > kOscWindowSec) samples_.pop_front();

        std::vector<float> abs[3], norms;
        cv::Point3f smooth(0, 0, 0);
        int nSmooth = 0;
        for (const auto& s : samples_) {
            if (nowSec - s.t <= kFitSettleSec) {
                for (int i = 0; i < 3; ++i) abs[i].push_back(std::abs(axis(s.e, i)));
                norms.push_back(static_cast<float>(cv::norm(s.e)));
            }
            if (nowSec - s.t <= kFitSmoothSec) { smooth += s.e; ++nSmooth; }
        }
        smooth *= 1.0f / std::max(nSmooth, 1);
        const bool fullWindow = static_cast<int>(norms.size()) >= kFitMinSamples
            && nowSec - samples_.front().t >= 0.8 * kFitSettleSec;
        r.medianAbs = {median(abs[0]), median(abs[1]), median(abs[2])};
        r.medianNorm = median(norms);

        for (int i = 0; i < 3; ++i) {
            const float med = axis(r.medianAbs, i);
            // Sign changes of this axis between samples where it is significant.
            int prevSign = 0;
            for (const auto& s : samples_) {
                const float v = axis(s.e, i);
                if (std::abs(v) < axisPx_) continue;
                const int sign = v > 0 ? 1 : -1;
                if (prevSign != 0 && sign != prevSign) ++r.flips[i];
                prevSign = sign;
            }
            if (!settled_[i]) {
                const bool still = med < axisPx_;
                const bool osc = useOsc_ && r.flips[i] >= kOscMinFlips && med < kOscMaxFactor * axisPx_;
                if (fullWindow && (still || osc)) settled_[i] = true;
            } else if (std::abs(axis(smooth, i)) > kResumeFactor * axisPx_) {
                settled_[i] = false;   // drifted away again
            }
        }
        const bool allSettled = settled_[0] && settled_[1] && settled_[2];
        if (allSettled && r.medianNorm < settlePx_) {
            r.aligned = true;
            r.mode = "settled";
        } else if (allSettled) {
            // Every axis is within its band but the net error is not: reopen the
            // largest axis so the drone does not sit with zero commands forever.
            int worst = 0;
            for (int i = 1; i < 3; ++i) if (axis(r.medianAbs, i) > axis(r.medianAbs, worst)) worst = i;
            settled_[worst] = false;
        }
        for (int i = 0; i < 3; ++i) {
            r.settled[i] = settled_[i];
            if (!r.aligned && !settled_[i]) axis(r.error, i) = axis(smooth, i);
        }
        return r;
    }

private:
    struct Sample { cv::Point3f e; double t; };
    std::deque<Sample> samples_;
    bool settled_[3] = {false, false, false};
    float settlePx_, axisPx_;
    bool useOsc_;
    static float& axis(cv::Point3f& v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }
    static float axis(const cv::Point3f& v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }
    static float median(std::vector<float> v) {
        if (v.empty()) return 0.0f;
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        return v[v.size() / 2];
    }
};

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
    // Translation error source driving the gate: "parallax" (PNEC t + median parallax,
    // TranslationGate), "flow" (option 1: signed fit of the de-rotated match
    // displacements) or "ecc" (option 2: dense ECC). --osc-arrival 1 also accepts the
    // overshoot detector (option 3) as arrival; --ecc-log 1 computes ECC for the log
    // even when it is not the source.
    const std::string transSource = getStr(flags, "--trans-source", "flow");
    const bool oscArrival = getInt(flags, "--osc-arrival", 1) != 0;
    const bool eccLog = getInt(flags, "--ecc-log", 0) != 0;
    // Flow/ecc gate thresholds, px: net error (vector length) for arrival, and the
    // per-axis settle threshold (default settle-px / sqrt(3), so all axes settled
    // implies the net test).
    const float settlePx = static_cast<float>(std::stod(getStr(flags, "--settle-px", "1.5")));
    const float axisSettlePx = static_cast<float>(std::stod(getStr(flags, "--axis-settle-px",
                                   std::to_string(settlePx / std::sqrt(3.0f)))));
    if (transSource != "parallax" && transSource != "flow" && transSource != "ecc") {
        std::cerr << "--trans-source must be parallax, flow or ecc" << std::endl;
        return 1;
    }

    const std::string cmdIp = getStr(flags, "--cmd_ip", "0.0.0.0");
    const int cmdPort = getInt(flags, "--cmd_port", 9020);
    const std::string msgIp = getStr(flags, "--relay_ip", "0.0.0.0");
    const int msgPort = getInt(flags, "--relay_port", 9010);

    const std::string logPath = expandUser(getStr(flags, "--log", "~/drone_repositioning/data/"));
    std::string targetImagePath = getStr(flags, "--target", "../target.png");
    const int imgHeight = getInt(flags, "--imgHeight", 1080);
    const int imgWidth = getInt(flags, "--imgWidth", 1920);
    const int timer = getInt(flags, "--timer", 300);
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

    PythonProcess python = launch_python_posix(onnx_matches, targetImagePath, appLogger);
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
    matcher.setEccEnabled(transSource == "ecc" || eccLog);
    appLogger.log("main", "Translation source: " + transSource + " settle-px=" + std::to_string(settlePx) + " axis-settle-px=" + std::to_string(axisSettlePx) + " osc-arrival=" + std::to_string(oscArrival)
                  + " ecc=" + std::to_string(transSource == "ecc" || eccLog));
    {
        std::ostringstream ss;
        ss << "Image matcher initialized with target image: " << targetImagePath;
        appLogger.log("main", ss.str());
    }
    bool hasStarted = false;
    bool complete = false;
    bool atTarget = false;
    TranslationGate translationGate;
    FitGate fitGate(settlePx, axisSettlePx, oscArrival);
    double arrivalTime = 0.0;

    // Main repositioning loop: wait for start, process frames, and send commands.
    double start_time = AlgoLogger::nowWallSec();
    while (!complete) {
        double currentTime = AlgoLogger::nowWallSec();
        if (client.stopRepositioning.load()) {
            appLogger.log("main", "Received command to stop repositioning");
            std::stringstream ss;
            // std::stringstream ss;
            ss << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << "-1" << "\n";
            std::string data_to_send = ss.str();
            if (!client.SendMetadata(data_to_send)){
                appLogger.log("main", "error: Could not send Stop command");
            }
            break;
        }
        if (currentTime - start_time > timer){
            if (hasStarted){
                appLogger.log("main", "Drone Repositioning Timed Out");
                std::stringstream ss;
                // reader.sendStopSignal();
                // std::stringstream ss;
                ss << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << "-1" << "\n";
                std::string data_to_send = ss.str();
                if (!client.SendMetadata(data_to_send)){
                    appLogger.log("main", "error: Could not send Stop command");
                }
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
        const auto tAlign0 = std::chrono::steady_clock::now();
        const auto [rotationMatrix, direction, transError_, success] = matcher.getAlignment(matches, frame);
        const double alignMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - tAlign0).count();

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
        
        const cv::Point3f rotation = rotationVec;
        const cv::Point3f unitDir = hasNanTrans ? cv::Point3f(0, 0, 0)
                                  : (unrealTest ? ConvertCVToUE(direction) : direction);
        const double nowSec = std::chrono::duration<double>(tAlign0.time_since_epoch()).count();
        // The parallax gate always runs so its decision stays in the log for comparison.
        const auto gate = translationGate.update(unitDir, transError, matcher.lastNoisePx(), nowSec);
        {
            std::ostringstream ss;
            ss << "translationGate: mode=" << gate.mode << " medianPx=" << gate.medianPx
               << " thresholdPx=" << gate.thresholdPx << " consistency=" << gate.consistency;
            appLogger.log("main", ss.str());
        }
        cv::Point3f translation = gate.error;
        bool translationAligned = gate.aligned;
        if (transSource != "parallax") {
            const TranslationFit& fit = transSource == "ecc" ? matcher.lastEccFit() : matcher.lastFlowFit();
            const cv::Point3f err = unrealTest ? ConvertCVToUE(fit.errorPx()) : fit.errorPx();
            const auto fg = fitGate.update(fit.valid, err, nowSec);
            translation = fg.error;
            translationAligned = fg.aligned;
            std::ostringstream ss;
            ss << "fitGate(" << transSource << "): mode=" << fg.mode << " settledAxes="
               << (fg.settled[0] ? 'x' : '-') << (fg.settled[1] ? 'y' : '-') << (fg.settled[2] ? 'z' : '-')
               << " medianAbsPx=" << fg.medianAbs.x << "," << fg.medianAbs.y << "," << fg.medianAbs.z
               << " medianNormPx=" << fg.medianNorm << " flips=" << fg.flips[0] << "," << fg.flips[1]
               << "," << fg.flips[2] << " errPx=" << fg.error.x << "," << fg.error.y << "," << fg.error.z;
            appLogger.log("main", ss.str());
        }

        // Compute command velocities. Parallax source: translation saturates on its
        // magnitude so the filtered direction is kept. Flow/ecc: per axis (below).
        cv::Point3f angleRateCmd = kMaxRotationRate * activation(rotation, kRotationGain);
        cv::Point3f cmdVx(0, 0, 0);
        const float transNorm = static_cast<float>(cv::norm(translation));
        if (transSource != "parallax") {
            // Per-axis speed, like rotation: each axis follows its own error, and a
            // settled axis (zero error) gets zero speed.
            cmdVx = kMaxVelocity * activation(translation, kTranslationGain);
        } else if (transNorm > 1e-6f) {
            const float speed = kMaxVelocity * activation(cv::Point3f(transNorm, 0, 0), kTranslationGain).x;
            cmdVx = translation * (speed / transNorm);
        }

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
        // float rotErrDeg = Eigen::AngleAxisd(rotationMatrix).angle() * 180.0 / M_PI;   // R: Eigen::Matrix3d
        // const cv::Point3f rotationVec()
        std::string dataToSend;
        if (client.respositionFunc(angleRateCmd, cmdVx, rotation, translationAligned, dataToSend, unrealTest) ) {
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
            // Save the end image before the -1 command: the controller may kill us once it gets it.
            reader.saveEndImage();
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
        AlgoLogger::FrameTiming ft;
        ft.matchId = matches.matchId;
        ft.camFrameId = matches.camFrameId;
        ft.skipped = matches.skipped;
        ft.pyFrameTs = matches.pyFrameTs;
        ft.pyDoneTs = matches.pyDoneTs;
        ft.recvTs = matches.recvTs;
        ft.alignMs = alignMs;
        algoLogger.log(currentTime, transError, rotation, unit_vect, parsed, ft);
        {
            std::ostringstream ss;
            ss.setf(std::ios::fixed);
            ss.precision(1);
            ss << "timing match_id=" << matches.matchId << " cam_frame=" << matches.camFrameId
               << " skipped=" << matches.skipped
               << " py_ms=" << (matches.pyDoneTs - matches.pyFrameTs) * 1000.0
               << " handoff_ms=" << (matches.recvTs - matches.pyDoneTs) * 1000.0
               << " align_ms=" << alignMs
               << " to_send_ms=" << (currentTime - matches.recvTs) * 1000.0
               << " age_ms=" << (currentTime - matches.pyFrameTs) * 1000.0;
            appLogger.log("main", ss.str());
        }

    }
    matchesReader.stop();
    reader.stop();          // sets is_alive = 0 on the frame segment: Python exits
    python.stop();          // wait for it, log its exit status, join the output thread

    client.stopReceiver();
    return 0;
}
