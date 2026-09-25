#pragma once
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstdio>
#include "Logger.h"
#include "Utils.h"

// --- POSIX Sockets Headers ---
#include <sys/socket.h> // For socket(), connect(), send(), etc.
#include <netinet/in.h> // For sockaddr_in structure
#include <arpa/inet.h>  // For inet_pton()
#include <unistd.h>     // For close()
#include <cerrno>       // For errno
#include <unistd.h>     // close()
#include <sys/socket.h> // recv()

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <array>
#include <vector>
#include <stdexcept>

// Define invalid socket and error check based on POSIX conventions
#define INVALID_SOCKET -1
#define SOCKET_ERROR   -1

// Reads video frames on a background thread (from a local camera, an RTSP stream via
// ffmpeg, or a TCP-pushed Unreal Engine frame feed) and republishes the latest
// grayscale frame two ways: (1) in-process via getFrame(), used by main.cpp's loop,
// and (2) into a POSIX shared-memory segment that the external Python
// SuperPoint/LightGlue process (src_py/matches_onnx.py) reads frames from.
class RtspReader {
public:
    // logImages: directory frames get optionally saved to. url: RTSP/TCP source
    // (ignored when start() is given a local cv::VideoCapture). unreal_test selects
    // the TCP Unreal Engine frame feed instead of RTSP/local camera.
    explicit RtspReader(Logger& logger, const std::string& logImages, const std::string& url,
                int width, int height, bool unreal_test = 0,
                const char* shm_name_frame = "/single_frame_shm" );
    ~RtspReader();

    // Starts the background reader thread. Pass externalCap to read from an
    // already-opened cv::VideoCapture (local camera/RTSP); omit it to use the
    // Unreal Engine TCP frame feed instead (see unreal_test_).
    void start(cv::VideoCapture* externalCap = nullptr);
    // Signals the reader thread to stop and joins it.
    void stop();
    // Writes the latest colour frame to imageEnd<start_time>.png (once).
    void saveEndImage();
    // Returns the most recently published frame and whether a new frame was actually
    // available since the last call.
    std::tuple<cv::Mat, bool> getFrame();

    static constexpr uint32_t WIDTH = 1920;
    static constexpr uint32_t HEIGHT = 1080;
    static constexpr uint32_t FRAME_SIZE = WIDTH * HEIGHT;  // 2,073,600 bytes

    // Fixed-size header at the start of the shared-memory segment; see the layout
    // comment above write_frame() in RstpReader.cpp. image_buffer_ (FRAME_SIZE bytes
    // of grayscale pixel data) immediately follows this header in the mapped region.
    struct Header {
        bool is_alive{0};
        bool is_writing{0};
        bool is_reading{0};
        uint32_t frame_id{0};
        uint32_t width{WIDTH};
        uint32_t height{HEIGHT};
    };

    static constexpr size_t TOTAL_SHM_SIZE = sizeof(Header) + FRAME_SIZE;

private:
    Logger& logger;
    std::string start_time;
    std::string logImages_;
    bool frameMemoryReady_ = false;
    std::string shm_name_frame_;
    bool start_rec = true;
    bool endImageSaved_ = false;
    cv::Mat frame;
    int fd_{-1};
    uint8_t* shm_ptr_{nullptr};
    Header* header_{nullptr};
    uint8_t* image_buffer_{nullptr};
    // Creates (or opens) and mmaps the /dev/shm frame segment described by Header.
    void create_and_map_shm();
    // Copies frame into the shared-memory image buffer and bumps frame_id, guarded by
    // the is_reading/is_writing handshake flags so the Python reader never sees a
    // torn frame.
    void write_frame(const cv::Mat& frame);
    // Marks the segment dead (is_alive = 0) and unmaps/unlinks it.
    void cleanupSharedMemory();

    // Background-thread entry point when reading from a local cv::VideoCapture/RTSP
    // stream (cap != nullptr): grabs frames, converts to grayscale, and publishes them.
    void readerLoop();
    // Background-thread entry point when unreal_test_ is set but a capture device is
    // used directly (V4L2 reconnect-aware loop); publishes frames the same way as readerLoop().
    void DroneReaderLoop();
    // True if the mean pixel brightness of image is below threshold.
    bool isImageDark(const cv::Mat& image, double threshold = 30.0);
    // Opens a TCP connection to ip:port for the Unreal Engine frame feed.
    bool Connect(const std::string& ip, int port);
    // Reads exactly len bytes from sock into buf, looping over partial recv()s.
    bool recvAll(int sock, uint8_t* buf, size_t len);
    bool receiveImage(int sock, cv::Mat& outImage);
    void CloseSocket();
    // Background-thread entry point when unreal_test_ is set: pulls raw GBRA frames
    // pushed over TCP by Unreal Engine, converts to BGR, and publishes them.
    void TcpReaderLoop();
    int client_socket = INVALID_SOCKET;
    std::string cmd_;
    int width_, height_;
    size_t frameSize_;
    bool unreal_test_;
    cv::VideoCapture* cap = nullptr;
    std::string ip;
    int port;
    std::thread thread_;
    std::atomic<bool> running_{false};

    std::mutex frameMutex_;
    std::mutex frameColourMutex_;
    cv::Mat lastFrame_;
    cv::Mat lastColourFrame_;

    FILE* pipe_{nullptr};
};
