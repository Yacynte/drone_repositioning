#pragma once
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstdio>

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

class RtspReader {
public:
    RtspReader(const std::string& url, int width, int height, bool unreal_test = 0, 
                const char* shm_name_frame = "/single_frame_shm" );
    ~RtspReader();

    void start(cv::VideoCapture* externalCap = nullptr);
    void stop();
    // bool getFrame(cv::Mat& out);
    std::tuple<cv::Mat, bool> getFrame();

    static constexpr uint32_t WIDTH = 1920;
    static constexpr uint32_t HEIGHT = 1080;
    static constexpr uint32_t FRAME_SIZE = WIDTH * HEIGHT;  // 2,073,600 bytes

    struct Header {
        bool is_alive{0};        // plain bool
        bool is_writing{0};
        bool is_reading{0};
        uint32_t frame_id{0};
        uint32_t width{WIDTH};   // uncommented
        uint32_t height{HEIGHT};
    };

    static constexpr size_t TOTAL_SHM_SIZE = sizeof(Header) + FRAME_SIZE;

private:
    bool frameMemoryReady_ = false;
    std::string shm_name_frame_;
    int fd_{-1};
    uint8_t* shm_ptr_{nullptr};
    Header* header_{nullptr};
    uint8_t* image_buffer_{nullptr};
    void create_and_map_shm();
    void write_frame(const cv::Mat& frame);
    void cleanupSharedMemory();

    void readerLoop();
    void DroneReaderLoop();
    bool isImageDark(const cv::Mat& image, double threshold = 30.0);
    bool Connect(const std::string& ip, int port);
    bool recvAll(int sock, uint8_t* buf, size_t len);
    bool receiveImage(int sock, cv::Mat& outImage);
    void CloseSocket();
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
    cv::Mat lastFrame_;
    // cv::Mat DroneLastFrame_;

    FILE* pipe_{nullptr};
};
