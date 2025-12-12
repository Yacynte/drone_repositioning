#pragma once
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstdio>

class RtspReader {
public:
    RtspReader(const std::string& url, int width, int height);
    ~RtspReader();

    void start();
    void stop();
    bool getFrame(cv::Mat& out);

private:
    void readerLoop();

    std::string cmd_;
    int width_, height_;
    int frameSize_;

    std::thread thread_;
    std::atomic<bool> running_{false};

    std::mutex frameMutex_;
    cv::Mat lastFrame_;

    FILE* pipe_{nullptr};
};
