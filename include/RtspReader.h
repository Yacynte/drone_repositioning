#pragma once
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstdio>

class RtspReader {
public:
    RtspReader(const std::string& url, int width, int height, bool unreal_test = 0 );
    ~RtspReader();

    void start(cv::VideoCapture* externalCap = nullptr);
    void stop();
    bool getFrame(cv::Mat& out);

private:
    void readerLoop();
    bool isImageDark(const cv::Mat& image, double threshold = 30.0);
    std::string cmd_;
    int width_, height_;
    int frameSize_;
    bool unreal_test_;
    cv::VideoCapture* cap = nullptr;

    std::thread thread_;
    std::atomic<bool> running_{false};

    std::mutex frameMutex_;
    cv::Mat lastFrame_;

    FILE* pipe_{nullptr};
};
