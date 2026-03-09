#include "RtspReader.h"

RtspReader::RtspReader(const std::string& url, int width, int height, bool unreal_test)
    : width_(width), height_(height), unreal_test_(unreal_test) 
{
    if (unreal_test_){

        frameSize_ = width_ * height_ * 3;

        cmd_ =
            "ffmpeg -rtsp_transport tcp -i \"" + url + "\" "
            "-f rawvideo -pix_fmt bgr24 -";
    }
    // if(externalCap != nullptr) cap = externalCap;

}

RtspReader::~RtspReader() {
    stop();
}

void RtspReader::start(cv::VideoCapture* externalCap) {
    if(!unreal_test_){
        if(externalCap != nullptr) cap = externalCap;
        else{
            std::cerr << "Error video capture is null" << std::endl;
        }
    }
    else{
        running_ = true;
        thread_ = std::thread(&RtspReader::readerLoop, this);
    }
    
}

void RtspReader::stop() {
    if (unreal_test_){
            running_ = false;
            if (pipe_) {
        #ifdef _WIN32
                _pclose(pipe_);
        #else
                pclose(pipe_);
        #endif
                pipe_ = nullptr;
            }
            if (thread_.joinable()) thread_.join();
    }
    else cap = nullptr;
}

bool RtspReader::getFrame(cv::Mat& out) {
    if (!unreal_test_) {
        cap->read(out);
        return true;
    }
    std::lock_guard<std::mutex> lock(frameMutex_);
    if (lastFrame_.empty()) return false;
    out = lastFrame_.clone();
    return true;
}

void RtspReader::readerLoop() {
#ifdef _WIN32
    pipe_ = _popen(cmd_.c_str(), "rb");
#else
    pipe_ = popen(cmd_.c_str(), "r");
#endif

    if (!pipe_) {
        printf("ERROR: Cannot start FFmpeg process.\n");
        return;
    }

    std::vector<unsigned char> buffer(frameSize_);

    while (running_) {
        size_t bytes = fread(buffer.data(), 1, frameSize_, pipe_);
        if (bytes < frameSize_) {
            // printf("WARN: Incomplete frame or stream ended.\n");
            continue;
        }

        cv::Mat frame(height_, width_, CV_8UC3, buffer.data());

        {
            std::lock_guard<std::mutex> lock(frameMutex_);
            frame.copyTo(lastFrame_);
        }
    }
}
