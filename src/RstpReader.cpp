#include "RtspReader.h"

RtspReader::RtspReader(const std::string& url, int width, int height, bool unreal_test, const char* shm_name_frame)
    : width_(width), height_(height), unreal_test_(unreal_test), shm_name_frame_(shm_name_frame) 
{
    
    if (unreal_test_){
        if(url.find("tcp") != std::string::npos){
            frameSize_ = width_ * height_ * 4; // Assuming 4 bytes per pixel for RGBA
            // Replace :// and : with spaces so the stream can extract them as words
            std::string modifiedUrl = url;
            for (char &c : modifiedUrl) {
                if (c == ':' || c == '/') c = ' ';
            }

            std::stringstream ss(modifiedUrl);
            std::string protocol, port_;

            // Stream skips the whitespace
            ss >> protocol >> ip >> port_;
            port = std::stoi(port_);
            std::cout << " [RtspReader] Connecting to " << ip << ":" << port << std::endl;
            if (!RtspReader::Connect(ip, port)) {
                std::cerr << " [RtspReader] Failed to connect to " << ip << ":" << port << std::endl;
            }
        }
        else{
            frameSize_ = width_ * height_ * 3;
            cmd_ =
                "ffmpeg -rtsp_transport tcp -i \"" + url + "\" "
                "-f rawvideo -pix_fmt bgr24 -";
        }

    }
    RtspReader::create_and_map_shm();
    // if(externalCap != nullptr) cap = externalCap;

}

RtspReader::~RtspReader() {
    stop();
    if (header_) {
      header_->is_alive = 0;  // Mark as not alive
    }
    if (shm_ptr_ != nullptr && shm_ptr_ != MAP_FAILED) {
      munmap(shm_ptr_, TOTAL_SHM_SIZE);
    }
    if (fd_ >= 0) {
      close(fd_);
      // Uncomment to remove from OS on program exit
      shm_unlink(shm_name_frame_.c_str());
    }
}


bool RtspReader::Connect(const std::string& ip, int port)
{
    CloseSocket(); // Close any existing socket

    // 1. Create a socket (AF_INET = IPv4, SOCK_STREAM = TCP)
    client_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket == INVALID_SOCKET) {
        std::cerr << " [RtspReader] Error at socket(): " << std::strerror(errno) << std::endl;
        return false;
    }

    // 2. Resolve the server address
    sockaddr_in server_addr{};
    // std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    
    // Convert IP string to network address structure
    if (inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << " [RtspReader] Invalid address/Address not supported" << std::endl;
        CloseSocket();
        return false;
    }

    // 3. Connect to the server
    int result = connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (result == SOCKET_ERROR) {
        std::cerr << " [RtspReader] Connection failed with error: " << std::strerror(errno) << std::endl;
        CloseSocket();
        return false;
    }

    std::cout << " [RtspReader] Successfully connected to server at " << ip << ":" << port << std::endl;
    return true;
}


void RtspReader::CloseSocket()
{
    if (client_socket != INVALID_SOCKET) {
        close(client_socket); // POSIX function to close a file descriptor
        client_socket = INVALID_SOCKET;
    }
    
}


bool RtspReader::recvAll(int sock, uint8_t* buf, size_t len) {
    size_t received = 0;
    while (received < len) {
        ssize_t n = recv(sock, buf + received, len - received, 0);
        if (n <= 0) return false;
        received += static_cast<size_t>(n);
    }
    return true;
}

bool RtspReader::receiveImage(int sock, cv::Mat& outImage) {
    uint8_t lenBuf[4];
    if (!recvAll(sock, lenBuf, 4)) return false;

    uint32_t imgSize = (lenBuf[0] << 24) | (lenBuf[1] << 16) | (lenBuf[2] << 8) | lenBuf[3];
    if (imgSize == 0 || imgSize > 100 * 1024 * 1024) return false;

    std::vector<uint8_t> buffer(imgSize);
    if (!recvAll(sock, buffer.data(), imgSize)) return false;

    outImage = cv::imdecode(buffer, cv::IMREAD_COLOR);
    return !outImage.empty();
}


void RtspReader::start(cv::VideoCapture* externalCap) {
    
    if(!unreal_test_){
        if(externalCap != nullptr) {
            std::cout << " [RtspReader] Loop for Live camera " << std::endl;    
            cap = externalCap;
            running_ = true;
            thread_ = std::thread(&RtspReader::DroneReaderLoop, this);
            std::cout << " [RtspReader] Started live image loop\n";
        }
        else{
            std::cerr << " [RtspReader] Error video capture is null" << std::endl;
        }
    }
    else{
        if(!ip.empty() && port > 0) {
            running_ = true;
            thread_ = std::thread(&RtspReader::TcpReaderLoop, this);
            std::cout << " [RtspReader] Starting TCP reader loop for IP: " << ip << ", Port: " << port << std::endl;
        }
        else{
            running_ = true;
            thread_ = std::thread(&RtspReader::readerLoop, this);
        }
    }
    
}

void RtspReader::stop() {
    running_ = false;
    if (unreal_test_){
            
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
    else {
        // running_ = false;

        if (thread_.joinable())
            thread_.join();

        if (cap != nullptr && cap->isOpened())
            cap->release();
        }
    cleanupSharedMemory();
    
}

// bool RtspReader::getFrame(cv::Mat& out)
std::tuple<cv::Mat, bool> RtspReader::getFrame()
{
    cv::Mat out;
    {
        std::lock_guard<std::mutex> lock(frameMutex_);

        if (lastFrame_.empty())
            return {cv::Mat(), false};

        out = std::move(lastFrame_);  // ← pointer swap, ~nanoseconds
                                      //   lastFrame_ is empty() == true after this
    }

    return {out, true};
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

        // std::cout << " width_: "<< width_ << " height_: " << height_ << std::endl;
        cv::Mat frame(height_, width_, CV_8UC3, buffer.data());
        cv::Mat grayFrame;
        cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
        {
            std::lock_guard<std::mutex> lock(frameMutex_);
            grayFrame.copyTo(lastFrame_);
        }
        
        write_frame(grayFrame);
    }
}

bool RtspReader::isImageDark(const cv::Mat& image, double threshold)
{
    cv::Mat gray;
    if (image.channels() == 1)
        gray = image;
    else
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::Scalar meanVal = cv::mean(gray);
    // std::cout << "image mean brightness: " << meanVal[0] << std::endl;

    return meanVal[0] < threshold;
}

void RtspReader::DroneReaderLoop(){
    std::cout << " [RtspReader] Started DroneReaderLoop\n";
    int droppedFrames = 0;
    cv::Mat frame_;
    bool suc;
    std::cout << "[RTSP Reader] test frame 0" << std::endl;
    while (droppedFrames < 20 && cap->grab() ) {
        // std::cout << "[RTSP Reader] read frame: " << droppedFrames << std::endl;
        suc = cap->read(frame_);
        if (!suc || frame_.empty()) {
            std::cerr << "[RTSP Reader] Failed to read frame"
                    << " suc=" << suc
                    << " empty=" << frame_.empty()
                    << std::endl;
        }
        // If your loop was blocked for a while, this rapidly skips 
        // through old buffered frames to catch up to live.
        // Break early if we think we are close to live (optional heuristic)
        droppedFrames++;
    }
    std::cout << "[RTSP Reader] test frame" << std::endl;
    if (suc) cv::imwrite("testcpp.png", frame_);
    else (std::cerr << "[RTSP Reader] cannot read test frame\n");

    // std::cout << "[RtspReader] Started DroneReaderLoop\n";

    auto reconnect = [&]() -> bool {
        std::cerr << "[CAM] Reconnecting to camera...\n";
        cap->release();

        // Wait for the device node to come back
        // for (int i = 0; i < 10; ++i) {
        //     std::this_thread::sleep_for(std::chrono::seconds(1));
        //     if (std::filesystem::exists(devicePath_)) break;
        //     std::cerr << "[CAM] Waiting for device... (" << i+1 << "/10)\n";
        // }

        cap->open("/dev/v4l/by-id/usb-UltraSemi_USB3_Video_20210623-video-index0", cv::CAP_V4L2);
        if (!cap->isOpened()) {
            std::cerr << "[CAM] Reconnect failed\n";
            return false;
        }

        // Re-apply your capture settings
        cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        cap->set(cv::CAP_PROP_FRAME_WIDTH,  1920);
        cap->set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        cap->set(cv::CAP_PROP_FPS,          30);
        cap->set(cv::CAP_PROP_BUFFERSIZE,   4);

        // Flush stale frames
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        cv::Mat tmp;
        for (int i = 0; i < 10; ++i) cap->read(tmp);

        std::cerr << "[CAM] Reconnected\n";
        return true;
    };

    int failCount = 0;
    const int MAX_FAILS = 100;
    const int MAX_TRIALS = 10;
    int trialCount = 0;

    while (running_) {
        cv::Mat frame;

        // std::cerr << "[CAM] before grab\n";

        // for (int i = 0; i < 5; ++i) {
        //     bool ok = cap->grab();
        //     // std::cerr << "[CAM] grab " << i << ": " << ok << "\n";
        // }

        // std::cerr << "[CAM] before read\n";

        bool success = false;
        try {
            success = cap->read(frame);
        } catch (const cv::Exception& e) {
            std::cerr << "[CAM] read exception: " << e.what() << "\n";
            continue;
        }

        // std::cerr << "[CAM] after read: "
        //         << success
        //         << " empty=" << frame.empty()
        //         << " size=" << frame.cols << "x" << frame.rows
        //         << "\n";

         if (!success || frame.empty()) {
            failCount++;
            // std::cerr << "[CAM] No frame (" << failCount << "/" << MAX_FAILS << ")\n";

            if (failCount >= MAX_FAILS) {
                failCount = 0;
                if (!reconnect())
                    std::cerr << "[CAM] No frame (" << trialCount << "/" << MAX_TRIALS << ")\n";
                    trialCount++;
                    if(trialCount >= MAX_TRIALS) break;
                    std::this_thread::sleep_for(std::chrono::seconds(2));
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            continue;
        }
        trialCount = 0; 
        failCount = 0;

        cv::Mat grayFrame;
        cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);

        {
            std::lock_guard<std::mutex> lock(frameMutex_);
            grayFrame.copyTo(lastFrame_);
        }

        // std::cerr << "[CAM] frame published\n";

        write_frame(grayFrame);
    }

}

void RtspReader::TcpReaderLoop() {
    // std::cout << "in TcpReaderLoop" << std::endl;
    while (running_) {
        // std::cout << "Waiting to receive image from TCP stream..." << std::endl;
        cv::Mat frame;
        // bool success = receiveImage(client_socket, frame);
        std::vector<uint8_t> buffer(frameSize_);
        bool success = recvAll(client_socket, buffer.data(), frameSize_);
        // std::cout << "Received frame of size: " << buffer.size() << std::endl;
        if (success ) {
            // frame = cv::imdecode(buffer, cv::IMREAD_COLOR);
            cv::Mat gbra_frame(height_, width_, CV_8UC4, buffer.data());
            // 2. Create the destination matrix
            cv::Mat frame;

            // 3. Convert GBRA to BGR 
            // Since G=0, B=1, R=2, A=3, transforming to BGR (1,0,2) requires a custom color mix or manual channel shuffling.
            // The cleanest native OpenCV way is to swap the channels manually:
            int from_to[] = { 0,0,  1,1,  2,2 }; // Map G->B, B->G, R->R
            frame.create(height_, width_, CV_8UC3);
            cv::mixChannels(&gbra_frame, 1, &frame, 1, from_to, 3);
            if (!frame.empty()) {
                
                cv::Mat grayFrame;
                cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
                {
                    std::lock_guard<std::mutex> lock(frameMutex_);
                    grayFrame.copyTo(lastFrame_);
                }
                write_frame(grayFrame);
            }
        }
    }
}


void RtspReader::create_and_map_shm() {
    // Shared memory layout:
    // Offset  Size  Type      Field
    // ─────────────────────────────────────────────────
    //   0     1     bool      is_alive
    //   1     1     bool      is_writing
    //   2     1     bool      is_reading
    //   3     1     [padding]
    //   4     4     uint32_t  frame_id
    //   8     4     uint32_t  width (1920 or your WIDTH)
    //  12     4     uint32_t  height (1080 or your HEIGHT)
    //  16     ?     uint8_t[] image_data (WIDTH * HEIGHT bytes)
    //
    // Total header size: 16 bytes (on 64-bit systems)
    // Image offset: shm_ptr + 16
 

    fd_ = shm_open(shm_name_frame_.c_str(), O_CREAT | O_RDWR, 0666);
    if (fd_ < 0) {
      throw std::runtime_error("shm_open failed");
    }

    // Allocate memory (~2.07 MB)
    if (ftruncate(fd_, TOTAL_SHM_SIZE) == -1) {
      close(fd_);
      throw std::runtime_error("ftruncate failed");
    }

    void* ptr = mmap(nullptr, TOTAL_SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (ptr == MAP_FAILED) {
      close(fd_);
      throw std::runtime_error("mmap failed");
    }

    shm_ptr_ = static_cast<uint8_t*>(ptr);
    header_ = reinterpret_cast<Header*>(shm_ptr_);
    image_buffer_ = shm_ptr_ + sizeof(Header);

    // Initialize Header Flags
    header_->is_alive = 1;
    header_->is_writing = 0;
    header_->is_reading = 0;
    header_->frame_id = 0;
    header_->width = WIDTH;
    header_->height = HEIGHT;

    std::cout << "[SHM Writer] Single frame memory ready at " << shm_name_frame_ << "\n";
}

void RtspReader::write_frame(const cv::Mat& frame) {
    // static std::atomic<uint32_t> last_frame_id{0};
    if (frame.empty()) return;
    cv::Mat gray_frame;
    if(frame.type() == CV_8UC3) {
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
    }
    else if(frame.type() == CV_8UC1) {
        gray_frame = frame;
    }
    else {
        header_->is_alive = 0; // Mark as not alive due to unsupported frame type
        std::cerr << " [SHM Writer] Unsupported frame type: " << frame.type() << std::endl;
        return;
    }

    if (gray_frame.cols != WIDTH || gray_frame.rows != HEIGHT || gray_frame.type() != CV_8UC1) {
    //   throw std::invalid_argument("Frame must be 1080p Grayscale (CV_8UC1)");
        throw std::invalid_argument(
                "Frame must be " + std::to_string(WIDTH) + "x" + std::to_string(HEIGHT) + 
                " Grayscale (CV_8UC1), got " + std::to_string(gray_frame.cols) + "x" + 
                std::to_string(gray_frame.rows)
            );
    }

    // 1. Skip writing if Python is actively reading to prevent corrupting the frame
    while (header_->is_reading) {
        std::this_thread::sleep_for(std::chrono::microseconds(10)); 
    }

    // 2. Set is_writing flag to 1
    header_->is_writing = 1;

    // 3. Fast memory copy (~0.05ms)
    // std::memcpy(image_buffer_, gray_frame.data, FRAME_SIZE);
    // 3. Write image data to shared memory
    // Memory layout:
    // [0-15]        : Header struct (3 bools + padding + uint32_t frame_id + width/height)
    // [16-end]      : Image buffer (WIDTH * HEIGHT bytes)
    // uint8_t* dest = image_buffer_;
    // const uint8_t* src = gray_frame.data;

    // Use memcpy for contiguous row data
    if (gray_frame.isContinuous()) {
        // Fast path: frame data is contiguous in memory
        std::memcpy(image_buffer_, gray_frame.data, FRAME_SIZE);
    } else {
        // Slow path: frame has padding between rows (shouldn't happen with OpenCV)
        for (int row = 0; row < gray_frame.rows; ++row) {
            std::memcpy(
                image_buffer_ + row * WIDTH,
                gray_frame.data + row * gray_frame.step,
                WIDTH
            );
        }
    }

    // 4. Atomically increment frame counter
    // Increment happens AFTER image is written to ensure reader sees complete frame
    header_->frame_id++;
    
    // 5. Signal that writing is done
    header_->is_writing = 0;

    // 4. Increment frame_id
    // header_->frame_id = header_->frame_id.fetch_add(1, std::memory_order_relaxed) + 1;
    // header_->frame_id.fetch_add(1, std::memory_order_relaxed);

    // 5. Clear is_writing flag
    // header_->is_writing.store(0, std::memory_order_release);

}

void RtspReader::cleanupSharedMemory() {
    // 1. Unmap the memory pointer (if it was successfully mapped)
    header_->is_alive = 0;
    std::this_thread::sleep_for(std::chrono::seconds(10));
    if (shm_ptr_ != nullptr) {
        // TOTAL_SHM_SIZE must be the same size you used in mmap()
        if (munmap(shm_ptr_, TOTAL_SHM_SIZE) == -1) {
            std::cerr << "munmap failed" << std::endl;
        }
        shm_ptr_ = nullptr;
        header_ = nullptr;
        image_buffer_ = nullptr;
    }

    // 2. Close the file descriptor (if it is open)
    if (fd_ != -1) {
        close(fd_);
        fd_ = -1;
    }

    // 3. Unlink the shared memory object from the system (/dev/shm/...)
    // Only call this if your process is responsible for destroying the shared memory region.
    if (shm_unlink(shm_name_frame_.c_str()) == -1) {
        // It's common to ignore ENOENT (already unlinked) if multiple processes share it
        if (errno != ENOENT) {
            std::cerr << "shm_unlink failed" << std::endl;
        }
    }
}