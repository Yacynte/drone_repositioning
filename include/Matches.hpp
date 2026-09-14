// sp_sg_reader.hpp
#pragma once
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <array>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <optional>
#include <sstream>
#include <thread> 
#include <chrono>
#include <Utils.h>
#include "Logger.h"

// Reads SuperPoint/LightGlue keypoint matches out of the POSIX shared-memory segment
// written by the external Python process (src_py/matches_onnx.py, `_write_matches`).
// The `Matches` struct itself is defined in Utils.h since both sides (this reader and
// ImageMatcher) need it.
//
// Shared memory layout (SHM_SIZE bytes, written by Python):
//   [0]      uint8   is_alive     (0 = Python has shut down)
//   [1]      uint8   is_writing   (1 while Python is mid-write)
//   [2]      uint8   is_reading   (1 while this reader is mid-copy)
//   [3..6]   uint32  frame_id     (incremented by Python on every new match set)
//   [7..10]  int32   n            (number of matches in this payload, may be 0)
//   [...]    n * (kpts0, kpts1, scores, covariances) — see parse() below.
class SPSGReader {
public:
    static constexpr int    MAX_KP   = 512;
    static constexpr size_t SHM_SIZE = 1 + 1 + 1 + 4 + 4 + MAX_KP * (2+2+1+3) * 4;

    // Opens the shared-memory segment `name`, retrying for up to ~5 minutes (600 *
    // 500ms) since Python may not have created it yet at process startup. Throws
    // std::runtime_error if it never becomes available.
    explicit SPSGReader(Logger& logger, const char* name = "/sp_sg_matches"): logger(logger) {

        for (int i = 0; i < 600; i++){
            fd_ = shm_open(name, O_RDWR, 0666);
            if (fd_ >= 0) {
                // Ensure Python finished setting up the payload size to prevent SIGBUS errors
                struct stat shm_stat;
                if (fstat(fd_, &shm_stat) == 0 && shm_stat.st_size >= static_cast<off_t>(SHM_SIZE)) {
                    break; // Fully initialized by Python!
                }
                // File exists but size isn't ready yet — close and retry
                close(fd_);
                fd_ = -1;
            }
            // std::cout << "Waiting for python \n";
            if (i % 10 == 0) { // Print once every 5 seconds to reduce terminal log spam
                {
                    std::ostringstream ss;
                    ss << "Waiting for Python shared memory " << name
                       << " (last error: " << std::strerror(errno) << ")";
                    logger.log("SPSGReader", ss.str());
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            fd_ = shm_open(name, O_RDWR, 0666);
        }
        if (fd_ < 0) {
            throw std::runtime_error( std::string("shm_open failed after timeout: ") + std::strerror(errno)); 
        }
        ptr_ = mmap(nullptr, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (ptr_ == MAP_FAILED)
            throw std::runtime_error("mmap failed");
    }

    ~SPSGReader() {
        if (ptr_ != MAP_FAILED && ptr_ != nullptr) {
            munmap(ptr_, SHM_SIZE);
        }
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    // Waits (busy-spin) for Python to finish writing a new frame's matches, then
    // copies the payload out and parses it. Returns std::nullopt if Python has set
    // is_alive = 0 (shut down); returns an empty Matches{} (newMatches = false) if the
    // frame_id hasn't advanced since the last call.
    std::optional<Matches> read() {
        volatile uint8_t* data  = static_cast<volatile uint8_t*>(ptr_);
        if (data[0] == 0) {
            logger.log("SPSGReader", "Python shut down, exiting.");
            return std::nullopt;
        }
        uint32_t current_id;
        
        // spin while WRITING flag is set
        // Retry loop: handles the race between spin-exit and frame_id read
        while (true) {
            while (data[1] == 1){
                #if defined(__x86_64__)
                    __builtin_ia32_pause();
                #else
                    asm volatile("yield" ::: "memory");
                #endif
            }
            memcpy(&current_id, (const uint8_t*)data + 3, 4);

            // Re-check: if Python started writing while we read, retry
            std::atomic_thread_fence(std::memory_order_acquire);  // ARM cache coherence
            if (data[1] == 0) break;
        }
        memcpy(&current_id, (const uint8_t*)data + 3, 4);

        if (current_id == last_frame_id_)
            return Matches{};   // same frame — return empty, don't print

        data[2] = 1;
        // take a snapshot
        static std::vector<uint8_t> snap(SHM_SIZE);
        memcpy(snap.data(), (const uint8_t*)data, SHM_SIZE);

        data[2] = 0;
        last_frame_id_ = current_id;
        return parse(snap.data(), logger);
    }

    // Signals Python to stop (is_alive = 0) and unmaps/closes the shared-memory segment.
    bool stop() {
        volatile uint8_t* data  = static_cast<volatile uint8_t*>(ptr_);
        data[0] = 0; // signal to Python to stop
        if (ptr_ != nullptr) {
            if (munmap(ptr_, SHM_SIZE) == -1) {
                logger.log("SPSGReader", "error: munmap failed");
            }
            ptr_ = nullptr;
        }

        if (fd_ != -1) {
            close(fd_);
            fd_ = -1;
        }

        return true;
    }

private:
    Logger& logger;
    int    fd_  = -1;
    void*  ptr_ = nullptr;
    Matches last_;
    uint32_t last_frame_id_ = 0;

    // Deserializes a raw shared-memory snapshot (as laid out in the class comment
    // above) into a Matches struct.
    static Matches parse(const uint8_t* p, Logger& logger) {
        Matches m;
        p += 3 + 4;                                      // skip flag
        memcpy(&m.n, p, 4);  p += 4;

        if (m.n <= 0 || m.n > MAX_KP) return m;

        m.kpts0.resize(m.n);
        m.kpts1.resize(m.n);
        m.scores.resize(m.n);
        m.covariances.resize(m.n);
        memcpy(m.kpts0.data(),  p, m.n * sizeof(cv::Point2f));  p += m.n * sizeof(cv::Point2f);
        memcpy(m.kpts1.data(),  p, m.n * sizeof(cv::Point2f));  p += m.n * sizeof(cv::Point2f);
        memcpy(m.scores.data(), p, m.n * sizeof(float)); p += m.n * sizeof(float);
        memcpy(m.covariances.data(), p, m.n * sizeof(cv::Point3f));
        m.newMatches = true;
        {
            std::ostringstream ss;
            ss << "Number of matches obtained: " << m.n;
            logger.log("SPSGReader", ss.str());
        }
        return m;
    }
};


// Standalone smoke test — uncomment to check the shared-memory link to Python without
// running the full ImageMatcher pipeline.
// int main() {
//     std::cout << "SHM_SIZE=" << SPSGReader::SHM_SIZE
//               << " MAX_KP=" << SPSGReader::MAX_KP << "\n";
//     SPSGReader reader("/sp_sg_matches");
//     std::cout << "Connected to shared memory.\n";

//     while (true) {
//         auto result = reader.read();   // blocks ~nanoseconds if Python mid-write

//         if (!result.has_value()) {
//             std::cout << "Python shut down, exiting.\n";
//             break;
//         }
//         Matches m = result.value();
//         if (m.n > 0) std::cout << "matches: " << m.n << "\n";
//     }
// }
