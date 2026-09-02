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

// struct Matches {
//     int n = 0;
//     std::vector<cv::Point2f> kpts0;
//     std::vector<cv::Point2f> kpts1;
//     std::vector<float> scores;
//     std::vector<cv::Point3f> covariances;
//     bool newMatches = false;
// };

class SPSGReader {
public:
    static constexpr int    MAX_KP   = 512;
    static constexpr size_t SHM_SIZE = 1 + 1 + 1 + 4 + 4 + MAX_KP * (2+2+1+3) * 4;

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

    // Spin until Python finishes writing, then copy.
    std::optional<Matches> read() {
        // const volatile uint8_t* base = static_cast<volatile uint8_t*>(ptr_);
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

        // while (data[1] == 2)  // spin while writing
        //     asm volatile("yield" ::: "memory");

        data[2] = 1;
        // take a snapshot
        // uint8_t snap[SHM_SIZE];
        static std::vector<uint8_t> snap(SHM_SIZE);
        memcpy(snap.data(), (const uint8_t*)data, SHM_SIZE);

        data[2] = 0;
        last_frame_id_ = current_id;
        return parse(snap.data(), logger);
    }

    bool stop() {
        volatile uint8_t* data  = static_cast<volatile uint8_t*>(ptr_);
        data[0] = 0; // signal to Python to stop
        if (ptr_ != nullptr) {
            // TOTAL_SHM_SIZE must be the same size you used in mmap()
            if (munmap(ptr_, SHM_SIZE) == -1) {
                logger.log("SPSGReader", "error: munmap failed");
            }
            ptr_ = nullptr;
        }
       
        // 2. Close the file descriptor (if it is open)
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

//         // for (int i = 0; i < m.n; ++i) {
//         //     float x0 = m.kpts0[i][0], y0 = m.kpts0[i][1];
//         //     float x1 = m.kpts1[i][0], y1 = m.kpts1[i][1];
//         //     float s  = m.scores[i];
//         //     // feed into your pipeline
//         // }
//     }
// }
