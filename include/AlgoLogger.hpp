#pragma once

#include <opencv2/core.hpp>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

class AlgoLogger {
public:
    using Clock = std::chrono::steady_clock;

    struct Command6 {
        // matches: ss << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << cmd_vx.z
        double c0 = 0, c1 = 0, c2 = 0, c3 = 0, c4 = 0, c5 = 0;
    };

    explicit AlgoLogger(std::string csv_path,
                        bool write_header = true,
                        size_t flush_every_n = 60,
                        bool thread_safe = true)
        : path_(std::move(csv_path)),
          flush_every_n_(flush_every_n),
          thread_safe_(thread_safe) 
    {
        out_.open(path_, std::ios::out | std::ios::app);
        if (!out_) {
            throw std::runtime_error("AlgoLogger: failed to open file: " + path_);
        }

        // If file is empty, write header (or if requested explicitly).
        if (write_header && out_.tellp() == 0) {
            writeHeader();
            out_.flush();
        }
    }

    ~AlgoLogger() {
        flush();
        out_.close();
    }

    // Timestamp helpers: seconds from steady_clock start
    static double nowSec() {
        const auto t = Clock::now().time_since_epoch();
        return std::chrono::duration<double>(t).count();
    }

    // Parse a command string like: "0,0,0,0,0,1.23\n"
    static std::optional<Command6> parseCommand6(const std::string& cmd) {
        Command6 c;
        std::stringstream ss(cmd);
        std::string item;
        std::vector<double> vals;
        vals.reserve(6);

        while (std::getline(ss, item, ',')) {
            // strip trailing newline/spaces
            while (!item.empty() && (item.back() == '\n' || item.back() == '\r' || item.back() == ' ' || item.back() == '\t'))
                item.pop_back();
            if (item.empty()) return std::nullopt;

            try {
                vals.push_back(std::stod(item));
            } catch (...) {
                return std::nullopt;
            }
        }
        if (vals.size() != 6) return std::nullopt;

        c.c0 = vals[0]; c.c1 = vals[1]; c.c2 = vals[2];
        c.c3 = vals[3]; c.c4 = vals[4]; c.c5 = vals[5];
        return c;
    }

    // Main logging call:
    // - img_ts: when image was received (seconds)
    // - send_ts: when reply was sent (seconds)
    // - rvec: rotation vector (Rodrigues) or just any 3-float vector
    // - t_px: translation in pixels
    // - cmd_raw: the exact string you sent (optional, recommended)
    // - cmd_parsed: parsed numeric command fields (optional)
    void log(double img_ts,
             double send_ts,
             const cv::Point3f& rvec,
             const cv::Point3f& t_px,
             const std::string& cmd_raw = "",
             const std::optional<Command6>& cmd_parsed = std::nullopt)
    {
        const double dt_ms = (send_ts - img_ts) * 1000.0;

        std::ostringstream line;
        line.setf(std::ios::fixed);
        line << std::setprecision(6)
             << img_ts << ","
             << send_ts << ","
             << std::setprecision(3) << dt_ms << ","
             << std::setprecision(6)
             << rvec.x << "," << rvec.y << "," << rvec.z << ","
             << std::setprecision(3)
             << t_px.x << "," << t_px.y << "," << t_px.z << ",";

        // command numeric fields (6 columns). If not provided, write empty
        if (cmd_parsed) {
            line << cmd_parsed->c0 << "," << cmd_parsed->c1 << "," << cmd_parsed->c2 << ","
                 << cmd_parsed->c3 << "," << cmd_parsed->c4 << "," << cmd_parsed->c5 << ",";
        } else {
            line << ",,,,,,";
        }

        // raw command as a quoted CSV cell (escape quotes)
        line << quoteCsv(cmd_raw) << "\n";

        writeLine(line.str());
    }

    void flush() {
        lock_();
        out_ << buffer_;
        buffer_.clear();
        out_.flush();
        unlock_();
        pending_lines_ = 0;
    }

private:
    std::string path_;
    std::ofstream out_;
    std::string buffer_;
    size_t flush_every_n_ = 60;
    size_t pending_lines_ = 0;
    bool thread_safe_ = true;
    std::mutex mtx_;

    void writeHeader() {
        out_ << "img_ts,send_ts,dt_ms,"
                "rvec_x,rvec_y,rvec_z,"
                "tx_px,ty_px,"
                "cmd0,cmd1,cmd2,cmd3,cmd4,cmd5,"
                "cmd_raw\n";
    }

    void writeLine(const std::string& s) {
        lock_();
        buffer_ += s;
        ++pending_lines_;
        if (pending_lines_ >= flush_every_n_) {
            out_ << buffer_;
            buffer_.clear();
            out_.flush();
            pending_lines_ = 0;
        }
        unlock_();
    }

    void lock_()   { if (thread_safe_) mtx_.lock(); }
    void unlock_() { if (thread_safe_) mtx_.unlock(); }

    static std::string quoteCsv(const std::string& in) {
        // CSV quote with "..." and escape quotes by doubling
        std::string out;
        out.reserve(in.size() + 2);
        out.push_back('"');
        for (char ch : in) {
            if (ch == '"') out += "\"\"";
            else if (ch == '\r') {/*skip*/}
            else out.push_back(ch);
        }
        out.push_back('"');
        return out;
    }
};
