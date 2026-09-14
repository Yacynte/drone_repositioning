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


// Structured, buffered CSV logger for per-frame repositioning data (error, rotation,
// translation, and the parsed outgoing command). One row is written per call to
// log(); see writeHeader() for the exact column layout. Buffers lines in memory and
// flushes to disk every flush_every_n rows (or via flush()/the destructor) to keep
// disk I/O off the hot path.
class AlgoLogger {
public:
    // Mirrors the 6 numeric fields of the "roll,pitch,yaw,vx,vy,vz,state" command
    // string sent to the drone/Unreal Engine (see MetadataTcpClient::respositionFunc).
    struct Command6 {
        double c0 = 0, c1 = 0, c2 = 0, c3 = 0, c4 = 0, c5 = 0;
    };

    // Opens csv_path in append mode. Writes the CSV header if the file is new/empty
    // and write_header is true. Throws std::runtime_error if the file can't be opened.
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

    // Current wall-clock time as fractional seconds since the Unix epoch.
    static double nowWallSec() {
        auto t = std::chrono::system_clock::now().time_since_epoch();
        return std::chrono::duration<double>(t).count();
    }


    // Parses a "c0,c1,c2,c3,c4,c5,state\n" command string (7 comma-separated numbers;
    // the trailing `state` field is parsed but not stored) into a Command6. Returns
    // std::nullopt if cmd isn't exactly 7 well-formed numbers.
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
        if (vals.size() != 7) return std::nullopt;

        c.c0 = vals[0]; c.c1 = vals[1]; c.c2 = vals[2];
        c.c3 = vals[3]; c.c4 = vals[4]; c.c5 = vals[5];
        return c;
    }

    // Appends one CSV row for the current frame:
    // - send_ts: wall-clock time the command was computed/sent (seconds)
    // - px_error: alignment/reprojection error (see writeHeader() for the column name)
    // - rvec: estimated rotation error (roll, pitch, yaw, degrees)
    // - unit_vec: normalized translation direction
    // - cmd_parsed: the 6 numeric fields of the outgoing drone command, or nullopt if
    //   this frame's alignment failed (writes an all-empty row in that case)
    void log(double send_ts,
             float px_error,
             const cv::Point3f& rvec,
             const cv::Point3f& unit_vec,
             const std::optional<Command6>& cmd_parsed = std::nullopt)
    {
        std::ostringstream line;
        line.setf(std::ios::fixed);

        if (cmd_parsed) {
            line << std::setprecision(6)
             << send_ts << ","
             << std::setprecision(3) << px_error << ","
             << std::setprecision(3)
             << unit_vec.x << "," << unit_vec.y << "," << unit_vec.z << ","
             << std::setprecision(3)
             << rvec.x << "," << rvec.y << "," << rvec.z << ","
             << std::setprecision(3)
             << cmd_parsed->c0 << "," << cmd_parsed->c1 << "," << cmd_parsed->c2 << ","
             << std::setprecision(3)
             << cmd_parsed->c3 << "," << cmd_parsed->c4 << "," << cmd_parsed->c5 << "\n";
        } else {
            line << ",,,,,, \n";
        }

        writeLine(line.str());
    }

    // Writes any buffered rows to disk immediately, regardless of flush_every_n.
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

    // Writes the CSV column header row (called once, on first open of a new file).
    void writeHeader() {
        out_ << "send_ts,repo_error,px_error_x, px_error_y,"
                "unit_vec_x, unit_vec_y, unit_vec_z,"
                "rot_error_x, rot_error_y, rot_error_z,"
                "w_x,w_y,w_z,v_x,v_y,v_z\n";
    }

    // Appends s to the in-memory buffer and flushes to disk once flush_every_n lines
    // have accumulated.
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

    // CSV-quotes a raw string (wraps in double quotes, escapes embedded quotes by
    // doubling them). Not currently used by log() but kept for logging raw command
    // strings if that's ever added back.
    static std::string quoteCsv(const std::string& in) {
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
