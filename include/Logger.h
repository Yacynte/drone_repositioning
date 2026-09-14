#pragma once

#include <fstream>
#include <mutex>
#include <string>
#include <sstream>

// Thread-safe, append-only text logger. Every log() call writes one line formatted
// as "[tag] message" and flushes immediately, so the log file stays current even if
// the process crashes. Used for human-readable runtime/debug logs (see AlgoLogger.hpp
// for the separate structured CSV logger).
class Logger
{
public:
    // Opens filename in append mode. Throws std::runtime_error if it can't be opened.
    explicit Logger(const std::string& filename);

    // Writes "[tag] message\n" to the log file. Safe to call from multiple threads.
    void log(const std::string& tag, const std::string& message);

private:
    std::ofstream file;
    std::mutex mutex;
};