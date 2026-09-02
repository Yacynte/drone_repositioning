#include "Logger.h"

Logger::Logger(const std::string& filename)
{
    file.open(filename, std::ios::app);

    if (!file.is_open())
        throw std::runtime_error("Failed to open log file");
}

void Logger::log(const std::string& tag, const std::string& message)
{
    std::lock_guard<std::mutex> lock(mutex);

    file << "[" << tag << "] "
         << message
         << '\n';

    file.flush();
}