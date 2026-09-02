#pragma once

#include <fstream>
#include <mutex>
#include <string>
#include <sstream>

class Logger
{
public:
    explicit Logger(const std::string& filename);

    void log(const std::string& tag, const std::string& message);

private:
    std::ofstream file;
    std::mutex mutex;
};