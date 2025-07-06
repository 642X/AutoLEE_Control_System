#include "Logger.h"
#include <iostream>

namespace SIAT {

LogLevel Logger::currentLevel = LogLevel::INFO;
std::mutex Logger::logMutex;

void Logger::setLogLevel(LogLevel level) {
    currentLevel = level;
}

void Logger::log(LogLevel level, const std::string& module, const std::string& message) {
    if (level < currentLevel) return;

    std::lock_guard<std::mutex> lock(logMutex);
    std::cout << "[" << levelToString(level) << "][SIAT][" << module << "] " << message << std::endl;
}

std::string Logger::levelToString(LogLevel level) {
    switch (level) {
        case LogLevel::INFO: return "INFO";
        case LogLevel::WARNING: return "WARNING";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::DEBUG: return "DEBUG";
        default: return "UNKNOWN";
    }
}

} // namespace SIAT
