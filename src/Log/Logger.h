#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <mutex>

namespace SIAT {

enum class LogLevel {
    INFO,
    WARNING,
    ERROR,
    DEBUG
};

class Logger {
public:
    // 设置日志级别（全局）
    static void setLogLevel(LogLevel level);

    // 带模块名称的日志接口
    static void log(LogLevel level, const std::string& module, const std::string& message);

private:
    static LogLevel currentLevel;
    static std::mutex logMutex;

    static std::string levelToString(LogLevel level);

    // 禁止实例化
    Logger() = delete;
    ~Logger() = delete;
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
};

} // namespace SIAT

// === 宏封装：简洁调用 ===
#define LOGI(module, msg) SIAT::Logger::log(SIAT::LogLevel::INFO, module, msg)
#define LOGW(module, msg) SIAT::Logger::log(SIAT::LogLevel::WARNING, module, msg)
#define LOGE(module, msg) SIAT::Logger::log(SIAT::LogLevel::ERROR, module, msg)
#define LOGD(module, msg) SIAT::Logger::log(SIAT::LogLevel::DEBUG, module, msg)

#endif // LOGGER_H
