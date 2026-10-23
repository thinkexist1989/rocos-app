#include "logger.hpp"
#include "spdlog/sinks/base_sink.h"
#include <map>

namespace {
class ManagerSink : public spdlog::sinks::base_sink<std::mutex> {
    void sink_it_(const spdlog::details::log_msg& msg) override {
        rocos::LogManager::instance().submit(static_cast<rocos::LogLevel>(msg.level),
            std::string(msg.logger_name.data(), msg.logger_name.size()),
            std::string(msg.payload.data(), msg.payload.size()), 0,
            std::chrono::duration_cast<std::chrono::microseconds>(msg.time.time_since_epoch()).count());
    }
    void flush_() override {} // 刷新仅由独立文件线程执行。
};
}
Logger::logger_ptr Logger::getInstance(const std::string& name) {
    static std::mutex mutex;
    static std::map<std::string, logger_ptr> instances;
    static auto sink = std::make_shared<ManagerSink>();
    std::lock_guard<std::mutex> lock(mutex);
    auto& logger = instances[name];
    if (!logger) {
        logger = std::make_shared<spdlog::logger>(name, sink);
        logger->set_level(spdlog::level::trace);
    }
    return logger;
}
