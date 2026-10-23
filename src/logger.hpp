#pragma once
#include <memory>
#include <string>
#include "spdlog/spdlog.h"
#include "log_manager.hpp"

#ifdef _WIN32
#define DllExport __declspec(dllexport)
#else
#define DllExport __attribute__((visibility("default")))
#endif

// 普通线程兼容入口，实时调用点必须使用 log_event.hpp 的固定事件入口。
class DllExport Logger {
public:
    using logger_ptr = std::shared_ptr<spdlog::logger>;
    static logger_ptr getInstance(const std::string& name);
    template <class... Args>
    static void logCode(rocos::LogLevel level, const std::string& module, int code,
                        fmt::format_string<Args...> format, Args&&... args) {
        rocos::LogManager::instance().submit(level, module,
            fmt::format(format, std::forward<Args>(args)...), code);
    }
};
