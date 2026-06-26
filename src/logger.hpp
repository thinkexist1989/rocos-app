#pragma once

#include <memory>
#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/daily_file_sink.h"
#include <spdlog/sinks/stdout_color_sinks.h>
#include "spdlog/fmt/bundled/color.h"

/*打印级别
trace: 深度诊断信息，详细的跟踪日志。
debug: 调试阶段的日志信息，通常只在开发环境启用。
info: 正常操作信息，如启动成功、请求完成等。
warn: 非常规但可容忍的情况，如使用了废弃功能。
error: 出现错误，但通常不影响系统整体运行。
critical: 严重错误，需要立即关注。
*/

#ifdef _WIN32
#define DllExport __declspec(dllexport)
#else
#define DllExport __attribute__((visibility("default")))
#endif


class DllExport Logger {
public:
    using string = std::string;
    using sink_ptr = spdlog::sink_ptr;
    using async_logger = spdlog::async_logger;
    using Logger_ptr = std::shared_ptr<Logger>;
    using logger_ptr = std::shared_ptr<async_logger>;

    explicit Logger(string name);

    ~Logger();

    static logger_ptr getInstance(const std::string &name);

private:

    // static sink_ptr daily_sink;
    // static sink_ptr console_sink;
    //
    // // 组合接收器
    // static std::vector<sink_ptr> sinks;

    static bool isThreadPoolInit;
    std::vector<sink_ptr> &getSinks() ;

    // 禁止复制和赋值
    // Logger(const Logger&) = delete;
    // Logger& operator=(const Logger&) = delete;

    logger_ptr logger_ptr_;

    string name_;
};

