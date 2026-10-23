#pragma once

#include "log_event.hpp"
#include <chrono>
#include <deque>
#include <future>
#include <functional>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace rocos {
struct LoggingConfig {
    std::string directory{"logs"};
    LogLevel file_level{LogLevel::Info}, console_level{LogLevel::Info};
    size_t max_file_size{50 * 1024 * 1024}, max_files{20};
    int flush_interval_ms{1000};
    void validate() const;
};
LogLevel parseLogLevel(const std::string& name);
const char* logLevelName(LogLevel level);
std::string logTimestamp(int64_t epoch_us);
int64_t logNowMicros();
struct FileLogRecord {
    LogLevel level;
    std::string line;
    int64_t epoch_us;
};
struct LogFileSnapshot {
    ~LogFileSnapshot();
    int fd{-1};
    size_t size{0};
    std::string filename, timestamp;
};
struct SnapshotResult {
    int status{503};
    std::shared_ptr<LogFileSnapshot> file;
};

// 只有文件线程接触写句柄。请求线程通过有界邮箱请求切点，不等待上游排空。
class LogStorage {
public:
    explicit LogStorage(LoggingConfig config, std::function<int64_t()> clock = logNowMicros);
    ~LogStorage();
    bool enqueue(const std::shared_ptr<FileLogRecord>& record);
    SnapshotResult snapshot(std::chrono::milliseconds timeout = std::chrono::milliseconds(250));
    uint64_t dropped() const { return dropped_.load(); }
    uint64_t failures() const { return failures_.load(); }
    void stop();
private:
    void run();
    void write(const FileLogRecord& record);
    void openFile(int64_t now);
    void cleanup();
    void serveSnapshots();
    LoggingConfig config_;
    std::function<int64_t()> clock_;
    SpscQueue<std::shared_ptr<FileLogRecord>, 2048> queue_;
    std::atomic<bool> running_{true};
    std::atomic<uint64_t> dropped_{0}, failures_{0};
    std::thread thread_;
    std::mutex requests_mutex_;
    std::deque<std::shared_ptr<std::promise<SnapshotResult>>> requests_;
    FILE* output_{nullptr};
    size_t size_{0};
    uint64_t serial_{0};
    std::string path_, day_;
};
} // namespace rocos
