#include "log_storage.hpp"
#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>

namespace rocos {
namespace {
const char* consoleColor(LogLevel level) {
    switch (level) {
        case LogLevel::Trace:
        case LogLevel::Debug: return "\033[36m"; // cyan
        case LogLevel::Info: return "\033[32m"; // green
        case LogLevel::Warn: return "\033[33m"; // yellow
        case LogLevel::Error: return "\033[91m"; // bright red
        case LogLevel::Critical: return "\033[97m\033[41m"; // white on red
    }
    return "";
}
}

void LoggingConfig::validate() const {
    if (directory.empty() || max_file_size == 0 || max_files == 0 || max_files > 10000 ||
        flush_interval_ms < 1 || flush_interval_ms > 60000 || file_level > LogLevel::Warn)
        throw std::invalid_argument("Invalid logging storage configuration");
}
const char* logLevelName(LogLevel level) {
    static const char* names[] = {"trace", "debug", "info", "warn", "error", "critical"};
    return names[static_cast<unsigned>(level)];
}
LogLevel parseLogLevel(const std::string& name) {
    for (unsigned i = 0; i < 6; ++i)
        if (name == logLevelName(static_cast<LogLevel>(i))) return static_cast<LogLevel>(i);
    throw std::invalid_argument("Unknown log level: " + name);
}
int64_t logNowMicros() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
std::string logTimestamp(int64_t epoch_us) {
    time_t seconds = epoch_us / 1000000;
    tm local{};
    localtime_r(&seconds, &local);
    char date[32], zone[8];
    strftime(date, sizeof(date), "%Y-%m-%dT%H:%M:%S", &local);
    strftime(zone, sizeof(zone), "%z", &local);
    std::ostringstream out;
    out << date << '.' << std::setw(3) << std::setfill('0') << (epoch_us / 1000 % 1000);
    std::string offset(zone);
    if (offset.size() == 5) offset.insert(3, ":");
    out << offset;
    return out.str();
}
LogFileSnapshot::~LogFileSnapshot() { if (fd >= 0) ::close(fd); }
LogStorage::LogStorage(LoggingConfig config, std::function<int64_t()> clock)
    : config_(std::move(config)), clock_(std::move(clock)) {
    config_.validate();
    thread_ = std::thread([this] { run(); });
}
LogStorage::~LogStorage() { stop(); }
void LogStorage::stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
}
bool LogStorage::enqueue(const std::shared_ptr<FileLogRecord>& record) {
    if (queue_.tryPush(record)) return true;
    dropped_.fetch_add(1);
    return false;
}
SnapshotResult LogStorage::snapshot(std::chrono::milliseconds timeout) {
    auto request = std::make_shared<std::promise<SnapshotResult>>();
    auto future = request->get_future();
    {
        std::lock_guard<std::mutex> lock(requests_mutex_);
        if (!running_ || requests_.size() >= 4) return {};
        requests_.push_back(request);
    }
    if (future.wait_for(timeout) != std::future_status::ready) return {};
    return future.get();
}
void LogStorage::cleanup() {
    namespace fs = std::filesystem;
    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(config_.directory)) {
        const auto name = entry.path().filename().string();
        if (entry.is_regular_file() && name.rfind("rocos-app_", 0) == 0 && entry.path().extension() == ".log")
            files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());
    size_t remaining = files.size();
    for (const auto& file : files) {
        if (remaining <= config_.max_files) break;
        if (file == path_) continue;
        // POSIX unlink 不影响已打开的快照 fd，不复用或截断旧文件。
        if (fs::remove(file)) --remaining;
    }
}
void LogStorage::openFile(int64_t now) {
    namespace fs = std::filesystem;
    fs::create_directories(config_.directory);
    auto stamp = logTimestamp(now);
    day_ = stamp.substr(0, 10);
    std::replace(stamp.begin(), stamp.end(), ':', '-');
    for (;;) {
        std::ostringstream filename;
        filename << "rocos-app_" << stamp << '_' << std::setw(6) << std::setfill('0') << serial_++ << ".log";
        path_ = (fs::path(config_.directory) / filename.str()).string();
        int fd = ::open(path_.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0640);
        if (fd < 0 && errno == EEXIST) continue;
        if (fd < 0) throw std::runtime_error("Cannot create log file");
        output_ = fdopen(fd, "w");
        if (!output_) { ::close(fd); throw std::runtime_error("Cannot open log stream"); }
        break;
    }
    size_ = 0;
    cleanup();
}
void LogStorage::write(const FileLogRecord& record) {
    if (record.level >= config_.console_level) {
        const bool color = ::isatty(STDOUT_FILENO) != 0;
        const std::string level_token = "[" + std::string(logLevelName(record.level)) + "]";
        const auto level_pos = record.line.find(level_token);
        bool failed = false;
        if (color && level_pos != std::string::npos) {
            const char* level_color = consoleColor(record.level);
            const char* reset = "\033[0m";
            failed =
                (std::fwrite(record.line.data(), 1, level_pos, stdout) != level_pos) ||
                (std::fwrite(level_color, 1, std::strlen(level_color), stdout) != std::strlen(level_color)) ||
                (std::fwrite(level_token.data(), 1, level_token.size(), stdout) != level_token.size()) ||
                (std::fwrite(reset, 1, std::strlen(reset), stdout) != std::strlen(reset)) ||
                (std::fwrite(record.line.data() + level_pos + level_token.size(), 1,
                             record.line.size() - level_pos - level_token.size(), stdout) !=
                 record.line.size() - level_pos - level_token.size());
        } else {
            failed = std::fwrite(record.line.data(), 1, record.line.size(), stdout) != record.line.size();
        }
        if (failed) ++failures_;
    }
    if (record.level < config_.file_level) return;
    // 每条记录保持完整，超大单条允许超过文件上限；事件入口已限制长度。
    if (!output_ || (size_ > 0 && size_ + record.line.size() > config_.max_file_size) ||
        day_ != logTimestamp(clock_()).substr(0, 10)) {
        if (output_) { fclose(output_); output_ = nullptr; }
        openFile(clock_());
    }
    if (fwrite(record.line.data(), 1, record.line.size(), output_) != record.line.size())
        throw std::runtime_error("Log file write failed");
    size_ += record.line.size();
    if (record.level >= LogLevel::Error && fflush(output_) != 0)
        throw std::runtime_error("Log file flush failed");
}
void LogStorage::serveSnapshots() {
    std::deque<std::shared_ptr<std::promise<SnapshotResult>>> requests;
    { std::lock_guard<std::mutex> lock(requests_mutex_); requests.swap(requests_); }
    for (auto& request : requests) {
        SnapshotResult result;
        if (!output_) result.status = failures_ ? 503 : 404;
        else if (fflush(output_) == 0) {
            auto file = std::make_shared<LogFileSnapshot>();
            file->fd = ::open(path_.c_str(), O_RDONLY | O_CLOEXEC);
            struct stat statbuf{};
            if (file->fd >= 0 && fstat(file->fd, &statbuf) == 0) {
                file->size = static_cast<size_t>(statbuf.st_size);
                file->filename = std::filesystem::path(path_).filename().string();
                file->timestamp = logTimestamp(clock_());
                result = {200, std::move(file)};
            } else ++failures_;
        } else {
            ++failures_;
            fclose(output_); output_ = nullptr;
        }
        request->set_value(std::move(result));
    }
}
void LogStorage::run() {
    auto next_flush = std::chrono::steady_clock::now();
    // 启动时创建空文件，空快照同样有稳定名称和句柄。
    try { openFile(clock_()); } catch (...) { ++failures_; }
    for (;;) {
        std::shared_ptr<FileLogRecord> record;
        const bool available = queue_.tryPop(record);
        if (available) {
            try { write(*record); } catch (...) {
                ++failures_;
                if (output_) { fclose(output_); output_ = nullptr; }
            }
        }
        serveSnapshots();
        if (std::chrono::steady_clock::now() >= next_flush) {
            if (output_ && fflush(output_) != 0) {
                ++failures_;
                fclose(output_); output_ = nullptr;
            }
            next_flush = std::chrono::steady_clock::now() + std::chrono::milliseconds(config_.flush_interval_ms);
        }
        if (!available) {
            if (!running_) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    if (output_) { fclose(output_); output_ = nullptr; }
    serveSnapshots();
}
} // namespace rocos
