#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "test/doctest.h"
#include "src/diagnostics_http_server.hpp"
#include <filesystem>
#include <fstream>
#include <cstring>

namespace rocos {
struct DiagnosticsHttpTestAccess {
    static void offline(DiagnosticsHttpServer& service, bool enable) {
        // cpp-httplib 用监听 fd 判断关闭；本测试使用内存流，从不操作此标记为 socket。
        service.server_.svr_sock_ = enable ? -2 : INVALID_SOCKET;
    }
    static bool request(DiagnosticsHttpServer& service, httplib::Stream& stream) {
        bool closed = false;
        return service.server_.process_request(stream, "127.0.0.1", 12345, "127.0.0.1", 8082, true, closed, nullptr);
    }
};
}
namespace {
using namespace rocos;
namespace fs = std::filesystem;
template <class F> bool until(F predicate, int timeout_ms = 5000) {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    do {
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (std::chrono::steady_clock::now() < end);
    return false;
}
class MemoryStream : public httplib::Stream {
public:
    explicit MemoryStream(std::string target, std::string headers = "", bool block_body = false)
        : input_("GET " + target + " HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n" + headers + "\r\n"), block_body_(block_body) {}
    bool is_readable() const override { return position_ < input_.size(); }
    bool wait_readable() const override { return is_readable(); }
    bool wait_writable() const override { return alive_; }
    bool is_peer_alive() const override { return alive_; }
    ssize_t read(char* ptr, size_t size) override {
        const auto count = std::min(size, input_.size() - position_);
        memcpy(ptr, input_.data() + position_, count);
        position_ += count;
        return count;
    }
    ssize_t write(const char* ptr, size_t size) override {
        std::unique_lock<std::mutex> lock(mutex_);
        if (block_body_ && output_.find("\r\n\r\n") != std::string::npos) {
            body_entered_ = true;
            cv_.notify_all();
            cv_.wait(lock, [&] { return !block_body_ || !alive_; });
        }
        if (!alive_) return -1;
        output_.append(ptr, size);
        cv_.notify_all();
        return size;
    }
    void get_remote_ip_and_port(std::string& ip, int& port) const override { ip = "127.0.0.1"; port = 12345; }
    void get_local_ip_and_port(std::string& ip, int& port) const override { ip = "127.0.0.1"; port = 8082; }
    socket_t socket() const override { return INVALID_SOCKET; }
    time_t duration() const override { return 0; }
    std::string output() { std::lock_guard<std::mutex> lock(mutex_); return output_; }
    bool bodyEntered() { std::lock_guard<std::mutex> lock(mutex_); return body_entered_; }
    void release() { std::lock_guard<std::mutex> lock(mutex_); block_body_ = false; cv_.notify_all(); }
    void close() { alive_ = false; release(); }
private:
    std::string input_, output_;
    size_t position_{0};
    std::atomic<bool> alive_{true};
    std::mutex mutex_;
    std::condition_variable cv_;
    bool block_body_{false}, body_entered_{false};
};
struct Connection {
    MemoryStream stream;
    std::atomic<bool> done{false};
    std::thread thread;
    Connection(DiagnosticsHttpServer& server, std::string path, std::string headers = "", bool blocked = false)
        : stream(std::move(path), std::move(headers), blocked), thread([this, &server] {
            DiagnosticsHttpTestAccess::request(server, stream);
            done = true;
        }) {}
    ~Connection() { stream.close(); if (thread.joinable()) thread.join(); }
    std::string header() {
        if (!until([&] { return stream.output().find("\r\n\r\n") != std::string::npos || done.load(); })) return "";
        auto output = stream.output();
        return output.substr(0, output.find("\r\n\r\n") + 4);
    }
};
struct Fixture {
    fs::path directory;
    std::unique_ptr<LogManager> logs;
    std::unique_ptr<DiagnosticsHttpServer> service;
    explicit Fixture(std::function<int64_t()> clock = logNowMicros) {
        char pattern[] = "/tmp/rocos-http-memory-XXXXXX";
        auto path = mkdtemp(pattern);
        if (!path) throw std::runtime_error("mkdtemp failed");
        directory = path;
        LoggingConfig config;
        config.directory = path;
        config.file_level = LogLevel::Trace;
        config.console_level = LogLevel::Critical;
        config.max_file_size = 32768;
        config.max_files = 1;
        logs = std::make_unique<LogManager>(config, std::move(clock));
        service = std::make_unique<DiagnosticsHttpServer>(*logs);
        DiagnosticsHttpTestAccess::offline(*service, true);
    }
    ~Fixture() {
        DiagnosticsHttpTestAccess::offline(*service, false);
        service.reset(); logs.reset(); fs::remove_all(directory);
    }
    std::string get(const std::string& path, const std::string& headers = "") {
        MemoryStream stream(path, headers);
        DiagnosticsHttpTestAccess::request(*service, stream);
        return stream.output();
    }
    LogJson query(const std::string& path = "/api/logs") {
        auto response = get(path);
        return LogJson::parse(response.substr(response.find("\r\n\r\n") + 4));
    }
    void alert(const std::string& message) {
        logs->submit(LogLevel::Warn, "Test", message);
        if (!until([&] { auto events = logs->query("", 1)["events"]; return !events.empty() && events.back()["message"] == message; }))
            throw std::runtime_error("Alert processing timed out");
    }
};
}
TEST_CASE_FIXTURE(Fixture, "In-memory HTTP parser validates query and returns six-field warning records") {
    auto empty = query();
    CHECK(empty["success"] == true);
    CHECK(empty["data"]["events"].empty());
    auto cursor = empty["data"]["next_id"].get<std::string>();
    for (auto path : {"/api/logs?limit=0", "/api/logs?limit=-1", "/api/logs?limit=501", "/api/logs?limit=1x", "/api/logs?after=bad", "/api/logs/stream?after=bad"}) {
        auto response = get(path);
        CHECK(response.find("400 Bad Request") != std::string::npos);
        CHECK(response.find("application/json") != std::string::npos);
    }
    alert("first alert"); alert("second alert");
    auto first = query("/api/logs?after=" + cursor + "&limit=1");
    CHECK(first["data"]["events"].size() == 1);
    CHECK(first["data"]["events"][0].size() == 6);
    CHECK(first["data"]["has_more"] == true);
    CHECK(first["data"]["events"][0]["code"].is_null());
    CHECK(query("/api/logs?after=previous:42")["data"]["gap"]["reason"] == "server_restarted");
}
TEST_CASE_FIXTURE(Fixture, "In-memory SSE quotas are atomic release reliably and preserve query admission") {
    std::vector<std::unique_ptr<Connection>> clients;
    for (int i = 0; i < 4; ++i) {
        auto client = std::make_unique<Connection>(*service, "/api/logs/stream");
        auto header = client->header();
        CHECK(header.find("200 OK") != std::string::npos);
        CHECK(header.find("text/event-stream") != std::string::npos);
        CHECK(header.find("X-Rocos-SSE-Limit: 4") != std::string::npos);
        clients.push_back(std::move(client));
    }
    auto rejected = get("/api/logs/stream");
    CHECK(rejected.find("429 Too Many Requests") != std::string::npos);
    CHECK(rejected.find("Retry-After: 5") != std::string::npos);
    CHECK(rejected.find("application/json") != std::string::npos);
    CHECK(query()["success"] == true);
    clients.front().reset();
    Connection replacement(*service, "/api/logs/stream");
    CHECK(replacement.header().find("200 OK") != std::string::npos);
    alert("live warning");
    REQUIRE(until([&] { return replacement.stream.output().find("live warning") != std::string::npos; }));
    auto start = std::chrono::steady_clock::now();
    service->stop();
    REQUIRE(until([&] { return replacement.done.load(); }));
    CHECK(std::chrono::steady_clock::now() - start < std::chrono::seconds(1));
}
TEST_CASE_FIXTURE(Fixture, "In-memory SSE precedence ordering replay restart gaps and 15 second heartbeat") {
    auto zero = logs->query("", 1)["next_id"].get<std::string>();
    alert("first warning");
    auto first = logs->query("", 1)["next_id"].get<std::string>();
    alert("second warning");
    Connection stream(*service, "/api/logs/stream?after=malformed", "Last-Event-ID: " + first + "\r\n");
    REQUIRE(stream.header().find("200 OK") != std::string::npos);
    REQUIRE(until([&] { return stream.stream.output().find("second warning") != std::string::npos; }));
    CHECK(stream.stream.output().find("first warning") == std::string::npos);
    auto events = logs->query(first, 1)["events"];
    CHECK(stream.stream.output().find("id: " + events[0]["id"].get<std::string>()) != std::string::npos);
    REQUIRE(until([&] { return stream.stream.output().find(": heartbeat") != std::string::npos; }, 17000));
    auto output = stream.stream.output();
    auto heartbeat = output.substr(output.find(": heartbeat"));
    CHECK(heartbeat.find("event:") == std::string::npos);
    CHECK(heartbeat.find("id:") == std::string::npos);
    Connection restarted(*service, "/api/logs/stream?after=previous:42");
    REQUIRE(restarted.header().find("200 OK") != std::string::npos);
    REQUIRE(until([&] { return restarted.stream.output().find("second warning") != std::string::npos; }));
    CHECK(restarted.stream.output().find("server_restarted") != std::string::npos);
    Connection replay(*service, "/api/logs/stream?after=" + zero);
    REQUIRE(replay.header().find("200 OK") != std::string::npos);
    REQUIRE(until([&] { return replay.stream.output().find("second warning") != std::string::npos; }));
    auto both = replay.stream.output();
    CHECK(both.find("first warning") < both.find("second warning"));
}
TEST_CASE_FIXTURE(Fixture, "In-memory download uses fixed length empty files chunks and independent quotas") {
    auto empty = get("/api/logs/file/latest");
    CHECK(empty.find("200 OK") != std::string::npos);
    CHECK(empty.find("Content-Length: 0") != std::string::npos);
    logs->submit(LogLevel::Info, "Test", "ordinary info record");
    alert("warning before snapshot");
    REQUIRE(until([&] { auto file = logs->snapshot(); return file.file && file.file->size > 0; }));
    Connection download(*service, "/api/logs/file/latest", "", true);
    auto header = download.header();
    CHECK(header.find("Content-Type: text/plain; charset=utf-8") != std::string::npos);
    CHECK(header.find("Content-Disposition: attachment; filename=\"rocos-app_") != std::string::npos);
    CHECK(header.find("X-Rocos-Log-Snapshot-Time:") != std::string::npos);
    REQUIRE(until([&] { return download.stream.bodyEntered(); }));
    CHECK(get("/api/logs/file/latest").find("429 Too Many Requests") != std::string::npos);
    alert("warning after snapshot");
    CHECK(query()["success"] == true);
    auto first_name = logs->snapshot().file->filename;
    for (int i = 0; i < 20; ++i) {
        logs->submit(LogLevel::Info, "Test", std::string(4000, 'x'));
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
    REQUIRE(until([&] { auto file = logs->snapshot(); return file.file && file.file->filename != first_name; }));
    CHECK_FALSE(fs::exists(directory / first_name));
    download.stream.release();
    REQUIRE(until([&] { return download.done.load(); }));
    auto response = download.stream.output();
    auto body = response.substr(response.find("\r\n\r\n") + 4);
    const auto length = std::stoull(header.substr(header.find("Content-Length: ") + 16));
    CHECK(body.size() == length);
    CHECK(body.find("ordinary info record") != std::string::npos);
    CHECK(body.find("warning before snapshot") != std::string::npos);
    CHECK(body.find("warning after snapshot") == std::string::npos);
    CHECK(get("/api/logs/file/latest").find("200 OK") != std::string::npos);
}
TEST_CASE_FIXTURE(Fixture, "In-memory download failed send releases quota and never appends JSON to file body") {
    alert("download failure probe");
    Connection download(*service, "/api/logs/file/latest", "", true);
    REQUIRE(download.header().find("200 OK") != std::string::npos);
    REQUIRE(until([&] { return download.stream.bodyEntered(); }));
    download.stream.close();
    REQUIRE(until([&] { return download.done.load(); }));
    CHECK(download.stream.output().find("\"success\"") == std::string::npos);
    CHECK(get("/api/logs/file/latest").find("200 OK") != std::string::npos);
}

TEST_CASE("In-memory snapshot timeout returns JSON 503 and releases download quota") {
    std::atomic<bool> stalled{true};
    Fixture fixture([&] {
        while (stalled) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return logNowMicros();
    });
    struct Release { std::atomic<bool>& flag; ~Release() { flag = false; } } release{stalled};
    auto start = std::chrono::steady_clock::now();
    auto response = fixture.get("/api/logs/file/latest");
    CHECK(response.find("503 Service Unavailable") != std::string::npos);
    CHECK(response.find("application/json") != std::string::npos);
    CHECK(std::chrono::steady_clock::now() - start < std::chrono::seconds(1));
    CHECK(fixture.get("/api/logs/file/latest").find("503 Service Unavailable") != std::string::npos);
    stalled = false;
    REQUIRE(until([&] { return fixture.get("/api/logs/file/latest").find("200 OK") != std::string::npos; }));
}
TEST_CASE_FIXTURE(Fixture, "In-memory file read failure after headers terminates instead of appending an error envelope") {
    for (int i = 0; i < 4; ++i) alert(std::string(7000, static_cast<char>('a' + i)));
    REQUIRE(until([&] { auto snapshot = logs->snapshot(); return snapshot.file && snapshot.file->size > 25000; }));
    auto snapshot = logs->snapshot();
    Connection download(*service, "/api/logs/file/latest", "", true);
    REQUIRE(download.header().find("200 OK") != std::string::npos);
    REQUIRE(until([&] { return download.stream.bodyEntered(); }));
    fs::resize_file(directory / snapshot.file->filename, 0); // 外部截断模拟读取中途失效。
    download.stream.release();
    REQUIRE(until([&] { return download.done.load(); }));
    auto response = download.stream.output();
    auto body = response.substr(response.find("\r\n\r\n") + 4);
    CHECK(body.size() < snapshot.file->size);
    CHECK(body.find("\"success\"") == std::string::npos);
    CHECK(get("/api/logs/file/latest").find("200 OK") != std::string::npos);
}
TEST_CASE_FIXTURE(Fixture, "In-memory concurrent SSE admission never exceeds the configured quota") {
    std::vector<std::unique_ptr<Connection>> clients;
    for (int i = 0; i < 12; ++i) clients.emplace_back(std::make_unique<Connection>(*service, "/api/logs/stream"));
    size_t accepted = 0, rejected = 0;
    for (auto& client : clients) {
        auto header = client->header();
        accepted += header.find("200 OK") != std::string::npos;
        rejected += header.find("429 Too Many Requests") != std::string::npos;
    }
    CHECK(accepted == 4);
    CHECK(rejected == 8);
}
