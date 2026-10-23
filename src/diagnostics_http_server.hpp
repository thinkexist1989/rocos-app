#pragma once
#include "log_manager.hpp"
#include "httplib.h"

namespace rocos {
struct DiagnosticsConfig {
    std::string host{"0.0.0.0"};
    int port{8082};
    size_t max_sse_clients{4}, max_downloads{1};
    void validate() const;
};
class DiagnosticsHttpServer {
public:
    explicit DiagnosticsHttpServer(LogManager& logs, DiagnosticsConfig config = {});
    ~DiagnosticsHttpServer();
    // port=0 仅供本地 HTTP 测试使用，返回实际绑定端口。
    int start();
    void stop();
private:
    void routes();
    LogManager& logs_;
    DiagnosticsConfig config_;
    friend struct DiagnosticsHttpTestAccess;
    class Transport : public httplib::Server { friend struct DiagnosticsHttpTestAccess; };
    Transport server_;
    std::thread thread_;
    std::shared_ptr<std::atomic<size_t>> sse_count_, download_count_;
    std::mutex subscriptions_mutex_;
    std::vector<std::weak_ptr<LogSubscription>> subscriptions_;
    std::atomic<bool> stopping_{false};
};
} // namespace rocos
