/**
 * @file udp_example.cpp
 * @brief UDP 伺服运动控制示例 —— 测试 1kHz 实时通信
 *
 * 用法: udp_example [host] [http_port] [udp_port]
 * 示例: udp_example localhost 8080 8081
 *
 * 工作流程:
 *   1. 通过 HTTP POST /api/robot/servo/start 启动伺服模式
 *   2. 通过 HTTP POST /api/robot/servo/mode 设置为关节空间模式
 *   3. 通过 UDP 以 1kHz 发送 MotionGeneratorCommand，接收 RobotState
 *   4. 每秒输出通信统计（实际频率、RTT、丢包率、服务器成功率）
 *   5. 收到 SIGINT 后通过 HTTP 停止伺服模式并安全退出
 *
 * 前置条件:
 *   - rocosAppMain 已启动（默认 --http_port=8080 --sim=true）
 *   - 机械臂已 enable: curl -X POST localhost:8080/api/robot/enable
 */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include <httplib.h>
#include <json.hpp>

#include "../src/servo_type.hpp"

// ============================================================================
// 全局标志 —— SIGINT 信号处理
// ============================================================================

namespace {

volatile sig_atomic_t g_running = 1;

void signalHandler(int /*signo*/) {
    g_running = 0;
}

}  // namespace

// ============================================================================
// main
// ============================================================================

int main(int argc, char* argv[]) {
    // ---- 解析命令行参数 ----
    const std::string host      = (argc >= 2) ? argv[1] : "localhost";
    const int         http_port = (argc >= 3) ? std::stoi(argv[2]) : 8080;
    const int         udp_port  = (argc >= 4) ? std::stoi(argv[3]) : 8081;

    std::cout << "╔══════════════════════════════════════════════╗\n";
    std::cout << "║   UDP 伺服运动控制示例 (1kHz)                 ║\n";
    std::cout << "╠══════════════════════════════════════════════╣\n";
    std::cout << "║ HTTP: " << host << ":" << http_port
              << "  |  UDP: " << host << ":" << udp_port << "\n";
    std::cout << "╚══════════════════════════════════════════════╝\n\n";

    // 注册信号处理
    if (signal(SIGINT, signalHandler) == SIG_ERR) {
        std::cerr << "[ERROR] 无法注册 SIGINT 信号处理器\n";
        return 1;
    }

    // ---- Step 1: 通过 HTTP 启动伺服模式 ----
    httplib::Client http(host, http_port);
    http.set_connection_timeout(5);


    // ---- Step 2: 设置伺服模式为关节空间 ----
    {
        nlohmann::json body;
        body["mode"] = "joint";
        auto res = http.Post("/api/robot/servo/mode",
                             body.dump(), "application/json");
        if (res) {
          auto json = nlohmann::json::parse(res->body);
          std::cout << "[INFO] ✓ 伺服模式: "
                    << json.value("message", "") << "\n";
        }
    }

    {
        nlohmann::json body;
        body["port"] = udp_port;
        auto res = http.Post("/api/robot/servo/start",
                             body.dump(), "application/json");

        if (!res) {
            std::cerr << "[ERROR] HTTP 连接失败 — rocosAppMain 是否已启动？\n";
            return 1;
        }

        auto json = nlohmann::json::parse(res->body);
        if (!json.value("success", false)) {
            std::cerr << "[ERROR] 启动伺服失败: "
                      << json.value("message", "unknown")
                      << " (code=" << json.value("code", -1) << ")\n";
            std::cerr << "[HINT]  请确认机械臂已 enable: "
                      << "curl -X POST " << host << ":" << http_port
                      << "/api/robot/enable\n";
            return 1;
        }
        std::cout << "[INFO] ✓ 伺服模式已启动, 端口="
                  << json["data"]["port"] << "\n";
    }

    // ---- Step 2: 设置伺服模式为关节空间 ----
    {
        nlohmann::json body;
        body["mode"] = "joint";
        auto res = http.Post("/api/robot/servo/mode",
                             body.dump(), "application/json");
        if (res) {
            auto json = nlohmann::json::parse(res->body);
            std::cout << "[INFO] ✓ 伺服模式: "
                      << json.value("message", "") << "\n";
        }
    }

    // ---- Step 3: 初始化 UDP socket ----
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "[ERROR] 无法创建 UDP socket\n";
        http.Post("/api/robot/servo/stop", "", "application/json");
        return 1;
    }

    // 解析服务器地址
    struct sockaddr_in server_addr {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port   = htons(static_cast<uint16_t>(udp_port));
    // 如果 host 是 "localhost"，解析为 127.0.0.1
    if (host == "localhost") {
        server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    } else if (inet_pton(AF_INET, host.c_str(),
                         &server_addr.sin_addr) != 1) {
        std::cerr << "[ERROR] 无法解析主机名: " << host << "\n";
        close(sockfd);
        http.Post("/api/robot/servo/stop", "", "application/json");
        return 1;
    }

    // connect UDP socket 以便使用 send/recv 代替 sendto/recvfrom
    if (connect(sockfd, reinterpret_cast<struct sockaddr*>(&server_addr),
                sizeof(server_addr)) < 0) {
        std::cerr << "[ERROR] UDP connect 失败\n";
        close(sockfd);
        http.Post("/api/robot/servo/stop", "", "application/json");
        return 1;
    }

    // 设置接收超时 500us —— 避免 recv 永久阻塞影响控制周期
    {
        struct timeval tv {};
        tv.tv_sec  = 0;
        tv.tv_usec = 500;
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char*>(&tv), sizeof(tv));
    }

    // ---- Step 4: 获取初始机器人状态 ----
    MotionGeneratorCommand cmd{};
    RobotState             state{};

    // 发送探测包以获取当前关节位置
    cmd.message_id = 1;
    send(sockfd, &cmd, sizeof(cmd), 0);

    {
        char    buf[sizeof(RobotState)];
        ssize_t n = recv(sockfd, buf, sizeof(buf), 0);
        if (n == static_cast<ssize_t>(sizeof(RobotState))) {
            std::memcpy(&state, buf, sizeof(RobotState));
            std::cout << "[INFO] ✓ 获取初始状态, server_msg_id="
                      << state.message_id << "\n";
            std::cout << "[INFO]   关节位置 q = [";
            for (int i = 0; i < 6; ++i) {
                std::cout << std::fixed << std::setprecision(4)
                          << state.q[i];
                if (i < 5) std::cout << ", ";
            }
            std::cout << "]\n";
        } else {
            std::cout << "[WARN] 首次探测未收到回复 (recv=" << n
                      << ")，将以零位为目标继续运行\n";
        }
    }

    // 以当前位置作为目标（hold position，仅测试通信链路）
    for (int i = 0; i < static_cast<int>(MAX_DOF); ++i) {
        cmd.q_c[i] = state.q[i];
    }

    // ---- Step 5: 1kHz 伺服控制循环 ----
    constexpr int    kTargetHz        = 1000;
    constexpr auto   kCycleDuration   = std::chrono::microseconds(1000);
    constexpr int    kReportInterval  = 1000;    // 每秒统计一次
    constexpr double kLateThresholdUs = 300.0;   // 超过此值视为周期滞后

    std::cout << "[INFO] 开始 1kHz 控制循环 ... (Ctrl+C 停止)\n";
    std::cout << "[INFO] 注：首次循环可能因冷缓存而偏慢，运行几秒后趋于稳定\n\n";

    // 统计变量
    int64_t total_loops   = 0;
    int64_t send_errors   = 0;
    int64_t recv_missed   = 0;   // recv 返回 ≤0 的次数
    int64_t recv_bad_size = 0;   // 收到但大小不匹配的畸形包
    int64_t late_cycles   = 0;
    double  sum_loop_us   = 0.0;
    double  last_rtt_us   = 0.0;
    double  min_rtt_us    = 1e9;
    double  max_rtt_us    = 0.0;

    uint64_t cmd_msg_id = 1;

    auto loop_start  = std::chrono::steady_clock::now();
    auto next_wakeup = loop_start;
    auto last_report = loop_start;
    int  loops_since_report = 0;

    while (g_running) {
        auto cycle_begin = std::chrono::steady_clock::now();

        // ---- 周期滞后检测 ----
        auto lag = std::chrono::duration_cast<std::chrono::microseconds>(
                       cycle_begin - next_wakeup).count();
        if (lag > static_cast<int64_t>(kLateThresholdUs)) {
            ++late_cycles;
            // 累积落后过多时重置调度基准，避免追赶导致更严重的滞后
            if (lag > static_cast<int64_t>(kCycleDuration.count() * 5)) {
                next_wakeup = cycle_begin;
            }
        }

        // ---- 发送 MotionGeneratorCommand ----
        cmd.message_id = ++cmd_msg_id;
        auto   send_t = std::chrono::steady_clock::now();
        ssize_t sent  = send(sockfd, &cmd, sizeof(cmd), 0);

        if (sent != static_cast<ssize_t>(sizeof(MotionGeneratorCommand))) {
            ++send_errors;
        }

        // ---- 接收 RobotState ----
        char    recv_buf[sizeof(RobotState)];
        ssize_t recvd = recv(sockfd, recv_buf, sizeof(recv_buf), 0);
        auto    recv_t = std::chrono::steady_clock::now();

        if (recvd == static_cast<ssize_t>(sizeof(RobotState))) {
            std::memcpy(&state, recv_buf, sizeof(RobotState));

            last_rtt_us = static_cast<double>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    recv_t - send_t).count());
            if (last_rtt_us < min_rtt_us) min_rtt_us = last_rtt_us;
            if (last_rtt_us > max_rtt_us) max_rtt_us = last_rtt_us;
        } else if (recvd <= 0) {
            ++recv_missed;
        } else {
            ++recv_bad_size;
        }

        // ---- 记录循环耗时 ----
        auto   cycle_end = std::chrono::steady_clock::now();
        double loop_us   = static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                cycle_end - cycle_begin).count());
        sum_loop_us += loop_us;

        ++total_loops;
        ++loops_since_report;

        // ---- 每秒输出统计 ----
        if (total_loops % kReportInterval == 0) {
            auto   now       = std::chrono::steady_clock::now();
            double elapsed_s = static_cast<double>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    now - last_report).count()) / 1e6;
            double actual_hz  = (elapsed_s > 0.0)
                                    ? static_cast<double>(loops_since_report) / elapsed_s
                                    : 0.0;
            double avg_loop   = (loops_since_report > 0)
                                    ? sum_loop_us / static_cast<double>(loops_since_report)
                                    : 0.0;
            double loss_rate  = (total_loops > 0)
                                    ? static_cast<double>(recv_missed) / static_cast<double>(total_loops) * 100.0
                                    : 0.0;
            double srv_success = state.control_command_success_rate * 100.0;

            std::cout << std::fixed << std::setprecision(1)
                      << "[STAT] "
                      << "频率: " << actual_hz << " Hz"
                      << " | 循环耗时: " << std::setprecision(0) << avg_loop << " us"
                      << " | RTT: " << last_rtt_us << " us"
                      << " | 丢包: " << std::setprecision(2) << loss_rate << "%"
                      << " | 服务器成功率: " << srv_success << "%"
                      << " | 滞后: " << late_cycles
                      << "\n";

            last_report         = now;
            loops_since_report  = 0;
            sum_loop_us         = 0.0;
        }

        // ---- 等待下一周期 ----
        next_wakeup += kCycleDuration;
        auto now = std::chrono::steady_clock::now();
        if (next_wakeup < now) {
            next_wakeup = now + kCycleDuration;
        }
        std::this_thread::sleep_until(next_wakeup);
    }

    // ---- Step 6: 安全退出 ----
    std::cout << "\n[INFO] 正在停止伺服控制 ...\n";

    // 关闭 UDP socket
    close(sockfd);

    {
        auto res = http.Post("/api/robot/servo/stop",
                             "", "application/json");
        if (res) {
            auto json = nlohmann::json::parse(res->body);
            std::cout << "[INFO] ✓ " << json.value("message", "") << "\n";
        } else {
            std::cout << "[WARN] HTTP 停止请求失败，伺服可能仍在运行\n";
        }
    }

    // ---- 最终报告 ----
    double total_elapsed = static_cast<double>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - loop_start).count()) / 1e6;
    double avg_hz = (total_elapsed > 0.0)
                        ? static_cast<double>(total_loops) / total_elapsed
                        : 0.0;
    double loss_rate_final = (total_loops > 0)
        ? static_cast<double>(recv_missed) / static_cast<double>(total_loops) * 100.0
        : 0.0;

    std::cout << "\n╔══════════════════════════════════════════════╗\n";
    std::cout << "║  最终统计报告                                  ║\n";
    std::cout << "╠══════════════════════════════════════════════╣\n"
              << "║ 运行时间:     " << std::setw(12) << std::fixed
              << std::setprecision(2) << total_elapsed << " 秒\n"
              << "║ 总循环数:     " << std::setw(12) << total_loops << "\n"
              << "║ 平均频率:     " << std::setw(12) << std::setprecision(1)
              << avg_hz << " Hz\n"
              << "║ 发送错误:     " << std::setw(12) << send_errors << "\n"
              << "║ 接收丢失:     " << std::setw(12) << recv_missed
              << " (" << std::setprecision(1) << loss_rate_final << "%)\n"
              << "║ 畸形包:       " << std::setw(12) << recv_bad_size << "\n"
              << "║ 周期滞后:     " << std::setw(12) << late_cycles << "\n"
              << "║ 最小 RTT:     " << std::setw(11) << std::setprecision(0)
              << min_rtt_us << " us\n"
              << "║ 最大 RTT:     " << std::setw(11) << max_rtt_us << " us\n"
              << "╚══════════════════════════════════════════════╝\n";

    // 达标判定：平均频率 ≥ 99% 目标即视为通过
    double score = (avg_hz / static_cast<double>(kTargetHz)) * 100.0;
    if (score >= 99.0) {
        std::cout << "\n✅ 1kHz 伺服通信测试通过! (频率达标率: "
                  << std::setprecision(1) << score << "%)\n";
    } else if (score >= 90.0) {
        std::cout << "\n⚠️  频率略低于目标 (达标率: "
                  << std::setprecision(1) << score
                  << "%)，建议检查系统负载或调度策略\n";
    } else {
        std::cout << "\n❌ 1kHz 伺服通信测试未达标 (达标率: "
                  << std::setprecision(1) << score << "%)\n";
    }

    return (score >= 90.0) ? 0 : 1;
}
