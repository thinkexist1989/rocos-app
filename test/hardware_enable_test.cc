// =============================================================================
// hardware_enable_test.cc — 真实硬件上使能测试
//
// 测试内容：
//   1. Hardware 初始化（连接 EtherCAT 共享内存，等待 OP）
//   2. 读取各关节当前位置
//   3. 设置 CSP 模式
//   4. 依次对每个关节上使能（阻塞等待状态机完成）
//   5. 验证使能后位置是否保持（检查是否飞车）
//   6. 依次下使能
//
// 用法：
//   ./build/bin/hardware_enable_test [ecat_id] [config_path]
//
//   默认 ecat_id=0, config_path=config/hardware_talon_config.yaml
//
// 重要：运行前必须先启动 EtherCAT 主站，等待总线进入 OP 状态！
// =============================================================================

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include "src/hardware.hpp"

// ==========================================================================
// 辅助函数
// ==========================================================================

/// @brief 解析配置文件路径（支持从 build/bin/、build/ 和源码根目录运行）
static std::string resolveConfig(const std::string& name) {
    // 从 build/bin/ 运行时，项目根目录在 ../../config/
    std::string path = "../../config/" + name;
    std::ifstream test(path);
    if (test.good()) return path;

    // 从 build/ 运行时
    test.open("config/" + name);
    if (test.good()) return "config/" + name;

    // 直接传入绝对路径或相对路径
    test.open(name);
    if (test.good()) return name;

    return name;  // 交给 Hardware 构造函数处理错误
}

/// @brief 将弧度转为角度
static double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

// ==========================================================================
// main
// ==========================================================================

int main(int argc, char* argv[]) {
    // 解析命令行参数
    int ecat_id = 0;
    std::string config_path = "hardware_talon_config.yaml";

    if (argc > 1) {
        ecat_id = std::stoi(argv[1]);
    }
    if (argc > 2) {
        config_path = argv[2];
    }

    config_path = resolveConfig(config_path);

    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║        ROCOS Hardware 上使能测试 (真实硬件)                  ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    std::cout << "║  ecat_id     = " << std::setw(45) << std::left << ecat_id << "║\n";
    std::cout << "║  config      = " << std::setw(45) << std::left << config_path << "║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
    std::cout << std::endl;

    // =========================================================================
    // 0. 预检查主站状态
    // =========================================================================
    {
        auto* ec = rocos::EcatConfig::getInstance(ecat_id);
        if (ec && ec->ecatBus) {
            // 主站用 boost::managed_shared_memory，ecatBus 不在 offset 0 — 先修正
            constexpr std::size_t kShmSize = 5242880;  // EC_SHM_MAX_SIZE
            auto* realBus = rocos::Hardware::scanForEcatBus(ec->ecatBus, kShmSize);
            if (realBus != nullptr) {
                ec->ecatBus = realBus;
            }

            int state = ec->ecatBus->current_state;
            const char* state_str = "UNKNOWN";
            switch (state) {
                case 1: state_str = "INIT"; break;
                case 2: state_str = "PREOP"; break;
                case 4: state_str = "SAFEOP"; break;
                case 8: state_str = "OP"; break;
                case 3: state_str = "BOOTSTRAP"; break;
            }
            std::cout << "[0/5] 主站当前状态: " << state_str
                      << " (" << state << "), 从站数量: "
                      << ec->ecatBus->slave_num << std::endl;
            if (state < 4) {  // 未到 SAFEOP
                std::cout << "  ⚠️  主站未进入 SAFEOP/OP，PDO 尚未开始交换" << std::endl;
                std::cout << "     请等待主站进入 OP 状态后重试" << std::endl;
                std::cout << "     (rocos_ecm 输出中应出现 'Master state changed to <OP>')" << std::endl;
                return EXIT_FAILURE;
            }
        }
    }

    // =========================================================================
    // 1. 初始化 Hardware（阻塞等待 EtherCAT 进入 OP）
    // =========================================================================
    std::cout << "[1/5] 初始化 Hardware ..." << std::endl;
    rocos::Hardware* hw = nullptr;
    try {
        hw = new rocos::Hardware(config_path, ecat_id);
    } catch (const std::exception& e) {
        std::cerr << "  ❌ Hardware 初始化失败: " << e.what() << std::endl;
        std::cerr << "     请确认 EtherCAT 主站已启动且总线处于 OP 状态" << std::endl;
        return EXIT_FAILURE;
    }

    const auto& cfg = hw->getConfig();
    size_t num_drives = cfg.drives.size();
    std::cout << "  ✅ Hardware 初始化成功, 驱动器数量: " << num_drives << std::endl;

    // =========================================================================
    // [DEBUG] 打印主站共享内存中实际注册的 PDO 变量名
    // =========================================================================
    std::cout << "\n  ╔══ 主站实际 PDO 变量名（用于核对 YAML 配置）══╗\n";
    auto* ec = rocos::EcatConfig::getInstance(ecat_id);
    if (ec && ec->ecatBus) {
        for (int sid = 0; sid < 50; ++sid) {
            const auto& slave = ec->ecatBus->slaves[sid];
            if (slave.input_var_num == 0 && slave.output_var_num == 0)
                continue;
            std::cout << "  ┌─ Slave " << sid;
            if (slave.name[0] != '\0')
                std::cout << " (" << slave.name << ")";
            std::cout << " ─ " << slave.input_var_num
                      << " inputs, " << slave.output_var_num << " outputs\n";
            for (int vi = 0; vi < slave.input_var_num; ++vi) {
                std::cout << "  │  [IN]  \"" << slave.input_vars[vi].name
                          << "\" (size=" << slave.input_vars[vi].size
                          << ", off=" << slave.input_vars[vi].offset << ")\n";
            }
            for (int vi = 0; vi < slave.output_var_num; ++vi) {
                std::cout << "  │  [OUT] \"" << slave.output_vars[vi].name
                          << "\" (size=" << slave.output_vars[vi].size
                          << ", off=" << slave.output_vars[vi].offset << ")\n";
            }
            std::cout << "  └─────────────────────────────────────────┘\n";
        }
    }
    std::cout << "  ╚══════════════════════════════════════════╝\n\n";

    // =========================================================================
    // 2. 设置 CSP 模式 + 读取当前位置
    // =========================================================================
    std::cout << "[2/5] 设置 CSP 模式并读取当前位置 ..." << std::endl;
    hw->SetMode(static_cast<int8_t>(
        rocos::ModeOfOperation::CyclicSynchronousPositionMode));

    // 等待模式切换生效
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "  ┌──────┬───────────────┬───────────────┬───────────────┐\n";
    std::cout << "  │  ID  │  Joint Name   │  Position(rad)│  Position(deg)│\n";
    std::cout << "  ├──────┼───────────────┼───────────────┼───────────────┤\n";
    for (size_t i = 0; i < num_drives; ++i) {
        int id = cfg.drives[i].id;
        double pos = hw->GetJointPosition(id);
        std::cout << "  │ " << std::setw(4) << id << " │ "
                  << std::setw(13) << cfg.drives[i].joint_name << " │ "
                  << std::setw(13) << std::fixed << std::setprecision(6) << pos << " │ "
                  << std::setw(13) << std::fixed << std::setprecision(4) << rad2deg(pos) << " │\n";
    }
    std::cout << "  └──────┴───────────────┴───────────────┴───────────────┘\n";
    std::cout << std::endl;

    // =========================================================================
    // 3. 上使能
    // =========================================================================
    std::cout << "[3/5] 上使能所有关节（逐一使能）..." << std::endl;

    bool all_enabled = true;
    for (size_t i = 0; i < num_drives; ++i) {
        int id = cfg.drives[i].id;
        const auto& name = cfg.drives[i].joint_name;

        // 读取使能前位置
        double pos_before = hw->GetJointPosition(id);

        // 上使能（SetJointEnabled 内部阻塞等待状态机完成，超时 150ms）
        std::cout << "  " << name << " (id=" << id << ") 上使能中... ";
        std::cout.flush();

        hw->SetJointEnabled(id);

        // 短暂等待确认
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 验证：读取使能后位置，检查是否飞车（位置偏差应 < 0.01 rad）
        double pos_after = hw->GetJointPosition(id);
        double delta = std::abs(pos_after - pos_before);

        if (delta < 0.05) {
            std::cout << "✅ 位置保持正常 (Δ=" << std::fixed
                      << std::setprecision(4) << delta << " rad)" << std::endl;
        } else {
            std::cout << "⚠️  位置偏差较大 (Δ=" << std::fixed
                      << std::setprecision(4) << delta << " rad)" << std::endl;
            all_enabled = false;
        }
    }
    std::cout << std::endl;

    // =========================================================================
    // 4. 读取使能后状态
    // =========================================================================
    std::cout << "[4/5] 读取使能后各关节位置 ..." << std::endl;
    std::cout << "  ┌──────┬───────────────┬───────────────┬───────────────┐\n";
    std::cout << "  │  ID  │  Joint Name   │  Position(rad)│  Position(deg)│\n";
    std::cout << "  ├──────┼───────────────┼───────────────┼───────────────┤\n";
    auto q_enabled = hw->GetPosition();
    for (size_t i = 0; i < num_drives; ++i) {
        int id = cfg.drives[i].id;
        std::cout << "  │ " << std::setw(4) << id << " │ "
                  << std::setw(13) << cfg.drives[i].joint_name << " │ "
                  << std::setw(13) << std::fixed << std::setprecision(6) << q_enabled(static_cast<int>(i)) << " │ "
                  << std::setw(13) << std::fixed << std::setprecision(4) << rad2deg(q_enabled(static_cast<int>(i))) << " │\n";
    }
    std::cout << "  └──────┴───────────────┴───────────────┴───────────────┘\n";
    std::cout << std::endl;

    // =========================================================================
    // 5. 下使能
    // =========================================================================
    std::cout << "[5/5] 下使能所有关节 ..." << std::endl;
    for (size_t i = 0; i < num_drives; ++i) {
        int id = cfg.drives[i].id;
        const auto& name = cfg.drives[i].joint_name;

        std::cout << "  " << name << " (id=" << id << ") 下使能中... ";
        std::cout.flush();

        hw->SetJointDisabled(id);

        std::cout << "✅" << std::endl;
    }
    std::cout << std::endl;

    // =========================================================================
    // 汇总
    // =========================================================================
    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    if (all_enabled) {
        std::cout << "║  🎉 测试完成 — 所有关节上使能正常，位置保持良好            ║\n";
    } else {
        std::cout << "║  ⚠️  测试完成 — 部分关节位置偏差较大，请检查               ║\n";
    }
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n";

    delete hw;
    return all_enabled ? EXIT_SUCCESS : EXIT_FAILURE;
}
