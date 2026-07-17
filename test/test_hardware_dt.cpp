// test_hardware_dt.cpp
// ---------------------------------------------------------------------------
// 测试：用真实 rocos::Hardware 读取共享内存中的 EcatBus::dt（uint32_t，单位 us）
//
// 路径:
//   mujoco 写入 ecatBus->dt
//     → 共享内存 /ecm0
//     → Hardware::GetDt() → getDt() → ecatBus->dt
//
// 编译: 主 CMake 目标 test_hardware_dt
// 运行: 先起 mujoco，再在 build/bin 下:
//   ./test_hardware_dt hardware_talon_config.yaml 0
// ---------------------------------------------------------------------------

#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "src/hardware.hpp"

static void usage(const char* prog) {
    std::cout
        << "用法: " << prog << " [hardware.yaml] [ecat_id]\n"
        << "  默认: hardware_talon_config.yaml  0  (在 build/bin 下运行)\n"
        << "\n"
        << "请先启动仿真，例如:\n"
        << "  rocos_mujoco_sim --model talon --cycle-time 1000\n"
        << "再运行:\n"
        << "  ./test_hardware_dt hardware_talon_config.yaml 0\n";
}

int main(int argc, char* argv[]) {
    if (argc >= 2 &&
        (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
        usage(argv[0]);
        return 0;
    }

    // 在 build/bin 下默认用拷贝过来的 yaml；也可传绝对/相对路径
    const std::string yaml =
        (argc >= 2) ? argv[1] : "hardware_talon_config.yaml";
    const int ecat_id = (argc >= 3) ? std::atoi(argv[2]) : 0;

    std::cout << "=== Hardware::GetDt() 测试 (dt 类型: uint32_t, 单位: us) ===\n"
              << "yaml=" << yaml << "  ecat_id=" << ecat_id << "\n\n";

    std::cout << "[1] 等待共享内存 /ecm" << ecat_id << " ...\n";
    for (int i = 0; i < 100; ++i) {
        const std::string path = "/dev/shm/ecm" + std::to_string(ecat_id);
        if (access(path.c_str(), R_OK) == 0) {
            break;
        }
        if (i == 99) {
            std::cerr << "[FAIL] 未找到共享内存，请先启动 mujoco 仿真。\n";
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    try {
        std::cout << "[2] 构造 rocos::Hardware ...\n";
        rocos::Hardware hw(yaml, ecat_id);

        std::cout << "[3] 读取 GetDt()（连续 5 次）:\n";
        for (int k = 0; k < 5; ++k) {
            const uint32_t dt = hw.GetDt();
            const double hz =
                (dt > 0) ? (1e6 / static_cast<double>(dt)) : 0.0;
            std::cout << "    [" << k << "] GetDt() = " << dt << " us"
                      << "  (" << hz << " Hz)"
                      << "  [uint32_t]\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
        }

        const uint32_t dt = hw.GetDt();
        if (dt == 0) {
            std::cerr << "\n[FAIL] GetDt()==0，主站可能未写入 dt。\n";
            return 1;
        }

        std::cout << "\n[OK] 读到 dt = " << dt << " us (uint32_t)\n"
                  << "     请与仿真 --cycle-time 对照是否一致。\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] " << e.what() << "\n";
        return 1;
    }
}
