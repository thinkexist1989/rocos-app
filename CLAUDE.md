# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 概述

ROCOS-App 是 ROCOS（机械臂控制平台）的 C++17 机器人控制器库。它驱动伺服硬件（EtherCAT/CANOpen等等）或仿真、执行运动学/动力学计算、规划运动，并通过 HTTP/JSON REST API 对外暴露全部功能。目标平台为 Linux（Ubuntu 22.04，可选搭配 ROS2 Humble）。

## 构建与测试

项目使用 CMake，采用源外构建（out-of-source）至 `build/`：

```bash
cmake -S . -B build
cmake --build build -j

# 运行全部测试
cd build && ctest

# 直接运行单个测试可执行文件（基于 doctest）
./build/bin/unit_test
./build/bin/unit_interval_motion_profile_test

# 过滤到某一个 doctest 用例
./build/bin/unit_test --test-case="Hardware"

# 运行控制器（默认仿真模式）
./build/bin/rocosAppMain --sim=true --http_port=8080
./build/bin/rocosAppMain --sim=false --id=0 --urdf=robot.urdf   # 真实硬件
```

安装前缀为 `/opt/rocos/app`（库、头文件、配置）。配置文件（`*.urdf`、`*.yaml`）在构建时复制到 `build/bin/`，使可执行文件在运行时能找到它们。

测试使用 [doctest](test/doctest.h)（单头文件，已内置）。新增测试用例放入 [test/unit_test.cc](test/unit_test.cc)，或在 [CMakeLists.txt](CMakeLists.txt) 的 `## Testing ##` 段落下接入新的可执行目标。

## 架构

代码组织为一组分层的共享库（定义于 [CMakeLists.txt](CMakeLists.txt)），每个库都是一个 `rocos::` CMake 目标。依赖自下而上流动——上层链接下层：

- **logger** —— 基于 spdlog 的日志封装（[src/logger.cc](src/logger.cpp)）。
- **hardware** —— EtherCAT 抽象层。`HardwareInterface`（[include/rocos_app/hardware_interface.hpp](src/hardware_interface.hpp)）是抽象基类；`Hardware` 是真实 EtherCAT 驱动，`HardwareSim` 是仿真器。要新增后端，继承 `HardwareInterface` 即可。`ethercat/` 头文件（command、status_word、control_word、drive_state、mode_of_operation）建模 CiA 402 伺服驱动状态机。
- **drive** —— 单轴控制：`Drive` 封装一个伺服，`DriveGuard` 运行实时控制循环，另含插补与机器人数学运算。
- **kinematics** —— 通过 orocos-kdl 实现正/逆运动学（FK/IK），IK 求解器使用 trac_ik 和 nlopt。
- **dynamics** —— 通过 orocos-kdl 实现刚体动力学。
- **robot** —— 顶层。`Robot` 类（[include/rocos_app/robot.hpp](src/robot.hpp)）是核心 API：持有 `HardwareInterface*`、运动学链（从 URDF 解析）以及运动指令（MoveJ/MoveL/MoveC/MoveP、拖拽示教、通过 `WorkMode` 实现的阻抗控制）。`JC_helper_*` 提供运动规划辅助（基于 Ruckig 的轨迹生成）,Robot类中包含一个Boost::sml状态机，其状态机流转图在docs/fsm.png中
- **robot_http_server** —— 提供 HTTP/JSON 接口，`Robot` 类的友元类，传入Robot，实现RESTful API通信。

`rocos_app` 是一个 INTERFACE 目标，打包 hardware+drive+kinematics+robot 供下游使用。`rocosAppMain`（[src/rocosAppMain.cc](src/rocosAppMain.cpp)）是入口：构造硬件（仿真或真实）、一个 `Robot` 和一个 `RobotHttpServer`，然后运行服务器。

### HTTP API

对外接口是 HTTP/JSON，**而非** gRPC —— gRPC 已被移除（见 [docs/dev_log.md](docs/dev_log.md)）。`RobotHttpServer`（[include/rocos_app/robot_http_server.h](src/robot_http_server.hpp)）是一个自包含类，仅依赖 `Robot*`，基于内置的 [cpp-httplib](3rdparty/httplib/) 和 [nlohmann/json](3rdparty/json/)（均为单头文件）构建。它注册了约 30 个路由（机器人状态、控制、运动、单轴/多轴、拖拽示教、标定），在线程池上执行异步运动，并返回可通过 `GET /api/move/status` 查询的 `task_id`。

新增或修改路由时，请同步更新设计文档与返回码表：
- [docs/http_api_design.md](docs/http_api_design.md) —— API 规范
- [docs/http_api_return_codes.md](docs/http_api_return_codes.md) —— 业务码参考

业务错误码为 4 位数字且分段：`0` 成功，`1xxx` 参数错误，`2xxx` 运动/规划，`3xxx` 机器人状态，`4xxx` 标定，`5xxx` 异步任务。所有响应均带有 `Access-Control-Allow-Origin: *`。

### 配置

[config/](config/) 存放控制器加载的运行时数据：`robot.urdf`（运动学模型）、`robotDH.yaml`（DH 参数，由 `DHParamsLoader` 解析）、`joint_impedance_control.yaml`、`calibration.yaml`，以及由 API 提供的 `models/` 网格文件。

### 第三方依赖

[3rdparty/](3rdparty/) 以 `add_subdirectory` 方式内置了全部依赖：nlopt、trac_ik、kdl_parser、ruckig、plog、spdlog、gflags、sml（状态机）、gripper，外加头文件库 httplib 和 json。通过 `find_package` 查找的系统依赖：Boost、yaml-cpp、Eigen3、OpenSSL、orocos_kdl。

## 编码规范

摘自 [.github/copilot-instructions.md](.github/copilot-instructions.md)——新代码请遵循：

- **命名**：内部函数 `lowerCamelCase`，对外调用/API 函数 `PascalCase`，类/结构体 `PascalCase`，常量/宏 `UPPER_SNAKE_CASE`，成员变量 `lower_case_`（结尾下划线）。
- **风格**：4 空格缩进，禁用 Tab，遵循 Clang-Format 风格。注释说明*为什么*而非*做了什么*——仅在逻辑复杂、Hack 代码或算法关键点处添加。
- **仅限 C++17** —— 禁止 C++20 特性（concepts、ranges），也禁止 C++98 老旧语法。
- **内存**：禁止裸 `new`/`delete`；使用 RAII，优先 `std::unique_ptr`，仅在确有共享所有权时使用 `std::shared_ptr`。
- 虚函数重写标注 `override`，不修改状态的成员函数标注 `const`，不抛异常的函数标注 `noexcept`。传递大对象用 `const T&`。
- 禁止忽略返回值；对入参指针操作前用 `if (ptr == nullptr)` 防御性检查；通过项目的 `LOG_ERROR` 宏记录错误。
- 解释和注释用中文，技术术语用英文。生成核心业务函数时，应附带单元测试骨架（注意：copilot 规范要求 GTest 骨架，但现有测试套件使用的是 doctest）。
