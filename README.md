<!--
 Copyright (c) 2021 'Yang Luo, luoyang@sia.cn'

 This software is released under the MIT License.
 https://opensource.org/licenses/MIT
-->

<div align="center">
  <img src="./rocos-app.png" alt="" height="150">
  <h1>ROCOS-Application</h1>
  <blockquote>The Application of Robotics for ROCOS </blockquote>
</div>
:warning: Rocos-App is under development, not finished~ :thinking:

版本遵循 [SemVer 2.0.0](https://semver.org/lang/zh-CN/)。当前处于面向 `3.1.0` 的内部测试阶段，使用 `3.1.0-rc.N`，测试稳定并确认后正式发布 `3.1.0`。实际版本以 `CMakeLists.txt` 为准；维护规则见 [版本维护 skill](.agents/skills/rocos-version-maintenance/SKILL.md)。

## Introduction

ROCOS-App is the application of robotics, which is designed as the robot controller for ROCOS. 



## 工具管理与六点法标定

已接入 Viz 的真实工具管理、六点采集与标定计算。工具保存到 `robot.frames` 指定的 YAML，支持重命名、激活与重启恢复；标定计算后需明确保存并单独激活。操作顺序、坐标约定和接口衔接见 [六点法标定与工具管理](docs/tool_calibration.md)。

## Dependencies

- orocos-kdl
- ruckig
- nlopt
- trac_ik
- Eigen3
- protobuf
- grpc
- tinyFSM



## Usage

### CMake Presets

使用预设需要 **CMake 3.21+** 和 **Ninja**，以及项目原有的编译依赖。根目录的 `CMakePresets.json` 提供以下配置：

| 预设 | 构建类型 | 用途 |
| --- | --- | --- |
| `debug` | Debug | 日常开发、断点调试 |
| `release` | Release | 优化构建、性能测试 |
| `relwithdebinfo` | RelWithDebInfo | 优化构建并保留调试符号 |
| `minsizerel` | MinSizeRel | 优先减小产物体积 |

在仓库根目录执行，以下以 `debug` 为例，替换预设名即可切换：

```bash
cmake --list-presets
cmake --preset debug
cmake --build --preset debug --parallel 2
ctest --preset debug

# 只构建一个目标或运行指定测试
cmake --build --preset debug --target rocosAppMain --parallel 2
ctest --preset debug -R '^move_joint_test$'
```

测试前先构建相应测试目标；仅构建 `rocosAppMain` 不会生成测试程序。`--parallel` 可按机器资源调整，测试预设默认输出失败详情。硬件使能和机器人运动集成程序仍需在明确准备好的硬件/模拟器环境中手动运行。

各配置独立使用 `build/<预设名>/`，可执行文件、动态库、编译数据库分别位于其下的 `bin/`、`lib/`、`compile_commands.json`。CMake 不再将 YAML、URDF 或脚本复制到 `bin/`。

支持 CMake Presets 的 IDE 可直接选择上述配置。个人编译器路径或额外缓存变量可放入根目录 `CMakeUserPresets.json`，继承仓库预设；该文件已加入 Git 忽略规则。

未使用预设时，原有 CMake 3.15+ 的手动方式仍可使用：`cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug`，随后执行 `cmake --build build --parallel 2`。

### 启动配置

`rocosAppMain` 默认直接读取构建时源目录下的 [`config/config.yaml`](config/config.yaml)，可从任意工作目录启动。运行前先确认文件中的硬件、模型和 EtherCAT 实例配置适用于当前环境。也可显式指定配置文件：

```bash
./build/debug/bin/rocosAppMain --config=/path/to/rocos-app/config/config.yaml
# 未使用预设时：
./build/bin/rocosAppMain --config=/path/to/rocos-app/config/config.yaml
```

配置包括 HTTP 地址、端口、控制权 TTL，以及 `robot` 下的硬件 YAML、EtherCAT ID、URDF、基座/末端 link、关节绑定和负载文件路径；`scripts` 和 `web_root` 分别指定 Lua 脚本和 Web 静态资源目录。相对资源路径以所选 `config.yaml` 的所在目录为基准，绝对路径直接使用。模型资源统一放在根目录 `models/`，默认通过 `../models/talon/robot.urdf` 选择 Talon 模型；切换模型时需同步调整硬件和关节绑定，HTTP URDF 接口也使用同一模型路径。

配置优先级为显式 HTTP 命令行参数 > YAML > 内置默认值。例如 `--http_port=8081 --http_control_ttl_seconds=30` 只覆盖本次启动。配置文件不存在、格式错误、端口不在 1–65535 或 TTL 非正数时，会在初始化机器人前报错退出。迁移旧构建目录后，`bin/` 中的历史配置副本不会被主程序加载；部署到其他位置时使用 `--config` 指定实际文件。


## Related Projects



## Contributor

:bust_in_silhouette: **Yang Luo (luoyang@sia.cn)**

:framed_picture: Icon Designed by :**Yuhan Ying (1027649507@qq.com)**
