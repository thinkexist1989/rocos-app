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

版本遵循 [SemVer 2.0.0](https://semver.org/lang/zh-CN/)。当前处于面向 `3.0.1` 的内部测试阶段，使用 `3.0.1-rc.N`，测试稳定后正式发布 `3.0.1`。实际版本以 `CMakeLists.txt` 为准；维护规则见 [版本维护 skill](skills/rocos-version-maintenance/SKILL.md)。

## Introduction

ROCOS-App is the application of robotics, which is designed as the robot controller for ROCOS. 



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

各配置独立使用 `build/<预设名>/`，可执行文件、动态库、编译数据库分别位于其下的 `bin/`、`lib/`、`compile_commands.json`。运行程序前先配置所需硬件或模拟器，并进入对应的 `build/<预设名>/bin/`，使 `robot.urdf`、`scripts/` 等相对路径正确解析。

支持 CMake Presets 的 IDE 可直接选择上述配置。个人编译器路径或额外缓存变量可放入根目录 `CMakeUserPresets.json`，继承仓库预设；该文件已加入 Git 忽略规则。

未使用预设时，原有 CMake 3.15+ 的手动方式仍可使用：`cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug`，随后执行 `cmake --build build --parallel 2`。

## Related Projects



## Contributor

:bust_in_silhouette: **Yang Luo (luoyang@sia.cn)**

:framed_picture: Icon Designed by :**Yuhan Ying (1027649507@qq.com)**
