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

EtherCAT shared-memory access uses the header-only
[SharedMemoryConfig](include/rocos_app/shared_memory_config.hpp); building and
using the installed CMake package do not require `rocos_ecm`. At runtime, an
EtherCAT master must create shared memory with the same layout before the
application connects.



## Usage





## Related Projects



## Contributor

:bust_in_silhouette: **Yang Luo (luoyang@sia.cn)**

:framed_picture: Icon Designed by :**Yuhan Ying (1027649507@qq.com)**