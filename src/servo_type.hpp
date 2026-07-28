// Copyright 2026, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

constexpr size_t MAX_DOF = 10;

#pragma pack(push, 1)   // 强制 1 字节对齐，消除 padding

enum class MotionMode : uint8_t {
  kJointPosition,
  kCartesianPosition,
  kNone
};

struct RobotState {     // Robot → Client, ~800+ bytes
  uint64_t message_id;              // 单调递增，兼作时间戳
  std::array<double, 16> flange_to_base;    // 法兰位姿（4x4 变换矩阵，row major）
  std::array<double, 16> tcp_to_base;    // TCP位姿

  std::array<double, MAX_DOF> tau;      // 关节扭矩
  std::array<double, MAX_DOF> load;     // 负载扭矩
  std::array<double, MAX_DOF> q;        // 关节位置
  std::array<double, MAX_DOF> dq;       // 关节速度

  std::array<bool, 41> errors;


  MotionMode mode;              // 机器人模式
  double control_command_success_rate; // 最近 100 个指令的成功率
};


struct MotionGeneratorCommand {   // 运动生成指令
  uint64_t message_id;
  std::array<double, 16> tcp_c;    // 笛卡尔位姿指令
  std::array<double, MAX_DOF> q_c;          // 关节位置指令

};

#pragma pack(pop)