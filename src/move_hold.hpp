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

#include "motion_interface.hpp"

namespace rocos {

/// @brief 保持位置不动的伪运动 —— 每帧始终输出构造时传入的关节角度。
///
/// 使用场景：状态机需要一个合法的 MotionInterface 实例，但实际上机器人
/// 应原地锁定。它不支持暂停、恢复或停止，调用对应接口将返回 FunctionNotSupported。
class MoveHold : public MotionInterface {
public:
    /// @param hold_pos 需要持续输出的关节位置，按值拷贝一份以保证所有权安全
    explicit MoveHold(const JntArray& hold_pos)
        : MotionInterface(nullptr), hold_pos_(hold_pos) {}

    ~MoveHold() override = default;

    // ─── MotionInterface ───

    Result ValidateParameters() const override { return Result::NoError; }

    Result Reset() override { return Result::NoError; }

    /// @brief 每周期调用，直接将保持位置写入 ref_out
    Result GenerateRef(Reference& ref_out) override {
        ref_out = hold_pos_;
        return Result::NoError;
    }

    // 不支持暂停、恢复、停止 —— 调用方不应对保持指令做流程干预
    bool CanPause()  const noexcept override { return false; }
    bool CanResume() const noexcept override { return false; }
    bool CanStop()   const noexcept override { return false; }

    Result Pause()  override { return Result::FunctionNotSupported; }
    Result Resume() override { return Result::FunctionNotSupported; }
    Result Stop()   override { return Result::FunctionNotSupported; }

    Result Update() override { return Result::NoError; }

private:
    JntArray hold_pos_;
};

}  // namespace rocos

