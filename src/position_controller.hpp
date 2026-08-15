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

#include "controller_interface.hpp"

namespace rocos {

class PositionController : public ControllerInterface {
public:
    PositionController() = default;
    ~PositionController() override;

    bool Reset() override;
    Result SetReady() override;
    Result SetHardware(HardwareInterface* hardware) override;
    Result SetModel(ModelInterface* model) override;
    Result GenerateCmd(const Reference& ref_in, JntArray& q_cmd) override;
    Result UpdateCmd(const JntArray& q_cmd) override;

private:
    Result ValidatePositionCommand(const JntArray& q_cmd);

    HardwareInterface* hardware_{nullptr};
    ModelInterface* model_{nullptr};
    bool mode_set_{false};  // 确保 CSP 模式只设置一次
    JntArray q_last_cmd_;        // 上一周期成功下发的指令位置（指令速度校验基线）
    bool has_last_cmd_{false};   // 是否已有上一周期指令基线
};

} // namespace rocos
