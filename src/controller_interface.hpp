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

#include "types.hpp"

#include "motion_interface.hpp"

namespace rocos {

class ControllerInterface {
 public:
  virtual ~ControllerInterface() = default;

  virtual Result SetContext(HardwareInterface* hardware) = 0;
  virtual Result SetModel(ModelInterface* model) = 0;
  virtual Result GenerateCmd(const Ref& ref_in, JntArray& q_cmd) = 0; //TODO: 更新Cmd但是不发送
  virtual Result UpdateCmd(const JntArray& q_cmd) = 0; //TODO: 发送Cmd
};

}  // namespace rocos