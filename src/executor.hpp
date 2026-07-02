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
#include "hardware_interface.hpp"
#include "motion_interface.hpp"
#include "result.hpp"
#include "types.hpp"

#include <mutex>

namespace rocos {

class Executor {
 public:
  explicit Executor(MotionInterface* motion, ControllerInterface* controller,
                    HardwareInterface* hardware);

  virtual ~Executor();

  Result Update();
  bool SwitchController(ControllerInterface* new_contorller);
  bool SwitchHardware(HardwareInterface* new_hardware);
  bool SwitchMotion(MotionInterface* new_motion);

 private:

  //TODO: 之所以保存接口的裸指针，是因为Executor只是拼装作用，不管理指针的生命周期
  MotionInterface* motion_;
  ControllerInterface* controller_;
  HardwareInterface* hardware_;

  std::mutex mtx_;
};

}  // namespace rocos
