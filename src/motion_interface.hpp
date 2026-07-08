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
#include "result.hpp"

namespace rocos {

class ModelInterface;

class MotionInterface {
 public:
  explicit MotionInterface(ModelInterface* model = nullptr) : model_(model) {}
  virtual ~MotionInterface() = default;

  virtual Result GenerateRef(Reference& ref_out) = 0;

  /// @brief 校验参数并初始化规划（合并 support + Reset）
  /// @return NoError=成功启动, PlanFinished=已到位无需运动, <0=错误
  virtual Result Reset() = 0;

  virtual bool supportsPause() const = 0;
  virtual bool supportsResume() const = 0;
  virtual bool supportsStop() const = 0;
  virtual Result support() const = 0;

  virtual Result Pause() = 0;
  virtual Result Resume() = 0;
  virtual Result Stop() = 0;
  virtual Result Update() = 0;

 protected:
  ModelInterface* model_{nullptr};
};

}  // namespace rocos