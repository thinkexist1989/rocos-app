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

namespace rocos {

class DriveInterface {
 public:
  virtual ~DriveInterface() = default;

  /////////////同时设置/获取所有关节////////////////
  virtual JntArray GetPosition() = 0;
  virtual JntArray GetVelocity() = 0;
  virtual JntArray GetTorque() = 0;
  virtual JntArray GetLoadTorque() = 0;

  virtual void SetPosition(const JntArray& q) = 0;
  virtual void SetVelocity(const JntArray& q_dot) = 0;
  virtual void SetTorque(const JntArray& tau) = 0;

  virtual void SetMode(int8_t mode) = 0;

  virtual void SetEnabled() = 0;
  virtual void SetDisabled() = 0;

  virtual JntState GetState() = 0; //TODO: 现在缺Drive状态反馈，只要有一个关节Error，就返回Error，只要有一个Disable就返回Disabled，全部Enabled

  /////////////单独设置/获取每个关节/////////////////
  virtual double GetJointPosition(int32_t id) = 0;
  virtual double GetJointVelocity(int32_t id) = 0;
  virtual double GetJointTorque(int32_t id) = 0;
  virtual double GetJointLoadTorque(int32_t id) = 0;

  virtual void SetJointPosition(int32_t id, double pos) = 0;
  virtual void SetJointVelocity(int32_t id, double vel) = 0;
  virtual void SetJointTorque(int32_t id, double tau) = 0;

  virtual void SetJointMode(int32_t id, int8_t mode) = 0;

  virtual void SetJointEnabled(int32_t id) = 0;
  virtual void SetJointDisabled(int32_t id) = 0;

  virtual JntState GetJointState(int32_t id) = 0;

  virtual std::string getJointName(int32_t id) = 0;
};

}  // namespace rocos