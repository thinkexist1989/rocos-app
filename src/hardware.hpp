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

#include "drive_interface.hpp"
#include "ecat_config.hpp"
#include "ft_sensor_interface.hpp"
#include "io_inteface.hpp"

namespace rocos {

struct Drive {
  std::string joint_name; //对应关节名称
  int id; //对应从站ID

};

struct FTSensor {
  int id;
};

struct IO {

};

class Hardware : public DriveInterface,
                 public FTSensorInterface,
                 public IOInteface {
 public:
  explicit Hardware(const std::string& urdf_file_path);
  ~Hardware();

  JntArray GetPosition() override;
  JntArray GetVelocity() override;
  JntArray GetTorque() override;
  JntArray GetLoadTorque() override;
  void SetPosition(const JntArray& q) override;
  void SetVelocity(const JntArray& q_dot) override;
  void SetTorque(const JntArray& tau) override;
  void SetMode(int8_t mode) override;
  void SetEnabled() override;
  void SetDisabled() override;
  double GetJointPosition(int32_t id) override;
  double GetJointVelocity(int32_t id) override;
  double GetJointTorque(int32_t id) override;
  double GetJointLoadTorque(int32_t id) override;
  void SetJointPosition(int32_t id, double pos) override;
  void SetJointVelocity(int32_t id, double vel) override;
  void SetJointTorque(int32_t id, double tau) override;
  void SetJointMode(int32_t id, int8_t mode) override;
  void SetJointEnabled(int32_t id) override;
  void SetJointDisabled(int32_t id) override;
  std::string getJointName(int32_t id) override;

  Wrench GetWrench() override;

  bool GetDigitalInput(int id, int channel) override;
  void SetDigitalOutput(int id, int channel, bool value) override;
  double GetAnalogInput(int id, int channel) override;
  void SetAnalogOutput(int id, int channel, double value) override;

 private:
  std::vector<Drive> drives_;
  std::vector<FTSensor> ft_sensors_;
  std::vector<IO> ios_;
};

}  // namespace rocos