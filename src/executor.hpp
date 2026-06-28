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

#include "dh_params_loader.hpp"
#include "dynamics.hpp"
#include "hardware_interface.hpp"
#include "kinematics.hpp"
#include "logger.hpp"

namespace rocos {
//! Class Robot
class Executor {

 public:
  explicit Executor();  // std::string yaml_path = "joint_impedance_control.yaml"

  ~Executor();

  std::string GetRobotState() const;

  int Start();

  int Pause();

  int Continue();

  int Stop();

  int SetEnabled();

  int SetDisabled();

  bool IsEnabled();

  bool IsDisabled();

  bool isMotionRunning() const;

  void waitControlCycle();

  void setEnabled();

  inline void setJointEnabled(int id) { }

  void setDisabled();

  inline void setJointDisabled(int id) { }

  inline void setJointMode(int id, ModeOfOperation mode) { }

  inline int getJointNum() const { }

  inline std::string getJointName(int id) {  }

  inline int getJointStatus(int id) { }

  ///////////////////用户单位信息///////////////////////
  inline double getJointPosition(int id) { }

  inline double getJointVelocity(int id) { }

  inline double getJointTorque(int id) { }

  inline double getJointLoadTorque(int id) { }
  // 获取滤波后的数据
  inline double getJointTorqueFilter(int id) { }
  inline double getJointSecondaryPositionInCnt(int id) { }

  inline void setJointPosition(int id, double pos) { }

  inline void setJointVelocity(int id, double vel) { }

  inline void setJointTorque(int id, double tor) { }

  inline Frame getFlange() { return Frame();}

  Frame getTool() { return Frame(); }

  Frame getObject() { return Frame(); }

 public:
  int JntToCart(const JntArray &q_in, Frame &p_out) {
    return kinematics_.JntToCart(q_in, p_out);
  }
  int CartToJnt(const JntArray &q_init, const Frame &p_in, JntArray &q_out) {
    return kinematics_.CartToJnt(q_init, p_in, q_out);
  }

 private:
  void initializeMotionExecutor();

 protected:
  HardwareInterface *hw_interface_{nullptr};

  // std::unique_ptr<motion::MotionSafetyGuard> motion_safety_guard_{nullptr};

  // std::unique_ptr<motion::PositionController> motion_position_controller_{nullptr};
  // std::unique_ptr<motion::RobotMotionContext<Robot>> motion_context_{nullptr};
  // std::unique_ptr<motion::MotionExecutor> motion_executor_{nullptr};
  // motion::ModelProvider model_provider_;


  std::string cali_yaml_path_ = "/opt/rocos/yaml/calibration.yaml";

  // YAML::Node yaml_node;


 public:
  Kinematics kinematics_;
  Dynamics dynamics_;

  friend class RobotHttpServer;  // 允许 Server 直接访问 Robot 的私有/保护成员
                                 // TODO: ================================

 private:

  std::mutex mtx;  // 互斥锁
  DHParamsLoader loader;

  //// 机器人状态机封装
  struct Impl;
  std::unique_ptr<Impl> impl_;

  Logger::logger_ptr log_ptr_ = nullptr;

  //////////FSM Related function (INTERNAL) ///////////////
 public:
  void on_fsm_reset();

  void on_fsm_enable();

  void on_fsm_disable();

  void on_fsm_start();

  void on_fsm_run();

  void on_fsm_stop();

  void on_fsm_pause();

  void on_fsm_continue();

  void on_fsm_identify();
};

}  // namespace rocos
