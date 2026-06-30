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
#include "logger.hpp"


// #include "dh_params_loader.hpp"
#include "model_interface.hpp"
#include "motion_interface.hpp"
#include "hardware_interface.hpp"

#include "executor.hpp"


namespace rocos {

class Robot {

  // enum class WorkMode {
  //   Position = 0,
  //   EeAdmitTeach = 1,
  //   JntAdmitTeach = 2,
  //   JntImp = 3,
  //   CartImp = 4
  // };

 public:
  explicit Robot();

  ~Robot();

  [[nodiscard]] std::string GetRobotState() const;

  Result Start();

  Result Pause();

  Result Continue();

  Result Stop();

  Result SetEnabled();

  Result SetDisabled();





  bool IsEnabled() const;

  bool IsDisabled() const;

  bool IsMotionRunning() const;

































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
  ModelInterface* model {nullptr};  //TODO: 机器人里可以保存子类指针，因为机器人会用到很多子类的功能，但是其他的模块接收接口指针即可
  HardwareInterface *hardware {nullptr};  //TODO: 同理，机器人里直接保存子类指针
  std::vector<MotionInterface*> motion {nullptr}; // movej, movel, movec
  std::vector<ControllerInterface*> controller {nullptr}; // position, CartAdmit, JntAdmit, JntImp, CartImp

  std::unique_ptr<Executor> executor {nullptr}; //执行器

 public:

  friend class RobotHttpServer;  // 允许 Server 直接访问 Robot 的私有/保护成员
                                 // TODO: ================================

 private:

  std::mutex mtx_;  // 互斥锁

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

  void on_fsm_servo();
};

}  // namespace rocos
