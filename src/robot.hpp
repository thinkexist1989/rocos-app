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

#include <map>
#include <string>
#include <thread>
#include <functional>

#include "types.hpp"
#include "result.hpp"
#include "logger.hpp"


// #include "dh_params_loader.hpp"
#include "model_interface.hpp"
#include "motion_interface.hpp"
#include "hardware_interface.hpp"

#include "executor.hpp"

#include <random>

namespace rocos {

class Robot {

 public:
  explicit Robot();

  ~Robot();

  [[nodiscard]] std::string GetStateString() const;

  // //! \brief 启动机器人运动，使机器人状态机进入 Running 状态
  // Result Start();

  //! \brief 暂停机器人运动，使机器人状态机进入 Pause 状态
  Result Pause();
  //! \brief 继续机器人运动，使机器人状态机进入 Running 状态
  Result Resume();
  //! \brief 停止机器人运动，使机器人状态机进入 Stopped 状态
  Result Stop();
  //! \brief 启动机器人运动，使机器人状态机进入 Running 状态
  Result Start();
  //! \brief 机器人上使能请求
  Result SetEnabled();
  //! \brief 机器人下使能请求
  Result SetDisabled();


  //! \brief 获取机器人当前是否上使能状态
  [[nodiscard]] bool IsEnabled() const;
  //！ \brief 获取机器人当前是否上使能状态
  [[nodiscard]] bool IsDisabled() const;
  //！ \brief 获取机器人当前是否正在运动
  [[nodiscard]] bool IsRunning() const;

  /// @brief 控制循环主函数，每周期调用 (1000Hz)
  void RunCycle();

  Result MoveJogging(const JogVec& direction, double speed,
                       double timeout = 0.1, double dir_threshold = 0.99);

  Result MoveJ(const JntArray& q_goal,
               double v_limit = 1.0, double a_limit = 2.0, double j_limit = 10.0);

  Result MoveL(const Frame& pose_goal,
               const std::string& tool_name = "",
               double v_limit = 1.0, double a_limit = 2.0, double j_limit = 10.0);

  void SetToolFrame(const std::string& name, const Frame& T_tool);
  Frame GetToolFrame(const std::string& name) const;

  // MoveC 圆心+角度
  Result MoveC(const Frame& pose_start, const Frame& center_frame, double theta,
               double v_limit = 1.0, double a_limit = 2.0, double j_limit = 10.0);

  // MoveC 三点圆弧（重载：第三个参数是 Frame）
  Result MoveC(const Frame& pose_start, const Frame& pose_via,
               const Frame& pose_goal,
               double v_limit = 1.0, double a_limit = 2.0, double j_limit = 10.0);































  void waitControlCycle();

  void setEnabled();

  void setDisabled();

  // inline void setJointMode(int id, ModeOfOperation mode) { }

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
  std::unique_ptr<ModelInterface> model {nullptr};  //TODO: 机器人里可以保存子类指针，因为机器人会用到很多子类的功能，但是其他的模块接收接口指针即可
  std::unique_ptr<HardwareInterface> hardware {nullptr};  //TODO: 同理，机器人里直接保存子类指针
  std::unique_ptr<MotionInterface> motion {nullptr}; // movej, movel, movec
  std::unique_ptr<ControllerInterface> controller {nullptr}; // position, CartAdmit, JntAdmit, JntImp, CartImp

  std::unique_ptr<Executor> executor {nullptr}; //执行器

 public:

  friend class RobotHttpServer;  // 允许 Server 直接访问 Robot 的私有/保护成员
                                 // TODO: ================================

 private:

  void controlLoop();                        // 控制线程函数
  std::thread control_thread_;               // 控制线程句柄
  std::mutex mtx_;
  std::map<std::string, Frame> tool_frames_;

  //// 机器人状态机封装
  struct Impl;
  std::unique_ptr<Impl> impl_;

  Logger::logger_ptr log_ptr_ = nullptr;

  std::function<Result> data_ready_callback_ = nullptr;  // 数据准备好回调函数（由硬件线程调用）

  //////////FSM Related function (INTERNAL) ///////////////
 public:
  void on_fsm_reset();

  void on_fsm_enable();

  void on_fsm_disable();

  void on_fsm_start();

  void on_fsm_run();

  void on_fsm_stop();

  void on_fsm_pause();

  void on_fsm_resume();

  void on_fsm_identify();

  void on_fsm_servo();


  //////////测试用///////////
  /// 使用 Bernoulli 分布随机生成一个 bool 值（概率各 50%）
  inline bool randomBool() noexcept {
    static std::mt19937 rng_{std::random_device{}()};
    static std::bernoulli_distribution dist_(0.5);
    return dist_(rng_);
  }



};



}  // namespace rocos
