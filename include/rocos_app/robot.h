// Copyright 2021, Yang Luo"
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

#ifndef ROCOS_APP_ROBOT_H
#define ROCOS_APP_ROBOT_H

#include <rocos_app/motion/motion_executor.h>
#include <rocos_app/motion/move_j_submission.h>
#include <rocos_app/motion/position_controller.h>
#include <rocos_app/motion/robot_fsm_gateway.h>
#include <rocos_app/motion/robot_motion_context.h>

#include <Eigen/Geometry>
#include <Eigen/QR>
#include <Eigen/StdVector>  //!< Eigen官网说明 https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html
#include <vector>

#include "DHParamsLoader.h"
#include "drive.h"
#include "dynamics.h"
#include "gripper.hpp"
#include "hardware_interface.h"
#include "kinematics.h"
#include "logger.h"

namespace rocos {
//! Class Robot
class Robot {
  friend class RobotServiceImpl;

  using JntArray = KDL::JntArray;
  using Frame = KDL::Frame;

 public:
  enum class WorkMode {
    Position = 0,
    EeAdmitTeach = 1,
    JntAdmitTeach = 2,
    JntImp = 3,
    CartImp = 4
  };

  explicit Robot(
      HardwareInterface *hw, const std::string &urdf_file_path = "robot.urdf",
      const std::string &base_link = "base_link",
      const std::string &tip =
          "link7");  // std::string yaml_path = "joint_impedance_control.yaml"

  ~Robot();

  std::string GetRobotState() const;

  int Start();

  int Pause();

  int Continue();

  int Stop();

  int SetEnabled();

  int SetDisabled();

  bool IsEnabled();

  bool IsDisabled();

  // 机器人状态机相关
  inline WorkMode getWorkMode() { return work_mode_; }
  bool setWorkMode(WorkMode mode);

  // 运动执行的状态进入/退出封装（基于 FSM，替代旧的
  // setRunState/is_running_motion）。 enterRunning：仅当处于 STOPPED
  // 才允许，原子地门控并转入 RUNNING；
  //               非 STOPPED 时事件被状态机忽略，返回 false（拒绝新运动）。
  // enterStopped：从 RUNNING/PAUSED/ERROR 回到 STOPPED，运动线程结束时调用。
  // isMotionRunning：是否有运动正占用机器人（FSM 处于 RUNNING）。
  bool enterRunning();
  void enterStopped();
  bool isMotionRunning() const;
  bool requestMotionStart();
  bool requestMotionPause();
  bool requestMotionContinue();
  bool requestMotionStop();
  bool notifyMotionError();
  void waitControlCycle();

  void setEnabled();

  inline void setJointEnabled(int id) { joints_[id]->setEnabled(); }

  void setDisabled();

  inline void setJointDisabled(int id) { joints_[id]->setDisabled(); }

  inline void setJointMode(int id, ModeOfOperation mode) {
    joints_[id]->setMode(mode);
  }

  inline int getJointNum() const { return jnt_num_; }

  inline std::string getJointName(int id) { return joints_[id]->getName(); }

  inline int getJointStatus(int id) { return joints_[id]->getDriveStateRPC(); }

  ///////////////////用户单位信息///////////////////////
  inline double getJointPosition(int id) { return joints_[id]->getPosition(); }

  inline double getJointVelocity(int id) { return joints_[id]->getVelocity(); }

  inline double getJointTorque(int id) { return joints_[id]->getTorque(); }

  inline double getJointLoadTorque(int id) {
    return joints_[id]->getLoadTorque();
  }
  // 获取滤波后的数据
  inline double getJointTorqueFilter(int id) {
    return joints_[id]->getSecondaryPositionInCnt();
  }
  inline double getJointSecondaryPositionInCnt(int id) {
    return joints_[id]->getSecondaryPositionInCnt();
  }

  inline void setJointPosition(int id, double pos) {
    joints_[id]->setPosition(pos);
  }

  inline void setJointVelocity(int id, double vel) {
    joints_[id]->setVelocity(vel);
  }

  inline void setJointTorque(int id, double tor) {
    joints_[id]->setTorque(tor);
  }

  /////////////////////获取原始信息//////////////////////////
  inline int32_t getJointPositionRaw(int id) {
    return joints_[id]->getPositionInCnt();
  }

  inline int32_t getJointVelocityRaw(int id) {
    return joints_[id]->getVelocityInCnt();
  }

  inline int16_t getJointTorqueRaw(int id) {
    return joints_[id]->getTorqueInCnt();
  }

  inline int16_t getJointLoadTorqueRaw(int id) {
    return joints_[id]->getLoadTorqueInCnt();
  }

  inline void setJointPositionRaw(int id, int32_t pos) {
    joints_[id]->setPositionInCnt(pos);
  }

  inline void setJointVelocityRaw(int id, int32_t vel) {
    joints_[id]->setVelocityInCnt(vel);
  }

  inline void setJointTorqueRaw(int id, int16_t tor) {
    joints_[id]->setTorqueInCnt(tor);
  }

  ////////////////////单位转换////////////////////////////
  inline double getJointCntPerUnit(int id) {
    return joints_[id]->getCntPerUnit();
  }

  inline double getJointTorquePerUnit(int id) {
    return joints_[id]->getTorquePerUnit();
  }

  inline double getJointRatio(int id) { return joints_[id]->getRatio(); }

  inline int32_t getJointPosZeroOffset(int id) {
    return joints_[id]->getPosZeroOffset();
  }

  inline std::string getJointUserUnitName(int id) {
    return joints_[id]->getUserUnitName();
  }

  inline void setJointCntPerUnit(int id, double cnt_per_unit) {
    joints_[id]->setCntPerUnit(cnt_per_unit);
  }

  inline void setJointTorquePerUnit(int id, double tor_per_unit) {
    joints_[id]->setTorquePerUnit(tor_per_unit);
  }

  inline void setJointRatio(int id, double ratio) {
    joints_[id]->setRatio(ratio);
  }

  inline void setJointPosZeroOffset(int id, int32_t offset) {
    joints_[id]->setPosZeroOffset(offset);
  }

  inline void setJointUserUnitName(int id, std::string name) {
    joints_[id]->setUserUnitName(name);
  }

  /// \brief 设置多关节速度约束
  /// \param max_vel 速度约束值
  inline void setJntVelLimits(std::vector<double> &max_vel) {}

  /// \brief 设置单关节速度约束
  /// \param id 关节ID
  /// \param max_vel 速度约束值
  inline void setJntVelLimit(int id, double max_vel) {}

  /// \brief 设置关节加速度约束
  /// \param max_acc 加速度约束值
  inline void setJntAccLimits(std::vector<double> &max_acc) {}

  /// \brief 获取多关节加速度约束
  /// \return 加速度约束值
  inline std::vector<double> getJntAccLimits() { return max_acc_; }

  /// \brief 设置单关节加速度约束
  /// \param id 关节ID
  /// \param max_acc 加速度约束值
  inline void setJntAccLimit(int id, double max_acc) {}

  /// \brief 获取单关节加速度约束
  /// \param id 关节ID
  /// \return 加速度约束值
  inline double getJntAccLimit(int id) { return max_acc_[id]; }

  /// \brief 设置多关节加加速约束
  /// \param max_jerk 多关节加加速约束值
  inline void setJntJerkLimits(std::vector<double> &max_jerk) {}

  /// \brief 获取多关节加加速约束
  /// \return 多关节加加速约束值
  inline std::vector<double> getJntJerkLimits() { return max_jerk_; }

  /// \brief 设置单关节加加速度约束
  /// \param id 关节id
  /// \param max_jerk 关节加加速约束值
  inline void setJntJerkLimit(int id, double max_jerk) {}

  /// \brief 获取单关节加加速度约束
  /// \param id 关节ID
  /// \return 关节加加速约束值
  inline double getJntJerkLimit(int id) { return max_jerk_[id]; }

  inline Frame getFlange() { }

  Frame getTool() { }

  Frame getObject() { }

 public:
  // 工具标定
  void tool_calibration(std::string frame) { }
  // 设置工具系
  void set_tool_frame(KDL::Frame &pose) { }
  // 设置工件系
  void set_object_frame(KDL::Frame &pose) { }

  void set_pose_frame(int id, KDL::Frame &pose_frame) { }

  Frame get_pose_frame(int id) { }
  Frame getPose_out() { return {}; }
  bool getErrorStateOfCal() { return false; }

 public:
  int JntToCart(const JntArray &q_in, Frame &p_out) {
    return kinematics_.JntToCart(q_in, p_out);
  }
  int CartToJnt(const JntArray &q_init, const Frame &p_in, JntArray &q_out) {
    return kinematics_.CartToJnt(q_init, p_in, q_out);
  }

 public:
  //! \brief 关节运动（支持位置和速度模式）
  //! \param q 各个关节位置
  //! \param speed 关节速度限制（leading axis）
  //! \param acceleration 关节加速度限制
  //! \param time 最短运行时间
  //! \param radius 过渡半径
  //! \param asynchronous 是否异步运行
  //! \return 错误标志位,成功返回0
  int MoveJ(JntArray q, double speed = 1.05, double acceleration = 1.4,
            double time = 0.0, double radius = 0.0, bool asynchronous = false);

  //! \brief 关节运动到指定笛卡尔位姿
  //! \param pose 位姿
  //! \param speed 关节速度限制（leading axis）
  //! \param acceleration 关节加速度限制
  //! \param time 最短运行时间
  //! \param radius 过渡半径
  //! \param asynchronous 是否异步运行
  //! \return 错误标志位,成功返回0
  int MoveJ_IK(Frame pose, double speed = 1.05, double acceleration = 1.4,
               double time = 0.0, double radius = 0.0,
               bool asynchronous = false);

  //! \brief 直线运动到指定位姿（支持位置和速度模式）
  //! \param pose 位姿
  //! \param speed 笛卡尔速度限制（leading axis）
  //! \param acceleration 笛卡尔加速度限制
  //! \param time 最短运行时间
  //! \param radius 过渡半径
  //! \param asynchronous 是否异步运行
  //! \param max_running_count MoveL规划失败重新尝试规划的最大次数
  //! \return 错误标志位,成功返回0
  int MoveL(Frame pose, double speed = 1.05, double acceleration = 1.4,
            double time = 0.0, double radius = 0.0, bool asynchronous = false,
            int max_running_count = 1);

  //! \brief 直线运动到关节空间指定位置
  //! \param q 关节位置
  //! \param speed 关节速度限制（leading axis）
  //! \param acceleration 关节加速度限制
  //! \param time 最短运行时间
  //! \param radius 过渡半径
  //! \param asynchronous 是否异步运行
  //! \return 错误标志位,成功返回0
  int MoveL_FK(JntArray q, double speed = 1.05, double acceleration = 1.4,
               double time = 0.0, double radius = 0.0,
               bool asynchronous = false);

  // TODO: Dragging要改成方向向量方式

 private:
  void initializeMotionExecutor();

 protected:
  HardwareInterface *hw_interface_{nullptr};
  std::vector<std::shared_ptr<Drive>> joints_;

  std::string urdf_file_path_;  // urdf文件路径
  std::string base_link_;
  std::string tip_;



  std::vector<double> max_vel_;
  std::vector<double> max_acc_;
  std::vector<double> max_jerk_;

  int jnt_num_;  // TODO: 关节数据要放到Model类中，删掉

  std::unique_ptr<motion::MotionSafetyGuard> motion_safety_guard_{nullptr};
  std::unique_ptr<motion::BasicRobotFsmGateway<Robot>> motion_fsm_gateway_{
      nullptr};
  std::unique_ptr<motion::PositionController> motion_position_controller_{
      nullptr};
  std::unique_ptr<motion::RobotMotionContext<Robot>> motion_context_{nullptr};
  std::unique_ptr<motion::MotionExecutor> motion_executor_{nullptr};
  motion::ModelProvider model_provider_;

  // sun
  Frame flange_;  //!< 法兰位置姿态
  Frame tool_;    //!< 工具位置姿态
  Frame object_;  //!< 工件位置姿态
  std::string cali_yaml_path_ = "/opt/rocos/yaml/calibration.yaml";
  YAML::Node yaml_node;

  WorkMode work_mode_{WorkMode::Position};  // 机器人当前模式，默认为位置模式

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

#endif  // ROCOS_APP_ROBOT_H
