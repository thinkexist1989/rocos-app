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

#include "robot.hpp"

#include <boost/sml.hpp>
#include <kdl_parser/kdl_parser.hpp>  // 用于将urdf文件解析为KDL::Tree

#include "hardware.hpp"

#include "move_joint.hpp"
#include "move_line.hpp"
#include "move_circle.hpp"
#include "move_jog.hpp"

#include "position_controller.hpp"
#include "joint_impedance_controller.hpp"
#include "cartesian_impedance_controller.hpp"



namespace {
// 状态定义
class ERROR_STATE {};  // 0 错误状态，任何状态发生错误都转到这个状态[初始状态]
class IDLE        {};  // 1 空闲状态，机器人传感器与执行器就绪，等待使能命令
class STOPPED     {};  // 2 停止状态，机器人上使能，不会动
class RUNNING     {};  // 3 运行状态，机器人正在执行运动
class PAUSED      {};  // 4 暂停状态，机器人暂停在当前位置，等待继续或停止命令
class SERVOING    {};  // 5 伺服状态，用于高速udp伺服指令发送

// class IDENTIFYING {};  // 动力学参数辨识状态，从STOPPED进入，辨识完成回到STOPPED

// 中间状态定义
class ENABLING  {}; // 6  正在上使能状态
class DISABLING {}; // 7  正在下使能状态
class STARTING  {}; // 8  正在启动状态，机器人正在启动
class STOPPING  {}; // 9  正在停止状态，机器人正在停止
class PAUSING   {}; // 10 正在暂停状态
class RESUMING  {}; // 11 正在恢复状态
class RESETTING {}; // 12 正在复位状态，机器人正在复位

// 机器人事件定义
struct EventRunning       {}; // 机器人开始运行事件
struct EventAtTarget      {}; // 机器人到达目标事件
struct EventStopped       {}; // 机器人已停止事件
struct EventEnabled       {}; // 机器人已上使能事件
struct EventDisabled      {}; // 机器人已下使能事件
struct EventErrorOccurred {}; // 发生错误
struct EventStartFailedReq {}; // 启动失败请求

// 指令事件定义
struct EventEnableReq  {}; // 上使能请求
struct EventDisableReq {}; // 下使能请求
struct EventStartReq   {}; // 启动请求
struct EventStopReq    {}; // 停止请求
struct EventPauseReq   {}; // 暂停请求
struct EventResumeReq  {}; // 继续请求
struct EventServoReq   {}; // 伺服请求
struct EventResetReq   {}; // 恢复请求


namespace sml = boost::sml;

const auto action_start = [](rocos::Robot& robot) { robot.on_fsm_start(); };
const auto action_run = [](rocos::Robot& robot) { robot.on_fsm_run(); }; //TODO：目前没有任何处理
const auto action_pause = [](rocos::Robot& robot) { robot.on_fsm_pause(); };
const auto action_resume = [](rocos::Robot& robot) { robot.on_fsm_resume(); };
const auto action_stop = [](rocos::Robot& robot) { robot.on_fsm_stop(); };
const auto action_reset = [](rocos::Robot& robot) { robot.on_fsm_reset(); };
const auto action_enable = [](rocos::Robot& robot) { robot.on_fsm_enable(); };
const auto action_disable = [](rocos::Robot& robot) { robot.on_fsm_disable(); };
const auto action_servo = [](rocos::Robot& robot) { robot.on_fsm_servo(); };
const auto action_error = [](rocos::Robot& robot) {}; //TODO: 进入错误状态时的必要处理

struct StateMachine {
  auto operator()() const noexcept {
    using namespace sml;
    return make_transition_table(
        // 初始化
        *state<class ERROR_STATE> + event<EventResetReq> = state<class RESETTING>,  // 0 ERROR_STATE->RESETTING
        state<class ERROR_STATE> + event<EventStopReq> = state<class STOPPING>,     //   ERROR_STATE->STOPPING
        state<class ERROR_STATE> + sml::on_entry<_> / action_error,                 //   ERROR_STATE->RESETTING

        state<class IDLE> + event<EventEnableReq> = state<class ENABLING>,   // 1 IDLE->ENABLING

        state<class STOPPED> + event<EventDisableReq> = state<class DISABLING>, // 2 STOPPED->DISABLING
        state<class STOPPED> + event<EventStartReq> = state<class STARTING>,    //   STOPPED->STARING
        state<class STOPPED> + event<EventServoReq> = state<class SERVOING>,    //   STOPPED->SERVOING

        state<class RUNNING> + sml::on_entry<_> / action_run,                   // 3 RUNNING
        state<class RUNNING> + event<EventStopped> = state<class STOPPED>,      //   RUNNING->STOPPED
        state<class RUNNING> + event<EventAtTarget> = state<class STOPPED>,     //   RUNNING->STOPPED

        state<class RUNNING> + event<EventPauseReq> = state<class PAUSING>,     //   RUNNING->PAUSING
        state<class RUNNING> + event<EventStopReq> = state<class STOPPING>,     //   RUNNING->STOPPING

        state<class PAUSED> + event<EventResumeReq> = state<class RESUMING>,    // 4 PAUSED->RESUMING
        state<class PAUSED> + event<EventStopReq> = state<class STOPPING>,      //   PAUSED->STOPPING

        state<class SERVOING> + sml::on_entry<_> / action_servo,                // 5 SERVOING
        state<class SERVOING> + event<EventStopReq> = state<class STOPPING>,    //   SERVOING->STOPPING

        state<class ENABLING> + sml::on_entry<_> / action_enable,               // 6 ENABLING
        state<class ENABLING> + event<EventEnabled> = state<class STOPPED>,     // 6 ENABLING->STOPPED

        state<class DISABLING> + sml::on_entry<_> / action_disable,             // 7 DISABLING
        state<class DISABLING> + event<EventDisabled> = state<class IDLE>,      //   DISABLING->IDLE


        state<class STARTING> + sml::on_entry<_> / action_start,                // 8 STARTING
        state<class STARTING> + event<EventAtTarget> = state<class STOPPED>,
        state<class STARTING> + event<EventStartFailedReq> = state<class STOPPED>,
        state<class STARTING> + event<EventRunning> = state<class RUNNING>,     //   STARTING->RUNNING

        state<class STOPPING> + sml::on_entry<_> / action_stop,                 // 9 STOPPING
        state<class STOPPING> + event<EventStopped> = state<class STOPPED>,     //   STOPPING->STOPPED

        state<class PAUSING> + sml::on_entry<_> / action_pause,                 // 10 PAUSING
        state<class PAUSING> + event<EventStopped> = state<class PAUSED>,       //   PAUSING->PAUSED

        state<class RESUMING> + sml::on_entry<_> / action_resume,               // 11 RESUMING
        state<class RESUMING> + event<EventRunning> = state<class RUNNING>,     //    RESUMING->RUNNING

        state<class RESETTING> + sml::on_entry<_> / action_reset,               // 12 RESETTING
        state<class RESETTING> + event<EventDisabled> = state<class IDLE>,      //    RESETTING->IDLE
        state<class RESETTING> + event<EventEnabled> =state<class STOPPED>,     //    RESETTING->STOPPED


        // ANY STATE JUMP TO ERROR_STATE
        state<class ERROR_STATE> + event<EventErrorOccurred> =  state<class ERROR_STATE>,  // 0
        state<class IDLE>        + event<EventErrorOccurred> =  state<class ERROR_STATE>,  // 1
        state<class STOPPED>     + event<EventErrorOccurred> =  state<class ERROR_STATE>,  // 2
        state<class RUNNING>     + event<EventErrorOccurred> =  state<class ERROR_STATE>,  // 3
        state<class PAUSED>      + event<EventErrorOccurred> =  state<class ERROR_STATE>,  // 4
        state<class SERVOING>    + event<EventErrorOccurred> =  state<class ERROR_STATE>,  // 5
        state<class ENABLING>    + event<EventErrorOccurred> =  state<class ERROR_STATE>,  // 6
        state<class DISABLING>   + event<EventErrorOccurred> =  state<class ERROR_STATE>,  // 7
        state<class STARTING>    + event<EventErrorOccurred> =  state<class ERROR_STATE>,  // 8
        state<class STOPPING>    + event<EventErrorOccurred> =  state<class ERROR_STATE>,  // 9
        state<class PAUSING>     + event<EventErrorOccurred> =  state<class ERROR_STATE>,  // 10
        state<class RESUMING>    + event<EventErrorOccurred> =  state<class ERROR_STATE>,  // 11
        state<class RESETTING>   + event<EventErrorOccurred> =  state<class ERROR_STATE>   // 12
    );
  }
};

}  // namespace

namespace rocos {

class Robot::Impl {
 public:
  explicit Impl(Robot& owner) : sm_{owner} {}

  // 运动线程与 HTTP 线程都会触发事件，必须串行化处理。
  // 用递归锁：action 回调（on_entry）内会再次调用 process_event（如 ENABLING
  // 进入时 on_fsm_enable 再触发 EventSuccess），属同线程重入，普通锁会死锁。
  template <typename Event>
  bool process_event(const Event& event) {
    std::lock_guard<std::recursive_mutex> lock(mtx_);
    return sm_.process_event(event);
  }

  template <typename TState>
  bool is(const TState& s) const {
    std::lock_guard<std::recursive_mutex> lock(mtx_);
    return sm_.is(s);
  }

 private:
  sml::sm<StateMachine> sm_;          // 状态机实例
  mutable std::recursive_mutex mtx_;  // 保护状态机的并发访问（允许同线程重入）
};

#pragma region 状态机action处理函数

void Robot::on_fsm_enable() {
  log_ptr_->info("机器人正在上使能中");

  // IsEnabled();

  if (randomBool()) {
    log_ptr_->info("机器人上使能成功，准备进入STOPPED状态");
    impl_->process_event(EventEnabled{});
  } else {
    log_ptr_->error("机器人上使能失败，准备进入ERROR_STATE状态");
    impl_->process_event(EventErrorOccurred{});
  }

}
void Robot::on_fsm_disable() {
  log_ptr_->info("机器人正在下使能中");

  // IsDisabled();



  if (randomBool()) {
    log_ptr_->info("机器人下使能成功，准备进入IDLE状态");
    impl_->process_event(EventDisabled{});
  } else {
    log_ptr_->error("机器人下使能失败，准备进入ERROR_STATE状态");
    impl_->process_event(EventErrorOccurred{});
  }
}
void Robot::on_fsm_start() {
 
  // 此处确认启动成功并转入 RUNNING。
  auto rc =data_ready_callback_();
  if (rc != Result::NoError) {
    
    if(rc== Result::PlanFinished)
      {
      log_ptr_->info("机器人已在目标位置，无需运动");
      impl_->process_event(EventAtTarget{});
      return;
      }

    log_ptr_->error("机器人启动失败，退回Stopped状态");
    impl_->process_event(EventStartFailedReq{});
    
      
    return;
  }
  impl_->process_event(EventRunning{});
  control_thread_ = std::thread(&Robot::controlLoop, this);

  
}
void Robot::on_fsm_run() {
  log_ptr_->info("Robot is running.");

  
}

void Robot::on_fsm_stop() {
  if (control_thread_.joinable()) control_thread_.join();
  impl_->process_event(EventStopped{});
}

void Robot::controlLoop() {
  // ToDo: 这里的while (IsRunning()) 改成while (IsControlActive())，只要处于需要控制周期的状态，就持续循环
  // bool Robot::IsControlActive() const {
//   return IsRunning()
//       || IsPausing()
//       || IsPaused()
//       || IsResuming()
//       || IsStopping();
// }然后while (IsRunning()) 改成while (IsControlActive())，只要处于需要控制周期的状态，就持续循环 
  while (IsRunning()) {
    
    RunCycle();
    
  }
}
void Robot::on_fsm_pause() {
  // PAUSING 进入：确认暂停并转入 PAUSED。
  impl_->process_event(EventStopped{});
}
void Robot::on_fsm_resume() {
  // CONTINUING 进入：确认继续并转回 RUNNING。
  impl_->process_event(EventRunning{});
}
void Robot::on_fsm_reset() {
  // log_ptr_->info("Robot is initializing...");
  log_ptr_->info("机器人正在Resetting，执行on_fsm_reset");


  // IsEnabled(); //TODO：要根据当前是否使能来确定状态

  if (randomBool()) {
    log_ptr_->info("机器人已经使能，准备进入STOPPED状态");
    impl_->process_event(EventEnabled{});  // 模拟初始化成功事件
  } else {
    log_ptr_->info("机器人未使能，准备进入IDLE状态");
    impl_->process_event(EventDisabled{});  // 模拟初始化失败事件
  }
}
void Robot::on_fsm_servo() {
  log_ptr_->info("Robot is servoing...");
}

#pragma endregion



Robot::Robot() : impl_(std::make_unique<Impl>(*this)) {
  log_ptr_ = Logger::getInstance("Robot");

  hardware = std::make_unique<Hardware>("./config/hardware.yaml", 0);

  executor = std::make_unique<Executor>();

  // motion = std::make_unique<MoveJoint>();
  // controller = std::make_unique<PositionController>(); //TODO: 默认加载位置控制器



  log_ptr_->info("机器人开始初始化");
  impl_->process_event(EventResetReq{});  // 进入初始化状态
}

Robot::~Robot() {
  // Delete logger pointer
  if (log_ptr_) {
    log_ptr_->flush();
    log_ptr_.reset();
  }

  // Delete FSM pack
  impl_.reset();
}

std::string Robot::GetStateString() const {
  if (impl_->is(sml::state<class IDLE>)) {
    return "IDLE";
  } else if (impl_->is(sml::state<class ENABLING>)) {
    return "ENABLING";
  } else if (impl_->is(sml::state<class DISABLING>)) {
    return "DISABLING";
  } else if (impl_->is(sml::state<class STARTING>)) {
    return "STARTING";
  } else if (impl_->is(sml::state<class STOPPING>)) {
    return "STOPPING";
  } else if (impl_->is(sml::state<class PAUSING>)) {
    return "PAUSING";
  } else if (impl_->is(sml::state<class RESUMING>)) {
    return "CONTINUING";
  } else if (impl_->is(sml::state<class RESETTING>)) {
    return "RESETTING";
  } else if (impl_->is(sml::state<class RUNNING>)) {
    return "RUNNING";
  } else if (impl_->is(sml::state<class PAUSED>)) {
    return "PAUSED";
  } else if (impl_->is(sml::state<class STOPPED>)) {
    return "STOPPED";
  } else if (impl_->is(sml::state<class SERVOING>)) {
    return "SERVOING";
  } else if (impl_->is(sml::state<class ERROR_STATE>)) {
    return "ERROR_STATE";
  } else {
    return "UNKNOWN_STATE";
  }
}

Result Robot::SetEnabled() {
  log_ptr_->info("收到上使能指令");
  if (!impl_->process_event(EventEnableReq{})) {
    log_ptr_->error("当前状态无法执行上使能指令");
    return Result::JointStateError;
  }

  return Result::NoError;
}

Result Robot::SetDisabled() {
  log_ptr_->info("收到下使能指令");
  if (!impl_->process_event(EventDisableReq{})) {
    log_ptr_->error("当前状态无法执行下使能指令");
    return Result::JointStateError;
  }

  return Result::NoError;
}





bool Robot::IsEnabled() const { return true; }

bool Robot::IsDisabled() const { return true; }

bool Robot::IsRunning() const {
  return impl_->is(sml::state<class RUNNING>) || impl_->is(sml::state<class STOPPING>) || impl_->is(sml::state<class PAUSING>) || impl_->is(sml::state<class RESUMING>);
}

void Robot::waitControlCycle() {}

void Robot::setEnabled() {}

void Robot::setDisabled() {}

/////// Motion Command /////////////

Result Robot::Pause() {

  return Result::NoError;
}

Result Robot::Resume() {

  return Result::NoError;
}

Result Robot::Stop() {
  return Result::NoError;
}

Result Robot::Start() {
  return Result::NoError;
}

// ============================================================================
// RunCycle：控制循环主函数，1000Hz 调用
// ============================================================================

void Robot::RunCycle() {
    if (!motion) return;

    // ① 推进 motion 状态
    Result r = motion->Update();

    if (r == Result::PlanFinished) {
        impl_->process_event(EventStopped{});   // RUNNING → STOPPED
        return;
    }
    if (static_cast<int>(r) < 0) {
        impl_->process_event(EventErrorOccurred{});
        return;
    }

    // ② 参考 → 指令 → 硬件
    if (executor) executor->Update();
}

// ============================================================================
// MoveJ：关节空间点到点，一次性下发跑完即停
// ============================================================================

Result Robot::MoveJ(const JntArray& q_goal,
                    double v_limit, double a_limit, double j_limit) {
    // if (!IsEnabled())   return Result::NotEnabled;
    // if (motion && IsRunning()) return Result::ConflictTaskRunning;

    data_ready_callback_= [this, q_goal, v_limit, a_limit, j_limit]() -> Result {
        const int n = getJointNum();
    JntArray q_start(static_cast<unsigned int>(n));
    for (int i = 0; i < n; ++i) q_start(i) = getJointPosition(i);

    auto new_motion = std::make_unique<MoveJoint>(
        q_start, q_goal, v_limit, a_limit, j_limit, /*dt=*/0.001);

    
    Result rc = new_motion->Reset();
    if (rc != Result::NoError) return rc;

    motion = std::move(new_motion);
    if (executor) 
    {executor->SwitchMotion(motion.get());
    return Result::NoError; 
    }
    else{
      log_ptr_->error("executor is nullptr, cannot switch motion");
      return Result::Fatal;
    }
    };
    if(impl_->process_event(EventStartReq{})) { 
        // STOPPED → RUNNING
    }
    else {
      if(impl_->is(sml::state<class IDLE>)) {
        log_ptr_->error("机器人处于IDLE状态，无法执行MoveJ指令，请先上使能");
        return Result::NotEnabled;
      }
      else if(impl_->is(sml::state<class ERROR_STATE>)) {
        log_ptr_->error("机器人处于ERROR_STATE状态，无法执行MoveJ指令，请先复位");
        return Result::Fatal;
      }
      else {
        log_ptr_->error("机器人当前状态无法执行MoveJ指令");
        return Result::ConflictTaskRunning;
      }

    }
    

    return Result::NoError;
}

// ============================================================================
// MoveL：笛卡尔直线，支持多工具坐标系
// ============================================================================

Result Robot::MoveL(const Frame& pose_goal,
                    const std::string& tool_name,
                    double v_limit, double a_limit, double j_limit) {

    data_ready_callback_ = [this, pose_goal, tool_name, v_limit, a_limit, j_limit]() -> Result {
        const int n = getJointNum();
        JntArray q_current(static_cast<unsigned int>(n));
        for (int i = 0; i < n; ++i) q_current(i) = getJointPosition(i);

        Frame pose_flange;
        if (model) model->ForwardKinematics(q_current, pose_flange);

        Frame T_tool;
        if (!tool_name.empty()) {
            auto it = tool_frames_.find(tool_name);
            if (it != tool_frames_.end()) T_tool = it->second;
        }

        auto new_motion = std::make_unique<MoveLine>(
            pose_flange, pose_goal * T_tool.Inverse(),
            v_limit, a_limit, j_limit, /*dt=*/0.001, model.get());

        Result rc = new_motion->Reset();
        if (rc != Result::NoError) return rc;

        motion = std::move(new_motion);
        if (executor) { executor->SwitchMotion(motion.get()); return Result::NoError; }
        log_ptr_->error("executor is nullptr");
        return Result::Fatal;
    };

    if (!impl_->process_event(EventStartReq{})) {
        if (impl_->is(sml::state<class IDLE>))  return Result::NotEnabled;
        if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
        return Result::ConflictTaskRunning;
    }
    return Result::NoError;
}

void Robot::SetToolFrame(const std::string& name, const Frame& T_tool) {
    tool_frames_[name] = T_tool;
}

Frame Robot::GetToolFrame(const std::string& name) const {
    auto it = tool_frames_.find(name);
    return (it != tool_frames_.end()) ? it->second : Frame();
}

// ============================================================================
// MoveC 圆心+角度
// ============================================================================

Result Robot::MoveC(const Frame& pose_start, const Frame& center_frame,
                    double theta, double v_limit, double a_limit, double j_limit) {

    data_ready_callback_ = [this, pose_start, center_frame, theta, v_limit, a_limit, j_limit]() -> Result {
        auto new_motion = std::make_unique<MoveCircle>(
            pose_start, center_frame, theta, v_limit, a_limit, j_limit, /*dt=*/0.001, model.get());

        Result rc = new_motion->Reset();
        if (rc != Result::NoError) return rc;

        motion = std::move(new_motion);
        if (executor) { executor->SwitchMotion(motion.get()); return Result::NoError; }
        log_ptr_->error("executor is nullptr");
        return Result::Fatal;
    };

    if (!impl_->process_event(EventStartReq{})) {
        if (impl_->is(sml::state<class IDLE>))  return Result::NotEnabled;
        if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
        return Result::ConflictTaskRunning;
    }
    return Result::NoError;
}

// ============================================================================
// MoveC 三点圆弧
// ============================================================================

Result Robot::MoveC(const Frame& pose_start, const Frame& pose_via,
                    const Frame& pose_goal,
                    double v_limit, double a_limit, double j_limit) {

    data_ready_callback_ = [this, pose_start, pose_via, pose_goal, v_limit, a_limit, j_limit]() -> Result {
        auto new_motion = std::make_unique<MoveCircle>(
            pose_start, pose_via, pose_goal, v_limit, a_limit, j_limit, /*dt=*/0.001, model.get());

        Result rc = new_motion->Reset();
        if (rc != Result::NoError) return rc;

        motion = std::move(new_motion);
        if (executor) { executor->SwitchMotion(motion.get()); return Result::NoError; }
        log_ptr_->error("executor is nullptr");
        return Result::Fatal;
    };

    if (!impl_->process_event(EventStartReq{})) {
        if (impl_->is(sml::state<class IDLE>))  return Result::NotEnabled;
        if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
        return Result::ConflictTaskRunning;
    }
    return Result::NoError;
}

// ============================================================================
// MoveJog：连续点动 + 活性保持
//
// 语义分派（与 motion_fsm_executor_design.md 场景 8/9/10/11 对齐）：
//   1) 当前 motion 已经是活跃 MoveJog  → 视为“喂饭”，转发 FeedJog()
//   2) 当前存在其他 motion 且正在跑    → 冲突拒绝
//   3) 其它情况                        → 新建 MoveJog，装入 executor，触发 FSM
// ============================================================================
Result Robot::MoveJogging(const JogVec& direction, double speed,
                          double timeout, double dir_threshold) {

    // ── 分支 1：已有活跃 MoveJog → 旁路喂饭 ──
    if (auto* jog = dynamic_cast<MoveJog*>(motion.get())) {
        const Result rc = jog->FeedJog(direction, speed);
        if (rc == Result::NoError) return rc;
    }

    // ── 分支 2：其他 motion 正在运行 → 拒绝 ──
    if (motion && IsRunning()) return Result::ConflictTaskRunning;

    // ── 分支 3：全新启动 ──
    data_ready_callback_ = [this, direction, speed, timeout, dir_threshold]() -> Result {
        auto new_jog = std::make_unique<MoveJog>(
            /*dt=*/0.001, timeout, model.get(), dir_threshold);

        const int n = getJointNum();
        JntArray q_current(static_cast<unsigned int>(n));
        for (int i = 0; i < n; ++i) q_current(i) = getJointPosition(i);
        new_jog->setInitialPosition(q_current);

        Result rc = new_jog->FeedJog(direction, speed);
        if (rc != Result::NoError) return rc;

        rc = new_jog->Reset();
        if (rc != Result::NoError) return rc;

        motion = std::move(new_jog);
        if (executor) { executor->SwitchMotion(motion.get()); return Result::NoError; }
        log_ptr_->error("executor is nullptr");
        return Result::Fatal;
    };

    if (!impl_->process_event(EventStartReq{})) {
        if (impl_->is(sml::state<class IDLE>))  return Result::NotEnabled;
        if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
        return Result::ConflictTaskRunning;
    }
    return Result::NoError;
}

}  // namespace rocos
