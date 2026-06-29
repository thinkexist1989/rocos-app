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

#define EPS 1e-7

namespace {
// 状态定义
class IDLE {};         // 空闲状态，机器人传感器与执行器就绪，等待使能命令
class STOPPED {};      // 停止状态，机器人上使能，不会动
class PAUSED {};       // 暂停状态，机器人暂停在当前位置，等待继续或停止命令
class RUNNING {};      // 运行状态，机器人正在执行运动
class ERROR_STATE {};  // 错误状态，任何状态发生错误都转到这个状态[初始状态]
class SERVOING {};     // 伺服状态，用于高速udp伺服指令发送
class IDENTIFYING {};  // 动力学参数辨识状态，从STOPPED进入，辨识完成回到STOPPED

// 中间状态定义
class RESETTING {};
class ENABLING {};
class DISABLING {};
class STARTING {};  // 启动状态，机器人正在启动
class STOPPING {};
class PAUSING {};
class CONTINUING {};

// 事件定义
struct EventResetReq {};       // 恢复请求
struct EventEnableReq {};      // 上使能请求
struct EventDisableReq {};     // 下使能请求
struct EventSuccess {};        // 成功事件
struct EventStartReq {};       // 启动请求
struct EventStopReq {};        // 停止请求
struct EventPauseReq {};       // 暂停请求
struct EventContinueReq {};    // 继续请求
struct EventErrorOccurred {};  // 发生错误
struct EventServoReq {};       // 伺服请求
struct EventIdentifyReq {};    // 动力学参数辨识请求
struct EventIsEnabled {};  // TODO: 检查使能状态事件(临时兼容性，要删除)

namespace sml = boost::sml;

const auto action_start = [](rocos::Robot& robot) { robot.on_fsm_start(); };
const auto action_run = [](rocos::Robot& robot) { robot.on_fsm_run(); };
const auto action_pause = [](rocos::Robot& robot) { robot.on_fsm_pause(); };
const auto action_continue = [](rocos::Robot& robot) {
  robot.on_fsm_continue();
};
const auto action_stop = [](rocos::Robot& robot) { robot.on_fsm_stop(); };
const auto action_reset = [](rocos::Robot& robot) { robot.on_fsm_reset(); };
const auto action_enable = [](rocos::Robot& robot) { robot.on_fsm_enable(); };
const auto action_disable = [](rocos::Robot& robot) { robot.on_fsm_disable(); };
const auto action_identify = [](rocos::Robot& robot) {
  robot.on_fsm_identify();
};

const auto action_servo = [](rocos::Robot& robot) { robot.on_fsm_servo(); };

struct StateMachine {
  auto operator()() const noexcept {
    using namespace sml;
    return make_transition_table(
        // 初始化
        *state<class ERROR_STATE> + event<EventResetReq> = state<class RESETTING>,
        state<class RESETTING> + sml::on_entry<_> / action_reset,
        state<class RESETTING> + event<EventSuccess> = state<class IDLE>,
        state<class RESETTING> + event<EventIsEnabled> =state<class STOPPED>,

        state<class IDLE> + event<EventEnableReq> = state<class ENABLING>,
        state<class ENABLING> + sml::on_entry<_> / action_enable,
        state<class ENABLING> + event<EventSuccess> = state<class STOPPED>,

        state<class STOPPED> + event<EventDisableReq> = state<class DISABLING>,
        state<class DISABLING> + sml::on_entry<_> / action_disable,
        state<class DISABLING> + event<EventSuccess> = state<class IDLE>,

        state<class STOPPED> + event<EventStartReq> = state<class STARTING>,
        state<class STARTING> + sml::on_entry<_> / action_start,
        state<class STARTING> + event<EventSuccess> = state<class RUNNING>,

        state<class STOPPED> + event<EventServoReq> = state<class SERVOING>,
        state<class SERVOING> + sml::on_entry<_> / action_servo,
        state<class SERVOING> + event<EventStopReq> = state<class STOPPING>,

        state<class STOPPED> + event<EventIdentifyReq> =
            state<class IDENTIFYING>,
        state<class IDENTIFYING> + sml::on_entry<_> / action_identify,
        state<class IDENTIFYING> + event<EventSuccess> = state<class STOPPED>,

        state<class RUNNING> + sml::on_entry<_> / action_run,
        state<class RUNNING> + event<EventPauseReq> = state<class PAUSING>,

        state<class PAUSING> + sml::on_entry<_> / action_pause,
        state<class PAUSING> + event<EventSuccess> =
            state<class PAUSED>,  // 中间状态直接跳转

        state<class PAUSED> + event<EventContinueReq> = state<class CONTINUING>,
        state<class CONTINUING> + sml::on_entry<_> / action_continue,
        state<class CONTINUING> + event<EventSuccess> =
            state<class RUNNING>,  // 中间状态直接跳转

        state<class PAUSED> + event<EventStopReq> = state<class STOPPING>,
        state<class RUNNING> + event<EventStopReq> = state<class STOPPING>,
        state<class ERROR_STATE> + event<EventStopReq> = state<class STOPPING>,
        state<class STOPPING> + sml::on_entry<_> / action_stop,
        state<class STOPPING> + event<EventSuccess> = state<class STOPPED>,

        // ANY STATE JUMP TO ERROR_STATE
        state<class ERROR_STATE> + event<EventErrorOccurred> =
            state<class ERROR_STATE>,  // 0
        state<class IDLE> + event<EventErrorOccurred> =
            state<class ERROR_STATE>,  // 1
        state<class STOPPED> + event<EventErrorOccurred> =
            state<class ERROR_STATE>,  // 2
        state<class RUNNING> + event<EventErrorOccurred> =
            state<class ERROR_STATE>,  // 3
        state<class PAUSED> + event<EventErrorOccurred> =
            state<class ERROR_STATE>,  // 4
        state<class SERVOING> + event<EventErrorOccurred> =
            state<class ERROR_STATE>,  // 5

        state<class ENABLING> + event<EventErrorOccurred> =
            state<class ERROR_STATE>,  // 6
        state<class DISABLING> + event<EventErrorOccurred> =
            state<class ERROR_STATE>,  // 7
        state<class STARTING> + event<EventErrorOccurred> =
            state<class ERROR_STATE>,  // 8
        state<class STOPPING> + event<EventErrorOccurred> =
            state<class ERROR_STATE>,  // 9
        state<class PAUSING> + event<EventErrorOccurred> =
            state<class ERROR_STATE>,  // 10
        state<class CONTINUING> + event<EventErrorOccurred> =
            state<class ERROR_STATE>,  // 11
        state<class RESETTING> + event<EventErrorOccurred> =
            state<class ERROR_STATE>,  // 12
        state<class IDENTIFYING> + event<EventErrorOccurred> =
            state<class ERROR_STATE>  // 13
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

void Robot::on_fsm_enable() {
  setEnabled();
  if (IsEnabled()) {
    impl_->process_event(EventSuccess{});
  } else {
    impl_->process_event(EventErrorOccurred{});
  }
}
void Robot::on_fsm_disable() {
  setDisabled();
  if (!IsDisabled()) {
    impl_->process_event(EventSuccess{});
  } else {
    impl_->process_event(EventErrorOccurred{});
  }
}
void Robot::on_fsm_start() {
  // STARTING 进入：运动线程已由 MoveJ/MoveL 等在调用线程启动，
  // 此处确认启动成功并转入 RUNNING。
  impl_->process_event(EventSuccess{});
}
void Robot::on_fsm_run() {
  // 已进入 RUNNING 状态，运动正在执行。仅记录，不再触发事件。
  log_ptr_->info("Robot is running.");
}
void Robot::on_fsm_stop() {
  // STOPPING 进入：运动已结束/被中止，确认停止并转入 STOPPED。
  impl_->process_event(EventSuccess{});
}
void Robot::on_fsm_pause() {
  // PAUSING 进入：确认暂停并转入 PAUSED。
  impl_->process_event(EventSuccess{});
}
void Robot::on_fsm_continue() {
  // CONTINUING 进入：确认继续并转回 RUNNING。
  impl_->process_event(EventSuccess{});
}
void Robot::on_fsm_identify() {
  // 动力学参数辨识入口。具体辨识算法（激励轨迹、最小二乘求解等）后续实现，
  // 此处仅占位：辨识流程结束后回报成功，状态机自动返回 STOPPED。
  log_ptr_->info("Robot dynamics identification started...");
  // TODO: 执行动力学参数辨识流程
  impl_->process_event(EventSuccess{});
}
void Robot::on_fsm_reset() {
  log_ptr_->info("Robot is initializing...");

  if (IsEnabled()) {
    impl_->process_event(EventIsEnabled{});  // 模拟初始化成功事件
  } else {
    impl_->process_event(EventSuccess{});  // 模拟初始化失败事件
  }
}
void Robot::on_fsm_servo() {
  log_ptr_->info("Robot is servoing...");
}

Robot::Robot() : impl_(std::make_unique<Impl>(*this)) {
  log_ptr_ = Logger::getInstance("Robot");

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

std::string Robot::GetRobotState() const {
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
  } else if (impl_->is(sml::state<class CONTINUING>)) {
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
  } else if (impl_->is(sml::state<class IDENTIFYING>)) {
    return "IDENTIFYING";
  } else if (impl_->is(sml::state<class ERROR_STATE>)) {
    return "ERROR_STATE";
  } else {
    return "UNKNOWN_STATE";
  }
}

int Robot::SetEnabled() {
  log_ptr_->info("Robot Enabling.....");
  if (!impl_->process_event(EventEnableReq{})) {
    log_ptr_->error("Failed to process EventEnableReq.");
    return -1;  // TODO: 需要替换为错误码
  }

  return 0;
}

int Robot::SetDisabled() {
  log_ptr_->info("Robot Disabling.....");
  if (!impl_->process_event(EventDisableReq{})) {
    log_ptr_->error("Failed to process EventDisableReq.");
    return -1;  // TODO: 需要替换为错误码
  }

  return 0;
}

bool Robot::IsEnabled() { return true; }

bool Robot::IsDisabled() { return true; }

bool Robot::IsMotionRunning() const {
  return impl_->is(sml::state<class RUNNING>);
}

void Robot::waitControlCycle() {}

void Robot::setEnabled() {}

void Robot::setDisabled() {}

/////// Motion Command /////////////

int Robot::Pause() {}

int Robot::Continue() {}

int Robot::Stop() {}

int Robot::Start() {}

}  // namespace rocos
