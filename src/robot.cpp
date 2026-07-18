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
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cctype>
#include <fstream>
#include <sstream>

#include "hardware.hpp"

#include "move_joint.hpp"
#include "move_line.hpp"           // v3.0 保留，原有在线规划版本
#include "move_line_offline.hpp"   // v3.0 离线规划 + 全轨迹 IK 验证版本
#include "move_circle.hpp"
#include "move_jog.hpp"
#include "move_null_jog.hpp"
#include "move_svd_jog.hpp"

#include "position_controller.hpp"
#include "joint_impedance_controller.hpp"
#include "cartesian_impedance_controller.hpp"
#include "model.hpp"


namespace {
    // 状态定义
    class ERROR_STATE {
    }; // 0 错误状态，任何状态发生错误都转到这个状态[初始状态]
    class IDLE {
    }; // 1 空闲状态，机器人传感器与执行器就绪，等待使能命令
    class STOPPED {
    }; // 2 停止状态，机器人上使能，不会动
    class RUNNING {
    }; // 3 运行状态，机器人正在执行运动
    class PAUSED {
    }; // 4 暂停状态，机器人暂停在当前位置，等待继续或停止命令
    class SERVOING {
    }; // 5 伺服状态，用于高速udp伺服指令发送

    // class IDENTIFYING {};  // 动力学参数辨识状态，从STOPPED进入，辨识完成回到STOPPED

    // 中间状态定义
    class ENABLING {
    }; // 6  正在上使能状态
    class DISABLING {
    }; // 7  正在下使能状态
    class STARTING {
    }; // 8  正在启动状态，机器人正在启动
    class STOPPING {
    }; // 9  正在停止状态，机器人正在停止
    class PAUSING {
    }; // 10 正在暂停状态
    class RESUMING {
    }; // 11 正在恢复状态
    class RESETTING {
    }; // 12 正在复位状态，机器人正在复位

    // 机器人事件定义
    struct EventRunning {
    }; // 机器人开始运行事件
    struct EventAtTarget {
    }; // 机器人到达目标事件
    struct EventStopped {
    }; // 机器人已停止事件
    struct EventEnabled {
    }; // 机器人已上使能事件
    struct EventDisabled {
    }; // 机器人已下使能事件
    struct EventErrorOccurred {
    }; // 发生错误
    struct EventStartFailedReq {
    }; // 启动失败请求
    struct EventPauseFailed { //TODO: 暂停失败很危险，应该删除 by think
    };

    struct EventSuccessed {
    };

    // 指令事件定义
    struct EventEnableReq {
    }; // 上使能请求
    struct EventDisableReq {
    }; // 下使能请求
    struct EventStartReq {
    }; // 启动请求
    struct EventStopReq {
    }; // 停止请求
    struct EventPauseReq {
    }; // 暂停请求
    struct EventResumeReq {
    }; // 继续请求
    struct EventServoReq {
    }; // 伺服请求
    struct EventResetReq {
    }; // 恢复请求

    bool isValidFrameNameValue(const std::string &name) {
        if (name.empty() || name.size() > 64) return false;
        for (const unsigned char ch : name) {
            if (!std::isalnum(ch) && ch != '_' && ch != '-' && ch != '.') {
                return false;
            }
        }
        return true;
    }

    std::vector<std::string> collectFrameNames(const std::map<std::string, rocos::Frame> &frames) {
        std::vector<std::string> names;
        names.reserve(frames.size());
        for (const auto &frame : frames) {
            names.push_back(frame.first);
        }
        return names;
    }

    std::string joinNames(const std::vector<std::string> &names) {
        std::ostringstream oss;
        for (size_t i = 0; i < names.size(); ++i) {
            if (i > 0) oss << ", ";
            oss << names[i];
        }
        return oss.str();
    }

    std::string joinDriveIds(const std::vector<int32_t> &ids) {
        std::ostringstream oss;
        for (size_t i = 0; i < ids.size(); ++i) {
            if (i > 0) oss << ", ";
            oss << ids[i];
        }
        return oss.str();
    }

    YAML::Node frameToYaml(const rocos::Frame &frame) {
        YAML::Node node;
        node["position"].push_back(frame.p.x());
        node["position"].push_back(frame.p.y());
        node["position"].push_back(frame.p.z());

        double qx = 0.0;
        double qy = 0.0;
        double qz = 0.0;
        double qw = 1.0;
        frame.M.GetQuaternion(qx, qy, qz, qw);
        node["orientation"].push_back(qx);
        node["orientation"].push_back(qy);
        node["orientation"].push_back(qz);
        node["orientation"].push_back(qw);
        return node;
    }

    bool yamlToFrame(const YAML::Node &node, rocos::Frame &frame) {
        const YAML::Node position = node["position"];
        const YAML::Node orientation = node["orientation"];
        if (!position || !orientation || !position.IsSequence() || !orientation.IsSequence()) {
            return false;
        }
        if (position.size() != 3 || orientation.size() != 4) {
            return false;
        }

        const double x = position[0].as<double>();
        const double y = position[1].as<double>();
        const double z = position[2].as<double>();
        const double qx = orientation[0].as<double>();
        const double qy = orientation[1].as<double>();
        const double qz = orientation[2].as<double>();
        const double qw = orientation[3].as<double>();

        frame = rocos::Frame(KDL::Rotation::Quaternion(qx, qy, qz, qw), KDL::Vector(x, y, z));
        return true;
    }

    YAML::Node frameMapToYaml(const std::map<std::string, rocos::Frame> &frames) {
        YAML::Node node;
        for (const auto &frame : frames) {
            node[frame.first] = frameToYaml(frame.second);
        }
        return node;
    }

    rocos::Twist rotateTwist(const rocos::Twist &twist, const KDL::Rotation &rotation) {
        return rocos::Twist(rotation * twist.vel, rotation * twist.rot);
    }

    bool yamlToFrameMap(const YAML::Node &node, std::map<std::string, rocos::Frame> &frames) {
        frames.clear();
        if (!node) return true;
        if (!node.IsMap()) return false;

        for (const auto &entry : node) {
            const std::string name = entry.first.as<std::string>();
            if (!isValidFrameNameValue(name)) return false;

            rocos::Frame frame;
            if (!yamlToFrame(entry.second, frame)) return false;
            frames[name] = frame;
        }
        return true;
    }


    namespace sml = boost::sml;

    const auto action_start = [](rocos::Robot &robot) { robot.on_fsm_start(); };
    const auto action_run = [](rocos::Robot &robot) { robot.on_fsm_run(); }; //TODO：目前没有任何处理
    const auto action_pause = [](rocos::Robot &robot) { robot.on_fsm_pause(); };
    const auto action_resume = [](rocos::Robot &robot) { robot.on_fsm_resume(); };
    const auto action_stop = [](rocos::Robot &robot) { robot.on_fsm_stop(); };
    const auto action_reset = [](rocos::Robot &robot) { robot.on_fsm_reset(); };
    const auto action_enable = [](rocos::Robot &robot) { robot.on_fsm_enable(); };
    const auto action_disable = [](rocos::Robot &robot) { robot.on_fsm_disable(); };
    const auto action_servo = [](rocos::Robot &robot) { robot.on_fsm_servo(); };
    const auto action_error = [](rocos::Robot &robot) {
    }; //TODO: 进入错误状态时的必要处理

    struct StateMachine {
        auto operator()() const noexcept {
            using namespace sml;
            return make_transition_table(
                // 初始化
                *state<class ERROR_STATE> + event<EventResetReq> = state<class RESETTING>, // 0 ERROR_STATE->RESETTING
                state<class ERROR_STATE> + event<EventStopReq> = state<class STOPPING>, //   ERROR_STATE->STOPPING
                state<class ERROR_STATE> + sml::on_entry<_> / action_error, //   ERROR_STATE->RESETTING

                state<class IDLE> + event<EventEnableReq> = state<class ENABLING>, // 1 IDLE->ENABLING

                state<class STOPPED> + event<EventDisableReq> = state<class DISABLING>, // 2 STOPPED->DISABLING
                state<class STOPPED> + event<EventStartReq> = state<class STARTING>, //   STOPPED->STARING
                state<class STOPPED> + event<EventServoReq> = state<class SERVOING>, //   STOPPED->SERVOING

                state<class RUNNING> + sml::on_entry<_> / action_run, // 3 RUNNING
                state<class RUNNING> + event<EventStopped> = state<class STOPPED>,
                //todo     EventAtTarget不是必要的应该直接success
                state<class RUNNING> + event<EventAtTarget> = state<class STOPPED>,
                state<class RUNNING> + event<EventSuccessed> = state<class STOPPED>, //   RUNNING->STOPPED

                state<class RUNNING> + event<EventPauseReq> = state<class PAUSING>, //   RUNNING->PAUSING
                state<class RUNNING> + event<EventStopReq> = state<class STOPPING>, //   RUNNING->STOPPING

                state<class PAUSED> + event<EventResumeReq> = state<class RESUMING>, // 4 PAUSED->RESUMING
                state<class PAUSED> + event<EventStopReq> = state<class STOPPING>, //   PAUSED->STOPPING

                state<class SERVOING> + sml::on_entry<_> / action_servo, // 5 SERVOING
                state<class SERVOING> + event<EventStopReq> = state<class STOPPING>, //   SERVOING->STOPPING

                state<class ENABLING> + sml::on_entry<_> / action_enable, // 6 ENABLING
                state<class ENABLING> + event<EventEnabled> = state<class STOPPED>, // 6 ENABLING->STOPPED

                state<class DISABLING> + sml::on_entry<_> / action_disable, // 7 DISABLING
                state<class DISABLING> + event<EventDisabled> = state<class IDLE>, //   DISABLING->IDLE


                state<class STARTING> + sml::on_entry<_> / action_start, // 8 STARTING
                state<class STARTING> + event<EventAtTarget> = state<class STOPPED>,
                state<class STARTING> + event<EventStartFailedReq> = state<class STOPPED>,
                state<class STARTING> + event<EventRunning> = state<class RUNNING>, //   STARTING->RUNNING

                state<class STOPPING> + sml::on_entry<_> / action_stop, // 9 STOPPING
                state<class STOPPING> + event<EventStopped> = state<class STOPPED>,
                state<class STOPPING> + event<EventSuccessed> = state<class STOPPED>, //   STOPPING->STOPPED

                state<class PAUSING> + sml::on_entry<_> / action_pause, // 10 PAUSING->RESUMING
                // TODO                这个地方应该是success
                state<class PAUSING> + event<EventStopped> = state<class PAUSED>,
                state<class PAUSING> + event<EventStopReq> = state<class STOPPING>,
                state<class PAUSING> + event<EventPauseFailed> = state<class RUNNING>, //TODO：暂停失败退回RUNNING是否合适？
                state<class PAUSING> + event<EventSuccessed> = state<class PAUSED>, //TODO: 为什么需要Successed
                //   PAUSING->PAUSED

                state<class RESUMING> + sml::on_entry<_> / action_resume, // 11 RESUMING
                state<class RESUMING> + event<EventRunning> = state<class RUNNING>,
                state<class RESUMING> + event<EventStopReq> = state<class STOPPING>,
                state<class RESUMING> + event<EventSuccessed> = state<class RUNNING>, //    RESUMING->RUNNING

                state<class RESETTING> + sml::on_entry<_> / action_reset, // 12 RESETTING
                state<class RESETTING> + event<EventDisabled> = state<class ERROR_STATE>, // RESETTING失败必须保持错误态
                state<class RESETTING> + event<EventEnabled> = state<class STOPPED>, //    RESETTING->STOPPED


                // ANY STATE JUMP TO ERROR_STATE
                state<class ERROR_STATE> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 0
                state<class IDLE> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 1
                state<class STOPPED> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 2
                state<class RUNNING> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 3
                state<class PAUSED> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 4
                state<class SERVOING> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 5
                state<class ENABLING> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 6
                state<class DISABLING> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 7
                state<class STARTING> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 8
                state<class STOPPING> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 9
                state<class PAUSING> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 10
                state<class RESUMING> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 11
                state<class RESETTING> + event<EventErrorOccurred> = state<class ERROR_STATE> // 12
            );
        }
    };
} // namespace

namespace rocos {
    class Robot::Impl {
    public:
        explicit Impl(Robot &owner) : sm_{owner} {
        }

        // 运动线程与 HTTP 线程都会触发事件，必须串行化处理。
        // 用递归锁：action 回调（on_entry）内会再次调用 process_event（如 ENABLING
        // 进入时 on_fsm_enable 再触发 EventSuccess），属同线程重入，普通锁会死锁。
        template<typename Event>
        bool process_event(const Event &event) {
            std::lock_guard<std::recursive_mutex> lock(mtx_);
            return sm_.process_event(event);
        }

        template<typename TState>
        bool is(const TState &s) const {
            std::lock_guard<std::recursive_mutex> lock(mtx_);
            return sm_.is(s);
        }

    private:
        sml::sm<StateMachine> sm_; // 状态机实例
        mutable std::recursive_mutex mtx_; // 保护状态机的并发访问（允许同线程重入）
    };

#pragma region 状态机action处理函数

    void Robot::on_fsm_enable() {
        log_ptr_->info("机器人正在上使能中");

        // IsEnabled();
        setEnabled();

        if (IsEnabled()) {
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


        setDisabled();
        if (IsDisabled()) {
            log_ptr_->info("机器人下使能成功，准备进入IDLE状态");
            impl_->process_event(EventDisabled{});
        } else {
            log_ptr_->error("机器人下使能失败，准备进入ERROR_STATE状态");
            impl_->process_event(EventErrorOccurred{});
        }
    }

    void Robot::on_fsm_start() {
        // 此处确认启动成功并转入 RUNNING。
        auto rc = data_ready_callback_();
        if (rc != Result::NoError) {
            if (rc == Result::PlanFinished) {
                log_ptr_->info("机器人已在目标位置，无需运动");
                impl_->process_event(EventAtTarget{});
                return;
            }

            log_ptr_->error("机器人启动失败，退回Stopped状态");
            impl_->process_event(EventStartFailedReq{});


            return;
        }


        if (control_thread_.joinable()) {
            log_ptr_->info("控制线程已存在，等待其退出");
            control_thread_.join();
        }
        //TODO: 简化为匿名函数，周期循环调用
        control_thread_ = std::thread([this]() {
            while (IsControlActive()) {
                RunCycle();
                waitControlCycle();
            }
        });


        impl_->process_event(EventRunning{});
    }

    void Robot::on_fsm_run() {
        log_ptr_->info("Robot is running.");

        // if (control_thread_.joinable()) {
        //     // if (control_thread_active_.load()) {
        //     //   // 控制线程仍在 RUNNING/PAUSING/PAUSED/RESUMING/STOPPING 中维持循环。
        //     //   return;
        //     // }
        //     control_thread_.join();
        // }

        // control_thread_active_.store(true);
        // control_thread_ = std::thread(&Robot::controlLoop, this);
    }

    void Robot::on_fsm_stop() {
        // if (control_thread_.joinable()) control_thread_.join();
        // impl_->process_event(EventStopped{});
        Result rc = executor->Stop();
        if (rc == Result::NoError) {
            log_ptr_->info("机器人停止成功，准备进入Stopped状态");
        } else {
            log_ptr_->error("机器人停止失败，退回ERROR状态");
            impl_->process_event(EventErrorOccurred{});
        }
    }

    // void Robot::controlLoop() {
    //
    //   while (IsControlActive()) {
    //     RunCycle();
    //     waitControlCycle();
    //   }
    //
    //   // control_thread_active_.store(false);
    // }

    void Robot::on_fsm_pause() {
        // PAUSING 进入：确认暂停并转入 PAUSED。
        Result rc = executor->Pause();
        if (rc == Result::NoError) {
            log_ptr_->info("机器人暂停成功，准备进入PAUSED状态");
        } else {
            //TODO: 暂停失败  这块我感觉不允许失败 by think
            log_ptr_->error("机器人暂停失败，退回RUNNING状态");
            impl_->process_event(EventPauseFailed{});
        }

        // impl_->process_event(EventStopped{});
    }

    void Robot::on_fsm_resume() {
        // CONTINUING 进入：确认继续并转回 RUNNING。
        Result rc = executor->Resume();
        if (rc == Result::NoError) {
            log_ptr_->info("机器人继续成功，准备进入RUNNING状态");
            impl_->process_event(EventRunning{});
        }

        // impl_->process_event(EventRunning{});
    }

    void Robot::on_fsm_reset() {
        log_ptr_->info("机器人进入RESETTING，开始清除报警并重新建立使能状态");

        if (control_thread_.joinable() &&
            control_thread_.get_id() != std::this_thread::get_id()) {
            log_ptr_->info("ResetFault等待控制线程退出");
            control_thread_.join();
        }

        if (motion) {
            const Result motion_reset = motion->Reset();
            if (motion_reset != Result::NoError && motion_reset != Result::PlanFinished) {
                log_ptr_->warn("ResetFault重置当前motion返回: {}", static_cast<int>(motion_reset));
            }
            motion.reset();
            if (executor) {
                executor->SwitchMotion(nullptr);
            }
        }

        if (controller && !controller->Reset()) {
            log_ptr_->error("ResetFault重置控制器失败，保持ERROR_STATE");
            impl_->process_event(EventErrorOccurred{});
            return;
        }

        if (hardware) {
            const Result clear_result = hardware->ClearFault();
            if (clear_result != Result::NoError &&
                clear_result != Result::FunctionNotSupported) {
                log_ptr_->error("ResetFault清除硬件报警失败: {}", static_cast<int>(clear_result));
                impl_->process_event(EventErrorOccurred{});
                return;
            }
            if (clear_result == Result::FunctionNotSupported) {
                log_ptr_->warn("当前Hardware未实现ClearFault，继续尝试重新使能");
            }
        }

        setEnabled();
        if (IsEnabled()) {
            log_ptr_->info("ResetFault成功，机器人已经使能，准备进入STOPPED状态");
            impl_->process_event(EventEnabled{}); // 模拟初始化成功事件
        } else {
            log_ptr_->error("ResetFault后重新使能失败，保持ERROR_STATE");
            impl_->process_event(EventErrorOccurred{});
        }
    }

    void Robot::on_fsm_servo() {
        log_ptr_->info("Robot is servoing...");
    }

#pragma endregion


    Robot::Robot() : impl_(std::make_unique<Impl>(*this)) {
        log_ptr_ = Logger::getInstance("Robot");

        hardware = std::make_unique<Hardware>("hardware_talon_config.yaml", 0); //TODO: 初始化Hardware指针，后续路径需要从配置文件加载
        model = std::make_unique<Model>("robot.urdf", "base_link", "link_7"); //TODO：初始化Model指针，后续路径需要从配置文件加载

        // --- JointBinding 初始化：在 controller 创建之前完成轴绑定 ---
        {
            const auto model_joint_names = model->GetJointNames();
            const auto hardware_drive_ids = hardware->GetDriveIds();

            log_ptr_->info("JointBinding 开始初始化: model joints={} [{}], hw drives={} [{}], yaml={}",
                           model_joint_names.size(), joinNames(model_joint_names),
                           hardware_drive_ids.size(), joinDriveIds(hardware_drive_ids),
                           joint_binding_path_);

            joint_binding_ = std::make_unique<JointBinding>();
            Result rc = joint_binding_->Configure(model_joint_names, hardware_drive_ids);
            if (rc != Result::NoError) {
                log_ptr_->error("JointBinding Configure 失败: model joints={}, hw drives={}",
                                model_joint_names.size(), hardware_drive_ids.size());
                throw std::runtime_error("JointBinding Configure failed: model joints="
                    + std::to_string(model_joint_names.size()) + " hw drives="
                    + std::to_string(hardware_drive_ids.size()));
            }

            // 检查 joint_binding.yaml 是否存在
            std::ifstream binding_file(joint_binding_path_);
            if (binding_file.good()) {
                log_ptr_->info("发现 JointBinding YAML，使用显式绑定: {}", joint_binding_path_);
                rc = joint_binding_->LoadFromYaml(joint_binding_path_);
                if (rc != Result::NoError) {
                    log_ptr_->error("JointBinding YAML 加载失败: {}", joint_binding_path_);
                    throw std::runtime_error("JointBinding LoadFromYaml failed: " + joint_binding_path_);
                }
            } else {
                // 无 YAML 时要求 model joint 数量 == hardware drive 数量
                if (model_joint_names.size() != hardware_drive_ids.size()) {
                    log_ptr_->error("JointBinding YAML 不存在且数量不匹配: yaml={}, model joints={}, hw drives={}",
                                    joint_binding_path_, model_joint_names.size(), hardware_drive_ids.size());
                    throw std::runtime_error(
                        "joint_binding.yaml missing and joint/drive count mismatch: model="
                        + std::to_string(model_joint_names.size())
                        + " hw=" + std::to_string(hardware_drive_ids.size()));
                }
                log_ptr_->warn("JointBinding YAML 不存在，使用默认顺序绑定: model joints={}, hw drives={}",
                               model_joint_names.size(), hardware_drive_ids.size());
                // 自动生成默认顺序绑定
                for (size_t i = 0; i < model_joint_names.size(); ++i) {
                    joint_binding_->Bind(model_joint_names[i], hardware_drive_ids[i]);
                }
            }

            rc = joint_binding_->Validate();
            if (rc != Result::NoError) {
                log_ptr_->error("JointBinding Validate 失败: yaml={}", joint_binding_path_);
                throw std::runtime_error("JointBinding Validate failed");
            }

            const auto model_index_to_drive_id = joint_binding_->GetModelIndexToDriveIds();
            log_ptr_->info("准备写入 Hardware JointBinding: model_index_to_drive_id=[{}]",
                           joinDriveIds(model_index_to_drive_id));
            rc = hardware->SetJointBinding(model_index_to_drive_id);
            if (rc != Result::NoError) {
                log_ptr_->error("Hardware SetJointBinding 失败: model_index_to_drive_id=[{}]",
                                joinDriveIds(model_index_to_drive_id));
                throw std::runtime_error("Hardware SetJointBinding failed");
            }

            log_ptr_->info("JointBinding 初始化完成: model joints={}, hw drives={}, binding={}",
                           model_joint_names.size(), hardware_drive_ids.size(),
                           joint_binding_path_);
        }

        controller = std::make_unique<PositionController>(); //TODO： 默认加载位置控制器

        Result rc = controller->SetHardware(hardware.get()); //TODO：控制器需要传入硬件指针，可以考虑初始化时导入

        if (rc != Result::NoError) {
            throw std::runtime_error("PositionController SetHardware failed");
        }

        rc = controller->SetModel(model.get());
        if (rc != Result::NoError) {
            throw std::runtime_error("PositionController SetModel failed");
        }

        executor = std::make_unique<Executor>(
            motion.get(),
            controller.get(),
            hardware.get());

        // motion = std::make_unique<MoveJoint>();
        // controller = std::make_unique<PositionController>(); //TODO: 默认加载位置控制器


        log_ptr_->info("机器人开始初始化");
        impl_->process_event(EventResetReq{}); // 进入初始化状态
    }

    Robot::~Robot() {
        if (control_thread_.joinable()) {
            control_thread_.join();
        }

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
            return "RESUMING";
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

    Robot::RobotStateSnapshot Robot::GetRobotStateSnapshot() const {
        RobotStateSnapshot snap;

        // 时间戳
        snap.timestamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // FSM 状态
        snap.state_string = GetStateString();
        snap.is_enabled = IsEnabled();
        snap.is_running = IsRunning();
        snap.control_active = IsControlActive();
        snap.motion_busy = IsMotionBusy();

        // 关节状态
        const int n = getJointNum();
        snap.joints.reserve(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            RobotStateSnapshot::JointState js;
            js.id = i;
            js.name = getJointName(i);
            js.position = getJointPosition(i);
            js.velocity = getJointVelocity(i);
            js.torque = getJointTorque(i);
            js.load_torque = getJointLoadTorque(i);
            js.status = getJointStatus(i);
            snap.joints.push_back(std::move(js));
        }

        // 法兰末端位姿
        snap.flange = getFlange();

        // 激活的坐标系
        snap.active_tool_frame_name = GetActiveToolFrameName();
        snap.active_object_frame_name = GetActiveObjectFrameName();
        snap.active_tool_frame = GetActiveToolFrame();
        snap.active_object_frame = GetActiveObjectFrame();

        // 硬件摘要
        snap.joint_num = n;
        if (hardware) {
            snap.hardware_state = static_cast<int>(hardware->GetState());
        }

        return snap;
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

    Result Robot::ResetFault() {
        log_ptr_->info("收到ResetFault清除报警指令，当前状态: {}", GetStateString());
        if (!impl_->is(sml::state<class ERROR_STATE>)) {
            log_ptr_->error("当前状态不是ERROR_STATE，拒绝ResetFault: {}", GetStateString());
            return Result::JointStateError;
        }

        if (!impl_->process_event(EventResetReq{})) {
            log_ptr_->error("ResetFault状态机事件被拒绝，当前状态: {}", GetStateString());
            return Result::JointStateError;
        }

        if (impl_->is(sml::state<class STOPPED>)) {
            return Result::NoError;
        }

        log_ptr_->error("ResetFault未能回到STOPPED，当前状态: {}", GetStateString());
        return Result::Fatal;
    }


    bool Robot::IsEnabled() const {
        auto state = hardware->GetState();
        return state == JntState::ENABLED;
    }

    bool Robot::IsDisabled() const {
        auto state = hardware->GetState();
        return state == JntState::DISABLED;
    }

    bool Robot::IsRunning() const {
        return impl_->is(sml::state<class RUNNING>);
    }

    bool Robot::IsControlActive() const {
        return impl_->is(sml::state<class RUNNING>)
               || impl_->is(sml::state<class PAUSING>)
               || impl_->is(sml::state<class PAUSED>)
               || impl_->is(sml::state<class RESUMING>)
               || impl_->is(sml::state<class STOPPING>);
    }

    bool Robot::IsMotionBusy() const {
        return impl_->is(sml::state<class STARTING>) || IsControlActive();
    }

    Result Robot::WaitMove() {
        using namespace std::chrono_literals;

        std::this_thread::sleep_for(20ms);
        while (true) {
            if (impl_->is(sml::state<class ERROR_STATE>)) {
                return Result::Fatal;
            }
            if (impl_->is(sml::state<class IDLE>)) {
                return Result::NotEnabled;
            }
            if (!IsMotionBusy()) {
                return Result::NoError;
            }
            std::this_thread::sleep_for(10ms);
        }
    }

    std::vector<Robot::JointInfo> Robot::GetJointInfo() const {
        std::vector<JointInfo> joint_infos;
        const auto *hw = dynamic_cast<const Hardware *>(hardware.get());
        if (hw == nullptr) return joint_infos;

        const auto &drives = hw->getConfig().drives;
        joint_infos.reserve(drives.size());
        for (const auto &drive : drives) {
            JointInfo info;
            info.id = drive.id;
            info.name = drive.joint_name;
            info.cnt_per_unit = drive.transform.cnt_per_unit;
            info.torque_per_unit = drive.transform.torque_per_unit;
            info.ratio = drive.transform.ratio;
            info.unit_name = drive.transform.user_unit_name;
            info.zero_offset = drive.transform.offset_pos_cnt;
            joint_infos.push_back(std::move(info));
        }
        return joint_infos;
    }

    void Robot::waitControlCycle() {
        hardware->WaitForSignal();
    }

    void Robot::setEnabled() {
        hardware->SetEnabled();
    }

    void Robot::setDisabled() {
        hardware->SetDisabled();
    }

    /////// Motion Command /////////////

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
            if (impl_->is(sml::state<class PAUSED>)) {
                r = executor->Update();
                if (static_cast<int>(r) < 0) {
                    impl_->process_event(EventErrorOccurred{});
                }

                return;
            }
            impl_->process_event(EventSuccessed{}); //
            return;
        }
        if (static_cast<int>(r) < 0) {
            impl_->process_event(EventErrorOccurred{});
            return;
        }

        // ② 参考 → 指令 → 硬件

        r = executor->Update();
        if (static_cast<int>(r) < 0) {
            impl_->process_event(EventErrorOccurred{});
            return;
        }
    }

    // ============================================================================
    // MoveJ：关节空间点到点，一次性下发跑完即停
    // ============================================================================

    Result Robot::MoveJ(const JntArray &q_goal,
                        double v_limit, double a_limit, double j_limit) {
        // if (!IsEnabled())   return Result::NotEnabled;
        // if (motion && IsRunning()) return Result::ConflictTaskRunning;

        data_ready_callback_ = [this, q_goal, v_limit, a_limit, j_limit]() -> Result {
            const int n = getJointNum();
            JntArray q_start(static_cast<unsigned int>(n));
            for (int i = 0; i < n; ++i) q_start(i) = getJointPosition(i);

            auto new_motion = std::make_unique<MoveJoint>(
                q_start, q_goal, v_limit, a_limit, j_limit, /*dt=*/0.001);


            Result rc = new_motion->Reset();
            if (rc != Result::NoError) return rc;

            motion = std::move(new_motion);

            executor->SwitchMotion(motion.get());
            return Result::NoError;
        };
        if (impl_->process_event(EventStartReq{})) {
            // STOPPED → RUNNING
        } else {
            if (impl_->is(sml::state<class IDLE>)) {
                log_ptr_->error("机器人处于IDLE状态，无法执行MoveJ指令，请先上使能");
                return Result::NotEnabled;
            } else if (impl_->is(sml::state<class ERROR_STATE>)) {
                log_ptr_->error("机器人处于ERROR_STATE状态，无法执行MoveJ指令，请先复位");
                return Result::Fatal;
            } else {
                log_ptr_->error("机器人当前状态无法执行MoveJ指令");
                return Result::ConflictTaskRunning;
            }
        }


        return Result::NoError;
    }

    // ============================================================================
    // MoveL：笛卡尔直线，支持多工具坐标系
    // ============================================================================

    Result Robot::MoveL(const Frame &pose_goal,
                        const std::string &tool_name,
                        double v_limit, double a_limit, double j_limit) {
        if (!tool_name.empty() && !HasToolFrame(tool_name)) {
            log_ptr_->error("工具坐标系不存在: {}", tool_name);
            return Result::ResourceUnavailable;
        }

        data_ready_callback_ = [this, pose_goal, tool_name, v_limit, a_limit, j_limit]() -> Result {
            const int n = getJointNum();
            JntArray q_current(static_cast<unsigned int>(n));
            for (int i = 0; i < n; ++i) q_current(i) = getJointPosition(i);

            Frame pose_flange;
            if (model) model->ForwardKinematics(q_current, pose_flange);

            Frame T_tool;
            if (!tool_name.empty()) {
                Result rc = GetToolFrame(tool_name, T_tool);
                if (rc != Result::NoError) return rc;
            }

            // v3.0 替换为离线规划版本：Reset 中全轨迹 IK 验证，
            // 任一点逆解失败直接退回 Stopped，避免在线执行到一半才发现不可达。
            // 暂停/继续走 Jacobian 积分通道，保证在笛卡尔直线上减速和恢复。
            auto new_motion = std::make_unique<MoveLineOffline>(
                pose_flange, pose_goal * T_tool.Inverse(),
                model.get(),
                v_limit, a_limit, j_limit, /*dt=*/0.001);

            new_motion->SetInitialJointPosition(q_current);  // IK 暖启动
            Result rc = new_motion->Reset();

            // ---- 原在线版本（保留以备回退） ----
            // auto new_motion = std::make_unique<MoveLine>(
            //     pose_flange, pose_goal * T_tool.Inverse(),
            //     v_limit, a_limit, j_limit, /*dt=*/0.001, model.get());
            // Result rc = new_motion->Reset();
            if (rc != Result::NoError) return rc;

            motion = std::move(new_motion);

            executor->SwitchMotion(motion.get());
            return Result::NoError;

            log_ptr_->error("executor is nullptr");
            return Result::Fatal;
        };

        if (!impl_->process_event(EventStartReq{})) {
            if (impl_->is(sml::state<class IDLE>)) return Result::NotEnabled;
            if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
            return Result::ConflictTaskRunning;
        }
        return Result::NoError;
    }

    Result Robot::MoveJ_IK(const Frame &pose_goal,
                           double v_limit, double a_limit, double j_limit) {
        if (!model) {
            log_ptr_->error("机器人模型未初始化，无法执行MoveJ_IK指令");
            return Result::ResourceUnavailable;
        }

        const int n = getJointNum();
        JntArray q_current(static_cast<unsigned int>(n));
        for (int i = 0; i < n; ++i) q_current(i) = getJointPosition(i);

        JntArray q_goal(static_cast<unsigned int>(n));
        Result rc = model->InverseKinematics(q_current, pose_goal, q_goal);
        if (rc != Result::NoError) return rc;

        return MoveJ(q_goal, v_limit, a_limit, j_limit);
    }

    Result Robot::MoveL_FK(const JntArray &q_goal,
                           const std::string &tool_name,
                           double v_limit, double a_limit, double j_limit) {
        if (!model) {
            log_ptr_->error("机器人模型未初始化，无法执行MoveL_FK指令");
            return Result::ResourceUnavailable;
        }

        Frame pose_goal;
        Result rc = model->ForwardKinematics(q_goal, pose_goal);
        if (rc != Result::NoError) return rc;

        return MoveL(pose_goal, tool_name, v_limit, a_limit, j_limit);
    }

    std::vector<std::string> Robot::GetToolFrameNames() const {
        return collectFrameNames(tool_frames_);
    }

    std::vector<std::string> Robot::GetObjectFrameNames() const {
        return collectFrameNames(object_frames_);
    }

    bool Robot::HasToolFrame(const std::string &name) const {
        return tool_frames_.find(name) != tool_frames_.end();
    }

    bool Robot::HasObjectFrame(const std::string &name) const {
        return object_frames_.find(name) != object_frames_.end();
    }

    bool Robot::IsValidFrameName(const std::string &name) const {
        return isValidFrameNameValue(name);
    }

    Result Robot::GetToolFrame(const std::string &name, Frame &frame) const {
        auto it = tool_frames_.find(name);
        if (it == tool_frames_.end()) return Result::ResourceUnavailable;
        frame = it->second;
        return Result::NoError;
    }

    Result Robot::GetObjectFrame(const std::string &name, Frame &frame) const {
        auto it = object_frames_.find(name);
        if (it == object_frames_.end()) return Result::ResourceUnavailable;
        frame = it->second;
        return Result::NoError;
    }

    Frame Robot::GetToolFrame(const std::string &name) const {
        auto it = tool_frames_.find(name);
        return (it != tool_frames_.end()) ? it->second : Frame();
    }

    Frame Robot::GetObjectFrame(const std::string &name) const {
        auto it = object_frames_.find(name);
        return (it != object_frames_.end()) ? it->second : Frame();
    }

    Result Robot::SetToolFrame(const std::string &name, const Frame &T_tool) {
        if (!IsValidFrameName(name)) return Result::IllegalParameter;
        tool_frames_[name] = T_tool;
        return Result::NoError;
    }

    Result Robot::SetObjectFrame(const std::string &name, const Frame &T_object) {
        if (!IsValidFrameName(name)) return Result::IllegalParameter;
        object_frames_[name] = T_object;
        return Result::NoError;
    }

    Result Robot::AddToolFrame(const std::string &name, const Frame &T_tool) {
        if (!IsValidFrameName(name)) return Result::IllegalParameter;
        auto inserted = tool_frames_.emplace(name, T_tool);
        return inserted.second ? Result::NoError : Result::CallingConflictError;
    }

    Result Robot::AddObjectFrame(const std::string &name, const Frame &T_object) {
        if (!IsValidFrameName(name)) return Result::IllegalParameter;
        auto inserted = object_frames_.emplace(name, T_object);
        return inserted.second ? Result::NoError : Result::CallingConflictError;
    }

    Result Robot::RemoveToolFrame(const std::string &name) {
        if (!IsValidFrameName(name)) return Result::IllegalParameter;
        if (tool_frames_.erase(name) == 0) return Result::ResourceUnavailable;
        if (active_tool_frame_name_ == name) active_tool_frame_name_.clear();
        return Result::NoError;
    }

    Result Robot::RemoveObjectFrame(const std::string &name) {
        if (!IsValidFrameName(name)) return Result::IllegalParameter;
        if (object_frames_.erase(name) == 0) return Result::ResourceUnavailable;
        if (active_object_frame_name_ == name) active_object_frame_name_.clear();
        return Result::NoError;
    }

    Result Robot::SetActiveToolFrame(const std::string &name) {
        if (!IsValidFrameName(name)) return Result::IllegalParameter;
        if (!HasToolFrame(name)) return Result::ResourceUnavailable;
        active_tool_frame_name_ = name;
        return Result::NoError;
    }

    Result Robot::SetActiveObjectFrame(const std::string &name) {
        if (!IsValidFrameName(name)) return Result::IllegalParameter;
        if (!HasObjectFrame(name)) return Result::ResourceUnavailable;
        active_object_frame_name_ = name;
        return Result::NoError;
    }

    std::string Robot::GetActiveToolFrameName() const {
        return active_tool_frame_name_;
    }

    std::string Robot::GetActiveObjectFrameName() const {
        return active_object_frame_name_;
    }

    Result Robot::GetActiveToolFrame(Frame &frame) const {
        if (active_tool_frame_name_.empty()) return Result::ResourceUnavailable;
        return GetToolFrame(active_tool_frame_name_, frame);
    }

    Result Robot::GetActiveObjectFrame(Frame &frame) const {
        if (active_object_frame_name_.empty()) return Result::ResourceUnavailable;
        return GetObjectFrame(active_object_frame_name_, frame);
    }

    Frame Robot::GetActiveToolFrame() const {
        return GetToolFrame(active_tool_frame_name_);
    }

    Frame Robot::GetActiveObjectFrame() const {
        return GetObjectFrame(active_object_frame_name_);
    }

    Result Robot::LoadFrames(const std::string &path) {
        if (path.empty()) {
            if (log_ptr_) log_ptr_->error("加载坐标系 YAML 失败: path 为空");
            return Result::IllegalParameter;
        }

        try {
            if (log_ptr_) log_ptr_->info("开始加载坐标系 YAML: {}", path);
            YAML::Node root = YAML::LoadFile(path);
            std::map<std::string, Frame> loaded_tools;
            std::map<std::string, Frame> loaded_objects;
            if (!yamlToFrameMap(root["tools"], loaded_tools)) {
                if (log_ptr_) log_ptr_->error("加载坐标系 YAML 失败: tools 字段非法, path={}", path);
                return Result::IllegalParameter;
            }
            if (!yamlToFrameMap(root["objects"], loaded_objects)) {
                if (log_ptr_) log_ptr_->error("加载坐标系 YAML 失败: objects 字段非法, path={}", path);
                return Result::IllegalParameter;
            }

            std::string active_tool = root["active_tool"].as<std::string>("");
            std::string active_object = root["active_object"].as<std::string>("");
            if (!active_tool.empty() && loaded_tools.find(active_tool) == loaded_tools.end()) {
                if (log_ptr_) {
                    log_ptr_->error("加载坐标系 YAML 失败: active_tool='{}' 不存在, path={}",
                                    active_tool, path);
                }
                return Result::IllegalParameter;
            }
            if (!active_object.empty() && loaded_objects.find(active_object) == loaded_objects.end()) {
                if (log_ptr_) {
                    log_ptr_->error("加载坐标系 YAML 失败: active_object='{}' 不存在, path={}",
                                    active_object, path);
                }
                return Result::IllegalParameter;
            }

            tool_frames_ = std::move(loaded_tools);
            object_frames_ = std::move(loaded_objects);
            active_tool_frame_name_ = std::move(active_tool);
            active_object_frame_name_ = std::move(active_object);
            if (log_ptr_) {
                log_ptr_->info("坐标系 YAML 加载完成: path={}, tools={}, objects={}, active_tool='{}', active_object='{}'",
                               path, tool_frames_.size(), object_frames_.size(),
                               active_tool_frame_name_, active_object_frame_name_);
            }
            return Result::NoError;
        } catch (const YAML::Exception &e) {
            if (log_ptr_) log_ptr_->error("加载坐标系 YAML 失败: {} => {}", path, e.what());
            return Result::ResourceUnavailable;
        } catch (const std::exception &e) {
            if (log_ptr_) log_ptr_->error("加载坐标系文件失败: {} => {}", path, e.what());
            return Result::ResourceUnavailable;
        }
    }

    Result Robot::SaveFrames(const std::string &path) const {
        if (path.empty()) {
            if (log_ptr_) log_ptr_->error("保存坐标系 YAML 失败: path 为空");
            return Result::IllegalParameter;
        }

        YAML::Node root;
        root["active_tool"] = active_tool_frame_name_;
        root["active_object"] = active_object_frame_name_;
        root["tools"] = frameMapToYaml(tool_frames_);
        root["objects"] = frameMapToYaml(object_frames_);

        std::ofstream out(path);
        if (!out.is_open()) {
            if (log_ptr_) log_ptr_->error("保存坐标系 YAML 失败: 无法打开文件 {}", path);
            return Result::ResourceUnavailable;
        }
        out << root;
        if (!out.good()) {
            if (log_ptr_) log_ptr_->error("保存坐标系 YAML 失败: 写入失败 {}", path);
            return Result::ResourceUnavailable;
        }
        if (log_ptr_) {
            log_ptr_->info("坐标系 YAML 保存完成: path={}, tools={}, objects={}, active_tool='{}', active_object='{}'",
                           path, tool_frames_.size(), object_frames_.size(),
                           active_tool_frame_name_, active_object_frame_name_);
        }
        return Result::NoError;
    }

    // ============================================================================
    // MoveC 圆心+角度
    // ============================================================================

    Result Robot::MoveC(const Frame &pose_start, const Frame &center_frame,
                        double theta, double v_limit, double a_limit, double j_limit) {
        data_ready_callback_ = [this, pose_start, center_frame, theta, v_limit, a_limit, j_limit]() -> Result {
            auto new_motion = std::make_unique<MoveCircle>(
                pose_start, center_frame, theta, v_limit, a_limit, j_limit, /*dt=*/0.001, model.get());

            Result rc = new_motion->Reset();
            if (rc != Result::NoError) return rc;

            motion = std::move(new_motion);

            executor->SwitchMotion(motion.get());
            return Result::NoError;

            log_ptr_->error("executor is nullptr");
            return Result::Fatal;
        };

        if (!impl_->process_event(EventStartReq{})) {
            if (impl_->is(sml::state<class IDLE>)) return Result::NotEnabled;
            if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
            return Result::ConflictTaskRunning;
        }
        return Result::NoError;
    }

    // ============================================================================
    // MoveC 三点圆弧
    // ============================================================================

    Result Robot::MoveC(const Frame &pose_start, const Frame &pose_via,
                        const Frame &pose_goal,
                        double v_limit, double a_limit, double j_limit) {
        data_ready_callback_ = [this, pose_start, pose_via, pose_goal, v_limit, a_limit, j_limit]() -> Result {
            auto new_motion = std::make_unique<MoveCircle>(
                pose_start, pose_via, pose_goal, v_limit, a_limit, j_limit, /*dt=*/0.001, model.get());

            Result rc = new_motion->Reset();
            if (rc != Result::NoError) return rc;

            motion = std::move(new_motion);

            executor->SwitchMotion(motion.get());
            return Result::NoError;

            log_ptr_->error("executor is nullptr");
            return Result::Fatal;
        };

        if (!impl_->process_event(EventStartReq{})) {
            if (impl_->is(sml::state<class IDLE>)) return Result::NotEnabled;
            if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
            return Result::ConflictTaskRunning;
        }
        return Result::NoError;
    }

    Result Robot::PauseMotion() {
        //TODO: 直接判断是否支持暂停，不支持直接忽略请求
        if (!executor->CanPause()) {
            return Result::FunctionNotSupported;
        }

        if (!impl_->process_event(EventPauseReq{})) {
            log_ptr_->error("当前状态无法执行暂停指令");
            return Result::Fatal;
        }

        return Result::NoError;
    }

    Result Robot::StopMotion() {
        //TODO: 直接判断是否支持停止，不支持直接忽略，但应该都需要支持，所以后续要删掉
        if (!executor->CanStop()) {
            return Result::FunctionNotSupported;
        }

        if (!impl_->process_event(EventStopReq{})) {
            log_ptr_->error("当前状态无法执行停止指令");
            return Result::Fatal;
        }
        return Result::NoError;
    }

    Result Robot::ResumeMotion() {
        //TODO: 直接判断是否支持恢复，不支持直接忽略请求
        if (!executor->CanResume()) {
            return Result::FunctionNotSupported;
        }

        if (!impl_->process_event(EventResumeReq{})) {
            log_ptr_->error("当前状态无法执行恢复指令");
            return Result::Fatal;
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
    Result Robot::MoveJogging(const Twist &direction,
                              JogFrame frame,
                              double speed,
                              double timeout,
                              double dir_threshold) {
        KDL::Rotation R_base_frame = KDL::Rotation::Identity();

        switch (frame) {
            case JogFrame::BASE:
                break;
            case JogFrame::FLANGE:
                R_base_frame = getFlange().M;
                break;
            case JogFrame::TOOL: {
                Frame T_flange_tool;
                const Result rc = GetActiveToolFrame(T_flange_tool);
                if (rc != Result::NoError) return rc;
                R_base_frame = (getFlange() * T_flange_tool).M;
                break;
            }
            case JogFrame::OBJECT: {
                Frame T_base_object;
                const Result rc = GetActiveObjectFrame(T_base_object);
                if (rc != Result::NoError) return rc;
                R_base_frame = T_base_object.M;
                break;
            }
        }

        Twist base_direction;
        base_direction.vel = R_base_frame * direction.vel;
        base_direction.rot = R_base_frame * direction.rot;
        return MoveJogging(JogVec{base_direction}, speed, timeout, dir_threshold);
    }

    Result Robot::MoveJogging(const JogVec &direction, double speed,
                              double timeout, double dir_threshold) {
        // ── 分支 1：已有活跃 MoveJog → 旁路喂饭 ──
        if (auto *jog = dynamic_cast<MoveJog *>(motion.get())) {
            const Result rc = jog->FeedJog(direction, speed);
            if (rc == Result::NoError) return rc;
        }

        // ── 分支 2：其他 motion 正在运行 → 拒绝 ──
        if (motion && IsRunning()) return Result::ConflictTaskRunning;

        // ── 分支 3：全新启动 ──
        data_ready_callback_ = [this, direction, speed, timeout,
                    dir_threshold]() -> Result {
                    auto new_jog = std::make_unique<MoveJog>(
                        /*dt=*/0.001, timeout, model.get(), dir_threshold);

                    const int n = getJointNum();
                    JntArray q_current(static_cast<unsigned int>(n));
                    for (int i = 0; i < n; ++i) q_current(i) = getJointPosition(i);
                    new_jog->setInitialPosition(q_current);
                    Result rc = new_jog->Reset();
                    if (rc != Result::NoError) return rc;

                    rc = new_jog->FeedJog(direction, speed);
                    if (rc != Result::NoError) return rc;

                    motion = std::move(new_jog);

                    executor->SwitchMotion(motion.get());
                    return Result::NoError;

                    log_ptr_->error("executor is nullptr");
                    return Result::Fatal;
                };

        if (!impl_->process_event(EventStartReq{})) {
            if (impl_->is(sml::state<class IDLE>)) return Result::NotEnabled;
            if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
            return Result::ConflictTaskRunning;
        }
        return Result::NoError;
    }

    // ============================================================================
    // MoveNullJogging — 零空间点动（伪逆投影法）
    // ============================================================================

    Result Robot::MoveNullJogging(const JntArray &intent_direction, double speed,
                                  double timeout, double dir_threshold) {
        // ── 分支 1：已有活跃 MoveNullJog → 旁路喂饭 ──
        if (auto *jog = dynamic_cast<MoveNullJog *>(motion.get())) {
            const Result rc = jog->FeedNullJog(intent_direction, speed);
            if (rc == Result::NoError) return rc;
        }

        // ── 分支 2：其他 motion 正在运行 → 拒绝 ──
        if (motion && IsRunning()) return Result::ConflictTaskRunning;

        // ── 分支 3：全新启动 ──
        data_ready_callback_ = [this, intent_direction, speed, timeout, dir_threshold]() -> Result {
            auto new_jog = std::make_unique<MoveNullJog>(
                /*dt=*/0.001, timeout, model.get(), dir_threshold);

            const int n = getJointNum();
            JntArray q_current(static_cast<unsigned int>(n));
            for (int i = 0; i < n; ++i) q_current(i) = getJointPosition(i);
            new_jog->setInitialPosition(q_current);

            Result rc = new_jog->Reset();
            if (rc != Result::NoError) return rc;

            rc = new_jog->FeedNullJog(intent_direction, speed);
            if (rc != Result::NoError) return rc;

            motion = std::move(new_jog);

            executor->SwitchMotion(motion.get());
            return Result::NoError;

            log_ptr_->error("executor is nullptr");
            return Result::Fatal;
        };

        if (!impl_->process_event(EventStartReq{})) {
            if (impl_->is(sml::state<class IDLE>)) return Result::NotEnabled;
            if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
            return Result::ConflictTaskRunning;
        }
        return Result::NoError;
    }

    // ============================================================================
    // MoveSvdJogging — 零空间点动（SVD 基向量法）
    // ============================================================================
    // ============================================================================

    Result Robot::MoveSvdJogging(const std::vector<double> &dim_speeds,
                                 double timeout, double dir_threshold) {
        // ── 分支 1：已有活跃 MoveSvdJog → 旁路喂饭 ──
        if (auto *jog = dynamic_cast<MoveSvdJog *>(motion.get())) {
            const Result rc = jog->FeedSvdJog(dim_speeds);
            if (rc == Result::NoError) return rc;
        }

        // ── 分支 2：其他 motion 正在运行 → 拒绝 ──
        if (motion && IsRunning()) return Result::ConflictTaskRunning;

        // ── 分支 3：全新启动 ──
        data_ready_callback_ = [this, dim_speeds, timeout, dir_threshold]() -> Result {
            auto new_jog = std::make_unique<MoveSvdJog>(
                /*dt=*/0.001, timeout, model.get(), dir_threshold);

            const int n = getJointNum();
            JntArray q_current(static_cast<unsigned int>(n));
            for (int i = 0; i < n; ++i) q_current(i) = getJointPosition(i);
            new_jog->setInitialPosition(q_current);

            Result rc = new_jog->Reset();
            if (rc != Result::NoError) return rc;

            rc = new_jog->FeedSvdJog(dim_speeds);
            if (rc != Result::NoError) return rc;

            motion = std::move(new_jog);

            executor->SwitchMotion(motion.get());
            return Result::NoError;

            log_ptr_->error("executor is nullptr");
            return Result::Fatal;
        };

        if (!impl_->process_event(EventStartReq{})) {
            if (impl_->is(sml::state<class IDLE>)) return Result::NotEnabled;
            if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
            return Result::ConflictTaskRunning;
        }
        return Result::NoError;
    }
} // namespace rocos
