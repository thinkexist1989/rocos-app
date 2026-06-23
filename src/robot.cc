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

#include <rocos_app/robot.h>
#include <kdl_parser/kdl_parser.hpp> // 用于将urdf文件解析为KDL::Tree
#include <rocos_app/motion/move_l_submission.h>
#include <rocos_app/motion/move_c_submission.h>
#include <boost/sml.hpp>
#include <algorithm>
#include <iomanip>
#include <sstream>

#define  MAX_JOINT_NUM 50
#define  EPS 1e-7

namespace {
// 状态定义
class IDLE          {};     // 空闲状态，机器人传感器与执行器就绪，等待使能命令
class STOPPED       {};     // 停止状态，机器人上使能，不会动
class PAUSED        {};     // 暂停状态，机器人暂停在当前位置，等待继续或停止命令
class RUNNING       {};     // 运行状态，机器人正在执行运动
class ERROR_STATE   {};     // 错误状态，任何状态发生错误都转到这个状态[初始状态]
class SERVOING      {};     // 伺服状态，用于高速udp伺服指令发送
class IDENTIFYING   {};     // 动力学参数辨识状态，从STOPPED进入，辨识完成回到STOPPED

// 中间状态定义
class RESETTING     {};
class ENABLING      {};
class DISABLING     {};
class STARTING      {};     // 启动状态，机器人正在启动
class STOPPING      {};
class PAUSING       {};
class CONTINUING    {};

// 事件定义
struct EventResetReq       {};        // 恢复请求
struct EventEnableReq      {};        // 上使能请求
struct EventDisableReq     {};        // 下使能请求
struct EventSuccess        {};        // 成功事件
struct EventStartReq       {};        // 启动请求
struct EventStopReq        {};        // 停止请求
struct EventPauseReq       {};        // 暂停请求
struct EventContinueReq    {};        // 继续请求
struct EventErrorOccurred  {};        // 发生错误
struct EventServoReq       {};        // 伺服请求
struct EventIdentifyReq    {};        // 动力学参数辨识请求
struct EventIsEnabled      {};        //TODO: 检查使能状态事件(临时兼容性，要删除)

namespace sml = boost::sml;

const auto action_start = [](rocos::Robot& robot) {
    robot.on_fsm_start();
};
const auto action_run = [](rocos::Robot& robot) { 
    robot.on_fsm_run();
};
const auto action_pause = [](rocos::Robot& robot) { 
    robot.on_fsm_pause();
};
const auto action_continue = [](rocos::Robot& robot) { 
    robot.on_fsm_continue();
};
const auto action_stop = [](rocos::Robot& robot) { 
    robot.on_fsm_stop();
};
const auto action_reset = [](rocos::Robot& robot) { 
    robot.on_fsm_reset();
};
const auto action_enable = [](rocos::Robot& robot) {
    robot.on_fsm_enable();
};
const auto action_disable = [](rocos::Robot& robot) {
    robot.on_fsm_disable();
};
const auto action_identify = [](rocos::Robot& robot) {
    robot.on_fsm_identify();
};

struct StateMachine {
  auto operator()() const noexcept {
    using namespace sml;
    return make_transition_table(
        // 初始化
       *state<class ERROR_STATE> + event<EventResetReq> = state<class RESETTING>,
        state<class RESETTING> + sml::on_entry<_> / action_reset,
        state<class RESETTING> + event<EventSuccess> = state<class IDLE>,  // 中间状态直接跳转
        state<class RESETTING> + event<EventIsEnabled> = state<class STOPPED>,  // 错误事件直接跳转到错误状态

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
        state<class SERVOING> + event<EventStopReq> = state<class STOPPING>,

        state<class STOPPED> + event<EventIdentifyReq> = state<class IDENTIFYING>,
        state<class IDENTIFYING> + sml::on_entry<_> / action_identify,
        state<class IDENTIFYING> + event<EventSuccess> = state<class STOPPED>,

        state<class RUNNING> + sml::on_entry<_> / action_run,
        state<class RUNNING> + event<EventPauseReq> = state<class PAUSING>,

        state<class PAUSING> + sml::on_entry<_> / action_pause,
        state<class PAUSING> + event<EventSuccess> = state<class PAUSED>,  // 中间状态直接跳转

        state<class PAUSED> + event<EventContinueReq> = state<class CONTINUING>,
        state<class CONTINUING> + sml::on_entry<_> / action_continue,
        state<class CONTINUING> + event<EventSuccess> = state<class RUNNING>,  // 中间状态直接跳转

        state<class PAUSED> + event<EventStopReq> = state<class STOPPING>,
        state<class RUNNING> + event<EventStopReq> = state<class STOPPING>,
        state<class ERROR_STATE> + event<EventStopReq> = state<class STOPPING>,
        state<class STOPPING> + sml::on_entry<_> / action_stop,
        state<class STOPPING> + event<EventSuccess> = state<class STOPPED>,

        // ANY STATE JUMP TO ERROR_STATE
        state<class ERROR_STATE>  + event<EventErrorOccurred> = state<class ERROR_STATE>,  //0
        state<class IDLE>         + event<EventErrorOccurred> = state<class ERROR_STATE>,  //1
        state<class STOPPED>      + event<EventErrorOccurred> = state<class ERROR_STATE>,  //2
        state<class RUNNING>      + event<EventErrorOccurred> = state<class ERROR_STATE>,  //3
        state<class PAUSED>       + event<EventErrorOccurred> = state<class ERROR_STATE>,  //4
        state<class SERVOING>     + event<EventErrorOccurred> = state<class ERROR_STATE>,  //5


        state<class ENABLING>     + event<EventErrorOccurred> = state<class ERROR_STATE>,  //6
        state<class DISABLING>    + event<EventErrorOccurred> = state<class ERROR_STATE>,  //7
        state<class STARTING>     + event<EventErrorOccurred> = state<class ERROR_STATE>,  //8
        state<class STOPPING>     + event<EventErrorOccurred> = state<class ERROR_STATE>,  //9
        state<class PAUSING>      + event<EventErrorOccurred> = state<class ERROR_STATE>,  //10
        state<class CONTINUING>   + event<EventErrorOccurred> = state<class ERROR_STATE>,  //11
        state<class RESETTING>    + event<EventErrorOccurred> = state<class ERROR_STATE>,  //12
        state<class IDENTIFYING>  + event<EventErrorOccurred> = state<class ERROR_STATE>   //13
    );
  }
};

}  // namespace

namespace rocos {
    class Robot::Impl {
    public:
        explicit Impl(Robot& owner) : sm_ {owner} {}

        // 运动线程与 HTTP 线程都会触发事件，必须串行化处理。
        // 用递归锁：action 回调（on_entry）内会再次调用 process_event（如 ENABLING
        // 进入时 on_fsm_enable 再触发 EventSuccess），属同线程重入，普通锁会死锁。
        template<typename Event>
        bool process_event(const Event& event) {
            std::lock_guard<std::recursive_mutex> lock(mtx_);
            return sm_.process_event(event);
        }

        template<typename TState>
        bool is(const TState& s) const {
            std::lock_guard<std::recursive_mutex> lock(mtx_);
            return sm_.is(s);
        }

    private:
        sml::sm<StateMachine> sm_;       // 状态机实例
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

        parseUrdf(urdf_file_path_, base_link_, tip_);

        target_positions_.resize(jnt_num_);
        target_velocities_.resize(jnt_num_);
        target_torques_.resize(jnt_num_);

        max_vel_.resize(jnt_num_);
        max_acc_.resize(jnt_num_);
        max_jerk_.resize(jnt_num_);
        // interp_.resize(jnt_num_);

        for (int i = 0; i < jnt_num_; ++i) {
            pos_[i] = joints_[i]->getPosition();
            target_positions_[i] = pos_[i];

            vel_[i] = joints_[i]->getVelocity();
            target_velocities_[i] = vel_[i];

            target_torques_[i] = joints_[i]->getTorque();

            max_vel_[i]  = joints_[i]->getMaxVel();
            max_acc_[i]  = joints_[i]->getMaxAcc();
            max_jerk_[i] = joints_[i]->getMaxJerk();

        }

        // sun 工具系的初始化
        std::vector<double> tool_param = {0, 0, 0, 0, 0, 0};
        std::vector<double> object_param = {0, 0, 0, 0, 0, 0};
        try
        {
            yaml_node = YAML::LoadFile(cali_yaml_path_);
            tool_param = yaml_node["T_tool_"].as<std::vector<double>>();

            object_param = yaml_node["T_object_"].as<std::vector<double>>();
        }
        catch (const std::exception &e)
        {
            log_ptr_->error("Failed to load calibration yaml file: {} => {}. ",cali_yaml_path_, e.what());
        }
        T_tool_.p.x(tool_param[0]);
        T_tool_.p.y(tool_param[1]);
        T_tool_.p.z(tool_param[2]);
        T_tool_.M = KDL::Rotation::RPY(tool_param[3], tool_param[4], tool_param[5]);

        T_object_.p.x(object_param[0]);
        T_object_.p.y(object_param[1]);
        T_object_.p.z(object_param[2]);
        T_object_.M = KDL::Rotation::RPY(object_param[3], object_param[4], object_param[5]);

        // log_ptr_->info("tool_param: {}", T_tool_.p.z());
        // log_ptr_->info("object_param: {}", T_object_.p.y());

        // 解析逆运动学求解器初始化
        KDL::JntArray q_min(joints_.size());
        KDL::JntArray q_max(joints_.size());
        for (int i = 0; i < joints_.size(); ++i) {
            q_min(i) = joints_[i]->getMinPosLimit();
            q_max(i) = joints_[i]->getMaxPosLimit();
        }

        startMotionThread();

        if(IsEnabled()) {
            impl_->process_event(EventIsEnabled{});  // 模拟初始化成功事件
        }
        else {
            impl_->process_event(EventSuccess{});  // 模拟初始化失败事件
        }

        initializeMotionExecutor();
    }

    Robot::Robot(HardwareInterface *hw,
                 const std::string &urdf_file_path,
                 const std::string &base_link,
                 const std::string &tip
    ) : hw_interface_(hw), urdf_file_path_(urdf_file_path), base_link_(base_link), tip_(tip), pos_(MAX_JOINT_NUM),
        vel_(MAX_JOINT_NUM), acc_(MAX_JOINT_NUM), impl_(std::make_unique<Impl>(*this)) {

        log_ptr_ = Logger::getInstance("Robot");

        impl_->process_event(EventResetReq{});  // 进入初始化状态

    }


    Robot::~Robot() {
        stopMotionThread();

        motion_executor_.reset();
        motion_context_.reset();
        motion_position_controller_.reset();
        motion_fsm_gateway_.reset();
        motion_safety_guard_.reset();

        // Delete logger pointer
        if (log_ptr_) {
            log_ptr_->flush();
            log_ptr_.reset();
        }

        // Delete FSM pack
        impl_.reset();

    }

    std::string Robot::GetRobotState() {
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
            return -1; //TODO: 需要替换为错误码
        }

        return 0;
    }

    int Robot::SetDisabled() {
        log_ptr_->info("Robot Disabling.....");
        if (!impl_->process_event(EventDisableReq{})) {
            log_ptr_->error("Failed to process EventDisableReq.");
            return -1; //TODO: 需要替换为错误码
        }

        return 0;
    }


    bool Robot::IsEnabled() {
        for (int i = 0; i < jnt_num_; i++) {
            //使能检查
            if (joints_[i]->getDriveState() != DriveState::OperationEnabled) {
                log_ptr_->error("joints[{}] is not Enabled", i);
                return false;
            }
        }
        return true;
    }

    bool Robot::IsDisabled() {
        for (int i = 0; i < jnt_num_; i++) {
            //使能检查
            if (joints_[i]->getDriveState() == DriveState::OperationEnabled) {
                log_ptr_->warn("joints[{}] is Enabled", i);
                return false;
            }
        }
        return true;
    }

    //////////////////////////////////////////////////////////////////////////////

    bool Robot::parseUrdf(const std::string &urdf_file_path,
                          const std::string &base_link,
                          const std::string &tip) {

        KDL::Tree tree;
        if (!kdl_parser::treeFromFile(urdf_file_path, tree)) {
            // 解析失败
            log_ptr_->error("Could not extract urdf to kdl tree!");

            return false;
        }
        if (!loader.loadFromYAML("/opt/rocos/yaml/robotDH.yaml")) 
        {
            log_ptr_->error("Failed to load DH parameters!");

            if (!kinematics_.setChain(tree, base_link, tip)) 
            {
                log_ptr_->error("Could not set kinematic chain!");
                return false;
            }
        }
        else {
            // 打印DH参数信息
            loader.printDHParameters();
            // 获取构建的运动学链
            Chain raw_chain = loader.getChain();

            if(!kinematics_.setChain(raw_chain)) {
                log_ptr_->error("Could not set kinematic chain from DH parameters!");
                return false;
            }
            log_ptr_->info("Kinematic chain set from DH parameters successfully.");

        }

        if (!parseDriveParamsFromUrdf(urdf_file_path)) {

            log_ptr_->error("Could not parse drive parameters!");

            return false;
        }

        KDL::JntArray q_min(joints_.size());
        KDL::JntArray q_max(joints_.size());

        for (int i = 0; i < joints_.size(); ++i) {
            q_min(i) = joints_[i]->getMinPosLimit();
            q_max(i) = joints_[i]->getMaxPosLimit();
        }

        kinematics_.setPosLimits(q_min, q_max);
        kinematics_.Initialize(); //初始化，构建IK solver;
        
        log_ptr_->info("Kinematics initialized successfully.");


        //TODO: 临时加入 by think
        if (!dynamics_.setChain(tree, base_link, tip))
        {
            log_ptr_->error("Could not set dynamics chain!");
            return false;
        }
        dynamics_.Initialize();

        log_ptr_->info("Dynamics initialized successfully.");



        return true;
    }

    //! \brief 从URDF中解析驱动器相关参数，这个函数只在parseUrdf()内部调用，在调用前，已经解析好KDL::Chain
    //! \param urdf_file_path urdf文件路径
    //! \return
    bool Robot::parseDriveParamsFromUrdf(const std::string &urdf_file_path) {
        log_ptr_->info("Parsing drive parameters from urdf file: {}", urdf_file_path);
        jnt_num_ = kinematics_.getChain().getNrOfJoints();

        joints_.clear(); // vector<Drive>清空
        std::ostringstream joint_table;
        joint_table << "\n"
                    << std::left
                    << std::setw(4) << "idx"
                    << std::setw(20) << "name"
                    << std::setw(6) << "id"
                    << std::setw(10) << "lower"
                    << std::setw(10) << "upper"
                    << std::setw(9) << "vel"
                    << std::setw(9) << "acc"
                    << std::setw(9) << "jerk"
                    << std::setw(10) << "ratio"
                    << std::setw(9) << "offset"
                    << std::setw(13) << "cnt/unit"
                    << std::setw(16) << "torque/unit"
                    << "unit\n"
                    << std::string(125, '-') << "\n"
                    << std::fixed << std::setprecision(4);
        int parsed_joint_idx = 0;

        tinyxml2::XMLDocument xml_doc;
        xml_doc.LoadFile(urdf_file_path.c_str()); // 解析urdf文件

        auto robot = xml_doc.FirstChildElement("robot");

        for (auto element = robot->FirstChildElement("joint"); element; element = element->NextSiblingElement(
                "joint")) {
            for (int i = 0; i < jnt_num_; ++i) {

                if (element->Attribute("name") == kinematics_.getChain().getSegment(i).getJoint().getName()) {
                    const auto *joint_name = element->Attribute("name");

                    auto hw = element->FirstChildElement("hardware");

                    auto id = hw->IntAttribute("id", -1); // 对应的硬件ID，若没指定默认为-1

                    auto jnt_ptr = std::make_shared<Drive>(hw_interface_, id); //获取相应硬件指针

                    jnt_ptr->setName(joint_name); //设置驱动器名称
                    jnt_ptr->setMode(ModeOfOperation::CyclicSynchronousPositionMode); //驱动器模式设置为CSP

                    auto limit = hw->FirstChildElement("limit");

                    const auto lower = limit->DoubleAttribute("lower", -M_PI);
                    const auto upper = limit->DoubleAttribute("upper", M_PI);
                    const auto vel = limit->DoubleAttribute("vel", 1.0);
                    const auto acc = limit->DoubleAttribute("acc", 10.0);
                    const auto jerk = limit->DoubleAttribute("jerk", 100.0);

                    jnt_ptr->setMinPosLimit(lower);
                    jnt_ptr->setMaxPosLimit(upper);
                    jnt_ptr->setMaxVel(vel);
                    jnt_ptr->setMaxAcc(acc);
                    jnt_ptr->setMaxJerk(jerk);

                    auto trans = hw->FirstChildElement("transform");

                    const auto ratio = trans->DoubleAttribute("ratio", 1.0);
                    const auto offset_pos_cnt = trans->IntAttribute("offset_pos_cnt", 0);
                    const auto cnt_per_unit = trans->DoubleAttribute("cnt_per_unit", 1.0);
                    const auto torque_per_unit = trans->DoubleAttribute("torque_per_unit", 1.0);
                    const auto *user_unit_name = trans->Attribute("user_unit_name") ? trans->Attribute("user_unit_name") : "rad";

                    jnt_ptr->setRatio(ratio);
                    jnt_ptr->setPosZeroOffset(offset_pos_cnt);
                    jnt_ptr->setCntPerUnit(cnt_per_unit);
                    jnt_ptr->setTorquePerUnit(torque_per_unit);
                    jnt_ptr->setUserUnitName(user_unit_name);

                    joint_table << std::left
                                << std::setw(4) << parsed_joint_idx
                                << std::setw(20) << joint_name
                                << std::setw(6) << id
                                << std::setw(10) << lower
                                << std::setw(10) << upper
                                << std::setw(9) << vel
                                << std::setw(9) << acc
                                << std::setw(9) << jerk
                                << std::setw(10) << ratio
                                << std::setw(9) << offset_pos_cnt
                                << std::setw(13) << cnt_per_unit
                                << std::setw(16) << torque_per_unit
                                << user_unit_name << "\n";
                    ++parsed_joint_idx;

                    joints_.push_back(jnt_ptr); // 将对应ID的hardware放入joints数组
                }
            }
        }

        log_ptr_->info("All joint parameters:{}", joint_table.str());

        return true;
    }

    bool Robot::setWorkMode(WorkMode mode) {

        switch (work_mode_) {
            case WorkMode::Position:
                break;
            case WorkMode::EeAdmitTeach:
                break;
            case WorkMode::JntAdmitTeach:
                break;
            case WorkMode::JntImp:
                break;
            case WorkMode::CartImp:
                break;
            default:
                break;
        }

        for (int i = 0; i < 10; i++)
            hw_interface_->waitForSignal(5);

        //TODO: 机器人只能在停止状态切换工作模式，有状态机，需要删掉
        if (isMotionRunning()) {
            log_ptr_->error("Robot is not stopped!");
            return false;
        }

        work_mode_ = mode;

        switch (mode) {
            case WorkMode::Position:
                log_ptr_->info("Robot work mode is set to Position");
                break;
            case WorkMode::EeAdmitTeach:
                log_ptr_->info("Robot work mode is set to EeAdmitTeach");
                break;
            case WorkMode::JntAdmitTeach:
                log_ptr_->info("Robot work mode is set to JntAdmitTeach");
                break;
            case WorkMode::JntImp:
                log_ptr_->info("Robot work mode is set to JntImp");
                break;
            case WorkMode::CartImp:
                log_ptr_->info("Robot work mode is set to CartImp");
                break;
            default:
                break;
        }

        return true;
    }

    bool Robot::enterRunning() {
        // 仅当处于 STOPPED 时 EventStartReq 才会被状态机处理（STOPPED→STARTING→RUNNING，
        // 中间态经 on_entry 回调自动推进）。其他状态下事件不被处理，process_event 返回
        // false —— 这就是原子的状态门控：RUNNING/PAUSED 等状态下新运动被直接拒绝，
        // 无需额外的标志位，也不存在 check-then-act 竞态。
        if (!requestMotionStart()) {
            log_ptr_->error("无法开始运动：机器人不处于 STOPPED 状态（当前：{}）", GetRobotState());
            return false;
        }
        return true;
    }

    void Robot::enterStopped() {
        // 运动线程结束或中止时调用，从 RUNNING/PAUSED/SERVOING/ERROR 回到 STOPPED
        // （STOPPING 中间态经 on_fsm_stop 自动推进）。若当前已是 STOPPED/IDLE，
        // EventStopReq 不被处理，视为无害空操作。
        requestMotionStop();
    }

    bool Robot::isMotionRunning() const {
        // 是否有运动正占用机器人（FSM 处于 RUNNING）。替代旧的 is_running_motion 读取。
        return impl_->is(sml::state<class RUNNING>);
    }

    bool Robot::requestMotionStart() {
        return impl_->process_event(EventStartReq{});
    }

    bool Robot::requestMotionPause() {
        return impl_->process_event(EventPauseReq{});
    }

    bool Robot::requestMotionContinue() {
        return impl_->process_event(EventContinueReq{});
    }

    bool Robot::requestMotionStop() {
        return impl_->process_event(EventStopReq{});
    }

    bool Robot::notifyMotionError() {
        return impl_->process_event(EventErrorOccurred{});
    }

    void Robot::waitControlCycle() {
        if (hw_interface_) {
            hw_interface_->waitForSignal(0);
        }
    }

    void Robot::initializeMotionExecutor() {
        motion::MotionSafetyLimits limits;
        limits.min_position.reserve(jnt_num_);
        limits.max_position.reserve(jnt_num_);
        limits.max_command_velocity.reserve(jnt_num_);
        limits.max_following_error.reserve(jnt_num_);

        for (int i = 0; i < jnt_num_; ++i) {
            const double min_position = joints_[i]->getMinPosLimit();
            const double max_position = joints_[i]->getMaxPosLimit();
            const double max_velocity = std::max(joints_[i]->getMaxVel(), EPS);
            limits.min_position.push_back(min_position);
            limits.max_position.push_back(max_position);
            limits.max_command_velocity.push_back(max_velocity);
            limits.max_following_error.push_back(
                std::max(max_position - min_position, 1.0));
        }

        motion_safety_guard_ =
            std::make_unique<motion::MotionSafetyGuard>(std::move(limits));
        motion_fsm_gateway_ =
            std::make_unique<motion::BasicRobotFsmGateway<Robot>>(*this);
        motion_position_controller_ =
            std::make_unique<motion::PositionController>();
        motion_context_ =
            std::make_unique<motion::RobotMotionContext<Robot>>(
                *this, *motion_safety_guard_, DELTA_T);

        // 初始化 ModelProvider 的 FK/IK 回调
        model_provider_.kinematics.forwardKinematics =
            [this](const std::vector<double>& q, std::vector<double>& pose_out) -> bool {
                if (static_cast<int>(q.size()) != jnt_num_) { return false; }
                KDL::JntArray q_kdl(jnt_num_);
                for (int i = 0; i < jnt_num_; ++i) { q_kdl(i) = q[i]; }
                KDL::Frame frame;
                if (kinematics_.JntToCart(q_kdl, frame) < 0) { return false; }
                pose_out.resize(7);
                pose_out[0] = frame.p.x();
                pose_out[1] = frame.p.y();
                pose_out[2] = frame.p.z();
                frame.M.GetQuaternion(pose_out[3], pose_out[4], pose_out[5], pose_out[6]);
                return true;
            };
        model_provider_.kinematics.inverseKinematics =
            [this](const std::vector<double>& q_seed,
                   const std::vector<double>& target_pose,
                   std::vector<double>& q_out) -> bool {
                if (target_pose.size() < 7) { return false; }
                KDL::JntArray q_kdl(jnt_num_);
                for (int i = 0; i < jnt_num_; ++i) {
                    q_kdl(i) = (i < static_cast<int>(q_seed.size())) ? q_seed[i] : 0.0;
                }
                KDL::Frame target;
                target.p = KDL::Vector(target_pose[0], target_pose[1], target_pose[2]);
                target.M = KDL::Rotation::Quaternion(
                    target_pose[3], target_pose[4], target_pose[5], target_pose[6]);
                KDL::JntArray result;
                if (kinematics_.CartToJnt(q_kdl, target, result) < 0) { return false; }
                q_out.resize(jnt_num_);
                for (int i = 0; i < jnt_num_; ++i) { q_out[i] = result(i); }
                return true;
            };
        model_provider_.kinematics.getDof = [this]() { return jnt_num_; };

        motion_executor_ =
            std::make_unique<motion::MotionExecutor>(
                *motion_fsm_gateway_,
                *motion_position_controller_,
                *motion_context_,
                model_provider_);
    }

    void Robot::setEnabled() {
        for_each(joints_.begin(), joints_.end(),
                 [=](std::shared_ptr<Drive> &d) { d->setEnabled(false); }); // 将抱闸设置为同时开启，不阻塞


        // set a temporary time point to prevent getting caught in an infinite loop
        auto driveStateChangeStartTimePoint = std::chrono::system_clock::now();

        outerloop:
        for (;;) {
            for (const auto &joint: joints_) {
                // First check timeout
                auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now() - driveStateChangeStartTimePoint);
                if (duration_us.count() > 150000) { //wait for 100ms  TODO: configuration_.driveStateChangeMaxTimeout
                    log_ptr_->warn("It takes too long ({} ms) to switch state!", duration_us.count() / 1000.0);
                    break;
                }

                if (joint->getDriveState() != DriveState::OperationEnabled) {
                    goto outerloop;
                }

            }
            break;
        }
    }

    void Robot::setDisabled() {
        for_each(joints_.begin(), joints_.end(),
                 [=](std::shared_ptr<Drive> &d) { d->setDisabled(false); }); // 将抱闸设置为同时开启，不阻塞



        // set a temporary time point to prevent getting caught in an infinite loop
        auto driveStateChangeStartTimePoint = std::chrono::system_clock::now();

        outerloop:
        for (;;) {
            for (const auto &joint: joints_) {
                // First check timeout
                auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now() - driveStateChangeStartTimePoint);
                if (duration_us.count() > 150000) { //wait for 100ms  TODO: configuration_.driveStateChangeMaxTimeout
                    log_ptr_->warn("It takes too long ({} ms) to switch state!", duration_us.count() / 1000.0);
                    break;
                }

                if (joint->getDriveState() != DriveState::SwitchOnDisabled) {
                    goto outerloop;
                }

            }
            break;
        }
    }

    //TODO: 这里需要修改
    void Robot::startMotionThread() {
        motion_thread_stop_requested_ = false;
        otg_motion_thread_ =
                std::make_shared<std::thread>(&Robot::motionThreadHandler, this);
    }
    //TODO: 这里需要修改
    void Robot::stopMotionThread() {
        motion_thread_stop_requested_ = true;
        if (otg_motion_thread_ && otg_motion_thread_->joinable()) {
            otg_motion_thread_->join();
        }
        otg_motion_thread_.reset();
    }
    //TODO: 这里需要修改，主要为了更新笛卡尔
    void Robot::motionThreadHandler() {

        std::ostringstream thread_id_ss;
        thread_id_ss << std::this_thread::get_id();
        log_ptr_->info("Motion thread is running on thread {}", thread_id_ss.str());

        //** vector 数组大小初始化 **//
        target_positions_.resize(jnt_num_);
        target_velocities_.resize(jnt_num_);
        target_torques_.resize(jnt_num_);
        // pos_.resize(jnt_num_);
        // vel_.resize(jnt_num_);
        // acc_.resize(jnt_num_);
        max_vel_.resize(jnt_num_);
        max_acc_.resize(jnt_num_);
        max_jerk_.resize(jnt_num_);

        //**-------------------------------**//
        //** vector 数组数值初始化 **//
        for (int i = 0; i < jnt_num_; ++i) {
            pos_[i] = joints_[i]->getPosition();
            target_positions_[i] = pos_[i];

            vel_[i] = joints_[i]->getVelocity();
            target_velocities_[i] = vel_[i];

            target_torques_[i] = joints_[i]->getTorque();

        }
        //**-------------------------------**//

        while (!motion_thread_stop_requested_) {  // while start

            hw_interface_->waitForSignal(9);

            //!< Update Flange State
            std::lock_guard<std::mutex> lock(mtx);  // 自动获取互斥锁，防止更新坐标系时读取，会读取空的Frame
            for (int i = 0; i < jnt_num_; i++)
            {
                if (joints_[i]->getDriveState() != DriveState::OperationEnabled)
                {
                    pos_[i] = joints_[i]->getPosition();
                    vel_[i] = 0;

                    joints_[i]->setPosition(pos_[i]); // 要把数据同步给共享内存
                    joints_[i]->setVelocity(vel_[i]);
                }
            }
            updateCartesianInfo(); //TODO: 更新笛卡尔信息
        }

        // process before exit:
    }

    /////// Motion Command /////////////

    //TODO: ======================MoveJ调用逻辑===========================
    int Robot::MoveJ(JntArray q, double speed, double acceleration, double time,
                     double radius, bool asynchronous) {

        if (!motion_executor_) {
            initializeMotionExecutor();
        }

        if (q.rows() != static_cast<unsigned int>(jnt_num_)) {
            log_ptr_->error("MoveJ target dimension does not match robot joints");
            return static_cast<int>(motion::ErrorCode::UnmatchedJointsNumber);
        }

        if (time != 0.0 || radius != 0.0) {
            log_ptr_->error("MoveJ executor path does not support time/radius yet");
            return static_cast<int>(motion::ErrorCode::IllegalParameter);
        }

        for (int i{0}; i < jnt_num_; i++) {
            if (q(i) > joints_[i]->getMaxPosLimit() ||
                q(i) < joints_[i]->getMinPosLimit()) {
                log_ptr_->error("MoveJ position command is out of range");
                return static_cast<int>(motion::ErrorCode::PosLimit);
            }
            if (speed > joints_[i]->getMaxVel() ||
                speed < (-1) * joints_[i]->getMaxVel()) {
                log_ptr_->error("MoveJ velocity command is out of range");
                return static_cast<int>(motion::ErrorCode::SpeedLimit);
            }
            if (acceleration > joints_[i]->getMaxAcc() ||
                acceleration < (-1) * joints_[i]->getMaxAcc()) {
                log_ptr_->error("MoveJ acceleration command is out of range");
                return static_cast<int>(motion::ErrorCode::AccLimit);
            }
            if (joints_[i]->getDriveState() != DriveState::OperationEnabled) {
                log_ptr_->error("MoveJ joint[{}] is not operation enabled", i);
                return static_cast<int>(motion::ErrorCode::NotAllAtOpState);
            }
            if (!(joints_[i]->getMode() == ModeOfOperation::CyclicSynchronousPositionMode ||
                  joints_[i]->getMode() == ModeOfOperation::CyclicSynchronousVelocityMode)) {
                log_ptr_->error("MoveJ不支持关节[{}]的当前模式 :{}", i, static_cast<int>(joints_[i]->getMode()));
                return static_cast<int>(motion::ErrorCode::CallingConflictError);
            }
        }

        std::vector<double> target_position(jnt_num_);
        for (int i = 0; i < jnt_num_; ++i) {
            target_position[i] = q(i);
        }

        const auto submit_result = motion::submitMoveJ(
            *this,
            *motion_executor_,
            target_position,
            std::abs(speed),
            std::abs(acceleration),
            DELTA_T);
        if (!submit_result.success) {
            log_ptr_->error("MoveJ executor submit failed: {}", submit_result.message);
            return submit_result.api_error_code;
        }

        if (!asynchronous) {
            while (motion_executor_->hasActiveCommand()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            const auto last_error = motion_executor_->lastError();
            if (!last_error.success) {
                log_ptr_->error("MoveJ executor failed: {}", last_error.message);
                return last_error.api_error_code;
            }
        }

        return 0;
    }

    int Robot::PauseMotion() {
        if (!motion_executor_) {
            initializeMotionExecutor();
        }

        const auto result = motion_executor_->pause();
        if (!result.success) {
            log_ptr_->error("PauseMotion failed: {}", result.message);
            return result.api_error_code;
        }
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
            if (motion_executor_->currentTaskStatus() ==
                motion::MotionTaskStatus::Paused) {
                return 0;
            }
            if (!motion_executor_->hasActiveCommand()) {
                const auto last_error = motion_executor_->lastError();
                return last_error.success ? 0 : last_error.api_error_code;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return static_cast<int>(motion::ErrorCode::NotAllAtOpState);
    }

    int Robot::ResumeMotion() {
        if (!motion_executor_) {
            initializeMotionExecutor();
        }

        const auto result = motion_executor_->resume();
        if (!result.success) {
            log_ptr_->error("ResumeMotion failed: {}", result.message);
            return result.api_error_code;
        }
        return 0;
    }

    int Robot::StopMotion() {
        if (!motion_executor_) {
            initializeMotionExecutor();
        }

        const auto result = motion_executor_->stop();
        if (!result.success) {
            log_ptr_->error("StopMotion failed: {}", result.message);
            return result.api_error_code;
        }
        return 0;
    }

    int Robot::MoveJ_IK(Frame pose, double speed, double acceleration, double time,
                        double radius, bool asynchronous) {

        std::ostringstream ss;
        ss << pose;
        log_ptr_->info("MoveJ_IK pose: {}", ss.str());

        JntArray q_init(jnt_num_);
        JntArray q_target(jnt_num_);
        for (int i = 0; i < jnt_num_; i++) {
            q_init.data[i] = pos_[i];
            q_target.data[i] = pos_[i];
        }
        if (kinematics_.CartToJnt(q_init, pose, q_target) < 0) {
            log_ptr_->error(" CartToJnt failed");
            return -1;
        }
        return MoveJ(q_target, speed, acceleration, time, radius, asynchronous);
    }
    //TODO: =========================MoveJ===============================

    //TODO: ======================MoveL调用逻辑===========================
    int Robot::MoveL(Frame pose, double speed, double acceleration, double time,
                     double radius, bool asynchronous, int max_running_count) {

        bool all_pos_mode{true};  //假设全部关节位置模式
        bool all_vel_mode{true};  //假设全部关节速度模式

        for (int i{0}; i < jnt_num_; i++) {
            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousPositionMode)
                all_pos_mode = false;

            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousVelocityMode)
                all_vel_mode = false;
        }

        // time==0 && radius==0 → executor 路径（MoveLCommand + S-curve profile）
        if (time == 0.0 && radius == 0.0) {
            if (!motion_executor_) {
                initializeMotionExecutor();
            }

            for (int i{0}; i < jnt_num_; i++) {
                if (joints_[i]->getDriveState() != DriveState::OperationEnabled) {
                    log_ptr_->error("MoveL joint[{}] is not operation enabled", i);
                    return static_cast<int>(motion::ErrorCode::NotAllAtOpState);
                }
            }

            const auto submit_result = motion::submitMoveL(
                *this,
                *motion_executor_,
                pose,
                speed,
                acceleration,
                DELTA_T);
            if (!submit_result.success) {
                log_ptr_->error("MoveL executor submit failed: {}", submit_result.message);
                return submit_result.api_error_code;
            }

            if (!asynchronous) {
                while (motion_executor_->hasActiveCommand()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                const auto last_error = motion_executor_->lastError();
                if (!last_error.success) {
                    log_ptr_->error("MoveL executor failed: {}", last_error.message);
                    return last_error.api_error_code;
                }
            }
            return 0;
        }

        // 旧路径（time/radius 不为零，或速度模式）
        // if (all_pos_mode)
        //     return MoveL_pos(pose, speed, acceleration, time, radius, asynchronous, max_running_count);
        // else if (all_vel_mode)
        //     return MoveL_vel(pose, speed, acceleration, time, radius, asynchronous, max_running_count);
        // else {
        //     log_ptr_->error("某关节为既不是位置模式也不是速度模式！");
        //     return -1;
        // }
    }

    //TODO: ========================MoveL=============================

    int Robot::MoveL_FK(JntArray q, double speed, double acceleration, double time,
                        double radius, bool asynchronous) {
        KDL::Frame target;
        kinematics_.JntToCart(q, target);
        
        std::ostringstream ss;
        ss << target;
        log_ptr_->info("Target pose is: {}", ss.str());

        return MoveL(target, speed, acceleration, time, radius, asynchronous);
    }

}  // namespace rocos
