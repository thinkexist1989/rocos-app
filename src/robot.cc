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
#include <boost/sml.hpp>
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
        state<class RESETTING>    + event<EventErrorOccurred> = state<class ERROR_STATE>   //12
    );
  }
};

}  // namespace

namespace rocos {
    class Robot::Impl {
    public:
        explicit Impl(Robot& owner) : sm_ {owner} {}

        template<typename Event>
        bool process_event(const Event& event) {
            return sm_.process_event(event);
        }

        template<typename TState>
        bool is(const TState& s) const {
            return sm_.is(s);
        }

    private:
        sml::sm<StateMachine> sm_;  // 状态机实例
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

    }
    void Robot::on_fsm_run() {

    }
    void Robot::on_fsm_stop() {

    }
    void Robot::on_fsm_pause() {

    }
    void Robot::on_fsm_continue() {

    }
    void Robot::on_fsm_reset() {
        log_ptr_->info("Robot is initializing...");

        parseUrdf(urdf_file_path_, base_link_, tip_);

        jointNum = jnt_num_;

        target_positions_.resize(jnt_num_);
        target_positions_prev_.resize(jnt_num_);
        target_velocities_.resize(jnt_num_);
        target_torques_.resize(jnt_num_);

        max_vel_.resize(jnt_num_);
        max_acc_.resize(jnt_num_);
        max_jerk_.resize(jnt_num_);
        interp_.resize(jnt_num_);

        for (int i = 0; i < jnt_num_; ++i) {
            pos_[i] = joints_[i]->getPosition();
            target_positions_[i] = pos_[i];
            target_positions_prev_[i] = pos_[i];

            vel_[i] = joints_[i]->getVelocity();
            target_velocities_[i] = vel_[i];

            target_torques_[i] = joints_[i]->getTorque();

            max_vel_[i]  = joints_[i]->getMaxVel();
            max_acc_[i]  = joints_[i]->getMaxAcc();
            max_jerk_[i] = joints_[i]->getMaxJerk();

            if (profile_type_ == trapezoid) {
                interp_[i] = new Trapezoid;
            } else if (profile_type_ == doubleS) {
                interp_[i] = new DoubleS;
            }
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
    }

    Robot::Robot(HardwareInterface *hw,
                 const string &urdf_file_path,
                 const string &base_link,
                 const string &tip
    ) : hw_interface_(hw), urdf_file_path_(urdf_file_path), base_link_(base_link), tip_(tip), pos_(MAX_JOINT_NUM),
        vel_(MAX_JOINT_NUM), acc_(MAX_JOINT_NUM), impl_(std::make_unique<Impl>(*this)) {

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


    }

    int Robot::SetDisabled() {
        log_ptr_->info("Robot Disabling.....");
        if (!impl_->process_event(EventDisableReq{})) {
            log_ptr_->error("Failed to process EventDisableReq.");
            return -1; //TODO: 需要替换为错误码
        }
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

    bool Robot::parseUrdf(const string &urdf_file_path,
                          const string &base_link,
                          const string &tip) {

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
    bool Robot::parseDriveParamsFromUrdf(const string &urdf_file_path) {
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

    // TODO: 切换HW指针，目前未实现
    bool Robot::switchHW(HardwareInterface *hw) { return false; }


    bool Robot::setWorkMode(WorkMode mode) {

        switch (work_mode_) {
            case WorkMode::Position:
                break;
            case WorkMode::EeAdmitTeach:
                this->stop_admittance_teaching();
                break;
            case WorkMode::JntAdmitTeach:
                this->stop_joint_admittance_teaching();
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
        if (getRunState() == RunState::Running) {
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
                this->admittance_teaching(true);
                break;
            case WorkMode::JntAdmitTeach:
                log_ptr_->info("Robot work mode is set to JntAdmitTeach");
                this->joint_admittance_teaching(true);
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

    bool Robot::setRunState(RunState state) {
        run_state_ = state;

        switch (state) {
            case RunState::Disabled:
                is_running_motion = false;
                break;
            case RunState::Stopped:
                is_running_motion = false;
                break;
            case RunState::Running:
                is_running_motion = true;
                break;
            default:
                break;
        }

        return true;
    }

    void Robot::setEnabled() {
        setRunState(RunState::Stopped); //TODO: 需要删除

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
        setRunState(RunState::Disabled); //TODO: 需要删除

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
        otg_motion_thread_ =
                std::make_shared<std::thread>(&Robot::motionThreadHandler, this);
    }
    //TODO: 这里需要修改
    void Robot::stopMotionThread() {
        setRunState(RunState::Stopped);

        // otg_motion_thread_->interrupt();
        otg_motion_thread_->join();  //等待运动线程结束
    }
    //TODO: 这里需要修改，主要为了更新笛卡尔
    void Robot::motionThreadHandler() {

        std::ostringstream thread_id_ss;
        thread_id_ss << std::this_thread::get_id();
        log_ptr_->info("Motion thread is running on thread {}", thread_id_ss.str());

        //** vector 数组大小初始化 **//
        target_positions_.resize(jnt_num_);
        target_positions_prev_.resize(jnt_num_);
        target_velocities_.resize(jnt_num_);
        target_torques_.resize(jnt_num_);
        // pos_.resize(jnt_num_);
        // vel_.resize(jnt_num_);
        // acc_.resize(jnt_num_);
        max_vel_.resize(jnt_num_);
        max_acc_.resize(jnt_num_);
        max_jerk_.resize(jnt_num_);
        interp_.resize(jnt_num_);
        need_plan_.resize(jnt_num_, false);
        //**-------------------------------**//
        //** vector 数组数值初始化 **//
        for (int i = 0; i < jnt_num_; ++i) {
            pos_[i] = joints_[i]->getPosition();
            target_positions_[i] = pos_[i];
            target_positions_prev_[i] = pos_[i];

            vel_[i] = joints_[i]->getVelocity();
            target_velocities_[i] = vel_[i];

            target_torques_[i] = joints_[i]->getTorque();

            if (profile_type_ == trapezoid) {
                interp_[i] = new Trapezoid;
            } else if (profile_type_ == doubleS) {
                interp_[i] = new DoubleS;
            }
        }
        //**-------------------------------**//

        while (true) {  // while start

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
            updateCartesianInfo();
        }

        // process before exit:
    }

    void Robot::moveJ(const vector<double> &target_pos,
                      const vector<double> &target_vel,
                      Robot::Synchronization sync) {
        if ((target_pos.size() != jnt_num_) || (target_vel.size() != jnt_num_)) {
            log_ptr_->error("MoveJ => Error Input Vector Size!");
            return;
        }

        sync_ = sync;

        target_positions_ = target_pos;
        target_velocities_ = target_vel;

        need_plan_.resize(jnt_num_, true);
    }

    /// \brief 停止单轴运动
    /// \param id 轴ID
    void Robot::stopSingleAxis(int id) {
        double dt = fabs(vel_[id]) / max_acc_[id];  // 所需要的减速时间
        target_positions_[id] =
                pos_[id] +
                dt * vel_[id] / 2.0;  // TODO：这个减速段计算有问题
        target_velocities_[id] = 0.0;
        least_motion_time_ = 0.0;

        auto sync = sync_;
        sync_ = SYNC_NONE;  //停止时候就不需要同步了

        log_ptr_->info("max_acc: {}; pos: {}; vel: {}", max_acc_[id], pos_[id].load(), vel_[id].load());
        log_ptr_->info("dt: {}; target_positions: {}", dt, target_positions_[id]);

        need_plan_[id] = true;
    }

    void Robot::stopMultiAxis() {
        sync_ = SYNC_NONE;  //停止时候就不需要同步了

        double wait_time = 0.0;

        for (int id = 0; id < jnt_num_; ++id) {
            double dt = fabs(vel_[id]) / max_acc_[id];  // 所需要的减速时间
            target_positions_[id] =
                    pos_[id] +
                    2 * (dt * vel_[id] / 2.0);  // TODO：这个减速段计算有问题
            target_velocities_[id] = 0.0;
            least_motion_time_ = 0.0;

            log_ptr_->info("max_acc: {}; pos: {}; vel: {}", max_acc_[id], pos_[id].load(), vel_[id].load());
            log_ptr_->info("dt: {}; target_positions: {}", dt, target_positions_[id]);

            need_plan_[id] = true;

            wait_time = wait_time <= dt ? wait_time : dt;
        }
    }

    /// 设置单轴运动
    /// \param id 轴ID
    /// \param pos 目标位置
    /// \param vel 目标速度
    /// \param max_vel 最大速度
    /// \param max_acc 最大加速度
    /// \param max_jerk 最大加加速度
    /// \param least_time 最短运行时间
    void Robot::moveSingleAxis(int id, double pos, double vel, double max_vel,
                               double max_acc, double max_jerk, double least_time) {
        target_positions_[id] = pos;
        target_velocities_[id] = vel;

        if (max_vel != -1) max_vel_[id] = max_vel;
        if (max_acc != -1) max_acc_[id] = max_acc;
        if (max_jerk != -1) max_jerk_[id] = max_jerk;
        if (least_time != -1) least_motion_time_ = least_time;

        need_plan_[id] = true;
    }

    /// 设置多轴运动
    /// \param target_pos 目标位置
    /// \param target_vel 目标速度
    /// \param max_vel 最大速度
    /// \param max_acc 最大加速度
    /// \param max_jerk 最大加加速度
    /// \param least_time 最短运行时间
    void Robot::moveMultiAxis(const vector<double> &target_pos,
                              const vector<double> &target_vel,
                              const vector<double> &max_vel,
                              const vector<double> &max_acc,
                              const vector<double> &max_jerk, double least_time) {
        if ((target_pos.size() != jnt_num_) || (target_vel.size() != jnt_num_) ||
            (max_vel.size() != jnt_num_) || (max_acc.size() != jnt_num_) ||
            (max_jerk.size() != jnt_num_)) {
            log_ptr_->error("moveMultiAxis => Error Input Vector Size!");
            return;
        }

        for (int id = 0; id < jnt_num_; ++id) {
            target_positions_[id] = target_pos[id];
            target_velocities_[id] = target_vel[id];

            if (max_vel[id] != -1) max_vel_[id] = max_vel[id];
            if (max_acc[id] != -1) max_acc_[id] = max_acc[id];
            if (max_jerk[id] != -1) max_jerk_[id] = max_jerk[id];

            need_plan_[id] = true;
        }

        if (least_time != -1) least_motion_time_ = least_time;
    }


    /////// Motion Command /////////////

    //TODO: ======================MoveJ调用逻辑===========================
    int Robot::MoveJ(JntArray q, double speed, double acceleration, double time,
                     double radius, bool asynchronous) {

        if (CheckBeforeMove(q, speed, acceleration, time, radius) < 0) {
            log_ptr_->error("given parameters is invalid");
            return -1;
        }

        for (int i{0}; i < jointNum; i++) {
            if (!(joints_[i]->getMode() == ModeOfOperation::CyclicSynchronousPositionMode ||
                  joints_[i]->getMode() == ModeOfOperation::CyclicSynchronousVelocityMode)) {
                log_ptr_->error("MoveJ不支持关节[{}]的当前模式 :{}", i, static_cast<int>(joints_[i]->getMode()));
                return -1;
            }
        }

        if (is_running_motion)  //最大异步执行一条任务
        {
            log_ptr_->error(" Motion is still running and waiting for it to finish");
            return -1;
        } else {
            // is_running_motion = true;
            setRunState(RunState::Running);
        }

        if (motion_thread_) {
            motion_thread_->join();
            motion_thread_ = nullptr;
        }

        if (asynchronous)  //异步执行
        {
            motion_thread_.reset(new std::thread{&Robot::RunMoveJ, this, q,
                                                   speed, acceleration, time,
                                                   radius});
        } else  //同步执行
        {
            motion_thread_.reset(new std::thread{&Robot::RunMoveJ, this, q,
                                                   speed, acceleration, time,
                                                   radius});
            motion_thread_->join();
            motion_thread_ = nullptr;
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

        for (int i{0}; i < jointNum; i++) {
            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousPositionMode)
                all_pos_mode = false;

            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousVelocityMode)
                all_vel_mode = false;
        }

        if (all_pos_mode)
            return MoveL_pos(pose, speed, acceleration, time, radius, asynchronous, max_running_count);
        else if (all_vel_mode)
            return MoveL_vel(pose, speed, acceleration, time, radius, asynchronous, max_running_count);
        else {
            log_ptr_->error("某关节为既不是位置模式也不是速度模式！");
            return -1;
        }
    }

    int Robot::MoveL_pos(Frame pose, double speed, double acceleration, double time,
                         double radius, bool asynchronous, int max_running_count) {
        for (int i{0}; i < jointNum; i++) {
            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousPositionMode) {
                log_ptr_->error(" 需要关节[{}]进入位置伺服模式", i);
                return -1;
            }
        }

        if (max_running_count < 1) {
            log_ptr_->error("max_running_count parameters must be greater than 0");
            return -1;
        }

        if (CheckBeforeMove(pose, speed, acceleration, time, radius) < 0) {
            log_ptr_->error("given parameters is invalid");
            return -1;
        }

        if (is_running_motion)  //最大一条任务异步执行
        {
            log_ptr_->error(" Motion is still running and waiting for it to finish");
            return -1;
        } else {
            //is_running_motion = true;
            setRunState(RunState::Running);
        }

        if (motion_thread_) {
            motion_thread_->join();
            motion_thread_ = nullptr;
        }

        //** 变量初始化 **//
        traj_.clear();
        KDL::JntArray q_init(jnt_num_);
        KDL::JntArray q_target(jnt_num_);
        std::vector<double> max_step;
        std::vector<KDL::Frame> traj_target;
        KDL::Frame frame_init;
        JntToCart(JC_helper::vector_2_JntArray(pos_), frame_init);
        int traj_count{0};
        //**-------------------------------**//

        for (int i = 0; i < jnt_num_; i++) {
            q_init(i) = pos_[i];
            q_target(i) = pos_[i];
            max_step.push_back(max_vel_[i] * DELTA_T);
        }

        if (JC_helper::link_trajectory(traj_target, frame_init, pose, speed, acceleration) < 0) {
            log_ptr_->error("link trajectory planning fail ");

            setRunState(RunState::Stopped); //TODO: RunState

            return -1;
        }

        //** 轨迹IK计算，计算失败，可以重新计算，有最大计算次数限制{max_running_count} **//
        int ik_count{0};
        for (; ik_count < max_running_count; ik_count++) {
            try {
                for (int i = 0; i < jnt_num_; i++) {
                    q_init(i) = pos_[i];
                }
                traj_.clear();

                log_ptr_->info("---------------------------------------");
                //sun
                // 尝试改为解析解接口
                for (const auto &target: traj_target) {

                    if (kinematics_.CartToJnt(q_init, target, q_target) < 0) {
                        log_ptr_->error(" CartToJnt failed on the {} times", ik_count);
                        throw -1;
                    }
                    //*防止奇异位置速度激增
                    for (int i = 0; i < jnt_num_; i++) {
                        if (abs(q_target(i) - q_init(i)) > max_step[i]) {
                            log_ptr_->error("joint[{}] speed is too fast", i);
                            log_ptr_->error("target speed = {} and max_step = {}", abs(q_target(i) - q_init(i)), max_step[i]);
                            throw -2;
                        }
                    }
                    //**-------------------------------**//
                    q_init = q_target;
                    traj_.push_back(q_target);

                }
                //在此处时，代表规划成功
                break;

            }
            catch (int flag_error) {
                switch (flag_error) {
                    case -1:
                        break;
                    case -2:
                        break;
                    default:
                        log_ptr_->error("Undefined error!");
                        setRunState(RunState::Stopped);
                        return -1;
                }
            }
            catch (...) {
                log_ptr_->error("Undefined error!");
                setRunState(RunState::Stopped);
                return -1;
            }

        }

        if (ik_count == max_running_count) {
            log_ptr_->error("CartToJnt still failed even after {} attempts", max_running_count);
            setRunState(RunState::Stopped);
            return -1;
        }

        //**-------------------------------**//

        if (asynchronous)  //异步执行
        {
            motion_thread_.reset(new std::thread{&Robot::RunMoveL, this, std::ref(traj_)});

            setRunState(RunState::Running); //TODO: RunState
        } else  //同步执行
        {
            motion_thread_.reset(new std::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            motion_thread_->join();
            motion_thread_ = nullptr;

            setRunState(RunState::Stopped); //TODO: RunState
        }

        return 0;
    }

    int Robot::MoveL_vel(Frame pose, double speed, double acceleration, double time,
                         double radius, bool asynchronous, int max_running_count) {
        for (int i{0}; i < jointNum; i++) {
            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousVelocityMode) {
                log_ptr_->error(" 需要关节[{}]进入速度伺服模式", i);
                return -1;
            }
        }

        if (time) {
            log_ptr_->error(" time not supported yet");
            return -1;
        }

        if (max_running_count < 1) {
            log_ptr_->error("max_running_count parameters must be greater than 0");
            return -1;
        }

        if (CheckBeforeMove(pose, speed, acceleration, time, radius) < 0) {
            log_ptr_->error("given parameters is invalid");
            return -1;
        }

        if (is_running_motion)  //最大一条任务异步执行 //TODO: RunState
        {
            log_ptr_->error(" Motion is still running and waiting for it to finish");
            return -1;
        } else {

            setRunState(RunState::Running); //TODO: RunState

        }

        if (motion_thread_) {
            motion_thread_->join();
            motion_thread_ = nullptr;
        }



        //** 变量初始化 **//
        KDL::JntArray q_init(jnt_num_);
        KDL::JntArray joint_vel(jnt_num_);
        std::vector<KDL::Twist> vel_target;
        KDL::Frame frame_current;
        JntToCart(JC_helper::vector_2_JntArray(pos_), frame_current);
        KDL::ChainIkSolverVel_pinv _ik_vel{kinematics_.getChain()};
        //**-------------------------------**//

        if (JC_helper::link_trajectory(vel_target, frame_current, pose, speed, acceleration) < 0) {
            log_ptr_->error("link trajectory planning fail ");

            setRunState(RunState::Stopped); //TODO: RunState

            return -1;
        }

        //** 轨迹IK计算，计算失败，可以重新计算，有最大计算次数限制{max_running_count} **//
        int ik_count{0};
        for (; ik_count < max_running_count; ik_count++) {
            try {
                for (int i = 0; i < jnt_num_; i++) {
                    q_init(i) = pos_[i];
                }
                traj_.clear();

                log_ptr_->info("---------------------------------------");

                for (const auto &target: vel_target) {
                    //!雅克比默认参考系为base,参考点为flange
                    if (_ik_vel.CartToJnt(q_init, target, joint_vel) != 0) {
                        log_ptr_->error("雅克比计算错误,错误号：{}", _ik_vel.CartToJnt(q_init, target, joint_vel));
                        throw -1;
                    }

                    //*防止奇异位置速度激增
                    for (int i = 0; i < jnt_num_; i++) {
                        if (abs(joint_vel(i)) > max_vel_[i]) {
                            log_ptr_->error("joint[{}] speed is too fast", i);
                            log_ptr_->error("target speed = {} and max_vel_ = {}", abs(joint_vel(i)), max_vel_[i]);
                            throw -2;
                        }

                    }
                    //**-------------------------------**//

                    //** 位置保护，雅克比计算需要位置检查 **//
                    q_init.data = q_init.data + joint_vel.data * DELTA_T;
                    for (int i = 0; i < jnt_num_; i++) {
                        if (q_init(i) > joints_[i]->getMaxPosLimit() ||
                            q_init(i) < joints_[i]->getMinPosLimit()) {
                            log_ptr_->error("关节[{}] 超过关节限位，求解失败", i);
                            throw -3;
                        }
                    }
                    //**-------------------------------**//

                    traj_.push_back(joint_vel);

                }
                //在此处时，代表规划成功
                break;

            }
            catch (int flag_error) {
                switch (flag_error) {
                    case -1:
                        break;
                    case -2:
                        break;
                    case -3:
                        break;
                    default:
                        log_ptr_->error("Undefined error!");
                        setRunState(RunState::Stopped); //TODO: RunState

                        return -1;
                }
            }
            catch (...) {
                log_ptr_->error("Undefined error!");
                setRunState(RunState::Stopped); //TODO: RunState

                return -1;
            }

        }

        if (ik_count == max_running_count) {
            log_ptr_->error("CartToJnt still failed even after {} attempts", max_running_count);

            setRunState(RunState::Stopped); //TODO: RunState

            return -1;
        }

        //**-------------------------------**//

        if (asynchronous)  //异步执行
        {
            motion_thread_.reset(new std::thread{&Robot::RunMoveL, this, std::ref(traj_)});

            setRunState(RunState::Running); //TODO: RunState

        } else  //同步执行
        {
            motion_thread_.reset(new std::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            motion_thread_->join();
            motion_thread_ = nullptr;

            setRunState(RunState::Stopped); //TODO: RunState

        }

        return 0;
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


    //TODO: ======================MoveC调用逻辑===========================
    int Robot::MoveC(Frame pose_via, Frame pose_to, double speed,
                     double acceleration, double time, double radius,
                     Robot::OrientationMode mode, bool asynchronous, int max_running_count) {
        bool all_pos_mode{true};  //假设全部关节位置模式
        bool all_vel_mode{true};  //假设全部关节速度模式

        for (int i{0}; i < jointNum; i++) {
            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousPositionMode)
                all_pos_mode = false;

            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousVelocityMode)
                all_vel_mode = false;
        }

        if (all_pos_mode)
            return MoveC_pos(pose_via, pose_to, speed, acceleration, time, radius, mode, asynchronous,
                             max_running_count);
        else if (all_vel_mode)
            return MoveC_vel(pose_via, pose_to, speed, acceleration, time, radius, mode, asynchronous,
                             max_running_count);
        else {
            log_ptr_->error("某关节为既不是位置模式也不是速度模式！");
            return -1;
        }

    }

    int Robot::MoveC(const KDL::Frame &center, double theta, int axiz, double speed,
                     double acceleration, double time, double radius,
                     Robot::OrientationMode mode, bool asynchronous, int max_running_count) {
        bool all_pos_mode{true};  //假设全部关节位置模式
        bool all_vel_mode{true};  //假设全部关节速度模式

        for (int i{0}; i < jointNum; i++) {
            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousPositionMode)
                all_pos_mode = false;

            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousVelocityMode)
                all_vel_mode = false;
        }

        if (all_pos_mode)
            return MoveC_pos(center, theta, axiz, speed, acceleration, time, radius, mode, asynchronous,
                             max_running_count);
        else if (all_vel_mode)
            return MoveC_vel(center, theta, axiz, speed, acceleration, time, radius, mode, asynchronous,
                             max_running_count);
        else {
            log_ptr_->error("某关节为既不是位置模式也不是速度模式！");
            return -1;
        }
    }

    int Robot::MoveC_pos(Frame pose_via, Frame pose_to, double speed,
                         double acceleration, double time, double radius,
                         Robot::OrientationMode mode, bool asynchronous, int max_running_count) {
        for (int i{0}; i < jointNum; i++) {
            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousPositionMode) {
                log_ptr_->error(" 需要关节[{}]进入位置伺服模式", i);
                return -1;
            }
        }

        if (time) {
            log_ptr_->error(" time not supported yet");
            return -1;
        }
        if (max_running_count < 1) {
            log_ptr_->error("max_running_count parameters must be greater than 0");
            return -1;
        }
        if (CheckBeforeMove(pose_via, speed, acceleration, time, radius) < 0) {
            log_ptr_->error("given parameters is invalid");
            return -1;
        }
        if (CheckBeforeMove(pose_to, speed, acceleration, time, radius) < 0) {
            log_ptr_->error("given parameters is invalid");
            return -1;
        }
        if (is_running_motion)  //最大一条任务异步执行
        {
            log_ptr_->error(" Motion is still running and waiting for it to finish");
            return -1;
        } else {
            // is_running_motion = true;
            setRunState(RunState::Running);
        }

        if (motion_thread_) {
            motion_thread_->join();
            motion_thread_ = nullptr;
        }

        //** 变量初始化 **//
        traj_.clear();
        KDL::JntArray q_init(jnt_num_);
        KDL::JntArray q_target(jnt_num_);
        KDL::Frame frame_init;
        JntToCart(JC_helper::vector_2_JntArray(pos_), frame_init);
        std::vector<double> max_step;
        bool orientation_fixed = mode == Robot::OrientationMode::FIXED;
        std::vector<KDL::Frame> traj_target;
        //**-------------------------------**//

        for (int i = 0; i < jnt_num_; i++) {
            q_init(i) = pos_[i];
            q_target(i) = pos_[i];
            max_step.push_back(max_vel_[i] * DELTA_T);
        }

        if (JC_helper::circle_trajectory(traj_target, frame_init, pose_via, pose_to, speed, acceleration,
                                         orientation_fixed) < 0) {
            log_ptr_->error("circle trajectory planning fail ");
            // is_running_motion =false;
            setRunState(RunState::Stopped);
            return -1;
        }

        //** 轨迹IK计算，计算失败，可以重新计算，有最大计算次数限制{max_running_count} **//
        int ik_count{0};
        for (; ik_count < max_running_count; ik_count++) {
            try {
                for (int i = 0; i < jnt_num_; i++) {
                    q_init(i) = pos_[i];
                }
                traj_.clear();

                log_ptr_->info("---------------------------------------");

                for (const auto &target: traj_target) {
                    if (kinematics_.CartToJnt(q_init, target, q_target) < 0) {
                        throw -1;
                    }
                    //** 防止奇异位置速度激增 **//
                    for (int i = 0; i < jnt_num_; i++) {
                        if (abs(q_target(i) - q_init(i)) > max_step[i]) {
                            log_ptr_->error("joint[{}] speep is too  fast", i);
                            log_ptr_->error("target speed = {} and  max_step={}", abs(q_target(i) - q_init(i)), max_step[i]);
                            throw -2;
                        }
                    }
                    //**-------------------------------**//
                    q_init = q_target;
                    traj_.push_back(q_target);
                }

                //在此处时，代表规划成功
                break;
            }
            catch (int flag_error) {
                switch (flag_error) {
                    case -1:
                        log_ptr_->error(" CartToJnt failed on the {} times", ik_count);
                        break;
                    case -2:
                        break;
                    default:
                        log_ptr_->error("Undefined error!");
                        // is_running_motion = false;
                        setRunState(RunState::Stopped);
                        return -1;
                }
            }
            catch (...) {
                log_ptr_->error("Undefined error!");
                // is_running_motion =false;
                setRunState(RunState::Stopped);
                return -1;
            }
        }

        if (ik_count == max_running_count) {
            log_ptr_->error("CartToJnt still failed even after {} attempts", max_running_count);
            // is_running_motion =false;
            setRunState(RunState::Stopped);
            return -1;
        }

        //**-------------------------------**//

        if (asynchronous)  //异步执行
        {
            motion_thread_.reset(new std::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            // is_running_motion = true;
            setRunState(RunState::Running);
        } else  //同步执行
        {
            motion_thread_.reset(new std::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            motion_thread_->join();
            motion_thread_ = nullptr;
            // is_running_motion = false;
            setRunState(RunState::Stopped);
        }

        return 0;
    }


    int Robot::MoveC_pos(const KDL::Frame &center, double theta, int axiz, double speed,
                         double acceleration, double time, double radius,
                         Robot::OrientationMode mode, bool asynchronous, int max_running_count) {
        for (int i{0}; i < jointNum; i++) {
            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousPositionMode) {
                log_ptr_->error(" 需要关节[{}]进入位置伺服模式", i);
                return -1;
            }
        }

        if (time) {
            log_ptr_->error(" time not supported yet");
            return -1;
        }
        if (max_running_count < 1) {
            log_ptr_->error("max_running_count parameters must be greater than 0");
            return -1;
        }
        if (CheckBeforeMove(flange_, speed, acceleration, time, radius) < 0) {
            log_ptr_->error("given parameters is invalid");
            return -1;
        }

        if (is_running_motion)  //最大一条任务异步执行
        {
            log_ptr_->error(" Motion is still running and waiting for it to finish");
            return -1;
        } else {
            // is_running_motion = true;
            setRunState(RunState::Running);
        }

        if (motion_thread_) {
            motion_thread_->join();
            motion_thread_ = nullptr;
        }

        //** 变量初始化 **//
        traj_.clear();
        KDL::JntArray q_init(jnt_num_);
        KDL::JntArray q_target(jnt_num_);
        KDL::Frame frame_init;
        JntToCart(JC_helper::vector_2_JntArray(pos_), frame_init);
        std::vector<double> max_step;
        bool orientation_fixed = mode == Robot::OrientationMode::FIXED;
        std::vector<KDL::Frame> traj_target;
        //**-------------------------------**//

        for (int i = 0; i < jnt_num_; i++) {
            q_init(i) = pos_[i];
            q_target(i) = pos_[i];
            max_step.push_back(max_vel_[i] * DELTA_T);
        }

        if (JC_helper::circle_trajectory(traj_target, frame_init, center, theta, axiz, speed, acceleration,
                                         orientation_fixed) < 0) {
            log_ptr_->error("circle trajectory planning fail ");
            // is_running_motion =false;
            setRunState(RunState::Stopped);
            return -1;
        }

        //** 轨迹IK计算，计算失败，可以重新计算，有最大计算次数限制{max_running_count} **//
        int ik_count{0};
        for (; ik_count < max_running_count; ik_count++) {
            try {
                for (int i = 0; i < jnt_num_; i++) {
                    q_init(i) = pos_[i];
                }
                traj_.clear();

                log_ptr_->info("---------------------------------------");

                for (const auto &target: traj_target) {
                    if (kinematics_.CartToJnt(q_init, target, q_target) < 0) {
                        throw -1;
                    }
                    //** 防止奇异位置速度激增 **//
                    for (int i = 0; i < jnt_num_; i++) {
                        if (abs(q_target(i) - q_init(i)) > max_step[i]) {
                            log_ptr_->error("joint[{}] speep is too  fast", i);
                            log_ptr_->error("target speed = {} and  max_step={}", abs(q_target(i) - q_init(i)), max_step[i]);
                            throw -2;
                        }
                    }
                    //**-------------------------------**//
                    q_init = q_target;
                    traj_.push_back(q_target);
                }

                //在此处时，代表规划成功
                break;
            }
            catch (int flag_error) {
                switch (flag_error) {
                    case -1:
                        log_ptr_->error(" CartToJnt failed on the {} times", ik_count);
                        break;
                    case -2:
                        log_ptr_->error("Undefined error!");
                        break;
                    default:
                        log_ptr_->error("Undefined error!");
                        // is_running_motion = false;
                        setRunState(RunState::Stopped);
                        return -1;
                }
            }
            catch (...) {
                log_ptr_->error("Undefined error!");
                // is_running_motion =false;
                setRunState(RunState::Stopped);
                return -1;
            }
        }

        if (ik_count == max_running_count) {
            log_ptr_->error("CartToJnt still failed even after {} attempts", max_running_count);
            // is_running_motion =false;
            setRunState(RunState::Stopped);
            return -1;
        }

        //**-------------------------------**//

        if (asynchronous)  //异步执行
        {
            motion_thread_.reset(new std::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            // is_running_motion = true;
            setRunState(RunState::Running);
        } else  //同步执行
        {
            motion_thread_.reset(new std::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            motion_thread_->join();
            motion_thread_ = nullptr;
            // is_running_motion = false;
            setRunState(RunState::Stopped);
        }

        return 0;
    }


    int Robot::MoveC_vel(Frame pose_via, Frame pose_to, double speed,
                         double acceleration, double time, double radius,
                         Robot::OrientationMode mode, bool asynchronous, int max_running_count) {
        for (int i{0}; i < jointNum; i++) {
            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousVelocityMode) {
                log_ptr_->error(" 需要关节[{}]进入速度伺服模式", i);
                return -1;
            }
        }

        if (time) {
            log_ptr_->error(" time not supported yet");
            return -1;
        }
        if (max_running_count < 1) {
            log_ptr_->error("max_running_count parameters must be greater than 0");
            return -1;
        }
        if (CheckBeforeMove(pose_via, speed, acceleration, time, radius) < 0) {
            log_ptr_->error("given parameters is invalid");
            return -1;
        }
        if (CheckBeforeMove(pose_to, speed, acceleration, time, radius) < 0) {
            log_ptr_->error("given parameters is invalid");
            return -1;
        }
        if (is_running_motion)  //最大一条任务异步执行
        {
            log_ptr_->error(" Motion is still running and waiting for it to finish");
            return -1;
        } else {
            // is_running_motion = true;
            setRunState(RunState::Running);
        }

        if (motion_thread_) {
            motion_thread_->join();
            motion_thread_ = nullptr;
        }


        //** 变量初始化 **//
        KDL::JntArray q_init(jnt_num_);
        KDL::JntArray joint_vel(jnt_num_);
        KDL::Frame current_frame;
        JntToCart(JC_helper::vector_2_JntArray(pos_), current_frame);
        bool orientation_fixed = mode == Robot::OrientationMode::FIXED;
        std::vector<KDL::Twist> traj_vel_target;
        KDL::ChainIkSolverVel_pinv _ik_vel{kinematics_.getChain()};
        //**-------------------------------**//

        if (JC_helper::circle_trajectory(traj_vel_target, current_frame, pose_via, pose_to, speed, acceleration,
                                         orientation_fixed) < 0) {
            log_ptr_->error("circle trajectory planning fail ");
            is_running_motion = false;
            return -1;
        }

        //** 轨迹IK计算，计算失败，可以重新计算，有最大计算次数限制{max_running_count} **//
        int ik_count{0};
        for (; ik_count < max_running_count; ik_count++) {
            try {
                for (int i = 0; i < jnt_num_; i++) {
                    q_init(i) = pos_[i];
                }
                traj_.clear();

                log_ptr_->info("---------------------------------------");

                for (const auto &target: traj_vel_target) {
                    //!雅克比默认参考系为base,参考点为flange
                    if (_ik_vel.CartToJnt(q_init, target, joint_vel) != 0) {
                        log_ptr_->error("雅克比计算错误,错误号：{}", _ik_vel.CartToJnt(q_init, target, joint_vel));
                        throw -1;
                    }

                    //*防止奇异位置速度激增
                    for (int i = 0; i < jnt_num_; i++) {
                        if (abs(joint_vel(i)) > max_vel_[i]) {
                            log_ptr_->error("joint[{}] speep is too  fast", i);
                            log_ptr_->error("target speed = {} and  max_vel_={}", abs(joint_vel(i)), max_vel_[i]);
                            throw -2;
                        }

                    }
                    //**-------------------------------**//

                    //** 位置保护，雅克比计算需要位置检查 **//
                    q_init.data = q_init.data + joint_vel.data * DELTA_T;
                    for (int i = 0; i < jnt_num_; i++) {
                        if (q_init(i) > joints_[i]->getMaxPosLimit() ||
                            q_init(i) < joints_[i]->getMinPosLimit()) {
                            log_ptr_->error("关节[{}] 超过关节限位，求解失败", i);
                            throw -3;
                        }
                    }
                    //**-------------------------------**//

                    traj_.push_back(joint_vel);
                }

                //在此处时，代表规划成功
                break;
            }
            catch (int flag_error) {
                switch (flag_error) {
                    case -1:
                        break;
                    case -2:
                        break;
                    case -3:
                        break;
                    default:
                        log_ptr_->error("Undefined error!");
                        // is_running_motion = false;
                        setRunState(RunState::Stopped);
                        return -1;
                }
            }
            catch (...) {
                log_ptr_->error("Undefined error!");
                // is_running_motion =false;
                setRunState(RunState::Stopped);
                return -1;
            }
        }

        if (ik_count == max_running_count) {
            log_ptr_->error("CartToJnt still failed even after {} attempts", max_running_count);
            // is_running_motion =false;
            setRunState(RunState::Stopped);
            return -1;
        }

        //**-------------------------------**//

        if (asynchronous)  //异步执行
        {
            motion_thread_.reset(new std::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            // is_running_motion = true;
            setRunState(RunState::Running);
        } else  //同步执行
        {
            motion_thread_.reset(new std::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            motion_thread_->join();
            motion_thread_ = nullptr;
            // is_running_motion = false;
            setRunState(RunState::Stopped);
        }

        return 0;
    }


    int Robot::MoveC_vel(const KDL::Frame &center, double theta, int axiz, double speed,
                         double acceleration, double time, double radius,
                         Robot::OrientationMode mode, bool asynchronous, int max_running_count) {
        for (int i{0}; i < jointNum; i++) {
            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousVelocityMode) {
                log_ptr_->error(" 需要关节[{}]进入速度伺服模式", i);
                return -1;
            }
        }

        if (time) {
            log_ptr_->error(" time not supported yet");
            return -1;
        }
        if (max_running_count < 1) {
            log_ptr_->error("max_running_count parameters must be greater than 0");
            return -1;
        }
        if (CheckBeforeMove(flange_, speed, acceleration, time, radius) < 0) {
            log_ptr_->error("given parameters is invalid");
            return -1;
        }

        if (is_running_motion)  //最大一条任务异步执行
        {
            log_ptr_->error(" Motion is still running and waiting for it to finish");
            return -1;
        } else {
            // is_running_motion = true;
            setRunState(RunState::Running);
        }

        if (motion_thread_) {
            motion_thread_->join();
            motion_thread_ = nullptr;
        }


        //** 变量初始化 **//
        KDL::JntArray q_init(jnt_num_);
        KDL::JntArray joint_vel(jnt_num_);
        KDL::Frame current_frame;
        JntToCart(JC_helper::vector_2_JntArray(pos_), current_frame);
        bool orientation_fixed = mode == Robot::OrientationMode::FIXED;
        std::vector<KDL::Twist> traj_vel_target;
        KDL::ChainIkSolverVel_pinv _ik_vel{kinematics_.getChain()};
        //**-------------------------------**//

        if (JC_helper::circle_trajectory(traj_vel_target, current_frame, center, theta, axiz, speed, acceleration,
                                         orientation_fixed) < 0) {
            log_ptr_->error("circle trajectory planning fail ");
            // is_running_motion =false;
            setRunState(RunState::Stopped);
            return -1;
        }

        //** 轨迹IK计算，计算失败，可以重新计算，有最大计算次数限制{max_running_count} **//
        int ik_count{0};
        for (; ik_count < max_running_count; ik_count++) {
            try {
                for (int i = 0; i < jnt_num_; i++) {
                    q_init(i) = pos_[i];
                }
                traj_.clear();

                log_ptr_->info("---------------------------------------");

                for (const auto &target: traj_vel_target) {
                    //!雅克比默认参考系为base,参考点为flange
                    if (_ik_vel.CartToJnt(q_init, target, joint_vel) != 0) {
                        log_ptr_->error("雅克比计算错误,错误号：{}", _ik_vel.CartToJnt(q_init, target, joint_vel));
                        throw -1;
                    }

                    //*防止奇异位置速度激增
                    for (int i = 0; i < jnt_num_; i++) {
                        if (abs(joint_vel(i)) > max_vel_[i]) {
                            log_ptr_->error("joint[{}] speep is too  fast", i);
                            log_ptr_->error("target speed = {} and  max_vel_={}", abs(joint_vel(i)), max_vel_[i]);
                            throw -2;
                        }

                    }
                    //**-------------------------------**//

                    //** 位置保护，雅克比计算需要位置检查 **//
                    q_init.data = q_init.data + joint_vel.data * DELTA_T;
                    for (int i = 0; i < jnt_num_; i++) {
                        if (q_init(i) > joints_[i]->getMaxPosLimit() ||
                            q_init(i) < joints_[i]->getMinPosLimit()) {
                            log_ptr_->error("关节[{}] 超过关节限位，求解失败", i);
                            throw -3;
                        }
                    }
                    //**-------------------------------**//

                    traj_.push_back(joint_vel);
                }

                //在此处时，代表规划成功
                break;
            }
            catch (int flag_error) {
                switch (flag_error) {
                    case -1:
                        break;
                    case -2:
                        break;
                    case -3:
                        break;
                    default:
                        log_ptr_->error("Undefined error!");
                        // is_running_motion = false;
                        setRunState(RunState::Stopped);
                        return -1;
                }
            }
            catch (...) {
                log_ptr_->error("Undefined error!");
                // is_running_motion =false;
                setRunState(RunState::Stopped);
                return -1;
            }
        }

        if (ik_count == max_running_count) {
            log_ptr_->error("CartToJnt still failed even after {} attempts", max_running_count);
            // is_running_motion =false;
            setRunState(RunState::Stopped);
            return -1;
        }

        //**-------------------------------**//

        if (asynchronous)  //异步执行
        {
            motion_thread_.reset(new std::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            // is_running_motion = true;
            setRunState(RunState::Running);
        } else  //同步执行
        {
            motion_thread_.reset(new std::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            motion_thread_->join();
            motion_thread_ = nullptr;
            // is_running_motion = false;
            setRunState(RunState::Running);
        }

        return 0;
    }

    //TODO: ======================MoveC===========================



    int Robot::MoveP(Frame pose, double speed, double acceleration, double time,
                     double radius, bool asynchronous) {
        log_ptr_->error("have not complicated yet");
        return 0;
    }

    int Robot::MovePath(const Path &path, bool asynchronous) {
        log_ptr_->error("have not complicated yet");
        return 0;
    }

    int Robot::MultiMoveL(const std::vector<KDL::Frame> &point, std::vector<double> bound_dist,
                          std::vector<double> max_path_v, std::vector<double> max_path_a, bool asynchronous,
                          int max_running_count) {

        for (int i{0}; i < jointNum; i++) {
            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousPositionMode) {
                log_ptr_->error(" 需要关节[{}]进入位置伺服模式", i);
                return -1;
            }
        }

        if (is_running_motion)  //最大一条任务异步执行
        {
            log_ptr_->error(" Motion is still running and waiting for it to finish");
            return -1;
        }

        if (motion_thread_) {
            motion_thread_->join();
            motion_thread_ = nullptr;
        }

        //** 变量初始化 **//
        std::vector<KDL::Frame> traj_target;
        std::vector<int> traj_index;
        KDL::Frame Cart_point;
        JntToCart(JC_helper::vector_2_JntArray(pos_), Cart_point);
        std::vector<size_t> vector_size{point.size(), bound_dist.size(), max_path_v.size(), max_path_a.size()};
        std::vector<double> max_step;
        KDL::JntArray q_init(jnt_num_);
        KDL::JntArray q_target(jnt_num_);

        //**-------------------------------**//

        //** 程序初始化 **//
        for (int i = 0; i < jnt_num_; i++) {
            q_init(i) = pos_[i];
            q_target(i) = q_init(i);
            max_step.push_back(max_vel_[i] * DELTA_T);
        }


        for (const auto &i: vector_size) {
            if (i != point.size()) {
                log_ptr_->error("MultiMoveL(): All vectors must be the same size");
                return -1;
            }
        }
        //**-------------------------------**//

        //** 规划 **//

        if (point.size() == 0) {
            log_ptr_->error("MultiMoveL(): point size is at least one or more");
            return -1;
        }
            //一段轨迹不存在圆弧过渡处理
        else if (point.size() == 1) {
            log_ptr_->info("***************第1次规划***************");
            if (JC_helper::link_trajectory(traj_target, Cart_point, point[0], 0, 0, max_path_v[0], max_path_a[0]) < 0) {
                log_ptr_->error("MultiMoveL(): given parameters is invalid in the 1th planning");
                return -1;
            }
            traj_index.push_back(traj_target.size());
        } else {
            KDL::Frame motion_frame_1;
            KDL::Frame motion_frame_2;
            double motion_v_1;
            double motion_v_2;
            int success{0};

            log_ptr_->info("***************第1次规划***************");
            success = JC_helper::multilink_trajectory(traj_target, Cart_point, point[0], point[1], motion_frame_1, 0,
                                                      motion_v_1, bound_dist[0], max_path_v[0], max_path_a[0],
                                                      max_path_v[1]);
            if (success < 0) {
                log_ptr_->error(" given parameters is invalid in the 1th planning ");
                return -1;
            }
            traj_index.push_back(traj_target.size());

            for (int i = 1; i < (point.size() - 1); i++) {
                log_ptr_->info("***************第{}次规划***************", i + 1);
                success = JC_helper::multilink_trajectory(traj_target, motion_frame_1, point[i], point[i + 1],
                                                          motion_frame_2, motion_v_1, motion_v_2, bound_dist[i],
                                                          max_path_v[i], max_path_a[i], max_path_v[i + 1]);
                if (success < 0) {
                    log_ptr_->error("given parameters is invalid in the {}th planning", i + 1);
                    return -1;
                }
                motion_frame_1 = motion_frame_2;
                motion_v_1 = motion_v_2;
                traj_index.push_back(traj_target.size());
            }

            log_ptr_->info("***************第{}次规划***************", point.size());
            success = JC_helper::link_trajectory(traj_target, motion_frame_1, point.back(), motion_v_1, 0,
                                                 max_path_v.back(), max_path_a.back());
            if (success < 0) {
                log_ptr_->error(" given parameters is invalid in the last of planning ");
                return -1;
            }
            traj_index.push_back(traj_target.size());
        }
        log_ptr_->info("***************规划全部完成***************");

        //**-------------------------------**//

        //** 轨迹IK计算，计算失败，可以重新计算，有最大计算次数限制{max_running_count} **//
        int ik_count{0};
        int CartToJnt_count{0};//指示第几次逆解
        int traj_current_pos{0};  //表示当前正处理第几段轨迹

        for (; ik_count < max_running_count; ik_count++) {
            try {
                for (int i = 0; i < jnt_num_; i++) {
                    q_init(i) = pos_[i];
                }
                traj_.clear();
                CartToJnt_count = 0;
                traj_current_pos = 0;
                log_ptr_->info("---------------------------------------");

                for (const auto &target: traj_target) {
                    if (kinematics_.CartToJnt(q_init, target, q_target) < 0) {
                        throw -1;
                    }

                    //*防止奇异位置速度激增
                    for (int i = 0; i < jnt_num_; i++) {
                        if (abs(q_target(i) - q_init(i)) > max_step[i]) {
                            for (int i = 0; i < traj_index.size(); i++) {
                                if (CartToJnt_count < traj_index[i]) {
                                    traj_current_pos = i + 1;
                                    break;
                                }
                            }

                            log_ptr_->error("joint[{}] speed is too fast on the {}TH trajectory", i, traj_current_pos);
                            log_ptr_->error("target speed = {} and max_step = {}", abs(q_target(i) - q_init(i)), max_step[i]);
                            log_ptr_->error("q_target({}) = {}", i, q_target(i) * 180 / M_PI);
                            log_ptr_->error("q_init({}) = {}", i, q_init(i) * 180 / M_PI);

                            throw -2;
                        }
                    }
                    //**-------------------------------**//

                    q_init = q_target;
                    traj_.push_back(q_target);

                    CartToJnt_count++;
                }

                break;//在此处时，代表规划成功
            }

            catch (int flag_error) {
                switch (flag_error) {
                    case -1: {
                        for (int i = 0; i < traj_index.size(); i++) {
                            if (CartToJnt_count < traj_index[i]) {
                                traj_current_pos = i + 1;
                                break;
                            }
                        }
                        log_ptr_->error("CartToJnt failed on the {}TH trajectory, please choose other interpolate Points", traj_current_pos);

                        break;
                    }
                    case -2:
                        break;
                    default:
                        log_ptr_->error("Undefined error!");
                        // is_running_motion = false;
                        setRunState(RunState::Stopped);
                        return -1;
                }
            }
            catch (...) {
                log_ptr_->error("Undefined error!");
                // is_running_motion = false;
                setRunState(RunState::Stopped);
                return -1;
            }
        }

        if (ik_count == max_running_count) {
            log_ptr_->error("\n\nCartToJnt still failed even after {} attempts", max_running_count);
            // is_running_motion = false;
            setRunState(RunState::Stopped);
            return -1;
        }


        if (asynchronous)  //异步执行
        {
            motion_thread_.reset(new std::thread{&Robot::RunMultiMoveL, this, std::ref(traj_)});
            // is_running_motion = true;
            setRunState(RunState::Running);
        } else  //同步执行
        {
            motion_thread_.reset(new std::thread{&Robot::RunMultiMoveL, this, std::ref(traj_)});
            motion_thread_->join();
            motion_thread_ = nullptr;
            // is_running_motion = false;
            setRunState(RunState::Stopped);
        }

        return 0;
    }

    int Robot::Dragging(DRAGGING_FLAG flag, DRAGGING_DIRRECTION dir, double max_speed, double max_acceleration) {
        //** 变量初始化 **//
        static std::atomic<bool> _dragging_finished_flag{true};
        static JC_helper::SmartServo_Joint _SmartServo_Joint{&_dragging_finished_flag};
        static JC_helper::SmartServo_Cartesian _SmartServo_Cartesian{&_dragging_finished_flag, kinematics_.getChain()};
        static JC_helper::SmartServo_Nullspace _SmartServo_Nullsapace{&_dragging_finished_flag, kinematics_.getChain()};
        static std::shared_ptr<std::thread> _thread_planning{nullptr};
        KDL::JntArray target_joint{static_cast< unsigned int >( jnt_num_ )};
        KDL::Frame target_frame{};
        int index{static_cast< int >( flag )};
        static DRAGGING_TYPE index_type;
        static DRAGGING_TYPE last_index_type;
        static DRAGGING_DIRRECTION last_dir;
        int res{-1};
        constexpr double vector_speed_scale{0.1};
        constexpr double rotation_speed_scale{0.2};
        //**-------------------------------**//

        //** 命令有效性检查 **//
        if (index < 0) {
            log_ptr_->error("未定义指令：{}", index);
            return -1;
        }

        if (index <= static_cast< int >( DRAGGING_FLAG::J6 ))  //当前命令类型为关节空间
        {
            //只检查速度、加速度,关节位置指令这里不检查，如果超过范围，则为最大/小关节值
            if (CheckBeforeMove(JntArray{static_cast< unsigned int >( jnt_num_ )}, max_speed, max_acceleration, 0, 0) <
                0) {
                log_ptr_->error("given parameters is invalid");
                return -1;
            }
            //预防机械臂6个关节时，下发第7关节的控制命令
            if (index >= jnt_num_) {
                log_ptr_->error(" command flag= DRAGGING_FLAG::J6 is not allow because of the jnt_num_={}", jnt_num_);
                return -1;
            }
            index_type = DRAGGING_TYPE::JOINT;
        } else if (index <= static_cast< int >( DRAGGING_FLAG::BASE_YAW )) //当前命令类型为笛卡尔空间
        {
            //只检查速度、加速度,笛卡尔指令不检查
            if (CheckBeforeMove(flange_, max_speed, max_acceleration, 0, 0) < 0) {
                log_ptr_->error("given parameters is invalid");
                return -1;
            }
            index_type = DRAGGING_TYPE::CARTESIAN;
        } else if (index <= static_cast< int >( DRAGGING_FLAG::NULLSPACE ))  // 当前指令为零空间运动
        {
            if (jnt_num_ < 7) {
                log_ptr_->error("当前机械臂关节数量为:{},无法实现零空间点动", jnt_num_);
                return -1;
            }
            index_type = DRAGGING_TYPE::NULLSPACE;
        } else  // 未定义指令
        {
            log_ptr_->error("未定义指令：{}", index);
            return -1;
        }
        //**-------------------------------**//

        //** 禁止在运动中，各种点动来回切换**//
        if (index_type != last_index_type && !_dragging_finished_flag) {
            log_ptr_->error("不允许点动指令运行中切换点动类型");
            return -1;
        }
        // 三种情况能通过检查：没改没完成、没改完成了、改了完成了
        last_index_type = index_type;
        //**-------------------------------**//

        if(dir == DRAGGING_DIRRECTION::NONE) { //说明想要停止了
            tick_count += 250; //超过100就会停止
            dir = last_dir;
        }
        else {
            last_dir = dir;
        }

        //** is_running_motion的作用：不允许其他运动异步运行时,执行dragging;不允许执行dragging时，执行其他离线类运动**//
        //** _dragging_finished_flag的作用：保证dragging 多次调用时，只初始化一次**//
        if (_dragging_finished_flag && is_running_motion) {

            log_ptr_->error("其他运动仍在运行，不允许执行点动功能");
            return -1;

        } else if (_dragging_finished_flag && !is_running_motion) {
            if (motion_thread_) {
                motion_thread_->join();
                motion_thread_ = nullptr;
            }
            // is_running_motion = true;
            setRunState(RunState::Running); // TODO: 感觉应该是Stopped，原来为Running
            traj_.clear();  //!
        }
        //**-------------------------------**//

        //** 心跳保持 **//
        if (is_running_motion) {
            tick_count++;
        }
        //**-------------------------------**//

        //** 线程初始化 **//
        // 新动作需要第一次初始化,然后等待_dragging_finished_flag
        //!_dragging_finished_flag由command()置false
        // 关节空间点动指令
        if (_dragging_finished_flag && index_type == DRAGGING_TYPE::JOINT) {
            if (_thread_planning) {
                _thread_planning->join();
                _thread_planning = nullptr;
            }
            _SmartServo_Joint.init(pos_, vel_, acc_, max_speed, max_acceleration, 4 * max_acceleration);
            _thread_planning.reset(
                    new std::thread{&JC_helper::SmartServo_Joint::RunSmartServo, &_SmartServo_Joint, this});
        }
            //笛卡尔空间点动指令
        else if (_dragging_finished_flag && index_type == DRAGGING_TYPE::CARTESIAN) {
            if (_thread_planning) {
                _thread_planning->join();
                _thread_planning = nullptr;
            }

            if (index % 10 <= 2)
                // 笛卡尔空间位置点动
                _SmartServo_Cartesian.init(this, max_speed * 0.4);
            else
                // 笛卡尔空间姿态点动
                _SmartServo_Cartesian.init(this, max_speed * 1.5);

            _thread_planning.reset(
                    new std::thread{&JC_helper::SmartServo_Cartesian::RunMotion, &_SmartServo_Cartesian, this});
        }
            // 零空间点动指令
        else if (_dragging_finished_flag && index_type == DRAGGING_TYPE::NULLSPACE) {
            if (_thread_planning) {
                _thread_planning->join();
                _thread_planning = nullptr;
            }

            _SmartServo_Nullsapace.init(this, max_speed);
            _thread_planning.reset(
                    new std::thread{&JC_helper::SmartServo_Nullspace::RunMotion, &_SmartServo_Nullsapace, this});
        }
        //**-------------------------------**//


        switch (flag) {
            case DRAGGING_FLAG::J0:
            case DRAGGING_FLAG::J1:
            case DRAGGING_FLAG::J2:
            case DRAGGING_FLAG::J3:
            case DRAGGING_FLAG::J4:
            case DRAGGING_FLAG::J5:
            case DRAGGING_FLAG::J6:

                for (int i = 0; i < jnt_num_; i++)
                    target_joint(i) = pos_[i];
                target_joint(index) = std::min(target_joint(index) + static_cast< double >( dir ) * max_speed * 1.5,
                                               joints_[index]->getMaxPosLimit());  //! 取最大速度1.5倍作为目标距离,不要太小
                target_joint(index) = std::max(target_joint(index), joints_[index]->getMinPosLimit());
                _SmartServo_Joint.command(target_joint);

                break;

            case DRAGGING_FLAG::FLANGE_X:
            case DRAGGING_FLAG::FLANGE_Y:
            case DRAGGING_FLAG::FLANGE_Z:
            case DRAGGING_FLAG::FLANGE_ROLL:
            case DRAGGING_FLAG::FLANGE_PITCH:
            case DRAGGING_FLAG::FLANGE_YAW:

                index = index - static_cast< int >( DRAGGING_FLAG::FLANGE_X ) + 1;

                index = index * static_cast< double >( dir );
                _SmartServo_Cartesian.command(index, "flange");
                break;

            case DRAGGING_FLAG::TOOL_X:
            case DRAGGING_FLAG::TOOL_Y:
            case DRAGGING_FLAG::TOOL_Z:
            case DRAGGING_FLAG::TOOL_ROLL:
            case DRAGGING_FLAG::TOOL_PITCH:
            case DRAGGING_FLAG::TOOL_YAW:

                index = index - static_cast< int >( DRAGGING_FLAG::TOOL_X ) + 1;
                index = index * static_cast< double >( dir );
                _SmartServo_Cartesian.command(index, "tool");
                break;


            case DRAGGING_FLAG::OBJECT_X:
            case DRAGGING_FLAG::OBJECT_Y:
            case DRAGGING_FLAG::OBJECT_Z:
            case DRAGGING_FLAG::OBJECT_ROLL:
            case DRAGGING_FLAG::OBJECT_PITCH:
            case DRAGGING_FLAG::OBJECT_YAW:
                index = index - static_cast< int >( DRAGGING_FLAG::OBJECT_X ) + 1;
                index = index * static_cast< double >( dir );
                _SmartServo_Cartesian.command(index, "object");
                break;
            case DRAGGING_FLAG::BASE_X:
            case DRAGGING_FLAG::BASE_Y:
            case DRAGGING_FLAG::BASE_Z:
            case DRAGGING_FLAG::BASE_ROLL:
            case DRAGGING_FLAG::BASE_PITCH:
            case DRAGGING_FLAG::BASE_YAW:

                index = index - static_cast< int >( DRAGGING_FLAG::BASE_X ) + 1;
                index = index * static_cast< double >( dir );
                _SmartServo_Cartesian.command(index, "base");
                break;

            case DRAGGING_FLAG::NULLSPACE:
                _SmartServo_Nullsapace.command(static_cast< int >( dir ));
                break;

            default:
                log_ptr_->error("Undefined command flag: {}", index);
                //! 在此处位置时会置位is_running_motion
                //! 如果没有jogging 运动线程 且 is_running_motion 被置位，那is_running_motion就会被永久卡住
                if (_dragging_finished_flag && is_running_motion)
                    // is_running_motion = false;
                    setRunState(RunState::Stopped);
                return -1;
        }
        return 0;
    }

    //TODO: 这些检查要在状态机中完成及确认，执行逻辑中，不应该继续判断，直接执行
    int Robot::CheckBeforeMove(const JntArray &q, double speed, double acceleration,
                               double time, double radius) {
        //** 数据有效性检查  **//
        //TODO 这里的速度、加速度目前只针对关节空间进行检查
        for (int i = 0; i < jnt_num_; i++) {
            //位置检查
            if (q(i) > joints_[i]->getMaxPosLimit() ||
                q(i) < joints_[i]->getMinPosLimit()) {
                log_ptr_->error("  Pos command is out of range");
                return -1;
            }
            //速度检查
            if (speed > joints_[i]->getMaxVel() ||
                speed < (-1) * joints_[i]->getMaxVel()) {
                log_ptr_->error(" Vel command is out of range");
                return -1;
            }
            //加速度检查
            if (acceleration > joints_[i]->getMaxAcc() ||
                acceleration < (-1) * joints_[i]->getMaxAcc()) {
                log_ptr_->error(" Acc command is out of range");
                return -1;
            }
            //使能检查
            if (joints_[i]->getDriveState() != DriveState::OperationEnabled) {
                log_ptr_->error(" joints[{}] is in OperationDisabled", i);
                return -1;
            }
        }
        if (time < 0) {
            log_ptr_->error("  time is less than 0 invalidly");
            return -1;
        }

        if (time) {
            log_ptr_->error(" time not supported yet");
            return -1;
        }

        return 0;
        //**-------------------------------**//
    }

    int Robot::CheckBeforeMove(const Frame &pos, double speed, double acceleration,
                               double time, double radius) {
        //** 数据有效性检查  **//
        //TODO 使用解析公式去验证目标pose是否可达

        for (int i = 0; i < jnt_num_; i++) {  //TODO 这里的速度、加速度目前只针对关节空间进行检查
            //速度检查
            if (speed > joints_[i]->getMaxVel() ||
                speed < (-1) * joints_[i]->getMaxVel()) {
                log_ptr_->error(" Vel command is out of range");
                return -1;
            }
            //加速度检查
            if (acceleration > joints_[i]->getMaxAcc() ||
                acceleration < (-1) * joints_[i]->getMaxAcc()) {
                log_ptr_->error("Acc command is out of range");
                return -1;
            }
            //使能检查
            if (joints_[i]->getDriveState() != DriveState::OperationEnabled) {
                log_ptr_->error("joints[{}] is in OperationDisabled", i);
                return -1;
            }
        }
        //**-------------------------------**//
        return 0;
    }

    void Robot::RunMoveJ(JntArray q, double speed, double acceleration, double time, double radius) {
        //** 变量初始化 **//
        double dt = 0.0;
        double max_time = 0.0;
        std::vector<std::shared_ptr<DoubleS> > interp(jnt_num_);
        std::vector<double> max_step;
        std::vector<double> target_pos;//为了速度检查
        std::vector<double> init_pos;//为了速度检查
        //**-------------------------------**//

        //** 程序初始化 **//
        for (auto &i: interp)
            i.reset(new DoubleS{});

        for (int i{0}; i < jnt_num_; i++) {
            max_step.push_back(max_vel_[i] * DELTA_T);
            target_pos.push_back(pos_[i]);
            init_pos.push_back(pos_[i]);
        }
        //**-------------------------------**//

        for (int i = 0; i < jnt_num_; ++i) {
            if (fabs(q(i) - pos_[i]) < EPS) {
                need_plan_[i] = false;
                continue;
            }
            need_plan_[i] = true;

            interp[i]->planDoubleSProfile(0,          // t
                                          pos_[i],  // p0
                                          q(i),     // pf
                                          0,          // v0
                                          0,          // vf
                                          speed, acceleration, max_jerk_[i]);

            if (!interp[i]->isValidMovement() || interp[i]->getDuration() <= 0) {
                log_ptr_->error("Joint[{}] MoveJ trajectory is infeasible", i);
                // is_running_motion = false;
                setRunState(RunState::Stopped);
                return;
            }
            max_time = max(max_time, interp[i]->getDuration());
        }

        for (int i = 0; i < jnt_num_; i++) {
            if (need_plan_[i])
                interp[i]->JC_scaleToDuration(max_time);
        }

        //** 速度检查 **//
        dt = 0;
        while (dt <= max_time) {
            for (int i = 0; i < jnt_num_; i++) {
                if (!need_plan_[i]) continue; //不需要规划的关节自然不需要速度检查

                target_pos[i] = interp[i]->pos(dt);

                if (abs(target_pos[i] - init_pos[i]) > max_step[i]) {
                    log_ptr_->error("joint[{}] speep is too  fast", i);
                    log_ptr_->error("target speed = {} and  max_speed={}", abs(target_pos[i] - init_pos[i]), max_step[i]);
                    log_ptr_->error("q_target( {} )  = {}", i, target_pos[i] * 180 / M_PI);
                    log_ptr_->error("q_init( {} ) ={}", i, init_pos[i] * 180 / M_PI);

                    // is_running_motion = false;
                    setRunState(RunState::Stopped);
                    return;
                } else
                    init_pos[i] = target_pos[i];
            }

            dt += DELTA_T;
        }
        //**-------------------------------**//

        //** 伺服控制 **//
        dt = 0;
        while (dt <= max_time) {
            if(!IsEnabled())
                goto Exit;

            for (int i = 0; i < jnt_num_; ++i) {
                if (!need_plan_[i])
                    continue;

                pos_[i] = interp[i]->pos(dt);
                vel_[i] = interp[i]->vel(dt);
                joints_[i]->setPosition(pos_[i]);//!都设置，自动根据模式选取位置或者速度伺服
                joints_[i]->setVelocity(vel_[i]);//!
            }
            dt += DELTA_T;


            hw_interface_->waitForSignal(0);
        }
        //**-------------------------------**//

        Exit:
        // is_running_motion = false;
        setRunState(RunState::Stopped);
    }

    void Robot::RunMoveL(const std::vector<KDL::JntArray> &traj) {

        log_ptr_->info("No. of waypoints: {}", traj.size());
        
        for (const auto &waypoints: traj) {
            if(!IsEnabled())
                goto Exit;

            for (int i = 0; i < jnt_num_; ++i) {
                if (joints_[i]->getMode() == ModeOfOperation::CyclicSynchronousPositionMode) {
                    pos_[i] = waypoints(i);
                    joints_[i]->setPosition(waypoints(i));
                } else if (joints_[i]->getMode() == ModeOfOperation::CyclicSynchronousVelocityMode) {
                    vel_[i] = waypoints(i);
                    pos_[i] = pos_[i] + vel_[i] * DELTA_T;
                    joints_[i]->setVelocity(vel_[i]);
                    joints_[i]->setPosition(pos_[i]);
                } else {
                    log_ptr_->error("关节[{}] 不支持此模式 :{}", i, static_cast<int> (joints_[i]->getMode()));
                    // is_running_motion = false;
                    setRunState(RunState::Stopped);
                    return;
                }
            }

            hw_interface_->waitForSignal(0);
        }

        Exit:
        // is_running_motion = false;
        setRunState(RunState::Stopped);
    }

    void Robot::RunMultiMoveL(const std::vector<KDL::JntArray> &traj) {

        log_ptr_->info("No. of waypoints: {}", traj.size());

        for (const auto &waypoints: traj) {
            if(!IsEnabled())
                goto Exit;

            for (int i = 0; i < jnt_num_; ++i) {
                pos_[i] = waypoints(i);
                joints_[i]->setPosition(waypoints(i));
            }
            hw_interface_->waitForSignal(0);
        }

        Exit:
        // is_running_motion = false;
        setRunState(RunState::Stopped);
    }

    int Robot::admittance_teaching(bool asynchronous) {
        if (is_running_motion)  // 最大一条任务异步执行
        {
            log_ptr_->error(" Motion is still running and waiting for it to finish");
            return -1;
        } else {
            // is_running_motion = true;
            setRunState(RunState::Running);
        }

        JC_helper::admittance admittance_control{this, &my_ft_sensor};

        if (admittance_control.init(flange_) < 0) {
            // is_running_motion = false;
            setRunState(RunState::Stopped);
            return -1;
        }

        admittance_control.smd.set_k(0);  // 临时修改,为了拖动

        std::shared_ptr<std::thread> _thread_ft_sensor{nullptr};
        _thread_ft_sensor.reset(new std::thread{&JC_helper::admittance::sensor_update, &admittance_control, this});

        flag_admittance_turnoff = false;

        _thread_admittance_teaching.reset(
                new std::thread{&JC_helper::admittance::Runteaching, &admittance_control, this, flange_,
                                &flag_admittance_turnoff});

        log_ptr_->info("开始示教");

        if (!asynchronous) {
            //** 等待关闭指令 **//
            while (!flag_admittance_turnoff)
                std::this_thread::sleep_for(std::chrono::duration<double>(0.002));
            //**-------------------------------**//

            _thread_admittance_teaching->join();
            _thread_ft_sensor->join();

            log_ptr_->info("结束示教");

            // is_running_motion = false;
            setRunState(RunState::Stopped);
        } else {
            _thread_admittance_teaching->detach();
        }

        return 0;
    }

    int Robot::stop_admittance_teaching() {
        flag_admittance_turnoff = true;
        return 0;
    }


    int Robot::admittance_link(KDL::Frame frame_target, double speed, double acceleration) {
        if (is_running_motion)  // 最大一条任务异步执行
        {
            log_ptr_->error(" Motion is still running and waiting for it to finish");
            return -1;
        } else {
            // is_running_motion = true;
            setRunState(RunState::Running);
        }

        JC_helper::admittance admittance_control{this, &my_ft_sensor};

        // admittance类里自带传感器类，需要初始化才能用
        if (admittance_control.init(flange_) < 0) {
            // is_running_motion = false;
            setRunState(RunState::Stopped);
            return -1;
        }

        std::shared_ptr<std::thread> _thread_ft_sensor{nullptr};
        _thread_ft_sensor.reset(new std::thread{&JC_helper::admittance::sensor_update, &admittance_control, this});

        std::shared_ptr<std::thread> _thread_admittance_link{nullptr};
        _thread_admittance_link.reset(
                new std::thread{&JC_helper::admittance::RunLink, &admittance_control, this, frame_target, speed,
                                acceleration});

        log_ptr_->info("开启导纳运动");

        _thread_admittance_link->join();
        _thread_ft_sensor->join();

        log_ptr_->info("结束导纳运动");

        // is_running_motion = false;
        setRunState(RunState::Stopped);
        return 0;
    }


    int Robot::servoJ(const KDL::JntArray &target_pos) {
        //** 速度检查 **//
        Eigen::MatrixXd joint_offset = (target_pos.data - JC_helper::vector_2_JntArray(pos_).data).cwiseAbs();
        for (int i = 0; i < jointNum; ++i)
            if (joint_offset(i) > joints_[i]->getMaxVel() * DELTA_T) {
                log_ptr_->error("target vel [{}]= {} deg/s is out of range", i, joint_offset(i) * KDL::rad2deg * 1000);
                hw_interface_->waitForSignal(0);
                return -1;
            }
        //**-------------------------------**//
        //** 位置伺服 **//
        for (int i = 0; i < jointNum; ++i) {
            pos_[i] = target_pos(i);
            joints_[i]->setPosition(target_pos(i));
        }
        hw_interface_->waitForSignal(0);
        //**-------------------------------**//
        return 0;
    }

    int Robot::servoL(const KDL::Frame &target_frame) {

        JntArray q_init(jnt_num_);
        JntArray q_out(jnt_num_);
        for (int i = 0; i < jnt_num_; i++) {
            q_init.data[i] = pos_[i];
            q_out.data[i] = pos_[i];
        }

        if (kinematics_.CartToJnt(q_init, target_frame, q_out) < 0) {
            log_ptr_->error("MoveL逆解失败");
            hw_interface_->waitForSignal(0);
            return -1;
        }
        else
            return servoJ(q_out);

    }

    int Robot::joint_admittance_teaching(bool asynchronous) {
        if (is_running_motion)  // 最大一条任务异步执行
        {
            log_ptr_->error(" Motion is still running and waiting for it to finish");
            return -1;
        } else {
            // is_running_motion = true;
            setRunState(RunState::Running);
        }

        JC_helper::admittance_joint *admittance_control = new JC_helper::admittance_joint{this};

        flag_admittance_joint_turnoff = false;


        _thread_admittance_teaching.reset(
                new std::thread{&JC_helper::admittance_joint::Runteaching, admittance_control, this,
                                &flag_admittance_joint_turnoff});

        log_ptr_->info("开始示教");

        if (!asynchronous) {
            //** 等待关闭指令 **//
            while (!flag_admittance_joint_turnoff)
                std::this_thread::sleep_for(std::chrono::duration<double>(0.002));
            //**-------------------------------**//

            _thread_admittance_teaching->join();

            log_ptr_->info("结束示教");

            delete admittance_control;

            setRunState(RunState::Stopped);
        } else {
            _thread_admittance_teaching->detach();
        }

        return 0;
    }

    int Robot::stop_joint_admittance_teaching() {
        flag_admittance_joint_turnoff = true;

        return 0;
    }


}  // namespace rocos
