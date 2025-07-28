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

#define  MAX_JOINT_NUM 50
#define  EPS 1e-7

namespace rocos {
    Robot::Robot(HardwareInterface *hw,
                 const string &urdf_file_path,
                 const string &base_link,
                 const string &tip
    ) : hw_interface_(hw), urdf_file_path_(urdf_file_path), pos_(MAX_JOINT_NUM),
        vel_(MAX_JOINT_NUM), acc_(MAX_JOINT_NUM) {

        parseUrdf(urdf_file_path, base_link, tip);

//        addAllJoints( ); // TODO: 这个应该直接加到参数解析里面，解析之后加入关节，顺序和主站顺序可能不一样

        jointNum = jnt_num_;

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

        for (int i = 0; i < jnt_num_; ++i) {
            pos_[i] = joints_[i]->getPosition();
            target_positions_[i] = pos_[i];
            target_positions_prev_[i] = pos_[i];

            vel_[i] = joints_[i]->getVelocity();
            target_velocities_[i] = vel_[i];

            target_torques_[i] = joints_[i]->getTorque();

            max_vel_[i] = joints_[i]->getMaxVel();
            max_acc_[i] = joints_[i]->getMaxAcc();
            max_jerk_[i] = joints_[i]->getMaxJerk();

            if (profile_type_ == trapezoid) {
                interp_[i] = new Trapezoid;
            } else if (profile_type_ == doubleS) {
                interp_[i] = new DoubleS;
            }
        }
//        kinematics_.initTechServo();

        static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
        plog::init<0>(plog::debug,
                      &consoleAppender);//终端显示   // Initialize the logger.

        // 实现密匙加密
        //  if ( JC_helper::authentication( ) < 0 )
        //  {
        //      exit( 0 );
        //  }
        // sun 工具系的初始化
        std::vector<double> tool_param = {0, 0, 0, 0, 0, 0};
        std::vector<double> object_param = {0, 0, 0, 0, 0, 0};
        try
        {
            yaml_node = YAML::LoadFile(yaml_path);
            tool_param = yaml_node["T_tool_"].as<std::vector<double>>();

            object_param = yaml_node["T_object_"].as<std::vector<double>>();
        }
        catch (const std::exception &e)
        {
            std::cout << "yaml文件读取失败" << std::endl;
        }
        T_tool_.p.x(tool_param[0]);
        T_tool_.p.y(tool_param[1]);
        T_tool_.p.z(tool_param[2]);
        T_tool_.M = KDL::Rotation::RPY(tool_param[3], tool_param[4], tool_param[5]);
        T_object_.p.x(object_param[0]);
        T_object_.p.y(object_param[1]);
        T_object_.p.z(object_param[2]);
        T_object_.M = KDL::Rotation::RPY(object_param[3], object_param[4], object_param[5]);
        std::cout << "tool_param: " << T_tool_.p.z() << std::endl;
        std::cout << "object_param: " << T_object_.p.y() << std::endl;



        // 解析逆运动学求解器初始化
        KDL::JntArray q_min(joints_.size());
        KDL::JntArray q_max(joints_.size());
        for (int i = 0; i < joints_.size(); ++i) {
            q_min(i) = joints_[i]->getMinPosLimit();
            q_max(i) = joints_[i]->getMaxPosLimit();
        }

        if (SRS_kinematics_.init(kinematics_.getChain(), q_min, q_max) < 0)
            exit(-1);

        startMotionThread();
    }


    Robot::~Robot() {

    }


    bool Robot::parseUrdf(const string &urdf_file_path,
                          const string &base_link,
                          const string &tip) {

        KDL::Tree tree;
        if (!kdl_parser::treeFromFile(urdf_file_path, tree)) {
            // 解析失败
            std::cerr << "[ERROR][rocos::robot] Could not extract urdf to kdl tree!" << std::endl;
            return false;
        }

        if (!kinematics_.setChain(tree, base_link, tip)) {
            std::cerr << "[ERROR][rocos::robot] Could not set kinematic chain!" << std::endl;
            return false;
        }

        if (!parseDriveParamsFromUrdf(urdf_file_path)) {
            std::cerr << "[ERROR][rocos::robot] Could not parse drive parameters!" << std::endl;
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

        return true;
    }

    //! \brief 从URDF中解析驱动器相关参数，这个函数只在parseUrdf()内部调用，在调用前，已经解析好KDL::Chain
    //! \param urdf_file_path urdf文件路径
    //! \return
    bool Robot::parseDriveParamsFromUrdf(const string &urdf_file_path) {
        jnt_num_ = kinematics_.getChain().getNrOfJoints();

        // if (kinematics_.getChain().getNrOfJoints() > jnt_num_) {
        //     // if the number of joints in urdf is LESS than that in hardware, just warning but it's fine
        //     std::cout << "[WARNING][rocos::robot] the hardware slave number is more than joint number." << std::endl;
        //     return false;
        // } else if (kinematics_.getChain().getNrOfJoints() < jnt_num_) {
        //     // if the number of joints in urdf is GREATER than that in hardware, error occured and return
        //     std::cerr << "[ERROR][rocos::robot] the hardware slave number is less than joint number." << std::endl;
        //     return false;
        // }

        joints_.clear(); // vector<Drive>清空

        tinyxml2::XMLDocument xml_doc;
        xml_doc.LoadFile(urdf_file_path.c_str()); // 解析urdf文件

        auto robot = xml_doc.FirstChildElement("robot");

        for (auto element = robot->FirstChildElement("joint"); element; element = element->NextSiblingElement(
                "joint")) {
            for (int i = 0; i < jnt_num_; ++i) {
                if (element->Attribute("name") == kinematics_.getChain().getSegment(i).getJoint().getName()) {
                    std::cout << "Joint" << std::endl
                              << "- name: " << element->Attribute("name") << "\n";

                    auto hw = element->FirstChildElement("hardware");

                    auto id = hw->IntAttribute("id", -1); // 对应的硬件ID，若没指定默认为-1
                    std::cout << "- id: " << id << std::endl;

                    auto jnt_ptr = boost::make_shared<Drive>(hw_interface_, id); //获取相应硬件指针

                    jnt_ptr->setName(element->Attribute("name")); //设置驱动器名称
                    jnt_ptr->setMode(ModeOfOperation::CyclicSynchronousPositionMode); //驱动器模式设置为CSP

                    auto limit = hw->FirstChildElement("limit");

                    jnt_ptr->setMinPosLimit(limit->DoubleAttribute("lower", -M_PI));
                    jnt_ptr->setMaxPosLimit(limit->DoubleAttribute("upper", M_PI));
                    jnt_ptr->setMaxVel(limit->DoubleAttribute("vel", 1.0));
                    jnt_ptr->setMaxAcc(limit->DoubleAttribute("acc", 10.0));
                    jnt_ptr->setMaxJerk(limit->DoubleAttribute("jerk", 100.0));

                    std::cout << "- limits: \n"
                              << "----- lower: " << limit->DoubleAttribute("lower", -M_PI) << std::endl
                              << "----- upper: " << limit->DoubleAttribute("upper", M_PI) << std::endl
                              << "----- vel: " << limit->DoubleAttribute("vel", 1.0) << std::endl
                              << "----- acc: " << limit->DoubleAttribute("acc", 10.0) << std::endl
                              << "----- jerk: " << limit->DoubleAttribute("jerk", 100.0) << std::endl;

                    auto trans = hw->FirstChildElement("transform");

                    jnt_ptr->setRatio(trans->DoubleAttribute("ratio", 1.0));
                    jnt_ptr->setPosZeroOffset(trans->IntAttribute("offset_pos_cnt", 0));
                    jnt_ptr->setCntPerUnit(trans->DoubleAttribute("cnt_per_unit", 1.0));
                    jnt_ptr->setTorquePerUnit(trans->DoubleAttribute("torque_per_unit", 1.0));

                    std::cout << "- transform: \n"
                              << "----- ratio: " << trans->DoubleAttribute("ratio", 1.0) << std::endl
                              << "----- offset_pos_cnt: " << trans->IntAttribute("offset_pos_cnt", 0) << std::endl
                              << "----- cnt_per_unit: " << trans->DoubleAttribute("cnt_per_unit", 1.0) << std::endl
                              << "----- torque_per_unit: " << trans->DoubleAttribute("torque_per_unit", 1.0)
                              << std::endl;
                    if (trans->Attribute("user_unit_name")) {
                        jnt_ptr->setUserUnitName(trans->Attribute("user_unit_name"));
                        std::cout << "----- user_unit_name: " << trans->Attribute("user_unit_name") << std::endl;
                    } else {
                        jnt_ptr->setUserUnitName(trans->Attribute("rad"));
                        std::cout << "----- user_unit_name: " << "rad" << std::endl;
                    }

                    joints_.push_back(jnt_ptr); // 将对应ID的hardware放入joints数组
                }
            }
        }

        return true;
    }


    void Robot::addAllJoints() {
        jnt_num_ = hw_interface_->getSlaveNumber();
        joints_.clear();
        for (int i = 0; i < jnt_num_; i++) {
            joints_.push_back(boost::make_shared<Drive>(hw_interface_, i));
            joints_[i]->setMode(ModeOfOperation::CyclicSynchronousPositionMode);
        }
    }

    // TODO: 切换HW指针
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

        // 机器人只能在停止状态切换工作模式
        if (getRunState() == RunState::Running) {
            std::cout << "[ERROR] Robot is not stopped!" << std::endl;
            return false;
        }

        work_mode_ = mode;

        switch (mode) {
            case WorkMode::Position:
                PLOG(plog::info) << "Robot work mode is set to Position";
                break;
            case WorkMode::EeAdmitTeach:
                PLOG(plog::info) << "Robot work mode is set to EeAdmitTeach";
                this->admittance_teaching(true);
                break;
            case WorkMode::JntAdmitTeach:
                PLOG(plog::info) << "Robot work mode is set to JntAdmitTeach";
                this->joint_admittance_teaching(true);
                break;
            case WorkMode::JntImp:
                PLOG(plog::info) << "Robot work mode is set to JntImp";
                break;
            case WorkMode::CartImp:
                PLOG(plog::info) << "Robot work mode is set to CartImp";
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


    // TODO: 测试用MoveJ，阻塞运行，需要改为private
    void Robot::moveJ(const std::vector<double> &pos,
                      const std::vector<double> &max_vel,
                      const std::vector<double> &max_acc,
                      const std::vector<double> &max_jerk,
                      Robot::Synchronization sync, ProfileType type) {
        if (pos.size() != jnt_num_ || max_vel.size() != jnt_num_ ||
            max_acc.size() != jnt_num_ || max_jerk.size() != jnt_num_) {
            std::cout << "[ERROR] MoveJ => Error Input Vector Size!" << std::endl;
            return;
        }

        std::vector<R_INTERP_BASE *> interp(jnt_num_);

        double max_time = 0.0;

        // Start trajectory generation....
        for (int i = 0; i < jnt_num_; i++) {
            auto p0 = joints_[i]->getPosition();
            switch (type) {
                case trapezoid:
                    interp[i] = new Trapezoid;
                    reinterpret_cast< Trapezoid * >( interp[i] )->planTrapezoidProfile(0, p0, pos[i], 0, 0, max_vel[i],
                                                                                       max_acc[i]);
                    break;
                case doubleS:
                    interp[i] = new DoubleS;
                    reinterpret_cast< DoubleS * >( interp[i] )->planDoubleSProfile(0, p0, pos[i], 0, 0, max_vel[i],
                                                                                   max_acc[i], max_jerk[i]);
                    break;
                default:
                    std::cout << "Not Supported Profile Type" << std::endl;
                    return;
            }

            max_time = max(max_time, interp[i]->getDuration());
        }

        // Sync scaling....
        if (sync == SYNC_TIME) {
            for_each(interp.begin(), interp.end(),
                     [=](R_INTERP_BASE *p) { p->scaleToDuration(max_time); });
        } else if (sync == SYNC_PHASE) {
            std::cout
                    << "[WARNING] Phase sync has not implemented...instead of time sync."
                    << std::endl;
            for_each(interp.begin(), interp.end(),
                     [=](R_INTERP_BASE *p) { p->scaleToDuration(max_time); });
        }

        // Start moving....
        double dt = 0.0;
        while (dt <= max_time) {
            hw_interface_->waitForSignal(9);

            for (int i = 0; i < jnt_num_; i++) {
                if (!interp[i]->isValidMovement()) {
                    continue;
                }
                switch (joints_[i]->getMode()) {
                    case ModeOfOperation::CyclicSynchronousPositionMode:
                        joints_[i]->setPosition(interp[i]->pos(dt));
                        break;
                    case ModeOfOperation::CyclicSynchronousVelocityMode:
                        joints_[i]->setVelocity(interp[i]->vel(dt));
                        break;
                    default:
                        std::cout << "Only Supported CSP and CSV" << std::endl;
                }
            }

            dt += DELTA_T;
        }

        // delete pointer
        for (auto &p: interp) {
            delete p;
        }
    }

    void Robot::setEnabled() {
        setRunState(RunState::Stopped);

        for_each(joints_.begin(), joints_.end(),
                 [=](boost::shared_ptr<Drive> &d) { d->setEnabled(false); }); // 将抱闸设置为同时开启，不阻塞


        // set a temporary time point to prevent getting caught in an infinite loop
        auto driveStateChangeStartTimePoint = boost::chrono::system_clock::now();

        outerloop:
        for (;;) {
            for (const auto &joint: joints_) {
                // First check timeout
                auto duration_us = boost::chrono::duration_cast<boost::chrono::microseconds>(
                        boost::chrono::system_clock::now() - driveStateChangeStartTimePoint);
                if (duration_us.count() > 150000) { //wait for 100ms  TODO: configuration_.driveStateChangeMaxTimeout
                    std::cout << "It takes too long (" << duration_us.count() / 1000.0 << " ms) to switch state!"
                              << std::endl;
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
        setRunState(RunState::Disabled);

        for_each(joints_.begin(), joints_.end(),
                 [=](boost::shared_ptr<Drive> &d) { d->setDisabled(false); }); // 将抱闸设置为同时开启，不阻塞



        // set a temporary time point to prevent getting caught in an infinite loop
        auto driveStateChangeStartTimePoint = boost::chrono::system_clock::now();

        outerloop:
        for (;;) {
            for (const auto &joint: joints_) {
                // First check timeout
                auto duration_us = boost::chrono::duration_cast<boost::chrono::microseconds>(
                        boost::chrono::system_clock::now() - driveStateChangeStartTimePoint);
                if (duration_us.count() > 150000) { //wait for 100ms  TODO: configuration_.driveStateChangeMaxTimeout
                    std::cout << "It takes too long (" << duration_us.count() / 1000.0 << " ms) to switch state!"
                              << std::endl;
                    break;
                }

                if (joint->getDriveState() != DriveState::SwitchOnDisabled) {
                    goto outerloop;
                }

            }
            break;
        }
    }

    void Robot::startMotionThread() {
//        setRunState(RunState::Running);

        otg_motion_thread_ =
                boost::make_shared<boost::thread>(&Robot::motionThreadHandler, this);
        //        boost::thread(&Robot::motionThreadHandler, this);
    }

    void Robot::stopMotionThread() {
        // is_running_ = false;
        setRunState(RunState::Stopped);

        otg_motion_thread_->interrupt();
        otg_motion_thread_->join();  //等待运动线程结束
    }

    void Robot::motionThreadHandler() {
        std::cout << "Motion thread is running on thread "
                  << boost::this_thread::get_id() << std::endl;
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

        // std::vector< double > dt( jnt_num_, 0.0 );  // delta T
        // double max_time = 0.0;

        while (true) {  // while start

            hw_interface_->waitForSignal(9);

            //!< Update Flange State
            std::lock_guard<std::mutex> lock(mtx);  // 自动获取互斥锁
            updateCartesianInfo();

            //     //! 屏蔽开始
            //    //!< Trajectory generating......
            //    max_time = 0.0;
            //    //** 轨迹生成 **//
            //    for (int i = 0; i < jnt_num_; ++i) {
            //      if (joints_[i]->getDriveState() != DriveState::OperationEnabled) {
            //        target_positions_[i] = joints_[i]->getPosition();
            //        //                    target_positions_prev_[i] = target_positions_[i];
            //
            //        continue;  // Disabled, ignore
            //      }
            //
            //      if (fabs(target_positions_[i] != target_positions_prev_[i]) ||
            //          need_plan_[i]) {  // need update
            //        target_positions_prev_[i] =
            //            target_positions_[i];  // assign to current target position
            //
            //        interp_[i]->planProfile(0,                          // t
            //                                pos_[i],                    // p0
            //                                target_positions_prev_[i],  // pf
            //                                vel_[i],                    // v0
            //                                target_velocities_[i],      // vf
            //                                max_vel_[i], max_acc_[i], max_jerk_[i]);
            //
            //        dt[i] = 0.0;  // once regenerate, dt = 0.0
            //        // consider at least execute time
            //        if (interp_[i]->getDuration() < least_motion_time_) {
            //          interp_[i]->scaleToDuration(least_motion_time_);
            //        }
            //        // record max_time
            //        max_time =
            //            max(max_time, interp_[i]->getDuration());  // get max duration time
            //
            //        need_plan_[i] = false;
            //      }
            //    }
            //    //**-------------------------------**//
            //
            //    //!< Sync scaling....
            //    //各关节时间同步
            //    if (sync_ == SYNC_TIME) {
            //      for_each(interp_.begin(), interp_.end(),
            //               [=](R_INTERP_BASE* p) { p->scaleToDuration(max_time); });
            //    }
            //    //各关节无需同步
            //    else if (sync_ == SYNC_NONE) {
            //    }
            //    //各关节相位同步
            //    else if (sync_ == SYNC_PHASE) {
            //      std::cout
            //          << "[WARNING] Phase sync has not implemented...instead of time sync."
            //          << std::endl;
            //      for_each(interp_.begin(), interp_.end(),
            //               [=](R_INTERP_BASE* p) { p->scaleToDuration(max_time); });
            //    }
            //
            //    //!< Start moving....
            //    for (int i = 0; i < jnt_num_; ++i) {
            //      if (joints_[i]->getDriveState() != DriveState::OperationEnabled) {
            //        continue;  // Disabled, ignore
            //      }
            //
            //      if (interp_[i]->isValidMovement()) {
            //        dt[i] += DELTA_T;
            //        pos_[i] = interp_[i]->pos(dt[i]);  //当前位置更新
            //        vel_[i] = interp_[i]->vel((dt[i]));
            //        acc_[i] = interp_[i]->acc(dt[i]);
            //
            //        switch (joints_[i]->getMode()) {
            //          case ModeOfOperation::CyclicSynchronousPositionMode:
            //            joints_[i]->setPosition(pos_[i]);
            //            break;
            //          case ModeOfOperation::CyclicSynchronousVelocityMode:
            //            joints_[i]->setVelocity(vel_[i]);
            //            break;
            //          default:
            //            std::cout << "Only Supported CSP and CSV" << std::endl;
            //        }
            //      } else {
            //        vel_[i] = 0.0;
            //      }
            ////      std::cout << " pos_[" << i << "]  = " << pos_[i] << std::endl;
            //    }
            //
            //    //! 屏蔽结束

            ////    std::cout << "----------------------" << std::endl;
        }

        // process before exit:
    }

    void Robot::moveJ(const vector<double> &target_pos,
                      const vector<double> &target_vel,
                      Robot::Synchronization sync) {
        if ((target_pos.size() != jnt_num_) || (target_vel.size() != jnt_num_)) {
            std::cout << "[ERROR] MoveJ => Error Input Vector Size!" << std::endl;
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
        //        target_positions_[id] = pos_[id];
        target_velocities_[id] = 0.0;
        least_motion_time_ = 0.0;

        auto sync = sync_;
        sync_ = SYNC_NONE;  //停止时候就不需要同步了

        std::cout << "max_acc: " << max_acc_[id] << "; pos: " << pos_[id]
                  << "; vel: " << vel_[id] << std::endl;
        std::cout << "dt: " << dt << "; target_positions: " << target_positions_[id]
                  << std::endl;

        need_plan_[id] = true;

        //        usleep(dt * 1000000);
        //        sync_ = sync;
    }

    void Robot::stopMultiAxis() {
        //        auto sync = sync_;
        sync_ = SYNC_NONE;  //停止时候就不需要同步了

        double wait_time = 0.0;

        for (int id = 0; id < jnt_num_; ++id) {
            double dt = fabs(vel_[id]) / max_acc_[id];  // 所需要的减速时间
            target_positions_[id] =
                    pos_[id] +
                    2 * (dt * vel_[id] / 2.0);  // TODO：这个减速段计算有问题
            //        target_positions_[id] = pos_[id];
            target_velocities_[id] = 0.0;
            least_motion_time_ = 0.0;

            std::cout << "max_acc: " << max_acc_[id] << "; pos: " << pos_[id]
                      << "; vel: " << vel_[id] << std::endl;
            std::cout << "dt: " << dt << "; target_positions: " << target_positions_[id]
                      << std::endl;

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
            std::cout << "[ERROR] moveMultiAxis: wrong size!" << std::endl;
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

    int Robot::MoveJ(JntArray q, double speed, double acceleration, double time,
                     double radius, bool asynchronous) {

        if (CheckBeforeMove(q, speed, acceleration, time, radius) < 0) {
            PLOG_ERROR << "given parameters is invalid";
            return -1;
        }

        for (int i{0}; i < jointNum; i++) {
            if (!(joints_[i]->getMode() == ModeOfOperation::CyclicSynchronousPositionMode ||
                  joints_[i]->getMode() == ModeOfOperation::CyclicSynchronousVelocityMode)) {
                PLOG_ERROR << "MoveJ不支持关节[" << i << "]的当前模式 :" << static_cast< int >( joints_[i]->getMode());
                return -1;
            }
        }

        if (is_running_motion)  //最大异步执行一条任务
        {
            PLOG_ERROR << " Motion is still running and waiting for it to finish";
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
            motion_thread_.reset(new boost::thread{&Robot::RunMoveJ, this, q,
                                                   speed, acceleration, time,
                                                   radius});
        } else  //同步执行
        {
            motion_thread_.reset(new boost::thread{&Robot::RunMoveJ, this, q,
                                                   speed, acceleration, time,
                                                   radius});
            motion_thread_->join();
            motion_thread_ = nullptr;
        }

        return 0;
    }

    int Robot::MoveJ_IK(Frame pose, double speed, double acceleration, double time,
                        double radius, bool asynchronous) {
        std::cout << "MoveJ_IK pose: " << pose << std::endl;
        JntArray q_init(jnt_num_);
        JntArray q_target(jnt_num_);
        for (int i = 0; i < jnt_num_; i++) {
            q_init.data[i] = pos_[i];
            q_target.data[i] = pos_[i];
        }
        if (kinematics_.CartToJnt(q_init, pose, q_target) < 0) {
            PLOG_ERROR << " CartToJnt failed";
            return -1;
        }
        return MoveJ(q_target, speed, acceleration, time, radius, asynchronous);
    }

    int Robot::MoveL(Frame pose, double speed, double acceleration, double time,
                     double radius, bool asynchronous, int max_running_count) {
        // if (pose == flange_) {
        //     PLOG_ERROR << "设置的目标等于当前位姿";
        //     return -1;
        // }

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
            PLOG_ERROR << "某关节为既不是位置模式也不是速度模式！";
            return -1;
        }
    }

    int Robot::MoveL_pos(Frame pose, double speed, double acceleration, double time,
                         double radius, bool asynchronous, int max_running_count) {
        for (int i{0}; i < jointNum; i++) {
            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousPositionMode) {
                PLOG_ERROR << " 需要关节[" << i << "]进入位置伺服模式";
                return -1;
            }
        }

        if (max_running_count < 1) {
            PLOG_ERROR << "max_running_count parameters must be greater than 0";
            return -1;
        }

        if (CheckBeforeMove(pose, speed, acceleration, time, radius) < 0) {
            PLOG_ERROR << "given parameters is invalid";
            return -1;
        }

        if (is_running_motion)  //最大一条任务异步执行
        {
            PLOG_ERROR << " Motion is still running and waiting for it to finish";
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
            PLOG_ERROR << "link trajectory planning fail ";
            //  is_running_motion =false;
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

                PLOG_INFO << "---------------------------------------";
                //sun
                // 尝试改为解析解接口
                for (const auto &target: traj_target) {
                    // if (SRS_kinematics_.JC_cartesian_to_joint_dir(target, q_init(2), q_init, q_target) < 0) {
                    //     PLOG_ERROR << " JC_cartesian_to_joint failed on the " << ik_count << " times";
                    //     throw -1;

                    // }
                    if (kinematics_.CartToJnt(q_init, target, q_target) < 0) {
                        PLOG_ERROR << " CartToJnt failed on the " << ik_count << " times";
                        throw -1;
                    }
                    //*防止奇异位置速度激增
                    for (int i = 0; i < jnt_num_; i++) {
                        if (abs(q_target(i) - q_init(i)) > max_step[i]) {
                            PLOG_ERROR << "joint[" << i << "] speep is too  fast";
                            PLOG_ERROR << "target speed = " << abs(q_target(i) - q_init(i))
                                       << " and  max_step=" << max_step[i];
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
                        PLOG_ERROR << "Undefined error!";
                        // is_running_motion = false;
                        setRunState(RunState::Stopped);
                        return -1;
                }
            }
            catch (...) {
                PLOG_ERROR << "Undefined error!";
                // is_running_motion =false;
                setRunState(RunState::Stopped);
                return -1;
            }

        }

        if (ik_count == max_running_count) {
            PLOG_ERROR << "CartToJnt still failed even after " << max_running_count << " attempts";
            // is_running_motion =false;
            setRunState(RunState::Stopped);
            return -1;
        }

        //**-------------------------------**//

        if (asynchronous)  //异步执行
        {
            motion_thread_.reset(new boost::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            // is_running_motion = true;
            setRunState(RunState::Running);
        } else  //同步执行
        {
            motion_thread_.reset(new boost::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            motion_thread_->join();
            motion_thread_ = nullptr;
            // is_running_motion = false;
            setRunState(RunState::Stopped);
        }

        return 0;
    }

    int Robot::MoveL_vel(Frame pose, double speed, double acceleration, double time,
                         double radius, bool asynchronous, int max_running_count) {
        for (int i{0}; i < jointNum; i++) {
            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousVelocityMode) {
                PLOG_ERROR << " 需要关节[" << i << "]进入速度伺服模式";
                return -1;
            }
        }

        if (time) {
            PLOG_ERROR << " time not supported yet";
            return -1;
        }

        if (max_running_count < 1) {
            PLOG_ERROR << "max_running_count parameters must be greater than 0";
            return -1;
        }

        if (CheckBeforeMove(pose, speed, acceleration, time, radius) < 0) {
            PLOG_ERROR << "given parameters is invalid";
            return -1;
        }

        if (is_running_motion)  //最大一条任务异步执行
        {
            PLOG_ERROR << " Motion is still running and waiting for it to finish";
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
        std::vector<KDL::Twist> vel_target;
        KDL::Frame frame_current;
        JntToCart(JC_helper::vector_2_JntArray(pos_), frame_current);
        KDL::ChainIkSolverVel_pinv _ik_vel{kinematics_.getChain()};
        //**-------------------------------**//

        if (JC_helper::link_trajectory(vel_target, frame_current, pose, speed, acceleration) < 0) {
            PLOG_ERROR << "link trajectory planning fail ";
            //  is_running_motion =false;
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

                PLOG_INFO << "---------------------------------------";

                for (const auto &target: vel_target) {
                    //!雅克比默认参考系为base,参考点为flange
                    if (_ik_vel.CartToJnt(q_init, target, joint_vel) != 0) {
                        PLOG_ERROR << "雅克比计算错误,错误号：" << _ik_vel.CartToJnt(q_init, target, joint_vel);
                        throw -1;
                    }

                    //*防止奇异位置速度激增
                    for (int i = 0; i < jnt_num_; i++) {
                        if (abs(joint_vel(i)) > max_vel_[i]) {
                            PLOG_ERROR << "joint[" << i << "] speep is too  fast";
                            PLOG_ERROR << "target speed = " << abs(joint_vel(i))
                                       << " and  max_vel_=" << max_vel_[i];
                            throw -2;
                        }

                    }
                    //**-------------------------------**//

                    //** 位置保护，雅克比计算需要位置检查 **//
                    q_init.data = q_init.data + joint_vel.data * DELTA_T;
                    for (int i = 0; i < jnt_num_; i++) {
                        if (q_init(i) > joints_[i]->getMaxPosLimit() ||
                            q_init(i) < joints_[i]->getMinPosLimit()) {
                            PLOG_ERROR << "关节[" << i << "] 超过关节限位，求解失败";
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
                        PLOG_ERROR << "Undefined error!";
                        // is_running_motion = false;
                        setRunState(RunState::Stopped);
                        return -1;
                }
            }
            catch (...) {
                PLOG_ERROR << "Undefined error!";
                // is_running_motion =false;
                setRunState(RunState::Stopped);
                return -1;
            }

        }

        if (ik_count == max_running_count) {
            PLOG_ERROR << "CartToJnt still failed even after " << max_running_count << " attempts";
            // is_running_motion =false;
            setRunState(RunState::Stopped);
            return -1;
        }

        //**-------------------------------**//

        if (asynchronous)  //异步执行
        {
            motion_thread_.reset(new boost::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            // is_running_motion = true;
            setRunState(RunState::Running);
        } else  //同步执行
        {
            motion_thread_.reset(new boost::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            motion_thread_->join();
            motion_thread_ = nullptr;
            // is_running_motion = false;
            setRunState(RunState::Stopped);
        }

        return 0;
    }


    int Robot::MoveL_FK(JntArray q, double speed, double acceleration, double time,
                        double radius, bool asynchronous) {
        KDL::Frame target;
        kinematics_.JntToCart(q, target);
        std::cout << "Target pose is: \n"
                  << target << std::endl;
        return MoveL(target, speed, acceleration, time, radius, asynchronous);
    }

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
            PLOG_ERROR << "某关节为既不是位置模式也不是速度模式！";
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
            PLOG_ERROR << "某关节为既不是位置模式也不是速度模式！";
            return -1;
        }
    }

    int Robot::MoveC_pos(Frame pose_via, Frame pose_to, double speed,
                         double acceleration, double time, double radius,
                         Robot::OrientationMode mode, bool asynchronous, int max_running_count) {
        for (int i{0}; i < jointNum; i++) {
            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousPositionMode) {
                PLOG_ERROR << " 需要关节[" << i << "]进入位置伺服模式";
                return -1;
            }
        }

        if (time) {
            PLOG_ERROR << " time not supported yet";
            return -1;
        }
        if (max_running_count < 1) {
            PLOG_ERROR << "max_running_count parameters must be greater than 0";
            return -1;
        }
        if (CheckBeforeMove(pose_via, speed, acceleration, time, radius) < 0) {
            PLOG_ERROR << "given parameters is invalid";
            return -1;
        }
        if (CheckBeforeMove(pose_to, speed, acceleration, time, radius) < 0) {
            PLOG_ERROR << "given parameters is invalid";
            return -1;
        }
        if (is_running_motion)  //最大一条任务异步执行
        {
            PLOG_ERROR << RED << " Motion is still running and waiting for it to finish" << WHITE;
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
            PLOG_ERROR << "circle trajectory planning fail ";
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

                PLOG_INFO << "---------------------------------------";

                for (const auto &target: traj_target) {
                    if (kinematics_.CartToJnt(q_init, target, q_target) < 0) {
                        throw -1;
                    }
                    //** 防止奇异位置速度激增 **//
                    for (int i = 0; i < jnt_num_; i++) {
                        if (abs(q_target(i) - q_init(i)) > max_step[i]) {
                            PLOG_ERROR << "joint[" << i << "] speep is too  fast";
                            PLOG_ERROR << "target speed = " << abs(q_target(i) - q_init(i))
                                       << " and  max_step=" << max_step[i];
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
                        PLOG_ERROR << " CartToJnt failed on the " << ik_count << " times";
                        break;
                    case -2:
                        break;
                    default:
                        PLOG_ERROR << "Undefined error!";
                        // is_running_motion = false;
                        setRunState(RunState::Stopped);
                        return -1;
                }
            }
            catch (...) {
                PLOG_ERROR << "Undefined error!";
                // is_running_motion =false;
                setRunState(RunState::Stopped);
                return -1;
            }
        }

        if (ik_count == max_running_count) {
            PLOG_ERROR << "CartToJnt still failed even after " << max_running_count << " attempts";
            // is_running_motion =false;
            setRunState(RunState::Stopped);
            return -1;
        }

        //**-------------------------------**//

        if (asynchronous)  //异步执行
        {
            motion_thread_.reset(new boost::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            // is_running_motion = true;
            setRunState(RunState::Running);
        } else  //同步执行
        {
            motion_thread_.reset(new boost::thread{&Robot::RunMoveL, this, std::ref(traj_)});
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
                PLOG_ERROR << " 需要关节[" << i << "]进入位置伺服模式";
                return -1;
            }
        }

        if (time) {
            PLOG_ERROR << " time not supported yet";
            return -1;
        }
        if (max_running_count < 1) {
            PLOG_ERROR << "max_running_count parameters must be greater than 0";
            return -1;
        }
        if (CheckBeforeMove(flange_, speed, acceleration, time, radius) < 0) {
            PLOG_ERROR << "given parameters is invalid";
            return -1;
        }

        if (is_running_motion)  //最大一条任务异步执行
        {
            PLOG_ERROR << RED << " Motion is still running and waiting for it to finish" << WHITE;
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
            PLOG_ERROR << "circle trajectory planning fail ";
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

                PLOG_INFO << "---------------------------------------";

                for (const auto &target: traj_target) {
                    if (kinematics_.CartToJnt(q_init, target, q_target) < 0) {
                        throw -1;
                    }
                    //** 防止奇异位置速度激增 **//
                    for (int i = 0; i < jnt_num_; i++) {
                        if (abs(q_target(i) - q_init(i)) > max_step[i]) {
                            PLOG_ERROR << "joint[" << i << "] speep is too  fast";
                            PLOG_ERROR << "target speed = " << abs(q_target(i) - q_init(i))
                                       << " and  max_step=" << max_step[i];
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
                        PLOG_ERROR << " CartToJnt failed on the " << ik_count << " times";
                        break;
                    case -2:
                        break;
                    default:
                        PLOG_ERROR << "Undefined error!";
                        // is_running_motion = false;
                        setRunState(RunState::Stopped);
                        return -1;
                }
            }
            catch (...) {
                PLOG_ERROR << "Undefined error!";
                // is_running_motion =false;
                setRunState(RunState::Stopped);
                return -1;
            }
        }

        if (ik_count == max_running_count) {
            PLOG_ERROR << "CartToJnt still failed even after " << max_running_count << " attempts";
            // is_running_motion =false;
            setRunState(RunState::Stopped);
            return -1;
        }

        //**-------------------------------**//

        if (asynchronous)  //异步执行
        {
            motion_thread_.reset(new boost::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            // is_running_motion = true;
            setRunState(RunState::Running);
        } else  //同步执行
        {
            motion_thread_.reset(new boost::thread{&Robot::RunMoveL, this, std::ref(traj_)});
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
                PLOG_ERROR << " 需要关节[" << i << "]进入速度伺服模式";
                return -1;
            }
        }

        if (time) {
            PLOG_ERROR << " time not supported yet";
            return -1;
        }
        if (max_running_count < 1) {
            PLOG_ERROR << "max_running_count parameters must be greater than 0";
            return -1;
        }
        if (CheckBeforeMove(pose_via, speed, acceleration, time, radius) < 0) {
            PLOG_ERROR << "given parameters is invalid";
            return -1;
        }
        if (CheckBeforeMove(pose_to, speed, acceleration, time, radius) < 0) {
            PLOG_ERROR << "given parameters is invalid";
            return -1;
        }
        if (is_running_motion)  //最大一条任务异步执行
        {
            PLOG_ERROR << " Motion is still running and waiting for it to finish" << WHITE;
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
            PLOG_ERROR << "circle trajectory planning fail ";
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

                PLOG_INFO << "---------------------------------------";

                for (const auto &target: traj_vel_target) {
                    //!雅克比默认参考系为base,参考点为flange
                    if (_ik_vel.CartToJnt(q_init, target, joint_vel) != 0) {
                        PLOG_ERROR << "雅克比计算错误,错误号：" << _ik_vel.CartToJnt(q_init, target, joint_vel);
                        throw -1;
                    }

                    //*防止奇异位置速度激增
                    for (int i = 0; i < jnt_num_; i++) {
                        if (abs(joint_vel(i)) > max_vel_[i]) {
                            PLOG_ERROR << "joint[" << i << "] speep is too  fast";
                            PLOG_ERROR << "target speed = " << abs(joint_vel(i))
                                       << " and  max_vel_=" << max_vel_[i];
                            throw -2;
                        }

                    }
                    //**-------------------------------**//

                    //** 位置保护，雅克比计算需要位置检查 **//
                    q_init.data = q_init.data + joint_vel.data * DELTA_T;
                    for (int i = 0; i < jnt_num_; i++) {
                        if (q_init(i) > joints_[i]->getMaxPosLimit() ||
                            q_init(i) < joints_[i]->getMinPosLimit()) {
                            PLOG_ERROR << "关节[" << i << "] 超过关节限位，求解失败";
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
                        PLOG_ERROR << "Undefined error!";
                        // is_running_motion = false;
                        setRunState(RunState::Stopped);
                        return -1;
                }
            }
            catch (...) {
                PLOG_ERROR << "Undefined error!";
                // is_running_motion =false;
                setRunState(RunState::Stopped);
                return -1;
            }
        }

        if (ik_count == max_running_count) {
            PLOG_ERROR << "CartToJnt still failed even after " << max_running_count << " attempts";
            // is_running_motion =false;
            setRunState(RunState::Stopped);
            return -1;
        }

        //**-------------------------------**//

        if (asynchronous)  //异步执行
        {
            motion_thread_.reset(new boost::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            // is_running_motion = true;
            setRunState(RunState::Running);
        } else  //同步执行
        {
            motion_thread_.reset(new boost::thread{&Robot::RunMoveL, this, std::ref(traj_)});
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
                PLOG_ERROR << " 需要关节[" << i << "]进入速度伺服模式";
                return -1;
            }
        }

        if (time) {
            PLOG_ERROR << " time not supported yet";
            return -1;
        }
        if (max_running_count < 1) {
            PLOG_ERROR << "max_running_count parameters must be greater than 0";
            return -1;
        }
        if (CheckBeforeMove(flange_, speed, acceleration, time, radius) < 0) {
            PLOG_ERROR << "given parameters is invalid";
            return -1;
        }

        if (is_running_motion)  //最大一条任务异步执行
        {
            PLOG_ERROR << " Motion is still running and waiting for it to finish";
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
            PLOG_ERROR << "circle trajectory planning fail ";
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

                PLOG_INFO << "---------------------------------------";

                for (const auto &target: traj_vel_target) {
                    //!雅克比默认参考系为base,参考点为flange
                    if (_ik_vel.CartToJnt(q_init, target, joint_vel) != 0) {
                        PLOG_ERROR << "雅克比计算错误,错误号：" << _ik_vel.CartToJnt(q_init, target, joint_vel);
                        throw -1;
                    }

                    //*防止奇异位置速度激增
                    for (int i = 0; i < jnt_num_; i++) {
                        if (abs(joint_vel(i)) > max_vel_[i]) {
                            PLOG_ERROR << "joint[" << i << "] speep is too  fast";
                            PLOG_ERROR << "target speed = " << abs(joint_vel(i))
                                       << " and  max_vel_=" << max_vel_[i];
                            throw -2;
                        }

                    }
                    //**-------------------------------**//

                    //** 位置保护，雅克比计算需要位置检查 **//
                    q_init.data = q_init.data + joint_vel.data * DELTA_T;
                    for (int i = 0; i < jnt_num_; i++) {
                        if (q_init(i) > joints_[i]->getMaxPosLimit() ||
                            q_init(i) < joints_[i]->getMinPosLimit()) {
                            PLOG_ERROR << "关节[" << i << "] 超过关节限位，求解失败";
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
                        PLOG_ERROR << "Undefined error!";
                        // is_running_motion = false;
                        setRunState(RunState::Stopped);
                        return -1;
                }
            }
            catch (...) {
                PLOG_ERROR << "Undefined error!";
                // is_running_motion =false;
                setRunState(RunState::Stopped);
                return -1;
            }
        }

        if (ik_count == max_running_count) {
            PLOG_ERROR << "CartToJnt still failed even after " << max_running_count << " attempts";
            // is_running_motion =false;
            setRunState(RunState::Stopped);
            return -1;
        }

        //**-------------------------------**//

        if (asynchronous)  //异步执行
        {
            motion_thread_.reset(new boost::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            // is_running_motion = true;
            setRunState(RunState::Running);
        } else  //同步执行
        {
            motion_thread_.reset(new boost::thread{&Robot::RunMoveL, this, std::ref(traj_)});
            motion_thread_->join();
            motion_thread_ = nullptr;
            // is_running_motion = false;
            setRunState(RunState::Running);
        }

        return 0;
    }


    int Robot::MoveP(Frame pose, double speed, double acceleration, double time,
                     double radius, bool asynchronous) {
        PLOG_ERROR << "have not complicated yet";
        return 0;
    }

    int Robot::MovePath(const Path &path, bool asynchronous) {
        PLOG_ERROR << "have not complicated yet";
        return 0;
    }

    int Robot::MultiMoveL(const std::vector<KDL::Frame> &point, std::vector<double> bound_dist,
                          std::vector<double> max_path_v, std::vector<double> max_path_a, bool asynchronous,
                          int max_running_count) {

        for (int i{0}; i < jointNum; i++) {
            if (joints_[i]->getMode() != ModeOfOperation::CyclicSynchronousPositionMode) {
                PLOG_ERROR << " 需要关节[" << i << "]进入位置伺服模式";
                return -1;
            }
        }

        if (is_running_motion)  //最大一条任务异步执行
        {
            PLOG_ERROR << " Motion is still running and waiting for it to finish";
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
                PLOG_ERROR << "MultiMoveL(): All vectors must be the same size";
                return -1;
            }
        }
        //**-------------------------------**//

        //** 规划 **//

        if (point.size() == 0) {
            PLOG_ERROR << "MultiMoveL(): point size is at least one or more";
            return -1;
        }
            //一段轨迹不存在圆弧过渡处理
        else if (point.size() == 1) {
            std::cout << GREEN << "***************第1次规划***************" << WHITE << std::endl;
            if (JC_helper::link_trajectory(traj_target, Cart_point, point[0], 0, 0, max_path_v[0], max_path_a[0]) < 0) {
                std::cerr << RED << "MultiMoveL(): given parameters is invalid in the 1th planning "
                          << WHITE << std::endl;
                return -1;
            }
            traj_index.push_back(traj_target.size());
        } else {
            KDL::Frame motion_frame_1;
            KDL::Frame motion_frame_2;
            double motion_v_1;
            double motion_v_2;
            int success{0};

            std::cout << GREEN << "***************第1次规划***************" << WHITE << std::endl;
            success = JC_helper::multilink_trajectory(traj_target, Cart_point, point[0], point[1], motion_frame_1, 0,
                                                      motion_v_1, bound_dist[0], max_path_v[0], max_path_a[0],
                                                      max_path_v[1]);
            if (success < 0) {
                PLOG_ERROR << " given parameters is invalid in the 1th planning ";
                return -1;
            }
            traj_index.push_back(traj_target.size());

            for (int i = 1; i < (point.size() - 1); i++) {
                std::cout << GREEN << "***************第" << i + 1 << "次规划***************" << WHITE << std::endl;
                success = JC_helper::multilink_trajectory(traj_target, motion_frame_1, point[i], point[i + 1],
                                                          motion_frame_2, motion_v_1, motion_v_2, bound_dist[i],
                                                          max_path_v[i], max_path_a[i], max_path_v[i + 1]);
                if (success < 0) {
                    PLOG_ERROR << "given parameters is invalid in the " << i + 1 << "th planning ";
                    return -1;
                }
                motion_frame_1 = motion_frame_2;
                motion_v_1 = motion_v_2;
                traj_index.push_back(traj_target.size());
            }

            std::cout << GREEN << "***************第" << point.size() << "次规划***************" << WHITE << std::endl;
            success = JC_helper::link_trajectory(traj_target, motion_frame_1, point.back(), motion_v_1, 0,
                                                 max_path_v.back(), max_path_a.back());
            if (success < 0) {
                PLOG_ERROR << " given parameters is invalid in the last of planning ";
                return -1;
            }
            traj_index.push_back(traj_target.size());
        }
        PLOG_INFO << "***************规划全部完成***************";

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
                PLOG_INFO << "---------------------------------------";

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

                            PLOG_ERROR << "joint[" << i << "] speep is too  fast on the " << traj_current_pos
                                       << "TH trajectory";
                            PLOG_ERROR << "target speed = " << abs(q_target(i) - q_init(i))
                                       << " and  max_step=" << max_step[i];
                            PLOG_ERROR << "q_target( " << i << " )  = " << q_target(i) * 180 / M_PI;
                            PLOG_ERROR << "q_init( " << i << " ) =" << q_init(i) * 180 / M_PI << WHITE;

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
                        PLOG_ERROR << "CartToJnt failed "
                                   << "on the " << traj_current_pos
                                   << "TH trajectory,please chose other interpolate Points ";

                        break;
                    }
                    case -2:
                        break;
                    default:
                        PLOG_ERROR << "Undefined error!";
                        // is_running_motion = false;
                        setRunState(RunState::Stopped);
                        return -1;
                }
            }
            catch (...) {
                PLOG_ERROR << "Undefined error!";
                // is_running_motion = false;
                setRunState(RunState::Stopped);
                return -1;
            }
        }

        if (ik_count == max_running_count) {
            PLOG_ERROR << "\n\nCartToJnt still failed even after " << max_running_count << " attempts";
            // is_running_motion = false;
            setRunState(RunState::Stopped);
            return -1;
        }

        //**-------------------------------**//


        // //** IK计算 **//
        // traj_.clear();
        // int count{0};
        // int p = 0;  //表示当前正处理第几段轨迹
        // for (const auto &pos_goal: traj_target) {
        //     q_init = q_target;
        //     if (kinematics_.CartToJnt(q_init, pos_goal, q_target) < 0) {
        //         for (int i = 0; i < traj_index.size(); i++)
        //             p = count < traj_index[i] ? i + 1 : p;  //找到当前是第几段轨迹
        //         std::cerr << RED << "MultiMoveL():CartToJnt failed on the " << p
        //                   << "TH trajectory,please chose other interpolate Points "
        //                   << WHITE << std::endl;
        //         return -1;
        //     }
        //     //** 防止奇异位置速度激增 **//
        //     for ( int i = 0; i < jnt_num_; i++ )
        //     {
        //         if ( abs( q_target( i ) - q_init( i ) ) > max_step[ i ] )
        //         {
        //             for ( int j = 0; j < traj_index.size( ); j++ )
        //                 p = count < traj_index[ j ] ? ( j + 1 ) : p;  //找到当前是第几段轨迹
        //             // PLOG_ERROR << "count =" << count;
        //             PLOG_ERROR << "joint[" << i << "] speep is too  fast on the " << p
        //                        << "th trajectory";
        //             PLOG_ERROR << "target speed = " << abs( q_target( i ) - q_init( i ) )
        //                        << " and  max_step=" << max_step[ i ];
        //             PLOG_ERROR << "q_target( " << i << " )  = " << q_target( i ) * 180 / M_PI;
        //             PLOG_ERROR << "q_init( " << i << " ) =" << q_init( i ) * 180 / M_PI << WHITE;


        //             return -1;
        //         }
        //     }
        //     //**-------------------------------**//
        //     traj_.push_back(q_target);
        //     count++;
        // }
        // //**-------------------------------**//


        if (asynchronous)  //异步执行
        {
            motion_thread_.reset(new boost::thread{&Robot::RunMultiMoveL, this, std::ref(traj_)});
            // is_running_motion = true;
            setRunState(RunState::Running);
        } else  //同步执行
        {
            motion_thread_.reset(new boost::thread{&Robot::RunMultiMoveL, this, std::ref(traj_)});
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
        static std::shared_ptr<boost::thread> _thread_planning{nullptr};
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
            PLOG_ERROR << "未定义指令：" << index;
            return -1;
        }

        if (index <= static_cast< int >( DRAGGING_FLAG::J6 ))  //当前命令类型为关节空间
        {
            //只检查速度、加速度,关节位置指令这里不检查，如果超过范围，则为最大/小关节值
            if (CheckBeforeMove(JntArray{static_cast< unsigned int >( jnt_num_ )}, max_speed, max_acceleration, 0, 0) <
                0) {
                PLOG_ERROR << "given parameters is invalid";
                return -1;
            }
            //预防机械臂6个关节时，下发第7关节的控制命令
            if (index >= jnt_num_) {
                PLOG_ERROR << " command flag= "
                           << " DRAGGING_FLAG::J6 "
                           << " is not allow because of the jnt_num_=" << jnt_num_;
                return -1;
            }
            index_type = DRAGGING_TYPE::JOINT;
        } else if (index <= static_cast< int >( DRAGGING_FLAG::BASE_YAW )) //当前命令类型为笛卡尔空间
        {
            //只检查速度、加速度,笛卡尔指令不检查
            if (CheckBeforeMove(flange_, max_speed, max_acceleration, 0, 0) < 0) {
                PLOG_ERROR << "given parameters is invalid";
                return -1;
            }
            index_type = DRAGGING_TYPE::CARTESIAN;
        } else if (index <= static_cast< int >( DRAGGING_FLAG::NULLSPACE ))  // 当前指令为零空间运动
        {
            if (jnt_num_ < 7) {
                PLOG_ERROR << "当前机械臂关节数量为:" << jnt_num_ << ",无法实现零空间点动";
                return -1;
            }
            index_type = DRAGGING_TYPE::NULLSPACE;
        } else  // 未定义指令
        {
            PLOG_ERROR << "未定义指令：" << index;
            return -1;
        }
        //**-------------------------------**//

        //** 禁止在运动中，各种点动来回切换**//
        if (index_type != last_index_type && !_dragging_finished_flag) {
            PLOG_ERROR << "不允许点动指令运行中切换点动类型";
            return -1;
        }
        // 三种情况能通过检查：没改没完成、没改完成了、改了完成了
        last_index_type = index_type;
        //**-------------------------------**//

        if(dir == DRAGGING_DIRRECTION::NONE) { //说明想要停止了
            tick_count += 250; //超过100就会停止
//            PLOG_INFO << "停止点动";
            dir = last_dir;
        }
        else {
            last_dir = dir;
        }

        //** is_running_motion的作用：不允许其他运动异步运行时,执行dragging;不允许执行dragging时，执行其他离线类运动**//
        //** _dragging_finished_flag的作用：保证dragging 多次调用时，只初始化一次**//
        if (_dragging_finished_flag && is_running_motion) {

            PLOG_ERROR << "其他运动仍在运行，不允许执行点动功能";
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
                    new boost::thread{&JC_helper::SmartServo_Joint::RunSmartServo, &_SmartServo_Joint, this});
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
                    new boost::thread{&JC_helper::SmartServo_Cartesian::RunMotion, &_SmartServo_Cartesian, this});
        }
            // 零空间点动指令
        else if (_dragging_finished_flag && index_type == DRAGGING_TYPE::NULLSPACE) {
            if (_thread_planning) {
                _thread_planning->join();
                _thread_planning = nullptr;
            }

            _SmartServo_Nullsapace.init(this, max_speed);
            _thread_planning.reset(
                    new boost::thread{&JC_helper::SmartServo_Nullspace::RunMotion, &_SmartServo_Nullsapace, this});
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

                // PLOG_WARNING << " 笛卡尔点动功能暂时不支持 {TOOL},替换为{BASE}";

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

                // PLOG_WARNING << " 笛卡尔点动功能暂时不支持 {OBJECT},替换为{BASE}";

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
                PLOG_ERROR << " Undefined command flag: " << index;
                //! 在此处位置时会置位is_running_motion
                //! 如果没有jogging 运动线程 且 is_running_motion 被置位，那is_running_motion就会被永久卡住
                if (_dragging_finished_flag && is_running_motion)
                    // is_running_motion = false;
                    setRunState(RunState::Stopped);
                return -1;
        }
        return 0;
    }

    int Robot::CheckBeforeMove(const JntArray &q, double speed, double acceleration,
                               double time, double radius) {
        //** 数据有效性检查  **//
        //TODO 这里的速度、加速度目前只针对关节空间进行检查
        for (int i = 0; i < jnt_num_; i++) {
            //位置检查
            if (q(i) > joints_[i]->getMaxPosLimit() ||
                q(i) < joints_[i]->getMinPosLimit()) {
                PLOG_ERROR << "  Pos command is out of range";
                return -1;
            }
            //速度检查
            if (speed > joints_[i]->getMaxVel() ||
                speed < (-1) * joints_[i]->getMaxVel()) {
                PLOG_ERROR << " Vel command is out of range";
                return -1;
            }
            //加速度检查
            if (acceleration > joints_[i]->getMaxAcc() ||
                acceleration < (-1) * joints_[i]->getMaxAcc()) {
                PLOG_ERROR << " Acc command is out of range";
                return -1;
            }
            //使能检查
            if (joints_[i]->getDriveState() != DriveState::OperationEnabled) {
                PLOG_ERROR << " joints[" << i << "]"
                           << "is in OperationDisabled ";
                return -1;
            }
        }
        if (time < 0) {
            PLOG_ERROR << "  time is less than 0 invalidly";
            return -1;
        }

        if (time) {
            PLOG_ERROR << " time not supported yet";
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
                PLOG_ERROR << " Vel command is out of range" << WHITE;
                return -1;
            }
            //加速度检查
            if (acceleration > joints_[i]->getMaxAcc() ||
                acceleration < (-1) * joints_[i]->getMaxAcc()) {
                PLOG_ERROR << "Acc command is out of range";
                return -1;
            }
            //使能检查
            if (joints_[i]->getDriveState() != DriveState::OperationEnabled) {
                PLOG_ERROR << "joints[" << i << "]"
                           << "is in OperationDisabled ";
                return -1;
            }
        }
        //**-------------------------------**//
        return 0;
    }
    bool Robot::isEnabled()
    {
        for (int i = 0; i < jnt_num_; i++) {
            //使能检查
            if (joints_[i]->getDriveState() != DriveState::OperationEnabled) {
                PLOG_ERROR << "joints[" << i << "]"
                            << "is in OperationDisabled ";
                return false;
            }
        }

        return true;
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

        // std::cout << "Joint Pos: \n"
        //           << GREEN << q.data << WHITE << std::endl;
        for (int i = 0; i < jnt_num_; ++i) {
            if (fabs(q(i) - pos_[i]) < EPS) {
//                PLOG_WARNING << " Target pos[" << i << "] is same as  pos_[" << i << "]";
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
                PLOG_ERROR << "Joint[" << i << "] MoveJ trajectory is infeasible";
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
                    PLOG_ERROR << "joint[" << i << "] speep is too  fast";
                    PLOG_ERROR << "target speed = " << abs(target_pos[i] - init_pos[i])
                               << " and  max_speed=" << max_step[i];
                    PLOG_ERROR << "q_target( " << i << " )  = " << target_pos[i] * 180 / M_PI;
                    PLOG_ERROR << "q_init( " << i << " ) =" << init_pos[i] * 180 / M_PI;

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
            if(!isEnabled())
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
        std::cout << "No. of waypoints: " << traj.size() << std::endl;
        
        for (const auto &waypoints: traj) {
            if(!isEnabled())
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
                    PLOG_ERROR << "关节[" << i << "] 不支持此模式 :" << static_cast<int> (joints_[i]->getMode());
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
        std::cout << "No. of waypoints: " << traj.size() << std::endl;

        for (const auto &waypoints: traj) {
            if(!isEnabled())
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
            PLOG_ERROR << " Motion is still running and waiting for it to finish";
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

        PLOG_INFO << "开始示教";

        if (!asynchronous) {
            //** 等待关闭指令 **//
            while (!flag_admittance_turnoff)
                std::this_thread::sleep_for(std::chrono::duration<double>(0.002));
            //**-------------------------------**//

            _thread_admittance_teaching->join();
            _thread_ft_sensor->join();

            PLOG_INFO << "结束示教";

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
            PLOG_ERROR << " Motion is still running and waiting for it to finish";
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

        PLOG_INFO << "开启导纳运动";

        _thread_admittance_link->join();
        _thread_ft_sensor->join();

        PLOG_INFO << "结束导纳运动";

        // is_running_motion = false;
        setRunState(RunState::Stopped);
        return 0;
    }


    int Robot::servoJ(const KDL::JntArray &target_pos) {
        //** 位置检查(解析求解器内置位置检查) **//
        // for ( int i = 0; i < jointNum; ++i )
        //     if ( target_pos( i ) > joints_[ i ]->getMaxPosLimit( ) || target_pos( i ) < joints_[ i ]->getMinPosLimit( ) )
        //     {
        //         PLOG_ERROR << "target pos [" << i << "]= " << target_pos( i ) * KDL::rad2deg << " is out of range ";
        //         hw_interface_->waitForSignal( 0 );
        //         return -1;
        //     }
        //**-------------------------------**//
        //** 速度检查 **//
        Eigen::MatrixXd joint_offset = (target_pos.data - JC_helper::vector_2_JntArray(pos_).data).cwiseAbs();
        for (int i = 0; i < jointNum; ++i)
            if (joint_offset(i) > joints_[i]->getMaxVel() * DELTA_T) {
                PLOG_ERROR << "target vel [" << i << "]= " << joint_offset(i) * KDL::rad2deg * 1000
                           << " deg/s is out of range ";
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
        KDL::JntArray joint_in(jointNum);
        KDL::JntArray joint_out(jointNum);
        joint_in = JC_helper::vector_2_JntArray(pos_);
        if (SRS_kinematics_.JC_cartesian_to_joint(target_frame, joint_in(2), joint_in, joint_out) < 0) {
            PLOG_ERROR << "逆解失败";
            hw_interface_->waitForSignal(0);
            return -1;
        } else
            return servoJ(joint_out);
    }

    int Robot::joint_admittance_teaching(bool asynchronous) {
        if (is_running_motion)  // 最大一条任务异步执行
        {
            PLOG_ERROR << " Motion is still running and waiting for it to finish";
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

        PLOG_INFO << "开始示教";

        if (!asynchronous) {
            //** 等待关闭指令 **//
            while (!flag_admittance_joint_turnoff)
                std::this_thread::sleep_for(std::chrono::duration<double>(0.002));
            //**-------------------------------**//

            _thread_admittance_teaching->join();

            PLOG_INFO << "结束示教";

            delete admittance_control;
            // is_running_motion = false;
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


    // TODO: 未完成,Sun的伺服驱动接口
    // 模仿UR机械臂的servoj函数的实现
    // SERVOJ的函数原理:
    // 目标位置target_pos,当前位置current_pos,当前速度current_vel,当前加速度current_acc,采样周期dt
    // 下一时刻的加速度next_acc = (target_pos - current_pos) *gain+current_vel*lookhead*gain
    // 下一时刻的速度next_vel = current_vel + next_acc*dt
    // 下一时刻的位置next_pos = current_pos + next_vel*dt
    int Robot::sun_servoJ(const KDL::JntArray &target_pos, const KDL::JntArray &max_vel, const KDL::JntArray &max_acc,
                          double Gain, double lookhead) {
        KDL::JntArray current_pose{jointNum};
        KDL::JntArray current_vel{jointNum};
        KDL::JntArray current_acc{jointNum};
        KDL::JntArray next_acc{jointNum};
        KDL::JntArray next_vel{jointNum};
        KDL::JntArray next_pos{jointNum};
        KDL::JntArray next_jerk{jointNum};
        double dt = DELTA_T;
        double look_head2 = 0.6;
        int pre = 0;
        bool is_first = true; // 是否可以一次性到达目标位置，不可以一直循环，可以退出
        for (int i = 0; i < jointNum; ++i) {
            current_pose(i) = pos_[i];
            current_vel(i) = vel_[i];
            current_acc(i) = acc_[i];
            next_acc(i) = 0;
            next_vel(i) = 0;
            next_pos(i) = 0;
            next_jerk(i) = 0;
            std::cout << "target_pos" << target_pos(i) << endl;

        }

        //Eigen::Matrix<double, jointNum, 1> joint_offset = (target_pos.data - JC_helper::vector_2_JntArray(pos_).data).cwiseAbs();
        // while (is_first)

        std::cout << "next_pose: ";
        for (int i = 0; i < jointNum; ++i) {
            next_jerk(i) = (0 - current_vel(i)) * Gain + (0 - current_acc(i)) * look_head2 * Gain +
                           (target_pos(i) - current_pose(i)) * Gain + (0 - current_vel(i)) * lookhead * Gain;
            next_acc(i) = next_jerk(i) * DELTA_T;
            //next_acc(i) = (target_pos(i) - current_pose(i)) * Gain +(0-current_vel(i) )* lookhead * Gain;
            if (next_acc(i) > max_acc(i) * DELTA_T) {
                next_acc(i) = max_acc(i) * DELTA_T;
            } else if (next_acc(i) < -max_acc(i) * DELTA_T) {
                next_acc(i) = -max_acc(i) * DELTA_T;
            }

            next_vel(i) = current_vel(i) + next_acc(i);

            if (next_vel(i) > max_vel(i) * DELTA_T) {
                next_vel(i) = max_vel(i) * DELTA_T;
            } else if (next_vel(i) < -max_vel(i) * DELTA_T) {
                next_vel(i) = -max_vel(i) * DELTA_T;
            }
            next_pos(i) = current_pose(i) + next_vel(i);

            //  if (abs(next_pos(i) - target_pos(i)) < 0.0001)
            // {
            //     pre=pre+1;
            //     // std::cout<<"目标位置: "<<target_pos(i)<<"next pose "<<next_pos(i)<<std::endl;
            // }
            pos_[i] = next_pos(i);
            vel_[i] = next_vel(i);
            acc_[i] = next_acc(i);
            joints_[i]->setPosition(next_pos(i));
            // std::cout<<next_pos(i)<<",";
            // current_pose(i) = next_pos(i);

            // current_vel(i) = next_vel(i);
        }
        std::cout << std::endl;
        // if(pre==jointNum)
        // {
        //     is_first=false;
        //     std::cout<<"over"<<std::endl;

        // }
        // pre=0;
        //**-------------------------------**//
        //** 位置伺服 **//

        hw_interface_->waitForSignal(0);


        //**-------------------------------**//
        return 0;
    }

//    //TODO: 测试代码，未来要移除 by think
//    int Robot::speed_scaling() {
//        //** 全局速度条规划器 **//
//        if (is_fraction_changed && std::abs(target_speed_fraction - current_speed_fraction) >= 0.0001) {
//            if (T_speed_scaling_ptr == nullptr) {
//                T_speed_scaling_ptr.reset(new Trapezoid{});
//            }
//
//            speed_scaling_dt = 0;
//            T_speed_scaling_ptr->planTrapezoidProfile(0, current_speed_fraction, target_speed_fraction,
//                                                      current_speed_fraction_vel, 0, 3.00, 8.0);
//            if (!T_speed_scaling_ptr->isValidMovement() || !(T_speed_scaling_ptr->getDuration() > 0)) {
//                PLOG_ERROR << "seed scaling is infeasible";
//                setRunState(RunState::Stopped);
//                exit(-1);
//            }
//            is_fraction_changed = false;
//        }
//
//        if (std::abs(target_speed_fraction - current_speed_fraction) >= 0.0001 && T_speed_scaling_ptr != nullptr) {
//            current_speed_fraction = T_speed_scaling_ptr->pos(speed_scaling_dt);
//            current_speed_fraction_vel = T_speed_scaling_ptr->vel(speed_scaling_dt);
//            current_speed_fraction_acc = T_speed_scaling_ptr->acc(speed_scaling_dt);
//            speed_scaling_dt += DELTA_T;
//        }
//        //**-------------------------------**//
//
//        return 0;
//    }
//
////TODO: 测试代码，未来要移除 by think
//int Robot::moveJ_with_speed_scaling( const KDL::JntArray& target_pos, double max_vel, double max_acc, double max_jerk )
//    {
//        //**  通过TCP实现动态调速，这是debug使用，不应该存在**//
//
//        static JC_helper::TCP_server speed_scaling_server;
//        static bool flag_TCP_server_init = false;
//        if ( !flag_TCP_server_init )
//        {
//                speed_scaling_server.init( );
//                std::thread( &JC_helper::TCP_server::RunServer, &speed_scaling_server ).detach( );  // 开启服务器
//                flag_TCP_server_init = true;
//        }
//        //**-------------------------------**//
//        //** 常规检查 **//
//        if ( CheckBeforeMove( target_pos, max_vel, max_acc, 0, 0 ) < 0 )
//        {
//                PLOG_ERROR << "given parameters is invalid";
//                return -1;
//        }
//
//        for ( int i{ 0 }; i < jointNum; i++ )
//        {
//            if ( !( joints_[ i ]->getMode( ) == ModeOfOperation::CyclicSynchronousPositionMode  ) )
//            {
//                PLOG_ERROR << "moveJ_with_speed_scaling不支持关节[" << i << "]的当前模式 :" << static_cast< int >( joints_[ i ]->getMode( ) );
//                return -1;
//            }
//        }
//
//        if ( is_running_motion )  // 最大异步执行一条任务
//        {
//            PLOG_ERROR << " Motion is still running and waiting for it to finish";
//            return -1;
//        }
//        else
//        {
//            setRunState( RunState::Running );
//        }
//        //**-------------------------------**//
//
//        auto doubleS    = rocos::DoubleS{ };
//
//        //** 找到哪个关节的运动范围最大 **//
//        double max_step = 0;
//        std::vector< double > pos_offset(jnt_num_);
//        for ( int i = 0; i < jnt_num_; i++ )
//        {
//            pos_offset[ i ] = target_pos( i ) - pos_[ i ];
//            max_step        = max( max_step, std::abs( pos_offset[ i ] ) );
//        }
//        if ( std::abs( max_step ) < 1e-4 )
//        {
//            PLOG_ERROR << " The target pos is the current pos";
//            setRunState( RunState::Stopped );
//            return -1;
//        }
//        //**-------------------------------**//
//
//        doubleS.planDoubleSProfile( 0,  // t
//                                    0,  // p0
//                                    1,  // pf
//                                    0,  // v0
//                                    0,  // vf
//                                    max_vel / max_step, max_acc / max_step, max_jerk / max_step );
//
//        if ( !doubleS.isValidMovement( ) || !( doubleS.getDuration( ) > 0 ) )
//        {
//            PLOG_ERROR << "movej trajectory is infeasible";
//            setRunState( RunState::Stopped );
//            return -1;
//        }
//
//        //** 伺服控制 **//
//        double max_time =  doubleS.getDuration( ) ;
//        double dt       = 0;
//        std::vector< double > pos_init( jnt_num_ );
//        for ( int i = 0; i < jnt_num_; i++ )
//            pos_init[ i ] = pos_[ i ];
//
//        while ( dt <= max_time )
//        {
//            //**  通过TCP实现动态调速，这是debug使用，不应该存在**//
//            if ( speed_scaling_server.flag_receive )
//            {
//                // PLOG_DEBUG << "Received=" << &speed_scaling_server.receive_buff[ 0 ];
//                set_target_speed_frcision( std::stod( &speed_scaling_server.receive_buff[ 0 ] ) * 0.01 );
//                speed_scaling_server.flag_receive = false;
//            }
//            //**-------------------------------**//
//
//            speed_scaling();
//
//            double doubleS_pos             = doubleS.pos( dt );
//            double doubleS_vel             = doubleS.vel( dt ) * current_speed_fraction;
//            double doubleS_acc             = doubleS.acc( dt ) * std::pow( current_speed_fraction, 2 ) + doubleS.vel( dt ) * current_speed_fraction_vel;
//
//            //**  记录数据，不应该存在**//
//            static double last_doubleS_acc = doubleS_acc;
//
//            speed_data_csv << std::setprecision( 5 ) << std::fixed;
//            speed_data_csv << current_speed_fraction << ",";      // 记录数据，不应该存在
//            speed_data_csv << current_speed_fraction_vel << ",";  // 记录数据，不应该存在
//            speed_data_csv << current_speed_fraction_acc<< ",";  // 记录数据，不应该存在
//            speed_data_csv <<  doubleS.vel( dt ) << ",";  // 记录数据，不应该存在
//            speed_data_csv <<  doubleS_vel << ",";  // 记录数据，不应该存在
//            speed_data_csv <<  doubleS.acc( dt ) << ",";  // 记录数据，不应该存在
//            speed_data_csv << doubleS_acc << ",";         // 记录数据，不应该存在
//            speed_data_csv << doubleS.jerk( dt ) << ",";   // 记录数据，不应该存在
//            speed_data_csv << (doubleS_acc-last_doubleS_acc)/DELTA_T << ",";   // 记录数据，不应该存在
//            speed_data_csv << doubleS_acc << ",";    // 记录数据，不应该存在
//            speed_data_csv << last_doubleS_acc << std::endl;   // 记录数据，不应该存在
//
//            last_doubleS_acc = doubleS_acc;
//
//            //**-------------------------------**//
//
//            for ( int i = 0; i < jnt_num_; ++i )
//            {
//                pos_[ i ] = pos_init[ i ] + pos_offset[ i ] * doubleS_pos;
//                vel_[ i ] = pos_offset[ i ] * doubleS_vel;
//                acc_[ i ] = pos_offset[ i ] * doubleS_acc;
//                joints_[ i ]->setPosition( pos_[ i ] );  //! 都设置，自动根据模式选取位置或者速度伺服
//                joints_[ i ]->setVelocity( vel_[ i ] );  //!
//            }
//            dt += DELTA_T*current_speed_fraction;
//
//            hw_interface_->waitForSignal( 0 );
//        }
//        //**-------------------------------**//
//        setRunState( RunState::Stopped );
//        speed_data_csv.close();
//        return 0;
//    }


}  // namespace rocos
