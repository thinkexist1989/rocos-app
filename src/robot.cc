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

#include "robot.h"

namespace rocos {

    Robot::Robot(boost::shared_ptr<HardwareInterface> hw) : hw_interface_(hw) {

        addAllJoints();

        target_positions_.resize(jnt_num_);
        target_positions_prev_.resize(jnt_num_);
        target_velocities_.resize(jnt_num_);
        target_torques_.resize(jnt_num_);
        pos_.resize(jnt_num_);
        vel_.resize(jnt_num_);
        acc_.resize(jnt_num_);
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
        }

        startMotionThread();

    }

    void Robot::addAllJoints() {
        jnt_num_ = hw_interface_->getSlaveNumber();
        joints_.clear();
        for (int i = 0; i < jnt_num_; i++) {
            joints_.push_back(boost::make_shared<Drive>(hw_interface_, i));
            joints_[i]->setMode(ModeOfOperation::CyclicSynchronousPositionMode);
        }
    }

    //TODO: 切换HW指针
    bool Robot::switchHW(boost::shared_ptr<HardwareInterface> hw) {
        return false;
    }


    //TODO: 测试用MoveJ，阻塞运行，需要改为private
    void
    Robot::moveJ(const std::vector<double> &pos, const std::vector<double> &max_vel, const std::vector<double> &max_acc,
                 const std::vector<double> &max_jerk, Robot::Synchronization sync, ProfileType type) {

        if (pos.size() != jnt_num_ || max_vel.size() != jnt_num_ || max_acc.size() != jnt_num_ ||
            max_jerk.size() != jnt_num_) {
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
                    reinterpret_cast<Trapezoid *>(interp[i])->planTrapezoidProfile(0, p0, pos[i], 0, 0, max_vel[i],
                                                                                   max_acc[i]);
                    break;
                case doubleS:
                    interp[i] = new DoubleS;
                    reinterpret_cast<DoubleS *>(interp[i])->planDoubleSProfile(0, p0, pos[i], 0, 0, max_vel[i],
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
            for_each(interp.begin(), interp.end(), [=](R_INTERP_BASE *p) { p->scaleToDuration(max_time); });
        } else if (sync == SYNC_PHASE) {
            std::cout << "[WARNING] Phase sync has not implemented...instead of time sync." << std::endl;
            for_each(interp.begin(), interp.end(), [=](R_INTERP_BASE *p) { p->scaleToDuration(max_time); });
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

            dt += 0.001;
        }

        // delete pointer
        for (auto &p: interp) {
            delete p;
        }
    }

    void Robot::setEnabled() {
        for_each(joints_.begin(), joints_.end(), [=](boost::shared_ptr<Drive> &d) { d->setEnabled(); });
    }

    void Robot::setDisabled() {
        for_each(joints_.begin(), joints_.end(), [=](boost::shared_ptr<Drive> &d) { d->setDisabled(); });
    }

    void Robot::startMotionThread() {
        is_running_ = true;
        otg_motion_thread_ = boost::make_shared<boost::thread>(&Robot::motionThreadHandler, this);
//        boost::thread(&Robot::motionThreadHandler, this);
    }

    void Robot::stopMotionThread() {
        is_running_ = false;
        otg_motion_thread_->interrupt();
        otg_motion_thread_->join(); //等待运动线程结束
    }

    void Robot::motionThreadHandler() {

        std::cout << "Motion thread is running on thread " << boost::this_thread::get_id() << std::endl;

        target_positions_.resize(jnt_num_);
        target_positions_prev_.resize(jnt_num_);
        target_velocities_.resize(jnt_num_);
        target_torques_.resize(jnt_num_);
        pos_.resize(jnt_num_);
        vel_.resize(jnt_num_);
        acc_.resize(jnt_num_);
        max_vel_.resize(jnt_num_);
        max_acc_.resize(jnt_num_);
        max_jerk_.resize(jnt_num_);
        interp_.resize(jnt_num_);
        need_plan_.resize(jnt_num_, false);

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

        std::vector<double> dt(jnt_num_, 0.0); // delta T
        double max_time = 0.0;

        while (is_running_) { // while start

            hw_interface_->waitForSignal(9);

            // Trajectory generating......
            max_time = 0.0;
            for (int i = 0; i < jnt_num_; ++i) {
                if (joints_[i]->getDriveState() != DriveState::OperationEnabled) {
                    target_positions_[i] = joints_[i]->getPosition();
//                    target_positions_prev_[i] = target_positions_[i];

                    continue; // Disabled, ignore
                }

                if (fabs(target_positions_[i] != target_positions_prev_[i]) || need_plan_[i]) { // need update
                    target_positions_prev_[i] = target_positions_[i]; // assign to current target position

                    interp_[i]->planProfile(0, // t
                                            pos_[i], // p0
                                            target_positions_prev_[i], // pf
                                            vel_[i],  // v0
                                            target_velocities_[i], // vf
                                            max_vel_[i],
                                            max_acc_[i],
                                            max_jerk_[i]);

                    dt[i] = 0.0; // once regenerate, dt = 0.0
                    // consider at least execute time
                    if (interp_[i]->getDuration() < least_motion_time_) {
                        interp_[i]->scaleToDuration(least_motion_time_);
                    }
                    // record max_time
                    max_time = max(max_time, interp_[i]->getDuration()); // get max duration time

                    need_plan_[i] = false;
                }
            }

            // Sync scaling....
            if (sync_ == SYNC_TIME) {
                for_each(interp_.begin(), interp_.end(), [=](R_INTERP_BASE *p) { p->scaleToDuration(max_time); });
            } else if (sync_ == SYNC_NONE) {

            } else if (sync_ == SYNC_PHASE) {
                std::cout << "[WARNING] Phase sync has not implemented...instead of time sync." << std::endl;
                for_each(interp_.begin(), interp_.end(), [=](R_INTERP_BASE *p) { p->scaleToDuration(max_time); });
            }


            // Start moving....
            for (int i = 0; i < jnt_num_; ++i) {
                if (joints_[i]->getDriveState() != DriveState::OperationEnabled) {
                    continue; // Disabled, ignore
                }

                if (interp_[i]->isValidMovement()) {
                    dt[i] += 0.001;
                    pos_[i] = interp_[i]->pos(dt[i]);  //当前位置更新
                    vel_[i] = interp_[i]->vel((dt[i]));
                    acc_[i] = interp_[i]->acc(dt[i]);

                    switch (joints_[i]->getMode()) {
                        case ModeOfOperation::CyclicSynchronousPositionMode:
                            joints_[i]->setPosition(pos_[i]);
                            break;
                        case ModeOfOperation::CyclicSynchronousVelocityMode:
                            joints_[i]->setVelocity(vel_[i]);
                            break;
                        default:
                            std::cout << "Only Supported CSP and CSV" << std::endl;
                    }

                } else {
                    vel_[i] = 0.0;
                }

            }

        }

        // process before exit:



    }

    void Robot::moveJ(const vector<double> &target_pos, const vector<double> &target_vel, Robot::Synchronization sync) {
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
        double dt = fabs(vel_[id]) / max_acc_[id]; // 所需要的减速时间
        target_positions_[id] = pos_[id] + 2*( dt * vel_[id] / 2.0); //TODO：这个减速段计算有问题
//        target_positions_[id] = pos_[id];
        target_velocities_[id] = 0.0;
        least_motion_time_ = 0.0;

        auto sync = sync_;
        sync_ = SYNC_NONE; //停止时候就不需要同步了

        std::cout << "max_acc: " << max_acc_[id] << "; pos: " << pos_[id] << "; vel: " << vel_[id] << std::endl;
        std::cout << "dt: " << dt << "; target_positions: " << target_positions_[id] << std::endl;

        need_plan_[id] = true;

//        usleep(dt * 1000000);
//        sync_ = sync;
    }

    void Robot::stopMultiAxis() {
//        auto sync = sync_;
        sync_ = SYNC_NONE; //停止时候就不需要同步了

        double wait_time = 0.0;

        for (int id = 0; id < jnt_num_; ++id) {
            double dt = fabs(vel_[id]) / max_acc_[id]; // 所需要的减速时间
            target_positions_[id] = pos_[id] + 2*( dt * vel_[id] / 2.0); //TODO：这个减速段计算有问题
//        target_positions_[id] = pos_[id];
            target_velocities_[id] = 0.0;
            least_motion_time_ = 0.0;

            std::cout << "max_acc: " << max_acc_[id] << "; pos: " << pos_[id] << "; vel: " << vel_[id] << std::endl;
            std::cout << "dt: " << dt << "; target_positions: " << target_positions_[id] << std::endl;

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
    void Robot::moveSingleAxis(int id, double pos, double vel, double max_vel, double max_acc, double max_jerk,
                               double least_time) {
        target_positions_[id] = pos;
        target_velocities_[id] = vel;

        if (max_vel != -1)
            max_vel_[id] = max_vel;
        if (max_acc != -1)
            max_acc_[id] = max_acc;
        if (max_jerk != -1)
            max_jerk_[id] = max_jerk;
        if (least_time != -1)
            least_motion_time_ = least_time;

        need_plan_[id] = true;
    }

    /// 设置多轴运动
    /// \param target_pos 目标位置
    /// \param target_vel 目标速度
    /// \param max_vel 最大速度
    /// \param max_acc 最大加速度
    /// \param max_jerk 最大加加速度
    /// \param least_time 最短运行时间
    void Robot::moveMultiAxis(const vector<double> &target_pos, const vector<double> &target_vel,
                              const vector<double> &max_vel, const vector<double> &max_acc,
                              const vector<double> &max_jerk, double least_time) {
        if ((target_pos.size() != jnt_num_) || (target_vel.size() != jnt_num_) || (max_vel.size() != jnt_num_) ||
            (max_acc.size() != jnt_num_) || (max_jerk.size() != jnt_num_)) {
            std::cout << "[ERROR] moveMultiAxis: wrong size!" << std::endl;
        }

        for (int id = 0; id < jnt_num_; ++id) {
            target_positions_[id] = target_pos[id];
            target_velocities_[id] = target_vel[id];

            if (max_vel[id] != -1)
                max_vel_[id] = max_vel[id];
            if (max_acc[id] != -1)
                max_acc_[id] = max_acc[id];
            if (max_jerk[id] != -1)
                max_jerk_[id] = max_jerk[id];

            need_plan_[id] = true;
        }

        if (least_time != -1)
            least_motion_time_ = least_time;
    }


}