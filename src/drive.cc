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

#include <drive.h>


namespace rocos {

    Drive::Drive(boost::shared_ptr<HardwareInterface> hw, int id) : hw_interface_(hw),
                                                                    id_(id) {
        if(id_ < 0) {
            std::cout << "[ERROR][rocos::Drive] Wrong hardware ID of joint" << std::endl;
            return;
        }

        drive_guard_ = DriveGuard::getInstance(); // 获取DriveGuard单例句柄
        drive_guard_->addDrive(this); // 将Drive实例添加到_driveGuard中来更新数据

        current_drive_state_ = hw_interface_->getDriverState(id_);
        std::cout << "Curr Drive State: " << current_drive_state_ << std::endl;
//        target_drive_state_ = DriveState::SwitchOnDisabled;
        target_drive_state_ = current_drive_state_;

        hw_interface_->setTargetPositionRaw(id_, hw_interface_->getActualPositionRaw(id_)); // set Current Pos

        hw_interface_->setModeOfOperation(id_, mode_); // Default CSP

    }

    bool Drive::setDriverState(const DriveState &driveState, bool waitForState) {
        bool success = false;
        /*
        ** locking the mutex_
        ** This is not done with a lock_guard here because during the waiting time the
        ** mutex_ must be unlocked periodically such that PDO writing (and thus state
        ** changes) may occur at all!
        */
//        mutex_.lock();

        // reset the "stateChangeSuccessful_" flag to false such that a new successful
        // state change can be detected
        state_change_successful_ = false;

        // make the state machine realize that a state change will have to happen
        conduct_state_change_ = true;

        // overwrite the target drive state
        target_drive_state_ = driveState;

        // set the hasRead flag to false such that at least one new reading will be
        // available when starting the state change
//        hasRead_ = false;

        // set the time point of the last pdo change to now
        drive_state_change_time_point_ = boost::chrono::system_clock::now();

        // set a temporary time point to prevent getting caught in an infinite loop
        auto driveStateChangeStartTimePoint = boost::chrono::system_clock::now();

        // return if no waiting is requested
        if (!waitForState) {
            // unlock the mutex
//            mutex_.unlock();
            // return true if no waiting is requested
            return true;
        }

        // Wait for the state change to be successful
        // during the waiting time the mutex MUST be unlocked!

        while (true) {
            // break loop as soon as the state change was successful
            if (state_change_successful_) {
                success = true;
                break;
            }

            // break the loop if the state change takes too long
            // this prevents a freezing of the end user's program if the hardware is not
            // able to change it's state.
            auto duration_us = boost::chrono::duration_cast<boost::chrono::microseconds>(
                    boost::chrono::system_clock::now() - driveStateChangeStartTimePoint);
            if (duration_us.count() > 150000) { //wait for 100ms  TODO: configuration_.driveStateChangeMaxTimeout
                std::cout << "It takes too long (" << duration_us.count() / 1000.0 << " ms) to switch state!"
                          << std::endl;
                break;
            }
            // unlock the mutex during sleep time
//            mutex_.unlock();
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
            // lock the mutex to be able to check the success flag
//            mutex_.lock();
        }
        // unlock the mutex one last time
//        mutex_.unlock();
        return success;
    }

    /// \brief 获取下一个状态控制字
    /// \param requestedDriveState 请求的驱动器状态
    /// \param currentDriveState  当前的驱动器状态
    /// \return 返回控制字
    Controlword Drive::getNextStateTransitionControlword(const DriveState &requestedDriveState,
                                                         const DriveState &currentDriveState) {
        Controlword controlword;
//        controlword = controlword_;
        controlword.setAllFalse();
        switch (requestedDriveState) {
            case DriveState::SwitchOnDisabled:
                switch (currentDriveState) {
                    case DriveState::SwitchOnDisabled:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "drive state has already been reached for '" << name_ << "'" << std::endl;
//                        MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::getNextStateTransitionControlword] "
//                                                  << "drive state has already been reached for '"
//                                                  << name_ << "'");
//                        addErrorToReading(ErrorType::PdoStateTransitionError);
                        break;
                    case DriveState::ReadyToSwitchOn:
                        controlword.setStateTransition7();
                        break;
                    case DriveState::SwitchedOn:
                        controlword.setStateTransition10();
                        break;
                    case DriveState::OperationEnabled:
                        controlword.setStateTransition9();
                        break;
                    case DriveState::QuickStopActive:
                        controlword.setStateTransition12();
                        break;
                    case DriveState::Fault:
                        controlword.setStateTransition15();
                        break;
                    default:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "PDO state transition not implemented for '" << name_ << "'" << std::endl;
//                        MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::getNextStateTransitionControlword] "
//                                                  <<
//                                                  << name_ << "'");
//                        addErrorToReading(ErrorType::PdoStateTransitionError);
                }
                break;

            case DriveState::ReadyToSwitchOn:
                switch (currentDriveState) {
                    case DriveState::SwitchOnDisabled:
                        controlword.setStateTransition2();
                        break;
                    case DriveState::ReadyToSwitchOn:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "drive state has already been reached for '" << name_ << "'" << std::endl;
                        break;
                    case DriveState::SwitchedOn:
                        controlword.setStateTransition6();
                        break;
                    case DriveState::OperationEnabled:
                        controlword.setStateTransition8();
                        break;
                    case DriveState::QuickStopActive:
                        controlword.setStateTransition12();
                        break;
                    case DriveState::Fault:
                        controlword.setStateTransition15();
                        break;
                    default:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "PDO state transition not implemented for '" << name_ << "'" << std::endl;

                }
                break;

            case DriveState::SwitchedOn:
                switch (currentDriveState) {
                    case DriveState::SwitchOnDisabled:
                        controlword.setStateTransition2();
                        break;
                    case DriveState::ReadyToSwitchOn:
                        controlword.setStateTransition3();
                        break;
                    case DriveState::SwitchedOn:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "drive state has already been reached for '" << name_ << "'" << std::endl;
                        break;
                    case DriveState::OperationEnabled:
                        controlword.setStateTransition5();
                        break;
                    case DriveState::QuickStopActive:
                        controlword.setStateTransition12();
                        break;
                    case DriveState::Fault:
                        controlword.setStateTransition15();
                        break;
                    default:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "PDO state transition not implemented for '" << name_ << "'" << std::endl;

                }
                break;

            case DriveState::OperationEnabled:
                switch (currentDriveState) {
                    case DriveState::SwitchOnDisabled:
                        controlword.setStateTransition2();
                        break;
                    case DriveState::ReadyToSwitchOn:
                        controlword.setStateTransition3();
                        break;
                    case DriveState::SwitchedOn:
                        controlword.setStateTransition4();
                        break;
                    case DriveState::OperationEnabled:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "drive state has already been reached for '" << name_ << "'" << std::endl;
                        break;
                    case DriveState::QuickStopActive:
                        controlword.setStateTransition12();
                        break;
                    case DriveState::Fault:
                        controlword.setStateTransition15();
                        break;
                    default:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "PDO state transition not implemented for '" << name_ << "'" << std::endl;

                }
                break;

            case DriveState::QuickStopActive:
                switch (currentDriveState) {
                    case DriveState::SwitchOnDisabled:
                        controlword.setStateTransition2();
                        break;
                    case DriveState::ReadyToSwitchOn:
                        controlword.setStateTransition3();
                        break;
                    case DriveState::SwitchedOn:
                        controlword.setStateTransition4();
                        break;
                    case DriveState::OperationEnabled:
                        controlword.setStateTransition11();
                        break;
                    case DriveState::QuickStopActive:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "drive state has already been reached for '" << name_ << "'" << std::endl;
                        break;
                    case DriveState::Fault:
                        controlword.setStateTransition15();
                        break;
                    default:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "PDO state transition not implemented for '" << name_ << "'" << std::endl;

                }
                break;

            default:
                std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                          << "PDO state cannot be reached for '" << name_ << "'" << std::endl;

        }

        return controlword;
    }

    /// \brief 处理状态机，在DriveGuard线程中循环调用
    void Drive::engageStateMachine() {
// locking the mutex
//        boost::lock_guard<boost::recursive_mutex> lock(mutex_);

        // elapsed time since the last new controlword
        auto microsecondsSinceChange =
                (boost::chrono::duration_cast<boost::chrono::microseconds>(
                        boost::chrono::system_clock::now() - drive_state_change_time_point_)).count();

        // get the current state
        // since we wait until "hasRead" is true, this is guaranteed to be a newly
        // read value
//        const DriveState currentDriveState = reading_.getDriveState();

        // check if the state change already was successful:
        if (current_drive_state_ == target_drive_state_) {
            num_of_successful_target_state_readings_++;
//            if (numberOfSuccessfulTargetStateReadings_ >= configuration_.minNumberOfSuccessfulTargetStateReadings) {
            if (num_of_successful_target_state_readings_ >= 3) {
                // disable the state machine
                conduct_state_change_ = false;
                num_of_successful_target_state_readings_ = 0;
                state_change_successful_ = true;
                return;
            }
//        } else if (microsecondsSinceChange > configuration_.driveStateChangeMinTimeout) {
        } else if (microsecondsSinceChange > 20000) {
            // get the next controlword from the state machine
            controlword_ = getNextStateTransitionControlword(target_drive_state_, current_drive_state_);
            drive_state_change_time_point_ = boost::chrono::system_clock::now();
//            std::cout << controlword_ << std::endl;
            hw_interface_->setControlwordRaw(id_, controlword_.getRawControlword()); // set control word

        }

        // set the "hasRead" variable to false such that there will definitely be a
        // new reading when this method is called again
//        hasRead_ = false;
    }

    /// \brief 上使能驱动器
    /// \param waitForState 是否阻塞？
    /// \return 是否上使能成功
    bool Drive::setEnabled(bool waitForState) {
        return setDriverState(DriveState::OperationEnabled, waitForState);
    }

    /// \brief 下使能驱动器
    /// \param waitForState 是否阻塞？
    /// \return 是否下使能成功
    bool Drive::setDisabled(bool waitForState) {
        return setDriverState(DriveState::SwitchOnDisabled, waitForState);
    }

    /// \brief 等待同步信号
    void Drive::waitForSignal() {
        hw_interface_->waitForSignal(id_);
    }

    /// \brief 设置驱动器模式
    /// \param mode CSP, CSV, CST
    void Drive::setMode(ModeOfOperation mode) {
        mode_ = mode;
        hw_interface_->setModeOfOperation(id_, mode_);
    }

    /// \brief 获取位置
    /// \return 位置值[User Custom Unit]
    double Drive::getPosition() {
        return getPositionInCnt() / cnt_per_unit_; // 单位转换
    }

    /// \brief 获取速度
    /// \return 速度值[User Custom Unit]
    double Drive::getVelocity() {
        return getVelocityInCnt() / cnt_per_unit_; // 单位转换
    }

    /// \brief 获取力矩
    /// \return 力矩值
    double Drive::getTorque() {
        return getTorqueInCnt(); // TODO:单位转换
    }

    double Drive::getLoadTorque() {
        return getLoadTorqueInCnt();
    }

    void Drive::setPositionInCnt(int32_t pos) {
        hw_interface_->setTargetPositionRaw(id_, pos + offset_pos_cnt_); // 加上偏移量
    }

    void Drive::setVelocityInCnt(int32_t vel) {
        hw_interface_->setTargetVelocityRaw(id_, vel);
    }

    void Drive::setTorqueInCnt(int16_t tor) {
        hw_interface_->setTargetTorqueRaw(id_, tor);
    }

    int32_t Drive::getPositionInCnt() {
        return hw_interface_->getActualPositionRaw(id_) - offset_pos_cnt_; // 减去偏移量
    }

    int32_t Drive::getVelocityInCnt() {
        return hw_interface_->getActualVelocityRaw(id_);
    }

    int16_t Drive::getTorqueInCnt() {
        return hw_interface_->getActualTorqueRaw(id_);
    }

    int16_t Drive::getLoadTorqueInCnt() {
        return hw_interface_->getLoadTorqueRaw(id_);
    }

    void Drive::moveToPositionInCnt(int32_t pos, double max_vel, double max_acc, double max_jerk, ProfileType type) {
        auto p0 = hw_interface_->getActualPositionRaw(id_);
        R_INTERP_BASE *interp;
        switch (type) {
            case trapezoid:
                interp = new Trapezoid;
                ((Trapezoid *) interp)->planTrapezoidProfile(0, p0, pos, 0, 0, max_vel, max_acc);
                break;
            case doubleS:
                interp = new DoubleS;
                ((DoubleS *) interp)->planDoubleSProfile(0, p0, pos, 0, 0, max_vel, max_acc, max_jerk);
                break;
            default:
                std::cout << "Not Supported Profile Type" << std::endl;
                return;
        }

        double dt = 0.0;
        double duration = interp->getDuration();
        std::cout << "interp duration is: " << duration << std::endl;

        while (dt <= duration && interp->isValidMovement()) {
            waitForSignal();
            switch (mode_) {
                case ModeOfOperation::CyclicSynchronousPositionMode:
                    hw_interface_->setTargetPositionRaw(id_, interp->pos(dt));
                    break;
                case ModeOfOperation::CyclicSynchronousVelocityMode:
                    hw_interface_->setTargetVelocityRaw(id_, interp->vel(dt));
                    break;
                default:
                    std::cout << "Only Supported CSP and CSV" << std::endl;
            }
            dt += 0.001;
        }

        delete interp;

    }

    void Drive::setPosition(double pos) {
        auto posInCnt = static_cast<int32_t>(pos * cnt_per_unit_);
        setPositionInCnt(posInCnt);
    }

    void Drive::setVelocity(double vel) {
        auto velInCnt = static_cast<int32_t>(vel * cnt_per_unit_);
        setVelocityInCnt(velInCnt);
    }

    void Drive::setTorque(double tor) {
        auto torInCnt = static_cast<int16_t>(tor * torque_per_unit_);
        setTorqueInCnt(torInCnt);
    }

}