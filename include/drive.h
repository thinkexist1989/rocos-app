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

#ifndef ROCOS_APP_DRIVE_H
#define ROCOS_APP_DRIVE_H

#include <include/hardware_interface.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <drive_guard.h>

#include <interpolate.h>

namespace rocos {

    class Drive {
        friend class DriveGuard;

    public:
        Drive(boost::shared_ptr<HardwareInterface> hw, int id);

        bool setDriverState(const DriveState &driveState, bool waitForState);

        bool setEnabled(bool waitForState = true);

        bool setDisabled(bool waitForState = true);

        void waitForSignal();

        inline Controlword getControlword() { return controlword_; }

        inline Statusword getStatusword() { return statusword_; }

        inline DriveState getDriveState() { return current_drive_state_; }

        inline int getDriveStateRPC() {
            if (current_drive_state_ == DriveState::Fault)
                return 3;
            else if (current_drive_state_ == DriveState::OperationEnabled)
                return 2;
            else if (current_drive_state_ == DriveState::NA)
                return 0;
            else
                return 1;
        }

        void setMode(ModeOfOperation mode);

        void setPositionInCnt(int32_t pos);

        void setVelocityInCnt(int32_t vel);

        void setTorqueInCnt(int16_t tor);

        void setPosition(double pos);

        void setVelocity(double vel);

        void setTorque(double tor);

        int32_t getPositionInCnt();

        int32_t getVelocityInCnt();

        int16_t getTorqueInCnt();

        int16_t getLoadTorqueInCnt();

        double getPosition();

        double getVelocity();

        double getTorque();

        double getLoadTorque();

        inline int getId() const { return id_; }

        inline std::string getName() { return hw_interface_->getSlaveName(id_); }
        inline void setName(const string& name) { name_ = name; } // 设置驱动器名称，在ecat_config.yaml的无所谓，
                                                                  // 目前是以urdf中的joint name为准，通过hardware id对应

        inline double getMinPosLimit() const { return min_pos_limit_; }
        inline void setMinPosLimit(double min_pos) { min_pos_limit_ = min_pos; }

        inline double getMaxPosLimit() const { return max_pos_limit_; }
        inline void setMaxPosLimit(double max_pos) { max_pos_limit_ = max_pos; }

        inline ModeOfOperation getMode() const { return mode_; }

        void moveToPositionInCnt(int32_t pos, double max_vel, double max_acc,
                                 double max_jerk = std::numeric_limits<double>::max(),
                                 ProfileType type = trapezoid); // Motion with interpolate

    private:
        std::string name_{}; // 关节名称
        Statusword statusword_{}; // 状态字
        Controlword controlword_{}; // 控制字
        DriveState current_drive_state_{DriveState::NA}; // 驱动器当前状态
        DriveState target_drive_state_{DriveState::NA}; // 驱动器目标状态

        ModeOfOperation mode_{ModeOfOperation::CyclicSynchronousPositionMode};

        bool conduct_state_change_{false}; //是否启动状态机 默认不启动
        std::atomic<bool> state_change_successful_{false}; //当前状态切换是否成功

        Timestamp drive_state_change_time_point_;

        uint16_t num_of_successful_target_state_readings_{0};

        mutable boost::recursive_mutex mutex_; // TODO: change name!!!!

        double ratio_{1.0}; // TODO: 暂时没用 Ratio = input / output

        int32_t offset_pos_cnt_{0}; // zero position in Cnt 零位偏移量

        double cnt_per_unit_{131072 / (2*M_PI) };// 每个单位对应脉冲数，比如cnt/rad, cnt/r, cnt/m 2^17=131072
//        double cnt_per_unit_{991232.0 / (2 * M_PI) };// 每个单位对应脉冲数，比如cnt/rad, cnt/r, cnt/m 2^17=131072

        double torque_per_unit_{1.0}; // 每个力矩单位对应的脉冲数，比如cnt/N，通常返回值是千分之一

        std::string user_unit_name_{"rad"}; //用户单位名称

    protected:

        void engageStateMachine();

        /// Get next control word according to the requested drive state and current drive state
        /// \param requestedDriveState
        /// \param currentDriveState
        /// \return the controlword to be sent
        Controlword getNextStateTransitionControlword(const DriveState &requestedDriveState,
                                                      const DriveState &currentDriveState);

    public:
        ///////////规划约束相关///////////////
        inline void setMaxVel(double maxVel) { max_vel_ = maxVel; } // 设置最大速度

        inline double getMaxVel() const { return max_vel_; } // 获取最大速度

        void setMaxAcc(double maxAcc) { max_acc_ = maxAcc; } // 设置最大加速度

        inline double getMaxAcc() const { return max_acc_; } // 获取最大加速度

        inline void setMaxJerk(double maxJerk) { max_jerk_ = maxJerk; } // 设置最大加加速度

        inline double getMaxJerk() const { return max_jerk_; } // 获取最大加加速度

        ///////////单位转换相关///////////////
        inline void setPosZeroOffset(int32_t offset) { offset_pos_cnt_ = offset; } // 设置位置零点偏移

        inline int32_t getPosZeroOffset() const { return offset_pos_cnt_; } //获取位置零点偏移

        inline void setCntPerUnit(double val) { cnt_per_unit_ = val; } // 设置位置、速度转换

        inline double getCntPerUnit() const { return cnt_per_unit_; } // 获取位置、速度转换

        inline void setTorquePerUnit(double val) { torque_per_unit_ = val; } // 设置力矩转换

        inline double getTorquePerUnit() const { return torque_per_unit_; } // 获取力矩转换

        inline void setRatio(double ratio) { ratio_ = ratio; } // 设置减速比

        inline double getRatio() const { return ratio_; }; // 获取减速比

        inline void setUserUnitName(const std::string& name) { user_unit_name_ = name; } // 设置用户单位名称

        inline std::string getUserUnitName() const { return user_unit_name_; } // 获取用户单位名称


    protected:

        boost::shared_ptr<HardwareInterface> hw_interface_{nullptr}; // The pointer of HardwareInterface instance
        int id_{0}; // drive id in bus
        double reduction_ratio_{1.0}; // reduction ratio
        double min_pos_limit_{-2.0};
        double max_pos_limit_{2.0};

        //TODO: 变换单位
        double max_vel_{1.0}; // [UserUnit]/s，比如 rad/s, mm/s
        double max_acc_{10.0}; // [UserUnit]/s^2，比如 rad/s^2
        double max_jerk_{100.0}; // [UserUnit]/s^3，比如 rad/s^3


        bool is_enabled_{false};

        boost::shared_ptr<DriveGuard> drive_guard_{nullptr};

    };
}


#endif //ROCOS_APP_DRIVE_H
