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

        inline Controlword getControlword() { return _controlword; }

        inline Statusword getStatusword() { return _statusword; }

        inline DriveState getDriveState() { return _currentDriveState; }

        inline int getDriveStateRPC() {
            if (_currentDriveState == DriveState::Fault)
                return 3;
            else if (_currentDriveState == DriveState::OperationEnabled)
                return 2;
            else if (_currentDriveState == DriveState::NA)
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

        inline int getId() const { return _id; }

        inline std::string getName() { return _hw_interface->getSlaveName(_id); }

        inline ModeOfOperation getMode() const { return _mode; }

        void moveToPositionInCnt(int32_t pos, double max_vel, double max_acc,
                                 double max_jerk = std::numeric_limits<double>::max(),
                                 ProfileType type = trapezoid); // Motion with interpolate

    private:
        std::string _name{}; // 关节名称
        Statusword _statusword{}; // 状态字
        Controlword _controlword{}; // 控制字
        DriveState _currentDriveState{DriveState::NA}; // 驱动器当前状态
        DriveState _targetDriveState{DriveState::NA}; // 驱动器目标状态

        ModeOfOperation _mode{ModeOfOperation::CyclicSynchronousPositionMode};

        bool _conductStateChange{false}; //是否启动状态机 默认不启动
        std::atomic<bool> _stateChangeSuccessful{false}; //当前状态切换是否成功

        Timestamp _driveStateChangeTimePoint;

        uint16_t _numberOfSuccessfulTargetStateReadings{0};

        mutable boost::recursive_mutex _mutex; // TODO: change name!!!!

        double _ratio{1.0}; // TODO: 暂时没用 Ratio = input / output

        int32_t _offsetPosInCnt{0}; // zero position in Cnt 零位偏移量

//        double _cntPerUnit{131072 / M_PI};// 每个单位对应脉冲数，比如cnt/rad, cnt/r, cnt/m 2^17=131072
        double _cntPerUnit{991232.0 / M_PI};// 每个单位对应脉冲数，比如cnt/rad, cnt/r, cnt/m 2^17=131072

        double _torquePerUnit{1.0}; // 每个力矩单位对应的脉冲数，比如cnt/N，通常返回值是千分之一

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
        inline void setMaxVel(double maxVel) { _max_vel = maxVel; } // 设置最大速度

        inline double getMaxVel() const { return _max_vel; } // 获取最大速度

        void setMaxAcc(double maxAcc) { _max_acc = maxAcc; } // 设置最大加速度

        inline double getMaxAcc() const { return _max_acc; } // 获取最大加速度

        inline void setMaxJerk(double maxJerk) { _max_jerk = maxJerk; } // 设置最大加加速度

        inline double getMaxJerk() const { return _max_jerk; } // 获取最大加加速度

        ///////////单位转换相关///////////////
        inline void setCntPerUnit(double val) { _cntPerUnit = val; } // 设置位置、速度转换

        inline double getCntPerUnit() { return _cntPerUnit; } // 获取位置、速度转换

        inline void setTorquePerUnit(double val) { _torquePerUnit = val; } // 设置力矩转换

        inline double setTorquePerUnit() { return _torquePerUnit; } // 获取力矩转换


    protected:

        boost::shared_ptr<HardwareInterface> _hw_interface{nullptr}; // The pointer of HardwareInterface instance
        int _id{0}; // drive id in bus
        double _reduction_ratio{1.0}; // reduction ratio
        double _minPosLimit{-2.0};
        double _maxPosLimit{2.0};

        //TODO: 变换单位
        double _max_vel{1.0}; // [UserUnit]/s，比如 rad/s, mm/s
        double _max_acc{1.0}; // [UserUnit]/s^2，比如 rad/s^2
        double _max_jerk{10.0}; // [UserUnit]/s^3，比如 rad/s^3


        bool _isEnabled{false};

        boost::shared_ptr<DriveGuard> _driveGuard{nullptr};

    };
}


#endif //ROCOS_APP_DRIVE_H
