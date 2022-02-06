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

#ifndef ROCOS_APP_ROBOT_H
#define ROCOS_APP_ROBOT_H

#include <hardware_interface.h>
#include <boost/smart_ptr.hpp>
#include <vector>
#include <drive.h>

#include <interpolate.h>


namespace rocos {

    class Robot {
        friend class RobotServiceImpl;

    public:
        enum Synchronization {
            SYNC_NONE,
            SYNC_TIME,
            SYNC_PHASE
        };

        explicit Robot(boost::shared_ptr<HardwareInterface> hw);

        bool switchHW(boost::shared_ptr<HardwareInterface> hw); //切换硬件指针

        void setEnabled();

        inline void setJointEnabled(int id) { _joints[id]->setEnabled(); }

        void setDisabled();

        inline void setJointDisabled(int id) { _joints[id]->setDisabled(); }

        inline void setJointMode(int id, ModeOfOperation mode) { _joints[id]->setMode(mode); }

        inline int getJointNum() const { return _jntNum; }

        inline std::string getJointName(int id) { return _joints[id]->getName(); }

        inline int getJointStatus(int id) { return _joints[id]->getDriveStateRPC(); }

        inline double getJointPosition(int id) { return _joints[id]->getPosition(); }

        inline double getJointVelocity(int id) { return _joints[id]->getVelocity(); }

        inline double getJointTorque(int id) { return _joints[id]->getTorque(); }

        inline double getJointLoadTorque(int id) { return _joints[id]->getLoadTorque(); }

        inline void setJointPosition(int id, double pos) { _joints[id]->setPosition(pos); }

        inline void setJointVelocity(int id, double vel) { _joints[id]->setVelocity(vel); }

        inline void setJointTorque(int id, double tor) { _joints[id]->setTorque(tor); }

        /// 多关节运动
        /// \param target_pos 位置
        /// \param target_vel 速度
        /// \param sync 同步模式
        void moveJ(const std::vector<double> &target_pos,
                   const std::vector<double> &target_vel,
                   Synchronization sync = SYNC_TIME
        );

        /// 单关节运动
        /// \param id 关节ID
        /// \param pos 位置
        /// \param vel 速度
        /// \param max_vel 最大速度
        /// \param max_acc 最大加速度
        /// \param max_jerk 最大加加速度
        /// \param least_time 最短运行时间
        void moveSingleAxis(int id, double pos,
                            double vel = 0.0,
                            double max_vel = -1,
                            double max_acc = -1,
                            double max_jerk = -1,
                            double least_time = -1);

        /// \brief 多轴运动
        /// \param target_pos 位置
        /// \param target_vel 速度
        /// \param max_vel 最大速度
        /// \param max_acc 最大加速度
        /// \param max_jerk 最大加加速度
        /// \param least_time 最短运行时间
        void moveMultiAxis(const std::vector<double> &target_pos,
                           const std::vector<double> &target_vel,
                           const std::vector<double> &max_vel,
                           const std::vector<double> &max_acc,
                           const std::vector<double> &max_jerk,
                           double least_time = -1);

        /// \brief 设置多关节速度约束
        /// \param max_vel 速度约束值
        inline void setJntVelLimits(std::vector<double> &max_vel) {
            _max_vel = max_vel;
            _needPlan.resize(_jntNum, true);
        }

        /// \brief 获取多关节速度约束
        /// \return 速度约束值
        inline std::vector<double> getJntVelLimits() { return _max_vel; };

        /// \brief 设置单关节速度约束
        /// \param id 关节ID
        /// \param max_vel 速度约束值
        inline void setJntVelLimit(int id, double max_vel) {
            _max_vel[id] = max_vel;
            _needPlan[id] = true;
        }

        /// \brief 获取单关节速度约束
        /// \param id 关节ID
        /// \return 速度约束值
        inline double getJntVelLimit(int id) { return _max_vel[id]; }

        /// \brief 设置关节加速度约束
        /// \param max_acc 加速度约束值
        inline void setJntAccLimits(std::vector<double> &max_acc) {
            _max_acc = max_acc;
            _needPlan.resize(_jntNum, true);
        }

        /// \brief 获取多关节加速度约束
        /// \return 加速度约束值
        inline std::vector<double> getJntAccLimits() { return _max_acc; }

        /// \brief 设置单关节加速度约束
        /// \param id 关节ID
        /// \param max_acc 加速度约束值
        inline void setJntAccLimit(int id, double max_acc) {
            _max_acc[id] = max_acc;
            _needPlan[id] = true;
        }

        /// \brief 获取单关节加速度约束
        /// \param id 关节ID
        /// \return 加速度约束值
        inline double getJntAccLimit(int id) { return _max_acc[id]; }

        /// \brief 设置多关节加加速约束
        /// \param max_jerk 多关节加加速约束值
        inline void setJntJerkLimits(std::vector<double> &max_jerk) {
            _max_jerk = max_jerk;
            _needPlan.resize(_jntNum, true);
        }

        /// \brief 获取多关节加加速约束
        /// \return 多关节加加速约束值
        inline std::vector<double> getJntJerkLimits() { return _max_jerk; }

        /// \brief 设置单关节加加速度约束
        /// \param id 关节id
        /// \param max_jerk 关节加加速约束值
        inline void setJntJerkLimit(int id, double max_jerk) {
            _max_jerk[id] = max_jerk;
            _needPlan[id] = true;
        }

        /// \brief 获取单关节加加速度约束
        /// \param id 关节ID
        /// \return 关节加加速约束值
        inline double getJntJerkLimit(int id) { return _max_jerk[id]; }

        /// \brief 设置规划Profile类型
        /// \param type Trapezoid、DoubleS
        inline void setProfileType(ProfileType type) {
            _profileType = type;
            _needPlan.resize(_jntNum, true);
        }

        /// \brief 设置同步模式
        /// \param sync 无同步、时间同步、相位同步
        inline void setSynchronization(Synchronization sync) {
            _sync = sync;
            _needPlan.resize(_jntNum, true);
        }


        /*****轨迹规划线程相关*****/
        void startMotionThread(); // 启动轨迹规划线程
        void stopMotionThread(); // 停止轨迹规划线程
        void motionThreadHandler(); // 轨迹规划相关处理句柄

    protected:
        void addAllJoints();

    public: // TODO: need changed to private
        // 测试用MoveJ
        void moveJ(const std::vector<double> &pos,
                   const std::vector<double> &max_vel,
                   const std::vector<double> &max_acc,
                   const std::vector<double> &max_jerk,
                   Synchronization sync = SYNC_TIME,
                   ProfileType type = ProfileType::trapezoid);

    protected:
        boost::shared_ptr<HardwareInterface> _hw_interface{nullptr};
        std::vector<boost::shared_ptr<Drive>> _joints;

        std::vector<double> _targetPositions;     // 当前目标位置
        std::vector<double> _targetPositionsPrev; // 上一次的目标位置

        std::vector<double> _targetVelocities;
        std::vector<double> _targetTorques;

        std::vector<double> _pos;
        std::vector<double> _vel;
        std::vector<double> _acc;

        std::vector<double> _max_vel;
        std::vector<double> _max_acc;
        std::vector<double> _max_jerk;

        std::vector<R_INTERP_BASE *> _interp;

        ProfileType _profileType{trapezoid};

        int _jntNum;

        double _leastMotionTime{0.0};

        Synchronization _sync{SYNC_TIME};

        bool _isRunning{false};
        bool _isExit{false};

        std::vector<bool> _needPlan; // 是否需要重新规划标志
    };

}


#endif //ROCOS_APP_ROBOT_H
