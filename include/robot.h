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
#include <kinematics.h>


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

        inline void setJointEnabled(int id) { joints_[id]->setEnabled(); }

        void setDisabled();

        inline void setJointDisabled(int id) { joints_[id]->setDisabled(); }

        inline void setJointMode(int id, ModeOfOperation mode) { joints_[id]->setMode(mode); }

        inline int getJointNum() const { return jnt_num_; }

        inline std::string getJointName(int id) { return joints_[id]->getName(); }

        inline int getJointStatus(int id) { return joints_[id]->getDriveStateRPC(); }

        ///////////////////用户单位信息///////////////////////
        inline double getJointPosition(int id) { return joints_[id]->getPosition(); }

        inline double getJointVelocity(int id) { return joints_[id]->getVelocity(); }

        inline double getJointTorque(int id) { return joints_[id]->getTorque(); }

        inline double getJointLoadTorque(int id) { return joints_[id]->getLoadTorque(); }

        inline void setJointPosition(int id, double pos) { joints_[id]->setPosition(pos); }

        inline void setJointVelocity(int id, double vel) { joints_[id]->setVelocity(vel); }

        inline void setJointTorque(int id, double tor) { joints_[id]->setTorque(tor); }

        /////////////////////获取原始信息//////////////////////////
        inline int32_t getJointPositionRaw(int id) { return joints_[id]->getPositionInCnt(); }

        inline int32_t getJointVelocityRaw(int id) { return joints_[id]->getVelocityInCnt(); }

        inline int16_t getJointTorqueRaw(int id) { return joints_[id]->getTorqueInCnt(); }

        inline int16_t getJointLoadTorqueRaw(int id) { return joints_[id]->getLoadTorqueInCnt(); }

        inline void setJointPositionRaw(int id, int32_t pos) { joints_[id]->setPositionInCnt(pos); }

        inline void setJointVelocityRaw(int id, int32_t vel) { joints_[id]->setVelocityInCnt(vel); }

        inline void setJointTorqueRaw(int id, int16_t tor) { joints_[id]->setTorqueInCnt(tor); }

        ////////////////////单位转换////////////////////////////
        inline double getJointCntPerUnit(int id) { return joints_[id]->getCntPerUnit(); }

        inline double getJointTorquePerUnit(int id) { return joints_[id]->getTorquePerUnit(); }

        inline double getJointRatio(int id) { return joints_[id]->getRatio(); }

        inline int32_t getJointPosZeroOffset(int id) { return joints_[id]->getPosZeroOffset(); }

        inline std::string getJointUserUnitName(int id) { return joints_[id]->getUserUnitName(); }

        inline void setJointCntPerUnit(int id, double cnt_per_unit) { joints_[id]->setCntPerUnit(cnt_per_unit); }

        inline void setJointTorquePerUnit(int id, double tor_per_unit) { joints_[id]->setTorquePerUnit(tor_per_unit); }

        inline void setJointRatio(int id, double ratio) { joints_[id]->setRatio(ratio); }

        inline void setJointPosZeroOffset(int id, int32_t offset) { joints_[id]->setPosZeroOffset(offset); }

        inline void setJointUserUnitName(int id, std::string name) { joints_[id]->setUserUnitName(name); }


        /// 多关节运动
        /// \param target_pos 位置
        /// \param target_vel 速度
        /// \param sync 同步模式
        void moveJ(const std::vector<double> &target_pos,
                   const std::vector<double> &target_vel,
                   Synchronization sync = SYNC_TIME
        );


        void stopSingleAxis(int id);
        void stopMultiAxis();

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
            max_vel_ = max_vel;
            need_plan_.resize(jnt_num_, true);
        }

        /// \brief 获取多关节速度约束
        /// \return 速度约束值
        inline std::vector<double> getJntVelLimits() { return max_vel_; };

        /// \brief 设置单关节速度约束
        /// \param id 关节ID
        /// \param max_vel 速度约束值
        inline void setJntVelLimit(int id, double max_vel) {
            max_vel_[id] = max_vel;
            need_plan_[id] = true;
        }

        /// \brief 获取单关节速度约束
        /// \param id 关节ID
        /// \return 速度约束值
        inline double getJntVelLimit(int id) { return max_vel_[id]; }

        /// \brief 设置关节加速度约束
        /// \param max_acc 加速度约束值
        inline void setJntAccLimits(std::vector<double> &max_acc) {
            max_acc_ = max_acc;
            need_plan_.resize(jnt_num_, true);
        }

        /// \brief 获取多关节加速度约束
        /// \return 加速度约束值
        inline std::vector<double> getJntAccLimits() { return max_acc_; }

        /// \brief 设置单关节加速度约束
        /// \param id 关节ID
        /// \param max_acc 加速度约束值
        inline void setJntAccLimit(int id, double max_acc) {
            max_acc_[id] = max_acc;
            need_plan_[id] = true;
        }

        /// \brief 获取单关节加速度约束
        /// \param id 关节ID
        /// \return 加速度约束值
        inline double getJntAccLimit(int id) { return max_acc_[id]; }

        /// \brief 设置多关节加加速约束
        /// \param max_jerk 多关节加加速约束值
        inline void setJntJerkLimits(std::vector<double> &max_jerk) {
            max_jerk_ = max_jerk;
            need_plan_.resize(jnt_num_, true);
        }

        /// \brief 获取多关节加加速约束
        /// \return 多关节加加速约束值
        inline std::vector<double> getJntJerkLimits() { return max_jerk_; }

        /// \brief 设置单关节加加速度约束
        /// \param id 关节id
        /// \param max_jerk 关节加加速约束值
        inline void setJntJerkLimit(int id, double max_jerk) {
            max_jerk_[id] = max_jerk;
            need_plan_[id] = true;
        }

        /// \brief 获取单关节加加速度约束
        /// \param id 关节ID
        /// \return 关节加加速约束值
        inline double getJntJerkLimit(int id) { return max_jerk_[id]; }

        /// \brief 设置规划Profile类型
        /// \param type Trapezoid、DoubleS
        inline void setProfileType(ProfileType type) {
            profile_type_ = type;
            need_plan_.resize(jnt_num_, true);
        }

        /// \brief 设置同步模式
        /// \param sync 无同步、时间同步、相位同步
        inline void setSynchronization(Synchronization sync) {
            sync_ = sync;
            need_plan_.resize(jnt_num_, true);
        }


        /*****轨迹规划线程相关*****/
        void startMotionThread(); // 启动轨迹规划线程
        void stopMotionThread(); // 停止轨迹规划线程
        void motionThreadHandler(); // 轨迹规划相关处理句柄

    protected:
        void addAllJoints();

    public:
        // TODO： 测试用MoveJ，阻塞运行，需要改为private
        void moveJ(const std::vector<double> &pos,
                   const std::vector<double> &max_vel,
                   const std::vector<double> &max_acc,
                   const std::vector<double> &max_jerk,
                   Synchronization sync = SYNC_TIME,
                   ProfileType type = ProfileType::trapezoid);

    protected:
        boost::shared_ptr<HardwareInterface> hw_interface_{nullptr};
        std::vector<boost::shared_ptr<Drive>> joints_;

        std::vector<double> target_positions_;     // 当前目标位置
        std::vector<double> target_positions_prev_; // 上一次的目标位置

        std::vector<double> target_velocities_;
        std::vector<double> target_torques_;

        std::vector<double> pos_;
        std::vector<double> vel_;
        std::vector<double> acc_;

        std::vector<double> max_vel_;
        std::vector<double> max_acc_;
        std::vector<double> max_jerk_;

        std::vector<R_INTERP_BASE *> interp_;

        ProfileType profile_type_{trapezoid};

        int jnt_num_;

        double least_motion_time_{0.0};

        Synchronization sync_{SYNC_TIME};

        bool is_running_{false};
        bool is_exit_{false};

        std::vector<bool> need_plan_; // 是否需要重新规划标志

        boost::shared_ptr<boost::thread> otg_motion_thread_ {nullptr}; //otg在线规划线程

        Kinematics kinematics;

    };

}


#endif //ROCOS_APP_ROBOT_H
