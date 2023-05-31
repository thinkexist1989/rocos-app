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

#include "drive.h"
#include "hardware_interface.h"
#include "interpolate.h"
#include "kinematics.h"
#include "JC_helper_kinematics.hpp"
#include "JC_helper_dynamics.hpp"
#include <Eigen/StdVector>  //!< Eigen官网说明 https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html
#include <boost/smart_ptr.hpp>
#include <vector>
#include "kdl_parser/kdl_parser.hpp" //!< 解析URDF文件
#include "gripper.hpp"
#include "JC_helper_TCP.hpp"

namespace rocos {
    //! Class Robot
    class Robot {
        friend class RobotServiceImpl;

    public:
        //! Class PathEntry is used by Class Path
        class PathEntry {
        public:
            enum MoveType {
                MOVE_J,
                MOVE_L,
                MOVE_P,
                MOVE_C
            };

        private:
            MoveType type_;
            JntArray q_;
            Frame pose_;
        };

        //! Class Path is used by MovePath
        class Path {
        private:
            std::vector<PathEntry> waypoints_;
        };

        enum Synchronization {
            SYNC_NONE,
            SYNC_TIME,
            SYNC_PHASE
        };

        enum OrientationMode {
            UNCONSTRAINED,
            FIXED
        };

        explicit Robot(boost::shared_ptr<HardwareInterface> hw,
                       const std::string &urdf_file_path = "robot.urdf",
                       const std::string &base_link = "base_link",
                       const std::string &tip = "link7");

        bool parseUrdf(const std::string &urdf_file_path,
                       const std::string &base_link,
                       const std::string &tip);

        bool parseDriveParamsFromUrdf(const std::string &urdf_file_path);

        bool switchHW(boost::shared_ptr<HardwareInterface> hw);  //切换硬件指针

        void setEnabled();

        inline void setJointEnabled(int id) { joints_[id]->setEnabled(); }

        void setDisabled();

        inline void setJointDisabled(int id) { joints_[id]->setDisabled(); }

        inline void setJointMode(int id, ModeOfOperation mode) {
            joints_[id]->setMode(mode);
        }

        inline int getJointNum() const { return jnt_num_; }

        inline std::string getJointName(int id) { return joints_[id]->getName(); }

        inline int getJointStatus(int id) { return joints_[id]->getDriveStateRPC(); }

        ///////////////////用户单位信息///////////////////////
        inline double getJointPosition(int id) { return joints_[id]->getPosition(); }

        inline double getJointVelocity(int id) { return joints_[id]->getVelocity(); }

        inline double getJointTorque(int id) { return joints_[id]->getTorque(); }

        inline double getJointLoadTorque(int id) {
            return joints_[id]->getLoadTorque();
        }

        inline void setJointPosition(int id, double pos) {
            joints_[id]->setPosition(pos);
        }

        inline void setJointVelocity(int id, double vel) {
            joints_[id]->setVelocity(vel);
        }

        inline void setJointTorque(int id, double tor) {
            joints_[id]->setTorque(tor);
        }

        /////////////////////获取原始信息//////////////////////////
        inline int32_t getJointPositionRaw(int id) {
            return joints_[id]->getPositionInCnt();
        }

        inline int32_t getJointVelocityRaw(int id) {
            return joints_[id]->getVelocityInCnt();
        }

        inline int16_t getJointTorqueRaw(int id) {
            return joints_[id]->getTorqueInCnt();
        }

        inline int16_t getJointLoadTorqueRaw(int id) {
            return joints_[id]->getLoadTorqueInCnt();
        }

        inline void setJointPositionRaw(int id, int32_t pos) {
            joints_[id]->setPositionInCnt(pos);
        }

        inline void setJointVelocityRaw(int id, int32_t vel) {
            joints_[id]->setVelocityInCnt(vel);
        }

        inline void setJointTorqueRaw(int id, int16_t tor) {
            joints_[id]->setTorqueInCnt(tor);
        }

        ////////////////////单位转换////////////////////////////
        inline double getJointCntPerUnit(int id) {
            return joints_[id]->getCntPerUnit();
        }

        inline double getJointTorquePerUnit(int id) {
            return joints_[id]->getTorquePerUnit();
        }

        inline double getJointRatio(int id) { return joints_[id]->getRatio(); }

        inline int32_t getJointPosZeroOffset(int id) {
            return joints_[id]->getPosZeroOffset();
        }

        inline std::string getJointUserUnitName(int id) {
            return joints_[id]->getUserUnitName();
        }

        inline void setJointCntPerUnit(int id, double cnt_per_unit) {
            joints_[id]->setCntPerUnit(cnt_per_unit);
        }

        inline void setJointTorquePerUnit(int id, double tor_per_unit) {
            joints_[id]->setTorquePerUnit(tor_per_unit);
        }

        inline void setJointRatio(int id, double ratio) {
            joints_[id]->setRatio(ratio);
        }

        inline void setJointPosZeroOffset(int id, int32_t offset) {
            joints_[id]->setPosZeroOffset(offset);
        }

        inline void setJointUserUnitName(int id, std::string name) {
            joints_[id]->setUserUnitName(name);
        }

        /// 多关节运动
        /// \param target_pos 位置
        /// \param target_vel 速度
        /// \param sync 同步模式
        void moveJ(const std::vector<double> &target_pos,
                   const std::vector<double> &target_vel,
                   Synchronization sync = SYNC_TIME);

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
        void moveSingleAxis(int id, double pos, double vel = 0.0, double max_vel = -1,
                            double max_acc = -1, double max_jerk = -1,
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

        inline Frame getFlange() { return flange_; }

        inline Frame getTool() { return tool_; }

        inline Frame getObject() { return object_; }

        /*****轨迹规划线程相关*****/
        void startMotionThread();    // 启动轨迹规划线程
        void stopMotionThread();     // 停止轨迹规划线程
        void motionThreadHandler();  // 轨迹规划相关处理句柄

    protected:
        void addAllJoints();

        int JntToCart(const JntArray &q_in, Frame &p_out) {
            return kinematics_.JntToCart(q_in, p_out);
        }

        int CartToJnt(const JntArray &q_init, const Frame &p_in, JntArray &q_out) {
            return kinematics_.CartToJnt(q_init, p_in, q_out);
        }

        //! 更新法兰系,工具系,工件系pose
        void updateCartesianInfo() {
            JntArray q_in(jnt_num_);
            for( int i{0};i<jnt_num_;i++)
            q_in(i) = pos_[i];
            //            std::cout << q_in.data << std::endl;

            // Flange Reference
            JntToCart(q_in, flange_);
            //            std::cout << "OK" << std::endl;
        }

    public:
        //! \brief 关节运动（支持位置和速度模式）
        //! \param q 各个关节位置
        //! \param speed 关节速度限制（leading axis）
        //! \param acceleration 关节加速度限制
        //! \param time 最短运行时间
        //! \param radius 过渡半径
        //! \param asynchronous 是否异步运行
        //! \return 错误标志位,成功返回0
        int MoveJ(JntArray q, double speed = 1.05, double acceleration = 1.4,
                  double time = 0.0, double radius = 0.0, bool asynchronous = false);

        //! \brief 关节运动到指定笛卡尔位姿
        //! \param pose 位姿
        //! \param speed 关节速度限制（leading axis）
        //! \param acceleration 关节加速度限制
        //! \param time 最短运行时间
        //! \param radius 过渡半径
        //! \param asynchronous 是否异步运行
        //! \return 错误标志位,成功返回0
        int MoveJ_IK(Frame pose, double speed = 1.05, double acceleration = 1.4,
                     double time = 0.0, double radius = 0.0,
                     bool asynchronous = false);

        //! \brief 直线运动到指定位姿（支持位置和速度模式）
        //! \param pose 位姿
        //! \param speed 笛卡尔速度限制（leading axis）
        //! \param acceleration 笛卡尔加速度限制
        //! \param time 最短运行时间
        //! \param radius 过渡半径
        //! \param asynchronous 是否异步运行
        //! \param max_running_count MoveL规划失败重新尝试规划的最大次数
        //! \return 错误标志位,成功返回0
        int MoveL(Frame pose, double speed = 1.05, double acceleration = 1.4,
                  double time = 0.0, double radius = 0.0, bool asynchronous = false,int max_running_count = 10);

        int MoveL_pos( Frame pose, double speed = 1.05, double acceleration = 1.4,
                       double time = 0.0, double radius = 0.0, bool asynchronous = false, int max_running_count = 10 );

        int MoveL_vel( Frame pose, double speed = 1.05, double acceleration = 1.4,
                       double time = 0.0, double radius = 0.0, bool asynchronous = false, int max_running_count = 10 );

        //! \brief 直线运动到关节空间指定位置
        //! \param q 关节位置
        //! \param speed 关节速度限制（leading axis）
        //! \param acceleration 关节加速度限制
        //! \param time 最短运行时间
        //! \param radius 过渡半径
        //! \param asynchronous 是否异步运行
        //! \return 错误标志位,成功返回0
        int MoveL_FK(JntArray q, double speed = 1.05, double acceleration = 1.4,
                     double time = 0.0, double radius = 0.0,
                     bool asynchronous = false);

        //! \brief 圆弧运动（支持位置和速度模式）
        //! \param pose_via 中间点,姿态给定无效，内部会自动计算
        //! \param pose_to 目标点,姿态给定无效，内部会自动计算
        //! \param speed 笛卡尔空间速度限制（leading axis）
        //! \param acceleration 笛卡尔空间加速度限制
        //! \param time 最短运行时间
        //! \param radius 过渡半径
        //! \param mode 姿态运行模式, UNCONSTRAINED姿态随动
        //! \param asynchronous 是否异步运行
        //! \param max_running_count MoveC规划失败重新尝试规划的最大次数
        //! \return 错误标志位,成功返回0

        int MoveC(Frame pose_via, Frame pose_to, double speed = 0.25,
                  double acceleration = 1.2, double time = 0.0, double radius = 0.0,
                  OrientationMode mode = UNCONSTRAINED, bool asynchronous = false,int max_running_count =10);

        int MoveC_pos(Frame pose_via, Frame pose_to, double speed = 0.25,
                  double acceleration = 1.2, double time = 0.0, double radius = 0.0,
                  OrientationMode mode = UNCONSTRAINED, bool asynchronous = false,int max_running_count =10);

        int MoveC_vel(Frame pose_via, Frame pose_to, double speed = 0.25,
                  double acceleration = 1.2, double time = 0.0, double radius = 0.0,
                  OrientationMode mode = UNCONSTRAINED, bool asynchronous = false,int max_running_count =10);

        //! \brief 圆弧运动（支持位置和速度模式）
        //! \param center 圆弧圆心位姿
        //! \param theta 圆弧旋转角度
        //! \param axiz   绕圆心位姿的哪个轴转（0-X、1-Y、2-Z）
        //! \param speed 笛卡尔速度限制（leading axis）
        //! \param acceleration 笛卡尔加速度限制
        //! \param time 最短运行时间
        //! \param radius 过渡半径
        //! \param mode 姿态运行模式, UNCONSTRAINED姿态随动
        //! \param asynchronous 是否异步运行
        //! \param max_running_count MoveC规划失败重新尝试规划的最大次数
        //! \return 错误标志位,成功返回0

        int MoveC( const KDL::Frame& center, double theta, int axiz = 2, double speed = 0.25,
                   double acceleration = 1.2, double time = 0.0, double radius = 0.0,
                   OrientationMode mode = UNCONSTRAINED, bool asynchronous = false, int max_running_count = 10 );

        int MoveC_pos( const KDL::Frame& center, double theta, int axiz = 2, double speed = 0.25,
                       double acceleration = 1.2, double time = 0.0, double radius = 0.0,
                       OrientationMode mode = UNCONSTRAINED, bool asynchronous = false, int max_running_count = 10 );

        int MoveC_vel( const KDL::Frame& center, double theta, int axiz = 2, double speed = 0.25,
                       double acceleration = 1.2, double time = 0.0, double radius = 0.0,
                       OrientationMode mode = UNCONSTRAINED, bool asynchronous = false, int max_running_count = 10 );

        //! \brief TODO: 什么是MoveP?
        //! \param pose 位姿
        //! \param speed 关节速度限制（leading axis）
        //! \param acceleration 关节加速度限制
        //! \param time 最短运行时间
        //! \param radius 过渡半径
        //! \param mode 姿态运行模式, UNCONSTRAINED姿态随动
        //! \param asynchronous 是否异步运行
        //! \return 错误标志位,成功返回0
        int MoveP( Frame pose, double speed = 1.05, double acceleration = 1.4,
                   double time = 0.0, double radius = 0.0, bool asynchronous = false );

        //! \brief 沿指定路径运动
        //! \param path 路径
        //! \param asynchronous
        //! \return 错误标志位,成功返回0
        int MovePath(const Path &path, bool asynchronous = false);


        /**
         * @brief 多段直线的连续运动
         * @note 1、过渡半径bound_dist可以为0，表示终止速度为0；2、point[0]代表第一个目标点 
         * @param point 目标点集
         * @param bound_dist 过渡半径
         * @param max_path_v 最大速度
         * @param max_path_a 最大加速度
         * @param asynchronous 是否异步运行
         * @param max_running_count 规划失败重新尝试规划的最大次数
         * 
         * @example 
            Frame f_p1;
            Frame f_p2;
            Frame f_p3;
            Frame f_p4;

            f_p1 = robot.getFlange() * Frame{ KDL::Rotation::RotX( 90 * M_PI / 180 ), Vector{ 0.3, 0.0, 0 } };
            f_p2 = f_p1 * Frame{ KDL::Rotation::RotY( 90 * M_PI / 180 ), Vector{ 0.0, -0.3, -0.0 } };
            f_p3 = f_p2 * Frame{ KDL::Rotation::RotX( -90 * M_PI / 180 ), Vector{ 0.0, 0.0, -0.3 } };
            f_p4 = f_p3 * Frame{ KDL::Rotation::RotZ( -90 * M_PI / 180 ), Vector{ 0.0, 0.0, 0.3 } };

            std::vector< KDL::Frame > points{ f_p1,f_p2, f_p3, f_p4 };
            std::vector< double > max_path_v{ 0.06, 0.12, 0.12, 0.24};
            std::vector< double > max_path_a{ 0.06, 0.06, 0.06, 0.06};
            std::vector< double > bound_dist{0.05,0.1,0.0,0.2};
            MultiMoveL( points, bound_dist, max_path_v, max_path_a, false );
         * @return int 失败为-1，成功为0
         */
        int
        MultiMoveL(const std::vector<KDL::Frame> &point, std::vector<double> bound_dist, std::vector<double> max_path_v,
                   std::vector<double> max_path_a, bool asynchronous = false,int max_running_count = 10);


        enum class DRAGGING_FLAG :int {
            J0 = 0, J1 = 1, J2 = 2, J3 = 3, J4 = 4, J5 = 5, J6 = 6,
            TOOL_X = 100, TOOL_Y = 101, TOOL_Z = 102, TOOL_ROLL = 103, TOOL_PITCH= 104, TOOL_YAW = 105,
            FLANGE_X = 200, FLANGE_Y = 201, FLANGE_Z = 202,FLANGE_ROLL = 203, FLANGE_PITCH = 204, FLANGE_YAW = 205,
            OBJECT_X = 300, OBJECT_Y = 301, OBJECT_Z = 302,OBJECT_ROLL= 303, OBJECT_PITCH = 304, OBJECT_YAW = 305,
            BASE_X = 400, BASE_Y = 401, BASE_Z = 402, BASE_ROLL= 403, BASE_PITCH = 404, BASE_YAW = 405,
            NULLSPACE = 500
        };

        enum class DRAGGING_DIRRECTION :int{
            NONE = 0, POSITION = 1, NEGATIVE = -1
        };

        enum class DRAGGING_TYPE : int
        {
            JOINT     = 0,
            CARTESIAN = 1,
            NULLSPACE = 2
        };

        /**
         * @brief 拖动示教功能
         * @note 第一次调用会启动ruckig线程，线程运行过程中需要至少100ms以内再次调用一次该函数（保持心跳）
         * @note 否则会触发紧急停止（进入速度无同步模式，速度快速降至0）
         * @param flag 命令标志
         * @param dir 正方向或反方向
         * @param max_speed 最大运行速度,其10%的值+当前位置等于目标位置
         * @param max_acceleration 最大关节速度
         * @return int 
         * @example 
         *   for ( int i = 0; i < 100; i++ )
         *  {
         *   robot.Dragging( Robot::DraggingFlag::J1, Robot::DraggingDirection::POSITION, 1, 1);
         *   std::this_thread::sleep_for( std::chrono::milliseconds( 50 ) );
         *   }
         * 
         */
        int Dragging(DRAGGING_FLAG flag, DRAGGING_DIRRECTION dir, double max_speed, double max_acceleration);

    private:
        //运动前检查数据有效性
        int CheckBeforeMove(const JntArray &q, double speed,
                            double acceleration, double time, double radius);

        int CheckBeforeMove(const Frame &pos, double speed,
                            double acceleration, double time, double radius);

        //实际movej执行线程
        void RunMoveJ(JntArray q, double speed = 1.05, double acceleration = 1.4,
                      double time = 0.0, double radius = 0.0);

        //实际movel执行线程
        void RunMoveL(const std::vector<KDL::JntArray> &traj);

        //实际multimovel执行线程
        void RunMultiMoveL(const std::vector<KDL::JntArray> &traj);
        int admittance_teaching( );
        int stop_admittance_teaching( );
        int admittance_link( KDL::Frame frame_target, double speed, double acceleration );

        /**
         * @brief 1000hz关节伺服接口
         *
         * @param target_pos 目标位置
         * @return int
         */
        int servoJ( const KDL::JntArray& target_pos );
        /**
         * @brief 1000hz位姿伺服接口
         *
         * @param target_frame 目标位置
         * @return int
         */
        int servoL( const KDL::Frame& target_frame );

    public:
        void test( );  // 为了测试

    private:
        // TODO： 测试用MoveJ，阻塞运行，需要改为private
        void
        moveJ(const std::vector<double> &pos,
              const std::vector<double> &max_vel,
              const std::vector<double> &max_acc,
              const std::vector<double> &max_jerk,
              Synchronization sync = SYNC_TIME,
              ProfileType type = ProfileType::trapezoid);

    protected:
        boost::shared_ptr<HardwareInterface> hw_interface_{nullptr};
        std::vector<boost::shared_ptr<Drive> > joints_;

        std::vector<double> target_positions_;       // 当前目标位置
        std::vector<double> target_positions_prev_;  // 上一次的目标位置

        std::vector<double> target_velocities_;
        std::vector<double> target_torques_;

        std::vector< std::atomic<double>> pos_;
        std::vector< std::atomic<double>> vel_;
        std::vector< std::atomic<double>> acc_;

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
        std::atomic<bool> is_running_motion{false};

        std::vector<bool> need_plan_;  // 是否需要重新规划标志

        boost::shared_ptr<boost::thread> otg_motion_thread_{
                nullptr};                                                 // otg在线规划线程
        boost::shared_ptr<boost::thread> motion_thread_{nullptr};  // 执行motion线程

        Kinematics kinematics_;
        JC_helper::inverse_special_to_SRS SRS_kinematics_;


        Frame flange_;  //!< 法兰位置姿态
        Frame tool_;    //!< 工具位置姿态
        Frame object_;  //!< 工件位置姿态

        std::vector<KDL::JntArray> traj_;
        std::atomic<int> tick_count{0};

    public:
        friend void JC_helper::SmartServo_Joint::RunSmartServo(rocos::Robot *);

        friend class JC_helper::SmartServo_Cartesian ;

        friend class JC_helper::SmartServo_Nullsapace ;

        friend void JC_helper::Joint_stop( rocos::Robot* robot_ptr, const KDL::JntArray& current_pos, const KDL::JntArray& last_pos, const KDL::JntArray& last_last_pos );

        friend class JC_helper::admittance ;

        friend int JC_helper::safety_servo( rocos::Robot* robot_ptr, const std::array< double, _joint_num >& target_pos );

        friend int JC_helper::safety_servo( rocos::Robot* robot_ptr, const std::vector< double >& target_pos );

        friend int JC_helper::safety_servo( rocos::Robot* robot_ptr, const KDL::JntArray& target_pos );

    private:
        JC_helper::ft_sensor my_ft_sensor;  // 6维力传感器
        bool flag_admittance_turnoff {false}; //导纳开关
    };



}  // namespace rocos

#endif  // ROCOS_APP_ROBOT_H
