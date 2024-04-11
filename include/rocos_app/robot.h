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
#include <Eigen/StdVector> //!< Eigen官网说明 https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html
#include <boost/smart_ptr.hpp>
#include <vector>
#include "kdl_parser/kdl_parser.hpp" //!< 解析URDF文件
#include "gripper.hpp"

namespace rocos
{
    //! Class Robot
    class Robot
    {
        friend class RobotServiceImpl;

        using JntArray = KDL::JntArray;
        using Frame = KDL::Frame;

    public:
        //! Class PathEntry is used by Class Path
        class PathEntry
        {
        public:
            enum MoveType
            {
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
        class Path
        {
        private:
            std::vector<PathEntry> waypoints_;
        };

        enum Synchronization
        {
            SYNC_NONE,
            SYNC_TIME,
            SYNC_PHASE
        };

        enum OrientationMode
        {
            UNCONSTRAINED,
            FIXED
        };

        enum class WorkMode
        {
            Position = 0,
            EeAdmitTeach = 1,
            JntAdmitTeach = 2,
            JntImp = 3,
            CartImp = 4
        };

        enum class RunState
        {
            Disabled = 0,
            Stopped = 1,
            Running = 2
        };

        explicit Robot(HardwareInterface *hw,
                       const string &urdf_file_path = "robot.urdf",
                       const string &base_link = "base_link",
                       const string &tip = "link7"); // std::string yaml_path = "joint_impedance_control.yaml"

        ~Robot();

        bool parseUrdf(const std::string &urdf_file_path,
                       const std::string &base_link,
                       const std::string &tip);

        bool parseDriveParamsFromUrdf(const std::string &urdf_file_path);

        bool switchHW(HardwareInterface *hw); // 切换硬件指针

        // 机器人状态机相关
        inline WorkMode getWorkMode() { return work_mode_; }
        bool setWorkMode(WorkMode mode);
        inline RunState getRunState() { return run_state_; }
        bool setRunState(RunState state);

        void setEnabled();

        inline void setJointEnabled(int id) { joints_[id]->setEnabled(); }

        void setDisabled();

        inline void setJointDisabled(int id) { joints_[id]->setDisabled(); }

        inline void setJointMode(int id, ModeOfOperation mode)
        {
            joints_[id]->setMode(mode);
        }

        inline int getJointNum() const { return jnt_num_; }

        inline std::string getJointName(int id) { return joints_[id]->getName(); }

        inline int getJointStatus(int id) { return joints_[id]->getDriveStateRPC(); }

        ///////////////////用户单位信息///////////////////////
        inline double getJointPosition(int id) { return joints_[id]->getPosition(); }

        inline double getJointVelocity(int id) { return joints_[id]->getVelocity(); }

        inline double getJointTorque(int id) { return joints_[id]->getTorque(); }

        inline double getJointLoadTorque(int id)
        {
            return joints_[id]->getLoadTorque();
        }
        // 获取滤波后的数据
        inline double getJointTorqueFilter(int id) { return joints_[id]->getSecondaryPositionInCnt(); }
        inline double getJointSecondaryPositionInCnt(int id) { return joints_[id]->getSecondaryPositionInCnt(); }

        inline void setJointPosition(int id, double pos)
        {
            joints_[id]->setPosition(pos);
        }

        inline void setJointVelocity(int id, double vel)
        {
            joints_[id]->setVelocity(vel);
        }

        inline void setJointTorque(int id, double tor)
        {
            joints_[id]->setTorque(tor);
        }

        /////////////////////获取原始信息//////////////////////////
        inline int32_t getJointPositionRaw(int id)
        {
            return joints_[id]->getPositionInCnt();
        }

        inline int32_t getJointVelocityRaw(int id)
        {
            return joints_[id]->getVelocityInCnt();
        }

        inline int16_t getJointTorqueRaw(int id)
        {
            return joints_[id]->getTorqueInCnt();
        }

        inline int16_t getJointLoadTorqueRaw(int id)
        {
            return joints_[id]->getLoadTorqueInCnt();
        }

        inline void setJointPositionRaw(int id, int32_t pos)
        {
            joints_[id]->setPositionInCnt(pos);
        }

        inline void setJointVelocityRaw(int id, int32_t vel)
        {
            joints_[id]->setVelocityInCnt(vel);
        }

        inline void setJointTorqueRaw(int id, int16_t tor)
        {
            joints_[id]->setTorqueInCnt(tor);
        }

        ////////////////////单位转换////////////////////////////
        inline double getJointCntPerUnit(int id)
        {
            return joints_[id]->getCntPerUnit();
        }

        inline double getJointTorquePerUnit(int id)
        {
            return joints_[id]->getTorquePerUnit();
        }

        inline double getJointRatio(int id) { return joints_[id]->getRatio(); }

        inline int32_t getJointPosZeroOffset(int id)
        {
            return joints_[id]->getPosZeroOffset();
        }

        inline std::string getJointUserUnitName(int id)
        {
            return joints_[id]->getUserUnitName();
        }

        inline void setJointCntPerUnit(int id, double cnt_per_unit)
        {
            joints_[id]->setCntPerUnit(cnt_per_unit);
        }

        inline void setJointTorquePerUnit(int id, double tor_per_unit)
        {
            joints_[id]->setTorquePerUnit(tor_per_unit);
        }

        inline void setJointRatio(int id, double ratio)
        {
            joints_[id]->setRatio(ratio);
        }

        inline void setJointPosZeroOffset(int id, int32_t offset)
        {
            joints_[id]->setPosZeroOffset(offset);
        }

        inline void setJointUserUnitName(int id, std::string name)
        {
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
        inline void setJntVelLimits(std::vector<double> &max_vel)
        {
            max_vel_ = max_vel;
            need_plan_.resize(jnt_num_, true);
        }

        /// \brief 获取多关节速度约束
        /// \return 速度约束值
        inline std::vector<double> getJntVelLimits() { return max_vel_; };

        /// \brief 设置单关节速度约束
        /// \param id 关节ID
        /// \param max_vel 速度约束值
        inline void setJntVelLimit(int id, double max_vel)
        {
            max_vel_[id] = max_vel;
            need_plan_[id] = true;
        }

        /// \brief 获取单关节速度约束
        /// \param id 关节ID
        /// \return 速度约束值
        inline double getJntVelLimit(int id) { return max_vel_[id]; }

        /// \brief 设置关节加速度约束
        /// \param max_acc 加速度约束值
        inline void setJntAccLimits(std::vector<double> &max_acc)
        {
            max_acc_ = max_acc;
            need_plan_.resize(jnt_num_, true);
        }

        /// \brief 获取多关节加速度约束
        /// \return 加速度约束值
        inline std::vector<double> getJntAccLimits() { return max_acc_; }

        /// \brief 设置单关节加速度约束
        /// \param id 关节ID
        /// \param max_acc 加速度约束值
        inline void setJntAccLimit(int id, double max_acc)
        {
            max_acc_[id] = max_acc;
            need_plan_[id] = true;
        }

        /// \brief 获取单关节加速度约束
        /// \param id 关节ID
        /// \return 加速度约束值
        inline double getJntAccLimit(int id) { return max_acc_[id]; }

        /// \brief 设置多关节加加速约束
        /// \param max_jerk 多关节加加速约束值
        inline void setJntJerkLimits(std::vector<double> &max_jerk)
        {
            max_jerk_ = max_jerk;
            need_plan_.resize(jnt_num_, true);
        }

        /// \brief 获取多关节加加速约束
        /// \return 多关节加加速约束值
        inline std::vector<double> getJntJerkLimits() { return max_jerk_; }

        /// \brief 设置单关节加加速度约束
        /// \param id 关节id
        /// \param max_jerk 关节加加速约束值
        inline void setJntJerkLimit(int id, double max_jerk)
        {
            max_jerk_[id] = max_jerk;
            need_plan_[id] = true;
        }

        /// \brief 获取单关节加加速度约束
        /// \param id 关节ID
        /// \return 关节加加速约束值
        inline double getJntJerkLimit(int id) { return max_jerk_[id]; }

        /// \brief 设置规划Profile类型
        /// \param type Trapezoid、DoubleS
        inline void setProfileType(ProfileType type)
        {
            profile_type_ = type;
            need_plan_.resize(jnt_num_, true);
        }

        /// \brief 设置同步模式
        /// \param sync 无同步、时间同步、相位同步
        inline void setSynchronization(Synchronization sync)
        {
            sync_ = sync;
            need_plan_.resize(jnt_num_, true);
        }

        inline Frame getFlange()
        {
            std::lock_guard<std::mutex> lock(mtx); // 自动获取互斥锁
            return flange_;
        }
        // sun
        Frame getTool()
        {
            // tool_=flange_*T_tool_;
            std::lock_guard<std::mutex> lock(mtx); // 自动获取互斥锁
            tool_ = flange_ * T_tool_;
            return tool_;
        }

        // sun
        Frame getObject()
        {
            std::lock_guard<std::mutex> lock(mtx); // 自动获取互斥锁
            // Object Reference
            object_ = T_object_;
            return object_;
        }

        Frame getT_tool_()
        {
            return T_tool_;
        }
        Frame getT_object_()
        {
            return T_object_;
        }

        /*****轨迹规划线程相关*****/
        void startMotionThread();   // 启动轨迹规划线程
        void stopMotionThread();    // 停止轨迹规划线程
        void motionThreadHandler(); // 轨迹规划相关处理句柄
        // sun
    public:
        void tool_calibration(std::string frame) // 工具标定
        {

            ErrorState = false;
            Eigen::MatrixXd R_EB(9, 3);
            Eigen::MatrixXd P_TB(9, 1);

            if (frame == "tool")
            {

                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        R_EB(i, j) = pose1.M.data[i * 3 + j] - pose2.M.data[i * 3 + j];
                        R_EB(i + 3, j) = pose2.M.data[i * 3 + j] - pose3.M.data[i * 3 + j];
                        R_EB(i + 6, j) = pose3.M.data[i * 3 + j] - pose4.M.data[i * 3 + j];
                    }
                    P_TB(i, 0) = pose2.p.data[i] - pose1.p.data[i];
                    P_TB(i + 3, 0) = pose3.p.data[i] - pose2.p.data[i];
                    P_TB(i + 6, 0) = pose4.p.data[i] - pose3.p.data[i];
                }
                Eigen::Vector3d Pos = (R_EB.transpose() * R_EB).inverse() * R_EB.transpose() * P_TB;
                // std::cout << "Pos" << Pos << std::endl;
                //  测试pinv

                // std::cout << "pinv_R_TB" << R_EB.completeOrthogonalDecomposition().pseudoInverse() << std::endl;
                Eigen::MatrixXd pinv_R_TB = R_EB.completeOrthogonalDecomposition().pseudoInverse();
                Eigen::Vector3d Pos1 = pinv_R_TB * P_TB;
                // std::cout << "Pos1" << Pos1 << std::endl;
                // Calibration of rotation
                // Calibration of rotation
                Eigen::Vector3d Vx{(pose5.p - pose4.p).data[0], (pose5.p - pose4.p).data[1], (pose5.p - pose4.p).data[2]};
                Eigen::Vector3d Vy{(pose6.p - pose4.p).data[0], (pose6.p - pose4.p).data[1], (pose6.p - pose4.p).data[2]};
                // std::cout<<"Vx"<<Vx<<std::endl;

                Vx.normalize();
                Vy.normalize();

                Eigen::Vector3d Vz = Vx.cross(Vy);
                Vy = Vz.cross(Vx);
                Eigen::Matrix3d R_TB;
                R_TB.block<3, 1>(0, 0) = Vx;
                R_TB.block<3, 1>(0, 1) = Vy;
                R_TB.block<3, 1>(0, 2) = Vz;
                // std::cout << "Vx" << Vx << std::endl;
                // std::cout << "Vy" << Vy << std::endl;
                // std::cout << "Vz" << Vz << std::endl;

                Eigen::Matrix3d R0_EB;
                KDL::Rotation pose4_Rotation = pose4.M;
                // std::cout << "pose4.m" << pose4.M.data[0] << "," << pose4.M.data[1] << "," << pose4.M.data[2] << "," << pose4.M.data[3] << "," << pose4.M.data[4] << "," << pose4.M.data[5] << "," << pose4.M.data[6] << "," << pose4.M.data[7] << "," << pose4.M.data[8] << std::endl;
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        R0_EB(i, j) = pose4_Rotation(i, j);
                    }
                }
                // 打印R0_EB和R_TB
                // std::cout << "R0_EB" << R0_EB << std::endl;
                // std::cout << "R_TB" << R_TB << std::endl;

                Eigen::Matrix3d Rot = R0_EB.inverse() * R_TB;
                // std::cout << "R0_EB.inverse()" << R0_EB.inverse() << std::endl;

                pose_out.p = KDL::Vector(Pos(0), Pos(1), Pos(2));
                pose_out.M = KDL::Rotation(Rot(0, 0), Rot(0, 1), Rot(0, 2), Rot(1, 0), Rot(1, 1), Rot(1, 2), Rot(2, 0), Rot(2, 1), Rot(2, 2));
                // std::cout << "pose_out" << pose_out.M.data[0] << "," << pose_out.M.data[1] << "," << pose_out.M.data[2] << "," << pose_out.M.data[3] << "," << pose_out.M.data[4] << "," << pose_out.M.data[5] << "," << pose_out.M.data[6] << "," << pose_out.M.data[7] << "," << pose_out.M.data[8] << std::endl;
                // 旋转矩阵转旋转向量
                Eigen::AngleAxisd rotation_vector2;
                rotation_vector2.fromRotationMatrix(Rot);
                double angle = rotation_vector2.angle();
                Eigen::Vector3d axis = rotation_vector2.axis();

                // std::cout << "axis" << axis[0] << "," << axis[1] << "," << axis[2] << std::endl;
                // std::cout << "angle" << angle << std::endl;
                Eigen::Vector3d rotVector = axis * angle;
                // 旋转矩阵转RPY
                double roll1, pitch1, yaw1;
                pose_out.M.GetRPY(roll1, pitch1, yaw1);

                // // 打印结果
                // std::cout << "工具系的位置" << pose_out.p.x() << "," << pose_out.p.y() << "," << pose_out.p.z() << std::endl;
                // std::cout << "工具系的旋转向量" << rotVector.x() / M_PI * 180 << "," << rotVector.y() / M_PI * 180 << "," << rotVector.z() / M_PI * 180 << std::endl;
                // std::cout << "工具系的RPY" << roll1 / M_PI * 180 << "," << pitch1 / M_PI * 180 << "," << yaw1 / M_PI * 180 << std::endl;
                // 工具系位置标定的误差
                double error = 0.0;
                error = (R_EB * Pos - P_TB).norm();
                std::cout << "工具系位置标定的误差" << error << std::endl;
                if (std::isnan(error) || error > 0.1)
                {
                    ErrorState = true;
                    std::cout << "工具系标定失败" << std::endl;
                }
            }
            else if (frame == "object")
            {
                Eigen::Vector3d Vx{(poseObject2.p - poseObject1.p).data[0], (poseObject2.p - poseObject1.p).data[1], (poseObject2.p - poseObject1.p).data[2]};
                Eigen::Vector3d Vy{(poseObject3.p - poseObject1.p).data[0], (poseObject3.p - poseObject1.p).data[1], (poseObject3.p - poseObject1.p).data[2]};
                Vx.normalize();
                Vy.normalize();
                Eigen::Vector3d Vz = Vx.cross(Vy);
                Vy = Vz.cross(Vx);
                Eigen::Matrix3d R_TB;
                R_TB.block<3, 1>(0, 0) = Vx;
                R_TB.block<3, 1>(0, 1) = Vy;
                R_TB.block<3, 1>(0, 2) = Vz;
                Eigen::Matrix3d R0_EB;
                KDL::Rotation pose4_Rotation = poseObject1.M;
                // std::cout << "pose4.m" << pose4.M.data[0] << "," << pose4.M.data[1] << "," << pose4.M.data[2] << "," << pose4.M.data[3] << "," << pose4.M.data[4] << "," << pose4.M.data[5] << "," << pose4.M.data[6] << "," << pose4.M.data[7] << "," << pose4.M.data[8] << std::endl;
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        R0_EB(i, j) = pose4_Rotation(i, j);
                    }
                }
                // 打印R0_EB和R_TB
                // std::cout << "R0_EB" << R0_EB << std::endl;
                // std::cout << "R_TB" << R_TB << std::endl;

                Eigen::Matrix3d Rot = R0_EB.inverse() * R_TB;
                 pose_out.p = KDL::Vector(poseObject1.p.x(), poseObject1.p.y(), poseObject1.p.z());
                pose_out.M = KDL::Rotation(Rot(0, 0), Rot(0, 1), Rot(0, 2), Rot(1, 0), Rot(1, 1), Rot(1, 2), Rot(2, 0), Rot(2, 1), Rot(2, 2));
                pose_out.M=poseObject1.M*pose_out.M;
                double roll1, pitch1, yaw1;
                pose_out.M.GetRPY(roll1, pitch1, yaw1);

                // 打印结果
                // std::cout << "工件系的位置" << pose_out.p.x() << "," << pose_out.p.y() << "," << pose_out.p.z() << std::endl;
                // std::cout << "工件系的RPY" << roll1 / M_PI * 180 << "," << pitch1 / M_PI * 180 << "," << yaw1 / M_PI * 180 << std::endl;
                 if (std::isnan(roll1) || std::isnan(pitch1) || std::isnan(yaw1)|| std::isnan(pose_out.p.x()) || std::isnan(pose_out.p.y()) || std::isnan(pose_out.p.z()))
                {
                    ErrorState = true;
                    std::cout << "工件系标定失败" << std::endl;
                }
                else
                {
                    std::cout << "工件系标定成功" << std::endl;
                }

            }
            else
            {
                std::cout << "frame error,无效坐标系" << std::endl;
            }
        }
        // 设置工具系
        void set_tool_frame(KDL::Frame &pose)
        {
            // 确定则设置T_tool_
            T_tool_ = pose;
            // 把T_tool_转换为RPY存放到yaml文件中
            std::vector<double> T_tool_rpy(6); // 初始化一个大小为6的数组，用于存储位置和RPY

            // 将位置信息存入数组
            T_tool_rpy[0] = T_tool_.p.x();
            T_tool_rpy[1] = T_tool_.p.y();
            T_tool_rpy[2] = T_tool_.p.z();

            // 计算并存储RPY
            double roll, pitch, yaw;
            T_tool_.M.GetRPY(roll, pitch, yaw);
            T_tool_rpy[3] = roll;
            T_tool_rpy[4] = pitch;
            T_tool_rpy[5] = yaw;

            yaml_node["T_tool_"] = T_tool_rpy;
            std::ofstream fout(yaml_path);
            std::ofstream fout1("/etc/rocos-yaml/calibration.yaml");

            fout << yaml_node;
            fout1 << yaml_node;
            fout.close();
            fout1.close();
            std::cout << "保存T_tool_成功" << std::endl;
        }
        void set_object_frame(KDL::Frame &pose)
        {
            T_object_ = pose;
            std::vector<double> T_object_rpy;
            T_object_rpy.push_back(T_object_.p.x());
            T_object_rpy.push_back(T_object_.p.y());
            T_object_rpy.push_back(T_object_.p.z());
            double roll, pitch, yaw;
            T_object_.M.GetRPY(roll, pitch, yaw);
            T_object_rpy.push_back(roll);
            T_object_rpy.push_back(pitch);
            T_object_rpy.push_back(yaw);
            yaml_node["T_object_"] = T_object_rpy;

            std::ofstream fout(yaml_path);
            std::ofstream fout1("etc/rocos-yaml/calibration.yaml");
            
            fout << yaml_node;
            fout1 << yaml_node;
            fout.close();
            fout1.close();
            std::cout << "保存T_object_成功" << std::endl;
        }

        void set_pose_frame(int id, KDL::Frame &pose_frame)
        {
            // 根据id选择要赋值的pose变量
            if (id == 1)
            {
                pose1 = pose_frame;
                std::cout << "pose1: " << pose1.p.x() << "," << pose1.p.y() << "," << pose1.p.z() << std::endl;
            }
            else if (id == 2)
            {
                pose2 = pose_frame;
                std::cout << "pose2: " << pose2.p.x() << "," << pose2.p.y() << "," << pose2.p.z() << std::endl;
            }
            else if (id == 3)
            {
                pose3 = pose_frame;
                std::cout << "pose3: " << pose3.p.x() << "," << pose3.p.y() << "," << pose3.p.z() << std::endl;
            }
            else if (id == 4)
            {
                pose4 = pose_frame;
                std::cout << "pose4: " << pose4.p.x() << "," << pose4.p.y() << "," << pose4.p.z() << std::endl;
            }
            else if (id == 5)
            {
                pose5 = pose_frame;
                std::cout << "pose5: " << pose5.p.x() << "," << pose5.p.y() << "," << pose5.p.z() << std::endl;
            }
            else if (id == 6)
            {
                pose6 = pose_frame;
                std::cout << "pose6: " << pose6.p.x() << "," << pose6.p.y() << "," << pose6.p.z() << std::endl;
            }
            else if (id == 7)
            {
                poseObject1 = pose_frame;
                std::cout << "poseObject1: " << poseObject1.p.x() << "," << poseObject1.p.y() << "," << poseObject1.p.z() << std::endl;
            }
            else if (id == 8)
            {
                poseObject2 = pose_frame;
                std::cout << "poseObject2: " << poseObject2.p.x() << "," << poseObject2.p.y() << "," << poseObject2.p.z() << std::endl;
            }

            else if (id == 9)
            {
                poseObject3 = pose_frame;
                std::cout << "poseObject3: " << poseObject3.p.x() << "," << poseObject3.p.y() << "," << poseObject3.p.z() << std::endl;
            }

            else
            {
                // 处理无效的id
                std::cerr << "Invalid id" << std::endl;
            }
        }
        Frame get_pose_frame(int id)
        {
            if (id == 1)
            {
                return pose1;
            }
            else if (id == 2)
            {
                return pose2;
            }
            else if (id == 3)
            {
                return pose3;
            }
            else if (id == 4)
            {
                return pose4;
            }
            else if (id == 5)
            {
                return pose5;
            }
            else if (id == 6)
            {
                return pose6;
            }
            else if (id == 7)
            {
                return poseObject1;
            }
            else if (id == 8)
            {
                return poseObject2;
            }
            else if (id == 9)
            {
                return poseObject3;
            }
            else
            {
                // 处理无效的id
                std::cerr << "Invalid id" << std::endl;
            }
        }
        Frame getPose_out()
        {
            return pose_out;
        }
        bool getErrorStateOfCal()
        {
            return ErrorState;
        }

    protected:
        void addAllJoints();

        int JntToCart(const JntArray &q_in, Frame &p_out)
        {
            return kinematics_.JntToCart(q_in, p_out);
        }

        int CartToJnt(const JntArray &q_init, const Frame &p_in, JntArray &q_out)
        {
            return kinematics_.CartToJnt(q_init, p_in, q_out);
        }

        //! 更新法兰系,工具系,工件系poseFlange
        void updateCartesianInfo()
        {
            JntArray q_in(jnt_num_);
            for (int i{0}; i < jnt_num_; i++)
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
                  double time = 0.0, double radius = 0.0, bool asynchronous = false, int max_running_count = 1);

        int MoveL_pos(Frame pose, double speed = 1.05, double acceleration = 1.4,
                      double time = 0.0, double radius = 0.0, bool asynchronous = false, int max_running_count = 1);

        int MoveL_vel(Frame pose, double speed = 1.05, double acceleration = 1.4,
                      double time = 0.0, double radius = 0.0, bool asynchronous = false, int max_running_count = 1);

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
                  OrientationMode mode = UNCONSTRAINED, bool asynchronous = false, int max_running_count = 1);

        int MoveC_pos(Frame pose_via, Frame pose_to, double speed = 0.25,
                      double acceleration = 1.2, double time = 0.0, double radius = 0.0,
                      OrientationMode mode = UNCONSTRAINED, bool asynchronous = false, int max_running_count = 1);

        int MoveC_vel(Frame pose_via, Frame pose_to, double speed = 0.25,
                      double acceleration = 1.2, double time = 0.0, double radius = 0.0,
                      OrientationMode mode = UNCONSTRAINED, bool asynchronous = false, int max_running_count = 1);

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

        int MoveC(const KDL::Frame &center, double theta, int axiz = 2, double speed = 0.25,
                  double acceleration = 1.2, double time = 0.0, double radius = 0.0,
                  OrientationMode mode = UNCONSTRAINED, bool asynchronous = false, int max_running_count = 1);

        int MoveC_pos(const KDL::Frame &center, double theta, int axiz = 2, double speed = 0.25,
                      double acceleration = 1.2, double time = 0.0, double radius = 0.0,
                      OrientationMode mode = UNCONSTRAINED, bool asynchronous = false, int max_running_count = 1);

        int MoveC_vel(const KDL::Frame &center, double theta, int axiz = 2, double speed = 0.25,
                      double acceleration = 1.2, double time = 0.0, double radius = 0.0,
                      OrientationMode mode = UNCONSTRAINED, bool asynchronous = false, int max_running_count = 1);

        //! \brief TODO: 什么是MoveP?
        //! \param pose 位姿
        //! \param speed 关节速度限制（leading axis）
        //! \param acceleration 关节加速度限制
        //! \param time 最短运行时间
        //! \param radius 过渡半径
        //! \param mode 姿态运行模式, UNCONSTRAINED姿态随动
        //! \param asynchronous 是否异步运行
        //! \return 错误标志位,成功返回0
        int MoveP(Frame pose, double speed = 1.05, double acceleration = 1.4,
                  double time = 0.0, double radius = 0.0, bool asynchronous = false);

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
                   std::vector<double> max_path_a, bool asynchronous = false, int max_running_count = 1);

        enum class DRAGGING_FLAG : int
        {
            J0 = 0,
            J1 = 1,
            J2 = 2,
            J3 = 3,
            J4 = 4,
            J5 = 5,
            J6 = 6,
            TOOL_X = 100,
            TOOL_Y = 101,
            TOOL_Z = 102,
            TOOL_ROLL = 103,
            TOOL_PITCH = 104,
            TOOL_YAW = 105,
            FLANGE_X = 200,
            FLANGE_Y = 201,
            FLANGE_Z = 202,
            FLANGE_ROLL = 203,
            FLANGE_PITCH = 204,
            FLANGE_YAW = 205,
            OBJECT_X = 300,
            OBJECT_Y = 301,
            OBJECT_Z = 302,
            OBJECT_ROLL = 303,
            OBJECT_PITCH = 304,
            OBJECT_YAW = 305,
            BASE_X = 400,
            BASE_Y = 401,
            BASE_Z = 402,
            BASE_ROLL = 403,
            BASE_PITCH = 404,
            BASE_YAW = 405,
            NULLSPACE = 500
        };

        enum class DRAGGING_DIRRECTION : int
        {
            NONE = 0,
            POSITION = 1,
            NEGATIVE = -1
        };

        enum class DRAGGING_TYPE : int
        {
            JOINT = 0,
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
        /**
         * @brief 1000hz关节伺服接口
         *
         * @param target_pos 目标位置
         * @return int
         */
        int servoJ(const KDL::JntArray &target_pos);

    private:
        // 运动前检查数据有效性
        int CheckBeforeMove(const JntArray &q, double speed,
                            double acceleration, double time, double radius);

        int CheckBeforeMove(const Frame &pos, double speed,
                            double acceleration, double time, double radius);

        // 实际movej执行线程
        void RunMoveJ(JntArray q, double speed = 1.05, double acceleration = 1.4,
                      double time = 0.0, double radius = 0.0);

        // 实际movel执行线程
        void RunMoveL(const std::vector<KDL::JntArray> &traj);

        // 实际multimovel执行线程
        void RunMultiMoveL(const std::vector<KDL::JntArray> &traj);
        int admittance_teaching(bool asynchronous = false);
        int stop_admittance_teaching();
        int admittance_link(KDL::Frame frame_target, double speed, double acceleration);

        // 关节导纳拖动示教sun
        int joint_admittance_teaching(bool asynchronous = false);

        /**
         * @brief 1000hz位姿伺服接口
         *
         * @param target_frame 目标位置
         * @return int
         */
        int sun_servoJ(const KDL::JntArray &target_pos, const KDL::JntArray &max_vel, const KDL::JntArray &max_acc, double Gain, double lookhead);
        /**
         * @brief 1000hz位姿伺服接口
         *
         * @param target_pose 目标位置
         * @param Gain 比例增益
         * @param lookhead 前瞻时间
         * @param max_vel 最大速度
         * @param max_acc 最大加速度
         * @return int
         */
        int servoL(const KDL::Frame &target_frame);

//        int moveJ_with_speed_scaling(const KDL::JntArray &target_pos, double max_vel, double max_acc, double max_jerk);

        /**
         * @brief 速度缩放函数，运动中每次循环需要调佣一次
         *
         * @return int
         */
//        int speed_scaling();

//        int set_target_speed_frcision(double i)
//        {
//            target_speed_fraction = i;
//            is_fraction_changed = true;
//            return 0;
//        }

    public:
        void test(); // 为了测试

        int stop_joint_admittance_teaching();
        bool isEnabled();

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
        HardwareInterface* hw_interface_{nullptr};
        std::vector<boost::shared_ptr<Drive>> joints_;

        std::string urdf_file_path_; // urdf文件路径

        std::vector<double> target_positions_;      // 当前目标位置
        std::vector<double> target_positions_prev_; // 上一次的目标位置

        std::vector<double> target_velocities_;
        std::vector<double> target_torques_;

        std::vector<std::atomic<double>> pos_;
        std::vector<std::atomic<double>> vel_;
        std::vector<std::atomic<double>> acc_;

        std::vector<double> max_vel_;
        std::vector<double> max_acc_;
        std::vector<double> max_jerk_;

        std::vector<R_INTERP_BASE *> interp_;

        ProfileType profile_type_{trapezoid};

        int jnt_num_;

        double least_motion_time_{0.0};

        Synchronization sync_{SYNC_TIME};

        std::atomic<bool> is_running_motion{false};

        std::vector<bool> need_plan_; // 是否需要重新规划标志

        boost::shared_ptr<boost::thread> otg_motion_thread_{
            nullptr};                                             // otg在线规划线程
        boost::shared_ptr<boost::thread> motion_thread_{nullptr}; // 执行motion线程

        JC_helper::inverse_special_to_SRS SRS_kinematics_;

        // sun
        Frame flange_; //!< 法兰位置姿态
        Frame tool_;   //!< 工具位置姿态
        Frame object_; //!< 工件位置姿态
        std::string yaml_path = "/etc/rocos-yaml/calibration.yaml";
        YAML::Node yaml_node;

        // 六点法标定
        Frame pose1;
        Frame pose2;
        Frame pose3;
        Frame pose4;
        Frame pose5;
        Frame pose6;
        Frame pose_out;
        Frame poseObject1;
        Frame poseObject2;
        Frame poseObject3;

        // 变换矩阵,记得从yaml文件中读取，以及写入到yaml文件中
        Frame T_tool_;
        Frame T_object_;
        // 计算工具系和工件系的变换矩阵的标志位
        bool ErrorState{false};

        std::vector<KDL::JntArray> traj_;
        std::atomic<int> tick_count{0};

        WorkMode work_mode_{WorkMode::Position};                        // 机器人当前模式，默认为位置模式
        RunState run_state_{RunState::Disabled};                        // 机器人当前状态，默认为下使能
        std::shared_ptr<rocos::Trapezoid> T_speed_scaling_ptr{nullptr}; // 速度缩放T型规划器
        std::atomic<double> current_speed_fraction = 1;                 // 当前的速度比例
        std::atomic<double> target_speed_fraction = 1;                  // 期望的速度比例
        std::atomic<double> current_speed_fraction_vel = 0;             // 当前的速度比例变化率
        std::atomic<double> current_speed_fraction_acc = 0;             // 当前的速度比例变化率的变化率
        std::atomic<bool> is_fraction_changed = false;                  // 标识是否需要重置规划器
        double speed_scaling_dt = 0;                                    // 规划器用时

        //**  记录数据，不应该存在**//
        std::ofstream speed_data_csv{"/home/jc/rocos-app/speed_scaling.csv"};
        //**-------------------------------**//

    public:
        Kinematics kinematics_;
        friend void JC_helper::SmartServo_Joint::RunSmartServo(rocos::Robot *);

        friend class JC_helper::SmartServo_Cartesian;

        friend class JC_helper::SmartServo_Nullspace;

        friend void JC_helper::Joint_stop(rocos::Robot *robot_ptr, const KDL::JntArray &current_pos, const KDL::JntArray &last_pos, const KDL::JntArray &last_last_pos);

        friend class JC_helper::admittance;
        // 声明友元类
        friend class JC_helper::admittance_joint;

        //        friend int JC_helper::safety_servo( rocos::Robot* robot_ptr, const std::array< double, 7 >& target_pos );

        friend int JC_helper::safety_servo(rocos::Robot *robot_ptr, const std::vector<double> &target_pos);

        friend int JC_helper::safety_servo(rocos::Robot *robot_ptr, const KDL::JntArray &target_pos);

    private:
        JC_helper::ft_sensor my_ft_sensor;         // 6维力传感器
        bool flag_admittance_turnoff{false};       // 导纳开关
        bool flag_admittance_joint_turnoff{false}; // 关节拖动开关
        std::mutex mtx;                            // 互斥锁

        std::shared_ptr<std::thread> _thread_admittance_teaching{nullptr};
    };

} // namespace rocos

#endif // ROCOS_APP_ROBOT_H
