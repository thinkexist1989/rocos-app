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

#include "logger.h"
#include "drive.h"
#include "hardware_interface.h"
// #include "interpolate.h"
#include "kinematics.h"
#include "dynamics.h"

// #include "JC_helper_kinematics.hpp"
// #include "JC_helper_dynamics.hpp"
#include <Eigen/StdVector> //!< Eigen官网说明 https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html
#include <Eigen/QR>
#include <Eigen/Geometry>
#include <vector>
#include "kdl_parser/kdl_parser.hpp" //!< 解析URDF文件
#include "gripper.hpp"
#include "DHParamsLoader.h"
#include <rocos_app/motion/motion_executor.h>
#include <rocos_app/motion/move_j_submission.h>
#include <rocos_app/motion/position_controller.h>
#include <rocos_app/motion/robot_fsm_gateway.h>
#include <rocos_app/motion/robot_motion_context.h>
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

        explicit Robot(HardwareInterface *hw,
                       const std::string &urdf_file_path = "robot.urdf",
                       const std::string &base_link = "base_link",
                       const std::string &tip = "link7"); // std::string yaml_path = "joint_impedance_control.yaml"

        ~Robot();

        std::string GetRobotState();

        int SetEnabled();

        int SetDisabled();

        bool IsEnabled();

        bool IsDisabled();





        bool parseUrdf(const std::string &urdf_file_path,
                       const std::string &base_link,
                       const std::string &tip);

        bool parseDriveParamsFromUrdf(const std::string &urdf_file_path);

        bool switchHW(HardwareInterface *hw); //TODO: 切换硬件指针

        // 机器人状态机相关
        inline WorkMode getWorkMode() { return work_mode_; }
        bool setWorkMode(WorkMode mode);

        // 运动执行的状态进入/退出封装（基于 FSM，替代旧的 setRunState/is_running_motion）。
        // enterRunning：仅当处于 STOPPED 才允许，原子地门控并转入 RUNNING；
        //               非 STOPPED 时事件被状态机忽略，返回 false（拒绝新运动）。
        // enterStopped：从 RUNNING/PAUSED/ERROR 回到 STOPPED，运动线程结束时调用。
        // isMotionRunning：是否有运动正占用机器人（FSM 处于 RUNNING）。
        bool enterRunning();
        void enterStopped();
        bool isMotionRunning() const;
        bool requestMotionStart();
        bool requestMotionPause();
        bool requestMotionContinue();
        bool requestMotionStop();
        bool notifyMotionError();
        void waitControlCycle();

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

        /// \brief 设置多关节速度约束
        /// \param max_vel 速度约束值
        inline void setJntVelLimits(std::vector<double> &max_vel)
        {
        }

        /// \brief 获取多关节速度约束
        /// \return 速度约束值
        inline std::vector<double> getJntVelLimits() { return max_vel_; };

        /// \brief 设置单关节速度约束
        /// \param id 关节ID
        /// \param max_vel 速度约束值
        inline void setJntVelLimit(int id, double max_vel)
        {
        }

        /// \brief 获取单关节速度约束
        /// \param id 关节ID
        /// \return 速度约束值
        inline double getJntVelLimit(int id) { return max_vel_[id]; }

        /// \brief 设置关节加速度约束
        /// \param max_acc 加速度约束值
        inline void setJntAccLimits(std::vector<double> &max_acc)
        {
        }

        /// \brief 获取多关节加速度约束
        /// \return 加速度约束值
        inline std::vector<double> getJntAccLimits() { return max_acc_; }

        /// \brief 设置单关节加速度约束
        /// \param id 关节ID
        /// \param max_acc 加速度约束值
        inline void setJntAccLimit(int id, double max_acc)
        {
        }

        /// \brief 获取单关节加速度约束
        /// \param id 关节ID
        /// \return 加速度约束值
        inline double getJntAccLimit(int id) { return max_acc_[id]; }

        /// \brief 设置多关节加加速约束
        /// \param max_jerk 多关节加加速约束值
        inline void setJntJerkLimits(std::vector<double> &max_jerk)
        {
        }

        /// \brief 获取多关节加加速约束
        /// \return 多关节加加速约束值
        inline std::vector<double> getJntJerkLimits() { return max_jerk_; }

        /// \brief 设置单关节加加速度约束
        /// \param id 关节id
        /// \param max_jerk 关节加加速约束值
        inline void setJntJerkLimit(int id, double max_jerk)
        {
        }

        /// \brief 获取单关节加加速度约束
        /// \param id 关节ID
        /// \return 关节加加速约束值
        inline double getJntJerkLimit(int id) { return max_jerk_[id]; }

        inline Frame getFlange()
        {
            std::lock_guard<std::mutex> lock(mtx); // 自动获取互斥锁
            return flange_;
        }
        Frame getTool()
        {
            // tool_=flange_*T_tool_;
            std::lock_guard<std::mutex> lock(mtx); // 自动获取互斥锁
            tool_ = flange_ * T_tool_;
            return tool_;
        }
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
            std::ofstream fout(cali_yaml_path_);
            std::ofstream fout1("/opt/rocos/yaml/calibration.yaml");

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

            std::ofstream fout(cali_yaml_path_);
            std::ofstream fout1("/opt/rocos/yaml/calibration.yaml");
            
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

    public:
        int JntToCart(const JntArray &q_in, Frame &p_out)
        {
            return kinematics_.JntToCart(q_in, p_out);
        }
        int CartToJnt(const JntArray &q_init, const Frame &p_in, JntArray &q_out)
        {
            return kinematics_.CartToJnt(q_init, p_in, q_out);
        }

    protected:
        //! 更新法兰系,工具系,工件系poseFlange
        void updateCartesianInfo()
        {
            JntArray q_in(jnt_num_);
            for (int i{0}; i < jnt_num_; i++)
                q_in(i) = joints_[i]->getPosition();
            // Flange Reference
            JntToCart(q_in, flange_);
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
        int PauseMotion();
        int ResumeMotion();
        int StopMotion();

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

        //TODO: Dragging要改成方向向量方式

    private:
        // 运动前检查数据有效性
        int CheckBeforeMove(const JntArray &q, double speed,
                            double acceleration, double time, double radius);

        int CheckBeforeMove(const Frame &pos, double speed,
                            double acceleration, double time, double radius);

        void initializeMotionExecutor();

    protected:
        HardwareInterface* hw_interface_{nullptr};
        std::vector<std::shared_ptr<Drive>> joints_;

        std::string urdf_file_path_; // urdf文件路径
        std::string base_link_;
        std::string tip_;

        std::vector<double> target_positions_;      // 当前目标位置

        std::vector<double> target_velocities_;
        std::vector<double> target_torques_;

        std::vector<std::atomic<double>> pos_;
        std::vector<std::atomic<double>> vel_;
        std::vector<std::atomic<double>> acc_;

        std::vector<double> max_vel_;
        std::vector<double> max_acc_;
        std::vector<double> max_jerk_;

        int jnt_num_; //TODO: 关节数据要放到Model类中，删掉


        std::atomic<bool> motion_thread_stop_requested_{false};
        std::shared_ptr<std::thread> otg_motion_thread_{nullptr};// otg在线规划线程
        std::shared_ptr<std::thread> motion_thread_{nullptr}; // 执行motion线程
        std::unique_ptr<motion::MotionSafetyGuard> motion_safety_guard_{nullptr};
        std::unique_ptr<motion::BasicRobotFsmGateway<Robot>> motion_fsm_gateway_{nullptr};
        std::unique_ptr<motion::PositionController> motion_position_controller_{nullptr};
        std::unique_ptr<motion::RobotMotionContext<Robot>> motion_context_{nullptr};
        std::unique_ptr<motion::MotionExecutor> motion_executor_{nullptr};
        motion::ModelProvider model_provider_;

        // sun
        Frame flange_; //!< 法兰位置姿态
        Frame tool_;   //!< 工具位置姿态
        Frame object_; //!< 工件位置姿态
        std::string cali_yaml_path_ ="/opt/rocos/yaml/calibration.yaml";
        YAML::Node yaml_node;

        //TODO: 这些要封装成Calibration类
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
        // std::shared_ptr<rocos::Trapezoid> T_speed_scaling_ptr{nullptr}; // 速度缩放T型规划器
        std::atomic<double> current_speed_fraction = 1;                 // 当前的速度比例
        std::atomic<double> target_speed_fraction = 1;                  // 期望的速度比例
        std::atomic<double> current_speed_fraction_vel = 0;             // 当前的速度比例变化率
        std::atomic<double> current_speed_fraction_acc = 0;             // 当前的速度比例变化率的变化率
        std::atomic<bool> is_fraction_changed = false;                  // 标识是否需要重置规划器
        double speed_scaling_dt = 0;                                    // 规划器用时

    public:
        Kinematics kinematics_;
        Dynamics dynamics_;


        //TODO: 所有的JC_helper类都需要处理掉
        // friend void JC_helper::SmartServo_Joint::RunSmartServo(rocos::Robot *);
        // friend class JC_helper::SmartServo_Cartesian;
        // friend class JC_helper::SmartServo_Nullspace;
        // friend void JC_helper::Joint_stop(rocos::Robot *robot_ptr, const KDL::JntArray &current_pos, const KDL::JntArray &last_pos, const KDL::JntArray &last_last_pos);
        // friend class JC_helper::admittance;
        // 声明友元类
        // friend class JC_helper::admittance_joint;
        // friend int JC_helper::safety_servo(rocos::Robot *robot_ptr, const std::vector<double> &target_pos);
        // friend int JC_helper::safety_servo(rocos::Robot *robot_ptr, const KDL::JntArray &target_pos);


        friend class RobotHttpServer; // 允许 Server 直接访问 Robot 的私有/保护成员
        //TODO: ================================

    private:
        // JC_helper::ft_sensor my_ft_sensor;         //TODO: 6维力传感器

        bool flag_admittance_turnoff{false};       //TODO: 导纳开关
        bool flag_admittance_joint_turnoff{false}; //TODO: 关节拖动开关

        std::mutex mtx;                            // 互斥锁
        DHParamsLoader loader;

        std::shared_ptr<std::thread> _thread_admittance_teaching{nullptr}; //TODO: 线程都要检查

        //// 机器人状态机封装
        struct Impl;
        std::unique_ptr<Impl> impl_;

        Logger::logger_ptr log_ptr_ = nullptr;

        std::unique_ptr<std::thread> run_thread_handler_  {nullptr}; // 机器人RUNNING开启的线程

        //////////FSM Related function (INTERNAL) ///////////////
    public:
        void on_fsm_reset();

        void on_fsm_enable();

        void on_fsm_disable();

        void on_fsm_start();

        void on_fsm_run();

        void on_fsm_stop();

        void on_fsm_pause();

        void on_fsm_continue();

        void on_fsm_identify();

    };

} // namespace rocos

#endif // ROCOS_APP_ROBOT_H
