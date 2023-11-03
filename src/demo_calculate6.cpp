// 机械臂的工具系六点标定
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h> //yaml-cpp头文件
#include <csignal>
#include <cstdio>
#include <cstdlib>

#include <rocos_app/drive.h>
#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/ethercat/hardware_sim.h>
#include <fstream>
#include <iostream>
#include <rocos_app/robot.h>
#include <rocos_app/robot_service.h>
#include <string>
#include <gflags/gflags.h>

class ToolCalibration
{
public:
    ToolCalibration(std::string yaml_path = "calculate.yaml")
    {
        // Load data from yaml file

        yaml_node = YAML::LoadFile(yaml_path);
        readPoseFromYAML(yaml_node, "pose1", pose1);
        readPoseFromYAML(yaml_node, "pose2", pose2);
        readPoseFromYAML(yaml_node, "pose3", pose3);
        readPoseFromYAML(yaml_node, "pose4", pose4);
        readPoseFromYAML(yaml_node, "pose5", pose5);
        readPoseFromYAML(yaml_node, "pose6", pose6);

        // Calibration of translation
        Eigen::MatrixXd R_EB(9, 3);
        Eigen::MatrixXd P_TB(9, 1);

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

        // 打印R_EB和P_TB
        // std::cout << "R_EB" << R_EB << std::endl;
        // std::cout << "P_TB" << P_TB << std::endl;

        std::cout << "solve_my"<<(R_EB.transpose() * R_EB).inverse() * R_EB.transpose() << std::endl;
        Eigen::Vector3d Pos = (R_EB.transpose() * R_EB).inverse() * R_EB.transpose() * P_TB;
        std::cout << "Pos" << Pos << std::endl;
        //测试pinv
   
        std::cout<<"pinv_R_TB"<<R_EB.completeOrthogonalDecomposition().pseudoInverse()<<std::endl;
        Eigen::MatrixXd pinv_R_TB = R_EB.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::Vector3d Pos1 = pinv_R_TB * P_TB;
        std::cout << "Pos1" << Pos1 << std::endl;

        // Calibration of rotation
        Eigen::Vector3d Vx{(pose5.p - pose4.p).data[0], (pose5.p - pose4.p).data[1], (pose5.p - pose4.p).data[2]};
        Eigen::Vector3d Vz{(pose6.p - pose4.p).data[0], (pose6.p - pose4.p).data[1], (pose6.p - pose4.p).data[2]};
        std::cout<<"Vx"<<Vx<<std::endl;
   
       Vx.normalize();
        Vz.normalize();
      
        Eigen::Vector3d Vy = Vz.cross(Vx);
        Vz = Vx.cross(Vy);
        
        Eigen::Matrix3d R_TB;
        R_TB.block<3, 1>(0, 0) = Vx;
        R_TB.block<3, 1>(0, 1) = Vy;
        R_TB.block<3, 1>(0, 2) = Vz;
        std::cout<<"Vx"<<Vx<<std::endl;
        std::cout<<"Vy"<<Vy<<std::endl;
        std::cout<<"Vz"<<Vz<<std::endl;
        
        Eigen::Matrix3d R0_EB;
        KDL::Rotation pose4_Rotation = pose4.M;
        std::cout<<"pose4.m"<<pose4.M.data[0]<<","<<pose4.M.data[1]<<","<<pose4.M.data[2]<<","<<pose4.M.data[3]<<","<<pose4.M.data[4]<<","<<pose4.M.data[5]<<","<<pose4.M.data[6]<<","<<pose4.M.data[7]<<","<<pose4.M.data[8]<<std::endl;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R0_EB(i, j) = pose4_Rotation(i, j);
            }
        }
        // 打印R0_EB和R_TB
        std::cout << "R0_EB" << R0_EB << std::endl;
        std::cout << "R_TB" << R_TB << std::endl;

        Eigen::Matrix3d Rot = R0_EB.inverse() * R_TB;
        std::cout<<"R0_EB.inverse()"<<R0_EB.inverse()<<std::endl;

        pose_out.p = KDL::Vector(Pos(0), Pos(1), Pos(2));
        pose_out.M = KDL::Rotation(Rot(0, 0), Rot(0, 1), Rot(0, 2), Rot(1, 0), Rot(1, 1), Rot(1, 2), Rot(2, 0), Rot(2, 1), Rot(2, 2));
        std::cout << "pose_out" << pose_out.M.data[0] << "," << pose_out.M.data[1] << "," << pose_out.M.data[2] << "," << pose_out.M.data[3] << "," << pose_out.M.data[4] << "," << pose_out.M.data[5] << "," << pose_out.M.data[6] << "," << pose_out.M.data[7] << "," << pose_out.M.data[8] << std::endl;
        // 旋转矩阵转旋转向量
        Eigen::AngleAxisd rotation_vector2;
        rotation_vector2.fromRotationMatrix(Rot);
        double angle = rotation_vector2.angle();
        Eigen::Vector3d axis = rotation_vector2.axis();

        std::cout << "axis" << axis[0] << "," << axis[1] << "," << axis[2] << std::endl;
        std::cout << "angle" << angle << std::endl;
        Eigen::Vector3d rotVector = axis * angle;
        // 旋转矩阵转RPY
        double roll1, pitch1, yaw1;
        pose_out.M.GetRPY(roll1, pitch1, yaw1);

       
        // 打印结果
        std::cout << "工具系的位置" << pose_out.p.x() << "," << pose_out.p.y() << "," << pose_out.p.z() << std::endl;
        std::cout << "工具系的旋转向量" << rotVector.x()/M_PI*180 << "," << rotVector.y()/M_PI*180 << "," << rotVector.z()/M_PI*180 << std::endl;
        std::cout << "工具系的RPY" << roll1 /M_PI*180 << "," << pitch1 /M_PI*180 << "," << yaw1 /M_PI*180 << std::endl;
        // 工具系位置标定的误差
        double error = 0.0;
        error = (R_EB * Pos - P_TB).norm();
        std::cout << "工具系位置标定的误差" << error << std::endl;
        ;
    }
    void readPoseFromYAML(const YAML::Node &yamlNode, const std::string &poseName, KDL::Frame &pose)
    {
        yaml_node = YAML::LoadFile(yaml_path);

        if (yamlNode[poseName])
        {
            const YAML::Node &poseNode = yamlNode[poseName];
            double x = poseNode[0].as<double>();
            double y = poseNode[1].as<double>();
            double z = poseNode[2].as<double>();
            double roll = poseNode[3].as<double>() * M_PI / 180.0;
            double pitch = poseNode[4].as<double>() * M_PI / 180.0;
            double yaw = poseNode[5].as<double>() * M_PI / 180.0;

            pose.p = KDL::Vector(x, y, z);
            // RPY
            pose.M = KDL::Rotation::RPY(roll, pitch, yaw);

            // Eigen::Vector3d rotationVector(roll, pitch, yaw);
            // Eigen::AngleAxisd angleAxis(rotationVector.norm(), rotationVector.normalized());

            // // 从 Eigen::AngleAxisd 获取轴向和角度
            // Eigen::Vector3d axisVector = angleAxis.axis();
            // double angle = angleAxis.angle();
            // pose.M = KDL::Rotation::Rot(KDL::Vector(axisVector[0], axisVector[1], axisVector[2]), angle);
            // std::cout << poseName << "的位置" << pose.p.x() << "," << pose.p.y() << "," << pose.p.z() << std::endl;
            // std::cout << poseName << "的旋转矩阵" << pose.M.data[0] << "," << pose.M.data[1] << "," << pose.M.data[2] << "," << pose.M.data[3] << "," << pose.M.data[4] << "," << pose.M.data[5] << "," << pose.M.data[6] << "," << pose.M.data[7] << "," << pose.M.data[8] << std::endl;
        }
    }

    YAML::Node yaml_node;
    KDL::Frame pose1;
    KDL::Frame pose2;
    KDL::Frame pose3;
    KDL::Frame pose4;
    KDL::Frame pose5;
    KDL::Frame pose6;
    KDL::Frame pose_out;
    std::string yaml_path = "calculate.yaml";
};

int main()
{
    ToolCalibration toolCal;
    return 0;
}
