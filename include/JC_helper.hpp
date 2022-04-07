#ifndef JC_HELPER_H
#define JC_HELPER_H

#include "kdl/frames.hpp"
#include "robot.h"
#include <iostream>
#include <vector>

#define RESET "\033[0m"

#define BLACK "\033[30m" /* Black */

#define RED "\033[31m" /* Red */

#define GREEN "\033[32m" /* Green */

#define YELLOW "\033[33m" /* Yellow */

#define BLUE "\033[34m" /* Blue */

#define MAGENTA "\033[35m" /* Magenta */

#define CYAN "\033[36m" /* Cyan */

#define WHITE "\033[37m" /* White */

#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */

#define BOLDRED "\033[1m\033[31m" /* Bold Red */

#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */

#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */

#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */

#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */

#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */

#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

//示例
//std::cout << BLUE << " hello world " << std::endl;

namespace JC_helper
{
    /**
      * @brief 姿态插值（四元素球面线性插值）
      *
      * @param start 开始姿态
      * @param end 结束姿态
      * @param s 百分比
      * @return std::vector< double > 四元素x,y,z,w
      */
    std::vector< double > UnitQuaternion_intep( const std::vector< double >& start,
                                                const std::vector< double >& end,
                                                double s );

    /**
      * @brief 姿态插值（轴角法角度线性插值）
      *
      * @param start 开始姿态
      * @param end 结束姿态
      * @param s 百分比
      * @return KDL::Rotation
      */
    KDL::Rotation RotAxisAngle( KDL::Rotation start, KDL::Rotation end, double s );


     /**
      * @brief 圆弧插值
      * 
      * @param F_base_circlestart 起点
      * @param F_base_circleend 终点
      * @param F_base_circleCenter 圆心
      * @param s_p 位置百分比
      * @param s_r 位态百分比
      * @param alpha 夹角
      * @return KDL::Frame 
      */
    KDL::Frame cirlular_trajectory( const KDL::Frame& F_base_circlestart, const KDL::Frame& F_base_circleend, const KDL::Frame& F_base_circleCenter, double s_p, double s_r, double alpha );

    /**
      * @brief 直线规划（使用doubleS速度曲线）
      * 
      * @param start 起始位姿
      * @param end 终止位姿
      * @return std::vector< KDL::Frame > 轨迹 
      */
    KDL::Frame link_trajectory( const KDL::Frame& start, const KDL::Frame& end, double s_p, double s_r );

    int link_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& start, const KDL::Frame& end, double v_start, double v_end, double max_path_v, double max_path_a );
 
    /**
      * @brief 
      * 
      * @param traj 存储的轨迹
      * @param f_start 应该开始运动的位姿
      * @param f_mid 中间的插值点
      * @param f_end 结束的插值点
      * @param next_f_start 下一段运动应该开始的位姿
      * @param current_path_start_v 应该开始运动的线速度
      * @param next_path_start_v 下一段运动应该开始的线速度
      * @param bound_dist 过渡半径
      * @param max_path_v 最大线速度
      * @param max_path_a 最大线加速度
      * @param next_max_path_v 下一段的最大线速度（如果无下一段了，此参数无意义）
      * @return int 
      */
    int multilink_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& f_start, const KDL::Frame& f_mid, const KDL::Frame& f_end, KDL::Frame& next_f_start, double current_path_start_v, double& next_path_start_v, double bound_dist, double max_path_v, double max_path_a, double next_max_path_v = 1 );

/**
 * @brief 给定三点，计算出圆心
 * 
 * @param center 
 * @param f_p1 
 * @param f_p2 
 * @param f_p3 
 * @return int 
 */
    int circle_center( KDL::Frame& center, const KDL::Frame& f_p1, const KDL::Frame& f_p2, const KDL::Frame& f_p3 );

/**
 * @brief movec轨迹计算
 * 
 * @param traj 输出计算结果
 * @param f_p1 第一点
 * @param f_p2 第二点
 * @param f_p3 第三点
 * @param max_path_v 最大速度
 * @param max_path_a 最大加速度
 * @param fixed_rotation 是否保持姿态：是->姿态不变；否->姿态指向圆心方向变换
 * @return int 
 */
    int circle_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& f_p1, const KDL::Frame& f_p2, const KDL::Frame& f_p3, double max_path_v = 0.01, double max_path_a = 0.01, bool fixed_rotation = true );

/**
 * @brief 位置保持，只旋转姿态
 * 
 * @param traj 输出计算结果
 * @param f_p 位置保持
 * @param f_r1 起始姿态
 * @param f_r2 终止姿态
 * @param max_path_v 最大速度
 * @param max_path_a 最大加速度
 * @param equivalent_radius 等效半径，用于乘以角度得到等效弧度
 * @return int 
 */
    int rotation_trajectory( std::vector< KDL::Frame >& traj, const KDL::Vector& f_p, const KDL::Rotation& f_r1, const KDL::Rotation& f_r2, double max_path_v = 0.01, double max_path_a = 0.01, double equivalent_radius = 0.01 );


}  // namespace JC_helper

#endif