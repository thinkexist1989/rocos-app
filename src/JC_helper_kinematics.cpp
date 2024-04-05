#include <rocos_app/JC_helper_kinematics.hpp>
#include <rocos_app/robot.h>

//** 显式指定关节数量 **//
uint32_t jointNum{0};

namespace JC_helper
{
#pragma region //* 轨迹计算函数

    /**
     * @brief 姿态插值（四元素球面线性插值）
     * @note 注意没有s的范围保护
     * @param start 开始姿态
     * @param end 结束姿态
     * @param s 百分比
     * @return std::vector< double > 四元素x,y,z,w
     */
    std::vector<double> UnitQuaternion_intep(const std::vector<double> &start,
                                             const std::vector<double> &end,
                                             double s)
    {
        constexpr double eps = 1E-7;

        if (s < 0 || s > 1)
        {
            PLOG_ERROR << "UnitQuaternion interpolation failure";
        }

        double cosTheta = start[0] * end[0] + start[1] * end[1] + start[2] * end[2] +
                          start[3] * end[3];

        std::vector<double> start_2 = start;

        //** 这里是为了取最短路径 **//
        if (cosTheta < 0)
        {
            for (int i = 0; i < 4; i++)
                start_2[i] *= -1;
            cosTheta *= -1;
        }
        //**-------------------------------**//

        double theta = acos(cosTheta);

        //! theta本应该计算为0，但是实际可能为0.000001，即使有(eps=1E-7)限定范围，仍然有误判可能,所以最好使用isnan()
        if (abs(theta) < eps || s == 0)
            return start_2;
        else
        {
            double coefficient_1 = sin((1 - s) * theta) / sin(theta);
            double coefficient_2 = sin((s)*theta) / sin(theta);

            std::vector<double> res{
                coefficient_1 * start_2[0] + coefficient_2 * end[0],
                coefficient_1 * start_2[1] + coefficient_2 * end[1],
                coefficient_1 * start_2[2] + coefficient_2 * end[2],
                coefficient_1 * start_2[3] + coefficient_2 * end[3]};

            //! 防止误判
            for (const auto &var : res)
                if (isnan(var))
                    return start_2;

            return res;
        }
    }

    KDL::Rotation RotAxisAngle(KDL::Rotation start, KDL::Rotation end, double s)
    {
        if (s > 1 || s < 0)
        {
            std::cerr << RED << "values of S outside interval [0,1]" << GREEN << std::endl;
        }

        KDL::Rotation R_start_end = start.Inverse() * end;
        KDL::Vector axis;
        double angle = R_start_end.GetRotAngle(axis);
        return start * KDL::Rotation::Rot2(axis, angle * s);
    }

    KDL::Frame circle(const KDL::Frame &F_base_circlestart, const KDL::Frame &F_base_circleend, const KDL::Frame &F_base_circleCenter, double s_p, double s_r, double alpha, bool &success)
    {
        using namespace KDL;

        if (s_p > 1 || s_p < 0)
        {
            PLOG_ERROR << "values of s_p outside interval [0,1]";
            success = false;
            return KDL::Frame{};
        }
        if (s_r > 1 || s_r < 0)
        {
            PLOG_ERROR << "values of s_r outside interval [0,1]";
            success = false;
            return KDL::Frame{};
        }

        KDL::Vector x = F_base_circlestart.p - F_base_circleCenter.p;
        double radius = x.Normalize();
        KDL::Vector tmpv = F_base_circleend.p - F_base_circleCenter.p; // 第二直线段上的半径段
        tmpv.Normalize();

        Vector z(x * tmpv); // Z轴
        double n = z.Normalize();

        if (n < epsilon)
        {
            std::cerr << RED << "circle(): Z Axis Calculation error " << GREEN << std::endl;
            success = false;
            return KDL::Frame{};
        }

        KDL::Frame F_base_circleCenter_(KDL::Rotation{x, (z * x), z}, F_base_circleCenter.p);
        std::vector<double> Quaternion_start{0, 0, 0, 0};
        std::vector<double> Quaternion_end{0, 0, 0, 0};
        std::vector<double> Quaternion_interp{0, 0, 0, 0};
        F_base_circlestart.M.GetQuaternion(Quaternion_start.at(0), Quaternion_start.at(1), Quaternion_start.at(2), Quaternion_start.at(3));
        F_base_circleend.M.GetQuaternion(Quaternion_end.at(0), Quaternion_end.at(1), Quaternion_end.at(2), Quaternion_end.at(3));
        Quaternion_interp = UnitQuaternion_intep(Quaternion_start, Quaternion_end, s_r);

        success = true;
        return KDL::Frame(KDL::Rotation::Quaternion(Quaternion_interp[0], Quaternion_interp[1], Quaternion_interp[2], Quaternion_interp[3]), F_base_circleCenter_ * Vector{radius * cos(s_p * alpha), radius * sin(s_p * alpha), 0});
    }
    
    int joint_pos( const KDL::JntArray& start, const KDL::JntArray& end, double s_p, KDL::JntArray& joint_pos )
    {
        if ( s_p > 1 || s_p < 0 )
        {
            PLOG_ERROR << "values of s_p outside interval [0,1]";
            return -1;
        }
        //** 变量初始化 **//
        KDL::JntArray joint_offset( jointNum );
        KDL::Subtract( end, start, joint_offset );
        KDL::Multiply( joint_offset, s_p, joint_offset );
        KDL::Add( start, joint_offset, joint_pos );

        return 0;
    }

    int link_pos(const KDL::Frame &start, const KDL::Frame &end, double s_p, double s_r, KDL::Frame &Cartesian_pos)
    {
        if (s_p > 1 || s_p < 0)
        {
            PLOG_ERROR << "values of s_p outside interval [0,1]";
            return -1;
        }
        if (s_r > 1 || s_r < 0)
        {
            PLOG_ERROR << "values of s_r outside interval [0,1]";
            return -1;
        }

        //** 变量初始化 **//
        Cartesian_pos.p = start.p + (end.p - start.p) * s_p;

        std::vector<double> Quaternion_start{0, 0, 0, 0};
        std::vector<double> Quaternion_end{0, 0, 0, 0};
        std::vector<double> Quaternion_interp{0, 0, 0, 0};
        start.M.GetQuaternion(Quaternion_start.at(0), Quaternion_start.at(1), Quaternion_start.at(2), Quaternion_start.at(3));
        end.M.GetQuaternion(Quaternion_end.at(0), Quaternion_end.at(1), Quaternion_end.at(2), Quaternion_end.at(3));
        Quaternion_interp = UnitQuaternion_intep(Quaternion_start, Quaternion_end, s_r);
        Cartesian_pos.M = KDL::Rotation::Quaternion(Quaternion_interp[0], Quaternion_interp[1], Quaternion_interp[2], Quaternion_interp[3]);

        return 0;
    }

    int link_vel(const KDL::Frame &start, const KDL::Frame &end, double v_p, double v_r, KDL::Twist &Cartesian_vel)
    {
        if (v_p < 0)
        {
            PLOG_ERROR << "values of v_p is less than 0";
            return -1;
        }
        if (v_r < 0)
        {
            PLOG_ERROR << "values of v_r is less than 0";
            return -1;
        }

        Cartesian_vel.vel = (end.p - start.p) * v_p;

        KDL::Rotation rot_start_end = start.M.Inverse() * end.M;
        KDL::Vector aixs = rot_start_end.GetRot();
        Cartesian_vel.rot = aixs * v_r;

        return 0;
    }

    int link_trajectory(std::vector<KDL::Frame> &traj, const KDL::Frame &start, const KDL::Frame &end, double v_start, double v_end, double max_path_v, double max_path_a)
    {
        if (end == start) // 起始和终止位置一致，无需规划
            return 0;
        else
        {
            //** 变量初始化 **//
            KDL::Vector Pstart = start.p;
            KDL::Vector Pend = end.p;
            double Plength = (Pend - Pstart).Norm();
            KDL::Rotation R_start_end = start.M.Inverse() * end.M;
            KDL::Vector ration_axis;
            double angle = R_start_end.GetRotAngle(ration_axis);
            const double equivalent_radius = 0.1; // TODO: 等效半径，但是具体应该是什么值可以考虑
            double Rlength = equivalent_radius * abs(angle);
            double Path_length{0};

            double T_link{0};
            ::rocos::DoubleS doubleS_P;
            ::rocos::DoubleS doubleS_R;
            //**-------------------------------**//

            // 大旋转，小移动(或没移动)情况，此时要求起始速度为0
            if (Rlength > Plength)
            {
                if (v_start != 0 || v_end != 0)
                {
                    PLOG_ERROR << "Error_MotionPlanning_Not_Feasible";
                    return -1;
                }
                Path_length = Rlength;
            }
            // 大移动，小旋转
            else
                Path_length = Plength;

            doubleS_P.planDoubleSProfile(0, 0, 1, v_start / Path_length, v_end / Path_length, max_path_v / Path_length, max_path_a / Path_length, max_path_a * 2 / Path_length);
            bool isplanned = doubleS_P.isValidMovement();
            if (!isplanned || doubleS_P.getDuration() <= 0)
            {
                PLOG_ERROR << "Error_MotionPlanning_Not_Feasible";
                return -1;
            }

            doubleS_R.planDoubleSProfile(0, 0, 1, 0, 0, max_path_v / Path_length, max_path_a / Path_length, max_path_a * 2 / Path_length);
            isplanned = doubleS_R.isValidMovement();
            if (!isplanned || doubleS_R.getDuration() <= 0)
            {
                PLOG_ERROR << "Error_MotionPlanning_Not_Feasible";
                return -1;
            }

            if (doubleS_R.getDuration() != doubleS_P.getDuration())
            {
                doubleS_R.JC_scaleToDuration(doubleS_P.getDuration());
            }

            T_link = doubleS_P.getDuration();

            double dt = 0;
            double s_p = 0;
            double s_r = 0;

            KDL::Frame link_target{};

            //** 轨迹计算 **//
            while (dt >= 0 && dt <= T_link)
            {
                s_p = doubleS_P.pos(dt);
                s_r = doubleS_R.pos(dt);

                if (link_pos(start, end, s_p, s_r, link_target) < 0)
                {
                    PLOG_ERROR << " link calculating failure";
                    return -1;
                }
                traj.push_back(link_target);
                dt = dt + 0.001;
            }
            //**-------------------------------**//
            return 0;
        }
    }

    int link_trajectory(std::vector<KDL::Frame> &traj, const KDL::Frame &start, const KDL::Frame &end, double max_path_v, double max_path_a)
    {
        //** 变量初始化 **//
        KDL::Vector Pstart = start.p;
        KDL::Vector Pend = end.p;
        double Plength = (Pend - Pstart).Norm();
        KDL::Rotation R_start_end = start.M.Inverse() * end.M;
        KDL::Vector ration_axis;
        double angle = R_start_end.GetRotAngle(ration_axis);
        const double equivalent_radius = 0.1;
        double Rlength = (equivalent_radius * abs(angle));
        double Path_length = std::max(Plength, Rlength);
        double s = 0;
        ::rocos::DoubleS doubleS;
        //**-------------------------------**//

        doubleS.planProfile(0, 0.0, 1.0, 0, 0, max_path_v / Path_length, max_path_a / Path_length,
                            max_path_a * 2 / Path_length);

        if (!doubleS.isValidMovement() || !(doubleS.getDuration() > 0))
        {
            PLOG_ERROR << "link trajectory "
                       << "is infeasible ";
            return -1;
        }

        //** 变量初始化 **//
        double dt = 0;
        double duration = doubleS.getDuration();
        KDL::Frame interp_frame{};
        //**-------------------------------**//

        //** 轨迹计算 **//
        while (dt <= duration)
        {
            s = doubleS.pos(dt);

            if (link_pos(start, end, s, s, interp_frame) < 0)
            {
                PLOG_ERROR << " link calculating failure";
                return -1;
            }
            traj.push_back(interp_frame);
            dt += 0.001;
        }
        //**-------------------------------**//

        return 0;
    }

    int link_trajectory(std::vector<KDL::Twist> &traj_vel, const KDL::Frame &start, const KDL::Frame &end, double max_path_v, double max_path_a)
    {
        //** 变量初始化 **//
        double Plength = (end.p - start.p).Norm();
        KDL::Rotation R_start_end = start.M.Inverse() * end.M;
        KDL::Vector ration_axis;
        double angle = R_start_end.GetRotAngle(ration_axis);
        const double equivalent_radius = 0.1;
        double Rlength = (equivalent_radius * abs(angle));
        double Path_length = std::max(Plength, Rlength);
        double doubles_vel = 0;
        ::rocos::DoubleS doubleS;
        //**-------------------------------**//

        doubleS.planProfile(0, 0.0, 1.0, 0, 0, max_path_v / Path_length, max_path_a / Path_length,
                            max_path_a * 2 / Path_length);

        if (!doubleS.isValidMovement() || !(doubleS.getDuration() > 0))
        {
            PLOG_ERROR << "link trajectory "
                       << "is infeasible ";
            return -1;
        }

        //** 变量初始化 **//
        double dt = 0;
        double duration = doubleS.getDuration();
        KDL::Twist interp_vel{};

        //**-------------------------------**//

        //** 轨迹计算 **//
        while (dt <= duration)
        {
            doubles_vel = doubleS.vel(dt);

            if (link_vel(start, end, doubles_vel, doubles_vel, interp_vel) < 0)
            {
                PLOG_ERROR << " link calculating failure";
                return -1;
            }

            traj_vel.push_back(interp_vel);
            dt += 0.001;
        }
        //**-------------------------------**//

        return 0;
    }

    int multilink_trajectory(std::vector<KDL::Frame> &traj, const KDL::Frame &f_start, const KDL::Frame &f_mid, const KDL::Frame &f_end, KDL::Frame &next_f_start, double current_path_start_v, double &next_path_start_v, double s_bound_dist, double max_path_v, double max_path_a, double next_max_path_v)
    {
        using namespace KDL;
        //** 变量初始化 **//
        constexpr double eps = 1E-7;
        Vector p1{f_start.p};
        Vector p_mid{f_mid.p};
        Vector p2{f_end.p};

        Vector ab = p_mid - p1;
        Vector bc = p2 - p_mid;

        double abdist = ab.Norm();
        double bcdist = bc.Norm();

        // bound_dist不再支持绝对距离，使用相对距离更合适
        double bound_dist = std::max(std::min(abs(s_bound_dist), 1.0), 0.0) * std::min(abdist, bcdist);
        //**-------------------------------**//

        //** 数据有效性检查 **//
        //! 注释，为了通过只旋转，不移动时的检查
        /*         if ( abdist < eps )
                {
                    std::cout << RED << "multi_link_trajectory()：f_start and  f_mid are too close " << GREEN << std::endl;
                    return -1;
                }

                if ( bcdist < eps )
                {
                    std::cout << RED << "multi_link_trajectory():f_mid and  f_end are too close " << GREEN << std::endl;
                    return -1;
                } */

        if (abdist < eps) // 说明两种情况：①本段只旋转，不移动，要求开始速度为0 ②上段bound_dist等于abdist等于bcdist.只有圆弧运动，要求结束速度不为0，既本段开始速度不为0
        {
            //! 目前第②情况不应该出现，或者很不好处理，选择报错处理
            //! 第①种情况可以处理
            if (current_path_start_v != 0)
            {
                PLOG_ERROR << "bound_dist of last motion  is too  large,try to decrease it";
                return -1;
            }
        }

        if (current_path_start_v < eps && abdist > eps && (abdist - bound_dist) < eps) // 只存在圆弧段，且上段速度为0，在圆弧匀速约束这不允许
        {
            PLOG_ERROR << "the Link length is not allowed to be equal to 0,When starting velocity of this motion is equal to 0,try to increase bound_dist " << GREEN << std::endl;
            return -1;
        }

        if (bound_dist > abdist) // 这个条件说明：本段只旋转，不移动时一定保证bound_dist=0
        {
            if (abdist < eps)
            {
                PLOG_WARNING << "bound_dist不为0;只旋转情况下,bound_dist需要为0,当前已强制置0";
                bound_dist = 0;
            }
            else
            {
                PLOG_ERROR << "bound_dist is too  large,try to decrease it";
                return -1;
            }
        }

        if (bound_dist > bcdist) // 这个条件说明：【正常情况下】如果下一段只旋转，不移动，则本段bound_dist必需=0(因为bcdist=0)，则本段结束速度一定为0
        {
            PLOG_ERROR << "bound_dist is too  large,try to decrease it";
            return -1;
        }

        if (current_path_start_v > max_path_v) //! 防止起始速度就超过最大可达线速度 ,理论上不可能发生
        {
            PLOG_ERROR << " current_path_start_v  > max_path_v";
            PLOG_ERROR << "current_path_start_v = " << current_path_start_v;
            PLOG_ERROR << " max_path_v = " << max_path_v << GREEN;
            return -1;
        }

        if (bound_dist < 0) // bound_dist必需为正数
        {
            PLOG_ERROR << "bound_dist must be positive ";
            return -1;
        }

        //**-------------------------------**//

        // 利用向量乘积公式，求得两向量的夹角 ,并且限定范围在0-180
        double cos_alpha = std::max(-1., std::min(dot(ab, bc) / abdist / bcdist, 1.));
        // 两段直线夹角接近0,则两段直线合并为一条处理
        if ((1 - cos_alpha) <= eps)
        {
            next_f_start = f_start;
            next_path_start_v = current_path_start_v;
            return 0;
        }
        // 两段直线夹角接近180，则不允许圆弧过渡
        else if ((cos_alpha - (-1)) <= eps)
            bound_dist = 0;

        // 求解两段直线的夹角和圆弧的半径
        double alpha = acos(cos_alpha);
        double radius = bound_dist * tan((M_PI - alpha) / 2);

        // 求解过渡半径占总长的百分比
        double s_bound_dist_1 = bound_dist < eps ? 0 : bound_dist / abdist;
        double s_bound_dist_2 = bound_dist < eps ? 0 : bound_dist / bcdist; // 避免只旋转时出现0/0

        Frame F_base_circlestart;
        if (link_pos(f_start, f_mid, 1 - s_bound_dist_1, 1 - s_bound_dist_1, F_base_circlestart) < 0)
        {
            PLOG_ERROR << "link calculation failure";
            return -1;
        }

        Frame F_base_circleend;
        if (link_pos(f_mid, f_end, s_bound_dist_2, s_bound_dist_2, F_base_circleend) < 0)
        {
            PLOG_ERROR << "link calculation failure";
            return -1;
        }

        Vector de = F_base_circlestart.p - f_start.p; // 实际要走的直线段的距离
        double dedist = de.Norm();                    // 实际要走的直线段的距离

        Vector V_base_t = ab * (ab * bc); // 圆弧中垂直于第一段直线的半径向量
        V_base_t.Normalize();

        Frame F_base_circleCenter{F_base_circlestart.p - V_base_t * radius};

        // double s_cirlular_v{ 0 };
        double cirlular_v{0}; // 应该使用绝对速度

        if (s_bound_dist_1 <= eps) // 不存在圆弧，圆弧速度为0
            cirlular_v = 0;
        else if (abs(1 - s_bound_dist_1) <= eps) // 为啥不加abs呢？理由：s_bound_dist_1的范围为：[0-1),没有加的必要
            cirlular_v = current_path_start_v;   // 不存在直线，圆弧速度为当前段运动速度
        else
            // cirlular_v = s_bound_dist_1 * std::min( max_path_v, next_max_path_v );  //考虑当前和下次的运动，选取最小值（代表当前运动和下一段运动的约束下，最大可行速度）
            cirlular_v = std::max(0.0, std::min(s_bound_dist_1 * 5, 1.0)) * std::min(max_path_v, next_max_path_v); // 考虑当前和下次的运动，选取最小值（代表当前运动和下一段运动的约束下，最大可行速度）

#pragma region // 第一段直线速度轨迹规划

        double T_link = 0;
        ::rocos::DoubleS doubleS_1_P;
        ::rocos::DoubleS doubleS_1_R;
        // 存在直线或只旋转情况，才需要规划;如果本段只旋转，不移动，那么bound_dist一定=0->s_bound_dist_1一定=0，下面程序一定执行
        if ((1 - s_bound_dist_1) > eps)
        {
            double Path_length{dedist};
            // dedist==0,代表只旋转，不移动的情况
            if (Path_length < eps)
            {
                KDL::Rotation R_start_end = f_start.M.Inverse() * F_base_circlestart.M;
                KDL::Vector ration_axis;
                double angle = R_start_end.GetRotAngle(ration_axis);
                constexpr double equivalent_radius = 0.1;
                double Rlength = equivalent_radius * abs(angle);
                Path_length = Rlength;
            }

            doubleS_1_P.planDoubleSProfile(0, 0, 1, current_path_start_v / Path_length, cirlular_v / Path_length, max_path_v / Path_length, max_path_a / Path_length, max_path_a * 2 / Path_length);
            bool isplanned = doubleS_1_P.isValidMovement();
            if (!isplanned || !(doubleS_1_P.getDuration() > 0))
            {
                PLOG_ERROR << "Linear trajectory planning fails,try to decrease given parameter [max_path_v] ";
                return -1;
            }

            doubleS_1_R.planDoubleSProfile(0, 0, 1, 0, 0, max_path_v / Path_length, max_path_a / Path_length, max_path_a * 2 / Path_length);
            isplanned = doubleS_1_R.isValidMovement();
            if (!isplanned || !(doubleS_1_R.getDuration() > 0))
            {
                PLOG_ERROR << "Linear trajectory planning fails,try to decrease given parameter [max_path_v] ";
                return -1;
            }

            if (doubleS_1_R.getDuration() != doubleS_1_P.getDuration())
            {
                doubleS_1_R.JC_scaleToDuration(doubleS_1_P.getDuration());
            }

            T_link = doubleS_1_P.getDuration();
        }
#pragma endregion

        if (s_bound_dist_1 <= eps) // 不存在圆弧，下段速度为0
        {
            next_path_start_v = 0;
        }
        else if ((1 - s_bound_dist_1) <= eps) // 不存在直线，下段速度为当前段运动速度
        {
            next_path_start_v = current_path_start_v;
        }
        else
        {
            next_path_start_v = cirlular_v; // 下段开始速度即为计算出的圆弧速度(因为我设定圆弧匀速，后期可能会改)
        }
        next_f_start = F_base_circleend; // 下一段运动应该开始的位姿

#pragma region // 圆弧段时间
        double T_cirlular = 0;
        ::rocos::DoubleS doubleS_2_R;
        if (s_bound_dist_1 > eps) // 存在圆弧才应该规划
        {
            T_cirlular = (radius * alpha) / next_path_start_v;

            doubleS_2_R.planDoubleSProfile(0, 0, 1, 0, 0, max_path_v / (radius * alpha), max_path_a / (radius * alpha), max_path_a * 2 / (radius * alpha));
            bool isplanned = doubleS_2_R.isValidMovement();
            if (!isplanned || !(doubleS_2_R.getDuration() > 0))
            {
                PLOG_ERROR << "Circular trajectory planning fails,try to decrease given parameter [max_path_v] ";
                return -1;
            }

            if (doubleS_2_R.getDuration() != T_cirlular)
            {
                doubleS_2_R.JC_scaleToDuration(T_cirlular);
            }
        }

#pragma endregion

        double t_total = 0;
        double s_p = 0;
        double s_r = 0;
        KDL::Frame link_target{};
        KDL::Frame circular_target{};
        bool circule_success{true};

        std::cout << "T  = " << (T_link + T_cirlular) << std::endl;
        std::cout << "T_cirlular= " << (T_cirlular) << std::endl;
        std::cout << "T_link= " << (T_link) << std::endl;

        //** 轨迹计算 **//
        while (t_total >= 0 && t_total <= (T_link + T_cirlular))
        {
            if (t_total >= 0 && t_total <= T_link && T_link != 0)
            {
                s_p = doubleS_1_P.pos(t_total);
                s_r = doubleS_1_R.pos(t_total);

                if (link_pos(f_start, F_base_circlestart, s_p, s_r, link_target) < 0)
                {
                    PLOG_ERROR << "link calculation failure";
                    return -1;
                }
                traj.push_back(link_target);
            }
            else
            {
                s_p = (t_total - T_link) / T_cirlular;
                s_r = doubleS_2_R.pos(t_total - T_link);

                circular_target = circle(F_base_circlestart, F_base_circleend, F_base_circleCenter, s_p, s_r, alpha, circule_success);
                if (!circule_success)
                {
                    PLOG_ERROR << "circular calculation failure";
                    return -1;
                }
                traj.push_back(circular_target);
            }

            t_total = t_total + 0.001;
        }
        //**-------------------------------**//
        return 0;
    }

    int circle_center(KDL::Frame &center, const KDL::Frame &f_p1, const KDL::Frame &f_p2, const KDL::Frame &f_p3)
    {
        const double eps = 1E-7;
        using namespace KDL;
        KDL::Vector v1 = f_p2.p - f_p1.p;
        KDL::Vector v2 = f_p3.p - f_p1.p;

        if (v1.Normalize() < eps)
        {
            PLOG_ERROR << "f_p1不能等于f_p2" << std::endl;
            return -1;
        }
        if (v2.Normalize() < eps)
        {
            PLOG_ERROR << "f_p1不能等于f_p3" << std::endl;
            return -1;
        }

        KDL::Vector axis_z{v2 * v1};

        if (axis_z.Normalize() < eps)
        {
            PLOG_ERROR << "三点不能共线或过于趋向直线" << std::endl;
            return -1;
        }

        KDL::Vector axis_x{v1};
        KDL::Vector axis_y{axis_z * axis_x};
        axis_y.Normalize();

        v1 = f_p2.p - f_p1.p;
        v2 = f_p3.p - f_p1.p;

        // 在新坐标系上
        //  f_p2 = [dot(v1,axis_x),0] = [bx,0]
        //  f_P3 = [dot(v2,axis_x),dot(v2,axis_y)=[cx,cy]
        // 圆心一定位于[bx/2,0]的直线上，所以假设圆心为[bx/2,0]
        // 在利用半径相等公式求解h

        double bx = dot(v1, axis_x);
        double cx = dot(v2, axis_x);
        double cy = dot(v2, axis_y);

        double h = ((cx - bx / 2) * (cx - bx / 2) + cy * cy - (bx / 2) * (bx / 2)) / (2 * cy);
        center.p = f_p1.p + axis_x * (bx / 2) + axis_y * h;
        return 0;
    }

    int circle_trajectory(std::vector<KDL::Frame> &traj, const KDL::Frame &f_p1, const KDL::Frame &f_p2, const KDL::Frame &f_p3, double max_path_v, double max_path_a, bool fixed_rotation)
    {
        using namespace KDL;
        const double epsilon = 1e-6;
        KDL::Frame center;

        if (circle_center(center, f_p1, f_p2, f_p3) < 0) // 找到圆心的位置
        {
            PLOG_ERROR << "unable to calculate center of circle";
            return -1;
        }

        KDL::Vector axis_x = f_p1.p - center.p;

        double radius = axis_x.Normalize();

        KDL::Vector axis_tem = f_p2.p - center.p; // 第二直线段上的半径段
        axis_tem.Normalize();

        KDL::Vector axis_z(axis_x * axis_tem); // 向上的Z轴
        if (axis_z.Normalize() < epsilon)
        {
            std::cout << RED << "circle_trajectory():axis_x and axis_tem  cannot be parallel" << std::endl;
            return -1;
        }

        KDL::Vector axis_y{(axis_z * axis_x)};
        axis_y.Normalize();

        KDL::Frame center_(KDL::Rotation{axis_x, axis_y, axis_z}, center.p); // 确定圆心的方向

        //** 计算旋转角度 **//
        KDL::Vector f_p2_ = center_.Inverse() * f_p2.p; // 变换f_p2.p从基座标系到圆心坐标系
        KDL::Vector f_p3_ = center_.Inverse() * f_p3.p; // 变换f_p3.p从基座标系到圆心坐标系
        double theta13 = 0;

        if (f_p3_(1) < 0) // 变换f_p3从-180>180到 0>360_
            theta13 = atan2(f_p3_(1), f_p3_(0)) + 2 * M_PI;
        else
            theta13 = atan2(f_p3_(1), f_p3_(0));
        PLOG_INFO << "theta13= " << theta13 * 180 / M_PI;
        //**-------------------------------**//

        ::rocos::DoubleS doubleS;
        double path_length = (radius * theta13);
        doubleS.planDoubleSProfile(0, 0, 1, 0, 0, max_path_v / path_length, max_path_a / path_length, 2 * max_path_a / path_length);
        if (!doubleS.isValidMovement() || !(doubleS.getDuration() > 0))
        {
            PLOG_ERROR << "planDoubleSProfile failed";
            return -1;
        }
        double T_total = doubleS.getDuration();
        std::cout << BLUE << "T_total = " << T_total << GREEN << std::endl;

        KDL::Rotation R_w_p1 = f_p1.M;
        KDL::Vector Rotation_axis = R_w_p1.Inverse() * axis_z;

        //** 轨迹规划 **//
        double dt{0.0};
        double s{0.0};
        while (dt <= T_total)
        {
            s = doubleS.pos(dt);
            if (fixed_rotation)
                traj.push_back(KDL::Frame(R_w_p1, center_ * KDL::Vector{radius * cos(s * theta13), radius * sin(s * theta13), 0}));
            else
                traj.push_back(KDL::Frame(R_w_p1 * KDL::Rotation::Rot2(Rotation_axis, theta13 * s), center_ * KDL::Vector{radius * cos(s * theta13), radius * sin(s * theta13), 0}));
            dt += 0.001;
        }
        return 0;
    }

    int circle_trajectory(std::vector<KDL::Frame> &traj, const KDL::Frame &f_p1, const KDL::Frame &center, double theta13, int axiz, double max_path_v, double max_path_a, bool fixed_rotation)
    {
        using namespace KDL;
        const double epsilon = 1e-6;

        double radius = (f_p1.p - center.p).Normalize(); // 半径

        if (radius < epsilon)
        {
            PLOG_ERROR << "圆弧圆心太靠近起始位置";
            return -1;
        }
        else if (abs(theta13) < epsilon)
        {
            PLOG_ERROR << "圆弧旋转角度太小";
            return -1;
        }
        else if (abs(axiz) > 2)
        {
            PLOG_ERROR << "圆弧旋转轴只能选择:【0-X、1-Y、2-Z】";
            return -1;
        }

        Vector center_f_p1 = f_p1.p - center.p; // 待旋转的向量
        Vector rot_axiz{};

        switch (axiz)
        {
        case 0:
            rot_axiz = center.M.UnitX();
            break;
        case 1:
            rot_axiz = center.M.UnitY();
            break;
        case 2:
            rot_axiz = center.M.UnitZ();
            break;
        default:
            PLOG_ERROR << "圆弧旋转轴只能选择:【0-X、1-Y、2-Z】";
            return -1;
        }

        ::rocos::DoubleS doubleS;
        double path_length = radius * abs(theta13);
        doubleS.planDoubleSProfile(0, 0, 1, 0, 0, max_path_v / path_length, max_path_a / path_length, 2 * max_path_a / path_length);
        if (!doubleS.isValidMovement() || !(doubleS.getDuration() > 0))
        {
            PLOG_ERROR << "planDoubleSProfile failed";
            return -1;
        }
        double T_total = doubleS.getDuration();

        //** 轨迹规划 **//
        double dt{0.0};
        double s{0.0};
        while (dt <= T_total)
        {
            s = doubleS.pos(dt);
            if (fixed_rotation)
                traj.push_back(KDL::Frame(f_p1.M, center.p + KDL::Rotation::Rot2(rot_axiz, theta13 * s) * center_f_p1));
            else
                traj.push_back(KDL::Frame(KDL::Rotation::Rot2(rot_axiz, theta13 * s) * f_p1.M, center.p + KDL::Rotation::Rot2(rot_axiz, theta13 * s) * center_f_p1));
            dt += 0.001;
        }
        return 0;
    }

    int circle_trajectory(std::vector<KDL::Twist> &traj_vel, const KDL::Frame &f_p1, const KDL::Frame &f_p2, const KDL::Frame &f_p3, double max_path_v, double max_path_a, bool fixed_rotation)
    {
        const double epsilon = 1e-6;
        KDL::Frame center;

        if (circle_center(center, f_p1, f_p2, f_p3) < 0) // 找到圆心的位置
        {
            PLOG_ERROR << "unable to calculate center of circle";
            return -1;
        }

        KDL::Vector axis_x = f_p1.p - center.p;

        double radius = axis_x.Normalize();

        KDL::Vector axis_tem = f_p2.p - center.p; // 第二直线段上的半径段
        axis_tem.Normalize();

        KDL::Vector axis_z(axis_x * axis_tem); // 向上的Z轴
        if (axis_z.Normalize() < epsilon)
        {
            PLOG_ERROR << "第一点和第二点太接近";
            return -1;
        }

        KDL::Vector axis_y{(axis_z * axis_x)};
        axis_y.Normalize();

        KDL::Frame center_(KDL::Rotation{axis_x, axis_y, axis_z}, center.p); // 确定圆心的方向

        //** 计算旋转角度 **//
        KDL::Vector f_p3_ = center_.Inverse() * f_p3.p;
        double theta13 = 0;

        if (f_p3_(1) < 0)
            theta13 = atan2(f_p3_(1), f_p3_(0)) + 2 * M_PI;
        else
            theta13 = atan2(f_p3_(1), f_p3_(0));
        std::cout << BLUE << "theta13= " << theta13 * 180 / M_PI << GREEN << std::endl;
        //**-------------------------------**//

        ::rocos::DoubleS doubleS;
        double path_length = (radius * theta13);
        doubleS.planDoubleSProfile(0, 0, 1, 0, 0, max_path_v / path_length, max_path_a / path_length, 2 * max_path_a / path_length);
        if (!doubleS.isValidMovement() || !(doubleS.getDuration() > 0))
        {
            PLOG_ERROR << "planDoubleSProfile failed";
            return -1;
        }
        double T_total = doubleS.getDuration();
        std::cout << BLUE << "T_total = " << T_total << GREEN << std::endl;

        KDL::Rotation R_w_p1 = f_p1.M;

        //** 轨迹规划 **//
        double dt{0.0};
        double doubleS_vel{0.0};
        double doubleS_pos{0.0};
        KDL::Twist interp_vel{};

        while (dt <= T_total)
        {
            doubleS_pos = doubleS.pos(dt);
            doubleS_vel = doubleS.vel(dt);

            KDL::Vector W_inter_pos = center_.M * KDL::Vector{radius * cos(doubleS_pos * theta13), radius * sin(doubleS_pos * theta13), 0};
            // !( axis_z * W_inter_pos )向量的模长 为 radius
            interp_vel.vel = doubleS_vel * theta13 * (axis_z * W_inter_pos);

            if (fixed_rotation)
            {
                interp_vel.rot = KDL::Vector{0, 0, 0};
            }
            else
            {
                interp_vel.rot = doubleS_vel * theta13 * axis_z;
            }

            traj_vel.push_back(interp_vel);

            dt += 0.001;
        }
        return 0;
    }

    int circle_trajectory(std::vector<KDL::Twist> &traj_vel, const KDL::Frame &f_p1, const KDL::Frame &center, double theta13, int axiz, double max_path_v, double max_path_a, bool fixed_rotation)
    {
        const double epsilon = 1e-6;

        if (abs(theta13) < epsilon)
        {
            PLOG_ERROR << "圆弧旋转角度太小";
            return -1;
        }
        else if (axiz > 2 || axiz < 0)
        {
            PLOG_ERROR << "圆弧旋转轴只能选择:【0-X、1-Y、2-Z】";
            return -1;
        }

        KDL::Vector W_p1_center = f_p1.p - center.p; // 待旋转的向量

        KDL::Vector center_p1 = center.Inverse() * f_p1.p;
        KDL::Vector rot_axiz{};
        KDL::Vector radius_axiz{}; // f_p1.p到真正圆点的向量

        switch (axiz)
        {
        case 0:
            rot_axiz = center.M.UnitX();
            radius_axiz = W_p1_center - rot_axiz * center_p1(0);
            break;
        case 1:
            rot_axiz = center.M.UnitY();
            radius_axiz = W_p1_center - rot_axiz * center_p1(1);
            break;
        case 2:
            rot_axiz = center.M.UnitZ();
            radius_axiz = W_p1_center - rot_axiz * center_p1(2);
            break;
        default:
            PLOG_ERROR << "圆弧旋转轴只能选择:【0-X、1-Y、2-Z】";
            return -1;
        }

        ::rocos::DoubleS doubleS;
        double path_length = radius_axiz.Norm() * abs(theta13);
        doubleS.planDoubleSProfile(0, 0, 1, 0, 0, max_path_v / path_length, max_path_a / path_length, 2 * max_path_a / path_length);
        if (!doubleS.isValidMovement() || !(doubleS.getDuration() > 0))
        {
            PLOG_ERROR << "planDoubleSProfile failed";
            return -1;
        }
        double T_total = doubleS.getDuration();

        //** 轨迹规划 **//
        double dt{0.0};
        double doubleS_vel{0.0};
        double doubleS_pos{0.0};
        KDL::Twist interp_vel{};

        while (dt <= T_total)
        {
            doubleS_pos = doubleS.pos(dt);
            doubleS_vel = doubleS.vel(dt);

            KDL::Vector inter_pos = KDL::Rotation::Rot2(rot_axiz, theta13 * doubleS_pos) * radius_axiz;
            // !( axis_z * inter_pos )向量的模长 为 radius
            interp_vel.vel = doubleS_vel * theta13 * (rot_axiz * inter_pos);

            if (fixed_rotation)
            {
                interp_vel.rot = KDL::Vector{0, 0, 0};
            }
            else
            {
                interp_vel.rot = doubleS_vel * theta13 * rot_axiz;
            }

            traj_vel.push_back(interp_vel);

            dt += 0.001;
        }

        return 0;
    }

    int rotation_trajectory(std::vector<KDL::Frame> &traj, const KDL::Vector &f_p, const KDL::Rotation &f_r1, const KDL::Rotation &f_r2, double max_path_v, double max_path_a, double equivalent_radius)
    {
        using namespace KDL;

        KDL::Rotation R_r1_r2 = f_r1.Inverse() * f_r2;
        KDL::Vector ration_axis;
        double angle = R_r1_r2.GetRotAngle(ration_axis);
        if (angle == 0)
        {
            std::cout << RED << "circle_trajectory(): given rotation parameters  cannot be the same   " << GREEN << std::endl;
            return -1;
        }

        ::rocos::DoubleS doubleS;
        double path_length = (equivalent_radius * angle);
        doubleS.planDoubleSProfile(0, 0, 1, 0, 0, max_path_v / path_length, max_path_a / path_length, 2 * max_path_a / path_length);
        if (!doubleS.isValidMovement() || !(doubleS.getDuration() > 0))
        {
            std::cout << RED << "circle_trajectory():planDoubleSProfile failed" << GREEN << std::endl;
            return -1;
        }
        double T_total = doubleS.getDuration();
        std::cout << BLUE << "T_total = " << T_total << GREEN << std::endl;

        //** 轨迹规划 **//
        double dt{0.0};
        double s{0.0};
        while (dt <= T_total)
        {
            s = doubleS.pos(dt);
            traj.push_back(KDL::Frame(f_r1 * KDL::Rotation::Rot2(ration_axis, angle * s), f_p));
            dt += 0.001;
        }
        return 0;
    }
#pragma endregion

#pragma region //*关节空间点动功能实现
    SmartServo_Joint::SmartServo_Joint(std::atomic<bool> *finished_flag_ptr)
    {
        joint_current.resize( jointNum );
        joint_target.resize( jointNum );
        joint_vel.resize( jointNum );
        joint_last_pos.resize( jointNum );
        joint_last_last_pos.resize( jointNum );
        init_joint.resize( jointNum );
        external_finished_flag_ptr = finished_flag_ptr;
    }

    //! init()只负责轨迹的信息重置，运行状态Flag由各运动线程结束后{手动重置}
    void SmartServo_Joint::init( rocos::Robot* robot_ptr, int flag, double target_vel, double max_vel, double max_acc, double max_jerk )
    {
        input.current_position[ 0 ]     = 0;
        input.current_velocity[ 0 ]     = 0;
        input.current_acceleration[ 0 ] = 0;

        if ( flag >= 600 )  // Runto情况-位置模型
        {
            RunTo_length      = 0;
            init_joint        = vector_2_JntArray( robot_ptr->pos_ );  // 需要记录开始位置
            auto RunTo_target = robot_ptr->get_RunTo_joint_target( flag - 600 );
            KDL::JntArray RunTo_joint_offset( jointNum );
            KDL::Subtract( init_joint, RunTo_target, RunTo_joint_offset );

            for ( int i = 0; i < jointNum; i++ ) RunTo_length = std::max( RunTo_length, std::abs( RunTo_joint_offset( i ) ) );

            input.target_position[ 0 ]     = 1;
            input.target_velocity[ 0 ]     = 0;
            input.target_acceleration[ 0 ] = 0;
            input.max_velocity[ 0 ]        = target_vel / RunTo_length;
            input.max_acceleration[ 0 ]    = max_vel / RunTo_length;
            input.max_jerk[ 0 ]            = max_acc / RunTo_length;
        }
        else  // 点动情况 -速度模式
        {
            input.target_position[ 0 ]     = target_vel;
            input.target_velocity[ 0 ]     = 0;
            input.target_acceleration[ 0 ] = 0;
            input.max_velocity[ 0 ]        = max_vel;
            input.max_acceleration[ 0 ]    = max_acc;
            input.max_jerk[ 0 ]            = max_jerk;
        }

        input.control_interface = ruckig::ControlInterface::Position;
        input.synchronization   = ruckig::Synchronization::None;

        flag_stop = false;

        KDL::SetToZero( joint_vel );

        for ( int i{ 0 }; i < jointNum; i++ )
        {
            joint_current( i )       = robot_ptr->pos_[ i ];
            joint_target( i )        = joint_current( i );
            joint_last_pos( i )      = joint_current( i );
            joint_last_last_pos( i ) = joint_current( i );
        }
        _joint_vel_index = 0;  // 0代表无关节

        PLOG_INFO << "关节空间点动初始化完成";
    }

    int SmartServo_Joint::update( KDL::JntArray& joint_vel, rocos::Robot* robot_ptr )
    {
        res = otg.update( input, output );
        if ( res != ruckig::Result::Working && res != ruckig::Result::Finished )
        {
            PLOG_ERROR << "OTG 计算失败";
            return -1;
        }

        const auto& res_vel = output.new_position;

        KDL::SetToZero( joint_vel );

        if ( abs( _joint_vel_index ) <= 6 )  // 点动
        {
            joint_vel( abs( _joint_vel_index ) - 1 ) = sign( _joint_vel_index ) * res_vel[ 0 ];
        }
        else  // Runto
        {
            KDL::JntArray interp_joint( jointNum );
            if ( joint_pos( init_joint, robot_ptr->get_RunTo_joint_target( _joint_vel_index - 600 ), res_vel[ 0 ], interp_joint ) < 0 )
            {
                PLOG_ERROR << "Runto目标点位不可达";
                return -1;
            }

            KDL::Subtract( interp_joint, joint_current, joint_vel );  // 这里求得是1ms的点位
            KDL::Divide( joint_vel, 0.001, joint_vel );               // 这里转为标准单位1s的关节速度
        }

        output.pass_to_input( input );

        if ( res == ruckig::Result::Working )
            return 1;
        else
            // PLOG_INFO << "完成！";
            return 0;
    }

    void SmartServo_Joint::RunSmartServo(rocos::Robot *robot_ptr)
    {
        //** 变量初始化 **//
        // std::unique_lock<std::mutex> input_lock(input_mutex, std::defer_lock);
        // ruckig::Result res;
        // int count{0};
        auto t_start = std::chrono::high_resolution_clock::now();
        auto t_stop = t_start;
        std::chrono::duration<double> duration{};

        int t_count = 0; // 时间计数
        int _tick_count{robot_ptr->tick_count};
        //**-------------------------------**//

        // 第一次启动需要等待command()
        while (*external_finished_flag_ptr)
        {
            ;  // 等待指令
        }

        while (true)
        {
            t_start = std::chrono::high_resolution_clock::now();

            //** 50ms进行一次心跳检查,紧急停止时不需要检查 **//
            if (!flag_stop) t_count++;

            if (t_count > 100)  // 50毫秒保持一次通信
            {
                t_count = 0;
                if (_tick_count != robot_ptr->tick_count)
                    _tick_count = robot_ptr->tick_count;
                else {
                    PLOG_WARNING << "点动指令时间间隔过长,停止";

                    joint_stop();  // 速度目标设置为0
                }
            }
            //**-------------------------------**//

            int res = update(joint_vel, robot_ptr);

            if (res < 0) // OTG的 error 状态
            {
                // 关节空间急停
                flag_stop = true;
                Joint_stop(robot_ptr, joint_current, joint_last_pos, joint_last_last_pos);
                sleep(2);
                break;
            }
            else  // working 或者finished状态
            {
                KDL::Multiply( joint_vel, servo_dt, joint_vel );
                KDL::Add( joint_current, joint_vel, joint_target );

                //** 速度和加速度保护 **//
                if (!flag_stop && check_vel_acc(joint_target, joint_current, joint_last_pos, 1, 5) < 0)
                {
                    //! 急停状态下不用速度检查，因为会和笛卡尔急停冲突（笛卡尔急停会使得关节加速度超大，必触发关节急停保护）
                    flag_stop = true;
                    Joint_stop(robot_ptr, joint_current, joint_last_pos, joint_last_last_pos); // 关节空间急停
                    PLOG_DEBUG<<"急停结束";
                    sleep(2);
                    break;
                }

                joint_last_last_pos = joint_last_pos;
                joint_last_pos = joint_current;
                joint_current = joint_target;
                //**-------------------------------**//

                //** 安全位置伺服,防止关节超限 **//
                safety_servo(robot_ptr, joint_target);
                //**-------------------------------**//

                if ( res == 0 && flag_stop )
                {
                    PLOG_INFO << "关节空间急停已完成";
                    break;
                }
            }
            t_stop   = std::chrono::high_resolution_clock::now( );
            duration = ( t_stop - t_start );
        }
        ( *external_finished_flag_ptr ) = true;                     // 这次smart servo已结束，等待下一次smart servo
                                                                    //        robot_ptr->is_running_motion    = false;  // 机械臂运动已结束，可以执行其他离线类运动
        robot_ptr->setRunState( rocos::Robot::RunState::Stopped );  // 机械臂运动已结束，可以执行其他离线类运动
    }

    void SmartServo_Joint::command( int joint_vel_index )
    {
        if ( _joint_vel_index == 0 ) _joint_vel_index = joint_vel_index;

        if ( !flag_stop )  // 如果需要紧急停止，那么就不允许在更改指令了
        {
            if ( _joint_vel_index != joint_vel_index )
            {
                PLOG_ERROR << "方向变换，停止！";
                joint_stop( );
            }
            else
                ( *external_finished_flag_ptr ) = false;
        }
        else
        {
            //            PLOG_DEBUG << "control is not allowed during emergency stop";
        }
    }

    void SmartServo_Joint::joint_stop(double max_vel, double max_acc, double max_jerk)
    {
        flag_stop = true;
        if (_joint_vel_index >= 600)  // Runto情况-位置模型
        {
            input.control_interface = ruckig::ControlInterface::Velocity;
            input.target_velocity[0] = 0;
            input.target_acceleration[0] = 0;
            input.max_acceleration[0] = max_vel/RunTo_length;
            input.max_jerk[0] = max_acc/RunTo_length;

        } else  // 点动情况
        {
            input.target_position[0] = 0;
            input.target_velocity[0] = 0;
            input.target_acceleration[0] = 0;

            input.max_velocity[0] = max_vel;
            input.max_acceleration[0] = max_acc;
            input.max_jerk[0] = max_jerk;
        }
    }
#pragma endregion

#pragma region //*笛卡尔空间点动功能实现
#if 0
    SmartServo_Cartesian::SmartServo_Cartesian( std::atomic< bool >* finished_flag_ptr )
    {
        external_finished_flag_ptr = finished_flag_ptr;
    }

    SmartServo_Cartesian::~SmartServo_Cartesian( )
    {
    }

    //! init()只负责轨迹的信息重置，运行状态Flag由各运动线程结束后{手动重置}
    void SmartServo_Cartesian::init( KDL::JntArray q_init, KDL::Frame p_init, double v_init, double a_init, double max_v, double max_a, double max_j )
    {
        _q_init          = q_init;
        target           = p_init;
        last_target      = target;
        last_last_target = last_target;
        _new_target      = target;

        traj_frame.clear( );
        traj_joint.clear( );

        FinishPlanningFrame = false;
        FinishRunPlanningIK = false;
        FinishRunMotion     = false;

        // if ( !thread_RunPlanningIK )
        //     thread_RunPlanningIK.reset( new boost::thread{ &SmartServo_Cartesian::RunSmartServo_Ik, this } );
        // if ( !thread_RunMotion )
        //     thread_RunMotion.reset( new boost::thread{ &SmartServo_Cartesian::RunSmartServo_Motion, this } );
        time_count = 300;

        PLOG_INFO << "smart servo init succesed";
    }

    void SmartServo_Cartesian::RunSmartServo_Plannig( )
    {
        //** 变量初始化 **//
        std::unique_lock< std::mutex > lock_traj_frame( mutex_traj_frame, std::defer_lock );  //不上锁
        std::unique_lock< std::mutex > lock_target( target_mutex, std::defer_lock );          //不上锁
        int _command_flag{ 0 };
        KDL::Frame path_p;
        double path_v{ 0 };
        double path_a{ 0 };
        int OnlineDoubleS_count{ 1 };
        bool flag_from_last_rotation{ false };
        double t_last_rotation{ 0.001 };              //上段姿态运行时刻
        double dt_link{ 0 };                          //当前段段姿态运行时刻
        constexpr double sleep_time{ 0.001 * 0.95 };  //最长睡眠时间为1ms,最高于这个值会导致运动不连续

        //**-------------------------------**//

        //第一次启动需要等待command()
        while ( *external_finished_flag_ptr )
            ;

        try
        {
            using namespace KDL;
            lock_target.lock( );

            while ( !on_stop_trajectory )
            {
                if ( command_flag != _command_flag )  //有新指令
                {
                    _command_flag = command_flag;

                    PLOG_DEBUG << "_command_flag = " << _command_flag;

                    //** 线程结束时，结束变量在此**//
                    if ( _command_flag == 0 )
                    {
                        FinishPlanningFrame = true;
                        // TODO 注意这个锁
                        lock_target.unlock( );
                        PLOG_INFO << "Plannig 结束";
                        break;
                    }
                    //**-------------------------------**//

                    else if ( _command_flag == 1 )  //启动过程直线规划
                    {
                        //** 直线参数设置 **//
                        KDL::Rotation R_stat_end = last_target.M.Inverse( ) * target.M;
                        KDL::Vector ration_axis;
                        double angle           = R_stat_end.GetRotAngle( ration_axis );
                        double path_length     = ( target.p - last_target.p ).Norm( );
                        double ratation_length = ( equivalent_radius * abs( angle ) );
                        double length          = std::max( path_length, ratation_length );
                        //测试发现，这两个即使参数一样，也会计算结果不一样
                        _OnlineDoubleS.calculate( 0, 1, 0, 0, 0, 0, max_path_v / length, max_path_a / length, max_path_a * 2 / length );
                        _rotaion_OnlineDoubleS.calculate( 0, 1, 0, 0, 0, 0, max_path_v / length, max_path_a / length, max_path_a * 2 / length );
                        //**-------------------------------**//

                        double s{ 0 };
                        double sd{ 0 };
                        double sdd{ 0 };
                        dt_link            = 0;
                        Frame _target      = target;
                        Frame _last_target = last_target;
                        double next_remain_length{ 0 };
                        OnlineDoubleS_count     = 1;
                        flag_from_last_rotation = false;
                        //** 直线段轨迹计算 **//
                        while ( 1 )
                        {
                            if ( on_stop_trajectory )
                            {
                                throw "on_stop_trajectory";
                            }

                            _rotaion_OnlineDoubleS.get_pos_vel_acc( OnlineDoubleS_count, s, sd, sdd );
                            if ( ( 1.0 - s ) > eps && _command_flag == command_flag )
                            {
                                path_v = path_length * sd;
                                path_a = path_length * sdd;
                                path_p = link_trajectory( _last_target, _target, s, s );
                                // double s_r;
                                // double sd_r;
                                // double sdd_r;
                                // _rotaion_OnlineDoubleS.get_pos_vel_acc( dt_link, s_r, sd_r, sdd_r );
                                // PLOG_DEBUG << "s_r =\n"
                                //            << s_r;
                                // PLOG_DEBUG << "s =\n"
                                //            << s;
                                // PLOG_DEBUG << "dt_link =\n"
                                //            << dt_link;
                                // PLOG_DEBUG << "OnlineDoubleS_count =\n"
                                //            << 0.001 * OnlineDoubleS_count;

                                lock_traj_frame.lock( );
                                traj_frame.push_back( path_p );
                                lock_traj_frame.unlock( );
                                dt_link = dt_link + 0.001;

                                OnlineDoubleS_count++;

                                lock_target.unlock( );
                                std::this_thread::sleep_for( std::chrono::duration< double >( sleep_time ) );
                                lock_target.lock( );

                                if ( _command_flag != command_flag )
                                {
                                    PLOG_DEBUG;
                                    //** 矫正直线过程中下一段新目标的参数 ，包括位置和姿态调整**//

                                    last_target.p      = _target.p;       //防止多次修改last_target
                                    last_last_target.p = _last_target.p;  //防止多次修改last_last_target

                                    last_target.M      = _target.M;
                                    last_last_target.M = _last_target.M;

                                    //**-------------------------------**//

                                    Vector ab = last_target.p - path_p.p;
                                    Vector bc = target.p - last_target.p;

                                    double cos_alpha = std::max( -1., std::min( dot( ab, bc ) / ab.Norm( ) / bc.Norm( ), 1. ) );

                                    if ( ( 1 - cos_alpha ) <= eps || bc.Norm( ) < eps )  //如果新目标为0度调用，那么立刻跳出
                                    {
                                        next_remain_length = 0;
                                        PLOG_DEBUG << "新目标0调用,立刻跳转";
                                    }
                                    else if ( ( cos_alpha - ( -1 ) ) <= eps )  //如果新目标为180度调用，那么立刻跳出
                                    {
                                        next_remain_length = 0;
                                        PLOG_DEBUG << "新目标180调用,立刻跳转";
                                    }
                                    else if ( ( next_remain_length = bc.Norm( ) ) < ab.Norm( ) )  //如果新目标太近就等待跳转，适用于过渡和0度平行情况
                                    {
                                        command_flag = _command_flag;
                                        PLOG_DEBUG << "新目标太近,不允许跳转";
                                    }
                                    else
                                    {
                                        next_remain_length = 0;
                                        PLOG_DEBUG << "新目标够远,立刻跳转";
                                    }
                                }

                                if ( next_remain_length > 0 )
                                {
                                    //** 矫正直线过程中下一段新目标的参数 ，包括位置和姿态调整**//

                                    //**-------------------------------**//
                                    if ( ( last_target.p - path_p.p ).Norm( ) < next_remain_length )
                                    {
                                        command_flag = _command_flag + 1;
                                        PLOG_DEBUG << "等待结束，可以跳出";
                                    }
                                }
                            }
                            else
                                break;
                        }
                        //**-------------------------------**//

                        if ( ( 1.0 - s ) <= eps )
                        {
                            command_flag = 0;
                            PLOG_DEBUG << "直线规划已完成";
                        }
                    }
                    else if ( _command_flag >= 2 )
                    {
#pragma region    //** 公共参数设置 **//

                        Vector p1{ path_p.p };
                        Vector p_mid{ last_target.p };
                        Vector p2{ target.p };
                        Vector ab            = p_mid - p1;
                        Vector bc            = p2 - p_mid;
                        double remain_length = ab.Norm( );
                        double abdist        = remain_length;
                        double bcdist        = bc.Norm( );
                        double cos_alpha     = std::max( -1., std::min( dot( ab, bc ) / abdist / bcdist, 1. ) );
                        double alpha         = acos( cos_alpha );
                        double radius        = 0;
                        PLOG_DEBUG;
#pragma endregion //**-------------------------------**//

                        //! 0度平行和只旋转目标 处理在此
                        if ( ( 1 - cos_alpha ) <= big_eps || bc.Norm( ) < 5 * big_eps )
                        {
                            PLOG_DEBUG;
                            alpha  = 0;
                            radius = 0;

#pragma region //**路径在线doubleS参数设置 **//
                            double length_circle_link{ ( target.p - path_p.p ).Norm( ) };

                            if ( length_circle_link <= eps )  //!两段直线都为虚直线，在后面会重新正确计算
                                _OnlineDoubleS.calculate( 0, 1, 0, 0, 0, 0, max_path_v, max_path_a, 2 * max_path_a );
                            else
                                _OnlineDoubleS.calculate( 0, 1, path_v / length_circle_link, 0, path_a / length_circle_link, 0, max_path_v / length_circle_link, max_path_a / length_circle_link, 2 * max_path_a / length_circle_link );

                            KDL::Rotation R_stat_end = last_target.M.Inverse( ) * target.M;
                            KDL::Vector ration_axis;
                            double angle = R_stat_end.GetRotAngle( ration_axis );  //新目标的旋转角度

#pragma endregion //**-------------------------------**//

#pragma region //**上段姿态在线doubleS参数设置 **//
                            double s_r{ 0 };
                            double sd_r{ 0 };
                            double sdd_r{ 0 };
                            KDL::Vector last_ration_axis;
                            double last_angle{ 0 };

                            double remain_angle{ ( path_p.M.Inverse( ) * last_target.M ).GetRotAngle( last_ration_axis ) };  //上一段的剩余角度
                            double radio_angle{ abs( remain_angle ) / ( abs( remain_angle ) + abs( angle ) ) };              //代表上段angle和本次angle的比重
                            if ( !( radio_angle >= 0 ) ) radio_angle = 0.5;                                                  // 0/0无效，这里为了规划成功，设置为0.5

                            if ( flag_from_last_rotation )
                            {
                                _last_rotaion_OnlineDoubleS.get_pos_vel_acc( t_last_rotation - 0.001, s_r, sd_r, sdd_r );
                            }
                            else
                            {
                                _rotaion_OnlineDoubleS.get_pos_vel_acc( dt_link - 0.001, s_r, sd_r, sdd_r );
                            }

                            if ( length_circle_link <= eps )  //!只旋转目标且不存在直线段了，不需要时间缩放
                            {
                                //使用last_last_target而不是path.M,是因为想保持最大速度和最大加速度和之前的一致,以及sd_r连续必需用last_angle
                                last_angle             = ( last_last_target.M.Inverse( ) * last_target.M ).GetRotAngle( last_ration_axis );
                                double ratation_length = ( equivalent_radius * abs( last_angle ) );
                                _last_rotaion_OnlineDoubleS.calculate( s_r, 1.0, sd_r, 0, sdd_r, 0, max_path_v / ratation_length, max_path_a / ratation_length, 2 * max_path_a / ratation_length );
                                PLOG_DEBUG << "不要时间缩放适配路径，独立规划";
                            }
                            else  //!还有直线段，正常处理
                            {
                                _last_rotaion_OnlineDoubleS.calculate( s_r, 1, sd_r, 0, sdd_r, 0, 1e7, 1e7, 1e7, _OnlineDoubleS.get_duration( ) * radio_angle );
                                PLOG_DEBUG << "时间缩放（适配路径）";
                            }

                            double last_ratation_duration{ abs( remain_angle ) < eps ? 0 : _last_rotaion_OnlineDoubleS.get_duration( ) };

                            // PLOG_ERROR << "last_ratation_duration =" << last_ratation_duration;
                            double var1;
                            double var2;
                            double var3;
                            _last_rotaion_OnlineDoubleS.get_pos_vel_acc( last_ratation_duration, var1, var2, var3 );

                            // PLOG_ERROR << "last s_r at end time  = " << var1;
                            // PLOG_ERROR << "last sd_r at end time  = " << var2;
                            // PLOG_ERROR << "last sdd_r at end time  = " << var3;

#pragma endregion //**-------------------------------**//

#pragma region //**本段姿态在线doubleS参数设置 **//

                            bool flag_is_same_axis{ false };

                            if ( length_circle_link <= eps )  //!只旋转目标且不存在直线段了，不需要时间缩放
                            {
                                //上次和本次旋转的轴一致，则只规划一次就够,单次最大旋转角度180度
                                if ( KDL::sign( angle ) == KDL::sign( remain_angle ) && ( angle + remain_angle ) < M_PI && abs( last_ration_axis( 0 ) - ration_axis( 0 ) ) < eps && abs( last_ration_axis( 1 ) - ration_axis( 1 ) ) < eps && abs( last_ration_axis( 2 ) - ration_axis( 2 ) ) < eps )
                                {
                                    PLOG_DEBUG << last_ration_axis( 2 );
                                    PLOG_DEBUG << ration_axis( 2 );

                                    PLOG_DEBUG << "两段姿态的旋转轴相同，统一规划";
                                    PLOG_DEBUG << "s_r = " << s_r;
                                    angle = angle + remain_angle;

                                    last_ratation_duration = 0;
                                    double ratation_length = equivalent_radius * abs( angle );
                                    //!注意 s_r 、sd_r、sdd_r不得瞎修改，这里是代表上次姿态的信息
                                    //取回真实速度和加速度，在换算到新的{ratation_length}下
                                    sd_r  = last_angle * equivalent_radius * sd_r / ratation_length;
                                    sdd_r = last_angle * equivalent_radius * sdd_r / ratation_length;
                                    _rotaion_OnlineDoubleS.calculate( 0, 1.0, sd_r, 0, sdd_r, 0, max_path_v / ratation_length, max_path_a / ratation_length, 2 * max_path_a / ratation_length );
                                    flag_is_same_axis = true;
                                }
                                else
                                {
                                    double ratation_length = equivalent_radius * abs( angle );
                                    _rotaion_OnlineDoubleS.calculate( 0, 1, 0, 0, 0, 0, max_path_v / ratation_length, max_path_a / ratation_length, 2 * max_path_a / ratation_length );
                                }
                            }
                            else
                                _rotaion_OnlineDoubleS.calculate( 0, 1, 0, 0, 0, 0, 1e7, 1e7, 1e7, _OnlineDoubleS.get_duration( ) * ( 1 - radio_angle ) );

                            double ratation_duration{ abs( angle ) < eps ? 0 : _rotaion_OnlineDoubleS.get_duration( ) };
                            _rotaion_OnlineDoubleS.get_pos_vel_acc( ratation_duration, s_r, sd_r, sdd_r );
                            // PLOG_ERROR << "ratation_duration  = " << ratation_duration;
                            // PLOG_ERROR << "s_r at end time  = " << s_r;
                            // PLOG_ERROR << "sd_r at end time  = " << sd_r;
                            // PLOG_ERROR << "sdd_r at end time  = " << sdd_r;

#pragma endregion //**-------------------------------**//

                            //两段虚直线时，把路劲时间限制为两段姿态的时间，那么0>1时，所用时间也正好时总姿态时间,并且截取到后两位
                            //截取的原因：_OnlineDoubleS对时间计算不精确，导致s_p=1时，略超过了预定的时间
                            //当前这个修改只在两段虚直线才会做
                            //!这个修改放弃了，目前采用时间区间判断
                            // if ( length_circle_link <= eps )
                            //     _OnlineDoubleS.calculate( 0, 1, 0, 0, 0, 0, 1e7, 1e7, 1e7, static_cast< double >( static_cast< int >( ( last_ratation_duration + ratation_duration ) * 100 ) ) / 100 );

                            double s_p{ 0 };
                            double sd_p{ 0 };
                            double sdd_p{ 0 };
                            OnlineDoubleS_count     = 1;
                            Frame _target           = target;
                            Frame _last_target      = last_target;
                            Frame _last_last_target = last_last_target;
                            Frame _path_p           = path_p;
                            t_last_rotation         = 0.001;
                            dt_link                 = 0.001;
                            double next_remain_length{ 0 };

                            // PLOG_DEBUG << "target =\n"
                            //            << target;
                            // PLOG_DEBUG << "last_target =\n"
                            //            << last_target;
                            // PLOG_DEBUG << "last_last_target =\n"
                            //            << last_last_target;

                            if ( length_circle_link <= eps )  //两段虚直线的情况
                            {
                                while ( 1 )  //第一段和第二段直线轨迹
                                {
                                    if ( on_stop_trajectory ) throw "on_stop_trajectory";

                                    _OnlineDoubleS.get_pos_vel_acc( OnlineDoubleS_count, s_p, sd_p, sdd_p );

                                    if ( dt_link <= ratation_duration && _command_flag == command_flag )
                                    {
                                        path_v = length_circle_link * sd_p;
                                        path_a = length_circle_link * sdd_p;
                                        path_p = link_trajectory( _path_p, _target, s_p );

                                        if ( t_last_rotation < last_ratation_duration )
                                        {
                                            _last_rotaion_OnlineDoubleS.get_pos_vel_acc( t_last_rotation, s_r, sd_r, sdd_r );

                                            if ( s_r > 1 )
                                            {
                                                PLOG_ERROR << "last_ratation_duration =" << last_ratation_duration;
                                                PLOG_ERROR << "t_last_rotation " << t_last_rotation;
                                                _last_rotaion_OnlineDoubleS.get_pos_vel_acc( t_last_rotation / 2, s_r, sd_r, sdd_r );
                                                PLOG_ERROR << "s_r at  t_last_rotation/2=  " << s_r;
                                                _last_rotaion_OnlineDoubleS.get_pos_vel_acc( last_ratation_duration, s_r, sd_r, sdd_r );
                                                PLOG_ERROR << "s_r at end  =  " << s_r;
                                                PLOG_ERROR << "sd_r at end time  = " << sd_r;
                                                PLOG_ERROR << "sdd_r at end time  = " << sdd_r;
                                                _last_rotaion_OnlineDoubleS.get_pos_vel_acc( t_last_rotation, s_r, sd_r, sdd_r );
                                            }

                                            path_p.M = ratation_trajectory( _last_last_target.M, _last_target.M, s_r );
                                            t_last_rotation += 0.001;
                                            flag_from_last_rotation = true;
                                        }
                                        else if ( dt_link <= ratation_duration )
                                        {
                                            _rotaion_OnlineDoubleS.get_pos_vel_acc( dt_link, s_r, sd_r, sdd_r );

                                            if ( s_r > 1 )
                                            {
                                                PLOG_ERROR;
                                                PLOG_ERROR << "dt_link = " << dt_link;
                                                PLOG_ERROR << "ratation_duration = " << ratation_duration;
                                                PLOG_ERROR << "s_r  = " << s_r;

                                                _rotaion_OnlineDoubleS.get_pos_vel_acc( ratation_duration, s_r, sd_r, sdd_r );
                                                PLOG_ERROR << "s_r at end time  = " << s_r;
                                                PLOG_ERROR << "sd_r at end time  = " << sd_r;
                                                PLOG_ERROR << "sdd_r at end time  = " << sdd_r;
                                                _rotaion_OnlineDoubleS.get_pos_vel_acc( dt_link, s_r, sd_r, sdd_r );
                                            }

                                            if ( flag_is_same_axis )
                                                path_p.M = ratation_trajectory( _path_p.M, _target.M, s_r );
                                            else
                                                path_p.M = ratation_trajectory( _last_target.M, _target.M, s_r );

                                            dt_link += 0.001;
                                            flag_from_last_rotation = false;
                                        }
                                        // PLOG_DEBUG << "dt_link =" << dt_link;
                                        // PLOG_DEBUG << "ratation_duration =" << ratation_duration;

                                        lock_traj_frame.lock( );
                                        traj_frame.push_back( path_p );
                                        lock_traj_frame.unlock( );

                                        OnlineDoubleS_count++;

                                        lock_target.unlock( );
                                        std::this_thread::sleep_for( std::chrono::duration< double >( sleep_time ) );
                                        lock_target.lock( );  //直接锁

                                        //** 检查新目标是否有足够的距离过渡，没有则以target-last_target距离定为过渡半径**//
                                        if ( _command_flag != command_flag )
                                        {
                                            PLOG_DEBUG << "第一段和第二段直线过程设置新目标";

                                            //** 矫正第一段直线过程中下一段新目标的参数 ，包括位置调整和姿态调整**//

                                            last_target.p      = _target.p;  //防止多次修改last_target
                                            last_last_target.p = path_p.p;

                                            if ( t_last_rotation <= last_ratation_duration )
                                            {
                                                last_target.M      = _last_target.M;
                                                last_last_target.M = _last_last_target.M;
                                                PLOG_DEBUG << "旧的姿态段";
                                            }
                                            else if ( dt_link <= ratation_duration )
                                            {
                                                last_target.M      = _target.M;
                                                last_last_target.M = flag_is_same_axis ? _path_p.M : _last_target.M;
                                                PLOG_DEBUG << "新的姿态段";
                                            }
                                            //**-------------------------------**//

                                            Vector ab = last_target.p - path_p.p;
                                            Vector bc = target.p - last_target.p;

                                            double cos_alpha = std::max( -1., std::min( dot( ab, bc ) / ab.Norm( ) / bc.Norm( ), 1. ) );

                                            if ( ( 1 - cos_alpha ) <= eps || bc.Norm( ) <= eps )  //新目标为0度平行或者只旋转，那么立刻跳出
                                            {
                                                next_remain_length = 0;
                                                PLOG_DEBUG << "新目标0调用,立刻跳转";
                                            }
                                            else if ( ( cos_alpha - ( -1 ) ) <= eps )  //如果新目标为180度调用，那么立刻跳出
                                            {
                                                next_remain_length = 0;
                                                PLOG_DEBUG << "新目标180调用,立刻跳转";
                                            }
                                            else if ( ( next_remain_length = bc.Norm( ) ) < ab.Norm( ) )  //如果新目标太近就等待跳转，适用于过渡和0度平行情况
                                            {
                                                command_flag = _command_flag;
                                                PLOG_DEBUG << "新目标太近,不允许跳转";
                                            }
                                            else  //需要圆弧过渡且下段距离足够，立刻退出
                                            {
                                                next_remain_length = 0;
                                                PLOG_DEBUG << "新目标足够远,立刻跳转";
                                            }
                                        }

                                        if ( next_remain_length > 0 )
                                        {
                                            //** 矫正第一段直线和第二段直线过程中下一段新目标的参数 ，包括姿态调整**//
                                            if ( t_last_rotation <= last_ratation_duration )
                                            {
                                                last_last_target.M = _last_last_target.M;
                                                last_target.M      = _last_target.M;
                                            }
                                            else if ( dt_link <= ratation_duration )
                                            {
                                                last_last_target.M = _last_target.M;
                                                last_target.M      = _target.M;
                                            }
                                            //**-------------------------------**//

                                            if ( ( last_target.p - path_p.p ).Norm( ) < next_remain_length )
                                            {
                                                command_flag = _command_flag + 1;
                                                PLOG_DEBUG << "等待结束，可以跳出";
                                            }
                                        }
                                        //**-------------------------------**//
                                    }
                                    else
                                        break;
                                }

                                if ( dt_link > ratation_duration )
                                {
                                    command_flag = 0;
                                    PLOG_DEBUG << "多段直线规划已完成";
                                }
                            }
                            else  //其余正常情况
                            {
                                while ( 1 )  //第一段和第二段直线轨迹
                                {
                                    if ( on_stop_trajectory ) throw "on_stop_trajectory";

                                    _OnlineDoubleS.get_pos_vel_acc( OnlineDoubleS_count, s_p, sd_p, sdd_p );

                                    if ( ( 1.0 - s_p ) > eps && _command_flag == command_flag )
                                    {
                                        path_v = length_circle_link * sd_p;
                                        path_a = length_circle_link * sdd_p;
                                        path_p = link_trajectory( _path_p, _target, s_p );

                                        if ( t_last_rotation <= last_ratation_duration )
                                        {
                                            _last_rotaion_OnlineDoubleS.get_pos_vel_acc( t_last_rotation, s_r, sd_r, sdd_r );
                                            path_p.M = ratation_trajectory( _last_last_target.M, _last_target.M, s_r );
                                            t_last_rotation += 0.001;
                                            flag_from_last_rotation = true;
                                        }
                                        else if ( dt_link <= ratation_duration )
                                        {
                                            _rotaion_OnlineDoubleS.get_pos_vel_acc( dt_link, s_r, sd_r, sdd_r );
                                            path_p.M = ratation_trajectory( _last_target.M, _target.M, s_r );
                                            dt_link += 0.001;
                                            flag_from_last_rotation = false;
                                        }
                                        // PLOG_DEBUG << "dt_link =" << dt_link;
                                        // PLOG_DEBUG << "ratation_duration =" << ratation_duration;

                                        lock_traj_frame.lock( );
                                        traj_frame.push_back( path_p );
                                        lock_traj_frame.unlock( );

                                        OnlineDoubleS_count++;

                                        lock_target.unlock( );
                                        std::this_thread::sleep_for( std::chrono::duration< double >( sleep_time ) );
                                        lock_target.lock( );  //直接锁

                                        //** 检查新目标是否有足够的距离过渡，没有则以target-last_target距离定为过渡半径**//
                                        if ( _command_flag != command_flag )
                                        {
                                            PLOG_DEBUG << "第一段和第二段直线过程设置新目标";

                                            //** 矫正第一段直线过程中下一段新目标的参数 ，包括位置调整和姿态调整**//

                                            last_target.p      = _target.p;  //防止多次修改last_target
                                            last_last_target.p = path_p.p;

                                            if ( t_last_rotation <= last_ratation_duration )
                                            {
                                                last_target.M      = _last_target.M;
                                                last_last_target.M = _last_last_target.M;
                                                PLOG_DEBUG << "旧的姿态段";
                                            }
                                            else if ( dt_link <= ratation_duration )
                                            {
                                                last_target.M      = _target.M;
                                                last_last_target.M = _last_target.M;
                                            }
                                            //**-------------------------------**//

                                            Vector ab = last_target.p - path_p.p;
                                            Vector bc = target.p - last_target.p;

                                            double cos_alpha = std::max( -1., std::min( dot( ab, bc ) / ab.Norm( ) / bc.Norm( ), 1. ) );

                                            if ( ( 1 - cos_alpha ) <= eps || bc.Norm( ) <= eps )  //新目标为0度平行或者只旋转，那么立刻跳出
                                            {
                                                next_remain_length = 0;
                                                PLOG_DEBUG << "新目标0调用,立刻跳转";
                                            }
                                            else if ( ( cos_alpha - ( -1 ) ) <= eps )  //如果新目标为180度调用，那么立刻跳出
                                            {
                                                next_remain_length = 0;
                                                PLOG_DEBUG << "新目标180调用,立刻跳转";
                                            }
                                            else if ( ( next_remain_length = bc.Norm( ) ) < ab.Norm( ) )  //如果新目标太近就等待跳转，适用于过渡和0度平行情况
                                            {
                                                command_flag = _command_flag;
                                                PLOG_DEBUG << "新目标太近,不允许跳转";
                                            }
                                            else  //需要圆弧过渡且下段距离足够，立刻退出
                                            {
                                                next_remain_length = 0;
                                                PLOG_DEBUG << "新目标足够远,立刻跳转";
                                            }
                                        }

                                        if ( next_remain_length > 0 )
                                        {
                                            //** 矫正第一段直线和第二段直线过程中下一段新目标的参数 ，包括姿态调整**//
                                            if ( t_last_rotation <= last_ratation_duration )
                                            {
                                                last_last_target.M = _last_last_target.M;
                                                last_target.M      = _last_target.M;
                                            }
                                            else if ( dt_link <= ratation_duration )
                                            {
                                                last_last_target.M = _last_target.M;
                                                last_target.M      = _target.M;
                                            }
                                            //**-------------------------------**//

                                            if ( ( last_target.p - path_p.p ).Norm( ) < next_remain_length )
                                            {
                                                command_flag = _command_flag + 1;
                                                PLOG_DEBUG << "等待结束，可以跳出";
                                            }
                                        }
                                        //**-------------------------------**//
                                    }
                                    else
                                        break;
                                }

                                if ( ( 1.0 - s_p ) <= eps )
                                {
                                    command_flag = 0;
                                    PLOG_DEBUG << "多段直线规划已完成";
                                }
                            }
                        }

                        //! 180度掉头处理
                        else if ( ( cos_alpha - ( -1 ) ) <= big_eps )
                        {
                            PLOG_DEBUG;
                            alpha  = M_PI;
                            radius = 0;

#pragma region //**路径在线doubleS参数设置 **//

                            double s_p{ 0 };
                            double sd_p{ 0 };
                            double sdd_p{ 0 };

                            double length_circle_link{ ( last_target.p - path_p.p ).Norm( ) };
                            _OnlineDoubleS.calculate( 0, 1, path_v / length_circle_link, 0, path_a / length_circle_link, 0, max_path_v / length_circle_link, max_path_a / length_circle_link, 2 * max_path_a / length_circle_link );

#pragma endregion //**-------------------------------**//

#pragma region //**姿态在线doubleS参数设置 **//
                            double s_r{ 0 };
                            double sd_r{ 0 };
                            double sdd_r{ 0 };

                            if ( flag_from_last_rotation )
                                _last_rotaion_OnlineDoubleS.get_pos_vel_acc( t_last_rotation - 0.001, s_r, sd_r, sdd_r );
                            else
                                _rotaion_OnlineDoubleS.get_pos_vel_acc( dt_link - 0.001, s_r, sd_r, sdd_r );

                            _last_rotaion_OnlineDoubleS.calculate( s_r, 1, sd_r, 0, sdd_r, 0, 1e7, 1e7, 1e7, _OnlineDoubleS.get_duration( ) );
                            double last_ratation_duration{ _last_rotaion_OnlineDoubleS.get_duration( ) };

#pragma endregion //**-------------------------------**//

                            PLOG_DEBUG;

                            Frame _target           = target;
                            Frame _last_target      = last_target;
                            Frame _last_last_target = last_last_target;
                            Frame _path_p           = path_p;
                            OnlineDoubleS_count     = 1;
                            t_last_rotation         = 0.001;
                            double next_remain_length{ 0 };

                            while ( 1 )
                            {
                                if ( on_stop_trajectory ) throw "on_stop_trajectory";

                                _OnlineDoubleS.get_pos_vel_acc( OnlineDoubleS_count, s_p, sd_p, sdd_p );

                                if ( ( 1.0 - s_p ) > eps && _command_flag == command_flag )
                                {
                                    path_v = length_circle_link * sd_p;
                                    path_a = length_circle_link * sdd_p;
                                    path_p = link_trajectory( _path_p, _last_target, s_p );

                                    if ( t_last_rotation <= last_ratation_duration )
                                    {
                                        _last_rotaion_OnlineDoubleS.get_pos_vel_acc( t_last_rotation, s_r, sd_r, sdd_r );
                                        path_p.M = ratation_trajectory( _last_last_target.M, _last_target.M, s_r );
                                        t_last_rotation += 0.001;
                                        flag_from_last_rotation = true;
                                    }

                                    lock_traj_frame.lock( );
                                    traj_frame.push_back( path_p );
                                    lock_traj_frame.unlock( );
                                    OnlineDoubleS_count++;

                                    lock_target.unlock( );
                                    std::this_thread::sleep_for( std::chrono::duration< double >( sleep_time ) );
                                    lock_target.lock( );

                                    //** 180度掉头准备过程中，设置了新目标**//
                                    if ( _command_flag != command_flag )
                                    {
                                        PLOG_DEBUG << "180度掉头中设置新目标";

                                        //** 矫正第一段直线过程中下一段新目标的参数 ，包括位置调整和姿态调整**//

                                        last_target.p      = _last_target.p;  //防止多次修改last_target
                                        last_last_target.p = path_p.p;        //防止多次修改last_last_target

                                        if ( t_last_rotation <= last_ratation_duration )  // 180度掉头时，没有姿态缩放，只会执行旧姿态段
                                        {
                                            last_target.M      = _last_target.M;
                                            last_last_target.M = _last_last_target.M;
                                        }

                                        //**-------------------------------**//

                                        Vector ab = last_target.p - path_p.p;
                                        Vector bc = target.p - last_target.p;

                                        double cos_alpha = std::max( -1., std::min( dot( ab, bc ) / ab.Norm( ) / bc.Norm( ), 1. ) );

                                        if ( ( 1 - cos_alpha ) <= eps || bc.Norm( ) <= eps )  //如果新目标为0度调用，那么立刻跳出
                                        {
                                            next_remain_length = 0;
                                            PLOG_DEBUG << "新目标0调用,立刻跳转";
                                        }
                                        else if ( ( cos_alpha - ( -1 ) ) <= eps )  //如果新目标为180度调用，那么立刻跳出
                                        {
                                            next_remain_length = 0;
                                            PLOG_DEBUG << "新目标180调用,立刻跳转";
                                        }
                                        else if ( ( next_remain_length = bc.Norm( ) ) < ab.Norm( ) )  //如果新目标太近就等待跳转，适用于过渡和0度平行情况
                                        {
                                            command_flag = _command_flag;
                                            PLOG_DEBUG << "新目标太近,不允许跳转";
                                        }
                                        else
                                        {
                                            next_remain_length = 0;
                                            PLOG_DEBUG << "新目标足够远,立刻跳转";
                                        }
                                    }

                                    if ( next_remain_length > 0 )
                                    {
                                        //** 矫正第一段直线和第二段直线过程中下一段新目标的参数 ，包括姿态调整**//

                                        //**-------------------------------**//

                                        if ( ( last_target.p - path_p.p ).Norm( ) < next_remain_length )
                                        {
                                            command_flag = _command_flag + 1;
                                            PLOG_DEBUG << "等待结束，可以跳出";
                                        }
                                    }
                                }
                                else
                                    break;
                            }

                            //!只有第一段直线消耗完，才应该进入 command_flag = 1，打断情况按正常处理
                            if ( abs( 1.0 - s_p ) <= eps )
                            {
                                command_flag = 1;
                                PLOG_DEBUG << "180掉头规划，准备已完成，进入commnd_flag=1";
                            }
                        }

                        //!两段直线有夹角，则需要圆弧过渡
                        else
                        {
                            PLOG_DEBUG;
#pragma region    //** 圆弧参数设置 **//

                            radius = remain_length * tan( ( M_PI - alpha ) / 2 );

                            //求解过渡半径占下一段直线的百分比
                            double s_bound_dist = remain_length / bcdist;
                            PLOG_DEBUG << "remain_length = " << remain_length;
                            PLOG_DEBUG << "bcdist = " << bcdist;
                            PLOG_DEBUG << "s_bound_dist = " << s_bound_dist;

                            Frame F_base_circlestart = Frame{ last_last_target.M, path_p.p };                 //注意，这里的旋转矩阵为last_last_target
                            Frame F_base_circleend   = link_trajectory( last_target, target, s_bound_dist );  //注意，这里的旋转矩阵为last_target
                            F_base_circleend.M       = last_target.M;

                            Vector V_base_t = ab * ( ab * bc );  //圆弧中垂直于第一段直线的半径向量
                            V_base_t.Normalize( );
                            Frame F_base_circleCenter{ F_base_circlestart.p - V_base_t * radius };

                            KDL::Vector tmpv = F_base_circleend.p - F_base_circleCenter.p;  //圆弧中垂直于第二段直线的半径向量
                            tmpv.Normalize( );

                            Vector z( V_base_t * tmpv );  // Z轴
                            double n = z.Normalize( );
                            if ( n < epsilon )
                            {
                                PLOG_ERROR << " Z Axis Calculation error";
                                // assert( 0 );
                                on_stop_trajectory = true;
                                throw "Z Axis Calculation error";
                            }
                            F_base_circleCenter.M = KDL::Rotation{ V_base_t, ( z * V_base_t ), z };
#pragma endregion //**-------------------------------**//

#pragma region //**路径在线doubleS参数设置 **//
                            double length_circle_link{ ( ( target.p - F_base_circleend.p ).Norm( ) < eps ? 0 : ( target.p - F_base_circleend.p ).Norm( ) ) + alpha * radius };
                            double bound_length = alpha * radius / length_circle_link;  //圆弧段长占总长百分比
                            _OnlineDoubleS.calculate( 0, 1, path_v / length_circle_link, 0, path_a / length_circle_link, 0, max_path_v / length_circle_link, max_path_a / length_circle_link, 2 * max_path_a / length_circle_link );

#pragma endregion //**-------------------------------**//

#pragma region    //** 直线段参数设置 **//

                            KDL::Rotation R_stat_end = last_target.M.Inverse( ) * target.M;
                            KDL::Vector ration_axis;
                            double angle           = R_stat_end.GetRotAngle( ration_axis );
                            double ratation_length = ( equivalent_radius * abs( angle ) );
                            double path_length     = ( target.p - F_base_circleend.p ).Norm( );
                            double length          = std::max( ratation_length, path_length );
#pragma endregion //**-------------------------------**//

#pragma region //**上段姿态在线doubleS参数设置 **//
                            double s_r{ 0 };
                            double sd_r{ 0 };
                            double sdd_r{ 0 };
                            double remain_angle{ ( path_p.M.Inverse( ) * last_target.M ).GetRotAngle( ration_axis ) };
                            double radio_angle{ abs( remain_angle / ( remain_angle + angle ) ) };  //代表上段angle和本次angle的比重
                            if ( !( radio_angle >= 0 ) ) radio_angle = 0.5;                        // 0/0无效，这里为了规划成功，设置为0.5

                            if ( flag_from_last_rotation )
                                _last_rotaion_OnlineDoubleS.get_pos_vel_acc( t_last_rotation - 0.001, s_r, sd_r, sdd_r );
                            else
                                _rotaion_OnlineDoubleS.get_pos_vel_acc( dt_link - 0.001, s_r, sd_r, sdd_r );

                            _last_rotaion_OnlineDoubleS.calculate( s_r, 1, sd_r, 0, sdd_r, 0, 1e7, 1e7, 1e7, _OnlineDoubleS.get_duration( ) * radio_angle );
                            double last_ratation_duration{ abs( remain_angle ) < eps ? 0 : _last_rotaion_OnlineDoubleS.get_duration( ) };

#pragma endregion //**-------------------------------**//

#pragma region //**本段姿态在线doubleS参数设置 **//

                            _rotaion_OnlineDoubleS.calculate( 0, 1, 0, 0, 0, 0, 1e7, 1e7, 1e7, _OnlineDoubleS.get_duration( ) * ( 1 - radio_angle ) );
                            double ratation_duration{ abs( angle ) < eps ? 0 : _rotaion_OnlineDoubleS.get_duration( ) };

#pragma endregion //**-------------------------------**//

                            double s_p{ 0 };
                            double sd_p{ 0 };
                            double sdd_p{ 0 };
                            OnlineDoubleS_count = 1;
                            t_last_rotation     = 0.001;
                            dt_link             = 0.001;

                            Frame _target           = target;
                            Frame _last_target      = last_target;
                            Frame _last_last_target = last_last_target;
                            double next_remain_length{ 0 };

                            while ( 1 )  //圆弧段计算
                            {
                                if ( on_stop_trajectory ) throw "on_stop_trajectory";

                                _OnlineDoubleS.get_pos_vel_acc( OnlineDoubleS_count, s_p, sd_p, sdd_p );

                                if ( s_p < bound_length && _command_flag == command_flag )
                                {
                                    path_v = length_circle_link * sd_p;
                                    path_a = length_circle_link * sdd_p;
                                    path_p = cirlular_trajectory( F_base_circlestart, F_base_circleend, F_base_circleCenter, s_p / bound_length, alpha );

                                    if ( t_last_rotation <= last_ratation_duration )
                                    {
                                        _last_rotaion_OnlineDoubleS.get_pos_vel_acc( t_last_rotation, s_r, sd_r, sdd_r );
                                        path_p.M = ratation_trajectory( _last_last_target.M, _last_target.M, s_r );
                                        t_last_rotation += 0.001;
                                        flag_from_last_rotation = true;
                                    }
                                    else if ( dt_link <= ratation_duration )
                                    {
                                        _rotaion_OnlineDoubleS.get_pos_vel_acc( dt_link, s_r, sd_r, sdd_r );
                                        path_p.M = ratation_trajectory( _last_target.M, _target.M, s_r );
                                        dt_link += 0.001;
                                        flag_from_last_rotation = false;
                                    }

                                    lock_traj_frame.lock( );
                                    traj_frame.push_back( path_p );
                                    lock_traj_frame.unlock( );

                                    OnlineDoubleS_count++;
                                    lock_target.unlock( );

                                    std::this_thread::sleep_for( std::chrono::duration< double >( sleep_time ) );
                                    lock_target.lock( );

                                    //** 检查新目标是否有足够的距离过渡，没有则以target-last_target距离定为过渡半径**//
                                    if ( _command_flag != command_flag )
                                    {
                                        PLOG_DEBUG << "圆弧过程设置新目标";

                                        //** 矫正圆弧过程中下一段新目标的参数 ，包括位置调整和姿态调整**//
                                        KDL::Vector tem_r = path_p.p - F_base_circleCenter.p;  //求解半径向量
                                        Vector tem_v( z * tem_r );                             //当前速度方向
                                        tem_v.Normalize( );
                                        double finished_cos_alpha = std::max( -1., std::min( dot( V_base_t, tem_r ) / V_base_t.Norm( ) / tem_r.Norm( ), 1. ) );
                                        double finishe_alpha      = acos( finished_cos_alpha );                               //求解已转角度
                                        tem_v                     = tem_v * ( length_circle_link - finishe_alpha * radius );  //求解已走弧度，然后算出当前速度向量上停止的位置

                                        last_target.p      = path_p.p + tem_v;
                                        last_last_target.p = path_p.p;  //! last_last_target.p其实不重要，后续都没用到，M却很重要

                                        if ( t_last_rotation <= last_ratation_duration )
                                        {
                                            last_last_target.M = _last_last_target.M;
                                            last_target.M      = _last_target.M;
                                        }
                                        else if ( dt_link <= ratation_duration )
                                        {
                                            last_last_target.M = _last_target.M;
                                            last_target.M      = _target.M;
                                        }
                                        //**-------------------------------**//

                                        Vector ab = last_target.p - path_p.p;
                                        Vector bc = target.p - last_target.p;

                                        double cos_alpha = std::max( -1., std::min( dot( ab, bc ) / ab.Norm( ) / bc.Norm( ), 1. ) );

                                        if ( ( 1 - cos_alpha ) <= eps )  //如果新目标为0度调用，那么立刻跳出
                                        {
                                            next_remain_length = 0;
                                            PLOG_DEBUG << "新目标0调用,立刻跳转";
                                        }
                                        else if ( ( cos_alpha - ( -1 ) ) <= eps )  //如果新目标为180度调用，那么立刻跳出
                                        {
                                            next_remain_length = 0;
                                            PLOG_DEBUG << "新目标180调用,立刻跳转";
                                        }
                                        else if ( ( next_remain_length = bc.Norm( ) ) < ab.Norm( ) )  //如果新目标太近就等待跳转，适用于过渡和0度平行情况
                                        {
                                            command_flag = _command_flag;
                                            PLOG_DEBUG << "新目标太近,不允许跳转";
                                        }
                                        else
                                        {
                                            next_remain_length = 0;
                                            PLOG_DEBUG << "新目标足够远,立刻跳转";
                                        }
                                    }

                                    if ( next_remain_length > 0 )
                                    {
                                        //** 矫正圆弧过程中下一段新目标的参数 ，包括位置调整和姿态调整**//
                                        Vector tem_r = path_p.p - F_base_circleCenter.p;  //求解半径向量
                                        Vector tem_v( z * tem_r );                        //当前速度方向
                                        tem_v.Normalize( );
                                        double finished_cos_alpha = std::max( -1., std::min( dot( V_base_t, tem_r ) / V_base_t.Norm( ) / tem_r.Norm( ), 1. ) );
                                        double finishe_alpha      = acos( finished_cos_alpha );                               //求解已转角度
                                        tem_v                     = tem_v * ( length_circle_link - finishe_alpha * radius );  //求解已走弧度，然后算出当前速度向量上停止的位置

                                        last_target.p      = path_p.p + tem_v;
                                        last_last_target.p = path_p.p;

                                        if ( t_last_rotation <= last_ratation_duration )
                                        {
                                            last_last_target.M = _last_last_target.M;
                                            last_target.M      = _last_target.M;
                                        }
                                        else if ( dt_link <= ratation_duration )
                                        {
                                            last_last_target.M = _last_target.M;
                                            last_target.M      = _target.M;
                                        }
                                        //**-------------------------------**//

                                        //! next_remain_length = ( target.p - last_target.p ).Norm( ) 这个刷新很重要,考虑一种情况，圆弧都走完了，还没能跳出，
                                        //!这个时候正好在直线段中 可以靠next_remain_length 正确地继续等待
                                        if ( ( last_target.p - path_p.p ).Norm( ) < ( next_remain_length = ( target.p - last_target.p ).Norm( ) ) )
                                        {
                                            command_flag = _command_flag + 1;
                                            PLOG_DEBUG << "等待结束，可以跳出";
                                        }
                                        // PLOG_DEBUG << " 继续等待中";
                                    }
                                    //**-------------------------------**//
                                }
                                else
                                    break;
                            }

                            if ( command_flag == _command_flag ) PLOG_DEBUG << "command_flag==_command_flag";
                            // if ( _target != target ) PLOG_DEBUG << "_target != target ";
                            // if ( ( last_target.p - path_p.p ).Norm( ) < eps ) PLOG_DEBUG << " ( last_target.p - path_p.p ).Norm( ) < eps";
                            // PLOG_DEBUG << "( last_target.p - path_p.p ).Norm( )  = " << ( last_target.p - path_p.p ).Norm( );

                            bool flag_is_rot_target{ false };

                            //!圆弧段设置目标，但圆弧走完都没能执行，没有后续的直线段
                            if ( command_flag == _command_flag && _target != target && ( last_target.p - path_p.p ).Norm( ) < big_eps )
                            {
                                PLOG_DEBUG << "圆弧段过程设置{只旋转目标}情况 且未能执行,进入command_flag=1处理";

                                command_flag = 1;
                                //设置这个的原因是{last_ratation_duration=0}和{ratation_duration=0}时，这两个变量不会刷新
                                last_last_target.M = _last_target.M;
                                last_target.M      = _target.M;
                                // PLOG_DEBUG << "target.m =";
                                // print_rotation( target.M );
                                // PLOG_DEBUG << "last_target.m = ";
                                // print_rotation( last_target.M );
                                // PLOG_DEBUG << "_target.m = ";
                                // print_rotation( _target.M );
                            }
                            //!圆弧段设置目标，但圆弧走完都没能执行，存在直线段，则可以立刻进入0度平行函数处理
                            else if ( command_flag == _command_flag && _target != target && ( last_target.p - path_p.p ).Norm( ) > big_eps )
                            {
                                flag_is_rot_target = true;
                                PLOG_DEBUG << "圆弧段过程设置{只旋转目标}情况 且未能执行,交付直线段处理";
                            }

                            while ( 1 )  //直线段段计算
                            {
                                if ( on_stop_trajectory ) throw "on_stop_trajectory";

                                _OnlineDoubleS.get_pos_vel_acc( OnlineDoubleS_count, s_p, sd_p, sdd_p );
                                if ( ( 1.0 - s_p ) > eps && _command_flag == command_flag )
                                {
                                    path_v = length_circle_link * sd_p;
                                    path_a = length_circle_link * sdd_p;
                                    path_p = link_trajectory( F_base_circleend, _target, ( s_p - bound_length ) / ( 1 - bound_length ) );

                                    // if ( last_ratation_duration == 0 ) link_flag = true;  //当前处于直线段，且上段姿态已经完成，所以可以放开修改

                                    if ( t_last_rotation <= last_ratation_duration )
                                    {
                                        _last_rotaion_OnlineDoubleS.get_pos_vel_acc( t_last_rotation, s_r, sd_r, sdd_r );
                                        path_p.M = ratation_trajectory( F_base_circlestart.M, F_base_circleend.M, s_r );
                                        t_last_rotation += 0.001;
                                        flag_from_last_rotation = true;
                                    }
                                    else if ( dt_link <= ratation_duration )
                                    {
                                        // link_flag = true;  //必需在直线段且上段姿态实现完，才能切换目标
                                        _rotaion_OnlineDoubleS.get_pos_vel_acc( dt_link, s_r, sd_r, sdd_r );
                                        path_p.M = ratation_trajectory( F_base_circleend.M, _target.M, s_r );
                                        dt_link += 0.001;
                                        flag_from_last_rotation = false;
                                    }

                                    lock_traj_frame.lock( );
                                    traj_frame.push_back( path_p );
                                    lock_traj_frame.unlock( );
                                    OnlineDoubleS_count++;

                                    lock_target.unlock( );
                                    std::this_thread::sleep_for( std::chrono::duration< double >( sleep_time ) );
                                    lock_target.lock( );

                                    if ( flag_is_rot_target )
                                    {
                                        PLOG_DEBUG << "圆弧段设置{只旋转目标}，且未执行，交付于直线段完成";
                                        command_flag++;
                                        flag_is_rot_target = false;
                                    }

                                    //** 检查新目标是否有足够的距离过渡，没有则以target-last_target距离定为过渡半径**//
                                    if ( _command_flag != command_flag )
                                    {
                                        PLOG_DEBUG << "直线过程设置新目标";
                                        //** 矫正直线过程中下一段新目标的参数 ，包括位置和姿态调整**//

                                        last_target.p      = _target.p;       //防止多次修改last_target
                                        last_last_target.p = _last_target.p;  //防止多次修改last_last_target

                                        if ( t_last_rotation <= last_ratation_duration )
                                        {
                                            last_last_target.M = _last_last_target.M;
                                            last_target.M      = _last_target.M;
                                        }
                                        else if ( dt_link <= ratation_duration )
                                        {
                                            last_last_target.M = _last_target.M;
                                            last_target.M      = _target.M;
                                        }
                                        //**-------------------------------**//

                                        Vector ab = last_target.p - path_p.p;
                                        Vector bc = target.p - last_target.p;

                                        double cos_alpha = std::max( -1., std::min( dot( ab, bc ) / ab.Norm( ) / bc.Norm( ), 1. ) );

                                        if ( ( 1 - cos_alpha ) <= eps || bc.Norm( ) <= eps )  //如果新目标为0度调用，那么立刻跳出
                                        {
                                            next_remain_length = 0;
                                            PLOG_DEBUG << "新目标0度调用,立刻跳转";
                                        }
                                        else if ( ( cos_alpha - ( -1 ) ) <= eps )  //如果新目标为180度调用，那么立刻跳出
                                        {
                                            next_remain_length = 0;
                                            PLOG_DEBUG << "新目标180调用,立刻跳转";
                                        }
                                        else if ( ( next_remain_length = bc.Norm( ) ) < ab.Norm( ) )  //如果新目标太近就等待跳转，适用于过渡和0度平行情况
                                        {
                                            command_flag = _command_flag;
                                            PLOG_DEBUG << "新目标太近,不允许跳转";
                                        }
                                        else
                                            next_remain_length = 0;
                                    }

                                    if ( next_remain_length > 0 )
                                    {
                                        //** 矫正直线过程中下一段新目标的参数 ，只有姿态调整**//
                                        if ( t_last_rotation <= last_ratation_duration )
                                        {
                                            last_last_target.M = _last_last_target.M;
                                            last_target.M      = _last_target.M;
                                        }
                                        else if ( dt_link <= ratation_duration )
                                        {
                                            last_last_target.M = _last_target.M;
                                            last_target.M      = _target.M;
                                        }
                                        //**-------------------------------**//

                                        if ( ( last_target.p - path_p.p ).Norm( ) < next_remain_length )
                                        {
                                            command_flag = _command_flag + 1;
                                            PLOG_DEBUG << "等待结束，可以跳出";
                                        }
                                    }
                                    //**-------------------------------**//
                                }
                                else
                                    break;
                            }

                            // command_flag!=1 是为了防止 篡改 上述的特殊处理
                            if ( ( 1.0 - s_p ) <= eps && command_flag != 1 && command_flag == _command_flag )
                            {
                                command_flag = 0;
                                PLOG_DEBUG << "多段直线规划已完成";
                            }
                        }
                    }
                    //**-------------------------------**//
                }
            }
        }
        catch ( const char* str )
        {
            PLOG_ERROR << "planning 触发急停";
            PLOG_ERROR << str;
            FinishPlanningFrame = true;
            command_flag        = 0;
        }
        catch ( const std::exception& e )
        {
            if ( on_stop_trajectory )
                PLOG_ERROR << "planning 触发急停";
            else
                PLOG_INFO << "planning 结束,未知异常";

            PLOG_ERROR << e.what( );
            FinishPlanningFrame = true;
            command_flag        = 0;
        }
        catch ( ... )
        {
            if ( on_stop_trajectory )
                PLOG_ERROR << "planning 触发急停";
            else
                PLOG_INFO << "planning  结束,未知异常";

            FinishPlanningFrame = true;
            command_flag        = 0;
        }
    }

    void SmartServo_Cartesian::RunSmartServo_Ik( rocos::Robot* robot_ptr )
    {
        //** 变量初始化 **//
        std::unique_lock< std::mutex > lock_traj_frame( mutex_traj_frame, std::defer_lock );  //不上锁
        std::unique_lock< std::mutex > lock_traj_joint( mutex_traj_joint, std::defer_lock );  //不上锁
        KDL::Frame frame_target;
        int count = 0;
        KDL::JntArray _q_target( jointNum );

        KDL::JntArray current_pos( jointNum );
        KDL::JntArray last_pos( jointNum );
        KDL::JntArray last_last_pos( jointNum );

        //**-------------------------------**//

        //** 程序初始化 **//
        for ( int i = 0; i < jointNum; i++ )
        {
            current_pos( i )   = robot_ptr->pos_[ i ];
            last_pos( i )      = current_pos( i );
            last_last_pos( i ) = current_pos( i );
        }

        //**-------------------------------**//

        //第一次启动需要等待command()
        while ( *external_finished_flag_ptr )
            ;

        //** 轨迹计算 **//
        while ( !on_stop_trajectory )
        {
            //** 读取最新Frame **//
            lock_traj_frame.lock( );
            if ( count < traj_frame.size( ) )
            {
                frame_target = traj_frame[ count ];
                lock_traj_frame.unlock( );
            }
            else
            {
                lock_traj_frame.unlock( );
                if ( FinishPlanningFrame )
                    break;  //全部frame已经取出。
                else
                    continue;  //未取出，但是当前已读取到最新数据。
            }
            count++;
            //**-------------------------------**//

            //** IK求解 **//
            if ( ( robot_ptr->kinematics_ ).CartToJnt( _q_init, frame_target, _q_target ) < 0 )
            {
                PLOG_ERROR << " CartToJnt failed  frame_target =\n"
                           << frame_target;
                // assert( 0 );
                on_stop_trajectory = true;
                break;
            }

            //**-------------------------------**//

            //** 速度和加速度保护 **//

            if ( on_stop_trajectory ) break;


            if ( check_vel_acc( _q_target, current_pos, last_pos, robot_ptr->max_vel_, robot_ptr->max_acc_ ) < 0 )
            {
                on_stop_trajectory = true;
                break;
            }
          
            last_last_pos = last_pos;
            last_pos      = current_pos;
            current_pos   = _q_target;
          
            //**-------------------------------**//

            lock_traj_joint.lock( );
            traj_joint.push_back( _q_target );
            lock_traj_joint.unlock( );
            _q_init = _q_target;
        }

        if ( on_stop_trajectory )
            PLOG_ERROR << "IK 触发急停";
        else
            PLOG_INFO << "IK结束";
        FinishRunPlanningIK = true;
    }

    void SmartServo_Cartesian::RunSmartServo_Motion( rocos::Robot* robot_ptr )
    {
        //** 变量初始化 **//
        std::unique_lock< std::mutex > lock_traj_joint( mutex_traj_joint, std::defer_lock );  //不上锁
        int traj_joint_count = 0;
        int count{ 0 };
        KDL::JntArray joint_command;
        int _tick_count{ robot_ptr->tick_count };

        KDL::JntArray current_pos( jointNum );
        KDL::JntArray last_pos( jointNum );
        KDL::JntArray last_last_pos( jointNum );
        //**-------------------------------**//

        //** 程序初始化 **//
        for ( int i = 0; i < jointNum; i++ )
        {
            current_pos( i )   = robot_ptr->pos_[ i ];
            last_pos( i )      = current_pos( i );
            last_last_pos( i ) = current_pos( i );
        }
        //**-------------------------------**//

        //第一次启动需要等待command()
        while ( *external_finished_flag_ptr )
            ;

        //** 正常情况下，接收IK解算后的关节轨迹 **//
        while ( !on_stop_trajectory )
        {
            //** 读取最新命令 **//
            lock_traj_joint.lock( );
            if ( traj_joint_count < traj_joint.size( ) )
            {
                joint_command = traj_joint[ traj_joint_count ];
                lock_traj_joint.unlock( );
            }
            else
            {
                lock_traj_joint.unlock( );
                if ( FinishRunPlanningIK )
                    break;  //全部frame已经取出
                else
                    continue;
            }

            traj_joint_count++;

            last_last_pos = last_pos;
            last_pos      = current_pos;
            current_pos   = joint_command;
            //**-------------------------------**//

            //** 位置伺服 **//
            for ( int i = 0; i < jointNum; ++i )
            {
                robot_ptr->pos_[ i ] = joint_command( i );
                robot_ptr->joints_[ i ]->setPosition( joint_command( i ) );
            }
            robot_ptr->hw_interface_->waitForSignal( 0 );
            //**-------------------------------**//

            time_count++;  //! 1ms计时，类内部使用

            //** 100ms进行一次心跳检查,超时触发紧急停止 **//
            if ( ( ++count ) == 100 )
            {
                count = 0;
                if ( _tick_count != robot_ptr->tick_count )
                {
                    _tick_count = robot_ptr->tick_count;
                }
                else
                {
                    PLOG_ERROR << "Some errors such as disconnecting from the controller";
                    on_stop_trajectory = true;
                }
            }
            //**-------------------------------**//
        }
        //**--------------------------------------------------------------**//

        //**紧急停止 ，可能原因：1.轨迹规划失败、2.IK求解失败 3.心跳保持超时**//
        if ( on_stop_trajectory )
        {
            PLOG_ERROR << "motion触发紧急停止";
            Joint_stop( robot_ptr, current_pos, last_pos, last_last_pos );
            // //! 触发急停后就冷静2秒，防止手一直按着触发急停
            std::this_thread::sleep_for( std::chrono::duration< double >{ 1 } );
        }
        else
            PLOG_INFO << "motion 结束";

        //!等待plannig规划线程完事以后，再统一把所有标志位归位
        while ( !FinishPlanningFrame ) std::this_thread::sleep_for( std::chrono::duration< double >{ 0.001 } );

        ( *external_finished_flag_ptr ) = true;   //这次smart servo已结束，等待下一次smart servo
        robot_ptr->is_running_motion    = false;  //机械臂运动已结束，可以执行其他离线类运动
        on_stop_trajectory              = false;
        PLOG_INFO << "SmartServo_Cartesian 全部结束";
    }

    int SmartServo_Cartesian::command( KDL::Frame new_target )
    {
        if ( !on_stop_trajectory )  //如果需要紧急停止，那么就不允许在更改指令了
        {
            if ( new_target == target )
            {
                PLOG_DEBUG << "目标一致，无效";
                return -1;
            }
            else if ( time_count < 240 )
            {
                PLOG_DEBUG << "240ms时间未到";
                return -1;
            }
            else
            {
                std::unique_lock< std::mutex > lock_target( target_mutex, std::defer_lock );  //不上锁
                time_count = 0;
                lock_target.lock( );
                last_last_target = last_target;
                last_target      = target;
                target           = new_target;
                lock_target.unlock( );
                command_flag++;
                PLOG_DEBUG << "命令已刷新";
                ( *external_finished_flag_ptr ) = false;
                return 0;
            }
        }
        else
        {
            PLOG_DEBUG << "control is not allowed during emergency stop";
            return -1;
        }
    }
    int SmartServo_Cartesian::command( const KDL::Vector& tem_v, const char* str )
    {
        if ( strcmp( str, "FLANGE" ) == 0 )
            return command( target * KDL::Frame{ tem_v } );

        else if ( strcmp( str, "BASE" ) == 0 )
            return command( KDL::Frame{ tem_v } * target );

        else
        {
            PLOG_ERROR << "don't support this command type:" << str;
            return -1;
        }
    }
    int SmartServo_Cartesian::command( const KDL::Rotation& tem_r, const char* str )
    {
        if ( strcmp( str, "FLANGE" ) == 0 )
            return command( target * KDL::Frame{ tem_r } );

        else if ( strcmp( str, "BASE" ) == 0 )
            return command( KDL::Frame{ tem_r * target.M, target.p } );

        else
        {
            PLOG_ERROR << "don't support this command type:" << str;
            return -1;
        }
    }

    KDL::Frame SmartServo_Cartesian::link_trajectory( const KDL::Frame& start, const KDL::Frame& end, double s_p )
    {
        if ( s_p > 1 || s_p < 0 )
        {
            on_stop_trajectory = true;
            PLOG_ERROR << "values of s_p outside interval [0,1] , s_p = " << s_p;
            throw "values of s_p outside interval [0,1] ";
        }

        // assert( !( s_p > 1 || s_p < 0 ) && "values of s_p outside interval [0,1]" );  //*! 只要s超出范围就退出

        //** 变量初始化 **//
        KDL::Vector Pstart = start.p;
        KDL::Vector Pend   = end.p;
        KDL::Vector P      = Pstart + ( Pend - Pstart ) * s_p;

        return KDL::Frame( start.M, P );
    }

    KDL::Frame SmartServo_Cartesian::link_trajectory( const KDL::Frame& start, const KDL::Frame& end, double s_p, double s_r )
    {
        if ( ( s_p > 1 || s_p < 0 ) )
        {
            on_stop_trajectory = true;
            PLOG_ERROR << "values of s_p outside interval [0,1] , s_p = " << s_p;
            throw "values of s_p outside interval [0,1]";
            // assert( 0 );  //*! 只要s超出范围就退出
        }
        if ( ( s_r > 1 || s_r < 0 ) )
        {
            on_stop_trajectory = true;
            PLOG_ERROR << "values of s_r outside interval [0,1]  , s_r =" << s_r;
            throw "values of s_p outside interval [0,1]";

            // assert( 0 );  //*! 只要s超出范围就退出
        }

        //** 变量初始化 **//
        KDL::Vector Pstart = start.p;
        KDL::Vector Pend   = end.p;
        KDL::Vector P      = Pstart + ( Pend - Pstart ) * s_p;

        std::vector< double > Quaternion_start{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_end{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_interp{ 0, 0, 0, 0 };
        start.M.GetQuaternion( Quaternion_start.at( 0 ), Quaternion_start.at( 1 ), Quaternion_start.at( 2 ), Quaternion_start.at( 3 ) );
        end.M.GetQuaternion( Quaternion_end.at( 0 ), Quaternion_end.at( 1 ), Quaternion_end.at( 2 ), Quaternion_end.at( 3 ) );
        Quaternion_interp = UnitQuaternion_intep( Quaternion_start, Quaternion_end, s_r );

        return KDL::Frame( KDL::Rotation::Quaternion( Quaternion_interp[ 0 ], Quaternion_interp[ 1 ], Quaternion_interp[ 2 ], Quaternion_interp[ 3 ] ), P );
    }

    std::vector< double > SmartServo_Cartesian::UnitQuaternion_intep( const std::vector< double >& start, const std::vector< double >& end, double s, bool flag_big_angle )
    {
        constexpr double eps = 1E-7;

        if ( ( s > 1 || s < 0 ) )
        {
            on_stop_trajectory = true;
            PLOG_ERROR << "values of s outside interval [0,1] , s = " << s;
            throw "values of s outside interval [0,1]";
        }

        double cosTheta = start[ 0 ] * end[ 0 ] + start[ 1 ] * end[ 1 ] + start[ 2 ] * end[ 2 ] +
                          start[ 3 ] * end[ 3 ];

        std::vector< double > start_2 = start;

        //** 指定得取大角度还是小角度 **//

        if ( ( cosTheta < 0 && !flag_big_angle ) || ( cosTheta >= 0 && flag_big_angle ) )
        {
            for ( int i = 0; i < 4; i++ ) start_2[ i ] *= -1;
            cosTheta *= -1;
        }

        //**-------------------------------**//

        double theta = acos( cosTheta );

        //! theta本应该计算为0，但是实际可能为0.000001，即使有(eps=1E-7)限定范围，仍然有误判可能,所以最好使用isnan()
        if ( abs( theta ) < eps || s == 0 )
            return start;
        else
        {
            double coefficient_1 = sin( ( 1 - s ) * theta ) / sin( theta );
            double coefficient_2 = sin( ( s )*theta ) / sin( theta );

            std::vector< double > res{
                coefficient_1 * start_2[ 0 ] + coefficient_2 * end[ 0 ],
                coefficient_1 * start_2[ 1 ] + coefficient_2 * end[ 1 ],
                coefficient_1 * start_2[ 2 ] + coefficient_2 * end[ 2 ],
                coefficient_1 * start_2[ 3 ] + coefficient_2 * end[ 3 ] };

            //!防止误判
            for ( const auto& var : res )
                if ( isnan( var ) )
                    return start_2;

            return res;
        }
    }

    KDL::Frame SmartServo_Cartesian::cirlular_trajectory( const KDL::Frame& F_base_circlestart, const KDL::Frame& F_base_circleend, const KDL::Frame& F_base_circleCenter, double s_p, double alpha )
    {
        using namespace KDL;

        if ( ( s_p > 1 || s_p < 0 ) )
        {
            on_stop_trajectory = true;
            PLOG_ERROR << "values of s outside interval [0,1] , s = " << s_p;
            throw "values of s outside interval [0,1] ";
            // assert( 0);  //*! 只要s超出范围就退出
        }

        double radius = ( F_base_circlestart.p - F_base_circleCenter.p ).Norm( );

        return KDL::Frame( F_base_circlestart.M, F_base_circleCenter * Vector{ radius * cos( s_p * alpha ), radius * sin( s_p * alpha ), 0 } );
    }

    KDL::Frame SmartServo_Cartesian::cirlular_trajectory( const KDL::Frame& F_base_circlestart, const KDL::Frame& F_base_circleend, const KDL::Frame& F_base_circleCenter, double s_p, double s_r, double alpha )
    {
        using namespace KDL;

        if ( ( s_p > 1 || s_p < 0 ) )
        {
            on_stop_trajectory = true;
            PLOG_ERROR << "values of s_p outside interval [0,1] , s_p = " << s_p;
            throw "values of s outside interval [0,1] ";

            // assert( 0);  //*! 只要s超出范围就退出
        }
        if ( ( s_r > 1 || s_r < 0 ) )
        {
            on_stop_trajectory = true;
            PLOG_ERROR << "values of s_r outside interval [0,1] , s_r = " << s_r;
            throw "values of s outside interval [0,1] ";

            // assert( 0);  //*! 只要s超出范围就退出
        }
        // assert( !( s_p > 1 || s_p < 0 ) && "values of s_p outside interval [0,1]" );  //*! 只要s超出范围就退出
        // assert( !( s_r > 1 || s_r < 0 ) && "values of s_r outside interval [0,1]" );  //*! 只要s超出范围就退出

        double radius = ( F_base_circlestart.p - F_base_circleCenter.p ).Norm( );

        std::vector< double > Quaternion_start{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_end{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_interp{ 0, 0, 0, 0 };
        F_base_circlestart.M.GetQuaternion( Quaternion_start.at( 0 ), Quaternion_start.at( 1 ), Quaternion_start.at( 2 ), Quaternion_start.at( 3 ) );
        F_base_circleend.M.GetQuaternion( Quaternion_end.at( 0 ), Quaternion_end.at( 1 ), Quaternion_end.at( 2 ), Quaternion_end.at( 3 ) );
        Quaternion_interp = UnitQuaternion_intep( Quaternion_start, Quaternion_end, s_r );

        return KDL::Frame( KDL::Rotation::Quaternion( Quaternion_interp[ 0 ], Quaternion_interp[ 1 ], Quaternion_interp[ 2 ], Quaternion_interp[ 3 ] ), F_base_circleCenter * Vector{ radius * cos( s_p * alpha ), radius * sin( s_p * alpha ), 0 } );
    }

    KDL::Rotation SmartServo_Cartesian::ratation_trajectory( const KDL::Rotation& start, const KDL::Rotation& end, double s_r, bool flag_big_angle )
    {
        using namespace KDL;

        if ( abs( 1 - s_r ) < middle_eps ) s_r = 1.0;

        if ( ( s_r > 1 || s_r < 0 ) )
        {
            on_stop_trajectory = true;
            PLOG_ERROR << "values of s_r outside interval [0,1] , s_r = " << s_r;
            throw "values of s outside interval [0,1] ";

            // assert( 0);  //*! 只要s超出范围就退出
        }

        // assert( !( s_r > 1 || s_r < 0 ) && "values of s_r outside interval [0,1]" );  //*! 只要s超出范围就退出

        std::vector< double > Quaternion_start{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_end{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_interp{ 0, 0, 0, 0 };
        start.GetQuaternion( Quaternion_start.at( 0 ), Quaternion_start.at( 1 ), Quaternion_start.at( 2 ), Quaternion_start.at( 3 ) );
        end.GetQuaternion( Quaternion_end.at( 0 ), Quaternion_end.at( 1 ), Quaternion_end.at( 2 ), Quaternion_end.at( 3 ) );
        Quaternion_interp = UnitQuaternion_intep( Quaternion_start, Quaternion_end, s_r, flag_big_angle );

        return KDL::Rotation::Quaternion( Quaternion_interp[ 0 ], Quaternion_interp[ 1 ], Quaternion_interp[ 2 ], Quaternion_interp[ 3 ] );
    }

    void OnlineDoubleS::calculate(
        double q0,
        double q1,
        double v_s,
        double v_e,
        double a_s,
        double a_e,

        double v_max,
        double a_max,
        double j_max

    )
    {
        input.target_position[ 0 ]      = q1;
        input.target_velocity[ 0 ]      = v_e;
        input.target_acceleration[ 0 ]  = a_e;
        input.current_position[ 0 ]     = q0;
        input.current_velocity[ 0 ]     = v_s;
        input.current_acceleration[ 0 ] = a_s;
        input.max_velocity              = { v_max };
        input.max_acceleration          = { a_max };
        input.max_jerk                  = { j_max };

        ruckig::Result result = otg.calculate( input, _trajectory );

        _trajectory.at_time( _trajectory.get_duration( ), position, velocity, acceleration );
        s = position[ 0 ];

        if ( count > 100 )
        {
            throw "resulit = Invalid input!";
        }
        else if ( static_cast< int >( result ) < 0 )
        {
            count++;
            calculate( q0, q1, v_s, v_e, a_s, a_e, v_max * 0.95, a_max * 0.9, j_max * 0.9 );
        }
        else if ( abs( 1.0 - s ) > middle_eps )
        {
            count++;
            calculate( q0, q1, v_s, v_e, a_s, a_e, v_max * 0.95, a_max * 0.9, j_max * 0.9 );
        }
        else
        {
            count = 0;
            return;
        }
    }
    void OnlineDoubleS::calculate(
        double q0,
        double q1,
        double v_s,
        double v_e,
        double a_s,
        double a_e,

        double v_max,
        double a_max,
        double j_max,
        double t )
    {
        input.target_position[ 0 ]      = q1;
        input.target_velocity[ 0 ]      = v_e;
        input.target_acceleration[ 0 ]  = a_e;
        input.current_position[ 0 ]     = q0;
        input.current_velocity[ 0 ]     = v_s;
        input.current_acceleration[ 0 ] = a_s;
        input.max_velocity              = { v_max };
        input.max_acceleration          = { a_max };
        input.max_jerk                  = { j_max };
        input.minimum_duration          = t;

        ruckig::Result result = otg.calculate( input, _trajectory );

        _trajectory.at_time( _trajectory.get_duration( ), position, velocity, acceleration );
        s = position[ 0 ];

        if ( count > 100 )
        {
            throw "resulit = Invalid input!";
        }
        else if ( static_cast< int >( result ) < 0 )
        {
            count++;
            calculate( q0, q1, v_s, v_e, a_s, a_e, v_max * 0.95, a_max * 0.9, j_max * 0.9, t );
        }
        else if ( abs( 1.0 - s ) > middle_eps )
        {
            count++;
            calculate( q0, q1, v_s, v_e, a_s, a_e, v_max * 0.95, a_max * 0.9, j_max * 0.9, t );
        }
        else
        {
            count = 0;
            return;
        }
    }

    double OnlineDoubleS::get_duration( )
    {
        return _trajectory.get_duration( ) > 0 ? _trajectory.get_duration( ) : -1;
    }

    void OnlineDoubleS::get_pos_vel_acc( int i, double& p, double& v, double& a )
    {
        _trajectory.at_time( T_S * i, position, velocity, acceleration );
        p = position[ 0 ];
        v = velocity[ 0 ];
        a = acceleration[ 0 ];
    }

    void OnlineDoubleS::get_pos_vel_acc( double t, double& p, double& v, double& a )
    {
        _trajectory.at_time( t, position, velocity, acceleration );
        p = position[ 0 ];
        v = velocity[ 0 ];
        a = acceleration[ 0 ];
    }

#endif

    SmartServo_Cartesian::SmartServo_Cartesian(std::atomic<bool> *finished_flag_ptr, const KDL::Chain &robot_chain) : _ik_vel{robot_chain}, FK_slover{robot_chain}
    {
        joint_current.resize(jointNum);
        joint_target.resize(jointNum);
        joint_vel.resize(jointNum);
        joint_last_pos.resize(jointNum);
        joint_last_last_pos.resize(jointNum);
        external_finished_flag_ptr = finished_flag_ptr;
    }

    void SmartServo_Cartesian::init(rocos::Robot *robot_ptr, int flag, double target_vel, double max_vel, double max_acc, double max_jerk)
    {
        input.current_position[0] = 0;
        input.current_velocity[0] = 0;
        input.current_acceleration[0] = 0;

        if (flag >= 700) { //Runto情况-位置模型
            FK_slover.JntToCart(vector_2_JntArray(robot_ptr->pos_),init_flange ); //需要记录开始位置
            KDL::Frame  RunTo_target =   robot_ptr->get_RunTo_Car_target(flag - 700);
            double Plength = (RunTo_target.p - init_flange.p).Norm();
            KDL::Rotation R_start_end = init_flange.M.Inverse() * RunTo_target.M;
            KDL::Vector ration_axis;
            double angle = R_start_end.GetRotAngle(ration_axis);
            const double equivalent_radius = 0.1; 
            double Rlength = equivalent_radius * abs(angle);
            RunTo_length = std::max(Plength,Rlength);
            input.target_position[0] = 1;
            input.target_velocity[0] = 0;
            input.target_acceleration[0] = 0;
            input.max_velocity[0] = target_vel/RunTo_length;
            input.max_acceleration[0] = max_vel/RunTo_length;
            input.max_jerk[0] = max_acc/RunTo_length;
        } else  // 点动情况 -速度模式
        {
            input.target_position[0] = target_vel;
            input.target_velocity[0] = 0;
            input.target_acceleration[0] = 0;
            input.max_velocity[0] = max_vel;
            input.max_acceleration[0] = max_acc;
            input.max_jerk[0] = max_jerk;
        }

        input.control_interface = ruckig::ControlInterface::Position;
        input.synchronization = ruckig::Synchronization::None;

        flag_stop = false;

        KDL::SetToZero(joint_vel);

        for (int i{0}; i < jointNum; i++)
        {
            joint_current(i) = robot_ptr->pos_[i];
            joint_target(i) = joint_current(i);
            joint_last_pos(i) = joint_current(i);
            joint_last_last_pos(i) = joint_current(i);
        }

        _Cartesian_vel_index = 0; // 0代表无方向

        _reference_frame.clear(); // 空字符代表无参考坐标系

        current_flange = KDL::Frame{};

               PLOG_INFO << "笛卡尔空间点动初始化完成";
    }

    int SmartServo_Cartesian::update(KDL::JntArray &joint_vel, rocos::Robot *robot_ptr)
    {
        res = otg.update(input, output);
        if (res != ruckig::Result::Working && res != ruckig::Result::Finished)
        {
            PLOG_ERROR << "OTG 计算失败";
            return -1;
        }

        const auto &res_vel = output.new_position;

        KDL::SetToZero(joint_vel);

        if (abs(_Cartesian_vel_index) <= 6)  // 点动
        {
            KDL::Twist Cartesian_vel{};

            if (abs(_Cartesian_vel_index) <= 3)  // 移动
            {
                Cartesian_vel.vel[abs(_Cartesian_vel_index) - 1] = sign(_Cartesian_vel_index) * res_vel[0];

            } else  // 旋转
            {
                Cartesian_vel.rot[abs(_Cartesian_vel_index) - 4] = sign(_Cartesian_vel_index) * res_vel[0];
            }
            //! 速度矢量的参考系默认为base系，参考点为flange
            if (_reference_frame == "flange") {  //** 转变速度矢量的参考系，由flange系变为base系，但没有改变参考点（还是flange） **//
                FK_slover.JntToCart(vector_2_JntArray(robot_ptr->pos_), current_flange);
                Cartesian_vel = current_flange.M * Cartesian_vel;
                // std::cout<<"Cartesian_vel.vel:"<<Cartesian_vel.vel<<std::endl;
                // std::cout<<"Cartesian_vel.rot:"<<Cartesian_vel.rot<<std::endl;
            }
            if (_reference_frame == "tool") {  //** 转变速度矢量的参考系，由tool系变为base系，改变参考点（改为tool） **//
                FK_slover.JntToCart(vector_2_JntArray(robot_ptr->pos_), current_flange);
                KDL::Frame tool2flange = robot_ptr->getT_tool_();

                KDL::Frame tool_base = current_flange * tool2flange;
                // Eigen::Vector3d angular_velocity(Cartesian_vel.rot.x(), Cartesian_vel.rot.y(), Cartesian_vel.rot.z());

                // // 将KDL::Frame的旋转部分转换为Eigen::Matrix3d
                // Eigen::Matrix3d rotation;
                // for (int i = 0; i < 3; ++i)
                // {
                //     for (int j = 0; j < 3; ++j)
                //     {
                //         rotation(i, j) = current_flange.M(i, j);
                //     }
                // }
                // Eigen::Matrix3d tool_rotation;
                // for (int i = 0; i < 3; ++i)
                // {
                //     for (int j = 0; j < 3; ++j)
                //     {
                //         tool_rotation(i, j) = tool_base.M(i, j);
                //     }
                // }
                // // 将KDL::Frame的位置部分转换为Eigen::Vector3d
                // Eigen::Vector3d position(-tool2flange.p.x(), -tool2flange.p.y(), -tool2flange.p.z());

                // // 计算叉积
                // Eigen::Vector3d cross_product = angular_velocity .cross(position);
                // KDL::Vector temp = KDL::Vector(cross_product(0), cross_product(1), cross_product(2));
                KDL::Vector t_f_tool = robot_ptr->getT_tool_().Inverse().p;

                Cartesian_vel.vel = tool_base.M * (Cartesian_vel.vel + Cartesian_vel.rot * t_f_tool);
                Cartesian_vel.rot = tool_base.M * Cartesian_vel.rot;

                //  Cartesian_vel = current_flange.M * Cartesian_vel;
                // std::cout << "Cartesian_vel.vel:" << Cartesian_vel.vel << std::endl;
                // std::cout << "Cartesian_vel.rot:" << Cartesian_vel.rot << std::endl;
            }
            if (_reference_frame == "object") {  //** 转变速度矢量的参考系，由flange系变为base系，改变参考点（还是object） **//
                                                 // objectt标定的就是工件相对于基座的位姿，所以不需要转换，移动就是沿着工件的坐标系移动，旋转就是绕工件的坐标系旋转，但是运动的点为末端法兰盘
                FK_slover.JntToCart(vector_2_JntArray(robot_ptr->pos_), current_flange);
                KDL::Frame object2base = robot_ptr->getT_object_();
                Cartesian_vel = object2base.M * Cartesian_vel;
            }

            //! 雅克比默认参考系为base,参考点为flange
            if (_ik_vel.CartToJnt(joint_current, Cartesian_vel, joint_vel) != 0) {
                PLOG_ERROR << "雅克比计算错误,错误号：" << _ik_vel.CartToJnt(joint_current, Cartesian_vel, joint_vel);
                return -1;
            }

        } else  // Runto
        {
            KDL::Frame interp_frame{};
            if (link_pos(init_flange, robot_ptr->get_RunTo_Car_target(_Cartesian_vel_index - 700), res_vel[0], res_vel[0], interp_frame) < 0) {
                PLOG_ERROR << "Runto目标点位不可达";
                return -1;
            }
            KDL::JntArray q_target(7);

            int axis_num = robot_ptr->getJointNum( );
            union_frame target_frame{ };
            if ( axis_num == 7 )  // 7自由度情况逆解
            {
                target_frame.target_7axis = std::pair< KDL::Frame, double >{ { interp_frame }, { joint_current( 2 ) } };
            }
            else  // 其他自由度情况逆解
            {
                target_frame.target_6axis = interp_frame;
            }
            if ( union_cartesian_to_joint( robot_ptr, target_frame, joint_current, q_target ) < 0 )
            {
                PLOG_ERROR << "Runto目标点位不可达";
                return -1;
            }

                KDL::Subtract( q_target, joint_current, joint_vel );  // 这里求得是1ms的点位
                KDL::Divide( joint_vel, 0.001, joint_vel );           // 这里转为标准单位1s的关节速度
            }

        output.pass_to_input(input);
   

        if (res == ruckig::Result::Working)
            return 1;
        else
            // PLOG_INFO << "完成！";
            return 0;
    }

    void SmartServo_Cartesian::RunMotion(rocos::Robot *robot_ptr)
    {
        int t_count = 0; // 时间计数
        int _tick_count{robot_ptr->tick_count};
        auto t_start = std::chrono::high_resolution_clock::now();
        auto t_stop = t_start;
        std::chrono::duration<double> duration;

        //! 由init()保证成立，由command()来打破
        while (*external_finished_flag_ptr)
        {
            ;  // 等待指令
        }

        while (true)
        {
            t_start = std::chrono::high_resolution_clock::now();

            //** 心跳检查 **//
            if (!flag_stop)
                t_count++;

            if (t_count > 100) // 50毫秒保持一次通信
            {
                t_count = 0;
                if (_tick_count != robot_ptr->tick_count)
                    _tick_count = robot_ptr->tick_count;
                else
                {
                    PLOG_WARNING << "点动指令时间间隔过长,停止";

                    Cartesian_stop(); // 速度目标设置为0
                }
            }
            //**-------------------------------**//

            int res = update(joint_vel, robot_ptr);

            if (res < 0) // OTG的 error 状态
            {
                // 关节空间急停
                flag_stop = true;
                Joint_stop(robot_ptr, joint_current, joint_last_pos, joint_last_last_pos);
                sleep(2);
                break;
            }
            else // working 或者finished状态
            {
                KDL::Multiply(joint_vel, servo_dt, joint_vel);
                KDL::Add(joint_current, joint_vel, joint_target);

                //** 速度和加速度保护 **//
                if (!flag_stop && check_vel_acc(joint_target, joint_current, joint_last_pos, 1, 5) < 0)
                {
                    //! 急停状态下不用速度检查，因为会和笛卡尔急停冲突（笛卡尔急停会使得关节加速度超大，必触发关节急停保护）
                    flag_stop = true;
                    Joint_stop(robot_ptr, joint_current, joint_last_pos, joint_last_last_pos); // 关节空间急停
                    PLOG_DEBUG<<"急停结束";
                    sleep(2);
                    break;
                }

                joint_last_last_pos = joint_last_pos;
                joint_last_pos = joint_current;
                joint_current = joint_target;
                //**-------------------------------**//

                //** 安全位置伺服,防止关节超限 **//
                safety_servo(robot_ptr, joint_target);
                //**-------------------------------**//

                if (res == 0 && flag_stop)
                {
                    //                    PLOG_INFO << "笛卡尔空间急停已完成";
                    break;
                }
            }
            t_stop = std::chrono::high_resolution_clock::now();
            duration = (t_stop - t_start);
            //            if ( duration.count( ) > 0.0015 )
            //            {
            //                PLOG_WARNING << "计算时间超时：" << duration.count( ) << "s";
            //            }
        }

        (*external_finished_flag_ptr) = true;                    // 这次smart servo已结束，等待下一次smart servo
                                                                 //        robot_ptr->is_running_motion    = false;  // 机械臂运动已结束，可以执行其他离线类运动
        robot_ptr->setRunState(rocos::Robot::RunState::Stopped); // 机械臂运动已结束，可以执行其他离线类运动
    }

    void SmartServo_Cartesian::command(int Cartesian_vel_index, const char *reference_frame)
    {
        if (_Cartesian_vel_index == 0)
            _Cartesian_vel_index = Cartesian_vel_index;

        if (_reference_frame.empty())
            _reference_frame = reference_frame;

        if (!flag_stop)
        {
            if (_Cartesian_vel_index != Cartesian_vel_index)
            {
                PLOG_ERROR << "方向变换，停止！";
                Cartesian_stop();
            }
            else if (_reference_frame != reference_frame)
            {
                PLOG_ERROR << "参考坐标系变换，停止！";
                Cartesian_stop();
            }
            else
                *external_finished_flag_ptr = false;
        }
        else
        {
            //            PLOG_WARNING << "紧急停止中,不允许修改目标";
        }
    }

    void SmartServo_Cartesian::Cartesian_stop(double max_vel, double max_acc, double max_jerk)
    {
        flag_stop = true;
        if (_Cartesian_vel_index >= 700)  // Runto情况-位置模型
        {
            input.control_interface = ruckig::ControlInterface::Velocity;
            input.target_velocity[0] = 0;
            input.target_acceleration[0] = 0;
            input.max_acceleration[0] = max_vel/RunTo_length;
            input.max_jerk[0] = max_acc/RunTo_length;

        } else  // 点动情况
        {
            input.target_position[0] = 0;
            input.target_velocity[0] = 0;
            input.target_acceleration[0] = 0;

            input.max_velocity[0] = max_vel;
            input.max_acceleration[0] = max_acc;
            input.max_jerk[0] = max_jerk;
        }
    }

#pragma endregion

#pragma region //*零空间点动实现

    SmartServo_Nullspace::SmartServo_Nullspace(std::atomic<bool> *finished_flag_ptr, const KDL::Chain &robot_chain) : jnt2jac(robot_chain), jac(jointNum),
                                                                                                                      U(Eigen::MatrixXd::Zero(6, jointNum)),
                                                                                                                      S(Eigen::VectorXd::Zero(jointNum)),
                                                                                                                      Sinv(Eigen::VectorXd::Zero(jointNum)),
                                                                                                                      V(Eigen::MatrixXd::Zero(jointNum, jointNum)),
                                                                                                                      tmp(Eigen::VectorXd::Zero(jointNum)),
                                                                                                                      fk_slover(robot_chain)

    {
        joint_current.resize(jointNum);
        joint_target.resize(jointNum);
        joint_vel.resize(jointNum);
        joint_last_pos.resize(jointNum);
        joint_last_last_pos.resize(jointNum);
        external_finished_flag_ptr = finished_flag_ptr;
    }

    void SmartServo_Nullspace::init(rocos::Robot *robot_ptr, double target_vel, double max_vel, double max_acc, double max_jerk)
    {
        input.current_position[0] = 0;
        input.current_velocity[0] = 0;
        input.current_acceleration[0] = 0;

        input.target_position[0] = target_vel;
        input.target_velocity[0] = 0;
        input.target_acceleration[0] = 0;

        input.max_velocity[0] = max_vel;
        input.max_acceleration[0] = max_acc;
        input.max_jerk[0] = max_jerk;

        input.control_interface = ruckig::ControlInterface::Position;
        input.synchronization = ruckig::Synchronization::None;

        flag_stop = false;

        KDL::SetToZero(joint_vel);

        for (int i{0}; i < jointNum; i++)
        {
            joint_current(i) = robot_ptr->pos_[i];
            joint_target(i) = joint_current(i);
            joint_last_pos(i) = joint_current(i);
            joint_last_last_pos(i) = joint_current(i);
        }

        _jogging_Direction = 0;  // 0代表无方向
        _max_jac_cul_index = -1; //-1代表无指定控制哪个关节
        _command_Direction = 0;  // 0表示未知命令方向与臂角方向是否一致
        PLOG_INFO << "零空间点动初始化完成";
    }

    int SmartServo_Nullspace::update(KDL::JntArray &joint_vel)
    {
        KDL::SetToZero(joint_vel);

        int error_code;
        error_code = jnt2jac.JntToJac(joint_current, jac);
        if (error_code != 0)
        {
            PLOG_ERROR << "雅克比计算错误:" << error_code;
            return -1;
        }

        error_code = KDL::svd_eigen_HH(jac.data, U, S, V, tmp, 150);
        if (error_code != 0)
        {
            PLOG_ERROR << "雅克比SVD计算错误:" << error_code;
            return -1;
        }

        for (int i = 0; i < jointNum; ++i)
        {
            Sinv(i) = fabs(S(i)) < 0.00001 ? 0.0 : 1.0 / S(i);
        }

        null_space_jac = Eigen::MatrixXd::Identity(7, 7) - V * Sinv.asDiagonal() * U.transpose() * U * S.asDiagonal() * V.transpose();

        //** 找到哪个关节运动范围最大 **//
        if (_max_jac_cul_index == -1)
        {
            null_space_jac.diagonal().maxCoeff(&_max_jac_cul_index);
            is_same_on_direction(null_space_jac.col(_max_jac_cul_index));
        }
        //**-------------------------------**//

        //** 判断当前位置下，能否进行零空间点动 **//
        int joint_index = 0;
        for (; joint_index < jointNum; joint_index++)
        {
            if (!null_space_jac.col(joint_index).isZero(1e-13))
                break;
        }
        if (joint_index == jointNum)
        {
            PLOG_ERROR << "当前构型下，无法进行臂角点动";
            return -1;
        }
        //**-------------------------------**//

        otg_res = otg.update(input, output);

        if (otg_res != ruckig::Result::Working && otg_res != ruckig::Result::Finished)
        {
            PLOG_ERROR << "OTG 计算失败";
            return -1;
        }

        const auto &otg_pos = output.new_position;

        tmp.setZero();
        tmp(_max_jac_cul_index) = _command_Direction * otg_pos[0];

        joint_vel.data = null_space_jac * tmp;

        output.pass_to_input(input);

        if (otg_res == ruckig::Result::Working)
            return 1;
        else
            // PLOG_INFO << "otg完成！";
            return 0;
    }

    void SmartServo_Nullspace::RunMotion(rocos::Robot *robot_ptr)
    {
        int t_count = 0; // 时间计数
        int _tick_count{robot_ptr->tick_count};
        auto t_start = std::chrono::high_resolution_clock::now();
        auto t_stop = t_start;
        std::chrono::duration<double> duration;

        //! 由init()保证成立，由command()来打破
        while (*external_finished_flag_ptr)
        {
            ; // 等待指令
        }

        while (1)
        {
            t_start = std::chrono::high_resolution_clock::now();

            //** 心跳检查 **//
            if (!flag_stop)
                t_count++;

            if (t_count > 100) // 100毫秒保持一次通信
            {
                t_count = 0;
                if (_tick_count != robot_ptr->tick_count)
                    _tick_count = robot_ptr->tick_count;
                else
                {
                    //                    PLOG_WARNING << "点动指令时间间隔过长,停止";
                    nullspace_stop(); // 速度目标设置为0
                }
            }
            //**-------------------------------**//

            int update_res = update(joint_vel);

            if (update_res < 0) // OTG的 error 状态
            {
                // 关节空间急停
                flag_stop = true;
                Joint_stop(robot_ptr, joint_current, joint_last_pos, joint_last_last_pos);
                sleep(2);
                break;
            }
            else // working 或者finished状态
            {
                KDL::Multiply(joint_vel, servo_dt, joint_vel);
                KDL::Add(joint_current, joint_vel, joint_target);

                //** 速度和加速度保护 **//
                if (!flag_stop && check_vel_acc(joint_target, joint_current, joint_last_pos, 1, 5) < 0)
                {
                    //! 急停状态下不用速度检查，因为会和笛卡尔急停冲突（笛卡尔急停会使得关节加速度超大，必触发关节急停保护）
                    flag_stop = true;
                    Joint_stop(robot_ptr, joint_current, joint_last_pos, joint_last_last_pos); // 关节空间急停
                    sleep(2);
                    break;
                }

                joint_last_last_pos = joint_last_pos;
                joint_last_pos = joint_current;
                joint_current = joint_target;
                //**-------------------------------**//

                //** 安全位置伺服,防止关节超限 **//
                safety_servo(robot_ptr, joint_target);
                //**-------------------------------**//

                if (update_res == 0 && flag_stop)
                {
                    //                    PLOG_INFO << "零空间急停已完成";
                    break;
                }
            }
            t_stop = std::chrono::high_resolution_clock::now();
            duration = (t_stop - t_start);
            //            if ( duration.count( ) > 0.0015 )
            //            {
            //                PLOG_WARNING << "计算时间超时：" << duration.count( ) << "s";
            //            }
        }

        (*external_finished_flag_ptr) = true;                    // 这次smart servo已结束，等待下一次smart servo
                                                                 //        robot_ptr->is_running_motion    = false;  // 机械臂运动已结束，可以执行其他离线类运动
        robot_ptr->setRunState(rocos::Robot::RunState::Stopped); // 机械臂运动已结束，可以执行其他离线类运动
    }

    void SmartServo_Nullspace::command(int jogging_Direction)
    {
        if (_jogging_Direction == 0)
            _jogging_Direction = jogging_Direction;

        if (!flag_stop)
        {
            if (_jogging_Direction != jogging_Direction)
            {
                PLOG_ERROR << "方向变换，停止！";
                nullspace_stop();
            }
            else
                *external_finished_flag_ptr = false;
        }
        else
            PLOG_WARNING << "紧急停止中,不允许修改目标";
    }

    void SmartServo_Nullspace::nullspace_stop(double max_vel, double max_acc, double max_jerk)
    {
        flag_stop = true;
        input.target_position[0] = 0;
        input.target_velocity[0] = 0;
        input.target_acceleration[0] = 0;

        input.max_velocity[0] = max_vel;
        input.max_acceleration[0] = max_acc;
        input.max_jerk[0] = max_jerk;
    }

    bool SmartServo_Nullspace::is_same_on_direction(const Eigen::Block<Eigen::Matrix<double, 7, 7>, 7, 1, true> &joint_vel)
    {
        // 如果不是7自由度直接退出
        if (jointNum != 7)
            return false;

        KDL::JntArray joint_target(jointNum);
        for (int i = 0; i < jointNum; i++)
        {
            joint_target(i) = joint_vel(i) * servo_dt + joint_current(i);
        }

        KDL::Frame frame_joint_4;
        KDL::Frame frame_joint_4_target;
        fk_slover.JntToCart(joint_current, frame_joint_4, 4);
        fk_slover.JntToCart(joint_target, frame_joint_4_target, 4);

        KDL::Rotation R_start_end = frame_joint_4.M.Inverse() * frame_joint_4_target.M;
        KDL::Vector axis;
        double angle = R_start_end.GetRotAngle(axis);
        axis = frame_joint_4.M * axis;

        KDL::Frame shoulder;
        KDL::Frame wrist;
        fk_slover.JntToCart(joint_current, shoulder, 2);
        fk_slover.JntToCart(joint_current, wrist, 6);
        KDL::Vector SW = wrist.p - shoulder.p;
        ;

        for (int i = 0; i < 3; i++)
            if (std::abs(SW(i)) > 0.001 && KDL::sign(SW(i)) != KDL::sign(axis(i)))
            {
                // GetRotAngle()计算的轴不是由1关节指向6关节
                angle = -1 * angle;
                break;
            }

        // PLOG_DEBUG << "SW = " << SW;
        // PLOG_DEBUG << "axis = " << axis;
        // PLOG_DEBUG << "angle = " << angle;
        // PLOG_DEBUG << "_jogging_Direction" << _jogging_Direction;

        if (KDL::sign(angle) == _jogging_Direction)
        {
            _command_Direction = 1;
            return true;
        }
        else
        {
            _command_Direction = -1;
            return false;
        }
    }

#pragma endregion

#pragma region //*逆解

    namespace slover_thet2
    {
        JC_double m(JC_double l_se, JC_double l_ew, JC_double thet_4)
        {
            return (l_se) + (l_ew)*cos(thet_4);
        }

        JC_double n(JC_double l_ew, JC_double thet_4)
        {
            return (l_ew)*sin(thet_4);
        }

        JC_double f_thet2(JC_double l_se, JC_double l_ew, JC_double thet_4, JC_double x_sw_z)
        {
            return atan2(m(l_se, l_ew, thet_4), n(l_ew, thet_4)) - atan2(x_sw_z, sqrt(pow(m(l_se, l_ew, thet_4), 2) + pow(n(l_ew, thet_4), 2) - pow(x_sw_z, 2)));
        }

        JC_double f_thet2_2(JC_double l_se, JC_double l_ew, JC_double thet_4, JC_double x_sw_z)
        {
            return atan2(m(l_se, l_ew, thet_4), n(l_ew, thet_4)) - atan2(x_sw_z, -sqrt(pow(m(l_se, l_ew, thet_4), 2) + pow(n(l_ew, thet_4), 2) - pow(x_sw_z, 2)));
        }
    } // namespace slover_thet2

    namespace slover_thet1
    {

        JC_double o(JC_double thet_2, JC_double l_se, JC_double l_ew, JC_double thet_4)
        {
            return sin(thet_2) * ((l_se) + (l_ew)*cos(thet_4));
        }

        JC_double p(JC_double l_ew, JC_double thet_4, JC_double thet_2)
        {
            return (l_ew)*sin(thet_4) * cos(thet_2);
        }

        JC_double sin_thet1(JC_double thet_2, JC_double l_se, JC_double l_ew, JC_double thet_4, JC_double x_sw_y)
        {
            return x_sw_y * (o(thet_2, l_se, l_ew, thet_4) + p(l_ew, thet_4, thet_2));
        }
        JC_double cos_thet1(JC_double thet_2, JC_double l_se, JC_double l_ew, JC_double thet_4, JC_double x_sw_x)
        {
            return x_sw_x * (o(thet_2, l_se, l_ew, thet_4) + p(l_ew, thet_4, thet_2));
        }

        JC_double f_thet1(JC_double thet_2, JC_double l_se, JC_double l_ew, JC_double thet_4, JC_double x_sw_x, JC_double x_sw_y)
        {
            return atan2(sin_thet1(thet_2, l_se, l_ew, thet_4, x_sw_y), cos_thet1(thet_2, l_se, l_ew, thet_4, x_sw_x));
        }

    } // namespace slover_thet1

    namespace slover_utility
    {
        // %* 旋转矩阵求解,参考冗余书籍p51
        Eigen::Matrix3d rot_matrix(JC_double thet, JC_double alpha)
        {
            Eigen::Matrix3d tem;
            tem << cos(thet), -sin(thet) * cos(alpha), sin(thet) * sin(alpha),
                sin(thet), cos(thet) * cos(alpha), -cos(thet) * sin(alpha),
                0, sin(alpha), cos(alpha);

            return tem;
        }
        // % *-------------------------------

        // % *向量转斜对称矩阵
        Eigen::Matrix3d ssm(JC_double x, JC_double y, JC_double z)
        {
            Eigen::Matrix3d tem;

            tem << 0, -z, y,
                z, 0, -x,
                -y, x, 0;

            return tem;
        }
        // % *------------------------------------------

        // % *+-360转+-180
        JC_double my_convert(JC_double x)
        {
            if (x > KDL::PI)
                return x - 2 * KDL::PI;
            else if (x < -KDL::PI)
                return 2 * KDL::PI + x;
            else if (x == 2 * KDL::PI)
                return 0;
            else if (x == -2 * KDL::PI)
                return 0;
            else
                return x;
        }
        // % *------------------------------------------

        // % *+-360转+-180
        std::array<JC_double, 7> &my_convert(std::array<JC_double, 7> &arg)
        {
            for (int i = 0; i < arg.size(); i++)
            {
                if (arg[i] > KDL::PI)
                    arg[i] = arg[i] - 2 * KDL::PI;
                else if (arg[i] < -KDL::PI)
                    arg[i] = 2 * KDL::PI + arg[i];
            }

            return arg;
        }
        // % *------------------------------------------

        // % *+KDL::rotaion转Eigen::Matrix3d
        Eigen::Matrix3d rot_to_matrix(const KDL::Rotation &arg)
        {
            Eigen::Matrix3d tem;

            for (int row = 0; row < 3; row++)
                for (int cul = 0; cul < 3; cul++)
                    tem(row, cul) = arg(row, cul);

            return tem;
        }
        // % *------------------------------------------

        // % *裁剪浮点数至指定位数
        JC_double roundn(const JC_double &arg, unsigned int precision)
        {
            // thet2 thet4 thet6 在等于0度时，计算结果为0.9999999999999998,因此需要截取
            std::stringstream ss;
            ss << std::setprecision(precision) << arg;

            return std::stod(ss.str());
            // return arg;
        }
        // % *------------------------------------------
        // % *sign函数，和matlab一致，x>0，返回1，=0返回0，<0返回-1
        int JC_sign( JC_double x )
        {
            if ( x > 0 )
                return 1;
            else if ( x < 0 )
                return -1;
            else
                return 0;
        }
        // % *------------------------------------------

        // % *求解腕部点在link_1的X轴的正向还是负向；
        // %status(1)==1表示腕部点在link_1的X轴正向或0点 - status(2)==1表示4关节>=0 - status(3)==1表示6关节>=0
        // %status(1)==0表示腕部点在link_1的X轴负向 - status(2)==0表示4关节取负 - status(3)==0表示6关节取负
        int X_axit_1_6( JC_double j2, JC_double j3, JC_double j4, JC_double d_se, JC_double d_ew )
        {
            int output = JC_sign( d_se * sin( j2 ) + d_ew * cos( j4 ) * sin( j2 ) + d_ew * cos( j2 ) * cos( j3 ) * sin( j4 ) );
            // % 为了和iiwa的status统一
            // %output=1表示腕部点在link_1的X轴正向或0点
            // %output=0表示腕部点在link_1的X轴负向
            return JC_sign( output + 1 );
        }
        // % *------------------------------------------

    } // namespace slover_utility

    int inverse_special_to_SRS::init(const KDL::Chain &dof_7_robot, const KDL::JntArray &pos_minimum, const KDL::JntArray &pos_maximum)
    {
        double RPY[3];
        dof_7_robot.getSegment(1).getFrameToTip().M.GetRPY(RPY[0], RPY[1], RPY[2]);
        joint_1_inverse = -1 * sin(RPY[0]);
        dof_7_robot.getSegment(3).getFrameToTip().M.GetRPY(RPY[0], RPY[1], RPY[2]);
        joint_3_inverse = -1 * sin(RPY[0]);
        dof_7_robot.getSegment(5).getFrameToTip().M.GetRPY(RPY[0], RPY[1], RPY[2]);
        joint_5_inverse = -1 * sin(RPY[0]);

        d_bs = std::abs(dof_7_robot.getSegment(0).getFrameToTip().p[2]); //%base系到shoulder的长度
        l_0_bs = {0, 0, d_bs};
        d_se = std::abs(dof_7_robot.getSegment(2).getFrameToTip().p[1]);
        d_ew = std::abs(dof_7_robot.getSegment(4).getFrameToTip().p[1]);
        d_wt = std::abs(dof_7_robot.getSegment(6).getFrameToTip().p[1]);
        l_7_wt = {0, 0, d_wt};

        for (int i = 0; i < jointNum; i++)
            if (pos_maximum(i) < pos_minimum(i))
            {
                PLOG_ERROR << "关节[" << i << "] 的minimum大于maximum,请检查限位范围";
                return -1;
            }

        _pos_minimum = pos_minimum;
        _pos_maximum = pos_maximum;
        return 0;
    }

    int inverse_special_to_SRS::JC_cartesian_to_joint(const KDL::Frame inter_T, const JC_double inter_joint_3, const KDL::JntArray &last_joint, KDL::JntArray &joint_out) const
    {
        // * 仿真环境设置
        //** 将DH坐标系转换为预定的标准DH **//
        KDL::JntArray _last_joint{last_joint};
        _last_joint(1) = joint_1_inverse * last_joint(1);
        _last_joint(3) = joint_3_inverse * last_joint(3);
        _last_joint(5) = joint_5_inverse * last_joint(5);
        //**-------------------------------**//

        std::vector<std::array<JC_double, 7>> res_thet; // 32组结果
        std::vector<double> joint_offset;               // 32组位置差
        //**-------------------------------**//

        try
        {
            KDL::Vector x_0_7 = inter_T.p;
            KDL::Rotation r_0_7 = inter_T.M;

            KDL::Vector x_0_sw = x_0_7 - l_0_bs - r_0_7 * l_7_wt;

            //! 腕部中心点在1关节轴线上奇异处理：
            if (x_0_sw(0) == 0 && x_0_sw(1) == 0)
                throw JC_exception{"腕部中心点在1关节轴线上奇异", -3};

            JC_double thet_4_tem = acos(slover_utility::roundn((pow(x_0_sw.Norm(), 2) - pow(d_se, 2) - pow(d_ew, 2)) / (2 * d_se * d_ew), 14));

            //! 4关节奇异处理：thet4接近0时->acos(>1)->thet_4_tem=nan
            if (std::isnan(thet_4_tem))
                throw JC_exception{"关节4奇异", -1};

            for (int reference_index = 0; reference_index < (2 + 2 * !(thet_4_tem == 0)); reference_index++)
            {
                JC_double thet_4 = 0;
                JC_double thet_r_2 = 0;

                switch (reference_index)
                {
                case 0:
                    thet_4 = thet_4_tem;
                    thet_r_2 = slover_thet2::f_thet2(d_se, d_ew, thet_4, x_0_sw(2));
                    break;

                case 1:
                    thet_4 = thet_4_tem;
                    thet_r_2 = slover_thet2::f_thet2_2(d_se, d_ew, thet_4, x_0_sw(2));
                    break;

                case 2:
                    thet_4 = -thet_4_tem;
                    thet_r_2 = slover_thet2::f_thet2(d_se, d_ew, thet_4, x_0_sw(2));
                    break;

                case 3:

                    thet_4 = -thet_4_tem;
                    thet_r_2 = slover_thet2::f_thet2_2(d_se, d_ew, thet_4, x_0_sw(2));
                    break;

                default:
                    throw JC_exception{"switch 异常", -2};
                    break;
                }

                JC_double thet_r_1 = slover_thet1::f_thet1(thet_r_2, d_se, d_ew, thet_4, x_0_sw(0), x_0_sw(1));

                Eigen::Matrix3d ref_r_0_3 = slover_utility::rot_matrix(thet_r_1, -KDL::PI_2) * slover_utility::rot_matrix(thet_r_2, KDL::PI_2) * slover_utility::rot_matrix(0, -KDL::PI_2);

                KDL::Vector u_0_sw;
                JC_double vct_len = x_0_sw.Norm();
                if (vct_len < 1e-6)
                    throw JC_exception{"腕部点与肘部点太接近", -8};
                else
                    u_0_sw = x_0_sw / vct_len;

                Eigen::Matrix3d ssm_u_0_sw = slover_utility::ssm(u_0_sw(0), u_0_sw(1), u_0_sw(2));

                Eigen::Matrix3d As = ssm_u_0_sw * ref_r_0_3;
                Eigen::Matrix3d Bs = -1 * ssm_u_0_sw * As;
                Eigen::Matrix3d Cs = (Eigen::Vector3d{u_0_sw(0), u_0_sw(1), u_0_sw(2)} * Eigen::RowVector3d{u_0_sw(0), u_0_sw(1), u_0_sw(2)}) * ref_r_0_3;

                Eigen::Matrix3d r_3_4_transpose = slover_utility::rot_matrix(thet_4, KDL::PI_2).transpose();
                Eigen::Matrix3d matrix_r_0_7 = slover_utility::rot_to_matrix(r_0_7);

                Eigen::Matrix3d Aw = r_3_4_transpose * As.transpose() * matrix_r_0_7;
                Eigen::Matrix3d Bw = r_3_4_transpose * Bs.transpose() * matrix_r_0_7;
                Eigen::Matrix3d Cw = r_3_4_transpose * Cs.transpose() * matrix_r_0_7;

                std::array<JC_double, 2> arm_angle{0, 0};
                {
                    JC_double a_n = As(2, 2);
                    JC_double b_n = Bs(2, 2);
                    JC_double c_n = Cs(2, 2);
                    JC_double a_d = -As(2, 0);
                    JC_double b_d = -Bs(2, 0);
                    JC_double c_d = -Cs(2, 0);

                    JC_double b = tan(inter_joint_3) * a_d - a_n;
                    JC_double a = tan(inter_joint_3) * b_d - b_n;
                    JC_double c = c_n - tan(inter_joint_3) * c_d;
                    arm_angle[0] = 2 * atan2((b + sqrt(pow(b, 2) + pow(a, 2) - pow(c, 2))), (a + c));
                    arm_angle[1] = 2 * atan2((b - sqrt(pow(b, 2) + pow(a, 2) - pow(c, 2))), (a + c));
                }

                JC_double invert_123 = 0;
                JC_double invert_567 = 0;
                JC_double thet1 = 0;
                JC_double thet1_tem = 0;
                JC_double thet2 = 0;
                JC_double thet2_tem = 0;
                JC_double thet3 = 0;
                JC_double thet3_tem = 0;
                JC_double thet4 = 0;
                JC_double thet4_tem = 0;
                JC_double thet5 = 0;
                JC_double thet5_tem = 0;
                JC_double thet6 = 0;
                JC_double thet6_tem = 0;
                JC_double thet7 = 0;
                JC_double thet7_tem = 0;

                for (const auto &interp_arm_angle : arm_angle)
                {
                    for (int inverst_index = 0; inverst_index < 4; inverst_index++)
                    {
                        switch (inverst_index)
                        {
                        case 0:
                            invert_123 = 0;
                            invert_567 = 0;
                            break;

                        case 1:
                            invert_123 = 0;
                            invert_567 = 1;
                            break;

                        case 2:
                            invert_123 = 1;
                            invert_567 = 0;
                            break;

                        case 3:
                            invert_123 = 1;
                            invert_567 = 1;
                            break;

                        default:
                            throw JC_exception{"switch 异常", -2};
                            break;
                        }

                        if (invert_123)
                            thet2 = -1 * thet2_tem;
                        else
                        {
                            JC_double sin_tem = slover_utility::roundn(-As(2, 1) * sin(interp_arm_angle) - Bs(2, 1) * cos(interp_arm_angle) - Cs(2, 1), 14);
                            thet2 = acos(sin_tem);
                            thet2_tem = thet2;
                        }

                        //! 2关节奇异处理
                        if (thet2 == 0)
                            throw JC_exception{"2关节奇异", -4};

                        if (invert_123)
                            thet1 = thet1_tem + KDL::PI;
                        else
                        {
                            JC_double sin_tem = -As(1, 1) * sin(interp_arm_angle) - Bs(1, 1) * cos(interp_arm_angle) - Cs(1, 1);
                            JC_double cos_tem = -As(0, 1) * sin(interp_arm_angle) - Bs(0, 1) * cos(interp_arm_angle) - Cs(0, 1);
                            thet1 = atan2(sin_tem * sin(thet2), cos_tem * sin(thet2));
                            thet1_tem = thet1;
                        }

                        if (invert_123)
                            thet3 = thet3_tem + KDL::PI;
                        else
                        {
                            JC_double sin_tem = As(2, 2) * sin(interp_arm_angle) + Bs(2, 2) * cos(interp_arm_angle) + Cs(2, 2);
                            JC_double cos_tem = -As(2, 0) * sin(interp_arm_angle) - Bs(2, 0) * cos(interp_arm_angle) - Cs(2, 0);
                            thet3 = atan2(sin_tem * sin(thet2), cos_tem * sin(thet2));
                            thet3_tem = thet3;
                        }

                        //! 共32组结果，加上这个限定就16组，和臂角法刚好相等
                        if (std::abs(slover_utility::my_convert(thet3) - inter_joint_3) > 1e-5)
                            continue;

                        if (invert_567)
                            thet6 = -1 * thet6_tem;
                        else
                        {
                            JC_double sin_tem = slover_utility::roundn(Aw(2, 2) * sin(interp_arm_angle) + Bw(2, 2) * cos(interp_arm_angle) + Cw(2, 2), 14);
                            thet6 = acos(sin_tem);
                            thet6_tem = thet6;
                        }

                        //! 6关节奇异处理
                        if (thet6 == 0)
                            throw JC_exception{"6关节奇异", -5};

                        if (invert_567)
                            thet5 = thet5_tem + KDL::PI;
                        else
                        {
                            JC_double sin_tem = Aw(1, 2) * sin(interp_arm_angle) + Bw(1, 2) * cos(interp_arm_angle) + Cw(1, 2);
                            JC_double cos_tem = Aw(0, 2) * sin(interp_arm_angle) + Bw(0, 2) * cos(interp_arm_angle) + Cw(0, 2);
                            thet5 = atan2(sin_tem * sin(thet6), cos_tem * sin(thet6));
                            thet5_tem = thet5;
                        }

                        if (invert_567)
                            thet7 = thet7_tem + KDL::PI;
                        else
                        {
                            JC_double sin_tem = Aw(2, 1) * sin(interp_arm_angle) + Bw(2, 1) * cos(interp_arm_angle) + Cw(2, 1);
                            JC_double cos_tem = -Aw(2, 0) * sin(interp_arm_angle) - Bw(2, 0) * cos(interp_arm_angle) - Cw(2, 0);
                            thet7 = atan2(sin_tem * sin(thet6), cos_tem * sin(thet6));
                            thet7_tem = thet7;
                        }

                        std::array<JC_double, 7> temp1{thet1, thet2, thet3, thet_4, thet5, thet6, thet7};

                        //! 最终检查,16组全检查
                        for (int i = 0; i < 7; i++)
                        {
                            if (std::isnan(temp1[i]))
                            {
                                std::stringstream string;
                                string << "关节[" << i << "]=Nan,结果无效";
                                throw JC_exception{string.str().c_str(), -6};
                            }
                            else if (isinf(temp1[i]))
                            {
                                std::stringstream string;
                                string << "关节[" << i << "]=Inf,结果无效";
                                throw JC_exception{string.str().c_str(), -7};
                            }
                        }

                        bool is_out_of_range{false};
                        slover_utility::my_convert(temp1);
                        for (int i = 0; i < 7; i++)
                            if (temp1[i] > _pos_maximum(i) || temp1[i] < _pos_minimum(i))
                            {
                                // PLOG_WARNING << "关节[" << i << "]超出关节限定范围,结果无效";
                                is_out_of_range = true;
                                break;
                            }

                        if (!is_out_of_range)
                            res_thet.emplace_back(temp1);
                    }
                }
            }

            // PLOG_DEBUG << "res_thet 的size = " << res_thet.size( );
            //** 预防无有效解 **//
            if (res_thet.size() == 0)
            {
                throw JC_exception{"无有效解", -9};
            }
            //**-------------------------------**//

            //** 找到所有结果中最小位移的那一组 **//
            constexpr std::array<JC_double, 7> K_p{0.7, 0.7, 0.7, 0.7, 0.3, 0.3, 0.3};

            for (const auto &iot : res_thet)
            {
                JC_double sum = 0;
                for (int joint_index = 0; joint_index < 7; joint_index++)
                {
                    sum += K_p[joint_index] * std::abs(iot[joint_index] - _last_joint(joint_index));
                }
                joint_offset.emplace_back(sum);
            }
            //**-------------------------------**//

            //** 路径数组存储最小位移的那一组 **//
            std::pair<std::vector<JC_double>::iterator, std::vector<JC_double>::iterator> min_offset_index = std::minmax_element(joint_offset.begin(), joint_offset.end());
            int index_offset = min_offset_index.first - joint_offset.begin();

            for (int i = 0; i < 7; i++)
                joint_out(i) = res_thet[index_offset][i];
            //**-------------------------------**//

            //** 将DH坐标系还原 **//
            joint_out(1) = joint_1_inverse * joint_out(1);
            joint_out(3) = joint_3_inverse * joint_out(3);
            joint_out(5) = joint_5_inverse * joint_out(5);
            //**-------------------------------**//

            return 0;
        }
        catch (const JC_exception &error)
        {
            PLOG_ERROR << error.error_str << ", 错误号：" << error.error_code;
            return error.error_code;
        }
        catch (...)
        {
            PLOG_ERROR << "未知错误";
            return -10;
        }
    }

    int inverse_special_to_SRS::JC_cartesian_to_joint_dir(const KDL::Frame inter_T, const JC_double inter_joint_3, const KDL::JntArray &last_joint, KDL::JntArray &joint_out) const
    {
        // * 仿真环境设置
        //** 将DH坐标系转换为预定的标准DH **//
        KDL::JntArray _last_joint{last_joint};
        _last_joint(1) = joint_1_inverse * last_joint(1);
        _last_joint(3) = joint_3_inverse * last_joint(3);
        _last_joint(5) = joint_5_inverse * last_joint(5);
        //**-------------------------------**//

        std::vector<std::array<JC_double, 7>> res_thet; //!理论只有一组结果
        //**-------------------------------**//

        //** 计算构型**//
        std::array<int ,3> status{};
        status[ 0 ] = slover_utility::X_axit_1_6( _last_joint( 2 - 1 ), _last_joint( 3 - 1 ), _last_joint( 4 - 1 ), d_se, d_ew );
        status[ 1 ] = slover_utility::JC_sign( slover_utility::JC_sign( _last_joint( 4 - 1 ) ) + 1 );
        status[ 2 ] = slover_utility::JC_sign( slover_utility::JC_sign( _last_joint( 6 - 1 ) ) + 1 );
        //**-------------------------------**//

        try
        {
            KDL::Vector x_0_7 = inter_T.p;
            KDL::Rotation r_0_7 = inter_T.M;
            KDL::Vector x_0_sw = x_0_7 - l_0_bs - r_0_7 * l_7_wt;

            //! 腕部中心点在1关节轴线上奇异处理：
            if (x_0_sw(0) == 0 && x_0_sw(1) == 0)
                throw JC_exception{"腕部中心点在1关节轴线上奇异", -3};

            JC_double thet_4_tem = acos(slover_utility::roundn((pow(x_0_sw.Norm(), 2) - pow(d_se, 2) - pow(d_ew, 2)) / (2 * d_se * d_ew), 14));

            //! 4关节奇异处理：thet4接近0时->acos(>1)->thet_4_tem=nan
            if (std::isnan(thet_4_tem))
                throw JC_exception{"关节4奇异", -1};

            for (int reference_index = 0; reference_index < (2 + 2 * !(thet_4_tem == 0)); reference_index++)
            {
                JC_double thet_4 = 0;
                JC_double thet_r_2 = 0;

                switch (reference_index)
                {
                case 0:
                    thet_4 = thet_4_tem;
                    thet_r_2 = slover_thet2::f_thet2(d_se, d_ew, thet_4, x_0_sw(2));
                    break;

                case 1:
                    thet_4 = thet_4_tem;
                    thet_r_2 = slover_thet2::f_thet2_2(d_se, d_ew, thet_4, x_0_sw(2));
                    break;

                case 2:
                    thet_4 = -thet_4_tem;
                    thet_r_2 = slover_thet2::f_thet2(d_se, d_ew, thet_4, x_0_sw(2));
                    break;

                case 3:

                    thet_4 = -thet_4_tem;
                    thet_r_2 = slover_thet2::f_thet2_2(d_se, d_ew, thet_4, x_0_sw(2));
                    break;

                default:
                    throw JC_exception{"switch 异常", -2};
                    break;
                }

                JC_double thet_r_1 = slover_thet1::f_thet1(thet_r_2, d_se, d_ew, thet_4, x_0_sw(0), x_0_sw(1));

                if ( status[ 1 ] != slover_utility::JC_sign( slover_utility::JC_sign( thet_4 ) + 1 ) )
                continue;

                if ( status[ 0 ] != slover_utility::X_axit_1_6( thet_r_2, 0, thet_4, d_se, d_ew ) )
                continue;

                Eigen::Matrix3d ref_r_0_3 = slover_utility::rot_matrix(thet_r_1, -KDL::PI_2) * slover_utility::rot_matrix(thet_r_2, KDL::PI_2) * slover_utility::rot_matrix(0, -KDL::PI_2);

                KDL::Vector u_0_sw;
                JC_double vct_len = x_0_sw.Norm();
                if (vct_len < 1e-6)
                    throw JC_exception{"腕部点与肘部点太接近", -8};
                else
                    u_0_sw = x_0_sw / vct_len;

                Eigen::Matrix3d ssm_u_0_sw = slover_utility::ssm(u_0_sw(0), u_0_sw(1), u_0_sw(2));

                Eigen::Matrix3d As = ssm_u_0_sw * ref_r_0_3;
                Eigen::Matrix3d Bs = -1 * ssm_u_0_sw * As;
                Eigen::Matrix3d Cs = (Eigen::Vector3d{u_0_sw(0), u_0_sw(1), u_0_sw(2)} * Eigen::RowVector3d{u_0_sw(0), u_0_sw(1), u_0_sw(2)}) * ref_r_0_3;

                Eigen::Matrix3d r_3_4_transpose = slover_utility::rot_matrix(thet_4, KDL::PI_2).transpose();
                Eigen::Matrix3d matrix_r_0_7 = slover_utility::rot_to_matrix(r_0_7);

                Eigen::Matrix3d Aw = r_3_4_transpose * As.transpose() * matrix_r_0_7;
                Eigen::Matrix3d Bw = r_3_4_transpose * Bs.transpose() * matrix_r_0_7;
                Eigen::Matrix3d Cw = r_3_4_transpose * Cs.transpose() * matrix_r_0_7;

                std::array<JC_double, 2> arm_angle{0, 0};
                {
                    JC_double a_n = As(2, 2);
                    JC_double b_n = Bs(2, 2);
                    JC_double c_n = Cs(2, 2);
                    JC_double a_d = -As(2, 0);
                    JC_double b_d = -Bs(2, 0);
                    JC_double c_d = -Cs(2, 0);

                    JC_double b = tan(inter_joint_3) * a_d - a_n;
                    JC_double a = tan(inter_joint_3) * b_d - b_n;
                    JC_double c = c_n - tan(inter_joint_3) * c_d;
                    arm_angle[0] = 2 * atan2((b + sqrt(pow(b, 2) + pow(a, 2) - pow(c, 2))), (a + c));
                    arm_angle[1] = 2 * atan2((b - sqrt(pow(b, 2) + pow(a, 2) - pow(c, 2))), (a + c));
                }

                JC_double invert_123 = 0;
                JC_double invert_567 = 0;
                JC_double thet1 = 0;
                JC_double thet1_tem = 0;
                JC_double thet2 = 0;
                JC_double thet2_tem = 0;
                JC_double thet3 = 0;
                JC_double thet3_tem = 0;
                JC_double thet4 = 0;
                JC_double thet4_tem = 0;
                JC_double thet5 = 0;
                JC_double thet5_tem = 0;
                JC_double thet6 = 0;
                JC_double thet6_tem = 0;
                JC_double thet7 = 0;
                JC_double thet7_tem = 0;

                for (const auto &interp_arm_angle : arm_angle)
                {
                    for (int inverst_index = 0; inverst_index < 4; inverst_index++)
                    {
                        switch (inverst_index)
                        {
                        case 0:
                            invert_123 = 0;
                            invert_567 = 0;
                            break;

                        case 1:
                            invert_123 = 0;
                            invert_567 = 1;
                            break;

                        case 2:
                            invert_123 = 1;
                            invert_567 = 0;
                            break;

                        case 3:
                            invert_123 = 1;
                            invert_567 = 1;
                            break;

                        default:
                            throw JC_exception{"switch 异常", -2};
                            break;
                        }

                        if (invert_123)
                            thet2 = -1 * thet2_tem;
                        else
                        {
                            JC_double sin_tem = slover_utility::roundn(-As(2, 1) * sin(interp_arm_angle) - Bs(2, 1) * cos(interp_arm_angle) - Cs(2, 1), 14);
                            thet2 = acos(sin_tem);
                            thet2_tem = thet2;
                        }

                        //! 2关节奇异处理
                        if (thet2 == 0)
                            throw JC_exception{"2关节奇异", -4};

                        if (invert_123)
                            thet1 = thet1_tem + KDL::PI;
                        else
                        {
                            JC_double sin_tem = -As(1, 1) * sin(interp_arm_angle) - Bs(1, 1) * cos(interp_arm_angle) - Cs(1, 1);
                            JC_double cos_tem = -As(0, 1) * sin(interp_arm_angle) - Bs(0, 1) * cos(interp_arm_angle) - Cs(0, 1);
                            thet1 = atan2(sin_tem * sin(thet2), cos_tem * sin(thet2));
                            thet1_tem = thet1;
                        }

                        if (invert_123)
                            thet3 = thet3_tem + KDL::PI;
                        else
                        {
                            JC_double sin_tem = As(2, 2) * sin(interp_arm_angle) + Bs(2, 2) * cos(interp_arm_angle) + Cs(2, 2);
                            JC_double cos_tem = -As(2, 0) * sin(interp_arm_angle) - Bs(2, 0) * cos(interp_arm_angle) - Cs(2, 0);
                            thet3 = atan2(sin_tem * sin(thet2), cos_tem * sin(thet2));
                            thet3_tem = thet3;
                        }

                        //! 共32组结果，加上这个限定就16组，和臂角法刚好相等
                        //! 这里主要判断两个arm_angle，哪一个是正确可以求解出inter_joint_3
                        if ( std::abs( slover_utility::my_convert( thet3 ) - inter_joint_3 ) > 1e-5 )
                            continue;

                        if ( status[ 0 ] != slover_utility::X_axit_1_6( thet2, thet3, thet_4, d_se, d_ew ) )
                            continue;

                        if (invert_567)
                            thet6 = -1 * thet6_tem;
                        else
                        {
                            JC_double sin_tem = slover_utility::roundn(Aw(2, 2) * sin(interp_arm_angle) + Bw(2, 2) * cos(interp_arm_angle) + Cw(2, 2), 14);
                            thet6 = acos(sin_tem);
                            thet6_tem = thet6;
                        }

                        //! 6关节奇异处理
                        if (thet6 == 0)
                            throw JC_exception{"6关节奇异", -5};

                        if (invert_567)
                            thet5 = thet5_tem + KDL::PI;
                        else
                        {
                            JC_double sin_tem = Aw(1, 2) * sin(interp_arm_angle) + Bw(1, 2) * cos(interp_arm_angle) + Cw(1, 2);
                            JC_double cos_tem = Aw(0, 2) * sin(interp_arm_angle) + Bw(0, 2) * cos(interp_arm_angle) + Cw(0, 2);
                            thet5 = atan2(sin_tem * sin(thet6), cos_tem * sin(thet6));
                            thet5_tem = thet5;
                        }

                        if (invert_567)
                            thet7 = thet7_tem + KDL::PI;
                        else
                        {
                            JC_double sin_tem = Aw(2, 1) * sin(interp_arm_angle) + Bw(2, 1) * cos(interp_arm_angle) + Cw(2, 1);
                            JC_double cos_tem = -Aw(2, 0) * sin(interp_arm_angle) - Bw(2, 0) * cos(interp_arm_angle) - Cw(2, 0);
                            thet7 = atan2(sin_tem * sin(thet6), cos_tem * sin(thet6));
                            thet7_tem = thet7;
                        }

                        // if ( thet6 == 0 && invert_567 )
                        //     continue;
                        if ( status[ 2 ] != slover_utility::JC_sign( slover_utility::JC_sign( thet6 ) + 1 ) )
                            continue;

                        std::array<JC_double, 7> temp1{thet1, thet2, thet3, thet_4, thet5, thet6, thet7};

                        for (int i = 0; i < 7; i++)
                        {
                            if (std::isnan(temp1[i]))
                            {
                                std::stringstream string;
                                string << "关节[" << i << "]=Nan,结果无效";
                                throw JC_exception{string.str().c_str(), -6};
                            }
                            else if (isinf(temp1[i]))
                            {
                                std::stringstream string;
                                string << "关节[" << i << "]=Inf,结果无效";
                                throw JC_exception{string.str().c_str(), -7};
                            }
                        }

                        bool is_out_of_range{false};
                        slover_utility::my_convert(temp1);
                        for (int i = 0; i < 7; i++)
                            if (temp1[i] > _pos_maximum(i) || temp1[i] < _pos_minimum(i))
                            {
                                // PLOG_WARNING << "关节[" << i << "]超出关节限定范围,结果无效";
                                is_out_of_range = true;
                                break;
                            }

                        if (!is_out_of_range)
                            res_thet.emplace_back(temp1);
                    }
                }
            }

            // PLOG_DEBUG << "res_thet 的size = " << res_thet.size( );
            //** 预防无有效解 **//
            if ( res_thet.size( ) == 0 )
            {
                throw JC_exception{ "无有效解", -9 };
            }
            //**-------------------------------**//

            //** 只可能有一组解 **//
            if ( res_thet.size( ) != 1 )
            {
                throw JC_exception{ "结果多解无效", -10 };
            }
            //**-------------------------------**//


            for (int i = 0; i < 7; i++)
                joint_out(i) = res_thet.back()[i];

            //** 将DH坐标系还原 **//
            joint_out(1) = joint_1_inverse * joint_out(1);
            joint_out(3) = joint_3_inverse * joint_out(3);
            joint_out(5) = joint_5_inverse * joint_out(5);
            //**-------------------------------**//

            return 0;
        }
        catch (const JC_exception &error)
        {
            PLOG_ERROR << error.error_str << ", 错误号：" << error.error_code;
            return error.error_code;
        }
        catch (...)
        {
            PLOG_ERROR << "未知错误";
            return -10;
        }
    }

    int union_cartesian_to_joint( rocos::Robot* robot_ptr, const union_frame& var, const KDL::JntArray& joint_current, KDL::JntArray& q_target )
    {
        int axis_num = robot_ptr->getJointNum( );
        if ( axis_num == 7 )//7自由度情况逆解
        {
            KDL::Frame interp_frame = var.target_7axis.first;
            double interp_J3        = var.target_7axis.second;

            if ( robot_ptr->SRS_kinematics_.JC_cartesian_to_joint_dir( interp_frame, interp_J3, joint_current, q_target ) < 0 )
            {
                PLOG_ERROR << "目标点位不可达";
                return -1;
            }
        }
        else if ( axis_num == 6 )//6自由度情况逆解
        {
            KDL::Frame interp_frame = var.target_6axis;
            if ( robot_ptr->kinematics_.CartToJnt( joint_current, interp_frame, q_target ) < 0 )
            {
                PLOG_ERROR << "目标点位不可达";
                return -1;
            }
        }
        else//其他自由度情况逆解
        {
            KDL::Frame interp_frame = var.target_6axis;
            if ( robot_ptr->kinematics_.CartToJnt( joint_current, interp_frame, q_target ) < 0 )
            {
                PLOG_ERROR << "目标点位不可达";
                return -1;
            }
        };

        return 0;
    }

#pragma endregion

    void Joint_stop(rocos::Robot *robot_ptr, const KDL::JntArray &current_pos, const KDL::JntArray &last_pos, const KDL::JntArray &last_last_pos)
    {
        //** 变量初始化 **//
        ruckig::Ruckig<ruckig::DynamicDOFs> otg{jointNum, 0.001};
        ruckig::InputParameter<ruckig::DynamicDOFs> input(jointNum);
        ruckig::OutputParameter<ruckig::DynamicDOFs> output(jointNum);
        ruckig::Result res;
        //**-------------------------------**//

        try
        {
            KDL::JntArray current_vel(jointNum);
            KDL::JntArray last_vel(jointNum);
            KDL::JntArray current_acc(jointNum);

            KDL::Subtract(current_pos, last_pos, current_vel);
            KDL::Divide(current_vel, 0.001, current_vel);

            KDL::Subtract(last_pos, last_last_pos, last_vel);
            KDL::Divide(last_vel, 0.001, last_vel);

            KDL::Subtract(current_vel, last_vel, current_acc);
            KDL::Divide(current_acc, 0.001, current_acc);

            input.control_interface = ruckig::ControlInterface::Velocity;
            input.synchronization = ruckig::Synchronization::None;

            for (int i = 0; i < jointNum; i++)
            {
                input.current_position[i] = robot_ptr->pos_[i];
                input.current_velocity[i] = KDL::sign(current_vel(i)) * std::min(abs(current_vel(i)), robot_ptr->max_vel_[i]);
                input.current_acceleration[i] = KDL::sign(current_acc(i)) * std::min(abs(current_acc(i)), robot_ptr->max_acc_[i]);

                input.target_position[i] = input.current_position[i];
                input.target_velocity[i] = 0;
                input.target_acceleration[i] = 0;

                input.max_velocity[i] = robot_ptr->joints_[i]->getMaxVel();
                input.max_acceleration[i] = robot_ptr->joints_[i]->getMaxAcc();
                input.max_jerk[i] = robot_ptr->joints_[i]->getMaxJerk();
            }

            while ((res = otg.update(input, output)) == ruckig::Result::Working)
            {
                safety_servo(robot_ptr, output.new_position);
                output.pass_to_input(input);
            }

            if (res != ruckig::Result::Finished)
            {
                //                PLOG_ERROR << "OTG 计算失败,停止运动";
                for (int i = 0; i < jointNum; ++i)
                    robot_ptr->joints_[i]->setPosition(robot_ptr->pos_[i]);
            }
            else
            {
                //                PLOG_INFO << "关节空间急停已完成";
            }
        }
        catch (const std::exception &e)
        {
            PLOG_ERROR << e.what();
            for (int i = 0; i < jointNum; ++i)
                robot_ptr->joints_[i]->setPosition(robot_ptr->pos_[i]);
        }
        catch (...)
        {
            PLOG_ERROR << "未知错误";
            for (int i = 0; i < jointNum; ++i)
                robot_ptr->joints_[i]->setPosition(robot_ptr->pos_[i]);
        }
    }

    int check_vel_acc(const KDL::JntArray &current_pos, const KDL::JntArray &last_pos, const KDL::JntArray &last_last_pos, const double max_vel, const double max_acc)
    {
        KDL::JntArray current_vel(jointNum);
        KDL::JntArray last_vel(jointNum);
        KDL::JntArray current_acc(jointNum);

        KDL::Subtract(current_pos, last_pos, current_vel);
        KDL::Divide(current_vel, 0.001, current_vel);

        KDL::Subtract(last_pos, last_last_pos, last_vel);
        KDL::Divide(last_vel, 0.001, last_vel);

        KDL::Subtract(current_vel, last_vel, current_acc);
        KDL::Divide(current_acc, 0.001, current_acc);

        for (int i{0}; i < jointNum; i++)
        {
            if (abs(current_vel(i)) > max_vel)
            {
                PLOG_ERROR << "joint[" << i << "] velocity is too  fast";
                PLOG_ERROR << "target velocity = " << current_vel(i) * KDL::rad2deg;
                PLOG_ERROR << "max velocity=" << max_vel * KDL::rad2deg;
                return -1;
            }

            if (abs(current_acc(i)) > max_acc)
            {
                PLOG_ERROR << "joint[" << i << "] acceleration is too  fast";
                PLOG_ERROR << "target acceleration = " << current_acc(i) * KDL::rad2deg;
                PLOG_ERROR << "max acceleration=" << max_acc * KDL::rad2deg;
                return -1;
            }
        }
        return 0;
    }

    int safety_servo(rocos::Robot *robot_ptr, const std::vector<double> &target_pos)
    {
        //** 伺服位置检查，无效则报错并程序终止 **//
        for (int i = 0; i < jointNum; ++i)
        {
            if (target_pos[i] > robot_ptr->joints_[i]->getMaxPosLimit() || target_pos[i] < robot_ptr->joints_[i]->getMinPosLimit())
            {
                PLOG_ERROR << "joint [" << i + 1 << "] ( " << target_pos[i] * KDL::rad2deg << " ) is out of range ";
                //                PLOG_ERROR << "program will be turn off after 4 seconds!!";
                //                std::this_thread::sleep_for( std::chrono::duration< double >( 4 ) );
                //                exit( -1 );
                return -1;
            }
        }
        //**-------------------------------**//

        //** 位置伺服 **//
        for (int i = 0; i < jointNum; ++i)
        {
            robot_ptr->pos_[i] = target_pos[i];
            robot_ptr->joints_[i]->setPosition(target_pos[i]);
        }
        robot_ptr->hw_interface_->waitForSignal(0);
        //**-------------------------------**//
        return 0;
    }

    int safety_servo(rocos::Robot *robot_ptr, const KDL::JntArray &target_pos)
    {
        //** 伺服位置检查，无效则报错并程序终止 **//
        for (int i = 0; i < jointNum; ++i)
        {
            if (target_pos(i) > robot_ptr->joints_[i]->getMaxPosLimit() || target_pos(i) < robot_ptr->joints_[i]->getMinPosLimit())
            {
                PLOG_ERROR << "joint [" << i + 1 << "] ( " << target_pos(i) * KDL::rad2deg << " ) is out of range ";
                //                PLOG_ERROR << "program will be turn off after 4 seconds!!";
                //                std::this_thread::sleep_for( std::chrono::duration< double >( 4 ) );
                //                exit( -1 );
                return -1;
            }
        }
        //**-------------------------------**//

        //** 位置伺服 **//
        for (int i = 0; i < jointNum; ++i)
        {
            robot_ptr->pos_[i] = target_pos(i);
            robot_ptr->joints_[i]->setPosition(target_pos(i));
        }
        robot_ptr->hw_interface_->waitForSignal(0);
        //**-------------------------------**//
        return 0;
    }
} // namespace JC_helper
