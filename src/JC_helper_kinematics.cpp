#include <rocos_app/JC_helper_kinematics.hpp>
#include <rocos_app/robot.h>

//** 显式指定关节数量 **//
size_t _joint_num { 0 };

namespace JC_helper
{
#pragma region  //* 轨迹计算函数

    /**
     * @brief 姿态插值（四元素球面线性插值）
     * @note 注意没有s的范围保护
     * @param start 开始姿态
     * @param end 结束姿态
     * @param s 百分比
     * @return std::vector< double > 四元素x,y,z,w
     */
    std::vector< double > UnitQuaternion_intep( const std::vector< double >& start,
                                                const std::vector< double >& end,
                                                double s )
    {
        constexpr double eps = 1E-7;

        if ( s < 0 || s > 1 )
        {
            PLOG_ERROR << "UnitQuaternion interpolation failure";
        }

        double cosTheta = start[ 0 ] * end[ 0 ] + start[ 1 ] * end[ 1 ] + start[ 2 ] * end[ 2 ] +
                          start[ 3 ] * end[ 3 ];

        std::vector< double > start_2 = start;

        //** 这里是为了取最短路径 **//
        if ( cosTheta < 0 )
        {
            for ( int i = 0; i < 4; i++ ) start_2[ i ] *= -1;
            cosTheta *= -1;
        }
        //**-------------------------------**//

        double theta = acos( cosTheta );

        //! theta本应该计算为0，但是实际可能为0.000001，即使有(eps=1E-7)限定范围，仍然有误判可能,所以最好使用isnan()
        if ( abs( theta ) < eps || s == 0 )
            return start_2;
        else
        {
            double coefficient_1 = sin( ( 1 - s ) * theta ) / sin( theta );
            double coefficient_2 = sin( ( s )*theta ) / sin( theta );

            std::vector< double > res{
                coefficient_1 * start_2[ 0 ] + coefficient_2 * end[ 0 ],
                coefficient_1 * start_2[ 1 ] + coefficient_2 * end[ 1 ],
                coefficient_1 * start_2[ 2 ] + coefficient_2 * end[ 2 ],
                coefficient_1 * start_2[ 3 ] + coefficient_2 * end[ 3 ] };

            //! 防止误判
            for ( const auto& var : res )
                if ( isnan( var ) )
                    return start_2;

            return res;
        }
    }

    KDL::Rotation RotAxisAngle( KDL::Rotation start, KDL::Rotation end, double s )
    {
        if ( s > 1 || s < 0 )
        {
            std::cerr << RED << "values of S outside interval [0,1]" << GREEN << std::endl;
        }

        KDL::Rotation R_start_end = start.Inverse( ) * end;
        KDL::Vector axis;
        double angle = R_start_end.GetRotAngle( axis );
        return start * KDL::Rotation::Rot2( axis, angle * s );
    }

    KDL::Frame circle( const KDL::Frame& F_base_circlestart, const KDL::Frame& F_base_circleend, const KDL::Frame& F_base_circleCenter, double s_p, double s_r, double alpha, bool& success )
    {
        using namespace KDL;

        if ( s_p > 1 || s_p < 0 )
        {
            PLOG_ERROR << "values of s_p outside interval [0,1]";
            success = false;
            return KDL::Frame{ };
        }
        if ( s_r > 1 || s_r < 0 )
        {
            PLOG_ERROR << "values of s_r outside interval [0,1]";
            success = false;
            return KDL::Frame{ };
        }

        KDL::Vector x    = F_base_circlestart.p - F_base_circleCenter.p;
        double radius    = x.Normalize( );
        KDL::Vector tmpv = F_base_circleend.p - F_base_circleCenter.p;  // 第二直线段上的半径段
        tmpv.Normalize( );

        Vector z( x * tmpv );  // Z轴
        double n = z.Normalize( );

        if ( n < epsilon )
        {
            std::cerr << RED << "circle(): Z Axis Calculation error " << GREEN << std::endl;
            success = false;
            return KDL::Frame{ };
        }

        KDL::Frame F_base_circleCenter_( KDL::Rotation{ x, ( z * x ), z }, F_base_circleCenter.p );
        std::vector< double > Quaternion_start{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_end{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_interp{ 0, 0, 0, 0 };
        F_base_circlestart.M.GetQuaternion( Quaternion_start.at( 0 ), Quaternion_start.at( 1 ), Quaternion_start.at( 2 ), Quaternion_start.at( 3 ) );
        F_base_circleend.M.GetQuaternion( Quaternion_end.at( 0 ), Quaternion_end.at( 1 ), Quaternion_end.at( 2 ), Quaternion_end.at( 3 ) );
        Quaternion_interp = UnitQuaternion_intep( Quaternion_start, Quaternion_end, s_r );

        success = true;
        return KDL::Frame( KDL::Rotation::Quaternion( Quaternion_interp[ 0 ], Quaternion_interp[ 1 ], Quaternion_interp[ 2 ], Quaternion_interp[ 3 ] ), F_base_circleCenter_* Vector{ radius * cos( s_p * alpha ), radius * sin( s_p * alpha ), 0 } );
    }

    int link_pos( const KDL::Frame& start, const KDL::Frame& end, double s_p, double s_r, KDL::Frame& Cartesian_pos )
    {
        if ( s_p > 1 || s_p < 0 )
        {
            PLOG_ERROR << "values of s_p outside interval [0,1]";
            return -1;
        }
        if ( s_r > 1 || s_r < 0 )
        {
            PLOG_ERROR << "values of s_r outside interval [0,1]";
            return -1;
        }

        //** 变量初始化 **//
        Cartesian_pos.p = start.p + ( end.p - start.p ) * s_p;

        std::vector< double > Quaternion_start{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_end{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_interp{ 0, 0, 0, 0 };
        start.M.GetQuaternion( Quaternion_start.at( 0 ), Quaternion_start.at( 1 ), Quaternion_start.at( 2 ), Quaternion_start.at( 3 ) );
        end.M.GetQuaternion( Quaternion_end.at( 0 ), Quaternion_end.at( 1 ), Quaternion_end.at( 2 ), Quaternion_end.at( 3 ) );
        Quaternion_interp = UnitQuaternion_intep( Quaternion_start, Quaternion_end, s_r );
        Cartesian_pos.M   = KDL::Rotation::Quaternion( Quaternion_interp[ 0 ], Quaternion_interp[ 1 ], Quaternion_interp[ 2 ], Quaternion_interp[ 3 ] );

        return 0;
    }

    int link_vel( const KDL::Frame& start, const KDL::Frame& end, double v_p, double v_r, KDL::Twist& Cartesian_vel )
    {
        if ( v_p < 0 )
        {
            PLOG_ERROR << "values of v_p is less than 0";
            return -1;
        }
        if ( v_r < 0 )
        {
            PLOG_ERROR << "values of v_r is less than 0";
            return -1;
        }

        Cartesian_vel.vel = ( end.p - start.p ) * v_p;

        KDL::Rotation rot_start_end = start.M.Inverse( ) * end.M;
        KDL::Vector aixs            = rot_start_end.GetRot( );
        Cartesian_vel.rot           = aixs * v_r;

        return 0;
    }

    int link_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& start, const KDL::Frame& end, double v_start, double v_end, double max_path_v, double max_path_a )
    {
        if ( end == start )  // 起始和终止位置一致，无需规划
            return 0;
        else
        {
            //** 变量初始化 **//
            KDL::Vector Pstart        = start.p;
            KDL::Vector Pend          = end.p;
            double Plength            = ( Pend - Pstart ).Norm( );
            KDL::Rotation R_start_end = start.M.Inverse( ) * end.M;
            KDL::Vector ration_axis;
            double angle                   = R_start_end.GetRotAngle( ration_axis );
            const double equivalent_radius = 0.1;  // TODO: 等效半径，但是具体应该是什么值可以考虑
            double Rlength                 = equivalent_radius * abs( angle );
            double Path_length{ 0 };

            double T_link{ 0 };
            ::rocos::DoubleS doubleS_P;
            ::rocos::DoubleS doubleS_R;
            //**-------------------------------**//

            // 大旋转，小移动(或没移动)情况，此时要求起始速度为0
            if ( Rlength > Plength )
            {
                if ( v_start != 0 || v_end != 0 )
                {
                    PLOG_ERROR << "Error_MotionPlanning_Not_Feasible";
                    return -1;
                }
                Path_length = Rlength;
            }
            // 大移动，小旋转
            else
                Path_length = Plength;

            doubleS_P.planDoubleSProfile( 0, 0, 1, v_start / Path_length, v_end / Path_length, max_path_v / Path_length, max_path_a / Path_length, max_path_a * 2 / Path_length );
            bool isplanned = doubleS_P.isValidMovement( );
            if (!isplanned || doubleS_P.getDuration() <= 0)
            {
                PLOG_ERROR << "Error_MotionPlanning_Not_Feasible";
                return -1;
            }

            doubleS_R.planDoubleSProfile( 0, 0, 1, 0, 0, max_path_v / Path_length, max_path_a / Path_length, max_path_a * 2 / Path_length );
            isplanned = doubleS_R.isValidMovement( );
            if (!isplanned || doubleS_R.getDuration() <= 0)
            {
                PLOG_ERROR << "Error_MotionPlanning_Not_Feasible";
                return -1;
            }

            if ( doubleS_R.getDuration( ) != doubleS_P.getDuration( ) )
            {
                doubleS_R.JC_scaleToDuration( doubleS_P.getDuration( ) );
            }

            T_link = doubleS_P.getDuration( );

            double dt  = 0;
            double s_p = 0;
            double s_r = 0;

            KDL::Frame link_target{ };

            //** 轨迹计算 **//
            while ( dt >= 0 && dt <= T_link )
            {
                s_p = doubleS_P.pos( dt );
                s_r = doubleS_R.pos( dt );

                if ( link_pos( start, end, s_p, s_r, link_target ) < 0 )
                {
                    PLOG_ERROR << " link calculating failure";
                    return -1;
                }
                traj.push_back( link_target );
                dt = dt + DELTA_T;
            }
            //**-------------------------------**//
            return 0;
        }
    }

    int link_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& start, const KDL::Frame& end, double max_path_v, double max_path_a )
    {
        //** 变量初始化 **//
        KDL::Vector Pstart        = start.p;
        KDL::Vector Pend          = end.p;
        double Plength            = ( Pend - Pstart ).Norm( );
        KDL::Rotation R_start_end = start.M.Inverse( ) * end.M;
        KDL::Vector ration_axis;
        double angle                   = R_start_end.GetRotAngle( ration_axis );
        const double equivalent_radius = 0.1;
        double Rlength                 = ( equivalent_radius * abs( angle ) );
        double Path_length             = std::max( Plength, Rlength );
        double s                       = 0;
        ::rocos::DoubleS doubleS;
        //**-------------------------------**//

        doubleS.planProfile( 0, 0.0, 1.0, 0, 0, max_path_v / Path_length, max_path_a / Path_length,
                             max_path_a * 2 / Path_length );

        if ( !doubleS.isValidMovement( ) || !( doubleS.getDuration( ) > 0 ) )
        {
            PLOG_ERROR << "link trajectory "
                       << "is infeasible ";
            return -1;
        }

        //** 变量初始化 **//
        double dt       = 0;
        double duration = doubleS.getDuration( );
        KDL::Frame interp_frame{ };
        //**-------------------------------**//

        //** 轨迹计算 **//
        while ( dt <= duration )
        {
            s = doubleS.pos( dt );

            if ( link_pos( start, end, s, s, interp_frame ) < 0 )
            {
                PLOG_ERROR << " link calculating failure";
                return -1;
            }
            traj.push_back( interp_frame );
            dt += DELTA_T;
        }
        //**-------------------------------**//

        return 0;
    }

    int link_trajectory( std::vector< KDL::Twist >& traj_vel, const KDL::Frame& start, const KDL::Frame& end, double max_path_v, double max_path_a )
    {
        //** 变量初始化 **//
        double Plength            = ( end.p - start.p ).Norm( );
        KDL::Rotation R_start_end = start.M.Inverse( ) * end.M;
        KDL::Vector ration_axis;
        double angle                   = R_start_end.GetRotAngle( ration_axis );
        const double equivalent_radius = 0.1;
        double Rlength                 = ( equivalent_radius * abs( angle ) );
        double Path_length             = std::max( Plength, Rlength );
        double doubles_vel             = 0;
        ::rocos::DoubleS doubleS;
        //**-------------------------------**//

        doubleS.planProfile( 0, 0.0, 1.0, 0, 0, max_path_v / Path_length, max_path_a / Path_length,
                             max_path_a * 2 / Path_length );

        if ( !doubleS.isValidMovement( ) || !( doubleS.getDuration( ) > 0 ) )
        {
            PLOG_ERROR << "link trajectory "
                       << "is infeasible ";
            return -1;
        }

        //** 变量初始化 **//
        double dt       = 0;
        double duration = doubleS.getDuration( );
        KDL::Twist interp_vel{ };

        //**-------------------------------**//

        //** 轨迹计算 **//
        while ( dt <= duration )
        {
            doubles_vel = doubleS.vel( dt );

            if ( link_vel( start, end, doubles_vel, doubles_vel, interp_vel ) < 0 )
            {
                PLOG_ERROR << " link calculating failure";
                return -1;
            }

            traj_vel.push_back( interp_vel );
            dt += DELTA_T;
        }
        //**-------------------------------**//

        return 0;
    }

    int multilink_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& f_start, const KDL::Frame& f_mid, const KDL::Frame& f_end, KDL::Frame& next_f_start, double current_path_start_v, double& next_path_start_v, double s_bound_dist, double max_path_v, double max_path_a, double next_max_path_v )
    {
        using namespace KDL;
        //** 变量初始化 **//
        constexpr double eps = 1E-7;
        Vector p1{ f_start.p };
        Vector p_mid{ f_mid.p };
        Vector p2{ f_end.p };

        Vector ab = p_mid - p1;
        Vector bc = p2 - p_mid;

        double abdist = ab.Norm( );
        double bcdist = bc.Norm( );

        // bound_dist不再支持绝对距离，使用相对距离更合适
        double bound_dist = std::max( std::min( abs( s_bound_dist ), 1.0 ), 0.0 ) * std::min( abdist, bcdist );
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

        if ( abdist < eps )  // 说明两种情况：①本段只旋转，不移动，要求开始速度为0 ②上段bound_dist等于abdist等于bcdist.只有圆弧运动，要求结束速度不为0，既本段开始速度不为0
        {
            //! 目前第②情况不应该出现，或者很不好处理，选择报错处理
            //! 第①种情况可以处理
            if ( current_path_start_v != 0 )
            {
                PLOG_ERROR << "bound_dist of last motion  is too  large,try to decrease it";
                return -1;
            }
        }

        if ( current_path_start_v < eps && abdist > eps && ( abdist - bound_dist ) < eps )  // 只存在圆弧段，且上段速度为0，在圆弧匀速约束这不允许
        {
            PLOG_ERROR << "the Link length is not allowed to be equal to 0,When starting velocity of this motion is equal to 0,try to increase bound_dist " << GREEN << std::endl;
            return -1;
        }

        if ( bound_dist > abdist )  // 这个条件说明：本段只旋转，不移动时一定保证bound_dist=0
        {
            if ( abdist < eps )
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

        if ( bound_dist > bcdist )  // 这个条件说明：【正常情况下】如果下一段只旋转，不移动，则本段bound_dist必需=0(因为bcdist=0)，则本段结束速度一定为0
        {
            PLOG_ERROR << "bound_dist is too  large,try to decrease it";
            return -1;
        }

        if ( current_path_start_v > max_path_v )  //! 防止起始速度就超过最大可达线速度 ,理论上不可能发生
        {
            PLOG_ERROR << " current_path_start_v  > max_path_v";
            PLOG_ERROR << "current_path_start_v = " << current_path_start_v;
            PLOG_ERROR << " max_path_v = " << max_path_v << GREEN;
            return -1;
        }

        if ( bound_dist < 0 )  // bound_dist必需为正数
        {
            PLOG_ERROR << "bound_dist must be positive ";
            return -1;
        }

        //**-------------------------------**//

        // 利用向量乘积公式，求得两向量的夹角 ,并且限定范围在0-180
        double cos_alpha = std::max( -1., std::min( dot( ab, bc ) / abdist / bcdist, 1. ) );
        // 两段直线夹角接近0,则两段直线合并为一条处理
        if ( ( 1 - cos_alpha ) <= eps )
        {
            next_f_start      = f_start;
            next_path_start_v = current_path_start_v;
            return 0;
        }
        // 两段直线夹角接近180，则不允许圆弧过渡
        else if ( ( cos_alpha - ( -1 ) ) <= eps )
            bound_dist = 0;

        // 求解两段直线的夹角和圆弧的半径
        double alpha  = acos( cos_alpha );
        double radius = bound_dist * tan( ( M_PI - alpha ) / 2 );

        // 求解过渡半径占总长的百分比
        double s_bound_dist_1 = bound_dist < eps ? 0 : bound_dist / abdist;
        double s_bound_dist_2 = bound_dist < eps ? 0 : bound_dist / bcdist;  // 避免只旋转时出现0/0

        Frame F_base_circlestart;
        if ( link_pos( f_start, f_mid, 1 - s_bound_dist_1, 1 - s_bound_dist_1, F_base_circlestart ) < 0 )
        {
            PLOG_ERROR << "link calculation failure";
            return -1;
        }

        Frame F_base_circleend;
        if ( link_pos( f_mid, f_end, s_bound_dist_2, s_bound_dist_2, F_base_circleend ) < 0 )
        {
            PLOG_ERROR << "link calculation failure";
            return -1;
        }

        Vector de     = F_base_circlestart.p - f_start.p;  // 实际要走的直线段的距离
        double dedist = de.Norm( );                        // 实际要走的直线段的距离

        Vector V_base_t = ab * ( ab * bc );                // 圆弧中垂直于第一段直线的半径向量
        V_base_t.Normalize( );

        Frame F_base_circleCenter{ F_base_circlestart.p - V_base_t * radius };

        // double s_cirlular_v{ 0 };
        double cirlular_v{ 0 };                       // 应该使用绝对速度

        if ( s_bound_dist_1 <= eps )                  // 不存在圆弧，圆弧速度为0
            cirlular_v = 0;
        else if ( abs( 1 - s_bound_dist_1 ) <= eps )  // 为啥不加abs呢？理由：s_bound_dist_1的范围为：[0-1),没有加的必要
            cirlular_v = current_path_start_v;        // 不存在直线，圆弧速度为当前段运动速度
        else
            // cirlular_v = s_bound_dist_1 * std::min( max_path_v, next_max_path_v );  //考虑当前和下次的运动，选取最小值（代表当前运动和下一段运动的约束下，最大可行速度）
            cirlular_v = std::max( 0.0, std::min( s_bound_dist_1 * 5, 1.0 ) ) * std::min( max_path_v, next_max_path_v );  // 考虑当前和下次的运动，选取最小值（代表当前运动和下一段运动的约束下，最大可行速度）

#pragma region                                                                                                            // 第一段直线速度轨迹规划

        double T_link = 0;
        ::rocos::DoubleS doubleS_1_P;
        ::rocos::DoubleS doubleS_1_R;
        // 存在直线或只旋转情况，才需要规划;如果本段只旋转，不移动，那么bound_dist一定=0->s_bound_dist_1一定=0，下面程序一定执行
        if ( ( 1 - s_bound_dist_1 ) > eps )
        {
            double Path_length{ dedist };
            // dedist==0,代表只旋转，不移动的情况
            if ( Path_length < eps )
            {
                KDL::Rotation R_start_end = f_start.M.Inverse( ) * F_base_circlestart.M;
                KDL::Vector ration_axis;
                double angle                       = R_start_end.GetRotAngle( ration_axis );
                constexpr double equivalent_radius = 0.1;
                double Rlength                     = equivalent_radius * abs( angle );
                Path_length                        = Rlength;
            }

            doubleS_1_P.planDoubleSProfile( 0, 0, 1, current_path_start_v / Path_length, cirlular_v / Path_length, max_path_v / Path_length, max_path_a / Path_length, max_path_a * 2 / Path_length );
            bool isplanned = doubleS_1_P.isValidMovement( );
            if ( !isplanned || !( doubleS_1_P.getDuration( ) > 0 ) )
            {
                PLOG_ERROR << "Linear trajectory planning fails,try to decrease given parameter [max_path_v] ";
                return -1;
            }

            doubleS_1_R.planDoubleSProfile( 0, 0, 1, 0, 0, max_path_v / Path_length, max_path_a / Path_length, max_path_a * 2 / Path_length );
            isplanned = doubleS_1_R.isValidMovement( );
            if ( !isplanned || !( doubleS_1_R.getDuration( ) > 0 ) )
            {
                PLOG_ERROR << "Linear trajectory planning fails,try to decrease given parameter [max_path_v] ";
                return -1;
            }

            if ( doubleS_1_R.getDuration( ) != doubleS_1_P.getDuration( ) )
            {
                doubleS_1_R.JC_scaleToDuration( doubleS_1_P.getDuration( ) );
            }

            T_link = doubleS_1_P.getDuration( );
        }
#pragma endregion

        if ( s_bound_dist_1 <= eps )  // 不存在圆弧，下段速度为0
        {
            next_path_start_v = 0;
        }
        else if ( ( 1 - s_bound_dist_1 ) <= eps )  // 不存在直线，下段速度为当前段运动速度
        {
            next_path_start_v = current_path_start_v;
        }
        else
        {
            next_path_start_v = cirlular_v;  // 下段开始速度即为计算出的圆弧速度(因为我设定圆弧匀速，后期可能会改)
        }
        next_f_start = F_base_circleend;     // 下一段运动应该开始的位姿

#pragma region                               // 圆弧段时间
        double T_cirlular = 0;
        ::rocos::DoubleS doubleS_2_R;
        if ( s_bound_dist_1 > eps )  // 存在圆弧才应该规划
        {
            T_cirlular = ( radius * alpha ) / next_path_start_v;

            doubleS_2_R.planDoubleSProfile( 0, 0, 1, 0, 0, max_path_v / ( radius * alpha ), max_path_a / ( radius * alpha ), max_path_a * 2 / ( radius * alpha ) );
            bool isplanned = doubleS_2_R.isValidMovement( );
            if ( !isplanned || !( doubleS_2_R.getDuration( ) > 0 ) )
            {
                PLOG_ERROR << "Circular trajectory planning fails,try to decrease given parameter [max_path_v] ";
                return -1;
            }

            if ( doubleS_2_R.getDuration( ) != T_cirlular )
            {
                doubleS_2_R.JC_scaleToDuration( T_cirlular );
            }
        }

#pragma endregion

        double t_total = 0;
        double s_p     = 0;
        double s_r     = 0;
        KDL::Frame link_target{ };
        KDL::Frame circular_target{ };
        bool circule_success{ true };

        std::cout << "T  = " << ( T_link + T_cirlular ) << std::endl;
        std::cout << "T_cirlular= " << ( T_cirlular ) << std::endl;
        std::cout << "T_link= " << ( T_link ) << std::endl;

        //** 轨迹计算 **//
        while ( t_total >= 0 && t_total <= ( T_link + T_cirlular ) )
        {
            if ( t_total >= 0 && t_total <= T_link && T_link != 0 )
            {
                s_p = doubleS_1_P.pos( t_total );
                s_r = doubleS_1_R.pos( t_total );

                if ( link_pos( f_start, F_base_circlestart, s_p, s_r, link_target ) < 0 )
                {
                    PLOG_ERROR << "link calculation failure";
                    return -1;
                }
                traj.push_back( link_target );
            }
            else
            {
                s_p = ( t_total - T_link ) / T_cirlular;
                s_r = doubleS_2_R.pos( t_total - T_link );

                circular_target = circle( F_base_circlestart, F_base_circleend, F_base_circleCenter, s_p, s_r, alpha, circule_success );
                if ( !circule_success )
                {
                    PLOG_ERROR << "circular calculation failure";
                    return -1;
                }
                traj.push_back( circular_target );
            }

            t_total = t_total + DELTA_T;
        }
        //**-------------------------------**//
        return 0;
    }

    int circle_center( KDL::Frame& center, const KDL::Frame& f_p1, const KDL::Frame& f_p2, const KDL::Frame& f_p3 )
    {
        const double eps = 1E-7;
        using namespace KDL;
        KDL::Vector v1 = f_p2.p - f_p1.p;
        KDL::Vector v2 = f_p3.p - f_p1.p;

        if ( v1.Normalize( ) < eps )
        {
            PLOG_ERROR << "f_p1不能等于f_p2" << std::endl;
            return -1;
        }
        if ( v2.Normalize( ) < eps )
        {
            PLOG_ERROR << "f_p1不能等于f_p3" << std::endl;
            return -1;
        }

        KDL::Vector axis_z{ v2 * v1 };

        if ( axis_z.Normalize( ) < eps )
        {
            PLOG_ERROR << "三点不能共线或过于趋向直线" << std::endl;
            return -1;
        }

        KDL::Vector axis_x{ v1 };
        KDL::Vector axis_y{ axis_z * axis_x };
        axis_y.Normalize( );

        v1 = f_p2.p - f_p1.p;
        v2 = f_p3.p - f_p1.p;

        // 在新坐标系上
        //  f_p2 = [dot(v1,axis_x),0] = [bx,0]
        //  f_P3 = [dot(v2,axis_x),dot(v2,axis_y)=[cx,cy]
        // 圆心一定位于[bx/2,0]的直线上，所以假设圆心为[bx/2,0]
        // 在利用半径相等公式求解h

        double bx = dot( v1, axis_x );
        double cx = dot( v2, axis_x );
        double cy = dot( v2, axis_y );

        double h = ( ( cx - bx / 2 ) * ( cx - bx / 2 ) + cy * cy - ( bx / 2 ) * ( bx / 2 ) ) / ( 2 * cy );
        center.p = f_p1.p + axis_x * ( bx / 2 ) + axis_y * h;
        return 0;
    }

    int circle_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& f_p1, const KDL::Frame& f_p2, const KDL::Frame& f_p3, double max_path_v, double max_path_a, bool fixed_rotation )
    {
        using namespace KDL;
        const double epsilon = 1e-6;
        KDL::Frame center;

        if ( circle_center( center, f_p1, f_p2, f_p3 ) < 0 )  // 找到圆心的位置
        {
            PLOG_ERROR << "unable to calculate center of circle";
            return -1;
        }

        KDL::Vector axis_x = f_p1.p - center.p;

        double radius = axis_x.Normalize( );

        KDL::Vector axis_tem = f_p2.p - center.p;  // 第二直线段上的半径段
        axis_tem.Normalize( );

        KDL::Vector axis_z( axis_x * axis_tem );  // 向上的Z轴
        if ( axis_z.Normalize( ) < epsilon )
        {
            std::cout << RED << "circle_trajectory():axis_x and axis_tem  cannot be parallel" << std::endl;
            return -1;
        }

        KDL::Vector axis_y{ ( axis_z * axis_x ) };
        axis_y.Normalize( );

        KDL::Frame center_( KDL::Rotation{ axis_x, axis_y, axis_z }, center.p );  // 确定圆心的方向

        //** 计算旋转角度 **//
        KDL::Vector f_p2_ = center_.Inverse( ) * f_p2.p;  // 变换f_p2.p从基座标系到圆心坐标系
        KDL::Vector f_p3_ = center_.Inverse( ) * f_p3.p;  // 变换f_p3.p从基座标系到圆心坐标系
        double theta13    = 0;

        if ( f_p3_( 1 ) < 0 )  // 变换f_p3从-180>180到 0>360_
            theta13 = atan2( f_p3_( 1 ), f_p3_( 0 ) ) + 2 * M_PI;
        else
            theta13 = atan2( f_p3_( 1 ), f_p3_( 0 ) );
        PLOG_INFO << "theta13= " << theta13 * 180 / M_PI;
        //**-------------------------------**//

        ::rocos::DoubleS doubleS;
        double path_length = ( radius * theta13 );
        doubleS.planDoubleSProfile( 0, 0, 1, 0, 0, max_path_v / path_length, max_path_a / path_length, 2 * max_path_a / path_length );
        if ( !doubleS.isValidMovement( ) || !( doubleS.getDuration( ) > 0 ) )
        {
            PLOG_ERROR << "planDoubleSProfile failed";
            return -1;
        }
        double T_total = doubleS.getDuration( );
        std::cout << BLUE << "T_total = " << T_total << GREEN << std::endl;

        KDL::Rotation R_w_p1      = f_p1.M;
        KDL::Vector Rotation_axis = R_w_p1.Inverse( ) * axis_z;

        //** 轨迹规划 **//
        double dt{ 0.0 };
        double s{ 0.0 };
        while ( dt <= T_total )
        {
            s = doubleS.pos( dt );
            if ( fixed_rotation )
                traj.push_back( KDL::Frame( R_w_p1, center_ * KDL::Vector{ radius * cos( s * theta13 ), radius * sin( s * theta13 ), 0 } ) );
            else
                traj.push_back( KDL::Frame( R_w_p1 * KDL::Rotation::Rot2( Rotation_axis, theta13 * s ), center_ * KDL::Vector{ radius * cos( s * theta13 ), radius * sin( s * theta13 ), 0 } ) );
            dt += DELTA_T;
        }
        return 0;
    }

    int circle_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& f_p1, const KDL::Frame& center, double theta13, int axiz, double max_path_v, double max_path_a, bool fixed_rotation )
    {
        using namespace KDL;
        const double epsilon = 1e-6;

        double radius = ( f_p1.p - center.p ).Normalize( );  // 半径

        if ( radius < epsilon )
        {
            PLOG_ERROR << "圆弧圆心太靠近起始位置";
            return -1;
        }
        else if ( abs( theta13 ) < epsilon )
        {
            PLOG_ERROR << "圆弧旋转角度太小";
            return -1;
        }
        else if ( abs( axiz ) > 2 )
        {
            PLOG_ERROR << "圆弧旋转轴只能选择:【0-X、1-Y、2-Z】";
            return -1;
        }

        Vector center_f_p1 = f_p1.p - center.p;  // 待旋转的向量
        Vector rot_axiz{ };

        switch ( axiz )
        {
            case 0:
                rot_axiz = center.M.UnitX( );
                break;
            case 1:
                rot_axiz = center.M.UnitY( );
                break;
            case 2:
                rot_axiz = center.M.UnitZ( );
                break;
            default:
                PLOG_ERROR << "圆弧旋转轴只能选择:【0-X、1-Y、2-Z】";
                return -1;
        }

        ::rocos::DoubleS doubleS;
        double path_length = radius * abs( theta13 );
        doubleS.planDoubleSProfile( 0, 0, 1, 0, 0, max_path_v / path_length, max_path_a / path_length, 2 * max_path_a / path_length );
        if ( !doubleS.isValidMovement( ) || !( doubleS.getDuration( ) > 0 ) )
        {
            PLOG_ERROR << "planDoubleSProfile failed";
            return -1;
        }
        double T_total = doubleS.getDuration( );

        //** 轨迹规划 **//
        double dt{ 0.0 };
        double s{ 0.0 };
        while ( dt <= T_total )
        {
            s = doubleS.pos( dt );
            if ( fixed_rotation )
                traj.push_back( KDL::Frame( f_p1.M, center.p + KDL::Rotation::Rot2( rot_axiz, theta13 * s ) * center_f_p1 ) );
            else
                traj.push_back( KDL::Frame( KDL::Rotation::Rot2( rot_axiz, theta13 * s ) * f_p1.M, center.p + KDL::Rotation::Rot2( rot_axiz, theta13 * s ) * center_f_p1 ) );
            dt += DELTA_T;
        }
        return 0;
    }

    int circle_trajectory( std::vector< KDL::Twist >& traj_vel, const KDL::Frame& f_p1, const KDL::Frame& f_p2, const KDL::Frame& f_p3, double max_path_v, double max_path_a, bool fixed_rotation )
    {
        const double epsilon = 1e-6;
        KDL::Frame center;

        if ( circle_center( center, f_p1, f_p2, f_p3 ) < 0 )  // 找到圆心的位置
        {
            PLOG_ERROR << "unable to calculate center of circle";
            return -1;
        }

        KDL::Vector axis_x = f_p1.p - center.p;

        double radius = axis_x.Normalize( );

        KDL::Vector axis_tem = f_p2.p - center.p;  // 第二直线段上的半径段
        axis_tem.Normalize( );

        KDL::Vector axis_z( axis_x * axis_tem );  // 向上的Z轴
        if ( axis_z.Normalize( ) < epsilon )
        {
            PLOG_ERROR << "第一点和第二点太接近";
            return -1;
        }

        KDL::Vector axis_y{ ( axis_z * axis_x ) };
        axis_y.Normalize( );

        KDL::Frame center_( KDL::Rotation{ axis_x, axis_y, axis_z }, center.p );  // 确定圆心的方向

        //** 计算旋转角度 **//
        KDL::Vector f_p3_ = center_.Inverse( ) * f_p3.p;
        double theta13    = 0;

        if ( f_p3_( 1 ) < 0 )
            theta13 = atan2( f_p3_( 1 ), f_p3_( 0 ) ) + 2 * M_PI;
        else
            theta13 = atan2( f_p3_( 1 ), f_p3_( 0 ) );
        std::cout << BLUE << "theta13= " << theta13 * 180 / M_PI << GREEN << std::endl;
        //**-------------------------------**//

        ::rocos::DoubleS doubleS;
        double path_length = ( radius * theta13 );
        doubleS.planDoubleSProfile( 0, 0, 1, 0, 0, max_path_v / path_length, max_path_a / path_length, 2 * max_path_a / path_length );
        if ( !doubleS.isValidMovement( ) || !( doubleS.getDuration( ) > 0 ) )
        {
            PLOG_ERROR << "planDoubleSProfile failed";
            return -1;
        }
        double T_total = doubleS.getDuration( );
        std::cout << BLUE << "T_total = " << T_total << GREEN << std::endl;

        KDL::Rotation R_w_p1 = f_p1.M;

        //** 轨迹规划 **//
        double dt{ 0.0 };
        double doubleS_vel{ 0.0 };
        double doubleS_pos{ 0.0 };
        KDL::Twist interp_vel{ };

        while ( dt <= T_total )
        {
            doubleS_pos = doubleS.pos( dt );
            doubleS_vel = doubleS.vel( dt );

            KDL::Vector W_inter_pos = center_.M * KDL::Vector{ radius * cos( doubleS_pos * theta13 ), radius * sin( doubleS_pos * theta13 ), 0 };
            // !( axis_z * W_inter_pos )向量的模长 为 radius
            interp_vel.vel = doubleS_vel * theta13 * ( axis_z * W_inter_pos );

            if ( fixed_rotation )
            {
                interp_vel.rot = KDL::Vector{ 0, 0, 0 };
            }
            else
            {
                interp_vel.rot = doubleS_vel * theta13 * axis_z;
            }

            traj_vel.push_back( interp_vel );

            dt += DELTA_T;
        }
        return 0;
    }

    int circle_trajectory( std::vector< KDL::Twist >& traj_vel, const KDL::Frame& f_p1, const KDL::Frame& center, double theta13, int axiz, double max_path_v, double max_path_a, bool fixed_rotation )
    {
        const double epsilon = 1e-6;

        if ( abs( theta13 ) < epsilon )
        {
            PLOG_ERROR << "圆弧旋转角度太小";
            return -1;
        }
        else if ( axiz > 2 || axiz < 0 )
        {
            PLOG_ERROR << "圆弧旋转轴只能选择:【0-X、1-Y、2-Z】";
            return -1;
        }

        KDL::Vector W_p1_center = f_p1.p - center.p;  // 待旋转的向量

        KDL::Vector center_p1 = center.Inverse( ) * f_p1.p;
        KDL::Vector rot_axiz{ };
        KDL::Vector radius_axiz{ };  // f_p1.p到真正圆点的向量

        switch ( axiz )
        {
            case 0:
                rot_axiz    = center.M.UnitX( );
                radius_axiz = W_p1_center - rot_axiz * center_p1( 0 );
                break;
            case 1:
                rot_axiz    = center.M.UnitY( );
                radius_axiz = W_p1_center - rot_axiz * center_p1( 1 );
                break;
            case 2:
                rot_axiz    = center.M.UnitZ( );
                radius_axiz = W_p1_center - rot_axiz * center_p1( 2 );
                break;
            default:
                PLOG_ERROR << "圆弧旋转轴只能选择:【0-X、1-Y、2-Z】";
                return -1;
        }

        ::rocos::DoubleS doubleS;
        double path_length = radius_axiz.Norm( ) * abs( theta13 );
        doubleS.planDoubleSProfile( 0, 0, 1, 0, 0, max_path_v / path_length, max_path_a / path_length, 2 * max_path_a / path_length );
        if ( !doubleS.isValidMovement( ) || !( doubleS.getDuration( ) > 0 ) )
        {
            PLOG_ERROR << "planDoubleSProfile failed";
            return -1;
        }
        double T_total = doubleS.getDuration( );

        //** 轨迹规划 **//
        double dt{ 0.0 };
        double doubleS_vel{ 0.0 };
        double doubleS_pos{ 0.0 };
        KDL::Twist interp_vel{ };

        while ( dt <= T_total )
        {
            doubleS_pos = doubleS.pos( dt );
            doubleS_vel = doubleS.vel( dt );

            KDL::Vector inter_pos = KDL::Rotation::Rot2( rot_axiz, theta13 * doubleS_pos ) * radius_axiz;
            // !( axis_z * inter_pos )向量的模长 为 radius
            interp_vel.vel = doubleS_vel * theta13 * ( rot_axiz * inter_pos );

            if ( fixed_rotation )
            {
                interp_vel.rot = KDL::Vector{ 0, 0, 0 };
            }
            else
            {
                interp_vel.rot = doubleS_vel * theta13 * rot_axiz;
            }

            traj_vel.push_back( interp_vel );

            dt += DELTA_T;
        }

        return 0;
    }

    int rotation_trajectory( std::vector< KDL::Frame >& traj, const KDL::Vector& f_p, const KDL::Rotation& f_r1, const KDL::Rotation& f_r2, double max_path_v, double max_path_a, double equivalent_radius )
    {
        using namespace KDL;

        KDL::Rotation R_r1_r2 = f_r1.Inverse( ) * f_r2;
        KDL::Vector ration_axis;
        double angle = R_r1_r2.GetRotAngle( ration_axis );
        if ( angle == 0 )
        {
            std::cout << RED << "circle_trajectory(): given rotation parameters  cannot be the same   " << GREEN << std::endl;
            return -1;
        }

        ::rocos::DoubleS doubleS;
        double path_length = ( equivalent_radius * angle );
        doubleS.planDoubleSProfile( 0, 0, 1, 0, 0, max_path_v / path_length, max_path_a / path_length, 2 * max_path_a / path_length );
        if ( !doubleS.isValidMovement( ) || !( doubleS.getDuration( ) > 0 ) )
        {
            std::cout << RED << "circle_trajectory():planDoubleSProfile failed" << GREEN << std::endl;
            return -1;
        }
        double T_total = doubleS.getDuration( );
        std::cout << BLUE << "T_total = " << T_total << GREEN << std::endl;

        //** 轨迹规划 **//
        double dt{ 0.0 };
        double s{ 0.0 };
        while ( dt <= T_total )
        {
            s = doubleS.pos( dt );
            traj.push_back( KDL::Frame( f_r1 * KDL::Rotation::Rot2( ration_axis, angle * s ), f_p ) );
            dt += DELTA_T;
        }
        return 0;
    }
#pragma endregion

#pragma region  //*关节空间点动功能实现
    SmartServo_Joint::SmartServo_Joint( std::atomic< bool >* finished_flag_ptr )
    {
        external_finished_flag_ptr = finished_flag_ptr;
        otg = new ruckig::Ruckig<ruckig::DynamicDOFs>(_joint_num, DELTA_T);
        input = new ruckig::InputParameter<ruckig::DynamicDOFs>(_joint_num);
        output = new ruckig::OutputParameter<ruckig::DynamicDOFs>(_joint_num);
    }

    //! init()只负责轨迹的信息重置，运行状态Flag由各运动线程结束后{手动重置}
    void SmartServo_Joint::init( const std::vector< std::atomic< double > >& q_init, const std::vector< std::atomic< double > >& v_init, const std::vector< std::atomic< double > >& a_init, double max_v, double max_a, double max_j )
    {
        input->control_interface = ruckig::ControlInterface::Position;
        input->synchronization   = ruckig::Synchronization::Phase;

        for ( int i = 0; i < _joint_num; i++ )
        {
            input->current_position[ i ]     = q_init[ i ];
            input->current_velocity[ i ]     = v_init[ i ];
            input->current_acceleration[ i ] = a_init[ i ];

            input->target_position[ i ]     = q_init[ i ];
            input->target_velocity[ i ]     = v_init[ i ];
            input->target_acceleration[ i ] = a_init[ i ];

            input->max_velocity[ i ]     = max_v;
            input->max_acceleration[ i ] = max_a;
            input->max_jerk[ i ]         = max_j;
        }

//        PLOG_INFO << "smart servo init succesed";
    }

    void SmartServo_Joint::RunSmartServo( rocos::Robot* robot_ptr )
    {
        //** 变量初始化 **//
        std::unique_lock< std::mutex > input_lock( input_mutex, std::defer_lock );
        ruckig::Result res;
        int count{ 0 };
        int _tick_count{ robot_ptr->tick_count };
        auto t_start = std::chrono::high_resolution_clock::now( );
        auto t_stop  = t_start;
        std::chrono::duration< double > duration{};

        //**-------------------------------**//

        // 第一次启动需要等待command()
        while ( *external_finished_flag_ptr )
        {
            std::this_thread::sleep_for( std::chrono::duration< double >{ 0.003 } );
            PLOG_INFO << "waiting for command";
        }

        while ( true )
        {
            t_start = std::chrono::high_resolution_clock::now( );

            input_lock.lock( );
            res = otg->update( *input, *output );
            input_lock.unlock( );

            if ( res == ruckig::Result::Finished )
            {
                ( *external_finished_flag_ptr ) = true;   // 这次smart servo已结束，等待下一次smart servo
//                robot_ptr->is_running_motion    = false;  // 机械臂运动已结束，可以执行其他离线类运动
                robot_ptr->setRunState(rocos::Robot::RunState::Stopped);  // 机械臂运动已结束，可以执行其他离线类运动
                on_stop_trajectory              = false;  // 这个必须设为false,因为新线程仍然使用同一个对象数据成员
//                PLOG_INFO << "smart servo has finished";
                break;
            }
            else if ( res == ruckig::Result::Working )
            {
                safety_servo( robot_ptr, output->new_position );

                input_lock.lock( );
                output->pass_to_input( *input );
                input_lock.unlock( );
            }
            else  // 计算失败，紧急停止
            {
                PLOG_ERROR << "otg 计算失败！";
                on_stop_trajectory = true;
                input_lock.lock( );
                input->control_interface = ruckig::ControlInterface::Velocity;
                input->synchronization   = ruckig::Synchronization::None;

                for ( int i = 0; i < _joint_num; i++ )
                {
                    input->target_velocity[ i ]     = 0.0;
                    input->target_acceleration[ i ] = 0.0;
                    input->max_velocity[ i ]        = robot_ptr->joints_[ i ]->getMaxVel( );
                    input->max_acceleration[ i ]    = robot_ptr->joints_[ i ]->getMaxAcc( );
                    input->max_jerk[ i ]            = robot_ptr->joints_[ i ]->getMaxJerk( );
                }
                input_lock.unlock( );
            }

            //** 50ms进行一次心跳检查,紧急停止时不需要检查 **//
            if ( ( ( ++count ) > 100 ) && !on_stop_trajectory )
            {
                count = 0;

                if ( _tick_count != robot_ptr->tick_count )
                {
                    _tick_count = robot_ptr->tick_count;
                }
                else
                {
//                    PLOG_WARNING << "点动指令时间间隔过长,停止";

                    on_stop_trajectory = true;
                    input_lock.lock( );
                    input->control_interface = ruckig::ControlInterface::Velocity;
                    input->synchronization   = ruckig::Synchronization::None;

                    for ( int i = 0; i < _joint_num; i++ )
                    {
                        input->target_velocity[ i ]     = 0.0;
                        input->target_acceleration[ i ] = 0.0;
                        input->max_velocity[ i ]        = robot_ptr->joints_[ i ]->getMaxVel( )*0.2;
                        input->max_acceleration[ i ]    = robot_ptr->joints_[ i ]->getMaxAcc( )*0.02;
                        input->max_jerk[ i ]            = robot_ptr->joints_[ i ]->getMaxJerk( )*0.01;
                    }
                    input_lock.unlock( );
                }
            }
            //**-------------------------------**//
            t_stop   = std::chrono::high_resolution_clock::now( );
            duration = ( t_stop - t_start );
//            if ( duration.count( ) > 0.0015 )
//            {
//                PLOG_WARNING << "计算时间超时：" << duration.count( ) << "s" << std::endl;
//            }
        }
    }

    void SmartServo_Joint::command( KDL::JntArray q_target )
    {
        if ( !on_stop_trajectory )  // 如果需要紧急停止，那么就不允许在更改指令了
        {
            std::unique_lock< std::mutex > input_lock( input_mutex );

            for ( int i = 0; i < _joint_num; i++ )
            {
                input->target_position[ i ]     = q_target( i );
                input->target_velocity[ i ]     = 0.0;
                input->target_acceleration[ i ] = 0.0;
            }

            ( *external_finished_flag_ptr ) = false;
        }
        else
        {
//            PLOG_DEBUG << "control is not allowed during emergency stop";
        }
    }

#pragma endregion

#pragma region         //*笛卡尔空间点动功能实现

    SmartServo_Cartesian::SmartServo_Cartesian( std::atomic< bool >* finished_flag_ptr, const KDL::Chain& robot_chain ) : _ik_vel{ robot_chain }, FK_slover{ robot_chain }
    {
        joint_current.resize( _joint_num );
        joint_target.resize( _joint_num );
        joint_vel.resize( _joint_num );
        joint_last_pos.resize( _joint_num );
        joint_last_last_pos.resize( _joint_num );
        external_finished_flag_ptr = finished_flag_ptr;
    }

    void SmartServo_Cartesian::init( rocos::Robot* robot_ptr, double target_vel, double max_vel, double max_acc, double max_jerk )
    {
        input.current_position[ 0 ]     = 0;
        input.current_velocity[ 0 ]     = 0;
        input.current_acceleration[ 0 ] = 0;

        input.target_position[ 0 ]     = target_vel;
        input.target_velocity[ 0 ]     = 0;
        input.target_acceleration[ 0 ] = 0;

        input.max_velocity[ 0 ]     = max_vel;
        input.max_acceleration[ 0 ] = max_acc;
        input.max_jerk[ 0 ]         = max_jerk;

        input.control_interface = ruckig::ControlInterface::Position;
        input.synchronization   = ruckig::Synchronization::None;

        flag_stop = false;

        KDL::SetToZero( joint_vel );

        for ( int i{ 0 }; i < _joint_num; i++ )
        {
            joint_current( i )       = robot_ptr->pos_[ i ];
            joint_target( i )        = joint_current( i );
            joint_last_pos( i )      = joint_current( i );
            joint_last_last_pos( i ) = joint_current( i );
        }

        _Cartesian_vel_index = 0;   // 0代表无方向

        _reference_frame.clear( );  // 空字符代表无参考坐标系

        current_flange = KDL::Frame{ };

//        PLOG_INFO << "笛卡尔空间点动初始化完成";
    }

    int SmartServo_Cartesian::update( KDL::JntArray& joint_vel, rocos::Robot* robot_ptr )
    {
        KDL::SetToZero( joint_vel );
        KDL::Twist Cartesian_vel{ };

        res = otg.update( input, output );

        if ( res != ruckig::Result::Working && res != ruckig::Result::Finished )
        {
            PLOG_ERROR << "OTG 计算失败";
            return -1;
        }

        const auto& res_vel = output.new_position;

        if ( abs( _Cartesian_vel_index ) <= 3 )  // 移动
        {
            Cartesian_vel.vel[ abs( _Cartesian_vel_index ) - 1 ] = sign( _Cartesian_vel_index ) * res_vel[ 0 ];
        }
        else  // 旋转
        {
            Cartesian_vel.rot[ abs( _Cartesian_vel_index ) - 4 ] = sign( _Cartesian_vel_index ) * res_vel[ 0 ];
        }

        if ( _reference_frame.compare( "flange" ) == 0 )
        {  //** 转变速度矢量的参考系，由flange系变为base系，但没有改变参考点（还是flange） **//
            FK_slover.JntToCart( vector_2_JntArray( robot_ptr->pos_ ), current_flange );
            Cartesian_vel = current_flange.M * Cartesian_vel;
        }

        output.pass_to_input( input );

        //! 雅克比默认参考系为base,参考点为flange
        if ( _ik_vel.CartToJnt( joint_current, Cartesian_vel, joint_vel ) != 0 )
        {
            PLOG_ERROR << "雅克比计算错误,错误号：" << _ik_vel.CartToJnt( joint_current, Cartesian_vel, joint_vel );
            return -1;
        }

        if ( res == ruckig::Result::Working )
            return 1;
        else
            // PLOG_INFO << "完成！";
            return 0;
    }

    void SmartServo_Cartesian::RunMotion( rocos::Robot* robot_ptr )
    {
        int t_count = 0;  // 时间计数
        int _tick_count{ robot_ptr->tick_count };
        auto t_start = std::chrono::high_resolution_clock::now( );
        auto t_stop  = t_start;
        std::chrono::duration< double > duration;

        //! 由init()保证成立，由command()来打破
        while ( *external_finished_flag_ptr )
        {
            ;  // 等待指令
        }

        while ( 1 )
        {
            t_start = std::chrono::high_resolution_clock::now( );

            //** 心跳检查 **//
            if ( !flag_stop )
                t_count++;

            if ( t_count > 100 )  // 50毫秒保持一次通信
            {
                t_count = 0;
                if ( _tick_count != robot_ptr->tick_count )
                    _tick_count = robot_ptr->tick_count;
                else
                {
//                    PLOG_WARNING << "点动指令时间间隔过长,停止";
                    Cartesian_stop( );  // 速度目标设置为0
                }
            }
            //**-------------------------------**//

            int res = update( joint_vel, robot_ptr );

            if ( res < 0 )  // OTG的 error 状态
            {
                // 关节空间急停
                flag_stop = true;
                Joint_stop( robot_ptr, joint_current, joint_last_pos, joint_last_last_pos );
                sleep( 2 );
                break;
            }
            else  // working 或者finished状态
            {
                KDL::Multiply( joint_vel, servo_dt, joint_vel );
                KDL::Add( joint_current, joint_vel, joint_target );

                //** 速度和加速度保护 **//
                if ( !flag_stop && check_vel_acc( joint_target, joint_current, joint_last_pos, 1, 5 ) < 0 )
                {
                    //! 急停状态下不用速度检查，因为会和笛卡尔急停冲突（笛卡尔急停会使得关节加速度超大，必触发关节急停保护）
                    flag_stop = true;
                    Joint_stop( robot_ptr, joint_current, joint_last_pos, joint_last_last_pos );  // 关节空间急停
                    sleep( 2 );
                    break;
                }

                joint_last_last_pos = joint_last_pos;
                joint_last_pos      = joint_current;
                joint_current       = joint_target;
                //**-------------------------------**//

                //** 安全位置伺服,防止关节超限 **//
                safety_servo( robot_ptr, joint_target );
                //**-------------------------------**//

                if ( res == 0 && flag_stop )
                {
//                    PLOG_INFO << "笛卡尔空间急停已完成";
                    break;
                }
            }
            t_stop   = std::chrono::high_resolution_clock::now( );
            duration = ( t_stop - t_start );
//            if ( duration.count( ) > 0.0015 )
//            {
//                PLOG_WARNING << "计算时间超时：" << duration.count( ) << "s";
//            }
        }

        ( *external_finished_flag_ptr ) = true;   // 这次smart servo已结束，等待下一次smart servo
//        robot_ptr->is_running_motion    = false;  // 机械臂运动已结束，可以执行其他离线类运动
        robot_ptr->setRunState(rocos::Robot::RunState::Stopped);  // 机械臂运动已结束，可以执行其他离线类运动
    }

    void SmartServo_Cartesian::command( int Cartesian_vel_index, const char* reference_frame )
    {
        if ( _Cartesian_vel_index == 0 )
            _Cartesian_vel_index = Cartesian_vel_index;

        if ( _reference_frame.empty( ) )
            _reference_frame = reference_frame;

        if ( !flag_stop )
        {
            if ( _Cartesian_vel_index != Cartesian_vel_index )
            {
                PLOG_ERROR << "方向变换，停止！";
                Cartesian_stop( );
            }
            else if ( _reference_frame.compare( reference_frame ) != 0 )
            {
                PLOG_ERROR << "参考坐标系变换，停止！";
                Cartesian_stop( );
            }
            else
                *external_finished_flag_ptr = false;
        }
        else{
//            PLOG_WARNING << "紧急停止中,不允许修改目标";
        }
    }

    void SmartServo_Cartesian::Cartesian_stop( double max_vel, double max_acc, double max_jerk )
    {
        flag_stop                      = true;
        input.target_position[ 0 ]     = 0;
        input.target_velocity[ 0 ]     = 0;
        input.target_acceleration[ 0 ] = 0;

        input.max_velocity[ 0 ]     = max_vel;
        input.max_acceleration[ 0 ] = max_acc;
        input.max_jerk[ 0 ]         = max_jerk;
    }

#pragma endregion

#pragma region  //*零空间点动实现

    SmartServo_Nullspace::SmartServo_Nullspace(std::atomic< bool >* finished_flag_ptr, const KDL::Chain& robot_chain ) : jnt2jac(robot_chain ), jac(_joint_num ),
                                                                                                                         U( Eigen::MatrixXd::Zero( 6, _joint_num ) ),
                                                                                                                         S( Eigen::VectorXd::Zero( _joint_num ) ),
                                                                                                                         Sinv( Eigen::VectorXd::Zero( _joint_num ) ),
                                                                                                                         V( Eigen::MatrixXd::Zero( _joint_num, _joint_num ) ),
                                                                                                                         tmp( Eigen::VectorXd::Zero( _joint_num ) ),
                                                                                                                         fk_slover( robot_chain )

    {
        joint_current.resize( _joint_num );
        joint_target.resize( _joint_num );
        joint_vel.resize( _joint_num );
        joint_last_pos.resize( _joint_num );
        joint_last_last_pos.resize( _joint_num );
        external_finished_flag_ptr = finished_flag_ptr;
    }

    void SmartServo_Nullspace::init(rocos::Robot* robot_ptr, double target_vel, double max_vel, double max_acc, double max_jerk )
    {
        input.current_position[ 0 ]     = 0;
        input.current_velocity[ 0 ]     = 0;
        input.current_acceleration[ 0 ] = 0;

        input.target_position[ 0 ]     = target_vel;
        input.target_velocity[ 0 ]     = 0;
        input.target_acceleration[ 0 ] = 0;

        input.max_velocity[ 0 ]     = max_vel;
        input.max_acceleration[ 0 ] = max_acc;
        input.max_jerk[ 0 ]         = max_jerk;

        input.control_interface = ruckig::ControlInterface::Position;
        input.synchronization   = ruckig::Synchronization::None;

        flag_stop = false;

        KDL::SetToZero( joint_vel );

        for ( int i{ 0 }; i < _joint_num; i++ )
        {
            joint_current( i )       = robot_ptr->pos_[ i ];
            joint_target( i )        = joint_current( i );
            joint_last_pos( i )      = joint_current( i );
            joint_last_last_pos( i ) = joint_current( i );
        }

        _jogging_Direction = 0;   // 0代表无方向
        _max_jac_cul_index = -1;  //-1代表无指定控制哪个关节
        _command_Direction = 0;   // 0表示未知命令方向与臂角方向是否一致
        PLOG_INFO << "零空间点动初始化完成";
    }

    int SmartServo_Nullspace::update(KDL::JntArray& joint_vel )
    {
        KDL::SetToZero( joint_vel );

        int error_code;
        error_code = jnt2jac.JntToJac( joint_current, jac );
        if ( error_code != 0 )
        {
            PLOG_ERROR << "雅克比计算错误:" << error_code;
            return -1;
        }

        error_code = KDL::svd_eigen_HH( jac.data, U, S, V, tmp, 150 );
        if ( error_code != 0 )
        {
            PLOG_ERROR << "雅克比SVD计算错误:" << error_code;
            return -1;
        }

        for ( int i = 0; i < _joint_num; ++i )
        {
            Sinv( i ) = fabs( S( i ) ) < 0.00001 ? 0.0 : 1.0 / S( i );
        }

        null_space_jac = Eigen::MatrixXd::Identity( 7, 7 ) - V * Sinv.asDiagonal( ) * U.transpose( ) * U * S.asDiagonal( ) * V.transpose( );

        //** 找到哪个关节运动范围最大 **//
        if ( _max_jac_cul_index == -1 )
        {
            null_space_jac.diagonal( ).maxCoeff( &_max_jac_cul_index );
            is_same_on_direction( null_space_jac.col( _max_jac_cul_index ) );
        }
        //**-------------------------------**//

        //** 判断当前位置下，能否进行零空间点动 **//
        int joint_index = 0;
        for ( ; joint_index < _joint_num; joint_index++ )
        {
            if ( !null_space_jac.col( joint_index ).isZero( 1e-13 ) )
                break;
        }
        if ( joint_index == _joint_num )
        {
            PLOG_ERROR << "当前构型下，无法进行臂角点动";
            return -1;
        }
        //**-------------------------------**//

        otg_res = otg.update( input, output );

        if ( otg_res != ruckig::Result::Working && otg_res != ruckig::Result::Finished )
        {
            PLOG_ERROR << "OTG 计算失败";
            return -1;
        }

        const auto& otg_pos = output.new_position;

        tmp.setZero( );
        tmp( _max_jac_cul_index ) = _command_Direction * otg_pos[ 0 ];

        joint_vel.data = null_space_jac * tmp;

        output.pass_to_input( input );

        if ( otg_res == ruckig::Result::Working )
            return 1;
        else
            // PLOG_INFO << "otg完成！";
            return 0;
    }

    void SmartServo_Nullspace::RunMotion(rocos::Robot* robot_ptr )
    {
        int t_count = 0;  // 时间计数
        int _tick_count{ robot_ptr->tick_count };
        auto t_start = std::chrono::high_resolution_clock::now( );
        auto t_stop  = t_start;
        std::chrono::duration< double > duration;

        //! 由init()保证成立，由command()来打破
        while ( *external_finished_flag_ptr )
        {
            ;  // 等待指令
        }

        while ( 1 )
        {
            t_start = std::chrono::high_resolution_clock::now( );

            //** 心跳检查 **//
            if ( !flag_stop )
                t_count++;

            if ( t_count > 100 )  // 100毫秒保持一次通信
            {
                t_count = 0;
                if ( _tick_count != robot_ptr->tick_count )
                    _tick_count = robot_ptr->tick_count;
                else
                {
//                    PLOG_WARNING << "点动指令时间间隔过长,停止";
                    nullspace_stop( );  // 速度目标设置为0
                }
            }
            //**-------------------------------**//

            int update_res = update( joint_vel );

            if ( update_res < 0 )  // OTG的 error 状态
            {
                // 关节空间急停
                flag_stop = true;
                Joint_stop( robot_ptr, joint_current, joint_last_pos, joint_last_last_pos );
                sleep( 2 );
                break;
            }
            else  // working 或者finished状态
            {
                KDL::Multiply( joint_vel, servo_dt, joint_vel );
                KDL::Add( joint_current, joint_vel, joint_target );

                //** 速度和加速度保护 **//
                if ( !flag_stop && check_vel_acc( joint_target, joint_current, joint_last_pos, 1, 5 ) < 0 )
                {
                    //! 急停状态下不用速度检查，因为会和笛卡尔急停冲突（笛卡尔急停会使得关节加速度超大，必触发关节急停保护）
                    flag_stop = true;
                    Joint_stop( robot_ptr, joint_current, joint_last_pos, joint_last_last_pos );  // 关节空间急停
                    sleep( 2 );
                    break;
                }

                joint_last_last_pos = joint_last_pos;
                joint_last_pos      = joint_current;
                joint_current       = joint_target;
                //**-------------------------------**//

                //** 安全位置伺服,防止关节超限 **//
                safety_servo( robot_ptr, joint_target );
                //**-------------------------------**//

                if ( update_res == 0 && flag_stop )
                {
//                    PLOG_INFO << "零空间急停已完成";
                    break;
                }
            }
            t_stop   = std::chrono::high_resolution_clock::now( );
            duration = ( t_stop - t_start );
//            if ( duration.count( ) > 0.0015 )
//            {
//                PLOG_WARNING << "计算时间超时：" << duration.count( ) << "s";
//            }
        }

        ( *external_finished_flag_ptr ) = true;   // 这次smart servo已结束，等待下一次smart servo
//        robot_ptr->is_running_motion    = false;  // 机械臂运动已结束，可以执行其他离线类运动
        robot_ptr->setRunState(rocos::Robot::RunState::Stopped);  // 机械臂运动已结束，可以执行其他离线类运动
    }

    void SmartServo_Nullspace::command(int jogging_Direction )
    {
        if ( _jogging_Direction == 0 )
            _jogging_Direction = jogging_Direction;

        if ( !flag_stop )
        {
            if ( _jogging_Direction != jogging_Direction )
            {
                PLOG_ERROR << "方向变换，停止！";
                nullspace_stop( );
            }
            else
                *external_finished_flag_ptr = false;
        }
        else
            PLOG_WARNING << "紧急停止中,不允许修改目标";
    }

    void SmartServo_Nullspace::nullspace_stop(double max_vel, double max_acc, double max_jerk )
    {
        flag_stop                      = true;
        input.target_position[ 0 ]     = 0;
        input.target_velocity[ 0 ]     = 0;
        input.target_acceleration[ 0 ] = 0;

        input.max_velocity[ 0 ]     = max_vel;
        input.max_acceleration[ 0 ] = max_acc;
        input.max_jerk[ 0 ]         = max_jerk;
    }

    bool SmartServo_Nullspace::is_same_on_direction(const Eigen::Block< Eigen::Matrix< double, 7, 7 >, 7, 1, true >& joint_vel )
    {
        // 如果不是7自由度直接退出
        if(_joint_num != 7)
            return false;

        KDL::JntArray joint_target( _joint_num );
        for ( int i = 0; i < _joint_num; i++ )
        {
            joint_target( i ) = joint_vel( i ) * servo_dt + joint_current( i );
        }

        KDL::Frame frame_joint_4;
        KDL::Frame frame_joint_4_target;
        fk_slover.JntToCart( joint_current, frame_joint_4, 4 );
        fk_slover.JntToCart( joint_target, frame_joint_4_target, 4 );

        KDL::Rotation R_start_end = frame_joint_4.M.Inverse( ) * frame_joint_4_target.M;
        KDL::Vector axis;
        double angle = R_start_end.GetRotAngle( axis );
        axis         = frame_joint_4.M * axis;

        KDL::Frame shoulder;
        KDL::Frame wrist;
        fk_slover.JntToCart( joint_current, shoulder, 2 );
        fk_slover.JntToCart( joint_current, wrist, 6 );
        KDL::Vector SW = wrist.p - shoulder.p;
        ;

        for ( int i = 0; i < 3; i++ )
            if ( std::abs( SW( i ) ) > 0.001 && KDL::sign( SW( i ) ) != KDL::sign( axis( i ) ) )
            {
                // GetRotAngle()计算的轴不是由1关节指向6关节
                angle = -1 * angle;
                break;
            }

        // PLOG_DEBUG << "SW = " << SW;
        // PLOG_DEBUG << "axis = " << axis;
        // PLOG_DEBUG << "angle = " << angle;
        // PLOG_DEBUG << "_jogging_Direction" << _jogging_Direction;

        if ( KDL::sign( angle ) == _jogging_Direction )
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

#pragma region  //*逆解

    namespace slover_thet2
    {
        JC_double m( JC_double l_se, JC_double l_ew, JC_double thet_4 )
        {
            return ( l_se ) + ( l_ew )*cos( thet_4 );
        }

        JC_double n( JC_double l_ew, JC_double thet_4 )
        {
            return ( l_ew )*sin( thet_4 );
        }

        JC_double f_thet2( JC_double l_se, JC_double l_ew, JC_double thet_4, JC_double x_sw_z )
        {
            return atan2( m( l_se, l_ew, thet_4 ), n( l_ew, thet_4 ) ) - atan2( x_sw_z, sqrt( pow( m( l_se, l_ew, thet_4 ), 2 ) + pow( n( l_ew, thet_4 ), 2 ) - pow( x_sw_z, 2 ) ) );
        }

        JC_double f_thet2_2( JC_double l_se, JC_double l_ew, JC_double thet_4, JC_double x_sw_z )
        {
            return atan2( m( l_se, l_ew, thet_4 ), n( l_ew, thet_4 ) ) - atan2( x_sw_z, -sqrt( pow( m( l_se, l_ew, thet_4 ), 2 ) + pow( n( l_ew, thet_4 ), 2 ) - pow( x_sw_z, 2 ) ) );
        }
    }  // namespace slover_thet2

    namespace slover_thet1
    {

        JC_double o( JC_double thet_2, JC_double l_se, JC_double l_ew, JC_double thet_4 )
        {
            return sin( thet_2 ) * ( ( l_se ) + ( l_ew )*cos( thet_4 ) );
        }

        JC_double p( JC_double l_ew, JC_double thet_4, JC_double thet_2 )
        {
            return ( l_ew )*sin( thet_4 ) * cos( thet_2 );
        }

        JC_double sin_thet1( JC_double thet_2, JC_double l_se, JC_double l_ew, JC_double thet_4, JC_double x_sw_y )
        {
            return x_sw_y * ( o( thet_2, l_se, l_ew, thet_4 ) + p( l_ew, thet_4, thet_2 ) );
        }
        JC_double cos_thet1( JC_double thet_2, JC_double l_se, JC_double l_ew, JC_double thet_4, JC_double x_sw_x )
        {
            return x_sw_x * ( o( thet_2, l_se, l_ew, thet_4 ) + p( l_ew, thet_4, thet_2 ) );
        }

        JC_double f_thet1( JC_double thet_2, JC_double l_se, JC_double l_ew, JC_double thet_4, JC_double x_sw_x, JC_double x_sw_y )
        {
            return atan2( sin_thet1( thet_2, l_se, l_ew, thet_4, x_sw_y ), cos_thet1( thet_2, l_se, l_ew, thet_4, x_sw_x ) );
        }

    }  // namespace slover_thet1

    namespace slover_utility
    {
        // %* 旋转矩阵求解,参考冗余书籍p51
        Eigen::Matrix3d rot_matrix( JC_double thet, JC_double alpha )
        {
            Eigen::Matrix3d tem;
            tem << cos( thet ), -sin( thet ) * cos( alpha ), sin( thet ) * sin( alpha ),
                sin( thet ), cos( thet ) * cos( alpha ), -cos( thet ) * sin( alpha ),
                0, sin( alpha ), cos( alpha );

            return tem;
        }
        // % *-------------------------------

        // % *向量转斜对称矩阵
        Eigen::Matrix3d ssm( JC_double x, JC_double y, JC_double z )
        {
            Eigen::Matrix3d tem;

            tem << 0, -z, y,
                z, 0, -x,
                -y, x, 0;

            return tem;
        }
        // % *------------------------------------------

        // % *+-360转+-180
        JC_double my_convert( JC_double x )
        {
            if ( x > KDL::PI )
                return x - 2 * KDL::PI;
            else if ( x < -KDL::PI )
                return 2 * KDL::PI + x;
            else if ( x == 2 * KDL::PI )
                return 0;
            else if ( x == -2 * KDL::PI )
                return 0;
            else
                return x;
        }
        // % *------------------------------------------

        // % *+-360转+-180
        std::array< JC_double, 7 >& my_convert( std::array< JC_double, 7 >& arg )
        {
            for ( int i = 0; i < arg.size( ); i++ )
            {
                if ( arg[ i ] > KDL::PI )
                    arg[ i ] = arg[ i ] - 2 * KDL::PI;
                else if ( arg[ i ] < -KDL::PI )
                    arg[ i ] = 2 * KDL::PI + arg[ i ];
            }

            return arg;
        }
        // % *------------------------------------------

        // % *+KDL::rotaion转Eigen::Matrix3d
        Eigen::Matrix3d rot_to_matrix( const KDL::Rotation& arg )
        {
            Eigen::Matrix3d tem;

            for ( int row = 0; row < 3; row++ )
                for ( int cul = 0; cul < 3; cul++ )
                    tem( row, cul ) = arg( row, cul );

            return tem;
        }
        // % *------------------------------------------

        // % *裁剪浮点数至指定位数
        JC_double roundn( const JC_double& arg, unsigned int precision )
        {
            // thet2 thet4 thet6 在等于0度时，计算结果为0.9999999999999998,因此需要截取
            std::stringstream ss;
            ss << std::setprecision( precision ) << arg;

            return std::stod( ss.str( ) );
            // return arg;
        }
        // % *------------------------------------------

    }  // namespace slover_utility

    int inverse_special_to_SRS::init( const KDL::Chain& dof_7_robot, const KDL::JntArray& pos_minimum, const KDL::JntArray& pos_maximum )
    {
        double RPY[ 3 ];
        dof_7_robot.getSegment( 1 ).getFrameToTip( ).M.GetRPY( RPY[ 0 ], RPY[ 1 ], RPY[ 2 ] );
        joint_1_inverse = -1 * sin( RPY[ 0 ] );
        dof_7_robot.getSegment( 3 ).getFrameToTip( ).M.GetRPY( RPY[ 0 ], RPY[ 1 ], RPY[ 2 ] );
        joint_3_inverse = -1 * sin( RPY[ 0 ] );
        dof_7_robot.getSegment( 5 ).getFrameToTip( ).M.GetRPY( RPY[ 0 ], RPY[ 1 ], RPY[ 2 ] );
        joint_5_inverse = -1 * sin( RPY[ 0 ] );

        d_bs   = std::abs( dof_7_robot.getSegment( 0 ).getFrameToTip( ).p[ 2 ] );  //%base系到shoulder的长度
        l_0_bs = { 0, 0, d_bs };
        d_se   = std::abs( dof_7_robot.getSegment( 2 ).getFrameToTip( ).p[ 1 ] );
        d_ew   = std::abs( dof_7_robot.getSegment( 4 ).getFrameToTip( ).p[ 1 ] );
        d_wt   = std::abs( dof_7_robot.getSegment( 6 ).getFrameToTip( ).p[ 1 ] );
        l_7_wt = { 0, 0, d_wt };

        for ( int i = 0; i < _joint_num; i++ )
            if ( pos_maximum( i ) < pos_minimum( i ) )
            {
                PLOG_ERROR << "关节[" << i << "] 的minimum大于maximum,请检查限位范围";
                return -1;
            }

        _pos_minimum = pos_minimum;
        _pos_maximum = pos_maximum;
        return 0;
    }

    int inverse_special_to_SRS::JC_cartesian_to_joint( KDL::Frame inter_T, JC_double inter_joint_3, const KDL::JntArray& last_joint, KDL::JntArray& joint_out )
    {
        // * 仿真环境设置
        //** 将DH坐标系转换为预定的标准DH **//
        KDL::JntArray _last_joint{ last_joint };
        _last_joint( 1 ) = joint_1_inverse * last_joint( 1 );
        _last_joint( 3 ) = joint_3_inverse * last_joint( 3 );
        _last_joint( 5 ) = joint_5_inverse * last_joint( 5 );
        //**-------------------------------**//

        std::vector< std::array< JC_double, 7 > > res_thet;  // 32组结果
        std::vector< double > joint_offset;                  // 32组位置差
        //**-------------------------------**//

        try
        {
            KDL::Vector x_0_7   = inter_T.p;
            KDL::Rotation r_0_7 = inter_T.M;

            KDL::Vector x_0_sw = x_0_7 - l_0_bs - r_0_7 * l_7_wt;

            //! 腕部中心点在1关节轴线上奇异处理：
            if ( x_0_sw( 0 ) == 0 && x_0_sw( 1 ) == 0 )
                throw JC_exception{ "腕部中心点在1关节轴线上奇异", -3 };

            JC_double thet_4_tem = acos( slover_utility::roundn( ( pow( x_0_sw.Norm( ), 2 ) - pow( d_se, 2 ) - pow( d_ew, 2 ) ) / ( 2 * d_se * d_ew ), 14 ) );

            //! 4关节奇异处理：thet4接近0时->acos(>1)->thet_4_tem=nan
            if ( std::isnan( thet_4_tem ) )
                throw JC_exception{ "关节4奇异", -1 };

            for ( int reference_index = 0; reference_index < ( 2 + 2 * !( thet_4_tem == 0 ) ); reference_index++ )
            {
                JC_double thet_4   = 0;
                JC_double thet_r_2 = 0;

                switch ( reference_index )
                {
                    case 0:
                        thet_4   = thet_4_tem;
                        thet_r_2 = slover_thet2::f_thet2( d_se, d_ew, thet_4, x_0_sw( 2 ) );
                        break;

                    case 1:
                        thet_4   = thet_4_tem;
                        thet_r_2 = slover_thet2::f_thet2_2( d_se, d_ew, thet_4, x_0_sw( 2 ) );
                        break;

                    case 2:
                        thet_4   = -thet_4_tem;
                        thet_r_2 = slover_thet2::f_thet2( d_se, d_ew, thet_4, x_0_sw( 2 ) );
                        break;

                    case 3:

                        thet_4   = -thet_4_tem;
                        thet_r_2 = slover_thet2::f_thet2_2( d_se, d_ew, thet_4, x_0_sw( 2 ) );
                        break;

                    default:
                        throw JC_exception{ "switch 异常", -2 };
                        break;
                }

                JC_double thet_r_1 = slover_thet1::f_thet1( thet_r_2, d_se, d_ew, thet_4, x_0_sw( 0 ), x_0_sw( 1 ) );

                Eigen::Matrix3d ref_r_0_3 = slover_utility::rot_matrix( thet_r_1, -KDL::PI_2 ) * slover_utility::rot_matrix( thet_r_2, KDL::PI_2 ) * slover_utility::rot_matrix( 0, -KDL::PI_2 );

                KDL::Vector u_0_sw;
                JC_double vct_len = x_0_sw.Norm( );
                if ( vct_len < 1e-6 )
                    throw JC_exception{ "腕部点与肘部点太接近", -8 };
                else
                    u_0_sw = x_0_sw / vct_len;

                Eigen::Matrix3d ssm_u_0_sw = slover_utility::ssm( u_0_sw( 0 ), u_0_sw( 1 ), u_0_sw( 2 ) );

                Eigen::Matrix3d As = ssm_u_0_sw * ref_r_0_3;
                Eigen::Matrix3d Bs = -1 * ssm_u_0_sw * As;
                Eigen::Matrix3d Cs = ( Eigen::Vector3d{ u_0_sw( 0 ), u_0_sw( 1 ), u_0_sw( 2 ) } * Eigen::RowVector3d{ u_0_sw( 0 ), u_0_sw( 1 ), u_0_sw( 2 ) } ) * ref_r_0_3;

                Eigen::Matrix3d r_3_4_transpose = slover_utility::rot_matrix( thet_4, KDL::PI_2 ).transpose( );
                Eigen::Matrix3d matrix_r_0_7    = slover_utility::rot_to_matrix( r_0_7 );

                Eigen::Matrix3d Aw = r_3_4_transpose * As.transpose( ) * matrix_r_0_7;
                Eigen::Matrix3d Bw = r_3_4_transpose * Bs.transpose( ) * matrix_r_0_7;
                Eigen::Matrix3d Cw = r_3_4_transpose * Cs.transpose( ) * matrix_r_0_7;

                std::array< JC_double, 2 > arm_angle{ 0, 0 };
                {
                    JC_double a_n = As( 2, 2 );
                    JC_double b_n = Bs( 2, 2 );
                    JC_double c_n = Cs( 2, 2 );
                    JC_double a_d = -As( 2, 0 );
                    JC_double b_d = -Bs( 2, 0 );
                    JC_double c_d = -Cs( 2, 0 );

                    JC_double b    = tan( inter_joint_3 ) * a_d - a_n;
                    JC_double a    = tan( inter_joint_3 ) * b_d - b_n;
                    JC_double c    = c_n - tan( inter_joint_3 ) * c_d;
                    arm_angle[ 0 ] = 2 * atan2( ( b + sqrt( pow( b, 2 ) + pow( a, 2 ) - pow( c, 2 ) ) ), ( a + c ) );
                    arm_angle[ 1 ] = 2 * atan2( ( b - sqrt( pow( b, 2 ) + pow( a, 2 ) - pow( c, 2 ) ) ), ( a + c ) );
                }

                JC_double invert_123 = 0;
                JC_double invert_567 = 0;
                JC_double thet1      = 0;
                JC_double thet1_tem  = 0;
                JC_double thet2      = 0;
                JC_double thet2_tem  = 0;
                JC_double thet3      = 0;
                JC_double thet3_tem  = 0;
                JC_double thet4      = 0;
                JC_double thet4_tem  = 0;
                JC_double thet5      = 0;
                JC_double thet5_tem  = 0;
                JC_double thet6      = 0;
                JC_double thet6_tem  = 0;
                JC_double thet7      = 0;
                JC_double thet7_tem  = 0;

                for ( const auto& interp_arm_angle : arm_angle )
                {
                    for ( int inverst_index = 0; inverst_index < 4; inverst_index++ )
                    {
                        switch ( inverst_index )
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
                                throw JC_exception{ "switch 异常", -2 };
                                break;
                        }

                        if ( invert_123 )
                            thet2 = -1 * thet2_tem;
                        else
                        {
                            JC_double sin_tem = slover_utility::roundn( -As( 2, 1 ) * sin( interp_arm_angle ) - Bs( 2, 1 ) * cos( interp_arm_angle ) - Cs( 2, 1 ), 14 );
                            thet2             = acos( sin_tem );
                            thet2_tem         = thet2;
                        }

                        //! 2关节奇异处理
                        if ( thet2 == 0 )
                            throw JC_exception{ "2关节奇异", -4 };

                        if ( invert_123 )
                            thet1 = thet1_tem + KDL::PI;
                        else
                        {
                            JC_double sin_tem = -As( 1, 1 ) * sin( interp_arm_angle ) - Bs( 1, 1 ) * cos( interp_arm_angle ) - Cs( 1, 1 );
                            JC_double cos_tem = -As( 0, 1 ) * sin( interp_arm_angle ) - Bs( 0, 1 ) * cos( interp_arm_angle ) - Cs( 0, 1 );
                            thet1             = atan2( sin_tem * sin( thet2 ), cos_tem * sin( thet2 ) );
                            thet1_tem         = thet1;
                        }

                        if ( invert_123 )
                            thet3 = thet3_tem + KDL::PI;
                        else
                        {
                            JC_double sin_tem = As( 2, 2 ) * sin( interp_arm_angle ) + Bs( 2, 2 ) * cos( interp_arm_angle ) + Cs( 2, 2 );
                            JC_double cos_tem = -As( 2, 0 ) * sin( interp_arm_angle ) - Bs( 2, 0 ) * cos( interp_arm_angle ) - Cs( 2, 0 );
                            thet3             = atan2( sin_tem * sin( thet2 ), cos_tem * sin( thet2 ) );
                            thet3_tem         = thet3;
                        }

                        //! 共32组结果，加上这个限定就16组，和臂角法刚好相等
                        if ( std::abs( slover_utility::my_convert( thet3 ) - inter_joint_3 ) > 1e-5 )
                            continue;

                        if ( invert_567 )
                            thet6 = -1 * thet6_tem;
                        else
                        {
                            JC_double sin_tem = slover_utility::roundn( Aw( 2, 2 ) * sin( interp_arm_angle ) + Bw( 2, 2 ) * cos( interp_arm_angle ) + Cw( 2, 2 ), 14 );
                            thet6             = acos( sin_tem );
                            thet6_tem         = thet6;
                        }

                        //! 6关节奇异处理
                        if ( thet6 == 0 )
                            throw JC_exception{ "6关节奇异", -5 };

                        if ( invert_567 )
                            thet5 = thet5_tem + KDL::PI;
                        else
                        {
                            JC_double sin_tem = Aw( 1, 2 ) * sin( interp_arm_angle ) + Bw( 1, 2 ) * cos( interp_arm_angle ) + Cw( 1, 2 );
                            JC_double cos_tem = Aw( 0, 2 ) * sin( interp_arm_angle ) + Bw( 0, 2 ) * cos( interp_arm_angle ) + Cw( 0, 2 );
                            thet5             = atan2( sin_tem * sin( thet6 ), cos_tem * sin( thet6 ) );
                            thet5_tem         = thet5;
                        }

                        if ( invert_567 )
                            thet7 = thet7_tem + KDL::PI;
                        else
                        {
                            JC_double sin_tem = Aw( 2, 1 ) * sin( interp_arm_angle ) + Bw( 2, 1 ) * cos( interp_arm_angle ) + Cw( 2, 1 );
                            JC_double cos_tem = -Aw( 2, 0 ) * sin( interp_arm_angle ) - Bw( 2, 0 ) * cos( interp_arm_angle ) - Cw( 2, 0 );
                            thet7             = atan2( sin_tem * sin( thet6 ), cos_tem * sin( thet6 ) );
                            thet7_tem         = thet7;
                        }

                        std::array< JC_double, 7 > temp1{ thet1, thet2, thet3, thet_4, thet5, thet6, thet7 };

                        //! 最终检查,16组全检查
                        for ( int i = 0; i < 7; i++ )
                        {
                            if ( std::isnan( temp1[ i ] ) )
                            {
                                std::stringstream string;
                                string << "关节[" << i << "]=Nan,结果无效";
                                throw JC_exception{ string.str( ).c_str( ), -6 };
                            }
                            else if ( isinf( temp1[ i ] ) )
                            {
                                std::stringstream string;
                                string << "关节[" << i << "]=Inf,结果无效";
                                throw JC_exception{ string.str( ).c_str( ), -7 };
                            }
                        }

                        bool is_out_of_range{ false };
                        slover_utility::my_convert( temp1 );
                        for ( int i = 0; i < 7; i++ )
                            if ( temp1[ i ] > _pos_maximum( i ) || temp1[ i ] < _pos_minimum( i ) )
                            {
                                // PLOG_WARNING << "关节[" << i << "]超出关节限定范围,结果无效";
                                is_out_of_range = true;
                                break;
                            }

                        if ( !is_out_of_range )
                            res_thet.emplace_back( temp1 );
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

            //** 找到所有结果中最小位移的那一组 **//
            constexpr std::array< JC_double, 7 > K_p{ 0.7, 0.7, 0.7, 0.7, 0.3, 0.3, 0.3 };

            for ( const auto& iot : res_thet )
            {
                JC_double sum = 0;
                for ( int joint_index = 0; joint_index < 7; joint_index++ )
                {
                    sum += K_p[ joint_index ] * std::abs( iot[ joint_index ] - _last_joint( joint_index ) );
                }
                joint_offset.emplace_back( sum );
            }
            //**-------------------------------**//

            //** 路径数组存储最小位移的那一组 **//
            std::pair< std::vector< JC_double >::iterator, std::vector< JC_double >::iterator > min_offset_index = std::minmax_element( joint_offset.begin( ), joint_offset.end( ) );
            int index_offset                                                                                     = min_offset_index.first - joint_offset.begin( );

            for ( int i = 0; i < 7; i++ )
                joint_out( i ) = res_thet[ index_offset ][ i ];
            //**-------------------------------**//

            //** 将DH坐标系还原 **//
            joint_out( 1 ) = joint_1_inverse * joint_out( 1 );
            joint_out( 3 ) = joint_3_inverse * joint_out( 3 );
            joint_out( 5 ) = joint_5_inverse * joint_out( 5 );
            //**-------------------------------**//

            return 0;
        }
        catch ( const JC_exception& error )
        {
            PLOG_ERROR << error.error_str << ", 错误号：" << error.error_code;
            return error.error_code;
        }
        catch ( ... )
        {
            PLOG_ERROR << "未知错误";
            return -10;
        }
    }

#pragma endregion

    void Joint_stop( rocos::Robot* robot_ptr, const KDL::JntArray& current_pos, const KDL::JntArray& last_pos, const KDL::JntArray& last_last_pos )
    {
        //** 变量初始化 **//
        ruckig::Ruckig<ruckig::DynamicDOFs> otg{_joint_num, DELTA_T};
        ruckig::InputParameter< ruckig::DynamicDOFs > input(_joint_num);
        ruckig::OutputParameter< ruckig::DynamicDOFs > output(_joint_num);
        ruckig::Result res;
        //**-------------------------------**//

        try
        {
            KDL::JntArray current_vel( _joint_num );
            KDL::JntArray last_vel( _joint_num );
            KDL::JntArray current_acc( _joint_num );

            KDL::Subtract( current_pos, last_pos, current_vel );
            KDL::Divide( current_vel, DELTA_T, current_vel );

            KDL::Subtract( last_pos, last_last_pos, last_vel );
            KDL::Divide( last_vel, DELTA_T, last_vel );

            KDL::Subtract( current_vel, last_vel, current_acc );
            KDL::Divide( current_acc, DELTA_T, current_acc );

            input.control_interface = ruckig::ControlInterface::Velocity;
            input.synchronization   = ruckig::Synchronization::None;

            for ( int i = 0; i < _joint_num; i++ )
            {
                input.current_position[ i ]     = robot_ptr->pos_[ i ];
                input.current_velocity[ i ]     = KDL::sign( current_vel( i ) ) * std::min( abs( current_vel( i ) ), robot_ptr->max_vel_[ i ] );
                input.current_acceleration[ i ] = KDL::sign( current_acc( i ) ) * std::min( abs( current_acc( i ) ), robot_ptr->max_acc_[ i ] );

                input.target_position[ i ]     = input.current_position[ i ];
                input.target_velocity[ i ]     = 0;
                input.target_acceleration[ i ] = 0;

                input.max_velocity[ i ]     = robot_ptr->joints_[ i ]->getMaxVel( );
                input.max_acceleration[ i ] = robot_ptr->joints_[ i ]->getMaxAcc( );
                input.max_jerk[ i ]         = robot_ptr->joints_[ i ]->getMaxJerk( );
            }

            while ( ( res = otg.update( input, output ) ) == ruckig::Result::Working )
            {
                safety_servo( robot_ptr, output.new_position );
                output.pass_to_input( input );
            }

            if ( res != ruckig::Result::Finished )
            {
//                PLOG_ERROR << "OTG 计算失败,停止运动";
                for ( int i = 0; i < _joint_num; ++i )
                    robot_ptr->joints_[ i ]->setPosition( robot_ptr->pos_[ i ] );
            }
            else {
//                PLOG_INFO << "关节空间急停已完成";
            }
        }
        catch ( const std::exception& e )
        {
            PLOG_ERROR << e.what( );
            for ( int i = 0; i < _joint_num; ++i )
                robot_ptr->joints_[ i ]->setPosition( robot_ptr->pos_[ i ] );
        }
        catch ( ... )
        {
            PLOG_ERROR << "未知错误";
            for ( int i = 0; i < _joint_num; ++i )
                robot_ptr->joints_[ i ]->setPosition( robot_ptr->pos_[ i ] );
        }
    }

    int check_vel_acc( const KDL::JntArray& current_pos, const KDL::JntArray& last_pos, const KDL::JntArray& last_last_pos, const double max_vel, const double max_acc )
    {
        KDL::JntArray current_vel( _joint_num );
        KDL::JntArray last_vel( _joint_num );
        KDL::JntArray current_acc( _joint_num );

        KDL::Subtract( current_pos, last_pos, current_vel );
        KDL::Divide( current_vel, DELTA_T, current_vel );

        KDL::Subtract( last_pos, last_last_pos, last_vel );
        KDL::Divide( last_vel, DELTA_T, last_vel );

        KDL::Subtract( current_vel, last_vel, current_acc );
        KDL::Divide( current_acc, DELTA_T, current_acc );

        for ( int i{ 0 }; i < _joint_num; i++ )
        {
            if ( abs( current_vel( i ) ) > max_vel )
            {
                PLOG_ERROR << "joint[" << i << "] velocity is too  fast";
                PLOG_ERROR << "target velocity = " << current_vel( i ) * KDL::rad2deg;
                PLOG_ERROR << "max velocity=" << max_vel * KDL::rad2deg;
                return -1;
            }

            if ( abs( current_acc( i ) ) > max_acc )
            {
                PLOG_ERROR << "joint[" << i << "] acceleration is too  fast";
                PLOG_ERROR << "target acceleration = " << current_acc( i ) * KDL::rad2deg;
                PLOG_ERROR << "max acceleration=" << max_acc * KDL::rad2deg;
                return -1;
            }
        }
        return 0;
    }

    int safety_servo( rocos::Robot* robot_ptr, const std::vector< double >& target_pos )
    {
        //** 伺服位置检查，无效则报错并程序终止 **//
        for ( int i = 0; i < _joint_num; ++i )
        {
            if ( target_pos[ i ] > robot_ptr->joints_[ i ]->getMaxPosLimit( ) || target_pos[ i ] < robot_ptr->joints_[ i ]->getMinPosLimit( ) )
            {
                PLOG_ERROR << "joint [" << i + 1 << "] ( " << target_pos[ i ] * KDL::rad2deg << " ) is out of range ";
                return -1;
            }
        }
        //**-------------------------------**//

        //** 位置伺服 **//
        for ( int i = 0; i < _joint_num; ++i )
        {
            robot_ptr->pos_[ i ] = target_pos[ i ];
            robot_ptr->joints_[ i ]->setPosition( target_pos[ i ] );
        }
        robot_ptr->hw_interface_->waitForSignal( 0 );
        //**-------------------------------**//
        return 0;
    }

    int safety_servo( rocos::Robot* robot_ptr, const KDL::JntArray& target_pos )
    {
        //** 伺服位置检查，无效则报错并程序终止 **//
        for ( int i = 0; i < _joint_num; ++i )
        {
            if ( target_pos( i ) > robot_ptr->joints_[ i ]->getMaxPosLimit( ) || target_pos( i ) < robot_ptr->joints_[ i ]->getMinPosLimit( ) )
            {
                PLOG_ERROR << "joint [" << i + 1 << "] ( " << target_pos( i ) * KDL::rad2deg << " ) is out of range ";
                return -1;
            }
        }
        //**-------------------------------**//

        //** 位置伺服 **//
        for ( int i = 0; i < _joint_num; ++i )
        {
            robot_ptr->pos_[ i ] = target_pos( i );
            robot_ptr->joints_[ i ]->setPosition( target_pos( i ) );
        }
        robot_ptr->hw_interface_->waitForSignal( 0 );
        //**-------------------------------**//
        return 0;
    }
}  // namespace JC_helper
