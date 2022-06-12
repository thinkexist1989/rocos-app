#include "JC_helper.hpp"
#include "robot.h"
/** 待处理问题：
 * 2. dragging 笛卡尔空间版 加入
 * 3. drggging 关节空间改进，允许给笛卡尔位姿
 */

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

            //!防止误判
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
        KDL::Vector tmpv = F_base_circleend.p - F_base_circleCenter.p;  //第二直线段上的半径段
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
        return KDL::Frame( KDL::Rotation::Quaternion( Quaternion_interp[ 0 ], Quaternion_interp[ 1 ], Quaternion_interp[ 2 ], Quaternion_interp[ 3 ] ), F_base_circleCenter_ * Vector{ radius * cos( s_p * alpha ), radius * sin( s_p * alpha ), 0 } );
    }

    KDL::Frame link( const KDL::Frame& start, const KDL::Frame& end, double s_p, double s_r, bool& success )
    {
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
        success           = true;
        return KDL::Frame( KDL::Rotation::Quaternion( Quaternion_interp[ 0 ], Quaternion_interp[ 1 ], Quaternion_interp[ 2 ], Quaternion_interp[ 3 ] ), P );
    }

    int link_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& start, const KDL::Frame& end, double v_start, double v_end, double max_path_v, double max_path_a )
    {
        if ( end == start )  //起始和终止位置一致，无需规划
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

            //大旋转，小移动(或没移动)情况，此时要求起始速度为0
            if ( Rlength > Plength )
            {
                if ( v_start != 0 || v_end != 0 )
                {
                    PLOG_ERROR << "Error_MotionPlanning_Not_Feasible";
                    return -1;
                }
                Path_length = Rlength;
            }
            //大移动，小旋转
            else
                Path_length = Plength;

            doubleS_P.planDoubleSProfile( 0, 0, 1, v_start / Path_length, v_end / Path_length, max_path_v / Path_length, max_path_a / Path_length, max_path_a * 2 / Path_length );
            bool isplanned = doubleS_P.isValidMovement( );
            if ( !isplanned || !( doubleS_P.getDuration( ) > 0 ) )
            {
                PLOG_ERROR << "Error_MotionPlanning_Not_Feasible";
                return -1;
            }

            doubleS_R.planDoubleSProfile( 0, 0, 1, 0, 0, max_path_v / Path_length, max_path_a / Path_length, max_path_a * 2 / Path_length );
            isplanned = doubleS_R.isValidMovement( );
            if ( !isplanned || !( doubleS_R.getDuration( ) > 0 ) )
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
            bool link_success{ true };
            KDL::Frame link_target{ };

            //** 轨迹计算 **//
            while ( dt >= 0 && dt <= T_link )
            {
                s_p         = doubleS_P.pos( dt );
                s_r         = doubleS_R.pos( dt );
                link_target = link( start, end, s_p, s_r, link_success );
                if ( !link_success )
                {
                    PLOG_ERROR << " link calculating failure";
                    return -1;
                }
                traj.push_back( link_target );
                dt = dt + 0.001;
            }
            //**-------------------------------**//
            return 0;
        }
    }

    int link_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& start, const KDL::Frame& end, double max_path_v, double max_path_a )
    {
        PLOG_DEBUG << "start = \n"
                   << start;

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
        bool link_success{ true };
        //**-------------------------------**//

        //** 轨迹计算 **//
        while ( dt <= duration )
        {
            s            = doubleS.pos( dt );
            interp_frame = link( start, end, s, s, link_success );
            if ( !link_success )
            {
                PLOG_ERROR << " link calculating failure";
                return -1;
            }
            traj.push_back( interp_frame );
            dt += 0.001;
        }
        //**-------------------------------**//

        return 0;
    }

    int multilink_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& f_start, const KDL::Frame& f_mid, const KDL::Frame& f_end, KDL::Frame& next_f_start, double current_path_start_v, double& next_path_start_v, double bound_dist, double max_path_v, double max_path_a, double next_max_path_v )
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
        //**-------------------------------**//

        //** 数据有效性检查 **//
        //!注释，为了通过只旋转，不移动时的检查
        // if ( abdist < eps )
        // {
        //     std::cout << RED << "multi_link_trajectory()：f_start and  f_mid are too close " << GREEN << std::endl;
        //     return -1;
        // }

        // if ( bcdist < eps )
        // {
        //     std::cout << RED << "multi_link_trajectory():f_mid and  f_end are too close " << GREEN << std::endl;
        //     return -1;
        // }

        if ( abdist < eps )  //说明本段是只旋转，不移动，那么要求开始速度为0
        {
            if ( current_path_start_v != 0 )  //不为0，只有一种情况，上段bound_dist等于abdist等于bcdist
            {
                std::cout << RED << "multi_link_trajectory():bound_dist of last motion  is too  large，try to decrease it" << GREEN << std::endl;
                return -1;
            }
        }

        if ( current_path_start_v < eps && abdist > eps && ( abdist - bound_dist ) < eps )  //只存在圆弧段，且上段速度为0，在圆弧匀速约束这不允许
        {
            std::cout << RED << " multi_link_trajectory()：the Link length is not allowed to be equal to 0,When starting velocity of this motion is equal to 0" << GREEN << std::endl;
            return -1;
        }

        if ( bound_dist > abdist )
        {
            std::cout << RED << "multi_link_trajectory():bound_dist is too  large，try to decrease it" << GREEN << std::endl;
            return -1;
        }

        if ( bound_dist > bcdist )  //这个条件说明：【正常情况下】如果本段只旋转，不移动，则上段bound_dist必需=0，则本段开始速度一定为0
        {
            std::cout << RED << "multi_link_trajectory():bound_dist is too  large，try to decrease it" << GREEN << std::endl;
        }

        if ( current_path_start_v > max_path_v )  //! 防止起始速度就超过最大可达线速度 ,理论上不可能发生
        {
            std::cout << RED << " current_path_start_v  > max_path_v" << std::endl;
            std::cout << "current_path_start_v = " << current_path_start_v << std::endl;
            std::cout << " max_path_v = " << max_path_v << GREEN << std::endl;
            return -1;
        }

        //**-------------------------------**//

        //利用向量乘积公式，求得两向量的夹角 ,并且限定范围在0-180
        double cos_alpha = std::max( -1., std::min( dot( ab, bc ) / abdist / bcdist, 1. ) );
        //两段直线夹角接近0,则两段直线合并为一条处理
        if ( ( 1 - cos_alpha ) <= eps )
        {
            next_f_start      = f_start;
            next_path_start_v = current_path_start_v;
            return 0;
        }
        //两段直线夹角接近180，则不允许圆弧过渡
        else if ( ( cos_alpha - ( -1 ) ) <= eps )
            bound_dist = 0;

        //求解两段直线的夹角和圆弧的半径
        double alpha  = acos( cos_alpha );
        double radius = bound_dist * tan( ( M_PI - alpha ) / 2 );

        //求解过渡半径占总长的百分比
        double s_bound_dist_1 = bound_dist == 0 ? 0 : bound_dist / abdist;
        double s_bound_dist_2 = bound_dist == 0 ? 0 : bound_dist / bcdist;  //避免只旋转时出现0/0
        bool link_success{ true };

        Frame F_base_circlestart = link( f_start, f_mid, 1 - s_bound_dist_1, 1 - s_bound_dist_1, link_success );
        if ( !link_success )
        {
            PLOG_ERROR << "link calculation failure";
            return -1;
        }

        Frame F_base_circleend = link( f_mid, f_end, s_bound_dist_2, s_bound_dist_2, link_success );
        if ( !link_success )
        {
            PLOG_ERROR << "link calculation failure";
            return -1;
        }

        Vector de     = F_base_circlestart.p - f_start.p;  //实际要走的直线段的距离
        double dedist = de.Norm( );                        //实际要走的直线段的距离

        Vector V_base_t = ab * ( ab * bc );  //圆弧中垂直于第一段直线的半径向量
        V_base_t.Normalize( );

        Frame F_base_circleCenter{ F_base_circlestart.p - V_base_t * radius };

        double s_cirlular_v{ 0 };

        if ( s_bound_dist_1 <= eps )  //不存在圆弧，圆弧速度为0
            s_cirlular_v = 0;
        else if ( ( 1 - s_bound_dist_1 ) <= eps )
            s_cirlular_v = current_path_start_v / ( radius * alpha );  //不存在直线，圆弧速度为当前段运动速度
        else
            s_cirlular_v = s_bound_dist_1 * std::min( max_path_v, next_max_path_v ) / ( radius * alpha );  //考虑当前和下次的运动，选取最小值（代表当前运动和下一段运动的约束下，最大可行速度）

#pragma region  // 第一段直线速度轨迹规划

        double T_link = 0;
        ::rocos::DoubleS doubleS_1_P;
        ::rocos::DoubleS doubleS_1_R;
        //存在直线，才需要规划
        if ( ( 1 - s_bound_dist_1 ) > eps )
        {
            double Path_length{ dedist };
            // dedist==0,代表只旋转，不移动的情况
            if ( Path_length == 0 )
            {
                KDL::Rotation R_start_end = f_start.M.Inverse( ) * F_base_circlestart.M;
                KDL::Vector ration_axis;
                double angle                   = R_start_end.GetRotAngle( ration_axis );
                const double equivalent_radius = 0.1;
                double Rlength                 = ( equivalent_radius * abs( angle ) );
                Path_length                    = Rlength;
            }

            doubleS_1_P.planDoubleSProfile( 0, 0, 1, current_path_start_v / Path_length, s_cirlular_v, max_path_v / Path_length, max_path_a / Path_length, max_path_a * 2 / Path_length );
            bool isplanned = doubleS_1_P.isValidMovement( );
            if ( !isplanned || !( doubleS_1_P.getDuration( ) > 0 ) )
            {
                std::cout << RED << "multi_link_trajectory()：Linear trajectory planning fails,try to decrease given parameter [max_path_v] " << GREEN << std::endl;
                return -1;
            }

            doubleS_1_R.planDoubleSProfile( 0, 0, 1, 0, 0, max_path_v / Path_length, max_path_a / Path_length, max_path_a * 2 / Path_length );
            isplanned = doubleS_1_R.isValidMovement( );
            if ( !isplanned || !( doubleS_1_R.getDuration( ) > 0 ) )
            {
                std::cout << RED << "multi_link_trajectory()：Linear trajectory planning fails,try to decrease given parameter [max_path_v] " << GREEN << std::endl;
                return -1;
            }

            if ( doubleS_1_R.getDuration( ) != doubleS_1_P.getDuration( ) )
            {
                doubleS_1_R.JC_scaleToDuration( doubleS_1_P.getDuration( ) );
            }

            T_link       = doubleS_1_P.getDuration( );
            s_cirlular_v = doubleS_1_P.vel( doubleS_1_P.getDuration( ) );  //!预防doubleS_1_P没有达到预期的速度，理论应该不会
        }
#pragma endregion

        if ( s_bound_dist_1 <= eps )  //不存在圆弧，下段速度为0
        {
            next_path_start_v = 0;
        }
        else if ( ( 1 - s_bound_dist_1 ) <= eps )  //不存在直线，下段速度为当前段运动速度
        {
            next_path_start_v = current_path_start_v;
        }
        else
        {
            next_path_start_v = s_cirlular_v * dedist;  //下段开始速度即为计算出的圆弧速度
        }
        next_f_start = F_base_circleend;  //下一段运动应该开始的位姿

#pragma region  //圆弧段时间
        double T_cirlular = 0;
        ::rocos::DoubleS doubleS_2_R;
        if ( s_bound_dist_1 > eps )
        {
            T_cirlular = 1 / ( next_path_start_v / ( radius * alpha ) );

            doubleS_2_R.planDoubleSProfile( 0, 0, 1, 0, 0, max_path_v / dedist, max_path_a / dedist, max_path_a * 2 / dedist );
            bool isplanned = doubleS_2_R.isValidMovement( );
            if ( !isplanned || !( doubleS_2_R.getDuration( ) > 0 ) )
            {
                std::cout << RED << "multi_link_trajectory()：Circular trajectory planning fails,try to decrease given parameter [max_path_v] " << std::endl;
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

                link_target = link( f_start, F_base_circlestart, s_p, s_r, link_success );
                if ( !link_success )
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

            t_total = t_total + 0.001;
        }
        //**-------------------------------**//
        return 0;
    }

    int circle_center( KDL::Frame& center, const KDL::Frame& f_p1, const KDL::Frame& f_p2, const KDL::Frame& f_p3 )
    {
        const double eps = 1E-7;
        using namespace KDL;
        Vector v1 = f_p2.p - f_p1.p;
        Vector v2 = f_p3.p - f_p1.p;

        if ( v1.Normalize( ) < eps )
        {
            std::cout << RED << "circle_center():f_p1不能等于f_p2" << std::endl;
            return -1;
        }
        if ( v2.Normalize( ) < eps )
        {
            std::cout << RED << "circle_center():f_p1不能等于f_p3" << std::endl;
            return -1;
        }

        Vector axis_z{ v2 * v1 };

        if ( axis_z.Normalize( ) < eps )
        {
            std::cout << RED << "circle_center():三点不能共线或过于趋向直线" << std::endl;
            return -1;
        }

        Vector axis_x{ v1 };
        Vector axis_y{ axis_z * axis_x };
        axis_y.Normalize( );

        v1 = f_p2.p - f_p1.p;
        v2 = f_p3.p - f_p1.p;

        //在新坐标系上
        // f_p2 = [dot(v1,axis_x),0] = [bx,0]
        // f_P3 = [dot(v2,axis_x),dot(v2,axis_y)=[cx,cy]
        //圆心一定位于[bx/2,0]的直线上，所以假设圆心为[bx/2,0]
        //在利用半径相等公式求解h

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
        Frame center;

        if ( circle_center( center, f_p1, f_p2, f_p3 ) < 0 )  //找到圆心的位置
        {
            std::cout << RED << "circle_trajectory():unable to calculate center of circle" << std::endl;
            return -1;
        }

        KDL::Vector axis_x = f_p1.p - center.p;

        double radius = axis_x.Normalize( );

        KDL::Vector axis_tem = f_p2.p - center.p;  //第二直线段上的半径段
        axis_tem.Normalize( );

        Vector axis_z( axis_x * axis_tem );  //向上的Z轴
        if ( axis_z.Normalize( ) < epsilon )
        {
            std::cout << RED << "circle_trajectory():axis_x and axis_tem  cannot be parallel" << std::endl;
            return -1;
        }

        Vector axis_y{ ( axis_z * axis_x ) };
        axis_y.Normalize( );

        KDL::Frame center_( KDL::Rotation{ axis_x, axis_y, axis_z }, center.p );  //确定圆心的方向

        //** 计算旋转角度 **//
        Vector f_p2_   = center_.Inverse( ) * f_p2.p;
        Vector f_p3_   = center_.Inverse( ) * f_p3.p;
        double theta12 = 0;
        double theta13 = 0;
        if ( f_p2_( 1 ) < 0 )
            theta12 = atan2( f_p2_( 1 ), f_p2_( 0 ) ) + 2 * M_PI;
        else
            theta12 = atan2( f_p2_( 1 ), f_p2_( 0 ) );
        if ( f_p3_( 1 ) < 0 )
            theta13 = atan2( f_p3_( 1 ), f_p3_( 0 ) ) + 2 * M_PI;
        else
            theta13 = atan2( f_p3_( 1 ), f_p3_( 0 ) );
        std::cout << BLUE << "theta13= " << theta13 * 180 / M_PI << GREEN << std::endl;
        //**-------------------------------**//

        ::rocos::DoubleS doubleS;
        double path_length = ( radius * theta13 );
        doubleS.planDoubleSProfile( 0, 0, 1, 0, 0, max_path_v / path_length, max_path_a / path_length, 2 * max_path_a / path_length );
        bool success = doubleS.isValidMovement( );
        if ( success < 0 || !( doubleS.getDuration( ) > 0 ) )
        {
            std::cout << RED << "circle_trajectory():planDoubleSProfile failed" << std::endl;
            return -1;
        }
        double T_total = doubleS.getDuration( );
        std::cout << BLUE << "T_total = " << T_total << GREEN << std::endl;

        KDL::Rotation R_w_p1 = f_p1.M;
        Vector Rotation_axis = R_w_p1.Inverse( ) * axis_z;

        //** 轨迹规划 **//
        double dt{ 0.0 };
        double s{ 0.0 };
        while ( dt <= T_total )
        {
            s = doubleS.pos( dt );
            if ( fixed_rotation )
                traj.push_back( KDL::Frame( R_w_p1, center_ * Vector{ radius * cos( s * theta13 ), radius * sin( s * theta13 ), 0 } ) );
            else
                traj.push_back( KDL::Frame( R_w_p1 * KDL::Rotation::Rot2( Rotation_axis, theta13 * s ), center_ * Vector{ radius * cos( s * theta13 ), radius * sin( s * theta13 ), 0 } ) );
            dt += 0.001;
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
        bool success = doubleS.isValidMovement( );
        if ( success < 0 || !( doubleS.getDuration( ) > 0 ) )
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
            dt += 0.001;
        }
        return 0;
    }
#pragma endregion

#pragma region  //*关节空间点动功能实现
    SmartServo_Joint::SmartServo_Joint( std::atomic< bool >* finished_flag_ptr )
    {
        external_finished_flag_ptr = finished_flag_ptr;
    }

    //! init()只负责轨迹的信息重置，运行状态Flag由各运动线程结束后{手动重置}
    void SmartServo_Joint::init( std::vector< double > q_init, std::vector< double > v_init, std::vector< double > a_init, double max_v, double max_a, double max_j )
    {
        input.control_interface = ruckig::ControlInterface::Position;
        input.synchronization   = ruckig::Synchronization::Phase;

        for ( int i = 0; i < _joint_num; i++ )
        {
            input.current_position[ i ]     = q_init[ i ];
            input.current_velocity[ i ]     = v_init[ i ];
            input.current_acceleration[ i ] = a_init[ i ];

            input.target_position[ i ]     = q_init[ i ];
            input.target_velocity[ i ]     = v_init[ i ];
            input.target_acceleration[ i ] = a_init[ i ];

            input.max_velocity[ i ]     = max_v;
            input.max_acceleration[ i ] = max_a;
            input.max_jerk[ i ]         = max_j;
        }

        PLOG_INFO << "smart servo init succesed";
    }

    void SmartServo_Joint::RunSmartServo( rocos::Robot* robot_ptr )
    {
        //** 变量初始化 **//
        std::unique_lock< std::mutex > input_lock( input_mutex, std::defer_lock );
        ruckig::Result res;
        int count{ 0 };
        int _tick_count{ robot_ptr->tick_count };
        //**-------------------------------**//

        //第一次启动需要等待command()
        while ( *external_finished_flag_ptr )
        {
            std::this_thread::sleep_for( std::chrono::duration< double >{ 0.003 } );
            PLOG_INFO << "waiting for command";
        }

        while ( 1 )
        {
            input_lock.lock( );
            res = otg.update( input, output );
            input_lock.unlock( );

            if ( res == ruckig::Result::Finished )
            {
                ( *external_finished_flag_ptr ) = true;   //这次smart servo已结束，等待下一次smart servo
                robot_ptr->is_running_motion    = false;  //机械臂运动已结束，可以执行其他离线类运动
                on_stop_trajectory              = false;  //这个必须设为false,因为新线程仍然使用同一个对象数据成员
                PLOG_INFO << "smart servo has finished";
                break;
            }
            else if ( res == ruckig::Result::Working )
            {
                const auto& p = output.new_position;

                for ( int i = 0; i < _joint_num; ++i )
                {
                    robot_ptr->pos_[ i ] = p[ i ];
                    robot_ptr->joints_[ i ]->setPosition( p[ i ] );
                }
                // PLOG_DEBUG << "p[ 0 ]=" << p[ 0 ] << " p[ 1]=" << p[ 1 ] ;
                input_lock.lock( );
                output.pass_to_input( input );
                input_lock.unlock( );

                robot_ptr->hw_interface_->waitForSignal( 0 );
            }
            //** 100ms进行一次心跳检查,紧急停止时不需要检查 **//
            if ( ( ( ++count ) == 100 ) && !on_stop_trajectory )
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
                    input_lock.lock( );
                    input.control_interface = ruckig::ControlInterface::Velocity;
                    input.synchronization   = ruckig::Synchronization::None;

                    for ( int i = 0; i < _joint_num; i++ )
                    {
                        input.target_velocity[ i ]     = 0.0;
                        input.target_acceleration[ i ] = 0.0;
                        input.max_velocity[ i ]        = robot_ptr->joints_[ i ]->getMaxVel( );
                        input.max_acceleration[ i ]    = robot_ptr->joints_[ i ]->getMaxAcc( );
                        input.max_jerk[ i ]            = robot_ptr->joints_[ i ]->getMaxJerk( );
                    }
                    input_lock.unlock( );
                }
            }
            //**-------------------------------**//
        }
    }

    void SmartServo_Joint::command( KDL::JntArray q_target )
    {
        if ( !on_stop_trajectory )  //如果需要紧急停止，那么就不允许在更改指令了
        {
            std::unique_lock< std::mutex > input_lock( input_mutex );

            for ( int i = 0; i < _joint_num; i++ )
            {
                input.target_position[ i ]     = q_target( i );
                input.target_velocity[ i ]     = 0.0;
                input.target_acceleration[ i ] = 0.0;
            }

            ( *external_finished_flag_ptr ) = false;
        }
        else
        {
            PLOG_DEBUG << "control is not allowed during emergency stop";
        }
    }

#pragma endregion

#pragma region  //*笛卡尔空间点动功能实现
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
        double t_last_rotation{ 0.001 };             //上段姿态运行时刻
        double dt_link{ 0 };                         //当前段段姿态运行时刻
        constexpr double sleep_time{ 0.001 * 0.8 };  //最长睡眠时间为1ms,最高于这个值会导致运动不连续

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
#pragma region  //** 公共参数设置 **//

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
#pragma endregion  //**-------------------------------**//

                        //! 0度平行和只旋转目标 处理在此
                        if ( ( 1 - cos_alpha ) <= big_eps || bc.Norm( ) < 5 * big_eps )
                        {
                            PLOG_DEBUG;
                            alpha  = 0;
                            radius = 0;

#pragma region  //**路径在线doubleS参数设置 **//
                            double length_circle_link{ ( target.p - path_p.p ).Norm( ) };

                            if ( length_circle_link <= eps )  //!两段直线都为虚直线，在后面会重新正确计算
                                _OnlineDoubleS.calculate( 0, 1, 0, 0, 0, 0, max_path_v, max_path_a, 2 * max_path_a );
                            else
                                _OnlineDoubleS.calculate( 0, 1, path_v / length_circle_link, 0, path_a / length_circle_link, 0, max_path_v / length_circle_link, max_path_a / length_circle_link, 2 * max_path_a / length_circle_link );

                            KDL::Rotation R_stat_end = last_target.M.Inverse( ) * target.M;
                            KDL::Vector ration_axis;
                            double angle = R_stat_end.GetRotAngle( ration_axis );  //新目标的旋转角度

#pragma endregion  //**-------------------------------**//

#pragma region  //**上段姿态在线doubleS参数设置 **//
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

#pragma endregion  //**-------------------------------**//

#pragma region  //**本段姿态在线doubleS参数设置 **//

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

#pragma endregion  //**-------------------------------**//

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

#pragma region  //**路径在线doubleS参数设置 **//

                            double s_p{ 0 };
                            double sd_p{ 0 };
                            double sdd_p{ 0 };

                            double length_circle_link{ ( last_target.p - path_p.p ).Norm( ) };
                            _OnlineDoubleS.calculate( 0, 1, path_v / length_circle_link, 0, path_a / length_circle_link, 0, max_path_v / length_circle_link, max_path_a / length_circle_link, 2 * max_path_a / length_circle_link );

#pragma endregion  //**-------------------------------**//

#pragma region  //**姿态在线doubleS参数设置 **//
                            double s_r{ 0 };
                            double sd_r{ 0 };
                            double sdd_r{ 0 };

                            if ( flag_from_last_rotation )
                                _last_rotaion_OnlineDoubleS.get_pos_vel_acc( t_last_rotation - 0.001, s_r, sd_r, sdd_r );
                            else
                                _rotaion_OnlineDoubleS.get_pos_vel_acc( dt_link - 0.001, s_r, sd_r, sdd_r );

                            _last_rotaion_OnlineDoubleS.calculate( s_r, 1, sd_r, 0, sdd_r, 0, 1e7, 1e7, 1e7, _OnlineDoubleS.get_duration( ) );
                            double last_ratation_duration{ _last_rotaion_OnlineDoubleS.get_duration( ) };

#pragma endregion  //**-------------------------------**//

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
#pragma region  //** 圆弧参数设置 **//

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
#pragma endregion  //**-------------------------------**//

#pragma region  //**路径在线doubleS参数设置 **//
                            double length_circle_link{ ( ( target.p - F_base_circleend.p ).Norm( ) < eps ? 0 : ( target.p - F_base_circleend.p ).Norm( ) ) + alpha * radius };
                            double bound_length = alpha * radius / length_circle_link;  //圆弧段长占总长百分比
                            _OnlineDoubleS.calculate( 0, 1, path_v / length_circle_link, 0, path_a / length_circle_link, 0, max_path_v / length_circle_link, max_path_a / length_circle_link, 2 * max_path_a / length_circle_link );

#pragma endregion  //**-------------------------------**//

#pragma region  //** 直线段参数设置 **//

                            KDL::Rotation R_stat_end = last_target.M.Inverse( ) * target.M;
                            KDL::Vector ration_axis;
                            double angle           = R_stat_end.GetRotAngle( ration_axis );
                            double ratation_length = ( equivalent_radius * abs( angle ) );
                            double path_length     = ( target.p - F_base_circleend.p ).Norm( );
                            double length          = std::max( ratation_length, path_length );
#pragma endregion  //**-------------------------------**//

#pragma region  //**上段姿态在线doubleS参数设置 **//
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

#pragma endregion  //**-------------------------------**//

#pragma region  //**本段姿态在线doubleS参数设置 **//

                            _rotaion_OnlineDoubleS.calculate( 0, 1, 0, 0, 0, 0, 1e7, 1e7, 1e7, _OnlineDoubleS.get_duration( ) * ( 1 - radio_angle ) );
                            double ratation_duration{ abs( angle ) < eps ? 0 : _rotaion_OnlineDoubleS.get_duration( ) };

#pragma endregion  //**-------------------------------**//

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
        KDL::JntArray _q_target( _joint_num );

        std::vector< double > max_step;
        //**-------------------------------**//

        for ( int i = 0; i < _joint_num; i++ )
        {
            max_step.push_back( robot_ptr->max_vel_[ i ] * 0.001 );
        }

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

            //** 速度保护 **//
            for ( int i = 0; i < _joint_num; i++ )
            {
                if ( abs( _q_target( i ) - _q_init( i ) ) > max_step[ i ] )
                {
                    PLOG_ERROR << "joint[" << i << "] speep is too  fast";
                    PLOG_ERROR << "target speed = " << abs( _q_target( i ) - _q_init( i ) )
                               << " and  max_step=" << max_step[ i ];
                    PLOG_ERROR << "_q_target( " << i << " )  = " << _q_target( i ) * 180 / M_PI;
                    PLOG_ERROR << "_q_init( " << i << " ) =" << _q_init( i ) * 180 / M_PI;
                    on_stop_trajectory = true;
                    break;
                }
            }
            if ( on_stop_trajectory ) break;
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
        std::array< double, _joint_num > joints_vel{ 0 };
        std::array< double, _joint_num > joints_last_vel{ 0 };
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
            //**-------------------------------**//

            //** 位置伺服 **//
            for ( int i = 0; i < _joint_num; ++i )
            {
                joints_last_vel[ i ] = joints_vel[ i ];
                joints_vel[ i ]      = robot_ptr->joints_[ i ]->getVelocity( );
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
            //!紧急停止措施，对于仿真是立刻停止，实物才能看到效果
            motion_stop( robot_ptr, joints_last_vel, joints_vel, 0.002 );
            //! 触发急停后就冷静3秒，防止手一直按着触发急停
            std::this_thread::sleep_for( std::chrono::duration< double >{ 3 } );
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
            return command( target * KDL::Frame{ target.M.Inverse( ) * tem_v } );
        else
            return -1;
    }
    int SmartServo_Cartesian::command( const KDL::Rotation& tem_r, const char* str )
    {
        if ( strcmp( str, "FLANGE" ) == 0 )
            return command( target * KDL::Frame{ tem_r } );
        else if ( strcmp( str, "BASE" ) == 0 )
            return command( KDL::Frame{ tem_r * target.M, target.p } );
        else
            return -1;
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

    void motion_stop( rocos::Robot* robot_ptr, std::array< double, _joint_num > joints_last_vel, std::array< double, _joint_num > joints_vel, double dt )
    {
        ruckig::Ruckig< _joint_num > otg{ 0.001 };
        ruckig::InputParameter< _joint_num > input;
        ruckig::OutputParameter< _joint_num > output;
        ruckig::Result res;
        input.control_interface = ruckig::ControlInterface::Velocity;
        input.synchronization   = ruckig::Synchronization::None;

        for ( int i = 0; i < _joint_num; i++ )
        {
            input.current_position[ i ]     = robot_ptr->pos_[ i ];
            input.current_velocity[ i ]     = KDL::sign( joints_vel[ i ] ) * std::min( abs( joints_vel[ i ] ), robot_ptr->joints_[ i ]->getMaxVel( ) );
            input.current_acceleration[ i ] = KDL::sign( joints_vel[ i ] - joints_last_vel[ i ] ) * std::min( abs( ( joints_vel[ i ] - joints_last_vel[ i ] ) / dt ), robot_ptr->joints_[ i ]->getMaxAcc( ) );
            input.target_position[ i ]      = input.current_position[ i ];
            input.target_velocity[ i ]      = 0;
            input.target_acceleration[ i ]  = 0;

            input.max_velocity[ i ]     = robot_ptr->joints_[ i ]->getMaxVel( );
            input.max_acceleration[ i ] = robot_ptr->joints_[ i ]->getMaxAcc( );
            input.max_jerk[ i ]         = robot_ptr->joints_[ i ]->getMaxJerk( );
        }

        while ( ( res = otg.update( input, output ) ) != ruckig::Result::Finished )
        {
            const auto& p = output.new_position;

            for ( int i = 0; i < _joint_num; ++i )
            {
                robot_ptr->pos_[ i ] = p[ i ];
                robot_ptr->joints_[ i ]->setPosition( p[ i ] );
            }
            output.pass_to_input( input );
            robot_ptr->hw_interface_->waitForSignal( 0 );
        }
    }
#pragma endregion

#pragma region  //* 6维力传感器

    ft_sensor::ft_sensor( const char* ip_dress ) : _ip_dress{ ip_dress }
    {
    }

    ft_sensor::~ft_sensor( )
    {
        UdpClose( &socketHandle );
    }

    int ft_sensor::init( KDL::Frame flange_pos )
    {
        if ( Connect( &socketHandle, _ip_dress.c_str( ), FT_PORT ) != 0 )
        {
            PLOG_ERROR << "Could not connect to device...";
            return -1;
        }
        SendCommand( &socketHandle, COMMAND_SPEED, FT_SPEED );
        SendCommand( &socketHandle, COMMAND_FILTER, FT_FILTER );
        // ! 爪子装上后，零漂消除不应该打开了，只有空载情况下才可以打开零漂消除
        SendCommand( &socketHandle, COMMAND_BIAS, FT_BIASING_OFF );

        std::this_thread::sleep_for( std::chrono::duration< double >{ 1 } );

        SendCommand( &socketHandle, COMMAND_START, 1 );
        res = Receive( &socketHandle );

        init_force_torque.force[ 0 ]  = res.fx / FORCE_DIV;
        init_force_torque.force[ 1 ]  = res.fy / FORCE_DIV;
        init_force_torque.force[ 2 ]  = res.fz / FORCE_DIV;
        init_force_torque.torque[ 0 ] = res.tx / TORQUE_DIV;
        init_force_torque.torque[ 1 ] = res.ty / TORQUE_DIV;
        init_force_torque.torque[ 2 ] = res.tz / TORQUE_DIV;

        //将起始收到的力信息转变到base坐标系下
        // TODO处理力矩
        init_force_torque.force = ( flange_pos * KDL::Frame{ KDL::Rotation::RPY( 0, 0, M_PI ), KDL::Vector( 0, 0, 0.035 ) } ) * init_force_torque.force;

        return 0;
    }

    void ft_sensor::getting_data( KDL::Frame flange_pos )
    {
        SendCommand( &socketHandle, COMMAND_START, 1 );
        res = Receive( &socketHandle );

        force_torque.force[ 0 ]  = res.fx / FORCE_DIV;
        force_torque.force[ 1 ]  = res.fy / FORCE_DIV;
        force_torque.force[ 2 ]  = res.fz / FORCE_DIV;
        force_torque.torque[ 0 ] = res.tx / TORQUE_DIV;
        force_torque.torque[ 1 ] = res.ty / TORQUE_DIV;
        force_torque.torque[ 2 ] = res.tz / TORQUE_DIV;

        //收到的力信息转换到base系
        force_torque.force = ( flange_pos * KDL::Frame{ KDL::Rotation::RPY( 0, 0, M_PI ), KDL::Vector( 0, 0, 0.035 ) } ) * force_torque.force;

        // 重力补偿
        force_torque.force = force_torque.force - init_force_torque.force;

        // TODO 处理力矩
        for ( int i{ 0 }; i < 3; i++ )
            force_torque.torque[ i ] = 0;

        //去除毛刺，限制3N
        for ( int i{ 0 }; i < 3; i++ )
            if ( abs( force_torque.force[ i ] ) < 3 )
                force_torque.force[ i ] = 0;
    }

    int ft_sensor::debug( KDL::Frame flange_pos )
    {
        getting_data( flange_pos );

        for ( int i{ 0 }; i < 3; i++ )
            PLOG_DEBUG.printf( "force[ %d ] = %f ", i, force_torque.force[ i ] );

        for ( int i{ 0 }; i < 3; i++ )
            PLOG_DEBUG.printf( "torque[ %d ] = %f ", i, force_torque.torque[ i ] );

        std::this_thread::sleep_for( std::chrono::duration< double >( 0.001 ) );
    }

#pragma endregion

#pragma region  //*导纳控制

    using AS = admittance::spring_mass_dump;

    AS::spring_mass_dump( )
    {   
        for ( int i{ 0 }; i < _joint_num; i++ )
            B[ i ] = 2 * 1 * sqrt( M[ i ] * K[ i ] );
    }

    void AS::calculate_translate( )
    {
        for ( int i{ 0 }; i < 3; i++ )
        {
            force_acc_offset[ i ] = ( TCP_force[ i ] - B[ i ] * force_vel_offset[ i ] - K[ i ] * force_pos_offset[ i ] ) / M[ i ];
            force_vel_offset[ i ] = admittance_dt * ( force_acc_offset[ i ] + force_last_acc_offset[ i ] ) / 2 + force_vel_offset[ i ];
            force_pos_offset[ i ] = admittance_dt * ( force_vel_offset[ i ] + force_last_vel_offset[ i ] ) / 2 + force_pos_offset[ i ];

            force_last_acc_offset[ i ] = force_acc_offset[ i ];
            force_last_vel_offset[ i ] = force_vel_offset[ i ];
        }
    }

    KDL::Rotation AS::calculate_rotation( )
    {
        KDL::Vector delta_rot;
        KDL::Vector current_rot;
        KDL::Rotation template_rot;

        for ( int i{ 0 }; i < 3; i++ )
        {
            torque_acc_offset[ i ] = ( TCP_torque[ i ] - B[ i ] * torque_vel_offset[ i ] - K[ i ] * torque_pos_offset[ i ] ) / M[ i ];
            torque_vel_offset[ i ] = admittance_dt * ( torque_acc_offset[ i ] + torque_last_acc_offset[ i ] ) / 2 + torque_vel_offset[ i ];

            delta_rot( i )   = admittance_dt * ( torque_vel_offset[ i ] + torque_last_vel_offset[ i ] ) / 2;
            current_rot( i ) = torque_pos_offset[ i ];

            torque_last_acc_offset[ i ] = torque_acc_offset[ i ];
            torque_last_vel_offset[ i ] = torque_vel_offset[ i ];
        }

        template_rot = KDL::Rotation::Rot( delta_rot, delta_rot.Norm( ) ) * KDL::Rotation::Rot( current_rot, current_rot.Norm( ) );

        current_rot = template_rot.GetRot( );

        for ( int i{ 0 }; i < 3; i++ )
            torque_pos_offset[ i ] = current_rot[ i ];

        return template_rot;
    }

    int AS::calculate( KDL::Frame& pos_offset  , double dt )
    {
        admittance_dt = dt;
        calculate_translate( );

        for ( int i{ 0 }; i < 3; i++ )
        {
            pos_offset.p[ i ] = force_pos_offset[ i ];
        }

        pos_offset.M = calculate_rotation( );

        return 0;
    }

    void AS::set_force( double force_x, double force_y, double force_z )
    {
        TCP_force[ 0 ] = force_x;
        TCP_force[ 1 ] = force_y;
        TCP_force[ 2 ] = force_z;
        // PLOG_DEBUG.printf( "TCP_force  = %f %f %f", TCP_force[ 0 ], TCP_force[ 1 ], TCP_force[ 2 ] );
    }

    void AS::set_torque( double tor_que_x, double tor_que_y, double tor_que_z )
    {
        TCP_torque[ 0 ] = tor_que_x;
        TCP_torque[ 1 ] = tor_que_y;
        TCP_torque[ 2 ] = tor_que_z;
        // PLOG_DEBUG.printf( "TCP_torque  = %f %f %f", TCP_torque[ 0 ], TCP_torque[ 1 ], TCP_torque[ 2 ] );
    }

    void AS::set_damp( double value )
    {
        static double damp = 1;
        damp += value;

        for ( int i{ 0 }; i < _joint_num; i++ )
            B[ i ] = 2 * damp * sqrt( M[ i ] * K[ i ] );

        PLOG_DEBUG << "damp  = " << damp;
    }

    int admittance::init( KDL::Frame flange_pos )
    {
        //** 6维力初始化 **//
        // if ( my_ft_sensor.init( flange_pos) < 0 )
        // {
        //     PLOG_ERROR << " force-torque sensor init failed ";
        //     return -1;
        // }
        //**-------------------------------**//

        PLOG_INFO << " init success";

        return 0;
    }

    void admittance::start( rocos::Robot* robot_ptr, const std::vector< KDL::Frame >& traj_target )
    {
        std::shared_ptr< std::thread > _thread_IK{ nullptr };
        std::shared_ptr< std::thread > _thread_motion{ nullptr };

        _thread_IK.reset( new std::thread{ &JC_helper::admittance::IK, this, robot_ptr, std::ref( traj_target ) } );
        _thread_motion.reset( new std::thread{ &JC_helper::admittance::motion, this, robot_ptr } );

        if ( traj_target.size( ) == 1 )  //示教模式
        {
            PLOG_INFO << " starting  teaching";

            std::string str;
            while ( str.compare( "break" ) != 0 )
            {
                PLOG_INFO << " enter 'break' to turnoff teaching function:";
                std::cin >> str;
            }
            on_stop_trajectory = true;
            _thread_motion->join( );
            _thread_IK->join( );
        }
        else  //运动导纳模式
        {
            PLOG_INFO << " starting  admittance motion";
            _thread_motion->join( );
            _thread_IK->join( );
        }

        PLOG_INFO << "admittance  全部结束";
    }

    void admittance::IK( rocos::Robot* robot_ptr, const std::vector< KDL::Frame >& traj_target )
    {
        //** 变量初始化 **//
        std::unique_lock< std::mutex > lock_traj_joint( mutex_traj_joint, std::defer_lock );  //不上锁
        KDL::Frame frame_target;
        int count = 0;
        KDL::JntArray _q_target( _joint_num );
        KDL::JntArray _q_init( _joint_num );
        std::vector< double > max_step;
        int_least64_t max_count{ 0 };
        //**-------------------------------**//

        for ( int i = 0; i < _joint_num; i++ )
        {
            max_step.push_back( robot_ptr->max_vel_[ i ] * 0.001 );
            _q_init( i )   = robot_ptr->pos_[ i ];
            _q_target( i ) = _q_init( i );
        }

        //** 轨迹计算 **//
        if ( traj_target.size( ) == 1 )  //示教模式
            max_count = numeric_limits< int_least64_t >::max( );
        else
            max_count = traj_target.size( );

        for ( int traj_count{ 0 }; traj_count < max_count; traj_count++ )
        {
            auto t_start = std::chrono::steady_clock::now( );

            //** 读取最新Frame **//
            if ( traj_target.size( ) == 1 )  //示教模式
                frame_target = frame_offset * traj_target[ 0 ];
            else
                frame_target = frame_offset * traj_target[ traj_count ];

            traj_count++;
            //**-------------------------------**//

            //** IK求解 **//
            if ( ( robot_ptr->kinematics_ ).CartToJnt( _q_init, frame_target, _q_target ) < 0 )
            {
                PLOG_ERROR << " CartToJnt failed  frame_target =\n"
                           << frame_target;
                on_stop_trajectory = true;
                break;
            }
            //**-------------------------------**//

            //** 速度保护 **//
            for ( int i = 0; i < _joint_num; i++ )
            {
                if ( abs( _q_target( i ) - _q_init( i ) ) > max_step[ i ] )
                {
                    PLOG_ERROR << "joint[" << i << "] speep is too  fast";
                    PLOG_ERROR << "target speed = " << abs( _q_target( i ) - _q_init( i ) )
                               << " and  max_step=" << max_step[ i ];
                    PLOG_ERROR << "_q_target( " << i << " )  = " << _q_target( i ) * 180 / M_PI;
                    PLOG_ERROR << "_q_init( " << i << " ) =" << _q_init( i ) * 180 / M_PI;
                    on_stop_trajectory = true;
                    break;
                }
            }

            if ( on_stop_trajectory ) break;
            //**-------------------------------**//

            lock_traj_joint.lock( );
            traj_joint.push_back( _q_target );
            lock_traj_joint.unlock( );
            _q_init = _q_target;

            auto t_stop     = std::chrono::steady_clock::now( );
            auto t_duration = std::chrono::duration< double >( t_stop - t_start );
            if ( t_duration.count( ) < 0.0008 )
            {
                std::this_thread::sleep_for( std::chrono::duration< double >( 0.0007 - t_duration.count( ) ) );
            }

            // TODO 导纳计算
            smd.calculate( frame_offset ,0.0008);
            smd.set_force( sin(0.001*traj_count)*5, 0, 0 );
            smd.set_torque( 0, 0, 0 );

        }

        if ( on_stop_trajectory )
            PLOG_ERROR << "IK 触发急停";
        else
            PLOG_INFO << "IK结束";
        FinishRunPlanningIK = true;
    }

    void admittance::motion( rocos::Robot* robot_ptr )
    {
        //** 变量初始化 **//
        std::unique_lock< std::mutex > lock_traj_joint( mutex_traj_joint, std::defer_lock );  //不上锁
        int traj_joint_count = 0;
        int count{ 0 };
        KDL::JntArray joint_command;
        int _tick_count{ robot_ptr->tick_count };
        std::array< double, _joint_num > joints_vel{ 0 };
        std::array< double, _joint_num > joints_last_vel{ 0 };
        //**-------------------------------**//

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
            //**-------------------------------**//

            //** 位置伺服 **//
            for ( int i = 0; i < _joint_num; ++i )
            {
                joints_last_vel[ i ] = joints_vel[ i ];
                joints_vel[ i ]      = robot_ptr->joints_[ i ]->getVelocity( );
                robot_ptr->pos_[ i ] = joint_command( i );
                robot_ptr->joints_[ i ]->setPosition( joint_command( i ) );
            }
            robot_ptr->hw_interface_->waitForSignal( 0 );
            //**-------------------------------**//
        }
        //**--------------------------------------------------------------**//

        //**紧急停止 ，可能原因：1.轨迹规划失败、2.IK求解失败 3.心跳保持超时**//
        if ( on_stop_trajectory )
        {
            PLOG_ERROR << "motion触发紧急停止";
            //!紧急停止措施，对于仿真是立刻停止，实物才能看到效果
            motion_stop( robot_ptr, joints_last_vel, joints_vel, 0.002 );
        }
        else
            PLOG_INFO << "motion 结束";

        // robot_ptr->is_running_motion    = false;  //机械臂运动已结束，可以执行其他离线类运动
        // on_stop_trajectory              = false;
    }

#pragma endregion

}  // namespace JC_helper