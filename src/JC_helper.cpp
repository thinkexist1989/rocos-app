#include "JC_helper.hpp"
#include "robot.h"

namespace JC_helper
{
    std::vector< double > UnitQuaternion_intep( const std::vector< double >& start,
                                                const std::vector< double >& end,
                                                double s )
    {
        constexpr double eps = 1E-7;
        

        if ( s > 1 || s < 0 )
        {
            std::cerr << RED << "values of S outside interval [0,1]" << GREEN << std::endl;
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
        if ( abs(theta)<eps   || s == 0 )
            return start_2;
        else
        {
            double coefficient_1 = sin( ( 1 - s ) * theta ) / sin( theta );
            double coefficient_2 = sin( ( s )*theta ) / sin( theta );

            return std::vector< double >{
                coefficient_1 * start_2[ 0 ] + coefficient_2 * end[ 0 ],
                coefficient_1 * start_2[ 1 ] + coefficient_2 * end[ 1 ],
                coefficient_1 * start_2[ 2 ] + coefficient_2 * end[ 2 ],
                coefficient_1 * start_2[ 3 ] + coefficient_2 * end[ 3 ] };
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

    KDL::Frame cirlular_trajectory( const KDL::Frame& F_base_circlestart, const KDL::Frame& F_base_circleend, const KDL::Frame& F_base_circleCenter, double s_p, double s_r, double alpha )
    {
        using namespace KDL;

        assert( !( s_p > 1 || s_p < 0 ) && "values of s_p outside interval [0,1]" );  //*! 只要s超出范围就退出
        assert( !( s_r > 1 || s_r < 0 ) && "values of s_r outside interval [0,1]" );  //*! 只要s超出范围就退出

        KDL::Vector x    = F_base_circlestart.p - F_base_circleCenter.p;
        double radius    = x.Normalize( );
        KDL::Vector tmpv = F_base_circleend.p - F_base_circleCenter.p;  //第二直线段上的半径段
        tmpv.Normalize( );

        Vector z( x * tmpv );  //Z轴
        double n = z.Normalize( );

        if ( n < epsilon )
        {
            std::cerr << RED << "cirlular_trajectory(): Z Axis Calculation error " << GREEN << std::endl;
            assert( 0 );
        }

        KDL::Frame F_base_circleCenter_( KDL::Rotation{ x, ( z * x ), z }, F_base_circleCenter.p );
        std::vector< double > Quaternion_start{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_end{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_interp{ 0, 0, 0, 0 };
        F_base_circlestart.M.GetQuaternion( Quaternion_start.at( 0 ), Quaternion_start.at( 1 ), Quaternion_start.at( 2 ), Quaternion_start.at( 3 ) );
        F_base_circleend.M.GetQuaternion( Quaternion_end.at( 0 ), Quaternion_end.at( 1 ), Quaternion_end.at( 2 ), Quaternion_end.at( 3 ) );
        Quaternion_interp = UnitQuaternion_intep( Quaternion_start, Quaternion_end, s_r );

        return KDL::Frame( KDL::Rotation::Quaternion( Quaternion_interp[ 0 ], Quaternion_interp[ 1 ], Quaternion_interp[ 2 ], Quaternion_interp[ 3 ] ), F_base_circleCenter_ * Vector{ radius * cos( s_p * alpha ), radius * sin( s_p * alpha ), 0 } );
    }

    KDL::Frame link_trajectory( const KDL::Frame& start, const KDL::Frame& end, double s_p, double s_r )
    {
        assert( !( s_p > 1 || s_p < 0 ) && "values of s_p outside interval [0,1]" );  //*! 只要s超出范围就退出
        assert( !( s_r > 1 || s_r < 0 ) && "values of s_r outside interval [0,1]" );  //*! 只要s超出范围就退出

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

    int link_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& start, const KDL::Frame& end, double v_start, double v_end, double max_path_v, double max_path_a )
    {
        if ( end == start )  //起始和终止位置一致，无需规划
            return 0;
        else
        {
            double T_link = 0;
            ::rocos::DoubleS doubleS_P;
            ::rocos::DoubleS doubleS_R;
            double path_length = ( end.p - start.p ).Norm( );
            //只旋转，不移地情况，要求v_start和v_end必需为0
            if ( path_length == 0 )
            {
                if ( v_start != 0 || v_end != 0 )
                {
                    std::cout << RED << "link_trajectory(): v_start OR v_end must be zero" << GREEN << std::endl;
                    return -1;
                }
                KDL::Rotation R_start_end = start.M.Inverse( ) * end.M;
                KDL::Vector ration_axis;
                double angle                   = R_start_end.GetRotAngle( ration_axis );
                const double equivalent_radius = 0.1;
                double Rlength                 = ( equivalent_radius * abs( angle ) );
                path_length                    = Rlength;
            }

            doubleS_P.planDoubleSProfile( 0, 0, 1, v_start / path_length, v_end / path_length, max_path_v / path_length, max_path_a / path_length, max_path_a * 2 / path_length );
            bool isplanned = doubleS_P.isValidMovement( );
            if ( !isplanned || !( doubleS_P.getDuration( ) > 0 ) )
            {
                std::cout << RED << "link_trajectory():Error_MotionPlanning_Not_Feasible" << WHITE << std::endl;
                return -1;
            }

            doubleS_R.planDoubleSProfile( 0, 0, 1, 0, 0, max_path_v / path_length, max_path_a / path_length, max_path_a * 2 / path_length );
            isplanned = doubleS_R.isValidMovement( );
            if ( !isplanned || !( doubleS_R.getDuration( ) > 0 ) )
            {
                std::cout << RED << "link_trajectory():Error_MotionPlanning_Not_Feasible" << WHITE << std::endl;
                return -1;
            }

            if ( doubleS_R.getDuration( ) != doubleS_P.getDuration( ) )
            {
                doubleS_R.JC_scaleToDuration( doubleS_P.getDuration( ) );
            }

            T_link = doubleS_P.getDuration( );

            double t_total = 0;
            double s_p     = 0;
            double s_r     = 0;

            //** 轨迹计算 **//
            while ( t_total >= 0 && t_total <= T_link )
            {
                s_p = doubleS_P.pos( t_total );
                s_r = doubleS_R.pos( t_total );
                traj.push_back( link_trajectory( start, end, s_p, s_r ) );
                t_total = t_total + 0.001;
            }
            //**-------------------------------**//
            return 0;
        }
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

        if ( bound_dist > bcdist )  //这个条件说明：【正常情况下】如果本段只旋转，不移地，则上段bound_dist必需=0，则本段开始速度一定为0
        {
            std::cout << RED << "multi_link_trajectory():bound_dist is too  large，try to decrease it" << GREEN << std::endl;
        }

        if ( current_path_start_v > max_path_v )  //! 防止起始速度就超过最大可达线速度 ,理论上不可能发生
        {
            std::cout << RED << " current_path_start_v  > max_path_v" << std::endl;
            std::cout << "current_path_start_v = " << current_path_start_v << std::endl;
            std::cout << " max_path_v = " << max_path_v << GREEN << std::endl;
            assert( 0 );
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

        Frame F_base_circlestart = link_trajectory( f_start, f_mid, 1 - s_bound_dist_1, 1 - s_bound_dist_1 );
        Frame F_base_circleend   = link_trajectory( f_mid, f_end, s_bound_dist_2, s_bound_dist_2 );

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
            //dedist==0,代表只旋转，不移动的情况
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
                traj.push_back( link_trajectory( f_start, F_base_circlestart, s_p, s_r ) );
            }
            else
            {
                s_p = ( t_total - T_link ) / T_cirlular;
                s_r = doubleS_2_R.pos( t_total - T_link );
                traj.push_back( cirlular_trajectory( F_base_circlestart, F_base_circleend, F_base_circleCenter, s_p, s_r, alpha ) );
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
        //f_p2 = [dot(v1,axis_x),0] = [bx,0]
        //f_P3 = [dot(v2,axis_x),dot(v2,axis_y)=[cx,cy]
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

    smart_servo::smart_servo( std::vector< KDL::JntArray >* traj_ptr, std::atomic< bool >* finished_flag_ptr )
    {
        external_traj_ptr          = traj_ptr;
        external_finished_flag_ptr = finished_flag_ptr;
    }

    void smart_servo::init( std::vector< double > q_init, std::vector< double > v_init, std::vector< double > a_init, double max_v, double max_a, double max_j )
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

    void smart_servo::init( KDL::Frame p_init, double v_init, double a_init, double max_v, double max_a, double max_j )
    {
        _Cartesian_state._p_init = p_init;
        _Cartesian_state._v_init = v_init;
        _Cartesian_state._a_init = a_init;
        _Cartesian_state._max_v  = max_v;
        _Cartesian_state._max_a  = max_a;
        _Cartesian_state._max_j  = max_j;

        PLOG_INFO << "smart servo init succesed";
    }

    void smart_servo::smart_servo_using_Joint( rocos::Robot* robot_ptr )
    {
        std::unique_lock< std::mutex > input_lock( input_mutex, std::defer_lock );
        ruckig::Result res;
        int count{ 0 };
        int _tick_count{ robot_ptr->tick_count };

        while ( *external_finished_flag_ptr ) PLOG_INFO << "----waiting for command----";

        while ( 1 )
        {
            input_lock.lock( );
            res = otg.update( input, output );
            input_lock.unlock( );

            if ( res == ruckig::Result::Finished )
            {
                ( *external_finished_flag_ptr ) = true;   //这次smart servo已结束，等待下一次smart servo
                robot_ptr->is_running_motion    = false;  //机械臂运动已结束，可以执行其他离线类运动
                on_stop_trajectory = false; //这个必须设为false,因为新线程仍然使用同一个对象数据成员
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
                    }
                    input_lock.unlock( );
                }
            }
            //**-------------------------------**//
        }
    }

    //TODO 笛卡尔空间下smart servo
    void smart_servo::smart_servo_using_Cartesian( )
    {
        PLOG_ERROR << " have not  completed yet" << std::endl;
    }

    //TODO 笛卡尔空间下smart servo,注意奇异位置速度激增问题
    void smart_servo::smart_servo_IK( rocos::Robot* )
    {
        PLOG_ERROR << " have not  completed yet";
    }
   
    //TODO 笛卡尔空间下smart servo
    void smart_servo::smart_servo_motion( rocos::Robot* robot_ptr )
    {
        //** 变量初始化 **//
        std::unique_lock< std::mutex > traj_lock( traj_mutex, std::defer_lock );
        int count{ 0 };
        int index{ 0 };
        KDL::JntArray joint_command;
        int _tick_count{ robot_ptr->tick_count };
        //**-------------------------------**//
        //** 等待command 开始 **//
        while ( *external_finished_flag_ptr ) PLOG_INFO << "----waiting for command----";
        //**-------------------------------**//

        while ( 1 )
        {
            traj_lock.lock( );
            //关节轨迹有余
            if ( index < ( *external_traj_ptr ).size( ) )
            {
                joint_command = ( *external_traj_ptr )[ index ];
                traj_lock.unlock( );

                for ( int i = 0; i < _joint_num; ++i )
                {
                    robot_ptr->pos_[ i ] = joint_command( i );
                    robot_ptr->joints_[ i ]->setPosition( joint_command( i ) );
                }

                robot_ptr->hw_interface_->waitForSignal( 0 );
            }
            else  //指向最后一个
            {
                traj_lock.unlock( );
                if ( _runnig_flag._FinishCartIK )
                    break;  //全部关节轨迹已经取出
                else
                    continue;  //还没规划完，本次跳过
            }

            index++;

            //** 100ms进行一次心跳检查,紧急停止时不需要检查 **//
            if ( ( ++count == 100 ) && !on_stop_trajectory )
            {
                count = 0;

                if ( _tick_count != robot_ptr->tick_count )
                    _tick_count = robot_ptr->tick_count;
                else
                {
                    PLOG_ERROR << "RunDragging():Some errors such as disconnecting from the controller";
                    on_stop_trajectory = true;
                    {
                        //TODO 笛卡尔下紧急停止
                    }
                }
            }
            //**-------------------------------**//
        }

        ( *external_finished_flag_ptr ) = true;   //这次smart servo已结束，等待下一次smart servo
        robot_ptr->is_running_motion    = false;  //机械臂运动已结束，可以执行其他离线类运动
    }

    void smart_servo::command( KDL::JntArray q_target )
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

            ( *external_finished_flag_ptr ) = false;  //如果第一次command，则会同时启动{规划}和{运动}线程
        }
        else
        {
            PLOG_DEBUG<<"control is not allowed during emergency stop";
        }
    }

    //TODO 笛卡尔空间下smart servo
    void smart_servo::command( KDL::Frame p_target )
    {
        PLOG_ERROR << " have not  completed yet";
    }

}  // namespace JC_helper