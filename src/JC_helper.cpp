#include "JC_helper.hpp"

namespace JC_helper
{
    std::vector< double > UnitQuaternion_intep( const std::vector< double >& start,
                                                const std::vector< double >& end,
                                                double s )
    {
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
        if ( theta == 0 || s == 0 )
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
        if ( end == start )
            return -1;

        double T_link = 0;
        rocos::DoubleS doubleS_P;
        rocos::DoubleS doubleS_R;
        double path_length = ( end.p - start.p ).Norm( );

        doubleS_P.planDoubleSProfile( 0, 0, 1, v_start / path_length, v_end / path_length, max_path_v / path_length, max_path_a / path_length, max_path_a * 2 / path_length );
        bool isplanned = doubleS_P.isValidMovement( );
        if ( !isplanned || !( doubleS_P.getDuration( ) > 0 ) )
        {
            std::cout << RED << "link_trajectory():Error_MotionPlanning_Not_Feasible" << WHITE << std::endl;
            ;
            return -1;
        }

        doubleS_R.planDoubleSProfile( 0, 0, 1, 0, 0, max_path_v / path_length, max_path_a / path_length, max_path_a * 2 / path_length );
        isplanned = doubleS_R.isValidMovement( );
        if ( !isplanned || !( doubleS_R.getDuration( ) > 0 ) )
        {
            std::cout << RED << "link_trajectory():Error_MotionPlanning_Not_Feasible" << WHITE << std::endl;
            ;
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

    int multilink_trajectory( std::vector< KDL::Frame >& traj, const KDL::Frame& f_start, const KDL::Frame& f_mid, const KDL::Frame& f_end, KDL::Frame& next_f_start, double current_path_start_v, double& next_path_start_v, double bound_dist, double max_path_v, double max_path_a, double next_max_path_v )
    {
        using namespace KDL;
        //** 变量初始化 **//
        double eps = 1E-7;
        Vector p1{ f_start.p };
        Vector p_mid{ f_mid.p };
        Vector p2{ f_end.p };

        Vector ab = p_mid - p1;
        Vector bc = p2 - p_mid;

        double abdist = ab.Norm( );
        double bcdist = bc.Norm( );
        //**-------------------------------**//

        //** 数据有效性检查 **//
        if ( abdist < eps )
        {
            std::cout << RED << "multi_link_trajectory()：f_start and  f_mid are too close " << GREEN << std::endl;
            return -1;
        }

        if ( bcdist < eps )
        {
            std::cout << RED << "multi_link_trajectory():f_mid and  f_end are too close " << GREEN << std::endl;
            return -1;
        }

        if ( bound_dist >= abdist )
        {
            std::cout << RED << "bound_dist is too  large，try to decrease it" << GREEN << std::endl;
            return -1;
        }

        if ( bound_dist >= bcdist )
        {
            std::cout << RED << "multi_link_trajectory()：bound_dist is too  large，try to decrease it" << GREEN << std::endl;
        }

        if ( current_path_start_v > max_path_v )  //! 防止起始速度就超过最大可达线速度 ,理论上不可能发生
        {
            std::cout << RED << " current_path_start_v  > max_path_v" << std::endl;
            std::cout << "current_path_start_v = " << current_path_start_v << std::endl;
            std::cout << " max_path_v = " << max_path_v << GREEN << std::endl;
            assert( 0 );
        }

        //**-------------------------------**//

        //求解过渡半径占总长的百分比
        double s_bound_dist_1 = bound_dist / abdist;
        double s_bound_dist_2 = bound_dist / bcdist;

        std::cout << "s_bound_dist_1 = " << s_bound_dist_1 << std::endl;
        std::cout << "s_bound_dist_2= " << s_bound_dist_2 << std::endl;

        //利用向量乘积公式，求得两向量的夹角 ,并且限定范围在0-180
        double alpha  = acos( std::max( -1., std::min( dot( ab, bc ) / abdist / bcdist, 1. ) ) );
        double radius = bound_dist * tan( ( M_PI - alpha ) / 2 );

        Frame F_base_circlestart = link_trajectory( f_start, f_mid, 1 - s_bound_dist_1, 1 - s_bound_dist_1 );
        Frame F_base_circleend   = link_trajectory( f_mid, f_end, s_bound_dist_2, s_bound_dist_2 );

        Vector de     = F_base_circlestart.p - f_start.p;  //实际要走的直线段的距离
        double dedist = de.Norm( );                        //实际要走的直线段的距离

        if ( current_path_start_v <= eps && ( 1 - s_bound_dist_1 ) <= eps )
        {
            std::cout << RED << " multi_link_trajectory()：the Link length is not allowed to be equal to 0,When Velocity of last motion is equal to 0" << GREEN << std::endl;
            return -1;
        }

        Vector V_base_t = ab * ( ab * bc );  //圆弧中垂直于第一段直线的半径向量
        V_base_t.Normalize( );

        Frame F_base_circleCenter{ F_base_circlestart.p - V_base_t * radius };

        double s_cirlular_v{ 0 };

        if ( s_bound_dist_1 <= eps )
            s_cirlular_v = 0;
        else if ( ( 1 - s_bound_dist_1 ) <= eps )
            s_cirlular_v = current_path_start_v / ( radius * alpha );
        else
            s_cirlular_v = s_bound_dist_1 * std::min( max_path_v, next_max_path_v ) / ( radius * alpha );  //考虑当前和下次的运动，选取最小值（代表当前运动和下一段运动的约束下，最大可行速度）

#pragma region  // 第一段直线速度轨迹规划

        double T_link = 0;
        rocos::DoubleS doubleS_1_P;
        rocos::DoubleS doubleS_1_R;
        //存在直线，才需要规划
        if ( ( 1 - s_bound_dist_1 ) > eps )
        {
            doubleS_1_P.planDoubleSProfile( 0, 0, 1, current_path_start_v / dedist, s_cirlular_v, max_path_v / dedist, max_path_a / dedist, max_path_a * 2 / dedist );
            bool isplanned = doubleS_1_P.isValidMovement( );
            if ( !isplanned || !( doubleS_1_P.getDuration( ) > 0 ) )
            {
                std::cout << RED << "multi_link_trajectory()：Linear trajectory planning fails,try to decrease given parameter [max_path_v] " << GREEN << std::endl;
                return -1;
            }

            doubleS_1_R.planDoubleSProfile( 0, 0, 1, 0, 0, max_path_v / dedist, max_path_a / dedist, max_path_a * 2 / dedist );
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
        rocos::DoubleS doubleS_2_R;
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
        std::cout << "T_total_1 = " << ( T_link ) << std::endl;
        std::cout << "T_cirlular= " << ( T_cirlular ) << std::endl;
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

}  // namespace JC_helper