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
        std::vector< double > Quaternion_start{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_end{ 0, 0, 0, 0 };
        std::vector< double > Quaternion_interp{ 0, 0, 0, 0 };
        start.M.GetQuaternion( Quaternion_start.at( 0 ), Quaternion_start.at( 1 ),
                                 Quaternion_start.at( 2 ), Quaternion_start.at( 3 ) );
        end.M.GetQuaternion( Quaternion_end.at( 0 ), Quaternion_end.at( 1 ),
                              Quaternion_end.at( 2 ), Quaternion_end.at( 3 ) );
        KDL::Frame interp_frame{ };
        //**-------------------------------**//

        //** 轨迹计算 **//
        while ( dt <= duration )
        {
            s                 = doubleS.pos( dt );
            if ( s < 0 || s > 1)
            {
                PLOG_ERROR << " link calculating failure";
                return -1;
            }
            KDL::Vector P     = Pstart + ( Pend - Pstart ) * s;
            Quaternion_interp = JC_helper::UnitQuaternion_intep( Quaternion_start, Quaternion_end, s );
            interp_frame.M    = KDL::Rotation::Quaternion( Quaternion_interp[ 0 ], Quaternion_interp[ 1 ],
                                                           Quaternion_interp[ 2 ], Quaternion_interp[ 3 ] );
            interp_frame.p    = P;
       
            traj.push_back( interp_frame );
            dt += 0.001;
        }
        //**-------------------------------**//
        return 0;
    }