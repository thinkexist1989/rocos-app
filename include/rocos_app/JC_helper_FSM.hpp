#ifndef JC_HELPER_FSM_H
#define JC_HELPER_FSM_H

#include "JC_helper_kinematics.hpp"
#include "highspeed_udp.hpp"
#include "interpolate.h"
#include "kdl/frames.hpp"
#include "tinyfsm.hpp"
#include <Eigen/Geometry>
#include <atomic>
#include <fstream>
#include <iostream>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <mutex>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Log.h>
#include <ruckig/ruckig.hpp>
#include <trac_ik/trac_ik.hpp>
#include <vector>

namespace rocos
{
    class Robot;

    struct EVEN_BASE : tinyfsm::Event
    {
        rocos::Robot* robot_prt;
    };

    struct EVEN_MOVEL : EVEN_BASE
    {
        rocos::cart_info target_info{ };
    };
    struct EVEN_MOVEJ : EVEN_BASE
    {
        rocos::joint_info target_info{ };
    };
    struct EVEN_IDLE : EVEN_BASE
    {
    };
    struct EVEN_PAUSEL : EVEN_BASE
    {
        rocos::cart_info target_info{ };
    };
    struct EVEN_PAUSEJ : EVEN_BASE
    {
        rocos::joint_info target_info{ };
    };
    struct EVEN_STOPL : EVEN_BASE
    {
    };
    struct EVEN_STOPJ : EVEN_BASE
    {
        KDL::JntArray current_pos;
        KDL::JntArray last_pos;
        KDL::JntArray last_last_pos;
    };

    struct STATE_BASE : tinyfsm::Fsm<rocos::STATE_BASE>
    {
        void react( tinyfsm::Event const& ){ };
        virtual void entry( void ){ }; /* entry actions in some states */
        virtual void exit( void ){ };  /* no exit actions */
    };

    struct STATE_MOVEJ : STATE_BASE
    {
        void react( EVEN_STOPJ const& even );
        void react( EVEN_MOVEJ const& even )
        {
            PLOG_ERROR << "拒绝 EVEN_MOVEJ切换到STATE_MOVEJ";
            throw -1;
        };
        void entry( void ) override { PLOG_INFO << "进入STATE_MOVEJ"; }; /* entry actions in some states */
        void exit( void ) override { PLOG_INFO << "退出STATE_MOVEJ"; };  /* no exit actions */
    };

    struct STATE_STOPJ : STATE_BASE
    {
        void react( EVEN_MOVEJ const& even )
        {
            PLOG_ERROR << "拒绝 STATE_STOPJ切换到STATE_MOVEJ";
            throw -1;
        };

        void entry( void ) override{ }; /* entry actions in some states */
        void exit( void ) override{ };  /* no exit actions */
    };

    struct STATE_IDLE : STATE_BASE
    {
        void react( EVEN_MOVEJ const& even )
        {
            PLOG_INFO << " STATE_IDLE切换到STATE_MOVEJ";
            transit< STATE_MOVEJ >( );
        };
        void entry( void ) override
        {
            PLOG_INFO << "进入STATE_IDLE";
        };                                                             /* entry actions in some states */
        void exit( void ) override { PLOG_INFO << "退出STATE_IDLE"; }; /* no exit actions */
    };

    using fsm_handle = STATE_BASE;

}  // namespace rocos

#endif