
#include <rocos_app/JC_helper_FSM.hpp>
#include <rocos_app/robot.h>
namespace rocos
{

    void STATE_MOVEJ::react( EVEN_STOPJ const& even )
    {
        std::unique_lock< std::mutex > motor_control( even.robot_prt->mtx_motor );  // 上锁

        JC_helper::Joint_stop( even.robot_prt, even.current_pos, even.last_pos, even.last_last_pos );

        even.robot_prt->setRunState( Robot::RunState::Stopped );

        transit< STATE_IDLE >( );
    }

}  // namespace rocos



FSM_INITIAL_STATE( rocos::STATE_BASE, rocos::STATE_IDLE );
