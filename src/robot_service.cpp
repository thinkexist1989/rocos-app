// Copyright 2021, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn

#include "robot_service.h"

#include <robot.h>

#include <google/protobuf/util/time_util.h>

namespace rocos {
    using namespace google::protobuf::util; //使用命名空间

    RobotServiceImpl::RobotServiceImpl(boost::shared_ptr<Robot> robot) : _robot(robot) {

    }

    grpc::Status
    RobotServiceImpl::ReadRobotState(::grpc::ServerContext *context, const ::rocos::RobotStateRequest *request,
                                     ::rocos::RobotStateResponse *response) {

        *response->mutable_header()->mutable_response_timestamp() = TimeUtil::GetCurrentTime(); // response timestamp
        for(int i = 0; i < _robot->getJointNum(); i++) {
            JointState jointState;
            jointState.set_status(static_cast<JointState_Status>(_robot->getJointStatus(i)));
            jointState.set_position(_robot->getJointPosition(i));
            jointState.set_velocity(_robot->getJointVelocity(i));
            jointState.set_acceleration(_robot->getJointTorque(i));
            jointState.set_load(_robot->getJointLoadTorque(i));

            *response->mutable_robot_state()->add_joint_states() = jointState;
        }

//        *response->mutable_robot_state()->add_joint_states()
        return grpc::Status::OK;
    }

    grpc::Status
    RobotServiceImpl::WriteRobotCommmand(::grpc::ServerContext *context, const ::rocos::RobotCommandRequest *request,
                                         ::rocos::RobotCommandResponse *response) {

        return grpc::Status::OK;
    }

}