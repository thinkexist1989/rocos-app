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

#ifndef ROCOS_APP_ROBOT_SERVICE_H
#define ROCOS_APP_ROBOT_SERVICE_H

#include <boost/smart_ptr.hpp>

#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>

#include <cmake-build-release-remote-host-advantech/robot_service.grpc.pb.h>
#include "robot_service.grpc.pb.h"

namespace rocos {
    class Robot;

    class RobotServiceImpl final : public RobotService::Service {
    public:
        RobotServiceImpl(boost::shared_ptr<Robot> robot);

        grpc::Status ReadRobotState(::grpc::ServerContext *context, const ::rocos::RobotStateRequest *request,
                                    ::rocos::RobotStateResponse *response) override;

        grpc::Status WriteRobotCommmand(::grpc::ServerContext *context, const ::rocos::RobotCommandRequest *request,
                                        ::rocos::RobotCommandResponse *response) override;

    private:
        boost::shared_ptr<Robot> _robot {nullptr};
    };

}

#endif //ROCOS_APP_ROBOT_SERVICE_H
