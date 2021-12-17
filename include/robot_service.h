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
#include <boost/thread.hpp>

#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>

//#include <cmake-build-release-remote-host-advantech/robot_service.grpc.pb.h>
#include "robot_service.grpc.pb.h"

namespace rocos {
    class Robot;

    class RobotServiceImpl final : public RobotService::Service {
    private:
        RobotServiceImpl(boost::shared_ptr<Robot> robot);
        ~RobotServiceImpl();

    public:
        static boost::shared_ptr<RobotServiceImpl> getInstance(boost::shared_ptr<Robot> robot);

        grpc::Status ReadRobotState(::grpc::ServerContext *context, const ::rocos::RobotStateRequest *request,
                                    ::rocos::RobotStateResponse *response) override;

        grpc::Status WriteRobotCommmand(::grpc::ServerContext *context, const ::rocos::RobotCommandRequest *request,
                                        ::rocos::RobotCommandResponse *response) override;

        void runServer(const std::string& address = "0.0.0.0:30001");

        void stopServer();

    private:
        void serverThread(const std::string& address);

    private:
        static boost::shared_ptr<RobotServiceImpl> _instance;
        boost::shared_ptr<Robot> _robotPtr {nullptr};

        std::unique_ptr<grpc::Server> _server {nullptr};

        boost::shared_ptr<boost::thread> _thread {nullptr};

        bool _isThreadRunning {false};

    };

}

#endif //ROCOS_APP_ROBOT_SERVICE_H
