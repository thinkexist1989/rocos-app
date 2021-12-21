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

#include <utility>

namespace rocos {
    using namespace google::protobuf::util; //使用命名空间

    RobotServiceImpl::RobotServiceImpl(Robot* robot) : _robotPtr(robot) {

    }

    RobotServiceImpl::~RobotServiceImpl() {

    }

    boost::shared_ptr<RobotServiceImpl> RobotServiceImpl::getInstance(Robot* robot) {
        if (_instance == nullptr) {
            _instance.reset(new RobotServiceImpl(robot), [](RobotServiceImpl *t) { delete t; }); // 因为默认访问不了private 析构函数,需传入删除器
        }
        return _instance;
    }

    grpc::Status
    RobotServiceImpl::ReadRobotState(::grpc::ServerContext *context, const ::rocos::RobotStateRequest *request,
                                     ::rocos::RobotStateResponse *response) {
        //ResponseHeader
        *response->mutable_header()->mutable_response_timestamp() = TimeUtil::GetCurrentTime(); // response timestamp

        auto robotState = response->mutable_robot_state();
        // JointState
        for(int i = 0; i < _robotPtr->getJointNum(); i++) {
            JointState jointState;
            jointState.set_name(_robotPtr->getJointName(i));
            jointState.set_status(static_cast<JointState_Status>(_robotPtr->getJointStatus(i)));
            jointState.set_position(_robotPtr->getJointPosition(i));
            jointState.set_velocity(_robotPtr->getJointVelocity(i));
            jointState.set_acceleration(_robotPtr->getJointTorque(i));
            jointState.set_load(_robotPtr->getJointLoadTorque(i));

            *robotState->add_joint_states() = jointState;
        }

        // Hardware State
        auto hw_state = response->mutable_robot_state()->mutable_hw_state();
        hw_state->set_hw_type(
                static_cast<HardwareState_HardwareType>(_robotPtr->_hw_interface->getHardwareType()));
        hw_state->set_min_cycle_time(_robotPtr->_hw_interface->getMinCycleTime());
        hw_state->set_max_cycle_time(_robotPtr->_hw_interface->getMaxCycleTime());
        hw_state->set_current_cycle_time(_robotPtr->_hw_interface->getCurrCycleTime());
        hw_state->set_slave_num(_robotPtr->_hw_interface->getSlaveNumber());


        return grpc::Status::OK;
    }

    grpc::Status
    RobotServiceImpl::WriteRobotCommmand(::grpc::ServerContext *context, const ::rocos::RobotCommandRequest *request,
                                         ::rocos::RobotCommandResponse *response) {
        //ResponseHeader
        *response->mutable_header()->mutable_response_timestamp() = TimeUtil::GetCurrentTime(); // response timestamp

        //Process Request RobotCommand
        if(request->command().has_enabled()) {
            _robotPtr->setEnabled();
        }
        else if(request->command().has_disabled()) {
            _robotPtr->setDisabled();
        }

        return grpc::Status::OK;
    }

    void RobotServiceImpl::runServer(const std::string& address, bool isDetached) {
        _thread = boost::make_shared<boost::thread>(boost::bind(&RobotServiceImpl::serverThread, this, address));
        if(isDetached) {
            _thread->detach();
        }
        else{
            _thread->join();
        }

    }

    void RobotServiceImpl::stopServer() {
        _server->Shutdown();
    }

    void RobotServiceImpl::serverThread(const std::string& address) {
        _isThreadRunning = true;

        grpc::EnableDefaultHealthCheckService(true);
        grpc::reflection::InitProtoReflectionServerBuilderPlugin();
        grpc::ServerBuilder builder;

        // Listen on the given address without any authentication mechanism.
        builder.AddListeningPort(address, grpc::InsecureServerCredentials());

        // Register "service" as the instance through which we'll communicate with
        // clients. In this case it corresponds to an *synchronous* service.
        builder.RegisterService(this);

        // Finally assemble the server.
        _server = builder.BuildAndStart();
        std::cout << "Server listening on " << address << std::endl;

        // Wait for the server to shutdown. Note that some other thread must be
        // responsible for shutting down the server for this call to ever return.

        _server->Wait();

        _isThreadRunning = false;
    }

    boost::shared_ptr<RobotServiceImpl> RobotServiceImpl::_instance = nullptr; // 单例模式对象

}