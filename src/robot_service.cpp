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

    RobotServiceImpl::RobotServiceImpl(Robot *robot) : _robotPtr(robot) {

    }

    RobotServiceImpl::~RobotServiceImpl() {
        _server->Shutdown();
    }

    boost::shared_ptr<RobotServiceImpl> RobotServiceImpl::getInstance(Robot *robot) {
        if (_instance == nullptr) {
            _instance.reset(new RobotServiceImpl(robot),
                            [](RobotServiceImpl *t) { delete t; }); // 因为默认访问不了private 析构函数,需传入删除器
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
        for (int i = 0; i < _robotPtr->getJointNum(); i++) {
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
        if (request->command().has_enabled()) {
            _robotPtr->setEnabled();
        } else if (request->command().has_disabled()) {
            _robotPtr->setDisabled();
        } else if (request->command().has_move_j()) {

        }
            /////////// Single Axis Command //////////////
        else if (request->command().has_single_axis_command()) {
            auto singleAxisCmd = request->command().single_axis_command();

            if (singleAxisCmd.has_enabled()) {                 ///////// enabled
                _robotPtr->setJointEnabled(singleAxisCmd.enabled().id());
            } else if (singleAxisCmd.has_disabled()) {        ///////// disabled
                _robotPtr->setJointDisabled(singleAxisCmd.disabled().id());
            } else if (singleAxisCmd.has_mode()) {           /////////// mode
                ModeOfOperation modeOfOperation;
                switch (singleAxisCmd.mode().value()) {
                    case MODE_CSP:
                        modeOfOperation = ModeOfOperation::CyclicSynchronousPositionMode;
                        break;
                    case MODE_CSV:
                        modeOfOperation = ModeOfOperation::CyclicSynchronousVelocityMode;
                        break;
                    case MODE_CST:
                        std::cout << "CST is not implemented!" << std::endl;
//                        modeOfOperation = ModeOfOperation::CyclicSynchronousTorqueMode;
                        break;
                }
                _robotPtr->setJointMode(singleAxisCmd.mode().id(), modeOfOperation);
            } else if (singleAxisCmd.has_move()) {         /////////// move
                double max_vel = -1, max_acc = -1, max_jerk = -1, least_time = -1;
                if (singleAxisCmd.move().has_max_vel())
                    max_vel = singleAxisCmd.move().max_vel();
                if (singleAxisCmd.move().has_max_acc())
                    max_acc = singleAxisCmd.move().max_acc();
                if (singleAxisCmd.move().has_max_jerk())
                    max_jerk = singleAxisCmd.move().max_jerk();
                if (singleAxisCmd.move().has_least_time())
                    least_time = singleAxisCmd.move().least_time();

//                std::cout << "max_vel: " << max_vel << "; max_acc: " << max_acc << "; max_jerk: " << max_jerk << std::endl;

                _robotPtr->moveSingleAxis(singleAxisCmd.move().id(), singleAxisCmd.move().pos(), 0.0, max_vel, max_acc,
                                          max_jerk, least_time);

            }
        }
            /////////// Multi Axis Command //////////////
        else if (request->command().has_multi_axis_command()) {
            auto multiAxisCmd = request->command().multi_axis_command();
            if (multiAxisCmd.has_enabled()) {
                _robotPtr->setEnabled();
            } else if (multiAxisCmd.has_disabled()) {
                _robotPtr->setDisabled();
            } else if (multiAxisCmd.has_mode()) {
                for(int i = 0; i < _robotPtr->_jntNum; i++) {
                    ModeOfOperation modeOfOperation;
                    switch (multiAxisCmd.mode().value().at(i)) {
                        case MODE_CSP:
                            modeOfOperation = ModeOfOperation::CyclicSynchronousPositionMode;
                            break;
                        case MODE_CSV:
                            modeOfOperation = ModeOfOperation::CyclicSynchronousVelocityMode;
                            break;
                        case MODE_CST:
                            std::cout << "CST is not implemented!" << std::endl;
//                        modeOfOperation = ModeOfOperation::CyclicSynchronousTorqueMode;
                            break;
                    }
                    _robotPtr->setJointMode(i, modeOfOperation);
                }
            }else if(multiAxisCmd.has_sync()) {
                _robotPtr->setSynchronization(static_cast<Robot::Synchronization>(multiAxisCmd.sync().value()));
            }
            else if (multiAxisCmd.has_move()) {
                for(int i = 0; i < _robotPtr->_jntNum; i++) {

                    _robotPtr->moveSingleAxis(i,
                                              multiAxisCmd.move().target_pos().at(i),
                                              0.0,
                                              multiAxisCmd.move().max_vel().at(i),
                                              multiAxisCmd.move().max_acc().at(i),
                                              multiAxisCmd.move().max_jerk().at(i),
                                              -1);

                }

            }
//            _robotPtr->setJointDisabled(request->command().single_axis_disabled().id());
        }

        return grpc::Status::OK;
    }

    void RobotServiceImpl::runServer(const std::string &address, bool isDetached) {
        _thread = boost::make_shared<boost::thread>(boost::bind(&RobotServiceImpl::serverThread, this, address));
        if (isDetached) {
            _thread->detach();
        } else {
            _thread->join();
        }

    }

    void RobotServiceImpl::stopServer() {
        _server->Shutdown();
    }

    void RobotServiceImpl::serverThread(const std::string &address) {
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