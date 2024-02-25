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

#include <rocos_app/robot_service.h>

#include <google/protobuf/util/time_util.h>
#include <rocos_app/robot.h>

#include <utility>

#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>

#include <filesystem>
#include <fstream>

namespace rocos {
    using namespace google::protobuf::util;  //使用命名空间

    RobotServiceImpl::RobotServiceImpl(Robot *robot) : robot_ptr_(robot) {}

    RobotServiceImpl::~RobotServiceImpl() { server_ptr_->Shutdown(); }

    boost::shared_ptr<RobotServiceImpl> RobotServiceImpl::getInstance(
            Robot *robot) {
        if (instance_ == nullptr) {
            instance_.reset(new RobotServiceImpl(robot), [](RobotServiceImpl *t) {
                delete t;
            });  // 因为默认访问不了private 析构函数,需传入删除器
        }
        return instance_;
    }


    grpc::Status
    RobotServiceImpl::GetRobotModel(::grpc::ServerContext *context, const ::google::protobuf::Empty *request,
                                    ::rocos::RobotModel *response) {

        urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDFFile(robot_ptr_->urdf_file_path_);
        if (!robot_model) {
            std::cerr << "Could not generate robot model" << std::endl;
            return grpc::Status::CANCELLED;
        }

        double roll, pitch, yaw;

        // 创建映射表
        std::map<int, std::string> enumMap;
        enumMap[urdf::Joint::FIXED] = "fixed";
        enumMap[urdf::Joint::REVOLUTE] = "revolute";
        enumMap[urdf::Joint::PRISMATIC] = "prismatic";
        enumMap[urdf::Joint::CONTINUOUS] = "continuous";

        std::cout << "robot: " << std::endl;
        response->set_name(robot_model->getName());

        std::vector<urdf::LinkSharedPtr> links;
        robot_model->getLinks(links);

        for (int i = 0; i < links.size(); i++) {

            Link link;

            std::cout << "  - name: " << links[i]->name << std::endl;
            link.set_name(links[i]->name);

            std::cout << "    order: " << i << std::endl;
            link.set_order(i);

            if (links[i]->parent_joint) {
                std::cout << "    type: " << enumMap[links[i]->parent_joint->type] << std::endl;
                link.set_type(static_cast<JointType>(links[i]->parent_joint->type));

                std::cout << "    translate: " << links[i]->parent_joint->parent_to_joint_origin_transform.position.x
                          << ", "
                          << links[i]->parent_joint->parent_to_joint_origin_transform.position.y << ", "
                          << links[i]->parent_joint->parent_to_joint_origin_transform.position.z << std::endl;
                link.mutable_translate()->set_x(links[i]->parent_joint->parent_to_joint_origin_transform.position.x);
                link.mutable_translate()->set_y(links[i]->parent_joint->parent_to_joint_origin_transform.position.y);
                link.mutable_translate()->set_z(links[i]->parent_joint->parent_to_joint_origin_transform.position.z);

                links[i]->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
                std::cout << "    rotate: " << roll << ", " << pitch << ", " << yaw << std::endl;
                link.mutable_rotate()->set_x(roll);
                link.mutable_rotate()->set_y(pitch);
                link.mutable_rotate()->set_z(yaw);

                std::cout << "    axis: " << links[i]->parent_joint->axis.x << ", "
                          << links[i]->parent_joint->axis.y << ", "
                          << links[i]->parent_joint->axis.z << std::endl;
                link.mutable_axis()->set_x(links[i]->parent_joint->axis.x);
                link.mutable_axis()->set_y(links[i]->parent_joint->axis.y);
                link.mutable_axis()->set_z(links[i]->parent_joint->axis.z);
            }

            if (links[i]->visual) {
                std::cout << "    translateLink: " << links[i]->visual->origin.position.x << ", "
                          << links[i]->visual->origin.position.y << ", "
                          << links[i]->visual->origin.position.z << std::endl;
                link.mutable_translatelink()->set_x(links[i]->visual->origin.position.x);
                link.mutable_translatelink()->set_y(links[i]->visual->origin.position.y);
                link.mutable_translatelink()->set_z(links[i]->visual->origin.position.z);

                links[i]->visual->origin.rotation.getRPY(roll, pitch, yaw);
                std::cout << "    rotateLink: " << roll << ", " << pitch << ", " << yaw << std::endl;
                link.mutable_rotatelink()->set_x(roll);
                link.mutable_rotatelink()->set_y(pitch);
                link.mutable_rotatelink()->set_z(yaw);

                std::cout << "    mesh: " << std::dynamic_pointer_cast<urdf::Mesh>(links[i]->visual->geometry)->filename
                          << std::endl;
                std::filesystem::path mesh_path = std::dynamic_pointer_cast<urdf::Mesh>(
                        links[i]->visual->geometry)->filename;
                link.set_mesh(mesh_path.filename());


            }


            *response->add_links() = link; // add link to response.links

        }


        return grpc::Status::OK;
    }

    grpc::Status RobotServiceImpl::GetLinkMesh(::grpc::ServerContext *context, const ::rocos::LinkMeshPath *request,
                                               ::rocos::LinkMeshFile *response) {

        std::cout << "request link file path: " << request->path() << std::endl;
        std::filesystem::path urdf_path(robot_ptr_->urdf_file_path_);
        std::filesystem::path file_path(urdf_path.parent_path() / request->path());

        std::ifstream file(file_path, std::ios::binary); // 以二进制方式打开文件
        if (!file.is_open()) {
            std::cout << "open file failed!" << std::endl;
            return grpc::Status::CANCELLED;
        }

        file.seekg(0, std::ios::end); // 定位到文件末尾
        std::streampos file_size = file.tellg(); // 获取文件大小

        file.seekg(0, std::ios::beg); // 定位到文件头
        std::vector<char> buffer(file_size); // 创建缓冲区

        file.read(buffer.data(), file_size); // 读取文件到缓冲区
        buffer.push_back('\0'); // 添加字符串结束符

        std::cout << "File Size: " << file_size / 1000.0 << " kB" << std::endl;

        response->set_name(file_path.filename().string()); // 设置response.name
        response->set_content(buffer.data(), buffer.size()); // 设置response.data

        return grpc::Status::OK;
    }

    grpc::Status RobotServiceImpl::ReadRobotInfo(
            ::grpc::ServerContext *context, const ::rocos::RobotInfoRequest *request,
            ::rocos::RobotInfoResponse *response) {
        // ResponseHeader
        *response->mutable_header()->mutable_response_timestamp() =
                TimeUtil::GetCurrentTime();  // response timestamp

        auto robotInfo = response->mutable_robot_info();
        // JointInfo
        for (int i = 0; i < robot_ptr_->getJointNum(); ++i) {
            JointInfo jointInfo;
            jointInfo.set_name(robot_ptr_->getJointName(i));
            jointInfo.set_cnt_per_unit(robot_ptr_->getJointCntPerUnit(i));
            jointInfo.set_torque_per_unit(robot_ptr_->getJointTorquePerUnit(i));
            jointInfo.set_ratio(robot_ptr_->getJointRatio(i));
            jointInfo.set_user_unit_name(robot_ptr_->getJointUserUnitName(i));
            jointInfo.set_pos_zero_offset(robot_ptr_->getJointPosZeroOffset(i));

            *robotInfo->add_joint_infos() = jointInfo;
        }

        return grpc::Status::OK;
    }

    grpc::Status RobotServiceImpl::ReadRobotState(
            ::grpc::ServerContext *context, const ::rocos::RobotStateRequest *request,
            ::rocos::RobotStateResponse *response) {
        // ResponseHeader
        *response->mutable_header()->mutable_response_timestamp() =
                TimeUtil::GetCurrentTime();  // response timestamp

        auto robotState = response->mutable_robot_state();
        if (request->has_raw_data() &&
            request->raw_data()) {  // 要求读取原始值 raw_data = true
            // JointState
            for (int i = 0; i < robot_ptr_->getJointNum(); ++i) {
                JointState jointState;
                jointState.set_name(robot_ptr_->getJointName(i));
                jointState.set_status(
                        static_cast<JointState_Status>(robot_ptr_->getJointStatus(i)));
                jointState.set_position(robot_ptr_->getJointPositionRaw(i));
                jointState.set_velocity(robot_ptr_->getJointVelocityRaw(i));
                jointState.set_acceleration(robot_ptr_->getJointTorqueRaw(i));
                jointState.set_load(robot_ptr_->getJointLoadTorque(i));
                jointState.set_raw_data(true);

                *robotState->add_joint_states() = jointState;
            }
        } else {  // 用户单位值 raw_data = false
            // JointState
            for (int i = 0; i < robot_ptr_->getJointNum(); ++i) {
                JointState jointState;
                jointState.set_name(robot_ptr_->getJointName(i));
                jointState.set_status(
                        static_cast<JointState_Status>(robot_ptr_->getJointStatus(i)));
                jointState.set_position(robot_ptr_->getJointPosition(i));
                jointState.set_velocity(robot_ptr_->getJointVelocity(i));
                jointState.set_acceleration(robot_ptr_->getJointTorque(i));
                jointState.set_load(robot_ptr_->getJointLoadTorque(i));

                *robotState->add_joint_states() = jointState;
            }
        }

        // Hardware State
        auto hw_state = response->mutable_robot_state()->mutable_hw_state();
        hw_state->set_hw_type(static_cast<HardwareState_HardwareType>(
                                      robot_ptr_->hw_interface_->getHardwareType()));
        hw_state->set_min_cycle_time(robot_ptr_->hw_interface_->getMinCycleTime());
        hw_state->set_max_cycle_time(robot_ptr_->hw_interface_->getMaxCycleTime());
        hw_state->set_current_cycle_time(
                robot_ptr_->hw_interface_->getCurrCycleTime());
        hw_state->set_slave_num(robot_ptr_->hw_interface_->getSlaveNumber());

        // Flange State
        auto flange = robot_ptr_->getFlange();
        auto flange_state = response->mutable_robot_state()->mutable_flange_state();
        auto position = flange_state->mutable_pose()->mutable_position();
        position->set_x(flange.p.x());
        position->set_y(flange.p.y());
        position->set_z(flange.p.z());
        auto rotation = flange_state->mutable_pose()->mutable_rotation();
        double x, y, z, w;
        flange.M.GetQuaternion(x, y, z, w);
        rotation->set_x(x);
        rotation->set_y(y);
        rotation->set_z(z);
        rotation->set_w(w);
        //tool State
        auto tool = robot_ptr_->getTool();
        auto tool_state = response->mutable_robot_state()->mutable_tool_state();
        auto tool_position = tool_state->mutable_pose()->mutable_position();
        tool_position->set_x(tool.p.x());
        tool_position->set_y(tool.p.y());
        tool_position->set_z(tool.p.z());
        auto tool_rotation = tool_state->mutable_pose()->mutable_rotation();
        double tool_x, tool_y, tool_z, tool_w;
        tool.M.GetQuaternion(tool_x, tool_y, tool_z, tool_w);
        tool_rotation->set_x(tool_x);
        tool_rotation->set_y(tool_y);
        tool_rotation->set_z(tool_z);
        tool_rotation->set_w(tool_w);
        //object State
        auto object = robot_ptr_->getObject();
        auto object_state = response->mutable_robot_state()->mutable_obj_state();
        auto object_position = object_state->mutable_pose()->mutable_position();
        object_position->set_x(object.p.x());
        object_position->set_y(object.p.y());
        object_position->set_z(object.p.z());
        auto object_rotation = object_state->mutable_pose()->mutable_rotation();
        double object_x, object_y, object_z, object_w;
        object.M.GetQuaternion(object_x, object_y, object_z, object_w);
        object_rotation->set_x(object_x);
        object_rotation->set_y(object_y);
        object_rotation->set_z(object_z);
        object_rotation->set_w(object_w);
        //T_tool_state
        auto T_tool = robot_ptr_->getT_tool_();
        auto T_tool_state = response->mutable_robot_state()->mutable_t_tool();
        auto T_tool_position = T_tool_state->mutable_pose()->mutable_position();
        T_tool_position->set_x(T_tool.p.x());
        T_tool_position->set_y(T_tool.p.y());
        T_tool_position->set_z(T_tool.p.z());
        auto T_tool_rotation = T_tool_state->mutable_pose()->mutable_rotation();
        double T_tool_x, T_tool_y, T_tool_z, T_tool_w;
        T_tool.M.GetQuaternion(T_tool_x, T_tool_y, T_tool_z, T_tool_w);
        T_tool_rotation->set_x(T_tool_x);
        T_tool_rotation->set_y(T_tool_y);
        T_tool_rotation->set_z(T_tool_z);
        T_tool_rotation->set_w(T_tool_w);
        //T_obj_state
        auto T_obj = robot_ptr_->getT_object_();
        auto T_obj_state = response->mutable_robot_state()->mutable_t_object();
        auto T_obj_position = T_obj_state->mutable_pose()->mutable_position();
        T_obj_position->set_x(T_obj.p.x());
        T_obj_position->set_y(T_obj.p.y());
        T_obj_position->set_z(T_obj.p.z());
        auto T_obj_rotation = T_obj_state->mutable_pose()->mutable_rotation();
        double T_obj_x, T_obj_y, T_obj_z, T_obj_w;
        T_obj.M.GetQuaternion(T_obj_x, T_obj_y, T_obj_z, T_obj_w);
        T_obj_rotation->set_x(T_obj_x);
        T_obj_rotation->set_y(T_obj_y);
        T_obj_rotation->set_z(T_obj_z);
        T_obj_rotation->set_w(T_obj_w);
        //pose_out state
        auto pose_out = robot_ptr_->getPose_out();
        auto pose_out_state = response->mutable_robot_state()->mutable_pose_out();
        auto pose_out_position = pose_out_state->mutable_pose()->mutable_position();
        pose_out_position->set_x(pose_out.p.x());
        pose_out_position->set_y(pose_out.p.y());
        pose_out_position->set_z(pose_out.p.z());
        auto pose_out_rotation = pose_out_state->mutable_pose()->mutable_rotation();
        double pose_out_x, pose_out_y, pose_out_z, pose_out_w;
        pose_out.M.GetQuaternion(pose_out_x, pose_out_y, pose_out_z, pose_out_w);
        pose_out_rotation->set_x(pose_out_x);
        pose_out_rotation->set_y(pose_out_y);
        pose_out_rotation->set_z(pose_out_z);
        pose_out_rotation->set_w(pose_out_w);

       





        return grpc::Status::OK;
    }

    grpc::Status RobotServiceImpl::WriteRobotCommmand(
            ::grpc::ServerContext *context, const ::rocos::RobotCommandRequest *request,
            ::rocos::RobotCommandResponse *response) {
        // ResponseHeader
        *response->mutable_header()->mutable_response_timestamp() =
                TimeUtil::GetCurrentTime();  //!< response timestamp

        // Process Request RobotCommand
        if (request->command().has_enabled()) {
            robot_ptr_->setEnabled();
        } else if (request->command().has_disabled()) {
            robot_ptr_->setDisabled();
        } else if (request->command().has_move_j()) {
        }
            //! Single Axis Command
        else if (request->command().has_single_axis_command()) {
            auto singleAxisCmd = request->command().single_axis_command();

            if (singleAxisCmd.has_enabled()) {  //!< enabled
                robot_ptr_->setJointEnabled(singleAxisCmd.enabled().id());
            } else if (singleAxisCmd.has_disabled()) {  //!< disabled
                robot_ptr_->setJointDisabled(singleAxisCmd.disabled().id());
            } else if (singleAxisCmd.has_mode()) {  //!< mode
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
                        //                        modeOfOperation =
                        //                        ModeOfOperation::CyclicSynchronousTorqueMode;
                        break;
                }
                robot_ptr_->setJointMode(singleAxisCmd.mode().id(), modeOfOperation);
            } else if (singleAxisCmd.has_move()) {  /////////// move
                auto id = singleAxisCmd.move().id();
                double pos = -1, max_vel = -1, max_acc = -1, max_jerk = -1,
                        least_time = -1;
                if (singleAxisCmd.move().has_raw_data() &&
                    singleAxisCmd.move().raw_data()) {
                    // TODO: 需要进行变换，但不用加入offset，offset为底层使用
                    auto cnt_per_unit = robot_ptr_->getJointCntPerUnit(id);
                    pos = singleAxisCmd.move().pos() / cnt_per_unit;
                    if (singleAxisCmd.move().has_max_vel())
                        max_vel = singleAxisCmd.move().max_vel() / cnt_per_unit;
                    if (singleAxisCmd.move().has_max_acc())
                        max_acc = singleAxisCmd.move().max_acc() / cnt_per_unit;
                    if (singleAxisCmd.move().has_max_jerk())
                        max_jerk = singleAxisCmd.move().max_jerk();
                    if (singleAxisCmd.move().has_least_time())
                        least_time = singleAxisCmd.move().least_time();
                } else {
                    pos = singleAxisCmd.move().pos();
                    if (singleAxisCmd.move().has_max_vel())
                        max_vel = singleAxisCmd.move().max_vel();
                    if (singleAxisCmd.move().has_max_acc())
                        max_acc = singleAxisCmd.move().max_acc();
                    if (singleAxisCmd.move().has_max_jerk())
                        max_jerk = singleAxisCmd.move().max_jerk();
                    if (singleAxisCmd.move().has_least_time())
                        least_time = singleAxisCmd.move().least_time();
                }

                //                std::cout << "max_vel: " << max_vel << "; max_acc: " <<
                //                max_acc << "; max_jerk: " << max_jerk << std::endl;

                robot_ptr_->moveSingleAxis(id, pos, 0.0, max_vel, max_acc, max_jerk,
                                           least_time);
            } else if (singleAxisCmd.has_stop()) {  /////////// stop
                std::cout << "stop" << std::endl;
                robot_ptr_->stopSingleAxis(singleAxisCmd.stop().id());
            }
        }
            //! Multi Axis Command
        else if (request->command().has_multi_axis_command()) {
            auto multiAxisCmd = request->command().multi_axis_command();
            if (multiAxisCmd.has_enabled()) {
                robot_ptr_->setEnabled();
            } else if (multiAxisCmd.has_disabled()) {
                robot_ptr_->setDisabled();
            } else if (multiAxisCmd.has_mode()) {
                for (int i = 0; i < robot_ptr_->jnt_num_; i++) {
                    ModeOfOperation modeOfOperation =
                            ModeOfOperation::CyclicSynchronousPositionMode;
                    switch (multiAxisCmd.mode().value().at(i)) {
                        case MODE_CSP:
                            modeOfOperation = ModeOfOperation::CyclicSynchronousPositionMode;
                            break;
                        case MODE_CSV:
                            modeOfOperation = ModeOfOperation::CyclicSynchronousVelocityMode;
                            break;
                        case MODE_CST:
                            std::cout << "CST is not implemented!" << std::endl;
                            //                        modeOfOperation =
                            //                        ModeOfOperation::CyclicSynchronousTorqueMode;
                            break;
                    }
                    robot_ptr_->setJointMode(i, modeOfOperation);
                }
            } else if (multiAxisCmd.has_sync()) {
                robot_ptr_->setSynchronization(
                        static_cast<Robot::Synchronization>(multiAxisCmd.sync().value()));
            } else if (multiAxisCmd.has_move()) {
                if (multiAxisCmd.move().has_sync()) {  //设置同步模式
                    robot_ptr_->setSynchronization(
                            static_cast<Robot::Synchronization>(multiAxisCmd.move().sync()));
                }

                if (multiAxisCmd.move().has_raw_data() &&
                    multiAxisCmd.move().raw_data()) {
                    for (int i = 0; i < robot_ptr_->jnt_num_; i++) {
                        auto cnt_per_unit = robot_ptr_->getJointCntPerUnit(i);
                        robot_ptr_->moveSingleAxis(
                                i, multiAxisCmd.move().target_pos().at(i) / cnt_per_unit, 0.0,
                                multiAxisCmd.move().max_vel().at(i) / cnt_per_unit,
                                multiAxisCmd.move().max_acc().at(i) / cnt_per_unit,
                                multiAxisCmd.move().max_jerk().at(i) / cnt_per_unit, -1);
                    }
                } else {
                    for (int i = 0; i < robot_ptr_->jnt_num_; i++) {
                        robot_ptr_->moveSingleAxis(i, multiAxisCmd.move().target_pos().at(i),
                                                   0.0, multiAxisCmd.move().max_vel().at(i),
                                                   multiAxisCmd.move().max_acc().at(i),
                                                   multiAxisCmd.move().max_jerk().at(i), -1);
                    }
                }
            } else if (multiAxisCmd.has_stop()) {
                std::cout << "stop" << std::endl;
                robot_ptr_->stopMultiAxis();
            }
            //            robot_ptr_->setJointDisabled(request->command().single_axis_disabled().id());
        }
            //! Motion Command
        else if (request->command().has_motion_command()) {
            auto motionCmd = request->command().motion_command();
            if (motionCmd.has_move_j()) {
                std::cout << "Received MoveJ command!" << std::endl;
                auto movej = motionCmd.move_j();
                JntArray q(movej.q().data().size());
                for (int i = 0; i < q.rows(); ++i) {
                    q(i) = movej.q().data().at(i);
                    std::cout << "q(" << i << "):" << q(i) << std::endl;
                }
                double speed = movej.speed();
                double acceleration = movej.acceleration();
                double time = movej.time();
                double radius = movej.radius();
                bool asynchronous = movej.asynchronous();
                robot_ptr_->MoveJ(q, speed, acceleration, time, radius, asynchronous);
            } else if (motionCmd.has_move_j_ik()) {
                std::cout << "Received MoveJ_IK command!" << std::endl;
                auto movej_ik = motionCmd.move_j_ik();
                Frame pose;
                pose.p.x(movej_ik.pose().position().x());
                pose.p.y(movej_ik.pose().position().y());
                pose.p.z(movej_ik.pose().position().z());

                pose.M = Rotation::Quaternion(
                        movej_ik.pose().rotation().x(), movej_ik.pose().rotation().y(),
                        movej_ik.pose().rotation().z(), movej_ik.pose().rotation().w());

                double speed = movej_ik.speed();
                double acceleration = movej_ik.acceleration();
                double time = movej_ik.time();
                double radius = movej_ik.radius();
                bool asynchronous = movej_ik.asynchronous();

                robot_ptr_->MoveJ_IK(pose, speed, acceleration, time, radius,
                                     asynchronous);
            } else if (motionCmd.has_move_l()) {
                std::cout << "Received MoveL command!" << std::endl;
                auto movel = motionCmd.move_l();
                Frame pose;
                pose.p.x(movel.pose().position().x());
                pose.p.y(movel.pose().position().y());
                pose.p.z(movel.pose().position().z());

                pose.M = Rotation::Quaternion(
                        movel.pose().rotation().x(), movel.pose().rotation().y(),
                        movel.pose().rotation().z(), movel.pose().rotation().w());

                double speed = movel.speed();
                double acceleration = movel.acceleration();
                double time = movel.time();
                double radius = movel.radius();
                bool asynchronous = movel.asynchronous();

                robot_ptr_->MoveL(pose, speed, acceleration, time, radius, asynchronous);
            } else if (motionCmd.has_move_l_fk()) {
                std::cout << "Received MoveL_FK command!" << std::endl;
                auto movel_fk = motionCmd.move_l_fk();
                JntArray q(movel_fk.q().data().size());
                for (int i = 0; i < q.rows(); ++i) {
                    q(i) = movel_fk.q().data().at(i);
                }
                double speed = movel_fk.speed();
                double acceleration = movel_fk.acceleration();
                double time = movel_fk.time();
                double radius = movel_fk.radius();
                bool asynchronous = movel_fk.asynchronous();

                robot_ptr_->MoveL_FK(q, speed, acceleration, time, radius, asynchronous);
            } else if (motionCmd.has_move_c()) {
                std::cout << "Received MoveC command!" << std::endl;
                auto movec = motionCmd.move_c();
                Frame pose_via;
                pose_via.p.x(movec.pose_via().position().x());
                pose_via.p.y(movec.pose_via().position().y());
                pose_via.p.z(movec.pose_via().position().z());

                pose_via.M = Rotation::Quaternion(
                        movec.pose_via().rotation().x(), movec.pose_via().rotation().y(),
                        movec.pose_via().rotation().z(), movec.pose_via().rotation().w());

                Frame pose_to;
                pose_to.p.x(movec.pose_to().position().x());
                pose_to.p.y(movec.pose_to().position().y());
                pose_to.p.z(movec.pose_to().position().z());

                pose_to.M = Rotation::Quaternion(
                        movec.pose_to().rotation().x(), movec.pose_to().rotation().y(),
                        movec.pose_to().rotation().z(), movec.pose_to().rotation().w());

                auto speed = movec.speed();
                auto acceleration = movec.acceleration();
                auto time = movec.time();
                auto radius = movec.radius();
                auto asynchronous = movec.asynchronous();
                auto mode = static_cast<Robot::OrientationMode>(movec.mode());

                robot_ptr_->MoveC(pose_via, pose_to, speed, acceleration, time, radius,
                                  mode, asynchronous);
            } else if (motionCmd.has_move_p()) {
                std::cout << "Received MoveP command!" << std::endl;
                auto movep = motionCmd.move_p();
                Frame pose;
                pose.p.x(movep.pose().position().x());
                pose.p.y(movep.pose().position().y());
                pose.p.z(movep.pose().position().z());

                pose.M = Rotation::Quaternion(
                        movep.pose().rotation().x(), movep.pose().rotation().y(),
                        movep.pose().rotation().z(), movep.pose().rotation().w());

                auto speed = movep.speed();
                auto acceleration = movep.acceleration();
                auto time = movep.time();
                auto radius = movep.radius();
                auto asynchronous = movep.asynchronous();

                robot_ptr_->MoveP(pose, speed, acceleration, time, radius, asynchronous);
            } else if (motionCmd.has_move_path()) {
                //! TODO: MovePath
                std::cout << "Received MovePath command!" << std::endl;
            }
            
        }
            //! Dragging Command
        else if (request->command().has_dragging_command()) {
            auto dragging_command = request->command().dragging_command();
            robot_ptr_->Dragging(static_cast<Robot::DRAGGING_FLAG>(dragging_command.flag()),
                                 static_cast<Robot::DRAGGING_DIRRECTION>(dragging_command.dir()),
                                 dragging_command.max_speed(), dragging_command.max_acceleration());
        }
            //! General Command
        else if (request->command().has_general_command()) {
            auto general_command = request->command().general_command();
            if (general_command.has_set_work_mode()) {
                robot_ptr_->setWorkMode(static_cast<Robot::WorkMode>(general_command.set_work_mode().value()));
            }
        }
        else if(request->command().has_calibration_command()){
            auto calibration_command = request->command().calibration_command();
            if(calibration_command.has_set_pose_frame())
            {
                std::cout << "Received set_pose_frame command!" << std::endl;
                auto pose_frame = calibration_command.set_pose_frame();
                auto id=pose_frame.id();

                Frame pose;
                pose.p.x(pose_frame.pose().position().x());
                pose.p.y(pose_frame.pose().position().y());
                pose.p.z(pose_frame.pose().position().z());
                pose.M=Rotation::Quaternion(pose_frame.pose().rotation().x(),
                                            pose_frame.pose().rotation().y(),
                                            pose_frame.pose().rotation().z(),
                                            pose_frame.pose().rotation().w());
                
                robot_ptr_->set_pose_frame(id,pose);
            }
            else if(calibration_command.has_tool_calibration())
            {
                robot_ptr_->tool_calibration();
            }
            else if(calibration_command.has_set_tool_frame())
            {
                auto set_tool_frame = calibration_command.set_tool_frame();
                Frame pose;
                pose.p.x(set_tool_frame.pose().position().x());
                pose.p.y(set_tool_frame.pose().position().y());
                pose.p.z(set_tool_frame.pose().position().z());
                pose.M=Rotation::Quaternion(
                        set_tool_frame.pose().rotation().x(), set_tool_frame.pose().rotation().y(),
                        set_tool_frame.pose().rotation().z(), set_tool_frame.pose().rotation().w());
                robot_ptr_->set_tool_frame(pose);

            }
            else if(calibration_command.has_set_object_frame())
            {
                auto set_object_frame = calibration_command.set_object_frame();
                Frame pose;
                pose.p.x(set_object_frame.pose().position().x());
                pose.p.y(set_object_frame.pose().position().y());
                pose.p.z(set_object_frame.pose().position().z());

                pose.M = Rotation::Quaternion(
                        set_object_frame.pose().rotation().x(), set_object_frame.pose().rotation().y(),
                        set_object_frame.pose().rotation().z(), set_object_frame.pose().rotation().w());
                robot_ptr_->set_object_frame(pose);
            }

        }
      

        return grpc::Status::OK;
    }

    void RobotServiceImpl::runServer(const std::string &address, bool isDetached) {
        thread_ = boost::make_shared<boost::thread>(
                boost::bind(&RobotServiceImpl::serverThread, this, address));
        if (isDetached) {
            thread_->detach();
        } else {
            thread_->join();
        }
    }

    void RobotServiceImpl::stopServer() { server_ptr_->Shutdown(); }

    void RobotServiceImpl::serverThread(const std::string &address) {
        is_thread_running_ = true;

        grpc::EnableDefaultHealthCheckService(true);
        grpc::reflection::InitProtoReflectionServerBuilderPlugin();
        grpc::ServerBuilder builder;

        // Listen on the given address without any authentication mechanism.
        builder.AddListeningPort(address, grpc::InsecureServerCredentials());

        // Register "service" as the instance through which we'll communicate with
        // clients. In this case it corresponds to an *synchronous* service.
        builder.RegisterService(this);

        // Finally assemble the server.
        server_ptr_ = builder.BuildAndStart();
        std::cout << "Server listening on " << address << std::endl;

        // Wait for the server to shutdown. Note that some other thread must be
        // responsible for shutting down the server for this call to ever return.

        server_ptr_->Wait();

        is_thread_running_ = false;
    }

    // 单例模式对象
    boost::shared_ptr<RobotServiceImpl> RobotServiceImpl::instance_ = nullptr;


}  // namespace rocos