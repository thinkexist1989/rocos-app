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

#include <rocos_app/ethercat/hardware.h>
#include <tinyxml2.h> // parse urdf


namespace rocos {

    Hardware::~Hardware() = default;

    Hardware::Hardware(const std::string &urdf_file_path) {
        _type = HW_ETHERCAT;
        ecPtr_ = EcatConfig::getInstance();

        slave_num_ = ecPtr_->getSlaveNum();

        pTargetPos_.resize(slave_num_, nullptr);
        pTargetVel_.resize(slave_num_, nullptr);
        pTargetTor_.resize(slave_num_, nullptr);
        pActualPos_.resize(slave_num_, nullptr);
        pActualVel_.resize(slave_num_, nullptr);
        pActualTor_.resize(slave_num_, nullptr);
        pLoadTor_.resize(slave_num_, nullptr);
        pSecondaryPos_.resize(slave_num_, nullptr);
        pSecondaryVel_.resize(slave_num_, nullptr);
        pStatusword_.resize(slave_num_, nullptr);
        pControlword_.resize(slave_num_, nullptr);
        pModeOfOp_.resize(slave_num_, nullptr);

        parseParamFormUrdf(urdf_file_path);

    }

    long Hardware::getTimestamp() {
        return ecPtr_->getTimestamp();
    }

    double Hardware::getMinCycleTime() {
        return ecPtr_->getBusMinCycleTime();
    }

    double Hardware::getMaxCycleTime() {
        return ecPtr_->getBusMaxCycleTime();
    }

    double Hardware::getAvgCycleTime() {
        return ecPtr_->getBusAvgCycleTime();
    }

    double Hardware::getCurrCycleTime() {
        return ecPtr_->getBusCurrentCycleTime();
    }

    void Hardware::setTargetPositionRaw(int id, int32_t pos) {
        if(pTargetPos_[id]) *pTargetPos_[id] = pos;
    }

    void Hardware::setTargetVelocityRaw(int id, int32_t vel) {
        if(pTargetVel_[id]) *pTargetVel_[id] = vel;
    }

    void Hardware::setTargetTorqueRaw(int id, int16_t tor) {
        if (pTargetTor_[id]) *pTargetTor_[id] = tor;
    }

    int32_t Hardware::getActualPositionRaw(int id) {
        return pActualPos_[id] ? *pActualPos_[id] : (int32_t) 0;
    }

    int32_t Hardware::getActualVelocityRaw(int id) {
        return pActualVel_[id] ? *pActualVel_[id] : (int32_t) 0;
    }

    int16_t Hardware::getActualTorqueRaw(int id) {
        return pActualTor_[id] ? *pActualTor_[id] : (int16_t) 0;
    }

    int16_t Hardware::getLoadTorqueRaw(int id) {
        return pLoadTor_[id] ? *pLoadTor_[id] : (int16_t) 0;
    }

    uint16_t Hardware::getStatuswordRaw(int id) {
        return pStatusword_[id] ? *pStatusword_[id] : (uint16_t) 0;
    }

    void Hardware::waitForSignal(int id) {
        ecPtr_->waitForSignal(id);
    }

    void Hardware::wait() {
       ecPtr_->wait();
    }

    int32_t Hardware::getSlaveNumber() {
        return ecPtr_->getSlaveNum();
    }

    void Hardware::setControlwordRaw(int id, uint16_t ctrlwd) {
        if(pControlword_[id]) *pControlword_[id] = ctrlwd;
    }

    void Hardware::setModeOfOperationRaw(int id, int8_t mode) {
        if(pModeOfOp_[id]) *pModeOfOp_[id] = mode;
    }

    std::string Hardware::getSlaveName(int id) {
        return ecPtr_->getSlaveName(id);
    }

    HardwareInterface::HWState Hardware::getHardwareState() {
        return static_cast<HardwareInterface::HWState >(ecPtr_->getBusCurrentState());
    }

    void Hardware::setHardwareState(HardwareInterface::HWState state) {
        ecPtr_->setBusRequestState(state);
    }

    int32_t Hardware::getSecondaryPositionRaw(int id) {
        return pSecondaryPos_[id] ? *pSecondaryPos_[id] : (int32_t) 0;
    }

    int32_t Hardware::getSecondaryVelocityRaw(int id) {
        return pSecondaryVel_[id] ? *pSecondaryVel_[id] : (int32_t) 0;
    }

    void Hardware::parseParamFormUrdf(const std::string &urdf_file_path) {
        tinyxml2::XMLDocument doc;
        doc.LoadFile(urdf_file_path.c_str());

        auto robot = doc.FirstChildElement("robot");
        for (auto jnt = robot->FirstChildElement("joint"); jnt; jnt = jnt->NextSiblingElement("joint")) {

            auto hw = jnt->FirstChildElement("hardware");

            auto id = hw->IntAttribute("id", -1); // 对应的硬件ID，若没指定默认为-1

            if(!hw->Attribute("type", "driver")) { // 如果type为driver，则为驱动关节
                continue;
            }

            std::cout << "joint < " << jnt->Attribute("name") << " >'s assigned slave <id: " << id << " > is a driver joint." << std::endl;


            auto inputs = hw->FirstChildElement("inputs");
            if(inputs) { // 关联相关字段
                auto status_word = inputs->FirstChildElement("status_word");
                if(status_word) {
                    pStatusword_[id] = ecPtr_->findSlaveInputVarPtrByName<uint16_t>(id, status_word->GetText());
                } else {
                    pStatusword_[id] = ecPtr_->findSlaveInputVarPtrByName<uint16_t>(id, "Status word");
                }

                auto position_actual_value = inputs->FirstChildElement("position_actual_value");
                if(position_actual_value) {
                    pActualPos_[id] = ecPtr_->findSlaveInputVarPtrByName<int32_t>(id, position_actual_value->GetText());
                } else {
                    pActualPos_[id] = ecPtr_->findSlaveInputVarPtrByName<int32_t>(id, "Position actual value");
                }

                auto velocity_actual_value = inputs->FirstChildElement("velocity_actual_value");
                if(velocity_actual_value) {
                    pActualVel_[id] = ecPtr_->findSlaveInputVarPtrByName<int32_t>(id, velocity_actual_value->GetText());
                } else {
                    pActualVel_[id] = ecPtr_->findSlaveInputVarPtrByName<int32_t>(id, "Velocity actual value");
                }

                auto torque_actual_value = inputs->FirstChildElement("torque_actual_value");
                if(torque_actual_value) {
                    pActualTor_[id] = ecPtr_->findSlaveInputVarPtrByName<int16_t>(id, torque_actual_value->GetText());
                } else {
                    pActualTor_[id] = ecPtr_->findSlaveInputVarPtrByName<int16_t>(id, "Torque actual value");
                }

                auto load_torque_value = inputs->FirstChildElement("load_torque_value");
                if(load_torque_value) {
                    pLoadTor_[id] = ecPtr_->findSlaveInputVarPtrByName<int16_t>(id, load_torque_value->GetText());
                } else {
                    pLoadTor_[id] = ecPtr_->findSlaveInputVarPtrByName<int16_t>(id, "Analog Input 1");
                }

                auto secondary_position_value = inputs->FirstChildElement("secondary_position_value");
                if(secondary_position_value) {
                    pSecondaryPos_[id] = ecPtr_->findSlaveInputVarPtrByName<int32_t>(id, secondary_position_value->GetText());
                } else {
                    pSecondaryPos_[id] = ecPtr_->findSlaveInputVarPtrByName<int32_t>(id, "Secondary position value");
                }

                auto secondary_velocity_value = inputs->FirstChildElement("secondary_velocity_value");
                if(secondary_velocity_value) {
                    pSecondaryVel_[id] = ecPtr_->findSlaveInputVarPtrByName<int32_t>(id, secondary_velocity_value->GetText());
                } else {
                    pSecondaryVel_[id] = ecPtr_->findSlaveInputVarPtrByName<int32_t>(id, "Secondary velocity value");
                }


            } else { // 全部采用默认值
                pStatusword_[id] = ecPtr_->findSlaveInputVarPtrByName<uint16_t>(id, "Status word");
                pActualPos_[id] = ecPtr_->findSlaveInputVarPtrByName<int32_t>(id, "Position actual value");
                pActualVel_[id] = ecPtr_->findSlaveInputVarPtrByName<int32_t>(id, "Velocity actual value");
                pActualTor_[id] = ecPtr_->findSlaveInputVarPtrByName<int16_t>(id, "Torque actual value");
                pLoadTor_[id] = ecPtr_->findSlaveInputVarPtrByName<int16_t>(id, "Analog Input 1");
                pSecondaryPos_[id] = ecPtr_->findSlaveInputVarPtrByName<int32_t>(id, "Secondary position value");
                pSecondaryVel_[id] = ecPtr_->findSlaveInputVarPtrByName<int32_t>(id, "Secondary velocity value");
            }

            auto outputs = hw->FirstChildElement("outputs");
            if(outputs) { // 关联相关字段
                auto control_word = outputs->FirstChildElement("control_word");
                if(control_word) {
                    pControlword_[id] = ecPtr_->findSlaveOutputVarPtrByName<uint16_t>(id, control_word->GetText());
                } else {
                    pControlword_[id] = ecPtr_->findSlaveOutputVarPtrByName<uint16_t>(id, "Control word");
                }

                auto mode_of_operation = outputs->FirstChildElement("mode_of_operation");
                if(mode_of_operation) {
                    pModeOfOp_[id] = ecPtr_->findSlaveOutputVarPtrByName<int8_t>(id, mode_of_operation->GetText());
                } else {
                    pModeOfOp_[id] = ecPtr_->findSlaveOutputVarPtrByName<int8_t>(id, "Mode of operation");
                }

                auto target_position = outputs->FirstChildElement("target_position");
                if(target_position) {
                    pTargetPos_[id] = ecPtr_->findSlaveOutputVarPtrByName<int32_t>(id, target_position->GetText());
                } else {
                    pTargetPos_[id] = ecPtr_->findSlaveOutputVarPtrByName<int32_t>(id, "Target position");
                }

                auto target_velocity = outputs->FirstChildElement("target_velocity");
                if(target_velocity) {
                    pTargetVel_[id] = ecPtr_->findSlaveOutputVarPtrByName<int32_t>(id, target_velocity->GetText());
                } else {
                    pTargetVel_[id] = ecPtr_->findSlaveOutputVarPtrByName<int32_t>(id, "Target velocity");
                }

                auto target_torque = outputs->FirstChildElement("target_torque");
                if(target_torque) {
                    pTargetTor_[id] = ecPtr_->findSlaveOutputVarPtrByName<int16_t>(id, target_torque->GetText());
                } else {
                    pTargetTor_[id] = ecPtr_->findSlaveOutputVarPtrByName<int16_t>(id, "Target torque");
                }

            }else {
                pControlword_[id] = ecPtr_->findSlaveOutputVarPtrByName<uint16_t>(id, "Control word");
                pModeOfOp_[id] = ecPtr_->findSlaveOutputVarPtrByName<int8_t>(id, "Mode of operation");
                pTargetPos_[id] = ecPtr_->findSlaveOutputVarPtrByName<int32_t>(id, "Target position");
                pTargetVel_[id] = ecPtr_->findSlaveOutputVarPtrByName<int32_t>(id, "Target velocity");
                pTargetTor_[id] = ecPtr_->findSlaveOutputVarPtrByName<int16_t>(id, "Target torque");
            }


        }
    }



}
