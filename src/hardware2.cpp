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

#include "hardware2.hpp"
#include <tinyxml2.h> // parse urdf


namespace rocos {

    Hardware::~Hardware() {

    };

    Hardware::Hardware(const std::string &urdf_file_path, int id) {
        _type = HW_ETHERCAT;
        ecPtr_ = EcatConfig::getInstance(id);

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
        pDigitalInputs_.resize(slave_num_, nullptr);

        pStatusword_.resize(slave_num_, nullptr);
        pControlword_.resize(slave_num_, nullptr);
        pModeOfOp_.resize(slave_num_, nullptr);
        pDigitalOutputs_.resize(slave_num_, nullptr);

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

    int32_t Hardware::getDigitalInputsRaw(int id) {
        return pDigitalInputs_[id] ? *pDigitalInputs_[id] : (int32_t) 0;
    }

    void Hardware::setDigitalOutputsRaw(int id, int32_t value) {
        
        if(pDigitalOutputs_[id]!=nullptr) 
        {
            *pDigitalOutputs_[id] = value;
        }
        
        
    }


}
