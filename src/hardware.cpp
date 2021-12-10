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

#include <ethercat/hardware.h>


namespace rocos {

    Hardware::~Hardware() {
    }

    Hardware::Hardware() {
        _type = HW_ETHERCAT;
        ecPtr = boost::make_shared<EcatConfig>();

        if(!ecPtr->getSharedMemory()) {
            std::cerr << "Hardware get shared memory failed. Please check if the Ec-Master is already running." << std::endl;
        }
    }

    Timestamp Hardware::getTimestamp() {
        return ecPtr->ecatInfo->timestamp;
    }

    double Hardware::getMinCycleTime() {
        return ecPtr->ecatInfo->minCyclcTime;
    }

    double Hardware::getMaxCycleTime() {
        return ecPtr->ecatInfo->maxCycleTime;
    }

    double Hardware::getAvgCycleTime() {
        return ecPtr->ecatInfo->avgCycleTime;
    }

    double Hardware::getCurrCycleTime() {
        return ecPtr->ecatInfo->currCycleTime;
    }

    void Hardware::setTargetPositionRaw(int id, int32_t pos) {
        ecPtr->setTargetPositionEC(id, pos);
    }

    void Hardware::setTargetVelocityRaw(int id, int32_t vel) {
        ecPtr->setTargetVelocityEC(id, vel);
    }

    void Hardware::setTargetTorqueRaw(int id, int32_t tor) {
        ecPtr->setTargetTorqueEC(id, tor);
    }

    int32_t Hardware::getActualPositionRaw(int id) {
        return ecPtr->getActualPositionEC(id);
    }

    int32_t Hardware::getActualVelocityRaw(int id) {
        return ecPtr->getActualVelocityEC(id);
    }

    int16_t Hardware::getActualTorqueRaw(int id) {
        return ecPtr->getActualTorqueEC(id);
    }

    int16_t Hardware::getLoadTorqueRaw(int id) {
        return ecPtr->getLoadTorqueEC(id);
    }

    uint16_t Hardware::getStatuswordRaw(int id) {
        return ecPtr->getStatusWordEC(id);
    }

    void Hardware::waitForSignal() {
        ecPtr->waitForSignal();
    }

    int32_t Hardware::getSlaveNumber() {
        return ecPtr->ecatInfo->slave_number;
    }

    void Hardware::setControlwordRaw(int id, uint16_t ctrlwd) {
        ecPtr->setControlwordEC(id, ctrlwd);
    }

    void Hardware::setModeOfOperationRaw(int id, int8_t mode) {
        ecPtr->setModeOfOperationEC(id, mode);
    }

    void Hardware::setModeOfOperation(int id, ModeOfOperation modeOfOperation) {
        ecPtr->setModeOfOperationEC(id, static_cast<int8_t>(modeOfOperation));
    }

    Statusword Hardware::getStatusword(int id) {
        Statusword status;
        status.setFromRawStatusword(ecPtr->getStatusWordEC(id));
        return status;
    }

    DriveState Hardware::getDriverState(int id) {
        Statusword status;
        status.setFromRawStatusword(ecPtr->getStatusWordEC(id));
        return status.getDriveState();
    }

}
