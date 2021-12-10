//
// Created by think on 2021/12/9.
//

#include <ethercat/hardware_sim.h>

namespace rocos {

    HardwareSim::HardwareSim() {

    }

    HardwareSim::~HardwareSim() = default;

    Timestamp HardwareSim::getTimestamp() {
        return HardwareInterface::getTimestamp();
    }

    double HardwareSim::getMinCycleTime() {
        return HardwareInterface::getMinCycleTime();
    }

    double HardwareSim::getMaxCycleTime() {
        return HardwareInterface::getMaxCycleTime();
    }

    double HardwareSim::getAvgCycleTime() {
        return HardwareInterface::getAvgCycleTime();
    }

    double HardwareSim::getCurrCycleTime() {
        return HardwareInterface::getCurrCycleTime();
    }

    int32_t HardwareSim::getSlaveNumber() {
        return HardwareInterface::getSlaveNumber();
    }

    void HardwareSim::setTargetPositionRaw(int id, int32_t pos) {
        HardwareInterface::setTargetPositionRaw(id, pos);
    }

    void HardwareSim::setTargetVelocityRaw(int id, int32_t vel) {
        HardwareInterface::setTargetVelocityRaw(id, vel);
    }

    void HardwareSim::setTargetTorqueRaw(int id, int32_t tor) {
        HardwareInterface::setTargetTorqueRaw(id, tor);
    }

    void HardwareSim::setControlwordRaw(int id, uint16_t ctrlwd) {
        HardwareInterface::setControlwordRaw(id, ctrlwd);
    }

    void HardwareSim::setModeOfOperationRaw(int id, int8_t mode) {
        HardwareInterface::setModeOfOperationRaw(id, mode);
    }

    void HardwareSim::setModeOfOperation(int id, ModeOfOperation modeOfOperation) {
        HardwareInterface::setModeOfOperation(id, modeOfOperation);
    }

    int32_t HardwareSim::getActualPositionRaw(int id) {
        return HardwareInterface::getActualPositionRaw(id);
    }

    int32_t HardwareSim::getActualVelocityRaw(int id) {
        return HardwareInterface::getActualVelocityRaw(id);
    }

    int16_t HardwareSim::getActualTorqueRaw(int id) {
        return HardwareInterface::getActualTorqueRaw(id);
    }

    int16_t HardwareSim::getLoadTorqueRaw(int id) {
        return HardwareInterface::getLoadTorqueRaw(id);
    }

    uint16_t HardwareSim::getStatuswordRaw(int id) {
        return HardwareInterface::getStatuswordRaw(id);
    }

    Statusword HardwareSim::getStatusword(int id) {
        return HardwareInterface::getStatusword(id);
    }

    DriveState HardwareSim::getDriverState(int id) {
        return HardwareInterface::getDriverState(id);
    }

} // namespace rocos