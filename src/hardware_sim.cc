//
// Created by think on 2021/12/9.
//

#include <rocos_app/ethercat/hardware_sim.h>

namespace rocos {

    HardwareSim::HardwareSim(int slave_num) : _slaveNumber(slave_num),
                                              _positionVec(slave_num),
                                              _velocityVec(slave_num),
                                              _torqueVec(slave_num),
                                              _loadTorqueVec(slave_num),
                                              _controlwordVec(slave_num),
                                              _statuswordVec(slave_num),
                                              _modeVec(slave_num),
                                              _driveStateVec(slave_num, DriveState::OperationEnabled) {

        _type = HW_SIM;
        for (auto &s: _statuswordVec) {
            s.setFromRawStatusword(4663); // in simulation all is enabled forever.
//            std::cout << "Init raw status word" <<  s << std::endl;
        }
    }

    HardwareSim::~HardwareSim() = default;

    Timestamp HardwareSim::getTimestamp() {
        return boost::chrono::system_clock::now();
    }

    int32_t HardwareSim::getSlaveNumber() {
        return _slaveNumber;
    }

    void HardwareSim::setTargetPositionRaw(int id, int32_t pos) {
        _positionVec[id] = pos;
    }

    void HardwareSim::setTargetVelocityRaw(int id, int32_t vel) {
        _velocityVec[id] = vel;
    }

    void HardwareSim::setTargetTorqueRaw(int id, int16_t tor) {
        _torqueVec[id] = tor;
    }

    void HardwareSim::setControlwordRaw(int id, uint16_t ctrlwd) {
        _controlwordVec[id].setFromRawControlword(ctrlwd);
    }

    void HardwareSim::setModeOfOperationRaw(int id, int8_t mode) {
        _modeVec[id] = mode;
    }

    void HardwareSim::setModeOfOperation(int id, ModeOfOperation modeOfOperation) {
        _modeVec[id] = static_cast<int8_t>(modeOfOperation);
    }

    int32_t HardwareSim::getActualPositionRaw(int id) {
        return _positionVec[id];
    }

    int32_t HardwareSim::getActualVelocityRaw(int id) {
        return _velocityVec[id];
    }

    int16_t HardwareSim::getActualTorqueRaw(int id) {
        return _torqueVec[id];
    }

    int16_t HardwareSim::getLoadTorqueRaw(int id) {
        return _loadTorqueVec[id];
    }

    uint16_t HardwareSim::getStatuswordRaw(int id) {
        return _statuswordVec[id].getRawStatusword();
    }

    Statusword HardwareSim::getStatusword(int id) {
        return _statuswordVec[id];
    }

    DriveState HardwareSim::getDriverState(int id) {
        return _driveStateVec[id];
    }

    void HardwareSim::setSlaveNumber(int slave_num) {
        _slaveNumber = slave_num;
        _positionVec.resize(slave_num, 0);
        _velocityVec.resize(slave_num, 0);
        _torqueVec.resize(slave_num, 0);
        _loadTorqueVec.resize(slave_num, 0);
        _modeVec.resize(slave_num, 0);
        _statuswordVec.resize(slave_num);
        _controlwordVec.resize(slave_num);
        _driveStateVec.resize(slave_num, DriveState::OperationEnabled);

        for (auto s: _statuswordVec) {
            s.setFromRawStatusword(4663);
        }
    }

    std::string HardwareSim::getSlaveName(int id) {
        return (slave_name_format_ % id).str();
    }


    double HardwareSim::getCurrCycleTime() {
        curr_cycle_time_ = 1000.0 + rand_num(generator);
        min_cycle_time_ = curr_cycle_time_ < min_cycle_time_ ? curr_cycle_time_ : min_cycle_time_;
        max_cycle_time_ = curr_cycle_time_ > max_cycle_time_ ? curr_cycle_time_ : max_cycle_time_;
        return curr_cycle_time_;
    }

} // namespace rocos