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

#include <ethercat/hardware_interface.h>

#include <boost/core/ignore_unused.hpp>

namespace rocos {


    HardwareInterface::~HardwareInterface() = default;

    HardwareInterface::HWType rocos::HardwareInterface::getHardwareType() {
        return HardwareInterface::HW_UNKNOWN;
    }

    std::string HardwareInterface::getHardwareTypeString(HardwareInterface::HWType type) {
        std::string str;
        switch (type) {
            case HW_UNKNOWN:
                str = "Unknown";
                break;
            case HW_ETHERCAT:
                str = "EtherCAT";
                break;
            case HW_SIM:
                str = "Simulation";
                break;
            default:
                break;
        }
        return str;
    }

    Statusword HardwareInterface::getStatusword(int id) {
        return Statusword();
    }

    uint16_t HardwareInterface::getStatuswordRaw(int id) {
        return Statusword().getRawStatusword();
    }

    void HardwareInterface::setTargetPositionRaw(int id, int32_t pos) {
        boost::ignore_unused(id, pos);
    }

    void HardwareInterface::setTargetVelocityRaw(int id, int32_t vel) {
        boost::ignore_unused(id, vel);
    }

    void HardwareInterface::setTargetTorqueRaw(int id, int32_t tor) {
        boost::ignore_unused(id, tor);
    }

    int32_t HardwareInterface::getActualPositionRaw(int id) {
        return 0;
    }

    int32_t HardwareInterface::getActualVelocityRaw(int id) {
        return 0;
    }

    int16_t HardwareInterface::getActualTorqueRaw(int id) {
        return 0;
    }


    int16_t HardwareInterface::getLoadTorqueRaw(int id) {
        boost::ignore_unused(id);
        return 0;
    }

    DriveState HardwareInterface::getDriverState(int id) {
        boost::ignore_unused(id);
        return DriveState();
    }

}

