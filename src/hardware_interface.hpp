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

/***********Hardware Interface**************
 * 这是硬件的接口抽象类，提供hardware的相关操作，主要针对EtherCAT，
 * 通过子类化HardwareInterface（子类为Hardware和HardwareSim）
 */



#ifndef ROCOS_APP_HARDWARE_INTERFACE_H
#define ROCOS_APP_HARDWARE_INTERFACE_H

#include <string>

#include "../include/rocos_app/ethercat/mode_of_operation.h"
#include "../include/rocos_app/ethercat/status_word.h"
#include "command.hpp"
#include "control_word.hpp"
#include "drive_state.hpp"

// #include <boost/chrono.hpp>

namespace rocos {

    class HardwareInterface {
    public:
        ///////Hardware Type Definition///////
        enum HWType {
            HW_UNKNOWN,
            HW_SIM,
            HW_ETHERCAT,
            HW_CAN,      // reserved
            HW_PROFINET  // reserved
        };


        enum HWState {
            UNKNOWN = 0,
            INIT = 1,
            PREOP = 2,
            SAFEOP = 4,
            OP = 8,
            READY = 8,

            BOOTSTRAP = 3
        };

        virtual ~HardwareInterface() = default;

        ///////////////////////Data Info/////////////////////////
        virtual long getTimestamp() = 0; // Timestamp

        virtual HWState getHardwareState() = 0;
        virtual void setHardwareState(HWState state) = 0;

        virtual double getMinCycleTime() = 0; // min cycle time
        virtual double getMaxCycleTime() = 0; // max cycle time
        virtual double getAvgCycleTime() = 0; // avg cycle time
        virtual double getCurrCycleTime() = 0; // current cycle time

        virtual int32_t getSlaveNumber() = 0;   // slave number

        virtual std::string getSlaveName(int id) = 0; // slave name

        virtual void waitForSignal(int id) = 0;    // Signal of Bus

        virtual void wait() = 0;    // wait Signal of Bus

        ///////////////////////Raw Data/////////////////////////
        virtual void setTargetPositionRaw(int id, int32_t pos) = 0;

        virtual void setTargetVelocityRaw(int id, int32_t vel) = 0;

        virtual void setTargetTorqueRaw(int id, int16_t tor) = 0;

        virtual void setControlwordRaw(int id, uint16_t ctrlwd) = 0;

        virtual void setModeOfOperationRaw(int id, int8_t mode) = 0;

        virtual void setModeOfOperation(int id, ModeOfOperation modeOfOperation) = 0;

        virtual int32_t getActualPositionRaw(int id) = 0;

        virtual int32_t getActualVelocityRaw(int id) = 0;

        virtual int16_t getActualTorqueRaw(int id) = 0;

        virtual int16_t getLoadTorqueRaw(int id) = 0;

        virtual int32_t getSecondaryPositionRaw(int id) = 0;

        virtual int32_t getSecondaryVelocityRaw(int id) = 0;

        virtual uint16_t getStatuswordRaw(int id) = 0;


        virtual Statusword getStatusword(int id) = 0;

        virtual DriveState getDriverState(int id) = 0;

        virtual int32_t getDigitalInputsRaw(int id) = 0;

        virtual void setDigitalOutputsRaw(int id, int32_t value) = 0;

        /////////////////////Hardware Type//////////////////////
        virtual HWType getHardwareType() = 0;

        virtual std::string getHardwareTypeString(HWType type) = 0;

    protected:
        HWType _type = HW_UNKNOWN;

    };
}


#endif //ROCOS_APP_HARDWARE_INTERFACE_H
