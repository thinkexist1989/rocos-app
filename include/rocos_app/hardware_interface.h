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

#include "ethercat/command.h"
#include "ethercat/control_word.h"
#include "ethercat/drive_state.h"
#include "ethercat/status_word.h"
#include "ethercat/mode_of_operation.h"

#include <string>
#include <mutex>
#include <atomic>

#include <boost/chrono.hpp>

namespace rocos {

    typedef boost::chrono::time_point<boost::chrono::system_clock> Timestamp;

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

        virtual ~HardwareInterface();

        ///////////////////////Data Info/////////////////////////
        virtual Timestamp getTimestamp(); // Timestamp

        virtual HWState getHardwareState();
        virtual void setHardwareState(HWState state);

        virtual double getMinCycleTime(); // min cycle time
        virtual double getMaxCycleTime(); // max cycle time
        virtual double getAvgCycleTime(); // avg cycle time
        virtual double getCurrCycleTime(); // current cycle time

        virtual int32_t getSlaveNumber();   // slave number

        virtual std::string getSlaveName(int id); // slave name

        virtual void waitForSignal(int id);    // Signal of Bus

        ///////////////////////Raw Data/////////////////////////
        virtual void setTargetPositionRaw(int id, int32_t pos);

        virtual void setTargetVelocityRaw(int id, int32_t vel);

        virtual void setTargetTorqueRaw(int id, int16_t tor);

        virtual void setControlwordRaw(int id, uint16_t ctrlwd);

        virtual void setModeOfOperationRaw(int id, int8_t mode);

        virtual void setModeOfOperation(int id, ModeOfOperation modeOfOperation);


        virtual int32_t getActualPositionRaw(int id);

        virtual int32_t getActualVelocityRaw(int id);

        virtual int16_t getActualTorqueRaw(int id);

        virtual int16_t getLoadTorqueRaw(int id);

        virtual uint16_t getStatuswordRaw(int id);


        virtual Statusword getStatusword(int id);

        virtual DriveState getDriverState(int id);

        /////////////////////Hardware Type//////////////////////
        virtual HWType getHardwareType();

        virtual std::string getHardwareTypeString(HWType type);

    protected:
        HWType _type = HW_UNKNOWN;

    };
}


#endif //ROCOS_APP_HARDWARE_INTERFACE_H
