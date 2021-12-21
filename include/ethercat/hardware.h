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

#ifndef ROCOS_APP_HARDWARE_H
#define ROCOS_APP_HARDWARE_H

#include <ethercat/command.h>
#include <ethercat/control_word.h>
#include <ethercat/drive_state.h>
#include <ethercat/status_word.h>
#include <ethercat/mode_of_operation.h>

#include <ethercat/ecat_config.hpp>

#include <hardware_interface.h>


#include <boost/smart_ptr.hpp>

namespace rocos {

    class Hardware : public HardwareInterface {
    public:
        Hardware();
        ~Hardware() override;

        Timestamp getTimestamp() override;

        double getMinCycleTime() override;

        double getMaxCycleTime() override;

        double getAvgCycleTime() override;

        double getCurrCycleTime() override;

        void setTargetPositionRaw(int id, int32_t pos) override;

        void setTargetVelocityRaw(int id, int32_t vel) override;

        void setTargetTorqueRaw(int id, int16_t tor) override;

        int32_t getActualPositionRaw(int id) override;

        int32_t getActualVelocityRaw(int id) override;

        int16_t getActualTorqueRaw(int id) override;

        int16_t getLoadTorqueRaw(int id) override;

        uint16_t getStatuswordRaw(int id) override;

        std::string getSlaveName(int id) override;

        int32_t getSlaveNumber() override;

        void setControlwordRaw(int id, uint16_t ctrlwd) override;

        void setModeOfOperationRaw(int id, int8_t mode) override;

        void setModeOfOperation(int id, ModeOfOperation modeOfOperation) override;

        Statusword getStatusword(int id) override;

        DriveState getDriverState(int id) override;

        void waitForSignal(int id = 0) override;

    protected:
        boost::shared_ptr<EcatConfig> ecPtr;
    };

}


#endif //ROCOS_APP_HARDWARE_H
