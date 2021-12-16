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

#ifndef ROCOS_APP_HARDWARE_SIM_H
#define ROCOS_APP_HARDWARE_SIM_H

#include <hardware_interface.h>
#include <vector>

namespace rocos {

    class HardwareSim : public HardwareInterface {
    public:
        explicit HardwareSim(int slave_num = 20);
        ~HardwareSim() override;

        //////////// Override Hardware Interface //////////////////
        Timestamp getTimestamp() override;

        int32_t getSlaveNumber() override;

        void setTargetPositionRaw(int id, int32_t pos) override;

        void setTargetVelocityRaw(int id, int32_t vel) override;

        void setTargetTorqueRaw(int id, int16_t tor) override;

        void setControlwordRaw(int id, uint16_t ctrlwd) override;

        void setModeOfOperationRaw(int id, int8_t mode) override;

        void setModeOfOperation(int id, ModeOfOperation modeOfOperation) override;

        int32_t getActualPositionRaw(int id) override;

        int32_t getActualVelocityRaw(int id) override;

        int16_t getActualTorqueRaw(int id) override;

        int16_t getLoadTorqueRaw(int id) override;

        uint16_t getStatuswordRaw(int id) override;

        Statusword getStatusword(int id) override;

        DriveState getDriverState(int id) override;


        ////////////////// SPECIAL ///////////////////////////////
        void setSlaveNumber(int slave_num);

    private:
        int _slaveNumber;
        std::vector<int32_t> _positionVec;
        std::vector<int32_t> _velocityVec;
        std::vector<int16_t> _torqueVec;
        std::vector<int16_t> _loadTorqueVec;

        std::vector<Controlword> _controlwordVec;
        std::vector<Statusword> _statuswordVec;
        std::vector<int8_t> _modeVec;
        std::vector<DriveState> _driveStateVec;
    };

}

#endif //ROCOS_APP_HARDWARE_SIM_H
