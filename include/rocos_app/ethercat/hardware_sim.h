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

#include "../hardware_interface.h"
#include <vector>
#include <boost/format.hpp>
#include <random>

namespace rocos {

    class HardwareSim : public HardwareInterface {
    public:
        explicit HardwareSim(int slave_num = 20);
        ~HardwareSim() override;

        //////////// Override Hardware Interface //////////////////
        long getTimestamp() override;

        int32_t getSlaveNumber() override;

        std::string getSlaveName(int id) override;

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

        inline double getMaxCycleTime() override { return max_cycle_time_; }

        inline double getMinCycleTime() override { return min_cycle_time_; }

        inline double getAvgCycleTime() override { return avg_cycle_time_; }

        inline double getCurrCycleTime() override;

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

        boost::format slave_name_format_ {"Slave_100%1%[ SIM ]"};

        double min_cycle_time_ {30000.0}; //最小循环时间，初值给一个很大的值
        double max_cycle_time_ {0.0};
        double avg_cycle_time_ {1000.0};
        double curr_cycle_time_ {0.0};

        std::default_random_engine generator;
        std::uniform_real_distribution<double> rand_num {-10.0, 10.0};
    };

}

#endif //ROCOS_APP_HARDWARE_SIM_H
