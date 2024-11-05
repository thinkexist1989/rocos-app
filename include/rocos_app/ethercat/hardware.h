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

#include "command.h"
#include "control_word.h"
#include "drive_state.h"
#include "status_word.h"
#include "mode_of_operation.h"

#include <rocos_ecm/ecat_config.h>

#include "../hardware_interface.h"


namespace rocos {

    class Hardware : public HardwareInterface {
    public:
        explicit Hardware(const std::string &urdf_file_path = "robot.urdf", int id = 0);

        ~Hardware() override;

        long getTimestamp() override;

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

        int32_t getDigitalInputsRaw(int id) override;

        int32_t getSecondaryPositionRaw(int id) override;

        int32_t getSecondaryVelocityRaw(int id) override;

        uint16_t getStatuswordRaw(int id) override;

        std::string getSlaveName(int id) override;

        int32_t getSlaveNumber() override;

        void setControlwordRaw(int id, uint16_t ctrlwd) override;

        void setModeOfOperationRaw(int id, int8_t mode) override;

        void setHardwareState(HWState state) override;

        HWState getHardwareState() override;

        void waitForSignal(int id) override;

        void wait() override;

        void parseParamFormUrdf(const std::string &urdf_file_path);

        void setDigitalOutputsRaw(int id, int32_t value) override;


    protected:
        EcatConfig* ecPtr_ {nullptr};

        std::vector<int32_t *> pTargetPos_;    // Target Position
        std::vector<int32_t *> pTargetVel_;    // Target Velocity
        std::vector<int16_t *> pTargetTor_;    // Target Torque
        std::vector<int32_t *> pActualPos_;    // Actual Position Value
        std::vector<int32_t *> pActualVel_;    // Actual Velocity Value
        std::vector<int16_t *> pActualTor_;    // Actual Torque Value
        std::vector<int16_t *> pLoadTor_;      // Load Torque Value
        std::vector<int32_t *> pSecondaryPos_; // Secondary Position Value
        std::vector<int32_t *> pSecondaryVel_; // Secondary Velocity Value
        std::vector<uint16_t *> pStatusword_;  // Statusword
        std::vector<uint16_t *> pControlword_; // Controlword
        std::vector<int8_t *> pModeOfOp_;      // Modes of operation

        std::vector<int32_t *> pDigitalInputs_; // Digital Inputs
        std::vector<int32_t *> pDigitalOutputs_; // Digital Outputs

        int slave_num_ {0};


    };

}


#endif //ROCOS_APP_HARDWARE_H
