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

namespace rocos {
    class HardwareInterface {
        virtual ~HardwareInterface();

        virtual bool setTargetPositionRaw(int id, int pos);

        virtual bool setTargetVelocityRaw(int id, int vel);

        virtual bool setTargetTorqueRaw(int id, int tor);

        virtual int getTargetPositionRaw(int id);

        virtual int getTargetVelocityRaw(int id);

        virtual int getTargetTorqueRaw(int id);

        virtual int getActualPositionRaw(int id);

        virtual int getActualVelocityRaw(int id);

        virtual int getActualTorqueRaw(int id);
    };
}


#endif //ROCOS_APP_HARDWARE_INTERFACE_H
