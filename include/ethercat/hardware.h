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

#include <ethercat/hardware_interface.h>

namespace rocos {

    class Hardware : HardwareInterface {
    public:
        ~Hardware() override;
    };

}


#endif //ROCOS_APP_HARDWARE_H
