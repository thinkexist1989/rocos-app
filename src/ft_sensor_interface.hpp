// Copyright 2026, Yang Luo"
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
#pragma once

#include "types.hpp"
#include "result.hpp"

namespace rocos {

    class FTSensorInterface {

    public:
        virtual ~FTSensorInterface() = default;

        virtual Result GetData(Wrench& wrench_out) = 0; //TODO: 直接获取力旋量

        virtual float getFx() const = 0;

        virtual float getFy() const= 0;

        virtual float getFz() const= 0;

        virtual float getMx() const= 0;

        virtual float getMy() const= 0;

        virtual float getMz() const= 0;


    };

} // rocos
