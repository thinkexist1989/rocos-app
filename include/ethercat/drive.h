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

#ifndef ROCOS_APP_DRIVE_H
#define ROCOS_APP_DRIVE_H

#include <ethercat/hardware_interface.h>

#include <boost/shared_ptr.hpp>

namespace rocos {

    class Drive {

    public:

        virtual bool setDriverState(const DriveState &driveState, bool waitForState);

    protected:
        std::string _name;
        Statusword _statusword;
        Controlword _controlword;
        DriveState _currentDriveState;
        DriveState _targetDriveState;

        bool _conductStateChange;
        std::atomic<bool> _stateChangeSuccessful;

        Timestamp _driveStateChangeTimePoint;


        mutable std::recursive_mutex _mutex; // TODO: change name!!!!

    protected:
        boost::shared_ptr<HardwareInterface> _hw_interface; // The pointer of HardwareInterface instance

        virtual Controlword getNextStateTransitionControlword(const DriveState &requestedDriveState,
                                                              const DriveState &currentDriveState);
    };
}


#endif //ROCOS_APP_DRIVE_H
