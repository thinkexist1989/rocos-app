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
#include <boost/thread.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/recursive_mutex.hpp>

namespace rocos {

    class Drive {

    public:
        Drive(HardwareInterface* hw, int id);

        virtual bool setDriverState(const DriveState &driveState, bool waitForState);

    protected:
        std::string _name {};
        Statusword _statusword {};
        Controlword _controlword {};
        DriveState _currentDriveState {};
        DriveState _targetDriveState {};

        bool _conductStateChange {false};
        std::atomic<bool> _stateChangeSuccessful {false};

        Timestamp _driveStateChangeTimePoint;

        uint16_t _numberOfSuccessfulTargetStateReadings {0};

        mutable boost::recursive_mutex _mutex; // TODO: change name!!!!

    protected:

        void engageStateMachine();

        /// Get next control word according to the requested drive state and current drive state
        /// \param requestedDriveState
        /// \param currentDriveState
        /// \return the controlword to be sent
        virtual Controlword getNextStateTransitionControlword(const DriveState &requestedDriveState,
                                                              const DriveState &currentDriveState);

    protected:

        boost::shared_ptr<HardwareInterface> _hw_interface {nullptr}; // The pointer of HardwareInterface instance
        int _id {0}; // drive id in bus
        double _reduction_ratio {1.0}; // reduction ratio
        double _minPosLimit {-2.0};
        double _maxPosLimit {2.0};

    };
}


#endif //ROCOS_APP_DRIVE_H
