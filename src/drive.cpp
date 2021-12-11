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

#include <drive.h>


namespace rocos {

    Drive::Drive(HardwareInterface *hw, int id) : _hw_interface(hw),
                                                  _id(id)
    {

    }

    bool Drive::setDriverState(const DriveState &driveState, bool waitForState) {
        bool success = false;
        /*
        ** locking the mutex_
        ** This is not done with a lock_guard here because during the waiting time the
        ** mutex_ must be unlocked periodically such that PDO writing (and thus state
        ** changes) may occur at all!
        */
//        _mutex.lock();

        // reset the "stateChangeSuccessful_" flag to false such that a new successful
        // state change can be detected
        _stateChangeSuccessful = false;

        // make the state machine realize that a state change will have to happen
        _conductStateChange = true;

        // overwrite the target drive state
        _targetDriveState = driveState;

        // set the hasRead flag to false such that at least one new reading will be
        // available when starting the state change
//        hasRead_ = false;

        // set the time point of the last pdo change to now
        _driveStateChangeTimePoint = boost::chrono::system_clock::now();

        // set a temporary time point to prevent getting caught in an infinite loop
        auto driveStateChangeStartTimePoint = boost::chrono::system_clock::now();

        // return if no waiting is requested
        if (!waitForState) {
            // unlock the mutex
//            mutex_.unlock();
            // return true if no waiting is requested
            return true;
        }

        // Wait for the state change to be successful
        // during the waiting time the mutex MUST be unlocked!

        while (true) {
            // break loop as soon as the state change was successful
            if (_stateChangeSuccessful) {
                success = true;
                break;
            }

            // break the loop if the state change takes too long
            // this prevents a freezing of the end user's program if the hardware is not
            // able to change it's state.
            if ((boost::chrono::duration_cast<boost::chrono::microseconds>(
                    boost::chrono::system_clock::now() - driveStateChangeStartTimePoint)).count() >
                10000) { // TODO: configuration_.driveStateChangeMaxTimeout
                break;
            }
            // unlock the mutex during sleep time
//            _mutex.unlock();
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
            // lock the mutex to be able to check the success flag
//            _mutex.lock();
        }
        // unlock the mutex one last time
//        _mutex.unlock();
        return success;
    }

    Controlword Drive::getNextStateTransitionControlword(const DriveState &requestedDriveState,
                                                         const DriveState &currentDriveState) {
        Controlword controlword;
        controlword.setAllFalse();
        switch (requestedDriveState) {
            case DriveState::SwitchOnDisabled:
                switch (currentDriveState) {
                    case DriveState::SwitchOnDisabled:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "drive state has already been reached for '" << _name << "'" << std::endl;
//                        MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::getNextStateTransitionControlword] "
//                                                  << "drive state has already been reached for '"
//                                                  << name_ << "'");
//                        addErrorToReading(ErrorType::PdoStateTransitionError);
                        break;
                    case DriveState::ReadyToSwitchOn:
                        controlword.setStateTransition7();
                        break;
                    case DriveState::SwitchedOn:
                        controlword.setStateTransition10();
                        break;
                    case DriveState::OperationEnabled:
                        controlword.setStateTransition9();
                        break;
                    case DriveState::QuickStopActive:
                        controlword.setStateTransition12();
                        break;
                    case DriveState::Fault:
                        controlword.setStateTransition15();
                        break;
                    default:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "PDO state transition not implemented for '" << _name << "'" << std::endl;
//                        MELO_ERROR_STREAM("[elmo_ethercat_sdk:Elmo::getNextStateTransitionControlword] "
//                                                  <<
//                                                  << name_ << "'");
//                        addErrorToReading(ErrorType::PdoStateTransitionError);
                }
                break;

            case DriveState::ReadyToSwitchOn:
                switch (currentDriveState) {
                    case DriveState::SwitchOnDisabled:
                        controlword.setStateTransition2();
                        break;
                    case DriveState::ReadyToSwitchOn:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "drive state has already been reached for '" << _name << "'" << std::endl;
                        break;
                    case DriveState::SwitchedOn:
                        controlword.setStateTransition6();
                        break;
                    case DriveState::OperationEnabled:
                        controlword.setStateTransition8();
                        break;
                    case DriveState::QuickStopActive:
                        controlword.setStateTransition12();
                        break;
                    case DriveState::Fault:
                        controlword.setStateTransition15();
                        break;
                    default:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "PDO state transition not implemented for '" << _name << "'" << std::endl;

                }
                break;

            case DriveState::SwitchedOn:
                switch (currentDriveState) {
                    case DriveState::SwitchOnDisabled:
                        controlword.setStateTransition2();
                        break;
                    case DriveState::ReadyToSwitchOn:
                        controlword.setStateTransition3();
                        break;
                    case DriveState::SwitchedOn:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "drive state has already been reached for '" << _name << "'" << std::endl;
                        break;
                    case DriveState::OperationEnabled:
                        controlword.setStateTransition5();
                        break;
                    case DriveState::QuickStopActive:
                        controlword.setStateTransition12();
                        break;
                    case DriveState::Fault:
                        controlword.setStateTransition15();
                        break;
                    default:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "PDO state transition not implemented for '" << _name << "'" << std::endl;

                }
                break;

            case DriveState::OperationEnabled:
                switch (currentDriveState) {
                    case DriveState::SwitchOnDisabled:
                        controlword.setStateTransition2();
                        break;
                    case DriveState::ReadyToSwitchOn:
                        controlword.setStateTransition3();
                        break;
                    case DriveState::SwitchedOn:
                        controlword.setStateTransition4();
                        break;
                    case DriveState::OperationEnabled:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "drive state has already been reached for '" << _name << "'" << std::endl;
                        break;
                    case DriveState::QuickStopActive:
                        controlword.setStateTransition12();
                        break;
                    case DriveState::Fault:
                        controlword.setStateTransition15();
                        break;
                    default:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "PDO state transition not implemented for '" << _name << "'" << std::endl;

                }
                break;

            case DriveState::QuickStopActive:
                switch (currentDriveState) {
                    case DriveState::SwitchOnDisabled:
                        controlword.setStateTransition2();
                        break;
                    case DriveState::ReadyToSwitchOn:
                        controlword.setStateTransition3();
                        break;
                    case DriveState::SwitchedOn:
                        controlword.setStateTransition4();
                        break;
                    case DriveState::OperationEnabled:
                        controlword.setStateTransition11();
                        break;
                    case DriveState::QuickStopActive:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "drive state has already been reached for '" << _name << "'" << std::endl;
                        break;
                    case DriveState::Fault:
                        controlword.setStateTransition15();
                        break;
                    default:
                        std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                                  << "PDO state transition not implemented for '" << _name << "'" << std::endl;

                }
                break;

            default:
                std::cerr << "[rocos::Drive::getNextStateTransitionControlword] "
                          << "PDO state cannot be reached for '" << _name << "'" << std::endl;

        }

        return controlword;
    }

    void Drive::engageStateMachine() {
// locking the mutex
        boost::lock_guard<boost::recursive_mutex> lock(_mutex);

        // elapsed time since the last new controlword
        auto microsecondsSinceChange =
                (boost::chrono::duration_cast<boost::chrono::microseconds>(boost::chrono::system_clock::now() - _driveStateChangeTimePoint)).count();

        // get the current state
        // since we wait until "hasRead" is true, this is guaranteed to be a newly
        // read value
//        const DriveState currentDriveState = reading_.getDriveState();

        // check if the state change already was successful:
        if (_currentDriveState == _targetDriveState) {
            _numberOfSuccessfulTargetStateReadings++;
//            if (numberOfSuccessfulTargetStateReadings_ >= configuration_.minNumberOfSuccessfulTargetStateReadings) {
            if (_numberOfSuccessfulTargetStateReadings >= 3) {
                // disable the state machine
                _conductStateChange = false;
                _numberOfSuccessfulTargetStateReadings = 0;
                _stateChangeSuccessful = true;
                return;
            }
//        } else if (microsecondsSinceChange > configuration_.driveStateChangeMinTimeout) {
        } else if (microsecondsSinceChange > 20000) {
            // get the next controlword from the state machine
            _controlword = getNextStateTransitionControlword(_targetDriveState, _currentDriveState);
            _driveStateChangeTimePoint = boost::chrono::system_clock::now();
        }

        // set the "hasRead" variable to false such that there will definitely be a
        // new reading when this method is called again
//        hasRead_ = false;
    }

}