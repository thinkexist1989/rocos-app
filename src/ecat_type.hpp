/*
Copyright 2021, Yang Luo"
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

@Author
Yang Luo, PHD
@email: yluo@hit.edu.cn

@Created on: 2021.11.29
@Last Modified: 2023.3.28 22:03
*/

/*-----------------------------------------------------------------------------
 * ecat_type.h
 * Description              EtherCAT Type Definitions
 *
 *---------------------------------------------------------------------------*/
#pragma once

#include <semaphore.h>  //sem

#include <cinttypes>
#include <iomanip>

#define MAX_SLAVE_NUM 50     // Maximal number of slaves in the EtherCAT network
#define MAX_PDINPUT_NUM 25   // Maximal number of PD Inputs per slave
#define MAX_PDOUTPUT_NUM 25  // Maximal number of PD Outputs per slave

#define MAX_PD_NAME_LEN 72     // Maximal length of a PD Variable name
#define MAX_SLAVE_NAME_LEN 80  // Maximal length of a slave name

#define EC_SEM_MUTEX "sync"
#define EC_SEM_NUM 10

#define EC_SHM "ecm"
#define EC_SHM_MAX_SIZE 5242880  // 5MB

#define ECAT_STATE_INIT 1
#define ECAT_STATE_PREOP 2
#define ECAT_STATE_SAFEOP 4
#define ECAT_STATE_OP 8
#define ECAT_STATE_BOOTSTRAP 3

namespace rocos {

/*!
 * An enum containing all the possible Error types.
 * Note that Errors and Faults are not the same thing.
 * Errors occur during setup, configuration and SDO reading / writing
 * Faults occur during PDO communication when the drive state jumps to "FAULT".
 */
enum class ErrorType : int {
  ConfigurationError,
  SdoWriteError,
  SdoReadError,
  ErrorReadingError,
  SdoStateTransitionError,
  PdoMappingError,
  RxPdoMappingError,
  TxPdoMappingError,
  RxPdoTypeError,
  TxPdoTypeError,
  PdoStateTransitionError,
  ModeOfOperationError
};

enum class ModeOfOperation : int8_t {
  NA = 0,
  ProfiledPositionMode = 1,
  ProfiledVelocityMode = 3,
  ProfiledTorqueMode = 4,
  HomingMode = 6,
  CyclicSynchronousPositionMode = 8,
  CyclicSynchronousVelocityMode = 9,
  CyclicSynchronousTorqueMode = 10
};

enum class DriveState : uint8_t {
  NotReadyToSwitchOn,
  SwitchOnDisabled,
  ReadyToSwitchOn,
  SwitchedOn,
  OperationEnabled,
  QuickStopActive,
  FaultReactionActive,
  Fault,
  NA
};

enum class StateTransition : uint8_t {
  _2,
  _3,
  _4,
  _5,
  _6,
  _7,
  _8,
  _9,
  _10,
  _11,
  _12,
  _15
};

inline std::ostream& operator<<(std::ostream& os,
                         const rocos::DriveState& driveState) {
  switch (driveState) {
    case rocos::DriveState::NotReadyToSwitchOn:
      os << "NotReadyToSwitchOn";
      break;
    case rocos::DriveState::SwitchOnDisabled:
      os << "SwitchOnDisabled";
      break;
    case rocos::DriveState::ReadyToSwitchOn:
      os << "ReadyToSwitchOn";
      break;
    case rocos::DriveState::SwitchedOn:
      os << "SwitchedOn";
      break;
    case rocos::DriveState::OperationEnabled:
      os << "OperationEnabled";
      break;
    case rocos::DriveState::QuickStopActive:
      os << "QuickStopActive";
      break;
    case rocos::DriveState::FaultReactionActive:
      os << "FaultReactionActive";
      break;
    case rocos::DriveState::Fault:
      os << "Fault";
      break;
    case rocos::DriveState::NA:
      os << "NA";
      break;
  }
  return os;
}

struct Statusword {
 private:
  bool readyToSwitchOn_{false};      // bit 0
  bool switchedOn_{false};           // bit 1
  bool operationEnabled_{false};     // bit 2
  bool fault_{false};                // bit 3
  bool voltageEnabled_{false};       // bit 4
  bool quickStop_{false};            // bit 5
  bool switchOnDisabled_{false};     // bit 6
  bool warning_{false};              // bit 7
  bool targetReached_{false};        // bit 10
  bool internalLimitActive_{false};  // bit 11
  bool followingError_{false};       // bit 13, CSV mode

  // the raw statusword
  uint16_t rawStatusword_{0};

 public:
  friend std::ostream& operator<<(std::ostream& os,
                                  const Statusword& statusword);

  void setFromRawStatusword(uint16_t status) {
    readyToSwitchOn_ = static_cast<bool>(status & 1 << (0));
    switchedOn_ = static_cast<bool>(status & 1 << (1));
    operationEnabled_ = static_cast<bool>(status & 1 << (2));
    fault_ = static_cast<bool>(status & 1 << (3));
    voltageEnabled_ = static_cast<bool>(status & 1 << (4));
    quickStop_ = static_cast<bool>(status & 1 << (5));
    switchOnDisabled_ = static_cast<bool>(status & 1 << (6));
    warning_ = static_cast<bool>(status & 1 << (7));
    targetReached_ = static_cast<bool>(status & 1 << (10));
    internalLimitActive_ = static_cast<bool>(status & 1 << (11));
    followingError_ = static_cast<bool>(status & 1 << (13));

    rawStatusword_ = status;
  }

  uint16_t getRawStatusword() { return rawStatusword_; }

  DriveState getDriveState() const {
    DriveState driveState = DriveState::NA;

    // MAN-G-DS402 manual page 47
    if ((rawStatusword_ & 0b0000000001001111) == 0b0000000000000000) {
      driveState = DriveState::NotReadyToSwitchOn;
    } else if ((rawStatusword_ & 0b0000000001001111) == 0b0000000001000000) {
      driveState = DriveState::SwitchOnDisabled;
    } else if ((rawStatusword_ & 0b0000000001101111) == 0b0000000000100001) {
      driveState = DriveState::ReadyToSwitchOn;
    } else if ((rawStatusword_ & 0b0000000001101111) == 0b0000000000100011) {
      driveState = DriveState::SwitchedOn;
    } else if ((rawStatusword_ & 0b0000000001101111) == 0b0000000000100111) {
      driveState = DriveState::OperationEnabled;
    } else if ((rawStatusword_ & 0b0000000001101111) == 0b0000000000000111) {
      driveState = DriveState::QuickStopActive;
    } else if ((rawStatusword_ & 0b0000000001001111) == 0b0000000000001111) {
      driveState = DriveState::FaultReactionActive;
    } else if ((rawStatusword_ & 0b0000000001001111) == 0b0000000000001000) {
      driveState = DriveState::Fault;
    } else {
      driveState = DriveState::Fault;
    }

    return driveState;
  }

  std::string getDriveStateString() const {
    DriveState driveState = getDriveState();
    switch (driveState) {
      case DriveState::SwitchOnDisabled:
        return "switch on disabled";
        break;
      case DriveState::ReadyToSwitchOn:
        return "ready to switch on";
        break;
      case DriveState::SwitchedOn:
        return "switched on";
        break;
      case DriveState::OperationEnabled:
        return "operation enabled";
        break;
      case DriveState::QuickStopActive:
        return "quick stop active";
        break;
      case DriveState::Fault:
        return "fault_";
        break;
      case DriveState::FaultReactionActive:
        return "fault_ reaction active";
      case DriveState::NotReadyToSwitchOn:
        return "not ready to switch on";
      default:
        return "N/A";
    }
  }
};

inline std::ostream& operator<<(std::ostream& os, const Statusword& statusword) {
  using std::setfill;
  using std::setw;
  std::string driveStateString = statusword.getDriveStateString();
  int gapSize2 = driveStateString.size() + 1;
  if (gapSize2 < 6) {
    gapSize2 = 6;
  }
  os << std::left << std::boolalpha << setw(gapSize2 + 27) << setfill('-')
     << "|"
     << "|\n"
     << setw(gapSize2 + 27) << setfill(' ') << "| Statusword"
     << "|\n"
     << setw(gapSize2 + 27) << setfill('-') << "|"
     << "|\n"
     << setw(25) << setfill(' ') << "| Name of Bit" << setw(gapSize2 + 2)
     << "| Value"
     << "|\n"
     << setw(25) << setfill('-') << "|" << setw(gapSize2 + 2) << "+"
     << "|\n"
     << setfill(' ') <<

      setw(25) << "| Ready to switch on:"
     << "| " << setw(gapSize2) << statusword.readyToSwitchOn_ << "|\n"
     << setw(25) << "| Switched on:"
     << "| " << setw(gapSize2) << statusword.switchedOn_ << "|\n"
     << setw(25) << "| Operation enabled:"
     << "| " << setw(gapSize2) << statusword.operationEnabled_ << "|\n"
     << setw(25) << "| Fault:"
     << "| " << setw(gapSize2) << statusword.fault_ << "|\n"
     << setw(25) << "| Voltage enabled:"
     << "| " << setw(gapSize2) << statusword.voltageEnabled_ << "|\n"
     << setw(25) << "| Quick stop:"
     << "| " << setw(gapSize2) << statusword.quickStop_ << "|\n"
     << setw(25) << "| Switch on disabled:"
     << "| " << setw(gapSize2) << statusword.switchOnDisabled_ << "|\n"
     << setw(25) << "| Warning:"
     << "| " << setw(gapSize2) << statusword.warning_ << "|\n"
     << setw(25) << "| Target reached:"
     << "| " << setw(gapSize2) << statusword.targetReached_ << "|\n"
     << setw(25) << "| Internal limit active:"
     << "| " << setw(gapSize2) << statusword.internalLimitActive_ << "|\n"
     <<
      // setw(25)<<"| Following error:"<<"|
      // "<<setw(gapSize2)<<statusword.followingError_<<"| \n"<< // mode of
      // operation specific
      setw(25) << setfill('-') << "|" << setw(gapSize2 + 2) << "+"
     << "|\n"
     << setfill(' ') << setw(25) << "| Resulting Drive State:"
     << "| " << setw(gapSize2) << driveStateString << "|\n"
     << setw(25) << setfill('-') << "|" << setw(gapSize2 + 2) << "+"
     << "|" <<

      std::noboolalpha << std::right << setfill(' ');

  return os;
}

struct Controlword {
  bool switchOn_{false};              // bit 0
  bool enableVoltage_{false};         // bit 1
  bool quickStop_{false};             // bit 2
  bool enableOperation_{false};       // bit 3
  bool newSetPoint_{false};           // bit 4 profiled position mode
  bool homingOperationStart_{false};  // bit 4 homing mode
  bool changeSetImmediately_{false};  // bit 5 profiled position mode
  bool relative_{false};              // bit 6 profiled position mode
  bool faultReset_{false};            // bit 7
  bool halt_{false};                  // bit 8

  void setFromRawControlword(uint16_t ctrlwd) {
    switchOn_ = (ctrlwd >> 1) & 0x01;
    enableVoltage_ = (ctrlwd >> 1) & 0x01;
    quickStop_ = (ctrlwd >> 2) & 0x01;
    enableOperation_ = (ctrlwd >> 3) & 0x01;
    // 4, 5, 6 homing,pp specific = false
    faultReset_ = (ctrlwd >> 7) & 0x01;
    halt_ = (ctrlwd >> 8) & 0x01;
  }
  /*!
   * get the control word as a 16 bit unsigned integer
   * THIS DOES NOT RESPECT THE MODE SPECIFIC OPTIONS!
   * The usually used cyclic modes do not need mode specific options.
   * @return	the raw controlword
   */
  uint16_t getRawControlword() {
    uint16_t rawControlword = 0;

    if (switchOn_) {
      rawControlword |= (1 << 0);
    }
    if (enableVoltage_) {
      rawControlword |= (1 << 1);
    }
    if (quickStop_) {
      rawControlword |= (1 << 2);
    }
    if (enableOperation_) {
      rawControlword |= (1 << 3);
    }
    if (faultReset_) {
      rawControlword |= (1 << 7);
    }
    if (halt_) {
      rawControlword |= (1 << 8);
    }

    return rawControlword;
  }

  /*!
   * State transition 2
   * SWITCH ON DISABLED -> READY TO SWITCH ON
   * This corresponds to a "shutdown" Controlword
   */
  void setStateTransition2() {
    setAllFalse();
    enableVoltage_ = true;
    quickStop_ = true;
  }

  /*!
   * State transition 3
   * READY TO SWITCH ON -> SWITCHED ON
   * This corresponds to a "switch on" Controlword
   */
  void setStateTransition3() {
    setAllFalse();
    switchOn_ = true;
    enableVoltage_ = true;
    quickStop_ = true;
  }

  /*!
   * State transition 4
   * SWITCHED ON -> ENABLE OPERATION
   */
  void setStateTransition4() {
    setAllFalse();
    switchOn_ = true;
    enableVoltage_ = true;
    quickStop_ = true;
    enableOperation_ = true;
  }

  /*!
   * State transition 5
   * OPERATION ENABLED -> SWITCHED ON
   * This corresponds to a "disable operation" Controlword
   */
  void setStateTransition5() {
    setAllFalse();
    switchOn_ = true;
    enableVoltage_ = true;
    quickStop_ = true;
  }

  /*!
   * State transition 6
   * SWITCHED ON -> READY TO SWITCH ON
   */
  void setStateTransition6() {
    setAllFalse();
    enableVoltage_ = true;
    quickStop_ = true;
  }

  /*!
   * State transition 7
   * READY TO SWITCH ON -> SWITCH ON DISABLED
   */
  void setStateTransition7() { setAllFalse(); }

  /*!
   * State transition 8
   * OPERATION ENABLED -> READY TO SWITCH ON
   */
  void setStateTransition8() {
    setAllFalse();
    enableVoltage_ = true;
    quickStop_ = true;
  }

  /*!
   * State transition 9
   * OPERATION ENABLED -> SWITCH ON DISABLED
   * This resets the elmo to the same state as on hardware startup
   * 0x0000
   */
  void setStateTransition9() { setAllFalse(); }

  /*!
   * State transition 10
   * SWITCHED ON -> SWITCH ON DISABLED
   * This Statusword is 0x0000
   */
  void setStateTransition10() { setAllFalse(); }

  /*!
   * State transition 11
   * OPERATION ENABLED -> QUICK STOP ACTIVE
   */
  void setStateTransition11() {
    setAllFalse();
    enableVoltage_ = true;
  }

  /*!
   * State transition 12
   * QUICK STOP ACTIVE -> SWITCH ON DISABLED
   */
  void setStateTransition12() { setAllFalse(); }

  /*!
   * State transition 15
   * FAULT -> SWITCH ON DISABLED
   */
  void setStateTransition15() {
    setAllFalse();
    faultReset_ = true;
  }

  /*!
   * Sets all bools of this struct to false
   */
  void setAllFalse() {
    switchOn_ = false;
    enableVoltage_ = false;
    quickStop_ = false;
    enableOperation_ = false;
    newSetPoint_ = false;
    homingOperationStart_ = false;
    changeSetImmediately_ = false;
    relative_ = false;
    faultReset_ = false;
    halt_ = false;
  }

  /*!
   * goes to the init state
   * Alias for state transition 2
   */
  void setInit() { setStateTransition2(); }

  friend std::ostream& operator<<(std::ostream& os,
                                  const Controlword& controlword);
};

inline std::ostream& operator<<(std::ostream& os, const Controlword& controlword) {
  using std::setfill;
  using std::setw;

  os << std::left << std::boolalpha << setw(40) << setfill('-') << "|"
     << "|\n"
     << setw(40) << setfill(' ') << "| Controlword"
     << "|\n"
     << setw(25) << setfill('-') << "|" << setw(8) << "+" << setw(7) << "+"
     << "|"
     << "\n"
     << setw(25) << setfill(' ') << "| Name"
     << "| Value | Mode |"
     << "\n"
     << setw(25) << setfill('-') << "|" << setw(8) << "+" << setw(7) << "+"
     << "|"
     << "\n"
     << setw(25) << setfill(' ') << "| switch on:"
     << "| " << setw(6) << controlword.switchOn_ << "|" << setw(6) << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| enable voltage:"
     << "| " << setw(6) << controlword.enableVoltage_ << "|" << setw(6)
     << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| quick stop:"
     << "| " << setw(6) << controlword.quickStop_ << "|" << setw(6) << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| enable operation:"
     << "| " << setw(6) << controlword.enableOperation_ << "|" << setw(6)
     << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| new set point:"
     << "| " << setw(6) << controlword.newSetPoint_ << "|" << setw(6) << " pp"
     << "|\n"
     << setw(25) << setfill(' ') << "| start homing:"
     << "| " << setw(6) << controlword.homingOperationStart_ << "|" << setw(6)
     << " hm"
     << "|\n"
     << setw(25) << setfill(' ') << "| change set:"
     << "| " << setw(6) << controlword.changeSetImmediately_ << "|" << setw(6)
     << " pp"
     << "|\n"
     << setw(25) << setfill(' ') << "| relative_:"
     << "| " << setw(6) << controlword.relative_ << "|" << setw(6) << " pp "
     << "|\n"
     << setw(25) << setfill(' ') << "| fault_ reset:"
     << "| " << setw(6) << controlword.faultReset_ << "|" << setw(6) << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| halt_:"
     << "| " << setw(6) << controlword.halt_ << "|" << setw(6) << " all"
     << "|\n"
     <<

      setw(25) << setfill('-') << "|" << setw(8) << "+" << setw(7) << "+"
     << "|" << std::right << std::noboolalpha;
  return os;
}

}  // namespace rocos