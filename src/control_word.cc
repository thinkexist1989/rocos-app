/*
** Copyright (2019-2020) Robotics Systems Lab - ETH Zurich:
** Jonas Junger, Johannes Pankert, Fabio Dubois, Lennart Nachtigall,
** Markus Staeuble
**
** This file is part of the elmo_ethercat_sdk.
** The elmo_ethercat_sdk is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** The elmo_ethercat_sdk is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with the elmo_ethercat_sdk. If not, see <https://www.gnu.org/licenses/>.
 *
 * Modified: Yang Luo, luoyang@sia.cn
 */

#include <iomanip>

#include <rocos_app/ethercat/control_word.h>

namespace rocos {

    std::ostream &operator<<(std::ostream &os, const Controlword &controlword) {
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
           << "| " << setw(6) << controlword.enableVoltage_ << "|" << setw(6) << " all"
           << "|\n"
           << setw(25) << setfill(' ') << "| quick stop:"
           << "| " << setw(6) << controlword.quickStop_ << "|" << setw(6) << " all"
           << "|\n"
           << setw(25) << setfill(' ') << "| enable operation:"
           << "| " << setw(6) << controlword.enableOperation_ << "|" << setw(6) << " all"
           << "|\n"
           << setw(25) << setfill(' ') << "| new set point:"
           << "| " << setw(6) << controlword.newSetPoint_ << "|" << setw(6) << " pp"
           << "|\n"
           << setw(25) << setfill(' ') << "| start homing:"
           << "| " << setw(6) << controlword.homingOperationStart_ << "|" << setw(6) << " hm"
           << "|\n"
           << setw(25) << setfill(' ') << "| change set:"
           << "| " << setw(6) << controlword.changeSetImmediately_ << "|" << setw(6) << " pp"
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

    uint16_t Controlword::getRawControlword() {
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

    void Controlword::setStateTransition2() {
        setAllFalse();
        enableVoltage_ = true;
        quickStop_ = true;
    }

    void Controlword::setStateTransition3() {
        setAllFalse();
        switchOn_ = true;
        enableVoltage_ = true;
        quickStop_ = true;
    }

    void Controlword::setStateTransition4() {
        setAllFalse();
        switchOn_ = true;
        enableVoltage_ = true;
        quickStop_ = true;
        enableOperation_ = true;
    }

    void Controlword::setStateTransition5() {
        setAllFalse();
        switchOn_ = true;
        enableVoltage_ = true;
        quickStop_ = true;
    }

    void Controlword::setStateTransition6() {
        setAllFalse();
        enableVoltage_ = true;
        quickStop_ = true;
    }

    void Controlword::setStateTransition7() {
        setAllFalse();
    }

    void Controlword::setStateTransition8() {
        setAllFalse();
        enableVoltage_ = true;
        quickStop_ = true;
    }

    void Controlword::setStateTransition9() {
        setAllFalse();
    }

    void Controlword::setStateTransition10() {
        setAllFalse();
    }

    void Controlword::setStateTransition11() {
        setAllFalse();
        enableVoltage_ = true;
    }

    void Controlword::setStateTransition12() {
        setAllFalse();
    }

    void Controlword::setStateTransition15() {
        setAllFalse();
        faultReset_ = true;
    }

    void Controlword::setAllFalse() {
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

    void Controlword::setInit() {
        setStateTransition2();
    }

    void Controlword::setFromRawControlword(uint16_t ctrlwd) {
        switchOn_ = (ctrlwd >> 1) & 0x01;
        enableVoltage_ = (ctrlwd >> 1) & 0x01;
        quickStop_ = (ctrlwd >> 2) & 0x01;
        enableOperation_ = (ctrlwd >> 3) & 0x01;
        // 4, 5, 6 homing,pp specific = false
        faultReset_ = (ctrlwd >> 7) & 0x01;
        halt_ = (ctrlwd >> 8) & 0x01;
    }

}  // namespace elmo
