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

#include "robot.h"

namespace rocos {

    Robot::Robot(boost::shared_ptr<HardwareInterface> hw) : _hw_interface(hw) {
        addAllJoints();
    }

    void Robot::addAllJoints() {
        _jntNum = _hw_interface->getSlaveNumber();
        _joints.clear();

        for (int i = 0; i < _jntNum; i++) {
            _joints.push_back(boost::make_shared<Drive>(_hw_interface, i));
            _joints[i]->setMode(ModeOfOperation::CyclicSynchronousPositionMode);
        }
    }

    void
    Robot::moveJ(const std::vector<double> &pos, const std::vector<double> &max_vel, const std::vector<double> &max_acc,
                 const std::vector<double> &max_jerk, Robot::Synchronization sync, ProfileType type) {

        if (pos.size() != _jntNum || max_vel.size() != _jntNum || max_acc.size() != _jntNum ||
            max_jerk.size() != _jntNum) {
            std::cout << "[ERROR] MoveJ => Error Input Vector Size!" << std::endl;
            return;
        }

        std::vector<R_INTERP_BASE *> interp(_jntNum);

        double max_time = 0.0;

        // Start trajectory generation....
        for (int i = 0; i < _jntNum; i++) {
            auto p0 = _joints[i]->getPosition();
            switch (type) {
                case trapezoid:
                    interp[i] = new Trapezoid;
                    reinterpret_cast<Trapezoid *>(interp[i])->planTrapezoidProfile(0, p0, pos[i], 0, 0, max_vel[i],
                                                                                   max_acc[i]);
                    break;
                case doubleS:
                    interp[i] = new DoubleS;
                    reinterpret_cast<DoubleS *>(interp[i])->planDoubleSProfile(0, p0, pos[i], 0, 0, max_vel[i],
                                                                               max_acc[i], max_jerk[i]);
                    break;
                default:
                    std::cout << "Not Supported Profile Type" << std::endl;
                    return;
            }

            max_time = max(max_time, interp[i]->getDuration());
        }

        // Sync scaling....
        if (sync == SYNC_TIME) {
            for_each(interp.begin(), interp.end(), [=](R_INTERP_BASE *p) { p->scaleToDuration(max_time); });
        } else if (sync == SYNC_PHASE) {
            std::cout << "[WARNING] Phase sync has not implemented...instead of time sync." << std::endl;
            for_each(interp.begin(), interp.end(), [=](R_INTERP_BASE *p) { p->scaleToDuration(max_time); });
        }

        // Start moving....
        double dt = 0.0;
        while (dt <= max_time) {
            _hw_interface->waitForSignal(9);

            for (int i = 0; i < _jntNum; i++) {
                if (!interp[i]->isValidMovement()) {
                    continue;
                }
                switch (_joints[i]->getMode()) {
                    case ModeOfOperation::CyclicSynchronousPositionMode:
                        _joints[i]->setPosition(interp[i]->pos(dt));
                        break;
                    case ModeOfOperation::CyclicSynchronousVelocityMode:
                        _joints[i]->setVelocity(interp[i]->vel(dt));
                        break;
                    default:
                        std::cout << "Only Supported CSP and CSV" << std::endl;
                }
            }

            dt += 0.001;
        }

        // delete pointer
        for (auto &p: interp) {
            delete p;
        }
    }

    void Robot::setEnabled() {
        for_each(_joints.begin(), _joints.end(), [=](boost::shared_ptr<Drive>& d) { d->setEnabled(); });
    }

    void Robot::setDisabled() {
        for_each(_joints.begin(), _joints.end(), [=](boost::shared_ptr<Drive>& d) { d->setDisabled(); });
    }
}