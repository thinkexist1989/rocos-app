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

        _targetPositions.resize(_jntNum);
        _targetPositionsPrev.resize(_jntNum);
        _targetVelocities.resize(_jntNum);
        _targetTorques.resize(_jntNum);
        _pos.resize(_jntNum);
        _vel.resize(_jntNum);
        _acc.resize(_jntNum);
        _max_vel.resize(_jntNum);
        _max_acc.resize(_jntNum);
        _max_jerk.resize(_jntNum);
        _interp.resize(_jntNum);

        for (int i = 0; i < _jntNum; ++i) {
            _pos[i] = _joints[i]->getPosition();
            _targetPositions[i] = _pos[i];
            _targetPositionsPrev[i] = _pos[i];

            _vel[i] = _joints[i]->getVelocity();
            _targetVelocities[i] = _vel[i];

            _targetTorques[i] = _joints[i]->getTorque();

            _max_vel[i] = _joints[i]->getMaxVel();
            _max_acc[i] = _joints[i]->getMaxAcc();
            _max_jerk[i] = _joints[i]->getMaxJerk();
        }

        startMotionThread();

    }

    void Robot::addAllJoints() {
        _jntNum = _hw_interface->getSlaveNumber();
        _joints.clear();
        for (int i = 0; i < _jntNum; i++) {
            _joints.push_back(boost::make_shared<Drive>(_hw_interface, i));
            _joints[i]->setMode(ModeOfOperation::CyclicSynchronousPositionMode);
        }
    }

    //TODO: 切换HW指针
    bool Robot::switchHW(boost::shared_ptr<HardwareInterface> hw) {
        return false;
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
        for_each(_joints.begin(), _joints.end(), [=](boost::shared_ptr<Drive> &d) { d->setEnabled(); });
    }

    void Robot::setDisabled() {
        for_each(_joints.begin(), _joints.end(), [=](boost::shared_ptr<Drive> &d) { d->setDisabled(); });
    }

    void Robot::startMotionThread() {
        _isRunning = true;
        boost::thread(&Robot::motionThreadHandler, this).detach();
    }

    void Robot::stopMotionThread() {
        _isRunning = false;
    }

    void Robot::motionThreadHandler() {

        std::cout << "Motion thread is running on thread " << boost::this_thread::get_id() << std::endl;

        _targetPositions.resize(_jntNum);
        _targetPositionsPrev.resize(_jntNum);
        _targetVelocities.resize(_jntNum);
        _targetTorques.resize(_jntNum);
        _pos.resize(_jntNum);
        _vel.resize(_jntNum);
        _acc.resize(_jntNum);
        _max_vel.resize(_jntNum);
        _max_acc.resize(_jntNum);
        _max_jerk.resize(_jntNum);
        _interp.resize(_jntNum);
        _needPlan.resize(_jntNum, false);

        for (int i = 0; i < _jntNum; ++i) {
            _pos[i] = _joints[i]->getPosition();
            _targetPositions[i] = _pos[i];
            _targetPositionsPrev[i] = _pos[i];

            _vel[i] = _joints[i]->getVelocity();
            _targetVelocities[i] = _vel[i];

            _targetTorques[i] = _joints[i]->getTorque();

            if (_profileType == trapezoid) {
                _interp[i] = new Trapezoid;
            } else if (_profileType == doubleS) {
                _interp[i] = new DoubleS;
            }

        }

        std::vector<double> dt(_jntNum, 0.0); // delta T
        double max_time = 0.0;

        while (_isRunning) { // while start

            _hw_interface->waitForSignal(9);

            // Trajectory generating......
            max_time = 0.0;
            for (int i = 0; i < _jntNum; ++i) {
                if (_joints[i]->getDriveState() != DriveState::OperationEnabled) {
                    _targetPositions[i] = _joints[i]->getPosition();
//                    _targetPositionsPrev[i] = _targetPositions[i];

                    continue; // Disabled, ignore
                }

                if (fabs(_targetPositions[i] != _targetPositionsPrev[i]) || _needPlan[i]) { // need update
                    _targetPositionsPrev[i] = _targetPositions[i]; // assign to current target position

                    _interp[i]->planProfile(0, // t
                                            _pos[i], // p0
                                            _targetPositionsPrev[i], // pf
                                            _vel[i],  // v0
                                            _targetVelocities[i], // vf
                                            _max_vel[i],
                                            _max_acc[i],
                                            _max_jerk[i]);

                    dt[i] = 0.0; // once regenerate, dt = 0.0
                    // consider at least execute time
                    if (_interp[i]->getDuration() < _leastMotionTime) {
                        _interp[i]->scaleToDuration(_leastMotionTime);
                    }
                    // record max_time
                    max_time = max(max_time, _interp[i]->getDuration()); // get max duration time

                    _needPlan[i] = false;
                }
            }

            // Sync scaling....
            if (_sync == SYNC_TIME) {
                for_each(_interp.begin(), _interp.end(), [=](R_INTERP_BASE *p) { p->scaleToDuration(max_time); });
            } else if (_sync == SYNC_NONE) {

            } else if (_sync == SYNC_PHASE) {
                std::cout << "[WARNING] Phase sync has not implemented...instead of time sync." << std::endl;
                for_each(_interp.begin(), _interp.end(), [=](R_INTERP_BASE *p) { p->scaleToDuration(max_time); });
            }


            // Start moving....
            for (int i = 0; i < _jntNum; ++i) {
                if (_joints[i]->getDriveState() != DriveState::OperationEnabled) {
                    continue; // Disabled, ignore
                }

                if (_interp[i]->isValidMovement()) {
                    dt[i] += 0.001;
                    _pos[i] = _interp[i]->pos(dt[i]);
                    _vel[i] = _interp[i]->vel((dt[i]));

                    switch (_joints[i]->getMode()) {
                        case ModeOfOperation::CyclicSynchronousPositionMode:
                            _joints[i]->setPosition(_pos[i]);
                            break;
                        case ModeOfOperation::CyclicSynchronousVelocityMode:
                            _joints[i]->setVelocity(_vel[i]);
                            break;
                        default:
                            std::cout << "Only Supported CSP and CSV" << std::endl;
                    }

                } else {
                    _vel[i] = 0.0;
                }

            }

        }

        // process before exit:



    }

    void Robot::moveJ(const vector<double> &target_pos, const vector<double> &target_vel, Robot::Synchronization sync) {
        if ((target_pos.size() != _jntNum) || (target_vel.size() != _jntNum)) {
            std::cout << "[ERROR] MoveJ => Error Input Vector Size!" << std::endl;
            return;
        }

        _sync = sync;

        _targetPositions = target_pos;
        _targetVelocities = target_vel;

        _needPlan.resize(_jntNum, true);

    }

    /// 设置单轴运动
    /// \param id 轴ID
    /// \param pos 目标位置
    /// \param vel 目标速度
    /// \param max_vel 最大速度
    /// \param max_acc 最大加速度
    /// \param max_jerk 最大加加速度
    /// \param least_time 最短运行时间
    void Robot::moveSingleAxis(int id, double pos, double vel, double max_vel, double max_acc, double max_jerk,
                               double least_time) {
        _targetPositions[id] = pos;
        _targetVelocities[id] = vel;

        if (max_vel != -1)
            _max_vel[id] = max_vel;
        if (max_acc != -1)
            _max_acc[id] = max_acc;
        if (max_jerk != -1)
            _max_jerk[id] = max_jerk;
        if (least_time != -1)
            _leastMotionTime = least_time;

        _needPlan[id] = true;
    }

    /// 设置多轴运动
    /// \param target_pos 目标位置
    /// \param target_vel 目标速度
    /// \param max_vel 最大速度
    /// \param max_acc 最大加速度
    /// \param max_jerk 最大加加速度
    /// \param least_time 最短运行时间
    void Robot::moveMultiAxis(const vector<double> &target_pos, const vector<double> &target_vel,
                              const vector<double> &max_vel, const vector<double> &max_acc,
                              const vector<double> &max_jerk, double least_time) {
        if ((target_pos.size() != _jntNum) || (target_vel.size() != _jntNum) || (max_vel.size() != _jntNum) ||
            (max_acc.size() != _jntNum) || (max_jerk.size() != _jntNum)) {
            std::cout << "[ERROR] moveMultiAxis: wrong size!" << std::endl;
        }

        for (int id = 0; id < _jntNum; id++) {
            _targetPositions[id] = target_pos[id];
            _targetVelocities[id] = target_vel[id];

            if (max_vel[id] != -1)
                _max_vel[id] = max_vel[id];
            if (max_acc[id] != -1)
                _max_acc[id] = max_acc[id];
            if (max_jerk[id] != -1)
                _max_jerk[id] = max_jerk[id];

            _needPlan[id] = true;
        }

        if (least_time != -1)
            _leastMotionTime = least_time;
    }

}