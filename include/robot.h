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

#ifndef ROCOS_APP_ROBOT_H
#define ROCOS_APP_ROBOT_H

#include <hardware_interface.h>
#include <boost/smart_ptr.hpp>
#include <vector>
#include <drive.h>


namespace rocos {

    class Robot {
    public:
        enum Synchronization {
            SYNC_NONE,
            SYNC_TIME,
            SYNC_PHASE
        };

        explicit Robot(boost::shared_ptr<HardwareInterface> hw);

        void moveJ(const std::vector<double> &pos,
                   const std::vector<double> &max_vel,
                   const std::vector<double> &max_acc,
                   const std::vector<double> &max_jerk,
                   Synchronization sync = SYNC_TIME,
                   ProfileType type = ProfileType::trapezoid);

        void setEnabled();

        void setDisabled();

        inline int getJointNum() { return _jntNum; }

        inline int getJointStatus(int id) { return _joints[id]->getDriveStateRPC(); }

        inline double getJointPosition(int id) { return _joints[id]->getPosition(); }

        inline double getJointVelocity(int id) { return _joints[id]->getVelocity(); }

        inline double getJointTorque(int id) { return _joints[id]->getTorque(); }

        inline double getJointLoadTorque(int id) { return _joints[id]->getLoadTorque(); }

    protected:
        void addAllJoints();

    protected:
        boost::shared_ptr<HardwareInterface> _hw_interface{nullptr};
        std::vector<boost::shared_ptr<Drive>> _joints;

        int _jntNum;
    };

}


#endif //ROCOS_APP_ROBOT_H
