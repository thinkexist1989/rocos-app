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


#include <drive_guard.h>
#include <drive.h>

namespace rocos {

    DriveGuard::DriveGuard() {
        _isThreadRunning = true;
        _thread = boost::make_shared<boost::thread>(boost::bind(&DriveGuard::workingThread, this));
        _thread->detach();
    }

    DriveGuard::~DriveGuard() {

    }

    boost::shared_ptr<DriveGuard> DriveGuard::getInstance() {
        if (_instance == nullptr) {
            _instance.reset(new DriveGuard(), [](DriveGuard *t) { delete t; }); // 因为默认访问不了private 析构函数,需传入删除器
        }
        return _instance;
    }

    void DriveGuard::addDrive(Drive *drive) {
        _drives.push_back(drive); //将驱动器添加到_drives中
    }

    void DriveGuard::workingThread() {
        std::cout << "Drive Guard is running on thread " << boost::this_thread::get_id() << std::endl;
        while (_isThreadRunning) {

            for (auto &d: _drives) {

                d->_currentDriveState = d->_hw_interface->getDriverState(d->_id);

                if (d->_conductStateChange)
                    d->engageStateMachine();

            }

            usleep(10000);

        }
        std::cout << "Drive Guard thread is terminated." << std::endl;
    }

    boost::shared_ptr<DriveGuard> DriveGuard::_instance = nullptr; // 单例模式对象

}
