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


#include "include/rocos_app/drive_guard.h"
#include "include/rocos_app/drive.h"

namespace rocos {

    DriveGuard::DriveGuard() {
        is_thread_running_ = true;
        thread_ = boost::make_shared<boost::thread>(&DriveGuard::workingThread, this);
//        thread_ = boost::make_shared<boost::thread>(boost::bind(&DriveGuard::workingThread, this));
//        thread_->detach();
    }

    DriveGuard::~DriveGuard() {
        is_thread_running_ = false;
        thread_->interrupt(); // 向线程发送结束请求
        thread_->join(); // 等待线程结束
    }

    boost::shared_ptr<DriveGuard> DriveGuard::getInstance() {
        if (instance_ == nullptr) {
            instance_.reset(new DriveGuard(), [](DriveGuard *t) { delete t; }); // 因为默认访问不了private 析构函数,需传入删除器
        }
        return instance_;
    }

    void DriveGuard::addDrive(Drive *drive) {
        drives_.push_back(drive); //将驱动器添加到_drives中
    }

    void DriveGuard::workingThread() {
        std::cout << "Drive Guard is running on thread " << boost::this_thread::get_id() << std::endl;
        while (is_thread_running_) {

            for (auto &d: drives_) {

                d->current_drive_state_ = d->hw_interface_->getDriverState(d->id_);

                if (d->conduct_state_change_)
                    d->engageStateMachine();

            }

            usleep(10000);
        }
        std::cout << "Drive Guard thread is terminated." << std::endl;
    }

    boost::shared_ptr<DriveGuard> DriveGuard::instance_ = nullptr; // 单例模式对象

}
