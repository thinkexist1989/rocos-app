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

#ifndef ROCOS_APP_DRIVE_GUARD_H
#define ROCOS_APP_DRIVE_GUARD_H

/* DriveGuard类采用单例模式进行设计
 * 每一个驱动器都会添加到DriveGuard中进行监测
 *
 * 这个类作为驱动器状态监测类，用于监测所有驱动器状态，状态机切换
 * 后续会添加驱动器位置超限监测、速度超限监测等
 */

#include <boost/noncopyable.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <vector>

namespace rocos {

    class Drive; //Drive类前置声明

    class DriveGuard : boost::noncopyable {
    private:
        DriveGuard();

        ~DriveGuard();

    public:
        static boost::shared_ptr<DriveGuard> getInstance(); //获取句柄

        void addDrive(Drive* drive);

        void workingThread();

    private:
        static boost::shared_ptr<DriveGuard> instance_; // 单例模式对象

        std::vector<Drive*> drives_; // 用于保存驱动器指针

        boost::shared_ptr<boost::thread> thread_ {nullptr};

        bool is_thread_running_ {false};



    };

} // namespace rocos

#endif //ROCOS_APP_DRIVE_GUARD_H
