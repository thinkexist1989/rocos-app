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
        static boost::shared_ptr<DriveGuard> _instance; // 单例模式对象

        std::vector<Drive*> _drives;

        boost::shared_ptr<boost::thread> _thread {nullptr};

        bool _isThreadRunning {false};



    };

} // namespace rocos

#endif //ROCOS_APP_DRIVE_GUARD_H
