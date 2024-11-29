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

#ifndef ROCOS_APP_KINEMATICS_H
#define ROCOS_APP_KINEMATICS_H

#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>

#include "kdl/frames.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/chain.hpp"
#include "kdl/tree.hpp"

#include "trac_ik/trac_ik.hpp" //逆运动学处理

namespace rocos {
    using namespace KDL;

    class Kinematics {
    public:
        Kinematics();
        Kinematics(const std::string& urdf_file_path,
                   const std::string& base_link,
                   const std::string& tip);
        Kinematics(const KDL::Chain& chain);
        ~Kinematics();

        bool setChain(const KDL::Chain& chain); // 直接传入Chain
        bool setChain(const std::string& base_link, const std::string& tip); //通过设置base_link和tip修改运动链
        bool setChain(const KDL::Tree& tree, const std::string& base_link, const std::string& tip); //传入Tree

        bool setPosLimits(const KDL::JntArray& q_min, const KDL::JntArray& q_max);

        const KDL::Chain& getChain() { return chain_; } //返回运动链

        void Initialize();

        int JntToCart(const JntArray& q_in, Frame& p_out);
        int CartToJnt(const JntArray &q_init, const Frame &p_in, JntArray &q_out);

        //TODO: for test;
        static void getChainTechServo(KDL::Chain& chain, KDL::JntArray& q_min, KDL::JntArray& q_max);
        static void getChain7Dofs(KDL::Chain& chain, KDL::JntArray& q_min, KDL::JntArray& q_max);

        void initTechServo() {
            //TODO: 初始化泰科6-DOF机械臂
            getChainTechServo(chain_, q_min_, q_max_);
            Initialize(); // 初始化
        }
        void initChain7Dofs() {
            //TODO: 初始化7-DOF机械臂
            getChain7Dofs(chain_, q_min_, q_max_);
            Initialize(); // 初始化
        }

    private:
        KDL::Tree tree_;
        KDL::Chain chain_; // KDL运动链
        KDL::JntArray q_min_; // 关节最小位置
        KDL::JntArray q_max_; // 关节最大位置

        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
        std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver_;

    };

}


#endif //ROCOS_APP_KINEMATICS_H
