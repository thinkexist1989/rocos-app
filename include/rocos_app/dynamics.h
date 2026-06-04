// Copyright 2026, Yang Luo"
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

#ifndef ROCOS_APP_DYNAMICS_H
#define ROCOS_APP_DYNAMICS_H

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <memory>

#include <kdl/chainidsolver.hpp>
#include <kdl/chainfdsolver.hpp>

#include <rocos_app/logger.h>

namespace rocos {
  using namespace KDL;

  class Dynamics {
  public:
    Dynamics();
    Dynamics(const KDL::Chain& chain);
    ~Dynamics();


    bool setChain(const KDL::Chain& chain);
    bool setChain(const KDL::Tree& tree, const std::string& base_link, const std::string& tip);


    int FwdDyn(const JntArray &q, const JntArray &q_dot, const JntArray &torques, const Wrenches& f_ext,JntArray &q_dotdot); //这个是fd
    int InvDyn(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext,JntArray &torques); //这个是id

    const KDL::Chain& getChain() { return chain_; } //返回运动链

    void Initialize();

private:
    KDL::Tree  tree_;
    KDL::Chain chain_;

    std::unique_ptr<KDL::ChainFdSolver> fd_solver_;
    std::unique_ptr<KDL::ChainIdSolver> id_solver_;

    Logger::logger_ptr log_ptr_ = nullptr;

  };

}







#endif // ROCOS_APP_DYNAMICS_H
