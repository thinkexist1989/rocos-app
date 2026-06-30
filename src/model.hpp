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
#pragma once

#include <memory>

#include "model_interface.hpp"

namespace rocos {

class Model : public ModelInterface {
 public:
  Model(std::string urdf_file_path);
  ~Model();

  Result ForwardKinematics(const JntArray& q_in, Frame& p_out) override;
  Result InverseKinematics(const JntArray& q_in, const Frame& p_in,
                           JntArray& q_out) override;
  Result ForwardDynamics(const JntArray& q, const JntArray& q_dot,
                         const JntArray& torques, const Wrenches& f_ext,
                         JntArray& q_dotdot) override;
  Result InverseDynamics(const JntArray& q, const JntArray& q_dot,
                         const JntArray& q_dotdot, const Wrenches& f_ext,
                         JntArray& torques) override;

 private:

  Chain chain_;

  std::string urdf_file_path_;

  std::unique_ptr<ChainFkSolverPos> fk_solver_;
  std::unique_ptr<ChainIkSolverPos> ik_solver_;

  std::unique_ptr<ChainFdSolver> fd_solver_;
  std::unique_ptr<ChainIdSolver> id_solver_;



};

}  // namespace rocos
