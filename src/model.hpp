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

#include "logger.hpp"
#include "model_interface.hpp"

namespace rocos {

class Model : public ModelInterface {
 public:
  explicit Model(const std::string& urdf_file_path,
                 const std::string& base_link,
                 const std::string& tip);

  ~Model() override = default;

  Result ForwardKinematics(const JntArray& q_in, Frame& p_out) override;
  Result InverseKinematics(const JntArray& q_in, const Frame& p_in,
                           JntArray& q_out) override;
  Result ForwardDynamics(const JntArray& q, const JntArray& q_dot,
                         const JntArray& torques, const Wrenches& f_ext,
                         JntArray& q_dotdot) override;
  Result InverseDynamics(const JntArray& q, const JntArray& q_dot,
                         const JntArray& q_dotdot, const Wrenches& f_ext,
                         JntArray& torques) override;

  inline void SetChain(const Chain& chain) { chain_ = chain; }

  bool SetChain(const Tree& tree, const std::string& base_link,
                const std::string& tip);

  void SetGravity(const Vector& gravity);

  // TODO: 关节限位是给内部调用，不能外部设置，通过URDF配置
  void SetPosLimits(const JntArray& q_min, const JntArray& q_max);

  void UpdateSolvers();

  bool ParseUrdf(const std::string &urdf_file_path,
               const std::string &base_link,
               const std::string &tip);

 private:
  Tree tree_;
  Chain chain_;

  std::string urdf_file_path_;
  std::string urdf_string_;

  std::unique_ptr<ChainFkSolverPos> fk_solver_;
  std::unique_ptr<ChainIkSolverPos> ik_solver_;

  std::unique_ptr<ChainFdSolver> fd_solver_;
  std::unique_ptr<ChainIdSolver> id_solver_;

  JntArray q_min_;
  JntArray q_max_;

  Vector gravity_{0.0, 0.0, -9.81};

  Logger::logger_ptr log_ptr_ = nullptr;
};

}  // namespace rocos
