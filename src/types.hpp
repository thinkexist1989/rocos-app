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

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfdsolver.hpp>
#include <kdl/chainidsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace rocos {

  using Vector = KDL::Vector;
  using Jacobian = KDL::Jacobian;
  using JntArray = KDL::JntArray;
  using Frame = KDL::Frame;
  using Wrench = KDL::Wrench;
  using Chain = KDL::Chain;
  using ChainIkSolverPos = KDL::ChainIkSolverPos;
  using ChainFkSolverPos = KDL::ChainFkSolverPos;
  using ChainFdSolver = KDL::ChainFdSolver;
  using ChainIdSolver = KDL::ChainIdSolver;


}  // namespace rocos