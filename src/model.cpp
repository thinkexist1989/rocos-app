#include "model.hpp"

#include <kdl/chainfdsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf_parser/urdf_parser.h>

namespace rocos {

Model::Model(const std::string& urdf_file_path,
             const std::string& base_link,
             const std::string& tip) {
  log_ptr_ = Logger::getInstance("Model");

  //先根据文件路径把内容加载到urdf_string_
  std::ifstream urdf_file(urdf_file_path);
  if (!urdf_file) {
    log_ptr_->error("Could not open URDF file: {}", urdf_file_path);
    throw std::runtime_error("Could not open urdf file");
  }
  urdf_string_ = std::string((std::istreambuf_iterator<char>(urdf_file)),
                             std::istreambuf_iterator<char>());

  if (!ParseUrdf(urdf_string_, base_link, tip)) {
    log_ptr_->error("Could not extract  urdf!");


  }





}

Result Model::ForwardKinematics(const JntArray& q_in, Frame& p_out) {
  if (fk_solver_->JntToCart(q_in, p_out) < 0) {
    log_ptr_->error("Forward kinematics computation failed!");
    return Result::FkCalcFail;
  }
  return Result::NoError;
}

Result Model::InverseKinematics(const JntArray& q_in, const Frame& p_in,
                                JntArray& q_out) {
  if (ik_solver_->CartToJnt(q_in, p_in, q_out) < 0) {
    log_ptr_->error("Inverse kinematics computation failed!");
    return Result::IkCalcFail;
  }
  return Result::NoError;
}

Result Model::ForwardDynamics(const JntArray& q, const JntArray& q_dot,
                              const JntArray& torques, const Wrenches& f_ext,
                              JntArray& q_dotdot) {
  if (fd_solver_->CartToJnt(q, q_dot, torques, f_ext, q_dotdot) < 0) {
    log_ptr_->error("Forward dynamics computation failed!");
    return Result::FdCalcFail;
  }
  return Result::NoError;
}

Result Model::InverseDynamics(const JntArray& q, const JntArray& q_dot,
                              const JntArray& q_dotdot, const Wrenches& f_ext,
                              JntArray& torques) {
  if (id_solver_->CartToJnt(q, q_dot, q_dotdot, f_ext, torques) < 0) {
    log_ptr_->error("Inverse dynamics computation failed!");
    return Result::IdCalcFail;
  }
  return Result::NoError;
}

bool Model::SetChain(const Tree& tree, const std::string& base_link,
                     const std::string& tip) {
  if (!tree.getChain(base_link, tip, chain_)) {
    // 从KDL::Tree获取运动链失败
    log_ptr_->error("Could not get chain from kdl tree!");
    return false;
  }

  q_min_.resize(chain_.getNrOfJoints());
  q_max_.resize(chain_.getNrOfJoints());

  return true;
}

void Model::SetGravity(const Vector& gravity) {
  gravity_ = gravity;
}

void Model::SetPosLimits(const JntArray& q_min, const JntArray& q_max) {
  q_min_ = q_min;
  q_max_ = q_max;
}

void Model::UpdateSolvers() {
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(chain_, q_min_, q_max_);

  fd_solver_ = std::make_unique<KDL::ChainFdSolver_RNE>(chain_, gravity_);
  id_solver_ = std::make_unique<KDL::ChainIdSolver_RNE>(chain_, gravity_);

}

bool Model::ParseUrdf(const std::string& urdf_string,
                      const std::string& base_link, const std::string& tip) {

  auto model = urdf::parseURDF(urdf_string);
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*model, tree)) {
    // 解析失败
    log_ptr_->error("Could not extract urdf to Tree!");

    return false;
  }

  if (!SetChain(tree, base_link, tip)) {
    log_ptr_->error("Could not set kinematic chain!");
    return false;
  }
  
  const double kDefaultLowerLimit = -KDL::PI;
  const double kDefaultUpperLimit = KDL::PI;








  return true;



}

}  // namespace rocos