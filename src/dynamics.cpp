#include "dynamics.hpp"

namespace rocos {

Dynamics::Dynamics() {
  log_ptr_ = Logger::getInstance("Dynamics");
}

Dynamics::Dynamics(const KDL::Chain &chain) {

}

Dynamics::~Dynamics() {

}

void Dynamics::Initialize() {

}

bool Dynamics::setChain(const KDL::Chain &chain) {
  chain_ = chain;

  return true;

}

bool Dynamics::setChain(const KDL::Tree &tree, const std::string &base_link,
                        const std::string &tip) {
  tree_ = tree;

  if(!tree_.getChain(base_link, tip, chain_)) {
    // 从KDL::Tree获取运动链失败
    log_ptr_->error("Could not get chain from kdl tree!");
    return false;
  }

  return true;
}

int Dynamics::FwdDyn(const JntArray &q, const JntArray &q_dot,
                         const JntArray &torques, const Wrenches &f_ext,
                         JntArray &q_dotdot) {

  return 0;
}

int Dynamics::InvDyn(const JntArray &q, const JntArray &q_dot,
                     const JntArray &q_dotdot, const Wrenches &f_ext,
                     JntArray &torques) {

  return 0;
}

} // namespace rocos