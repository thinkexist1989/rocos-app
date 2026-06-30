#include "model.hpp"
#include <trac_ik/trac_ik.hpp>


namespace rocos {

Model::Model(std::string urdf_file_path) {


}

Model::~Model() {

}

Result Model::ForwardKinematics(const JntArray& q_in, Frame& p_out) {
  return Result::NoError;

}

Result Model::InverseKinematics(const JntArray& q_in, const Frame& p_in,
                                JntArray& q_out) {
  return Result::NoError;

}

Result Model::ForwardDynamics(const JntArray& q, const JntArray& q_dot,
                              const JntArray& torques, const Wrenches& f_ext,
                              JntArray& q_dotdot) {
  return Result::NoError;

}

Result Model::InverseDynamics(const JntArray& q, const JntArray& q_dot,
                              const JntArray& q_dotdot, const Wrenches& f_ext,
                              JntArray& torques) {
  return Result::NoError;
}

} // rocos