#include "executor.hpp"

namespace rocos {

Executor::Executor(MotionInterface* motion, ControllerInterface* controller,
                   HardwareInterface* hardware) {}

Executor::~Executor() {}

Result Executor::Update() {
  Ref ref;

  auto res = motion_->UpdateRef(ref);

  if (res < 0) {
     return res;
  }

  JntArray q_cmd;

  res = controller_->UpdateCmd(ref, q_cmd);

  if (res < 0) {
    return res;
  }




  return Result::NoError;

}

}  // namespace rocos
