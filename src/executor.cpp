#include "executor.hpp"

namespace rocos {

Executor::Executor(MotionInterface* motion, ControllerInterface* controller,
                   HardwareInterface* hardware) {}

Executor::~Executor() {}

Result Executor::Update() {
  //更新过程需要加锁




  Ref ref;

  auto res = motion_->GenerateRef(ref); //

  if (res < 0) {
     return res;
  }

  JntArray q_cmd;

  res = controller_->GenerateCmd(ref, q_cmd); //TODO：生成指令，没有发是因为有可能在Executor中有额外处理

  if (res < 0) {
    return res;
  }


  controller_->UpdateCmd(q_cmd); //TODO: 给Hardware发送指令


  return Result::NoError;

}
bool Executor::SwitchController(ControllerInterface* new_contorller) {



}


bool Executor::SwitchHardware(HardwareInterface* new_hardware) {


}

bool Executor::SwitchMotion(MotionInterface* new_motion) {


}

}  // namespace rocos
