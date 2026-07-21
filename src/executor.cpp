#include "executor.hpp"

namespace rocos {

Executor::Executor(MotionInterface* motion, ControllerInterface* controller,
                   HardwareInterface* hardware) {

  SwitchMotion(motion);
  SwitchController(controller);
  SwitchHardware(hardware);

  profiler_ = std::make_unique<PerformanceProfiler>(
    std::vector<PerformanceMeasureInfo>{
        {kMotionMeasurement, "Motion"},
        {kControllerMeasurement, "Controller"},
        {kExecutorMeasurement, "Executor Update"},
        {kExecutorMeasurement, "Cycle Time"}});

}

Executor::~Executor() {
  if (motion_)
    motion_->Reset();

  if (controller_)
    controller_->Reset();

  if (hardware_)
    hardware_->Reset();

  motion_ = nullptr;
  controller_ = nullptr;




}

Result Executor::Update() {
  if (!motion_ || !controller_) return Result::PlanError;

  profiler_->MeasureEnd(kCycleMeasurement);
  profiler_->MeasureStart(kExecutorMeasurement);

  // ① Motion → 当前参考位姿
  profiler_->MeasureStart(kMotionMeasurement);
  Reference ref;
  Result res = motion_->GenerateRef(ref);
  profiler_->MeasureEnd(kMotionMeasurement);
  if (static_cast<int>(res) < 0) return res;

  // ② Controller → 参考转关节指令
  profiler_->MeasureStart(kControllerMeasurement);
  JntArray q_cmd;
  res = controller_->GenerateCmd(ref, q_cmd);
  profiler_->MeasureEnd(kControllerMeasurement);
  if (static_cast<int>(res) < 0) return res;

  // ③ 下发硬件
  res = controller_->UpdateCmd(q_cmd);
if (static_cast<int>(res) < 0) return res;


  profiler_->MeasureEnd(kExecutorMeasurement);
  profiler_->MeasureStart(kCycleMeasurement);
  return Result::NoError;
}

bool Executor::SwitchController(ControllerInterface* new_controller) {
  if (controller_)
    controller_->Reset();

  controller_ = new_controller;
  return true;

}

bool Executor::SwitchHardware(HardwareInterface* new_hardware) {
  if (hardware_)
        hardware_->Reset(); //对之前的hardware指针进行重置

  hardware_ = new_hardware;
  return true;
}

bool Executor::SwitchMotion(MotionInterface* new_motion) {
  motion_ = new_motion; // 资源释放只需在子类析构处理，这里只需要直接替换
  return true;
}

}  // namespace rocos
