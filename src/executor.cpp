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

Executor::Executor() {




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
  // 更新过程需要加锁
  // std::lock_guard<std::mutex> lock(mtx_);

  profiler_->MeasureEnd(kCycleMeasurement); //这个是测量循环周期，所以要在结束时Start，下次进入时End
  profiler_->MeasureStart(kExecutorMeasurement);


  Reference ref;
  profiler_->MeasureStart(kMotionMeasurement);
  auto res = motion_->GenerateRef(ref);
  profiler_->MeasureEnd(kMotionMeasurement);

  if (res < 0) {
    return res;
  }
  else if (res > 0) {
    return res;
  }

  JntArray q_cmd;
  profiler_->MeasureStart(kControllerMeasurement);
  res = controller_->GenerateCmd(
      ref, q_cmd);  // TODO：生成指令，没有发是因为有可能在Executor中有额外处理
  profiler_->MeasureEnd(kControllerMeasurement);
  if (res < 0) {
    return res;
  }

  controller_->UpdateCmd(q_cmd);  // TODO: 给Hardware发送指令

  profiler_->MeasureEnd(kExecutorMeasurement);

  profiler_->MeasureStart(kCycleMeasurement); //这个是测量循环周期，所以要在结束时Start，下次进入时End


  return Result::NoError;
}

bool Executor::SwitchController(ControllerInterface* new_contorller) {
  // std::lock_guard<std::mutex> lock(mtx_);

  if (controller_)
    controller_->Reset();

  controller_ = new_contorller;

}

bool Executor::SwitchHardware(HardwareInterface* new_hardware) {
  // std::lock_guard<std::mutex> lock(mtx_);

  if (hardware_)
        hardware_->Reset(); //对之前的hardware指针进行重置

  hardware_ = new_hardware;
}

bool Executor::SwitchMotion(MotionInterface* new_motion) {
  // std::lock_guard<std::mutex> lock(mtx_);

  if (!motion_)
    motion_->Reset(); //对之前的motion指针进行重置

  motion_ = new_motion;
}

}  // namespace rocos
