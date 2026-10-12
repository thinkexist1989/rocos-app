#include "executor.hpp"

namespace rocos {

Executor::Executor(MotionInterface* motion, ControllerInterface* controller,
                   HardwareInterface* hardware) {

  log_ptr_ = Logger::getInstance("Executor");

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

  Result res = Result::NoError;
  int sum = 0; //TODO:

  if (motion_ == nullptr || controller_ == nullptr || hardware_ == nullptr) {
    log_ptr_->error(
        "Executor update has null dependency: motion={}, controller={}, hardware={}",
        static_cast<const void*>(motion_), static_cast<const void*>(controller_),
        static_cast<const void*>(hardware_));
  }

  profiler_->MeasureEnd(kCycleMeasurement);
  profiler_->MeasureStart(kExecutorMeasurement);

  // ① Motion → 当前参考位姿
  profiler_->MeasureStart(kMotionMeasurement);
  Reference ref;
  res = motion_->GenerateRef(ref);

  profiler_->MeasureEnd(kMotionMeasurement);

  if (static_cast<int>(res) < 0) {
    log_ptr_->error("Executor motion phase failed: result={}", static_cast<int>(res));
    return res;
  }

  sum += static_cast<int>(res);

  // ② Controller → 参考转关节指令
  profiler_->MeasureStart(kControllerMeasurement);
  JntArray q_cmd;
  res = controller_->GenerateCmd(ref, q_cmd);
  profiler_->MeasureEnd(kControllerMeasurement);

  if (static_cast<int>(res) < 0) {
    log_ptr_->error("Executor controller phase failed: result={}",
                    static_cast<int>(res));
    return res;
  }

  sum += static_cast<int>(res);


  // ③ 下发硬件
  res = controller_->UpdateCmd(q_cmd);

  if (static_cast<int>(res) < 0) {
    log_ptr_->error("Executor hardware command phase failed: result={}",
                    static_cast<int>(res));
    return res;
  }

  sum += static_cast<int>(res);


  profiler_->MeasureEnd(kExecutorMeasurement);
  profiler_->MeasureStart(kCycleMeasurement);

  if (sum > 0) {
    return Result::PlanFinished;
  }

  return Result::NoError;
}

bool Executor::SwitchController(ControllerInterface* new_controller) {

  controller_ = new_controller;
  return true;

}

bool Executor::SwitchHardware(HardwareInterface* new_hardware) {
  hardware_ = new_hardware;
  return true;
}

bool Executor::SwitchMotion(MotionInterface* new_motion) {
  motion_ = new_motion; // 资源释放只需在子类析构处理，这里只需要直接替换
  return true;
}

}  // namespace rocos
