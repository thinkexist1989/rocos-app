# Controller Servo Validation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement first-stage controller-local Servo validation: parse URDF velocity/effort limits in `Model`, expose them through `ModelInterface`, and make position/torque controllers validate commands before hardware writes.

**Architecture:** `Model` remains the control-layer source of joint limits in KDL/model joint order. Position-style controllers use `model_` plus `hardware_->GetPosition()` and `hardware_->GetDt()` to reject invalid commands before calling `SetMode()` or `SetPosition()`. Torque-style controllers validate current joint position, current joint velocity, and final torque command against URDF limits before calling `SetMode()` or `SetTorque()`. Ready and destructor paths call `UpdateCmd(q_hold)` so they reuse the same validation.

**Explicitly skipped:** Do not add `Robot` initialization-time URDF/hardware limit consistency checks in this phase. URDF limits are allowed to be stricter than hardware limits.

**Tech Stack:** C++17, KDL, urdfdom, doctest, CMake/CTest.

---

### Task 1: Model URDF Velocity And Effort Limits

**Files:**
- Modify: `src/model_interface.hpp`
- Modify: `src/model.hpp`
- Modify: `src/model.cpp`
- Test: `test/model_test.cc`

- [x] **Step 1: Write failing tests**

Add tests in `test/model_test.cc` that call `GetVelocityLimit()` and `GetEffortLimit()` on the talon model and assert lower/upper/velocity/effort arrays have seven joints in model order.

- [x] **Step 2: Run red test**

Run: `cmake --build cmake-build-codex --target model_test -j2`

Expected: compile failure because `GetVelocityLimit()` and `GetEffortLimit()` do not exist yet.

- [x] **Step 3: Implement minimal model support**

Add limit getters to `ModelInterface`; add concrete getters and `q_effort_` storage to `Model`; resize `q_vel_` and `q_effort_` with the chain; parse standard `urdf_joint->limits` and the existing custom `<hardware><limit vel=... effort=...>` tags.

- [x] **Step 4: Run green test**

Run: `cmake --build cmake-build-codex --target model_test -j2 && ./cmake-build-codex/bin/model_test`

Expected: test binary exits 0.

### Task 2: PositionController Command Validation

**Files:**
- Modify: `src/position_controller.hpp`
- Modify: `src/position_controller.cpp`
- Test: `test/position_controller_test.cc`

- [x] **Step 1: Write failing tests**

Add tests in `test/position_controller_test.cc` proving `UpdateCmd()` rejects commands with missing model, position limit violations, and `(q_cmd - q_last_cmd) / dt` velocity violations without calling `SetPosition()`.

- [x] **Step 2: Run red test**

Run: `cmake --build cmake-build-codex --target position_controller_test -j2 && ./cmake-build-codex/bin/position_controller_test`

Expected: tests fail because `UpdateCmd()` currently writes commands directly.

- [x] **Step 3: Implement minimal validation**

Add private `ValidatePositionCommand(const JntArray&)`; call it at the start of `UpdateCmd()` before `SetMode()` and `SetPosition()`. Use `ModelInterface` lower/upper/velocity limits and `hardware_->GetPosition()` / `GetDt()`.

- [x] **Step 4: Run green test**

Run: `cmake --build cmake-build-codex --target position_controller_test -j2 && ./cmake-build-codex/bin/position_controller_test`

Expected: test binary exits 0.

### Task 3: Ready And Destructor Use UpdateCmd

**Files:**
- Modify: `src/position_controller.cpp`
- Test: `test/position_controller_test.cc`

- [x] **Step 1: Write failing tests**

Add tests proving `SetReady()` and `PositionController` destructor call through the same safe path and do not directly write mode/position outside `UpdateCmd()`.

- [x] **Step 2: Run red test**

Run: `cmake --build cmake-build-codex --target position_controller_test -j2 && ./cmake-build-codex/bin/position_controller_test`

Expected: tests fail against the current direct `SetPosition(GetPosition())` / `SetMode(8)` paths.

- [x] **Step 3: Implement minimal routing**

Change `SetReady()` to `WaitForSignal()` then `UpdateCmd(hardware_->GetPosition())`. Change the destructor to do the same best-effort path and ignore the returned error because destructors cannot report `Result`.

- [x] **Step 4: Run targeted green tests**

Run: `cmake --build cmake-build-codex --target position_controller_test -j2 && ./cmake-build-codex/bin/position_controller_test`

Expected: test binary exits 0.

### Task 4: JointAdmittanceController Position Validation

**Files:**
- Modify: `src/joint_admittance_controller.hpp`
- Modify: `src/joint_admittance_controller.cpp`
- Test: `test/joint_admittance_controller_test.cc`

- [x] **Step 1: Write failing tests**

Add tests proving `UpdateCmd()` rejects missing model, q_out position limit violations, and `(q_out - q_last_cmd) / dt` velocity violations without calling `SetMode()` or `SetPosition()`.

- [x] **Step 2: Run red test**

Run: `cmake --build cmake-build-codex --target joint_admittance_controller_test -j2 && ./cmake-build-codex/bin/joint_admittance_controller_test`

Expected: tests fail because `UpdateCmd()` currently writes `q_out` directly.

- [x] **Step 3: Implement minimal validation**

Add private `ValidatePositionCommand(const JntArray&)`; call it after `q_out` is computed and before `SetMode()` / `SetPosition()`. Use URDF lower/upper/velocity from `ModelInterface` and current joint position from hardware. Route `SetReady()` and destructor through `UpdateCmd(hardware_->GetPosition())`.

- [x] **Step 4: Run green test**

Run: `cmake --build cmake-build-codex --target joint_admittance_controller_test -j2 && ./cmake-build-codex/bin/joint_admittance_controller_test`

Expected: test binary exits 0.

### Task 5: JointImpedanceController Torque Validation

**Files:**
- Modify: `src/joint_impedance_controller.hpp`
- Modify: `src/joint_impedance_controller.cpp`
- Test: `test/joint_impedance_controller_test.cc`

- [x] **Step 1: Write failing tests**

Add tests proving `UpdateCmd()` rejects missing model, current position outside URDF limits, current velocity over URDF limit, and final torque over URDF effort without calling `SetMode()` or `SetTorque()`.

- [x] **Step 2: Run red test**

Run: `cmake --build cmake-build-codex --target joint_impedance_controller_test -j2 && ./cmake-build-codex/bin/joint_impedance_controller_test`

Expected: tests fail because `UpdateCmd()` currently writes torque directly and allows missing model.

- [x] **Step 3: Implement minimal validation**

Add private `ValidateTorqueCommand(const JntArray&)`; call it after final torque is computed and before `SetMode()` / `SetTorque()`. Route `SetReady()` and destructor through `UpdateCmd(hardware_->GetPosition())`.

- [x] **Step 4: Run green test**

Run: `cmake --build cmake-build-codex --target joint_impedance_controller_test -j2 && ./cmake-build-codex/bin/joint_impedance_controller_test`

Expected: test binary exits 0.

### Task 6: CartesianImpedanceController Torque Validation

**Files:**
- Modify: `src/cartesian_impedance_controller.hpp`
- Modify: `src/cartesian_impedance_controller.cpp`
- Test: `test/cartesian_impedance_controller_test.cc`

- [x] **Step 1: Write detailed failing tests**

Add detailed tests proving:
- missing model is rejected before torque writes;
- current position outside lower/upper URDF limits returns `PosLimit`;
- current velocity over URDF velocity limit returns `SpeedLimit`;
- final torque over URDF effort limit returns `ForceLimit`;
- non-finite final torque returns `ParameterNanOrInf`;
- `SetReady()` and destructor route through `UpdateCmd(hardware_->GetPosition())` instead of directly writing gravity torque.

- [x] **Step 2: Run red test**

Run: `cmake --build cmake-build-codex --target cartesian_impedance_controller_test -j2 && ./cmake-build-codex/bin/cartesian_impedance_controller_test`

Expected: tests fail because `UpdateCmd()` currently has no URDF-limit guard and Ready/destructor directly write hardware.

- [x] **Step 3: Implement minimal validation**

Add private `ValidateTorqueCommand(const JntArray&)`; call it after existing internal torque rate/saturation logic and before saving `tau_prev_`, `SetMode()`, and `SetTorque()`. Route `SetReady()` and destructor through `GenerateCmd(q_hold)`/`UpdateCmd(q_hold)` or equivalent state setup so all hardware torque writes pass through validation.

- [x] **Step 4: Run green test**

Run: `cmake --build cmake-build-codex --target cartesian_impedance_controller_test -j2 && ./cmake-build-codex/bin/cartesian_impedance_controller_test`

Expected: test binary exits 0.

### Task 7: Robot Limit Consistency Check

- [x] **Skipped by request**

Do not implement startup URDF/hardware consistency checks. URDF limits may be stricter than hardware limits.

### Task 8: Final Verification

**Files:**
- Verify only.

- [x] **Step 1: Run focused tests**

Run: `./cmake-build-codex/bin/model_test && ./cmake-build-codex/bin/position_controller_test && ./cmake-build-codex/bin/joint_admittance_controller_test && ./cmake-build-codex/bin/joint_impedance_controller_test && ./cmake-build-codex/bin/cartesian_impedance_controller_test`

Expected: all binaries exit 0.

- [x] **Step 2: Run CTest subset**

Run: `ctest --test-dir cmake-build-codex -R 'model_test|position_controller_test|joint_admittance_controller_test|joint_impedance_controller_test|cartesian_impedance_controller_test' --output-on-failure`

Expected: all selected tests pass.

- [x] **Step 3: Commit implementation**

Commit only relevant source, test, and plan files. Do not add unrelated `docs/ROCOS  API.openapi copy.yaml`.
