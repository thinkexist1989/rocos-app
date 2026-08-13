# Controller Servo Validation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the first three Servo protection tasks: parse URDF velocity/effort limits in `Model`, expose them through `ModelInterface`, and make `PositionController::UpdateCmd()` validate position commands before hardware writes.

**Architecture:** `Model` remains the control-layer source of joint limits in KDL/model joint order. `PositionController` uses `model_` plus `hardware_->GetPosition()` and `hardware_->GetDt()` to reject invalid commands before calling `SetMode()` or `SetPosition()`. Ready and destructor paths call `UpdateCmd(q_hold)` so they reuse the same validation.

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

Add tests in `test/position_controller_test.cc` proving `UpdateCmd()` rejects commands with missing model, position limit violations, and `(q_cmd - q_actual) / dt` velocity violations without calling `SetPosition()`.

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

### Task 4: Final Verification

**Files:**
- Verify only.

- [x] **Step 1: Run focused tests**

Run: `./cmake-build-codex/bin/model_test && ./cmake-build-codex/bin/position_controller_test`

Expected: both binaries exit 0.

- [x] **Step 2: Run CTest subset**

Run: `ctest --test-dir cmake-build-codex -R 'model_test|position_controller_test' --output-on-failure`

Expected: all selected tests pass.

- [ ] **Step 3: Commit implementation**

Commit only relevant source, test, and plan files. Do not add unrelated `docs/ROCOS  API.openapi copy.yaml`.
