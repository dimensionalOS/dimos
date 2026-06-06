## 1. Adapter Implementation

- [x] 1.1 Create `dimos/hardware/manipulators/dm_motor_arm/adapter.py` with `DMMotorArm` implementing `ManipulatorAdapter`, registered as `dm_motor_arm`.
- [x] 1.2 Add lazy `can_motor_control` Python binding import inside adapter construction/connect paths so registry discovery does not fail when the binding is absent.
- [x] 1.3 Implement DMMotor lifecycle methods for connect, disconnect, enable, disable, clear error, state reads, and supported position/effort command writes through the Python binding.
- [x] 1.4 Implement adapter-owned tick/cache behavior so one DimOS state-read cycle returns coherent position, velocity, and effort values without independently ticking per field.
- [x] 1.5 Implement binding-unavailable and lifecycle-error handling with explicit selected-adapter errors when `can_motor_control` is unavailable.
- [x] 1.6 Add configuration support for binding robot construction from an existing binding TOML path and/or DimOS adapter kwargs, preserving DimOS joint ordering and defaulting CAN-FD on through `canfd=True`.
- [x] 1.7 Preserve existing `openarm` adapter registration and behavior; do not migrate current OpenArm blueprints unless they explicitly opt into `dm_motor_arm`.

## 2. Gravity Compensation

- [x] 2.1 Add model-based gravity compensation support in the DMMotor adapter using the current measured joint state and configured robot model.
- [x] 2.2 Ensure gravity-compensation-only commands use zero position stiffness and configurable low/no damping so joints remain free to move.
- [x] 2.3 Add stale/invalid-state handling that avoids sending gravity compensation commands based on invalid state and surfaces an operator-visible warning or fault.
- [x] 2.4 Add shutdown handling for gravity compensation that disables or stops commanding the arm on stop, interruption, and disconnect.

## 3. Blueprints and Registry

- [x] 3.1 Add an opt-in DMMotor hardware blueprint using `adapter_type="dm_motor_arm"` without replacing existing OpenArm blueprints.
- [x] 3.2 Add adapter kwargs for enabling/disabling in-place DMMotor gravity compensation.
- [x] 3.3 Keep blueprint names clear enough for `dimos list` to distinguish the DMMotor coordinator from existing OpenArm coordinator operation.
- [x] 3.4 Regenerate `dimos/robot/all_blueprints.py` with `pytest dimos/robot/test_all_blueprints_generation.py` if new runnable blueprints are added.

## 4. Tests

- [x] 4.1 Add unit tests that adapter registry discovery includes `dm_motor_arm` without requiring top-level `can_motor_control` import success.
- [x] 4.2 Add missing-binding tests showing selected DMMotor adapter use fails with a clear message and does not auto-install dependencies.
- [x] 4.3 Add adapter lifecycle tests with a fake or mocked binding robot covering connect, enable, read, write, disable, and disconnect order.
- [x] 4.4 Add tick/cache tests proving one DimOS state-read cycle does not call the binding tick once per position, velocity, and effort read.
- [x] 4.5 Add command-shape/order tests for DMMotor position, effort, unsupported velocity, and MIT/gravity-compensation commands in DimOS joint order.
- [x] 4.6 Add gravity-compensation tests proving commands use feed-forward torque with zero position stiffness and avoid sending commands on stale state.

## 5. Documentation

- [x] 5.1 Update `docs/capabilities/manipulation/openarm_integration.md` to document the existing `openarm` adapter and new `dm_motor_arm` Python-binding path separately.
- [x] 5.2 Document that `dimos[manipulation]` installs `can-motor-control` on supported platforms and that selected adapter use fails clearly if `can_motor_control` is absent.
- [x] 5.3 Document adapter-level gravity compensation, upstream OpenArm ROS2 gain presets, and position/effort-only command semantics.
- [x] 5.4 Document staged bring-up: mock/vcan, one-motor validation, full-arm state monitor, gravity compensation, then trajectory-control validation.
- [x] 5.5 Update contributor or coding-agent docs only if implementation introduces a reusable lazy optional binding adapter pattern.

## 6. Verification

- [x] 6.1 Run `openspec validate add-dm-motor-arm-adapter`.
- [x] 6.2 Run focused manipulator adapter tests for `dm_motor_arm` and existing `openarm` coverage.
- [x] 6.3 Run focused coordinator tests that cover `ConnectedHardware` state reads and command writes if adapter/coordinator behavior changes.
- [x] 6.4 Run `pytest dimos/robot/test_all_blueprints_generation.py` if blueprint entries or generated registry output change.
- [x] 6.5 Run documentation validation for changed docs, including `md-babel-py run docs/capabilities/manipulation/openarm_integration.md` if executable blocks are added or changed.
- [x] 6.6 Run a mock or vcan DMMotor adapter smoke test through the library/blueprint surface before using real hardware.
- [ ] 6.7 Manually QA the staged hardware procedure on real DMMotor hardware only after mock/vcan validation: one motor enable/read, one motor low-rate hold, full-arm state monitor, adapter gravity compensation, then optional trajectory-control validation.
