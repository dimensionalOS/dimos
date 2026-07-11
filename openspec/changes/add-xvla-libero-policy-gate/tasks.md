## 1. X-VLA backend capability

- [x] 1.1 Add a LeRobot policy-family selection seam that preserves the existing VLA-JEPA default while allowing `policy_family="xvla"`.
- [x] 1.2 Add X-VLA policy class import/loader support with a family-specific dependency/setup error message.
- [x] 1.3 Ensure backend metadata reports policy family, checkpoint id, resolved policy class, device, episode reset support, and chunk inference support for X-VLA.
- [x] 1.4 Preserve existing VLA-JEPA backend behavior and tests while adding focused unit tests for X-VLA family selection and clear missing-dependency failures.

## 2. X-VLA LIBERO contract and chunk semantics

- [x] 2.1 Verify whether `lerobot/xvla-libero` uses the existing LIBERO observation keys, language metadata, state shape, and action surface; record that X-VLA requires a separate 20D state and absolute 7D action surface.
- [x] 2.2 Reuse the existing LIBERO contract if X-VLA semantics match; otherwise add a narrow X-VLA LIBERO contract through the contract registry.
- [x] 2.3 Add contract tests proving finite X-VLA `N x 7` chunk output converts to `RobotPolicyActionChunk` with action-space id `libero.ee_pose_axis_angle_gripper.absolute.v1`.
- [x] 2.4 Add rejection tests for incompatible X-VLA output shape, non-finite values, or unsupported action semantics.

## 3. Script and configuration surface

- [x] 3.1 Extend `scripts/benchmarks/demo_lerobot_libero_policy_rollout.py` so callers can select VLA-JEPA or X-VLA policy family/checkpoint without changing the live topology.
- [x] 3.2 Add script/configuration support for a two-stage X-VLA gate: synchronous benchmark stage followed by module-native live stage.
- [x] 3.3 Ensure fake/fixed backend tests continue to cover module-native live stream wiring without requiring X-VLA dependencies.
- [x] 3.4 Add script tests proving X-VLA configuration can run the synchronous benchmark path without ControlCoordinator policy chunk execution.
- [x] 3.5 Add script tests proving X-VLA configuration deploys `RobotPolicyModule`, `ControlCoordinator`, stream remapping, and policy trigger RPC through the same live path.
- [x] 3.6 Ensure X-VLA artifacts include policy family, checkpoint id, stage name, inference method/status counts when applicable, chunk counts, consumed actions, stale deactivations, and cleanup status.

## 4. Smoke validation and two-stage real gate

- [x] 4.1 Document the X-VLA dependency invocation pattern without adding X-VLA dependencies to the main `pyproject.toml`.
- [x] 4.2 Run an X-VLA import/load smoke that verifies policy class resolution, checkpoint description, processor setup, and device placement.
- [x] 4.3 Run an X-VLA inference smoke that verifies finite chunk output with shape compatible with the LIBERO 7D action surface.
- [x] 4.4 Run the 10-episode `libero_object` X-VLA synchronous benchmark gate with pass condition `success_rate > 0.50`.
- [ ] 4.5 Run the 10-episode `libero_object` X-VLA module-native live policy stream gate with pass condition `success_rate > 0.50` after the benchmark stage passes.
- [ ] 4.6 Save and reference both X-VLA gate artifact directories, including the passing benchmark artifact with videos at `artifacts/benchmark/lerobot-xvla-libero-benchmark-10-image-flip-videos/` and the pending live-path diagnostics.

## 5. Regression validation and docs

- [x] 5.1 Run existing focused VLA-JEPA policy rollout tests to confirm no regression in the current gate.
- [x] 5.2 Run X-VLA-focused backend, contract, policy module, policy chunk task, and benchmark script tests.
- [x] 5.3 Run OpenSpec validation for `add-xvla-libero-policy-gate` in strict mode.
- [x] 5.4 Update developer documentation to describe VLA-JEPA vs X-VLA policy selection, local dependency setup, and expected gate commands.
