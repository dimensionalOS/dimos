## 1. X-VLA backend capability

- [ ] 1.1 Add a LeRobot policy-family selection seam that preserves the existing VLA-JEPA default while allowing `policy_family="xvla"`.
- [ ] 1.2 Add X-VLA policy class import/loader support with a family-specific dependency/setup error message.
- [ ] 1.3 Ensure backend metadata reports policy family, checkpoint id, resolved policy class, device, episode reset support, and chunk inference support for X-VLA.
- [ ] 1.4 Preserve existing VLA-JEPA backend behavior and tests while adding focused unit tests for X-VLA family selection and clear missing-dependency failures.

## 2. X-VLA LIBERO contract and chunk semantics

- [ ] 2.1 Verify whether `lerobot/xvla-libero` uses the existing LIBERO observation keys, language metadata, 8D state, and normalized 7D action surface.
- [ ] 2.2 Reuse the existing LIBERO contract if X-VLA semantics match; otherwise add a narrow X-VLA LIBERO contract through the contract registry.
- [ ] 2.3 Add contract tests proving finite X-VLA `N x 7` chunk output converts to `RobotPolicyActionChunk` with action-space id `libero.ee_delta_6d_gripper.normalized.v1`.
- [ ] 2.4 Add rejection tests for incompatible X-VLA output shape, non-finite values, or unsupported action semantics.

## 3. Script and configuration surface

- [ ] 3.1 Extend `scripts/benchmarks/demo_lerobot_libero_policy_rollout.py` so callers can select VLA-JEPA or X-VLA policy family/checkpoint without changing the live topology.
- [ ] 3.2 Add script/configuration support for a two-stage X-VLA gate: synchronous benchmark stage followed by module-native live stage.
- [ ] 3.3 Ensure fake/fixed backend tests continue to cover module-native live stream wiring without requiring X-VLA dependencies.
- [ ] 3.4 Add script tests proving X-VLA configuration can run the synchronous benchmark path without ControlCoordinator policy chunk execution.
- [ ] 3.5 Add script tests proving X-VLA configuration deploys `RobotPolicyModule`, `ControlCoordinator`, stream remapping, and policy trigger RPC through the same live path.
- [ ] 3.6 Ensure X-VLA artifacts include policy family, checkpoint id, stage name, inference method/status counts when applicable, chunk counts, consumed actions, stale deactivations, and cleanup status.

## 4. Smoke validation and two-stage real gate

- [ ] 4.1 Document the X-VLA dependency invocation pattern without adding X-VLA dependencies to the main `pyproject.toml`.
- [ ] 4.2 Run an X-VLA import/load smoke that verifies policy class resolution, checkpoint description, processor setup, and device placement.
- [ ] 4.3 Run an X-VLA inference smoke that verifies finite chunk output with shape compatible with the LIBERO 7D action surface.
- [ ] 4.4 Run the 10-episode `libero_object` X-VLA synchronous benchmark gate with pass condition `success_rate > 0.50`.
- [ ] 4.5 Run the 10-episode `libero_object` X-VLA module-native live policy stream gate with pass condition `success_rate > 0.50` after the benchmark stage passes.
- [ ] 4.6 Save and reference both X-VLA gate artifact directories, including benchmark summary artifacts and per-episode live-path diagnostics.

## 5. Regression validation and docs

- [ ] 5.1 Run existing focused VLA-JEPA policy rollout tests to confirm no regression in the current gate.
- [ ] 5.2 Run X-VLA-focused backend, contract, policy module, policy chunk task, and benchmark script tests.
- [ ] 5.3 Run OpenSpec validation for `add-xvla-libero-policy-gate` in strict mode.
- [ ] 5.4 Update developer documentation to describe VLA-JEPA vs X-VLA policy selection, local dependency setup, and expected gate commands.
