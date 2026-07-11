## 1. Runtime Session Seam

- [x] 1.1 Define a module-native policy evaluation runtime session interface for reset, step, runtime description, cleanup, and latest stream snapshots.
- [x] 1.2 Implement a LIBERO runtime-module session adapter that reads deterministic runtime-module image and runtime-event snapshots.
- [x] 1.3 Add fake runtime-session test helpers that model reset, step, stream snapshots, missing-stream cases, and cleanup without HTTP payload methods.

## 2. Evaluation Refactor

- [x] 2.1 Refactor `BenchmarkPolicyEvalRunner` to consume stream snapshots directly when building robot policy observations.
- [x] 2.2 Remove `RuntimeClient.payload()` and synthetic `data_ref` dependencies from policy rollout evaluation code.
- [x] 2.3 Keep episode lifecycle, policy reset, action adaptation, success gate, artifact writing, and optional video output behavior unchanged.
- [x] 2.4 Update rollout artifact records to report observed stream names from the module-native snapshot seam.

## 3. Demo Cleanup

- [x] 3.1 Update `demo_lerobot_libero_policy_rollout.py` to use the new runtime session adapter instead of its local payload compatibility shim.
- [x] 3.2 Remove or explicitly deprecate HTTP-only runtime CLI flags that no longer affect module-native rollout execution.
- [x] 3.3 Ensure the demo still passes native LIBERO action mode, camera width, camera height, benchmark, init-state, and video options into the runtime module path.

## 4. Tests and Validation

- [x] 4.1 Update policy rollout unit tests for direct stream snapshot observation building and missing-stream failure handling.
- [x] 4.2 Update runtime/demo tests so no test double implements or asserts `payload()` behavior.
- [x] 4.3 Run targeted policy rollout and runtime sidecar tests.
- [x] 4.4 Run ruff on changed policy rollout, runtime adapter, and demo files.
- [x] 4.5 Validate `streamline-policy-eval-runtime-seam` with OpenSpec strict validation.
