## Context

The archived `add-live-policy-stream-rollout` change established a module-native live policy stream path for LIBERO:

```text
LIBERO runtime observations
        │
        ▼
RobotPolicyModule.policy_observation
        │
        ▼
async backend chunk inference
        │
        ▼
RobotPolicyModule.policy_action_chunk
        │
        ▼
ControlCoordinator.robot_policy_action_chunk
        │
        ▼
PolicyChunkControlTask → LIBERO native runtime step
```

That path is architecturally policy-oriented, but the current concrete LeRobot backend imports VLA-JEPA directly and the live parity gate is named and configured around `lerobot/VLA-JEPA-LIBERO`. X-VLA gives us a useful second policy because it is a separate LeRobot policy class/checkpoint (`lerobot/xvla-libero`) with LIBERO evaluation support, while still targeting the same LIBERO observation/action surface: agent-view image, wrist image, robot state, task language, and normalized 7D action chunks.

## Goals / Non-Goals

**Goals:**

- Support X-VLA LIBERO as a real LeRobot policy option without replacing the existing VLA-JEPA path.
- Preserve the module-native live topology: deployed runtime module, deployed `RobotPolicyModule`, deployed `ControlCoordinator`, DimOS streams, and RPC refill trigger.
- Keep the policy boundary action-chunk based so ControlCoordinator continues to own receding-horizon execution and stale-action behavior.
- Add a two-stage 10-episode `libero_object` X-VLA acceptance gate: synchronous benchmark first, then module-native live parity, each with `success_rate > 0.50`.
- Keep optional LeRobot/X-VLA dependencies outside the main project dependencies; use the local sidecar/`uv run --with ...` pattern for real gate execution.

**Non-Goals:**

- Do not train or fine-tune X-VLA.
- Do not replace the VLA-JEPA gate or lower its acceptance criteria.
- Do not add a new DimOS CLI command.
- Do not make ControlCoordinator policy-specific or teach the policy module physical controller mappings.
- Do not add PI0/PI0Fast as part of this change; those are intentionally excluded because they are too large for the intended local gate.

## Decisions

### Decision 1: Treat X-VLA as a second real policy gate, not just a checkpoint swap

X-VLA should run through the same live policy stream path and produce the same style of `RobotPolicyActionChunk`, but the implementation should not pretend every LeRobot checkpoint uses `VLAJEPAPolicy`. The gate must exercise policy class selection/import, processor setup, metadata, chunk inference, and diagnostics with a non-VLA-JEPA LeRobot policy class.

Alternatives considered:

- **Only change `--checkpoint`**: rejected because the current backend imports VLA-JEPA directly and would not prove backend policy agnosticism.
- **Add a completely separate benchmark script**: rejected because it would weaken the topology parity claim.

### Decision 2: Prefer a configurable LeRobot LIBERO backend seam, with family-specific import adapters

The backend should support a policy-family selector such as `vla_jepa` or `xvla`, mapping each family to its policy class import, dependency hint, and default checkpoint. This keeps `RobotPolicyModule` configuration stable while acknowledging that LeRobot policy families may live in different modules and require different extras.

```text
backend_type="lerobot"
backend_params={
  "policy_family": "xvla",
  "checkpoint_id": "lerobot/xvla-libero",
  "device": "cuda",
  "use_action_chunk": true,
}
```

Alternatives considered:

- **Separate `lerobot_xvla` backend type**: simpler first patch, but duplicates processor/device/chunk handling and makes future policy-family additions noisier.
- **Fully dynamic import from checkpoint config only**: attractive long term, but riskier because error messages and optional dependency guidance become less explicit.

### Decision 3: Reuse the existing LIBERO contract if X-VLA semantics match

The current `vla_jepa_libero` contract maps DimOS robot policy observations to LeRobot LIBERO batch keys and validates `libero.ee_delta_6d_gripper.normalized.v1` 7D outputs. X-VLA should reuse this contract if its checkpoint processor expects the same observation keys and emits the same action shape/range. If X-VLA requires different key names, action-mode metadata, domain id, or prompt formatting, introduce a narrow `xvla_libero` contract rather than contaminating the VLA-JEPA contract with conditional behavior.

Alternatives considered:

- **Rename the existing contract immediately to `lerobot_libero`**: good eventual direction, but it can hide policy-specific semantic drift unless X-VLA is first verified against the same mapping.
- **Always create an X-VLA contract**: safer isolation, but unnecessary if the checkpoint-saved LeRobot processor handles policy-specific prompt/action-mode details and the existing LIBERO observation/action semantics match.

### Decision 4: Use a two-stage X-VLA gate

Use the same `libero_object` 10-episode slice and hard pass condition `success_rate > 0.50` for two sequential checks:

1. **Synchronous benchmark stage**: validates that X-VLA can run through the fast lockstep benchmark path, including runtime reset/snapshot, observation assembly, backend inference, action adaptation, scoring, artifacts, and cleanup.
2. **Module-native live stage**: validates that the same policy family can run through deployed `RobotPolicyModule`, stream-delivered observations/chunks, ControlCoordinator refill RPC, and policy chunk task execution.

The second stage should not be considered acceptance-ready unless the first stage has passed for the same policy family/checkpoint and comparable episode slice. This separates policy/backend/contract correctness from live-control topology correctness and makes failures easier to localize.

Both stages should write separate artifacts naming the policy family/checkpoint and stage so VLA-JEPA, X-VLA benchmark, and X-VLA live evidence do not overwrite each other.

### Decision 5: Add smoke validation before the expensive real gate

Because X-VLA dependency shape, chunk method availability, and processor expectations are less locally proven than VLA-JEPA, implementation should include a fake/unit path plus a short real-policy smoke before the full 10-episode gate:

1. Import/load smoke: policy class resolves and checkpoint metadata can be described.
2. Contract smoke: one LIBERO observation converts into backend input accepted by processors.
3. Inference smoke: `predict_action_chunk` or equivalent chunk output is finite and shape-compatible.
4. Synchronous benchmark gate: 10 episodes with `success_rate > 0.50`.
5. Module-native live gate: 10 episodes with `success_rate > 0.50`.

## Risks / Trade-offs

- **X-VLA optional dependencies differ from VLA-JEPA** → Keep dependency guidance family-specific and run real gates with `uv run --with ...` rather than modifying main `pyproject.toml`.
- **X-VLA does not expose `predict_action_chunk` under the same method name** → Detect method availability in backend tests and fail with a clear unsupported-policy message; do not silently fall back to single-action live execution unless the spec is updated.
- **X-VLA action semantics differ despite 7D LIBERO output** → Add or select an X-VLA-specific contract if smoke tests show different keys, action mode, or prompt handling.
- **X-VLA is too heavy or locally unstable** → Keep the change scoped so VLA-JEPA remains the primary existing gate; record setup/integration aborts separately from policy failures.
- **A single episode gives misleading signal** → Use the 10-episode gate for acceptance in both benchmark and live stages, not a cherry-picked single episode.
- **Live failures can obscure policy failures** → Require the synchronous benchmark stage first so backend/contract/policy viability is established before diagnosing live-control topology.

## Migration Plan

1. Add policy-family selection behind the existing backend/contract registry seams.
2. Add tests and documentation for selecting VLA-JEPA vs X-VLA without changing the live topology.
3. Run existing VLA-JEPA focused tests to confirm no regression.
4. Run X-VLA smoke validation.
5. Run the X-VLA 10-episode `libero_object` synchronous benchmark gate and save artifacts.
6. Run the X-VLA 10-episode `libero_object` live policy stream gate and save artifacts.

Rollback is straightforward: keep VLA-JEPA code paths and remove or disable the X-VLA family configuration/gate if X-VLA cannot be supported locally.

## Open Questions

- Which LeRobot installation extra is required for X-VLA in the current LeRobot release (`xvla`, broader VLA extra, or source install without extra)?
- Does `lerobot/xvla-libero` expose `predict_action_chunk`, or does it require a family-specific multi-step inference method?
- Does the checkpoint-saved processor fully encode X-VLA action mode/domain id, or must DimOS pass explicit X-VLA contract metadata?
- Should the existing `vla_jepa_libero` contract be renamed after X-VLA proves semantic compatibility, or should it remain as-is for backward clarity?
