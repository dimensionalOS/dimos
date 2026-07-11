## Context

Simulator runtime execution has moved from HTTP sidecars to DimOS `Module` RPCs plus typed streams. The minimal merge kept policy rollout working by adapting module stream outputs back into the existing policy-evaluation `RuntimeClient` shape, including synthetic `data_ref` values and a `payload()` method. That compatibility layer is intentionally temporary: it allows the merge to stay small, but it preserves HTTP-era vocabulary and control flow in a code path that now uses placed runtime modules.

Policy rollout needs two kinds of data per tick:

- Synchronous reset/step metadata: reward, done, success, runtime description, and action protocol validation.
- Stream snapshots: camera images, camera metadata, and robot-state events published by the runtime module.

The current seam mixes these by modeling stream payloads as if they were HTTP payload references. This change makes the module-native split explicit.

## Goals / Non-Goals

**Goals:**

- Replace the HTTP-shaped `RuntimeClient.payload()` dependency with an explicit module-native runtime session or observation snapshot interface.
- Keep `BenchmarkPolicyEvalRunner` responsible for episode lifecycle, policy reset, action conversion, success gate, artifacts, and optional videos.
- Keep `RobotPolicyModule` focused on inference only.
- Keep the LeRobot LIBERO rollout demo on native LIBERO runtime actions and the placed `LiberoProRuntimeModule` path.
- Preserve current artifact compatibility where useful: summary, episodes JSONL, runtime description, checkpoint metadata, videos, and cleanup status.

**Non-Goals:**

- Redesign DimOS transport internals or image transport performance.
- Change the VLA-JEPA LIBERO contract semantics or action space.
- Reintroduce an HTTP sidecar server or HTTP payload endpoint.
- Require real LIBERO/LeRobot dependencies in normal unit tests.

## Decisions

### Use a module-native runtime session seam

Create a runtime session abstraction for policy evaluation that owns the placed runtime module connection and exposes reset/step plus latest stream snapshots. For lockstep policy evaluation, the placed runtime module should also expose a deterministic latest-snapshot readback captured on the runtime owner side when it publishes stream values. The runner should receive observation values directly from the session instead of resolving `ObservationFrame.data_ref` through `payload()`.

Alternative considered: keep synthetic `data_ref` values indefinitely. This avoids touching the runner, but keeps the wrong abstraction and makes future stream transport work harder to reason about.

### Keep stream snapshot assembly outside the policy module

The runtime module should publish `Image`, `CameraInfo`, and `ObservationFrame` events for stream consumers, and should retain the latest observation arrays/metadata for lockstep readback by the runtime session. The existing `LiberoRobotPolicyObservationBuilder` can then convert snapshot values and runtime metadata into `RobotPolicyObservation`.

Alternative considered: let `RobotPolicyModule` subscribe to runtime streams directly. That would couple policy inference to benchmark runtime lifecycle and break the intended boundary where benchmark evaluation owns rollout orchestration.

### Treat video capture as a consumer of stream snapshots

Video writing should use the same image arrays that feed policy observations. This keeps video artifacts and policy input aligned and avoids a second data path.

Alternative considered: keep separate video payload resolution through `data_ref`. That preserves duplicate data paths and hides when videos and policy observations diverge.

### Remove HTTP-only rollout flags deliberately

Once the runtime session seam is in place, `--runtime-host`, `--runtime-port`, `--startup-timeout-s`, and `--timeout-s` should be removed or explicitly deprecated if backward CLI compatibility is needed. Module runtime configuration should be expressed through runtime placement/environment options, camera options, benchmark selection, and rollout limits.

Alternative considered: keep the flags as no-ops. That avoids breaking old command lines but leaves misleading knobs in the CLI.

## Risks / Trade-offs

- **Risk:** Stream callbacks and synchronous RPC responses may race in deployed workers. → **Mitigation:** capture latest stream snapshots on the runtime owner side and have policy evaluation read that deterministic snapshot after each reset or step.
- **Risk:** Removing `payload()` may require changes to fake runtime tests and helper objects. → **Mitigation:** update test doubles to implement the new session/snapshot seam directly, not an HTTP compatibility surface.
- **Risk:** CLI flag removal can break copied commands. → **Mitigation:** either keep deprecated aliases for one cycle with a warning, or document the replacement in the rollout docs.
- **Risk:** Snapshot naming may collapse multiple cameras if frame IDs are absent. → **Mitigation:** use `Image.frame_id` when present and require configured camera names for policy rollout.

## Migration Plan

1. Introduce the module-native runtime session/snapshot interface alongside the existing compatibility shim.
2. Update `BenchmarkPolicyEvalRunner` and tests to consume stream snapshots directly.
3. Update the LeRobot LIBERO rollout demo to use the new seam and remove synthetic payload generation.
4. Remove or deprecate HTTP-only rollout CLI flags.
5. Remove the old `payload()` compatibility path and stale docs/spec wording.
