## Reference implementation map

### Code-policy reference

Reference tip: `cc/feat/code-as-policy-interface` at `06a77a26c`.

The reusable implementation is the final Jupyter-backed form introduced by
`ff025d1fb` and refined through `06a77a26c`:

| Reference path | Current destination | Treatment |
| --- | --- | --- |
| `dimos/agents/code_policy.py` | `dimos/agents/code_policy.py` | Port the generic `CodePolicyConfig`, bounded plain-text collector, lazy kernel lifecycle, bootstrap of `app` and `memory`, execution lock, timeout interrupt/restart, `python_exec`, and module cleanup. Add attempt-scoped session identity, structured execution records, and a non-skill `reset_session()` RPC. |
| `dimos/agents/test_code_policy.py` | `dimos/agents/test_code_policy.py` | Port the focused collector, one-skill, persistence, exception, timeout, busy, and cleanup tests. Extend them for reset and evidence identity. |
| `docs/capabilities/agents/code-policy.md` | same path | Port only generic execution, observability, and trust-boundary documentation. Replace the manipulation demo and direct-tool routing discussion with the evaluation use case. |
| `pyproject.toml`, `uv.lock` | same paths | Add only `ipykernel>=7,<8` and `jupyter-client>=8,<9` to the existing agents dependency surface if they are not already present, then regenerate the lock. |

Explicitly excluded:

- the xArm agentic blueprint, manipulation prompt, and blueprint registry changes;
- the manipulation policy recorder and all object-aggregation changes;
- Memory2 core refactors unrelated to opening the configured recorder database;
- manipulation, Viser, and execution-manager work from the branch history;
- the reference behavior that leaves direct skills visible to the model.

### DimSim PR 3249 reference

Reference head: `2f13260068f3264ef6382dd511a571c4ddb5f40a`.
The branch is a behavioral reference only.

| Reference behavior/path | Current destination | Treatment |
| --- | --- | --- |
| `dimos/robot/unitree/dimsim_connection.py` sensor transports and duplicate suppression | current file of the same name | Restore `/odom`, `/lidar`, and `/color_image` subscriptions using current `LCMTransport`, forwarding samples into the cached observables and current TF chain. |

## Verification status

- Focused Python verification: 133 passed, 1 live-oracle test deselected.
- Pi adapter verification: 25 passed, including TypeScript compilation.
- Ruff check, Ruff format check, `git diff --check`, and the blueprint registry
  check in CI mode pass.
- Repository mypy reaches six pre-existing errors in
  `dimos/stream/audio/node_simulated.py` and
  `dimos/benchmark/spatial/questions.py`; no reported error is in this change.
- A real subscription-authenticated attempt completed at
  `/tmp/dimsim-agent-eval-real/attempts-final/attempt_2b374fac02c948f2a3cbe70e59b4a4bd`.
  Infrastructure and required evidence completed successfully; the task
  failed normally after the 180-second episode deadline. The native evaluator
  recorded a final distance of 3.385 m.
- The attempt retained 25 successful `python_exec` calls under one Pi session
  and one code-policy session. The native Pi transcript exposed only
  `python_exec`; the complete attached MCP inventory was retained separately.
  Artifact sizes and SHA-256 digests match the manifest, lifecycle sequence
  numbers are contiguous, and the selected private entity/revision/digest
  values are absent from the Pi prompts and transcript.
- The live probe exposed and corrected current-stack integration defects:
  stream type mismatch, internal-agent-only skill composition, unrelated
  speech/web dependencies, unstable startup oracle capture, fixed 30-second
  scene timeouts, MCP readiness timeout handling, exact applied-pose
  comparison after tolerance validation, missing local Pi agent cwd, and
  unreaped Pi adapter children after startup timeout.
- Actionable navigation gap: spatial memory resolved every bathtub query to
  the reset pose, so the planner immediately declared arrival while the native
  evaluator remained 3.385 m away. The stack also repeatedly reported missing
  `world -> camera_optical` and `world -> base_link` transforms. Correcting
  DimSim TF/observation localization is follow-up navigation work, not an
  evaluation-infrastructure failure.
- The first real Pi model-runtime cold start took about 315 seconds. A warm
  retry started Pi in 1.6 seconds. Manual smoke configuration should allow a
  600-second readiness/session-start window when exercising a cold local
  subscription cache.
| `misc/DimSim/cli/bridge/physics.ts` `/cmd_vel`, collision-aware physics, deadman stop, odometry | pinned external DimSim bridge plus current `GO2Connection.cmd_vel` transport | Reuse the pinned bridge's existing `/cmd_vel` and `/odom` behavior. Configure the current blueprint transport explicitly; do not copy the historical TypeScript bridge into DimOS. |
| `misc/DimSim/cli/bridge/eval-reset.ts` and protocol reset messages | `dimos/benchmark/agent_eval/dimsim/` adapter plus the smallest typed `SceneClient` controls | Preserve authoritative pose application, motion clearing, actual-pose acknowledgement, and correlation. Use current browser/control primitives and verify the resulting live oracle view. |
| `misc/DimSim/evals/protocol.ts` | evaluation backend models | Preserve attempt/operation correlation, readiness, reset, start, result, abort, cleanup, terminal status, failure stage, duration, and evidence. Do not preserve workflow URLs, internal-agent output, or legacy compatibility messages. |
| `misc/DimSim/evals/rubrics.ts` | private DimSim navigation evaluator | Preserve private authoritative geometry lookup and metric evidence, but replace title substring matching, point-to-AABB distance, and browser workflow configuration with the generated exact entity ID and robot-footprint polygon contract. Add contract-owned velocity/dwell semantics. |
| `dimos/simulation/dimsim/unitree_skill_container.py` | current normal skill containers and navigation modules | Use it only as evidence that repeated zero commands and fresh odometry are necessary. Do not port its private relative-motion skill or duplicate the current navigation stack. |
| `dimos/simulation/dimsim/go2_connection.py` fresh-frame wait | evaluation observation layer if needed | Preserve only the post-motion freshness property through current timestamps/recorder queries; do not subclass the current Go2 connection solely for evaluation. |

Explicitly excluded:

- `agentic_blueprint.py`, internal `McpClient`, agent driver/output sidecars,
  turn-control code, and task runner;
- historical spatial-memory, navigation, manipulation, and visualization modules;
- vendored `misc/DimSim` source and scene assets;
- workflow JavaScript imports, substring-based semantic lookup, agent-output
  success conditions, and duplicate task definitions.

## Current interface probe

The current stack establishes these authoritative interfaces:

| Concern | Current interface |
| --- | --- |
| RGB | `DimSim` publishes `/color_image`; `DimSimConnection.video_stream()` feeds `GO2Connection.color_image: Out[Image]`. |
| Point cloud | `DimSim` publishes `/lidar`; `DimSimConnection.lidar_stream()` feeds `GO2Connection.lidar: Out[PointCloud2]`, which is the stream consumed by the current mapper. |
| Odometry and TF | `/odom` carries `PoseStamped`; `DimSimConnection.odom_stream()` feeds `GO2Connection._publish_tf()`, its `odom: Out[PoseStamped]`, and the `world -> base_link -> camera` TF chain. |
| Map | `VoxelGridMapper` consumes the Go2 lidar stream and publishes the current global map; `CostMapper` publishes `global_costmap`. |
| Spatial memory | `SpatialMemory` consumes `color_image` and TF-backed pose state; `Go2Memory`/a dedicated `Recorder` is the persistent agent-visible observation boundary. |
| Navigation status | `ReplanningAStarPlanner.get_state()` is the authoritative RPC; `goal_reached` is the useful terminal stream. The declared `navigation_state` output is not currently populated and must not be treated as authoritative. |
| Motion command | `ReplanningAStarPlanner.nav_cmd_vel` and `MovementManager.cmd_vel` feed `GO2Connection.cmd_vel: In[Twist]`. The evaluation blueprint must assign `LCMTransport("/cmd_vel", Twist)` so the pinned DimSim bridge receives commands. |
| Cancellation | `ReplanningAStarPlanner.cancel_goal()` is the authoritative navigation RPC. Terminal cleanup also publishes repeated `Twist.zero()` commands across the `/cmd_vel` transport because cancellation alone does not prove the simulator command has cleared. |

The checked-out pinned upstream DimSim revision
`050aa1ec96e71524cdc9d84575ba5bc46d82993e` was inspected locally. It contains
server-side `/cmd_vel` physics with a deadman timeout, `/odom`, `/lidar`,
`/color_image`, and a control-channel teleport primitive. It does not contain
PR 3249's correlated reset/evaluation protocol or native velocity-dwell rubric.
Those missing lifecycle semantics stay in the new private adapter; the adapter
uses the smallest current control primitives, emits repeated zero velocity,
acknowledges the observed applied pose, and validates a fresh oracle view. The
release remains bound to the exact pinned upstream revision.

## Generated destination versus episode binding

The generated release owns semantic task identity and every truth-bearing
predicate field:

- scene, upstream, semantic-profile, semantic-schema, reset, generator,
  predicate-policy, and template revisions;
- opaque task identity and exact public instruction;
- exact bathtub entity ID;
- robot-footprint-to-target-outer-footprint metric and one-metre threshold;
- linear/angular stopped tolerances and stationary dwell duration;
- canonical embodiment footprint and profile-selected reset pose.

The local evaluation configuration owns only operational context:

- release root and selected task ID;
- output root and attached-service endpoints;
- Pi authentication binding, model, and thinking configuration;
- readiness/reset/tool-call timeouts and the 180-second episode deadline;
- artifact and trusted-local execution settings.

It cannot override a generated target, metric, threshold, stopped-state
parameter, source revision, or reset profile.
