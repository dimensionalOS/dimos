## Context

DimOS already has two major foundations for robot learning rollout work:

- Memory2 provides typed named streams, SQLite persistence, timestamps, optional pose metadata, blob-backed payloads, tags, live queries, replay, and stream alignment.
- The ControlCoordinator provides a deterministic tick loop, coordinator-owned hardware writes, passive control tasks, joint-state publishing, per-joint arbitration, and preemption notifications.

The missing piece is a learning-specific contract that connects recording, dataset export, and policy rollout without sacrificing raw stream fidelity. Today, a developer can record streams and separately execute trajectories, but there is no shared robot contract that defines how raw recorded streams become LeRobot training rows, how live streams become policy inputs, and how policy actions return to coordinator-native commands.

The first implementation target is manipulation with joint trajectory action chunks. This matters because the initial target fixes the most safety-sensitive and schema-sensitive parts of the design: joint ordering, gripper representation, camera/state features, trajectory timing, chunk preemption, and ControlCoordinator integration. Mobile/base policies can reuse the same frame-alignment and Memory2/export concepts later, but would choose different action surfaces such as `Twist`.

## Goals / Non-Goals

**Goals:**

- Define a manipulation-first learning rollout architecture using Memory2 as the raw recording substrate and ControlCoordinator as the hardware-control boundary.
- Preserve raw typed streams as the canonical recording artifact, then derive policy/export views through a shared robot contract.
- Support fixed-FPS LeRobot export from Memory2 recordings without requiring LeRobot during normal robot operation.
- Represent policy output as joint trajectory action chunks for the first version.
- Ensure preempted chunks are dropped as whole chunks and reported as preempted/dropped, not resumed or partially replayed.
- Document episode storage alternatives in Memory2 and leave the final storage decision open until implementation/spec work resolves operational needs.

**Non-Goals:**

- Direct LeRobot writing during live recording.
- Direct hardware writes from policy inference modules.
- Training pipeline implementation, model registry, or policy optimization logic.
- Full mobile/quadruped action support in the first implementation.
- Hard real-time policy inference at coordinator tick rate.
- Replacing existing Memory2 recorder, replay, or ControlCoordinator abstractions.

## DimOS Architecture

The design should use five layers. The important boundary is between temporal stream assembly and robot/LeRobot schema conversion:

```text
Live typed streams / raw Memory2 streams
        │
        ▼
Raw episode recording (Memory2 streams)
        │
        ├── lossless replay/query/latest-at/find-closest
        ▼
TemporalSampleAssembler
        │
        ├── aligns async streams by timestamp/tolerance
        ├── handles missing/held/optional inputs
        ▼
RobotContract.to_lerobot / from_lerobot
        │
        ├── Offline LeRobot exporter
        └── RobotPolicyModule / PolicyRolloutModule
                 │
                 ▼
          RobotAction / action chunk adapter
                 │
                 ▼
           ControlCoordinator task
```

### Recording layer

Manipulation rollout recording should build on `dimos.memory2.module.Recorder` and `SqliteStore`. A rollout recorder declares typed `In[...]` ports for the streams required by the selected robot/blueprint and writes them to Memory2 with timestamps, pose metadata when available, and episode metadata according to the final episode-storage decision.

Memory2 is the canonical raw artifact. The recorder must not resample, pack, or flatten live streams into LeRobot-style rows before storage. It should preserve native stream frequency, raw timestamps, typed payloads, and sparse metadata so later consumers can re-project the data without losing information.

Initial manipulation observation streams should include:

- `joint_state: JointState` from the ControlCoordinator or manipulation hardware state surface.
- One or more camera `Image` streams, with stable camera names.
- Optional pose/odometry, gripper state, force/torque, or task context streams when available.

Initial manipulation action streams should include:

- executed or commanded joint trajectory chunk records;
- action chunk status records;
- gripper command/state when included in the action vector.

The recorder should preserve raw stream timestamps. It should not force LeRobot row shape during live recording.

### Episode storage data shape

Episode storage needs one logical metadata shape even if the physical Memory2 layout remains open. The logical shape should be a small envelope that can be written as a typed metadata stream, persisted as recording-level metadata, or both:

```text
RolloutEpisode
  episode_id: str                  # stable within dataset/session
  dataset_id: str | None           # collection or training dataset grouping
  robot_id: str | None             # robot serial/name or mock/sim identifier
  robot_contract: str              # contract key used for projection/export
  task: str                        # natural language or task label
  operator: str | None             # optional local/user label when available
  started_at_ns: int               # monotonic or wall-clock source documented by store
  ended_at_ns: int | None
  status: "recording" | "success" | "aborted" | "cancelled" | "failed"
  status_reason: str | None
  stream_names: dict[str, str]     # logical role -> Memory2 stream name
  tags: list[str]
```

The raw observation/action streams remain independent streams. The episode metadata only defines membership and status. A sample manipulation episode can therefore have:

```text
/episodes                  RolloutEpisodeEvent or RolloutEpisode snapshot
/joint_state               JointState
/gripper_state             GripperState or robot-specific typed payload
/camera/wrist/image        Image
/camera/overhead/image     Image
/action                    JointState or robot-specific typed payload for commanded or executed action
/task_text                 Text/task metadata payload, if not only in episode metadata
```

Three physical layouts are viable:

#### Strategy A: one Memory2 DB/file per episode

```text
rollouts/
  dataset.yaml or manifest.json
  episodes/
    ep_000001.memory2.sqlite
      recording_metadata: {dataset_id, episode_id, robot_contract, task, status, ...}
      streams:
        /joint_state
        /camera/wrist/image
        /camera/overhead/image
        /action
        /episode
    ep_000002.memory2.sqlite
```

- Best for: demos collected as retryable units, simple deletion, simple upload/download, and operational isolation.
- Weakness: dataset-level queries and bulk export must iterate many DBs/files and reconcile repeated stream schemas.
- Export lookup shape: the exporter reads `dataset.yaml`, opens each episode DB, validates its metadata, then exports each successful episode independently.

#### Strategy B: one Memory2 DB/file per collection session with episode events

```text
rollouts/
  session_2026_06_09_teleop.memory2.sqlite
    recording_metadata: {session_id, dataset_id, robot_id, robot_contract, operator, ...}
    streams:
      /episodes              # start/stop/status events or snapshots keyed by episode_id
      /joint_state
      /camera/wrist/image
      /camera/overhead/image
      /action
```

`/episodes` contains records such as:

```text
{episode_id: "ep_000001", event: "start", started_at_ns: 1000, task: "pick cube"}
{episode_id: "ep_000001", event: "stop", ended_at_ns: 9000, status: "success"}
{episode_id: "ep_000002", event: "start", started_at_ns: 12000, task: "pick cube"}
{episode_id: "ep_000002", event: "stop", ended_at_ns: 15000, status: "aborted", status_reason: "operator_cancel"}
```

- Best for: teleop collection sessions where many attempts share robot, operator, cameras, and calibration context.
- Weakness: partial deletion/retry requires tombstones or copy/compact tooling; boundary events must be robust under abnormal shutdown.
- Export lookup shape: the exporter scans `/episodes`, slices raw streams by `[started_at_ns, ended_at_ns]`, and exports only selected statuses.

#### Strategy C: one dataset-level Memory2 DB/file with an episode index

```text
rollouts/
  dataset_pick_cube.memory2.sqlite
    recording_metadata: {dataset_id, schema_version, robot_contracts, collection_notes, ...}
    streams:
      /episode_index         # durable snapshots keyed by episode_id
      /sessions              # optional session metadata keyed by session_id
      /joint_state/<robot_or_session>
      /camera/wrist/image/<robot_or_session>
      /action/<robot_or_session>
```

`/episode_index` can store complete episode membership records:

```text
{
  episode_id: "ep_000123",
  session_id: "session_2026_06_09_teleop",
  interval_ns: [1000, 9000],
  status: "success",
  task: "pick cube",
  robot_contract: "xarm7_v1",
  stream_names: {
    "joint_state": "/joint_state/session_2026_06_09_teleop",
    "wrist_image": "/camera/wrist/image/session_2026_06_09_teleop",
    "action_chunk": "/policy_action_chunk/session_2026_06_09_teleop"
  }
}
```

- Best for: dataset curation, cross-episode queries, split assignment, bulk validation, and schema migrations across a collection.
- Weakness: largest failure domain, hardest partial deletion, and more complex stream naming to avoid collisions across sessions or robots.
- Export lookup shape: the exporter reads `/episode_index`, validates each episode's stream mapping, then slices shared streams by indexed intervals.

The recommended first implementation should prefer Strategy B unless implementation uncovers Memory2 tooling constraints. It keeps a natural collection-session boundary, avoids one-file-per-attempt explosion, and still preserves raw streams. Strategy A should remain available as a low-risk fallback for manual QA and operational simplicity. Strategy C is better treated as a later dataset curation/export optimization once episode indexing and compaction are proven.

### Temporal sample assembly layer

DimOS learning rollouts need a first-class unification step because the source data is not one synchronized observation stream. It is a set of typed asynchronous streams with different payload formats, clock rates, and arrival behavior. The design should introduce a `TemporalSampleAssembler` between raw streams and robot-specific conversion.

The assembler is the train/rollout symmetry boundary for time. It must be usable from both:

- live stream caches in `PolicyRolloutModule`; and
- offline Memory2 streams in the LeRobot exporter.

The assembler owns:

- stream role mapping, such as `joint_state`, `wrist_image`, `overhead_image`, `action`, `action_status`, and `task`;
- live latest-value caches for push-based DimOS `In[...]` streams;
- offline lookup against Memory2 streams using timestamp windows, `at(...)`, `align(...)`, or closest-sample lookup;
- per-role timestamp tolerances rather than one global tolerance;
- missing-data behavior, such as required, hold-last, optional, or skip-sample;
- readiness validation that checks stream presence, non-zero sample counts, and timestamp overlap before export;
- output timestamp selection, using the sample/export timeline timestamp as canonical for the assembled sample;
- source timestamp retention for debugging and validation.

The assembler outputs a robot-native sample, for example `RobotLearningSample` or `PolicyObservation`, containing native DimOS payloads (`JointState`, `Image`, task text, action records, and source timestamps). It must not emit LeRobot rows directly and must not flatten, normalize, or reorder robot state/action vectors. Those are robot-contract responsibilities.

For offline export, the exporter creates a fixed-FPS timeline, asks the assembler for the native sample at each export timestamp, then passes that sample to the robot contract. For live inference, the policy module uses the assembler's latest-cache path to produce the same native sample shape from recently received stream values. The assembled sample is transient and must not replace raw Memory2 recording.

### Robot contract and LeRobot conversion layer

The robot contract is the train/rollout symmetry boundary for robot semantics and the current LeRobot schema. Since LeRobot is the only first target, the first design should keep explicit LeRobot methods on the contract rather than introduce a generic format adapter framework.

A robot-specific contract, for example a `RobotContract`, owns:

- contract identity and version;
- required sample roles and expected payload types, expressed as requirements on the assembler rather than direct subscriptions or Memory2 queries;
- joint ordering and gripper representation;
- camera keys, image shapes, and image conventions;
- action vector layout;
- `features()` or equivalent LeRobot feature schema generation using plain dict/dataclass types without importing LeRobot in core runtime paths;
- `to_lerobot(sample, include_action=False)` conversion from a native assembled sample to LeRobot frame fields;
- `from_lerobot(action_vec)` conversion from a LeRobot/policy action vector to a robot-native `RobotAction` intent;
- optional normalization hooks if later required.

The contract must not own stream synchronization, buffering, timestamp lookup, Memory2 querying, policy scheduling, or hardware command execution. It is a schema and semantic conversion boundary, not a data-source orchestrator.

The linked Tea-style/reference contract pattern is useful for `features()`, `to_lerobot(...)`, and `from_lerobot(...)`. The Rerun-specific `from_rerun_row(...)` shape should not be copied directly into DimOS Memory2. Its source-specific responsibility belongs in the `TemporalSampleAssembler`, after which the contract receives one native assembled sample regardless of whether the sample came from live streams or recorded Memory2 streams.

### Policy rollout layer

`RobotPolicyModule` or `PolicyRolloutModule` should be the backend-hosting policy runtime. It owns policy/model lifecycle, rollout start/stop state, low-rate inference scheduling, backend selection, and chunk submission. It subscribes to the same typed streams used by recording, maintains latest observations, uses the temporal sample assembler to build a native sample, and delegates policy inference to a backend implementation.

The module should support multiple backend implementations behind a common policy backend API. `LeRobotBackend` is the first implementation, but the boundary should leave room for scripted/test backends, ONNX backends, remote policy services, or other learned-policy runtimes without changing stream assembly or coordinator submission.

The common backend API should be chunk-oriented rather than raw-vector-oriented. A minimal shape is:

```text
PolicyBackend
  initialize()
  generate_action_chunk(sample: RobotLearningSample) -> RobotActionChunk
  reset()
  close()
```

For the LeRobot backend, `generate_action_chunk(...)` should call `RobotContract.to_lerobot(sample)`, run the LeRobot policy, decode policy output with `RobotContract.from_lerobot(action_vec)`, and return a robot-native action chunk. Future backends can produce the same `RobotActionChunk` without going through LeRobot.

The policy module should not itself be a LeRobot adapter. Its job is orchestration: get a current native sample, call the selected backend, validate or drop the returned chunk according to rollout state, and hand accepted chunks to coordinator-managed execution.

The policy module should submit chunks to coordinator-managed control via a module reference/RPC or a typed stream consumed by a task. It should not write manipulator adapters or low-level motor commands directly.

For the first manipulation path, accepted chunks should become `JointTrajectory` commands and be submitted to the configured ControlCoordinator task, likely through `ControlCoordinator.task_invoke(task_name, "execute", {"trajectory": trajectory})` unless a dedicated chunk task needs richer status. The conversion from `RobotActionChunk` to `JointTrajectory` may live in a small action adapter or contract-adjacent helper, but coordinator timing and preemption remain task-owned.

If a public DimOS `Spec` Protocol is needed, define one around chunk submission/status rather than model internals. The protocol should hide whether the implementation uses `task_invoke`, a stream, or a specialized task method.

### Control layer

The first action format is a joint trajectory chunk. The implementation can begin by reusing the existing trajectory task path where appropriate, but the design should allow a specialized policy chunk task if chunk identity/status cannot be represented cleanly by `JointTrajectoryTask`.

A policy chunk task should:

- implement the existing `ControlTask` pattern;
- use `CoordinatorState.t_now` for timing;
- return `JointCommandOutput` for joint positions in coordinator-owned ticks;
- expose chunk status including accepted, executing, completed, preempted, dropped, cancelled, and rejected;
- drop an entire chunk if any controlled joint is preempted;
- reject stale chunks when a newer chunk supersedes them.

The ControlCoordinator remains the hardware authority. Arbitration, timeout, cancellation, dry-run, and safety behavior should remain coordinator/task-owned.

### Export layer

LeRobot integration is exporter-only for dataset writing, but the live LeRobot policy path can use the same contract methods to build inference inputs and decode policy outputs. Normal DimOS recording should not require LeRobot imports. The offline exporter reads raw Memory2 recordings, asks the temporal sample assembler for fixed-FPS native samples, applies the robot contract's `to_lerobot(...)` conversion, and writes a LeRobot dataset with feature schema, episode metadata, task labels, image/video features, observations, and actions.

The derived LeRobot dataset is an artifact, not the source of truth. If export settings, alignment tolerance, image encoding, or feature layout need to change, developers should be able to regenerate the LeRobot dataset from the raw Memory2 recording.

The exporter should treat LeRobot as an optional dependency. Missing dependency errors should be feature-specific and should not affect core DimOS startup.

### Blueprint, CLI, skills, and registries

Blueprints should compose rollout recorders and policy modules explicitly so existing robot blueprints do not change by default. If new runnable blueprints are added, regenerate `dimos/robot/all_blueprints.py` with:

```bash
pytest dimos/robot/test_all_blueprints_generation.py
```

CLI surfaces are optional but likely useful for export and inspection, for example commands to summarize a rollout recording and export it to LeRobot. Skills/MCP exposure is not required for the first implementation, though future start/stop recording or start/stop rollout tools can wrap module RPCs.

## Decisions

1. **Manipulation first, not generic/mobile first.**
   - Rationale: joint trajectory chunks are the most concrete requirement and align with the existing ControlCoordinator trajectory task path.
   - Alternative: design a fully generic action abstraction first. This risks delaying the first usable path and hiding manipulation safety details.

2. **Joint trajectory chunks are the first action representation.**
   - Rationale: trajectories match coordinator-owned sampled control and make chunk preemption observable.
   - Alternative: dense joint positions or Cartesian chunks. These can be added later as additional chunk types once the recording/frame/export contract is stable.

3. **Raw typed Memory2 recordings are canonical; projections are derived.**
   - Rationale: Memory2 preserves raw asynchronous streams and keeps robot operation independent of LeRobot availability/versioning.
   - Alternative: store canonical rollout frames directly. This would make training rows easy to produce but would lose stream-level timing/frequency information and make later export-policy changes harder.

4. **TemporalSampleAssembler replaces canonical frame storage for stream unification.**
   - Rationale: raw streams are asynchronous and typed. A dedicated assembler can preserve train/rollout timing symmetry without making framed rows the recording format. It maps raw recording or live latest caches to one native sample at a chosen timestamp.
   - Alternative: let each robot contract query streams and align inputs directly. This couples robot semantics to Memory2/live-stream mechanics and makes future source changes harder.

5. **RobotContract owns LeRobot conversion for the first use case.**
   - Rationale: LeRobot is the only immediate format target, so explicit `features()`, `to_lerobot(...)`, and `from_lerobot(...)` methods keep the first implementation concrete while preserving one robot-specific source of truth for joint ordering, camera keys, and gripper mapping.
   - Alternative: introduce a generic format adapter framework now. This adds abstraction before a second format exists and risks hiding the first policy/export path behind indirection.

6. **Fixed export FPS applies only to derived export.**
   - Rationale: LeRobot expects fixed-FPS frame rows. Memory2 can preserve raw timestamps and the exporter can resample/align deterministically.
   - Alternative: action-stream or policy-tick clock. These are useful for live inference, but they do not by themselves guarantee LeRobot-compatible dense frame rows.

7. **RobotPolicyModule owns backend orchestration, not backend-specific inference.**
   - Rationale: the module should provide one rollout runtime for start/stop, sample assembly, backend selection, `generate_action_chunk(...)`, chunk validation, and coordinator submission. LeRobot is the first backend, but scripted/test, ONNX, remote, or other learned-policy backends should plug in behind the same action-chunk API.
   - Alternative: implement a LeRobot-only policy module. This would be faster initially but would mix rollout lifecycle, LeRobot tensor conversion, and coordinator submission in one component.

8. **Episode storage decision remains open.**
   Three alternatives should be evaluated before implementation:

   **Separate Memory2 recording per episode**
   - Pros: simple episode boundaries, easier deletion/retry, smaller failure domain, natural mapping to demonstration files.
   - Cons: harder cross-episode querying/statistics, many files at scale, more work to batch export, repeated stream registry metadata.

   **Separate Memory2 recording per collection session with an episode metadata stream**
   - Pros: preserves session context while allowing multiple episodes, supports batch export, avoids one-file-per-episode explosion, and keeps raw streams contiguous.
   - Cons: requires robust episode boundary/status records and tooling for partial episode deletion or retry.

   **Combined Memory2 recording with episode metadata**
   - Pros: easier multi-episode queries, one dataset-level artifact, better for shared stream registries and bulk export.
   - Cons: requires robust episode boundary/status metadata, harder partial deletion/retry, corruption or schema drift affects more data.

   PR #2093's per-episode `.rrd` pattern is strong evidence for operational simplicity, but Memory2 may still prefer per-session recording if query/export ergonomics are more important. The design leaves the final choice open until specs/tasks define collection UX and operational constraints.

## Safety / Simulation / Replay

Policy rollout must preserve the ControlCoordinator as the only component that writes to manipulation hardware. Policy modules generate chunks; coordinator tasks execute or reject them. This keeps arbitration, dry-run, cancellation, and preemption semantics centralized.

Preemption is safety-sensitive. If a higher-priority task wins any joint in an active policy chunk, the chunk must be considered dropped/preempted as a whole. The task must not resume the remainder of that chunk after the preempting task releases control.

Simulation and replay should use the same temporal sample assembler, robot contract, and chunk submission surfaces as hardware. Memory2 replay should be able to feed the assembler and contract for export validation and, where safe, policy dry-run validation. Manual QA should include a mock/sim manipulation coordinator, a policy chunk happy path, a preemption path, raw recording inspection, and LeRobot export of a short recorded episode.

## Risks / Trade-offs

- **Train/rollout mismatch:** If live policy inputs and exported training frames diverge, learned policies may fail. Mitigation: centralize timing/stream assembly in `TemporalSampleAssembler` and robot-specific LeRobot/action mapping in one robot contract.
- **Premature framing/data loss:** If live data is packed into frames before recording, raw stream frequency/timing information is lost. Mitigation: record raw typed streams first and treat LeRobot/policy frames as derived views.
- **Timestamp alignment ambiguity:** Raw streams are asynchronous while LeRobot export is fixed-FPS. Mitigation: preserve raw timestamps, treat assembled-sample timestamps as canonical for projections, and make per-role alignment tolerance/missing-sample behavior explicit.
- **Episode metadata drift:** If episode boundaries/status are stored inconsistently, exports become unreliable. Mitigation: evaluate separate-recording vs combined-recording trade-offs before implementation and test aborted/preempted episodes.
- **LeRobot version churn:** Dataset format and writer APIs can change. Mitigation: keep LeRobot optional and isolated in exporter code.
- **Chunk status complexity:** Existing `JointTrajectoryTask` may not expose enough chunk identity/status. Mitigation: add a specialized policy chunk task only if reuse would obscure required behavior.
- **Hardware safety:** Policy outputs can be unsafe even if well-formed. Mitigation: rely on coordinator arbitration and add validation limits at the control task boundary where required by existing robot contracts.

## Migration / Rollout

This should be additive. Existing recorders, robot blueprints, ControlCoordinator tasks, and replay paths should continue to behave unchanged unless a developer opts into rollout recording or policy rollout components.

Rollout steps:

1. Define specs for recording, frame alignment, policy rollout control, chunk preemption, and LeRobot export.
2. Implement the manipulation-first temporal sample assembler, robot contract, and raw Memory2 recording/export path.
3. Add the multi-backend robot policy module and coordinator chunk execution/status path.
4. Add optional CLI/docs for recording inspection and LeRobot export.
5. Add example/mock manipulation blueprint only if useful; regenerate blueprint registry if a runnable blueprint is added.

Rollback should be simple because the new behavior is opt-in: remove rollout modules/tasks/CLI usage from blueprints and continue using existing Memory2 and ControlCoordinator flows.

## Open Questions

- Should episodes be stored as separate Memory2 recordings, combined recordings with episode metadata, or per-session recordings with an episode metadata stream?
- Should chunk submission use `ControlCoordinator.task_invoke`, a typed stream, or a dedicated DimOS `Spec` Protocol?
- What is the initial manipulation sample schema and robot contract, including required stream roles, joint ordering, gripper representation, camera names, and action-vector layout?
- What is the exact `PolicyBackend.generate_action_chunk(...)` signature and `RobotActionChunk` shape shared by LeRobot and future backends?
- Should action chunks include only commanded future joint positions, or both commanded and actually executed positions for export?
- What per-stream alignment tolerances and missing-sample policies should the assembler use by default for live rollout and fixed-FPS export?
- Which manipulation stack should provide the first manual QA surface: mock coordinator, xArm/Piper, or simulation?
