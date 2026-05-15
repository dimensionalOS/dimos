# Robot imitation learning stack: teleop + recording + policy

> Draft — cherry-picked from `cc/teleop-override` (1-week trial) onto `cc/learning`.
> Excludes openspec docs, agent-skill scaffolding, deprecated visualizers, and personal tooling from the source branch.

## Summary

This PR lands the three pillars of an imitation-learning loop on Dimos, wired end-to-end on the Piper:

1. **Teleop** — Pink-IK driver fed by Quest VR or keyboard, with safety guards and per-robot config presets.
2. **Recording** — Rerun-native, per-episode `.rrd` recorder with an operator-driven episode boundary on a Quest button.
3. **Policy learning** — `PolicyNode` that runs a learned manipulation policy on the same command surface as teleop, with a LeRobot inference backend, a per-robot `RobotContract` abstraction, and an `.rrd → LeRobotDataset v3.0` converter.

The same robot contract (`PiperRobotContract`) is reused on both ends: when recording, it converts live Quest+camera messages into LeRobot-shaped frames; when running a trained policy, it converts the policy's action vector back into a `JointState` command on the coordinator's joint-command port. That symmetry is what closes the loop without bespoke glue at each end.

## Install notes ⚠️

The `manipulation` and `perception` extras are **mutually exclusive** in this PR:

- `manipulation` requires `lerobot >= 0.5.1`, which pulls `huggingface-hub >= 1.0`
- `perception` pins `transformers < 4.54`, which needs `huggingface-hub < 1.0`

Install them one at a time alongside `all`:

```bash
uv sync --extra all --extra manipulation   # for teleop + policy
# or
uv sync --extra all --extra perception     # for vision-language stack
```

The `all` extra has been updated to exclude both so this choice is explicit. The `manipulation` extra also requires Python ≥ 3.12 (LeRobot constraint).

## What's shipped

### Pillar 1 — Pink IK teleoperation

A new task type built on top of the [Pink](https://github.com/stephane-caron/pink) QP-based differential IK solver, plugged into the existing `ControlCoordinator` / `TickLoop`.

- **Task hierarchy** (`dimos/control/tasks/pink_teleop_task.py`):
  - `BasePinkIKTask` → `SingleFramePinkIKTask` → `SingleArmPinkIKTask` — one unified runtime class that handles all single-arm flavors. Pinocchio model loading, task arbitration, solver selection (DAQP preferred, falls back to any `qpsolvers` backend).
  - Per-robot **config presets** (no separate runtime classes): `PinkIKTaskConfig`, `SingleArmPinkIKTaskConfig`, `XArm7IKTaskConfig`, `PiperPinkIKTaskConfig`. Each preset captures end-effector frame, posture/damping costs, and gripper mapping for one robot.
- **Coordinator integration** (`dimos/control/coordinator.py`):
  - New `single_arm_pink_ik` task type registered.
  - New `desired_joint_action: Out[JointState]` output port — publishes post-arbitration joint commands (position / velocity / torque routed to the matching `JointState` field) for downstream recording and policy inputs.
  - `CARTESIAN_TARGET_TASK_TYPES` tuple unifies cartesian-target subscription.
  - New `TaskConfig` fields: `max_joint_delta_deg` (default 5°), `timeout` (default 0.5s), `end_effector_frame`.
- **Tick loop** (`dimos/control/tick_loop.py`): publishes the post-arbitration joint command each cycle as frame id `{frame_id}/desired_action`.
- **Catalog wiring**:
  - `dimos/robot/catalog/ufactory.py` → `xarm7_single_arm_pink_task_config()`
  - `dimos/robot/catalog/piper.py` → `piper_single_arm_pink_task_config()` (uses `PIPER_ARM_FK_MODEL`, renamed from `PIPER_FK_MODEL`)
- **Safeguards** in the solve loop: max joint delta per tick, target timeout (rejects stale targets), joint-name validation against the loaded Pinocchio model, IK-failure rejection with logging.
- **Quest + keyboard plumbing**:
  - Quest blueprints route Quest pose → Pink IK with `task_names={"right": "teleop_piper"}` (control on right controller).
  - `dimos/web/robot_web_interface.py` — `RobotWebInterface` accepts a `host` param, so Quest WebXR can bind to `0.0.0.0` (`--listen-host`); previously hardcoded to loopback.
  - `dimos/hardware/sensors/camera/module.py` — nested `WebcamConfig` via `-o camera.webcam.camera_index=N`.
  - `dimos/teleop/keyboard/keyboard_teleop_module.py` — `model_path` accepts `str | Path`.
- **Dependencies** (`pyproject.toml`): `pin-pink >= 4.2.0`, `qpsolvers[open-source-solvers] >= 4.12.0`, `daqp >= 0.8.5`.
- **Tests**: `dimos/control/tasks/test_pink_teleop_task.py` (lifecycle, timeout, joint-delta safety, gripper actuation, validation), `dimos/control/test_control.py` (coordinator + tick-loop integration).

### Pillar 2 — Recording (Rerun-native, per-episode)

End-to-end pipeline for capturing demonstrations while teleoperating, designed so that the live preview and the on-disk recording can never drift.

The whole recording package on `cc/learning` lives under `dimos/manipulation/data_collection/` — recorder, Quest-specific episode boundary, Piper blueprint config, and tests sit side-by-side rather than being split between `dimos/visualization/rerun/` and `dimos/teleop/quest/`.

- **Recorder** (`dimos/manipulation/data_collection/recorder.py`):
  - `RerunDataRecorder` is a standalone Module that owns its own `rr.RecordingStream` + `rr.FileSink`, independent of the live-viewer bridge. It writes **one `.rrd` per episode**.
  - State machine: starts **IDLE** (no disk writes). `toggle_recording()` flips between `RECORDING` ↔ `IDLE`. `_open_episode()` opens a fresh `.rrd`; `_close_episode()` flushes, closes, and removes empty files.
  - **Pure typed inputs** — every recorder input arrives on an `In[T]` stream slot, the same idiom `PolicyModule` uses:
    - `image: In[Image]` → logged under `/observation/camera/{camera_key}`.
    - `joint_state: In[JointState]` → logged as per-joint scalars under `/observation/state/<joint>`.
    - `desired_joint_action: In[JointState]` → logged as per-joint scalars under `/action/<joint>`.
  - No `pubsubs` / `topic_to_entity` / `visual_override` / `subscribe_all` machinery — the recorder no longer imports from the bridge, and blueprint wiring picks each input by slot name + type (same way `PolicyModule` is wired).
  - `_log_episode_metadata()` writes `/meta/*` entities (episode id, session id, operator) so downstream consumers don't need out-of-band metadata.
  - Bridge failures don't affect recording; recorder failures don't affect viewing.
- **Episode boundary** (`dimos/manipulation/data_collection/quest_episode_boundary.py`):
  - `QuestEpisodeBoundary` is a Quest-specific Module that watches a configurable button field (default: `right_secondary`, i.e. the B button) and dispatches edges to `recorder.toggle_recording()` with 500 ms debounce.
  - Keeps teleop-specific UX out of the generic recorder. The `Quest` prefix advertises the dependency on `dimos.teleop.quest.quest_types.Buttons`; a future non-Quest variant (`KeyboardEpisodeBoundary`, …) slots in alongside it.
- **Live preview + recorder config** (`dimos/manipulation/data_collection/piper_blueprint_config.py`):
  - `joint_state_to_rerun_scalars()` — per-joint scalar time-series, measured vs. commanded overlay. The recorder calls this directly from its typed-slot handlers; the bridge still uses it via `visual_override` for live viewing.
  - `piper_data_collection_rerun_blueprint()` — camera left / joint plots right.
  - `_piper_data_collection_topic_to_entity()` is now consumed only by the bridge — the recorder's data inputs no longer go through this callback.
  - The shared `piper_data_collection_rerun_config()` dict carries bridge-side keys (`visual_override`, `topic_to_entity`, `blueprint`) and recorder-side keys (`camera_key`, `record_path_factory`, `recording_id_factory`, `episode_metadata`) side-by-side; each consumer picks its subset.
- **Quest blueprints** (`dimos/teleop/quest/blueprints.py`):
  - `teleop_quest_piper_data_collection` chains `CameraModule.blueprint()` → `RerunDataRecorder` (all inputs via typed slots) → live viewer → `QuestEpisodeBoundary`. A `(RerunDataRecorder, "image", "color_image")` remapping binds the recorder's `image` slot to the `color_image` transport, mirroring the policy blueprint's identical remapping for `PolicyModule.image`.
  - `teleop_quest_xarm7` / `teleop_quest_piper` include hardware viz via `ManipulationModule.blueprint()` unconditionally.
- **README** (`dimos/teleop/quest/README.md`): documents three runtime modes — mock Meshcat (default), real hardware IP, MuJoCo (`--simulation`).
- **Tests** (all under `dimos/manipulation/data_collection/`): `test_recorder.py` (typed-slot lifecycle for image/joint_state/desired_joint_action), `test_quest_episode_boundary.py`, `test_piper_integration.py`, `test_piper_blueprint_config.py`.

### Pillar 3 — Policy learning

A runtime policy module that swaps the teleop driver for a trained model, plus the dataset pipeline needed to train it. Class is named **`PolicyModule`** to match the rest of DimOS (`CameraModule`, `ManipulationModule`, `ArmTeleopModule`, …).

- **Policy module** (`dimos/manipulation/policy/module.py`, 481L):
  - `PolicyModule` is a Module that assembles a single `image: In[Image]` slot, joint state, task description, and Quest button streams into a `PolicyObservation`, runs inference on its own thread at `PolicyModuleConfig.policy_rate`, and publishes `JointState` on the `joint_command` output — **same port shape as teleop**, so the rest of the stack doesn't know which driver is in control.
  - **One typed camera input** with a `camera_key: str = "main"` config carrying the observation-dict key the backend sees in `PolicyObservation.images`. (The multi-camera scaffolding from the original design was deliberately collapsed to a single typed slot.)
  - Key methods: `assemble_observation()`, `_inference_loop()`, `_tick_once()` (calls `backend.select_action()` → `PolicyCommand`).
  - **Rollout gate** (`QuestRolloutToggle` + `start_rollout()` / `stop_rollout()` / `is_rollout_active()` RPCs): publication is gated `False` at startup; an operator press on a Quest button (default `left_secondary`) toggles rollout, and both transitions reset the backend so no buffered chunk survives.
  - **Teleop preempt**: whenever any button in `teleop_engage_buttons` goes high, the backend is reset and publication pauses — buffered policy actions are dropped. Re-checks after `select_action()` returns so a button frame arriving mid-inference still drops the in-flight command.
  - **Mandatory buttons subscription** when `teleop_engage_buttons` is non-empty (failure propagates from `start()`); a **buttons grace period** (`buttons_grace_period: float = 2.0`) gates publication until the first Buttons frame is received, logging a warning if the period elapses first.
- **Rollout toggle** (`dimos/manipulation/policy/quest_rollout_toggle.py`):
  - Quest-button-driven Module that calls into `PolicyModule.start_rollout()` / `stop_rollout()` on a debounced rising edge. Keeps the runtime gate cleanly composable into the existing button stream.
- **Backend registry** (`dimos/manipulation/policy/registry.py`):
  - Public API: `register_backend()`, `create_backend()`, `available_backends()`, `is_registered()`.
  - Built-ins (auto-registered on `backends` import):
    - `"test"` (`backends/test.py`) — `TestPolicy`, a dependency-free sinusoidal stand-in. Closed-loop center: when no explicit center is passed, the backend captures the live joint state on each `reset()` and shifts the trajectory so `position(t=0)` equals the captured pose — engage/disengage handoff is jump-free regardless of configured phase.
    - `"lerobot"` (`backends/lerobot.py`) — `LeRobotBackend` wraps the LeRobot inference path. Lazy-imports `lerobot` only in `initialize()` so the module is importable without the `manipulation` extra installed. `_PolicyAutoLoader` prefers the concrete-class path via `get_policy_class()` because LeRobot 0.5.x's `PreTrainedPolicy.from_pretrained()` returns the abstract base and fails at instantiation.
- **Robot contract abstraction** (`dimos/manipulation/policy/contract.py`):
  - `RobotContract` (ABC) converts in three directions:
    - **rrd row → LeRobot frame dict** (offline training prep)
    - **Live `Image` + `JointState` → LeRobot frame dict** (online inference)
    - **Backend action vector → coordinator-native `JointState` command**
  - One contract definition powers both recording (data shapes) and inference (action mapping), preventing schema drift between train and deploy.
- **Piper contract** (`dimos/manipulation/policy/contracts/piper.py`, 302L):
  - `PiperRobotContract` maps the Piper teleop schema (`/observation/camera/usb`, `/observation/state/<joint>`, `/action/<joint>`) to LeRobot shapes.
  - Joint names auto-derive from the existing robot catalog factory (single source of truth).
  - `GripperBinarization` converts continuous gripper positions to binary `{open, closed}` actions (default threshold 0.7) so the policy's action space matches typical demonstration datasets.
- **Coordinator integration** (`dimos/control/blueprints/teleop.py`):
  - `coordinator_teleop_piper_with_policy` adds a third **low-priority `policy_servo_arm` `JointServoTask`** below the Pink IK teleop task on the Piper coordinator. The policy module's `joint_command` output drives this servo task through the coordinator's existing `_on_joint_command` routing.
  - `piper_policy_joint_names()` / `piper_policy_overlapping_teleop_tasks()` exports keep the joint set and the overlap declaration in one place.
- **Blueprint helpers** (`dimos/manipulation/policy/blueprint.py`):
  - `policy_servo_task_config()` auto-derives the servo task priority to be strictly below every overlapping teleop task's priority (or validates an explicit one), enforcing the teleop-preempts invariant at build time.
  - `policy_engage_buttons()` derives `teleop_engage_buttons` from the overlapping teleop tasks' `hand` fields — single-arm setups get only the relevant controller's button, never both.
- **Quest deployment blueprints** (`dimos/teleop/quest/blueprints.py`):
  - Shared atoms / transports / remappings hoisted into `_PIPER_POLICY_SHARED_ATOMS`, `_PIPER_POLICY_TRANSPORTS`, `_PIPER_POLICY_REMAPPINGS`. (Lifted to module scope so the AST blueprint detector picks them up.)
  - `teleop_quest_piper_policy` — LeRobot ACT backend, `camera_key="usb"`, runs on CPU by default.
  - `teleop_quest_piper_policy_test` — `TestPolicy` backend, identical wiring. Useful for verifying the full pipeline (teleop preempt, rollout toggle, camera plumbing, coordinator arbitration) without loading a model or requiring `lerobot`/`torch`.
- **Dataset converter** (`scripts/datasets/rrd_to_lerobot.py`, new — 515L):
  - CLI that converts per-episode `.rrd` files (written by `RerunDataRecorder`) into LeRobot v2 dataset directories. One `.rrd` becomes exactly one LeRobot episode.
  - Frames gate on camera arrival (~30 Hz); joint state and action use Rerun's latest-at semantics.
  - Configurable: contract, FPS, gripper binarization, output location.
- **Docs** (user-facing, new):
  - `docs/usage/datasets.md` (158L) — end-to-end `.rrd → LeRobotDataset v2` guide, gripper binarization options, contract selection.
  - `docs/usage/visualization.md` — recording semantics (entity layout, IDLE/RECORDING states, B-button toggle, debounce, empty-file removal) and training workflows.
- **Dependency**: `lerobot >= 0.5.1; python_version >= '3.12'` added to the `manipulation` extra. See **Install notes** above for the perception/manipulation conflict.
- **Tests** (~2300 lines under `dimos/manipulation/policy/`, 109 passing on cc/learning):
  - `test_module.py` (616L), `test_module_blueprint_wiring.py` (119L)
  - `test_blueprint.py`, `test_contract.py`, `test_contracts_registry.py`
  - `test_piper_contract.py` (298L), `test_registry.py`, `test_quest_rollout_toggle.py` (168L)
  - `test_test_policy.py`, `test_rrd_to_lerobot_cli.py` (171L), `test_rrd_to_lerobot_integration.py` (211L, skipped without the `manipulation` extra installed)

### Supporting — Piper hardware adapter fixes

Long-tail correctness fixes on `dimos/hardware/manipulators/piper/adapter.py` that block reliable teleop + recording. Confirmed against the Piper SDK demo flow.

- **Gripper stroke units** — read/write paths now use the SDK's native units (0.001 mm, `GRIPPER_STROKE_UNITS_PER_M = 1_000_000`) instead of percentage. Values are clamped to max opening. This was the root cause of the gripper not reaching commanded positions.
- **Gripper-init sequencing** — `_gripper_initialized` flag tracks the SDK two-step (status `0x02` clear + `0x01` enable) so the gripper is properly armed before first command.
- **Graceful shutdown** — `_move_to_zero_position()` polls until within `PIPER_SHUTDOWN_JOINT_TOLERANCE_RAD` (0.03 rad) of zero, then `_deactivate_gripper()` sends status `0x02` before `DisablePiper()`. Prevents the arm from collapsing on Ctrl-C.
- **Tests**: `dimos/hardware/manipulators/piper/test_adapter.py` (161L).

### Supporting — Module registry plumbing

- `dimos/core/global_config.py` (+ `test_global_config.py`) — listen-host defaults (`simulation_backend` was reverted in a follow-on commit on `cc/learning`).
- `dimos/robot/all_blueprints.py` (+ `test_get_all_blueprints.py`) — registers `policy-module`, `quest-rollout-toggle`, `rerun-data-recorder`, `quest-episode-boundary`, `teleop-quest-piper-data-collection`, `teleop-quest-piper-policy`, `teleop-quest-piper-policy-test`, and the `PIPER_ARM_FK_MODEL` rename.
- `dimos/robot/cli/dimos.py` — `inspect.isclass()` guard so path-typed model configs survive the arg-help/parser path.
- `dimos/robot/config.py` — `RobotConfig.get_task_config()` accepts an `end_effector_frame` override.

## Out of scope (deliberately not in this PR)

- **Rerun URDF visualizer** (`dimos/visualization/rerun/urdf_robot.py`) — superseded by `ManipulationModule(enable_viz=True)`, dropped from this PR.
- **Openspec / agent-skill artifacts** from the trial branch — kept on `cc/teleop-override` for reference, not merged.

## Next steps (improving demo performance)

The teleop loop, recording pipeline, and policy node are functional end-to-end on the Piper, but four concrete improvements should land before this becomes a daily-driver demo. Listed in suggested order of impact.

### 1. Teleoperation latency profiling — find the critical path

The current end-to-end path runs:

```
Quest controller pose
  → WebXR / WebSocket
  → RobotWebInterface             (dimos/web/robot_web_interface.py)
  → QuestTeleopModule              (dimos/teleop/quest/quest_teleop_module.py)
  → cartesian_command PoseStamped
  → ControlCoordinator arbitration (dimos/control/coordinator.py)
  → SingleArmPinkIKTask QP solve   (dimos/control/tasks/pink_teleop_task.py)
  → TickLoop publish               (dimos/control/tick_loop.py)
  → PiperAdapter.write_joint_positions
  → CAN frame to motor
```

Today the loop feels "rubbery" under fast motion, but we don't know which hop dominates. Plan:

- Stamp each message with monotonic timestamps as it crosses each stage (WebSocket recv, coordinator entry, post-solve, post-write).
- Render the per-stage deltas in Rerun via the same `joint_state_to_rerun_scalars()`-style scalar plots the data-collection viz already uses, so the operator sees a live latency budget alongside the live preview.
- Cross-check the tick thread with `py-spy top` / `viztracer` to catch GIL-bound stalls (especially in the Pink solve, which currently runs synchronously inside the tick).
- Likely suspects to confirm or rule out: WebSocket frame batching, Pink solve time per tick at the current task weights, blocking CAN writes in `PiperAdapter`.

**Exit criterion:** a documented per-stage latency budget (median + p99) and a named owner for the dominant stage.

### 2. Compliant control — survive hard contact with the table

The Piper runs pure position control, so a Quest operator who drives the end-effector through the table surface gets a stall + torque fault instead of a graceful yield. This is the single biggest source of "I can't let new people try the demo" friction — and it's even worse with a learned policy in the loop, where the operator can't trivially predict where the gripper will go next.

Two layers of mitigation, easiest first:

- **Workspace clip (pre-solver):** add a half-space inequality on the cartesian target before it reaches `SingleArmPinkIKTask` — clamp target z to a calibrated table-plane height plus a small margin. Implement either as a Pink task barrier (extend `BasePinkIKTask`) or as a pre-arbitration clip in `ControlCoordinator`. Cheap, robot-agnostic, no SDK dependency. Applies equally to teleop and policy actions.
- **Soft control near contact:** if the Piper SDK exposes torque or current limits per joint, drop them in the lower workspace region; otherwise reduce `max_joint_delta_deg` adaptively as the end-effector approaches the table plane.

Stretch goal: a true admittance/impedance scheme once we have an F/T sensor or wrist load estimate — out of scope for the immediate demo.

**Exit criterion:** a new operator can intentionally drive the gripper into the table (in teleop or via a misbehaving policy) and the arm yields without faulting; the recording continues.

### 3. Better preempt strategy — action chunks as a first-class type

Today `PolicyNode` emits one action per inference call and `ControlCoordinator` arbitrates one cartesian target per tick. That works, but it leaves performance on the table: learned policies (ACT, Diffusion Policy, etc.) generate **chunks** — a horizon of `H` future actions per inference call — and benefit from playing the whole chunk out smoothly between inferences. Without a chunk-aware layer, we either invoke the policy at tick rate (expensive, wasteful), or discard the rest of each chunk after the first sample (lossy, jittery).

Proposal:

- Introduce a `Trajectory[JointState]` (or `Trajectory[PoseStamped]`) message — an ordered list of samples with a nominal `dt`, plus an issue timestamp.
- Either extend `ControlCoordinator` to accept trajectories alongside the current per-tick targets, or insert a thin **chunk buffer** layer in front of it that plays the trajectory out tick-by-tick into the existing port.
- **Preemption semantics:** when a newer chunk arrives (from teleop engaging, from a new inference, or from a safety stop), discard the entire in-flight chunk and switch to the new one immediately. No interpolation or splicing — the whole chunk is the unit of commitment.
- Maps cleanly onto the existing teleop-preempt logic in `PolicyNode`: a teleop engage event simply drops the current chunk and resumes per-tick targets.

Open questions to settle during implementation:
- Does the buffer interpolate between samples at tick rate, or just step through at the chunk's nominal `dt`?
- Where does chunk-level arbitration live when multiple producers exist (teleop overriding a policy mid-rollout)?
- What's the right backpressure signal to the policy producer when chunks are arriving faster than they're consumed?

**Exit criterion:** existing teleop and per-action policy paths still work unchanged; an ACT-style policy emitting chunks at 10 Hz drives the arm smoothly at the 100 Hz tick rate; manual teleop preempts the policy cleanly mid-chunk.

### 4. Dynamic module inputs — let tasks and sensors declare their own ports

Today every `Module` declares its `In[T]` / `Out[T]` ports as static class annotations and the framework instantiates them via `get_type_hints()` introspection at `__init__` time (`dimos/core/module.py`). That assumption shows up in two places that already pinch as the demo grows beyond the single-arm + single-camera Piper:

- **Coordinator inputs are fixed.** `ControlCoordinator` exposes exactly four input ports — `joint_command`, `cartesian_command`, `twist_command`, `buttons` (`dimos/control/coordinator.py`). A new task type can only piggy-back on those: `single_arm_pink_ik` already overloads `cartesian_command` and uses `frame_id` as the task name to route. A dual-arm IK task, a hybrid cartesian + posture task, or anything that wants a streaming auxiliary target has to invent more in-band conventions on top of the same four ports.
- **Cameras are hardcoded.** `CameraModule` is camera-count-agnostic, but the wiring isn't: the literal `_CAMERA_KEY = "usb"` in `dimos/manipulation/data_collection/piper_blueprint_config.py`, the single `Spatial2DView` in the live layout, the single `CameraModule.blueprint()` instance in the Quest blueprint, the single typed `image: In[Image]` slot on `RerunDataRecorder` (and the matching slot on `PolicyModule`), and the contract's single `/observation/camera/usb` key all assume one camera. Adding a wrist or RealSense view requires coordinated edits across all of those plus `PiperRobotContract`.

Two paths, picking one closes both gaps:

- **Option A — dynamic input ports per active component.** Extend `Module` (or, more conservatively, `ControlCoordinator`, `RerunDataRecorder`, and `PolicyModule` individually) so additional `In[T]` ports can be registered at `__init__` from a config — one per active task for the coordinator, one per declared camera for the recorder and policy module. Requires teaching the introspection layer in `dimos/core/module.py` (and the blueprint atom extractor in `dimos/core/coordination/blueprints.py`) that the port set isn't fully known from class hints alone. Smallest change to the user surface; biggest change to the framework's invariants.
- **Option B — in-process module composition.** Make each task and each camera its own `Module` with its own static ports, and introduce a "subgraph" abstraction that lets a parent (the coordinator, the recorder, the policy module) own a set of child modules and expose their ports as part of its own surface. Blueprint wiring already happens by topic+type matching, so the transport layer needs no change — what's new is the parent/child ownership and a single canonical naming scheme for child-port topics (`<parent>/<child>/<port>`). Bigger conceptual change, but it keeps `Module` itself a static-introspection thing and pushes all the "what's active" decisions into blueprint composition where they already live.

Either path also unblocks two follow-ons that are awkward today: per-task replays (a task with its own ports records its own demonstrations), and policy contracts that don't have to enumerate camera keys ahead of time.

**Exit criterion:** adding a second camera (wrist + scene) to the Piper data-collection blueprint, and adding a posture-target stream to a new variant of `SingleArmPinkIKTask`, are each a single-file blueprint edit — no changes to `Module`, `ControlCoordinator`, `RerunDataRecorder`, or `PolicyModule` source.

## File map

### New files

| File | Lines | Purpose |
|---|---|---|
| **Pillar 1 — Pink IK** | | |
| `dimos/control/tasks/pink_teleop_task.py` | (large) | Pink IK task hierarchy — unified `SingleArmPinkIKTask` + per-robot config presets |
| `dimos/control/tasks/test_pink_teleop_task.py` | new | Pink IK task unit tests |
| **Pillar 2 — Recording** (all under `dimos/manipulation/data_collection/`) | | |
| `dimos/manipulation/data_collection/recorder.py` | 291 | `RerunDataRecorder` (per-episode `.rrd`; pure typed `image` / `joint_state` / `desired_joint_action` slots + `camera_key`) |
| `dimos/manipulation/data_collection/test_recorder.py` | 430 | Recorder lifecycle + I/O + typed-camera tests |
| `dimos/manipulation/data_collection/quest_episode_boundary.py` | 111 | `QuestEpisodeBoundary` — Quest button → `toggle_recording()` |
| `dimos/manipulation/data_collection/test_quest_episode_boundary.py` | 127 | Debounce + edge-detection tests |
| `dimos/manipulation/data_collection/piper_blueprint_config.py` | 292 | Live Rerun preview config + recorder atoms (single source of truth) |
| `dimos/manipulation/data_collection/test_piper_blueprint_config.py` | 257 | Preview / config tests |
| `dimos/manipulation/data_collection/test_piper_integration.py` | 281 | End-to-end recording integration |
| `dimos/teleop/quest/test_blueprints.py` | 271 | Quest blueprint integration tests |
| **Pillar 3 — Policy** | | |
| `dimos/manipulation/policy/module.py` | 481 | `PolicyModule` runtime module |
| `dimos/manipulation/policy/contracts/piper.py` | 302 | `PiperRobotContract` + `GripperBinarization` |
| `dimos/manipulation/policy/backends/lerobot.py` | 302 | `LeRobotBackend` |
| `dimos/manipulation/policy/backends/test.py` | 187 | `TestPolicy` (closed-loop sinusoid) |
| `dimos/manipulation/policy/contract.py` | 160 | `RobotContract` ABC |
| `dimos/manipulation/policy/blueprint.py` | 119 | `policy_servo_task_config` + `policy_engage_buttons` |
| `dimos/manipulation/policy/quest_rollout_toggle.py` | 117 | `QuestRolloutToggle` Module |
| `dimos/manipulation/policy/config.py` | 95 | `PolicyModuleConfig` |
| `dimos/manipulation/policy/__init__.py` | 94 | Public surface |
| `dimos/manipulation/policy/backend.py` | 72 | Backend protocol |
| `dimos/manipulation/policy/registry.py` | 69 | Backend registry |
| `dimos/manipulation/policy/command.py` | 68 | `PolicyCommand` / `JointPositionCommand` |
| `dimos/manipulation/policy/observation.py` | 59 | `PolicyObservation` |
| `dimos/manipulation/policy/contracts/registry.py` | 50 | Contract registry |
| `dimos/manipulation/policy/backends/__init__.py` | 45 | Auto-register built-ins |
| `dimos/manipulation/policy/test_module.py` | 616 | Policy module lifecycle + preempt + rollout tests |
| `dimos/manipulation/policy/test_piper_contract.py` | 298 | Contract round-trip tests |
| `dimos/manipulation/policy/test_rrd_to_lerobot_integration.py` | 211 | End-to-end converter test (skipped without `manipulation` extra) |
| `dimos/manipulation/policy/test_blueprint.py` | 192 | Blueprint priority + engage-button wiring |
| `dimos/manipulation/policy/test_test_policy.py` | 190 | TestPolicy backend (closed-loop center) |
| `dimos/manipulation/policy/test_quest_rollout_toggle.py` | 168 | Rollout enable/disable transitions |
| `dimos/manipulation/policy/test_rrd_to_lerobot_cli.py` | 171 | Converter CLI tests |
| `dimos/manipulation/policy/test_module_blueprint_wiring.py` | 119 | Module + blueprint integration |
| `dimos/manipulation/policy/test_contract.py` | 96 | Contract ABC |
| `dimos/manipulation/policy/test_registry.py` | 87 | Backend registry |
| `dimos/manipulation/policy/test_contracts_registry.py` | 61 | Contract registry |
| `scripts/datasets/rrd_to_lerobot.py` | 515 | `.rrd → LeRobotDataset v2` CLI |
| `docs/usage/datasets.md` | 158 | User guide for dataset conversion |
| **Supporting** | | |
| `dimos/hardware/manipulators/piper/test_adapter.py` | 161 | Piper SDK adapter tests |

### Modified files

| File | Purpose |
|---|---|
| **Pillar 1** | |
| `dimos/control/coordinator.py` | New `single_arm_pink_ik` task type, `desired_joint_action` output, cartesian-target unification |
| `dimos/control/tick_loop.py` | Publish post-arbitration joint commands |
| `dimos/control/test_control.py` | Coordinator + tick-loop tests for the new task type |
| `dimos/control/blueprints/teleop.py` | Wire Pink IK into teleop blueprints |
| `dimos/robot/catalog/ufactory.py` | `xarm7_single_arm_pink_task_config()` |
| `dimos/robot/catalog/piper.py` | `piper_single_arm_pink_task_config()` |
| `dimos/robot/catalog/openarm.py` | Small follow-on to the catalog binding refactor |
| `dimos/robot/manipulators/piper/blueprints.py` | `PIPER_FK_MODEL` → `PIPER_ARM_FK_MODEL` |
| `dimos/robot/config.py` | `end_effector_frame` task-config override |
| `dimos/teleop/keyboard/keyboard_teleop_module.py` | `model_path: str \| Path` |
| **Pillar 2** | |
| `dimos/teleop/quest/blueprints.py` | Data-collection + two policy deployment blueprints; episode boundary wiring; shared Piper-policy atoms / transports / remappings |
| `dimos/teleop/quest/quest_teleop_module.py` | Match new blueprint contract |
| `dimos/teleop/quest/README.md` | Document runtime modes |
| `dimos/teleop/README.md` | Reference Quest teleop modes from the top-level teleop docs |
| `dimos/web/robot_web_interface.py` | Configurable Quest bind host |
| `dimos/hardware/sensors/camera/module.py` | Nested `WebcamConfig` |
| `docs/usage/visualization.md` | Recording semantics + training workflows |
| **Pillar 3** | |
| `dimos/control/blueprints/teleop.py` | `coordinator_teleop_piper_with_policy` + `piper_policy_joint_names()` / `piper_policy_overlapping_teleop_tasks()` exports |
| **Supporting** | |
| `dimos/hardware/manipulators/piper/adapter.py` | Gripper unit fix, graceful shutdown, init sequencing |
| `dimos/core/global_config.py` | Listen-host config (simulation_backend reverted on cc/learning) |
| `dimos/core/test_global_config.py` | Listen-host defaults |
| `dimos/robot/all_blueprints.py` | Register `policy-module`, `quest-rollout-toggle`, recorder, `quest-episode-boundary`, data-collection blueprint, two policy deployment blueprints |
| `dimos/robot/test_get_all_blueprints.py` | Registry smoke test |
| `dimos/robot/cli/dimos.py` | `inspect.isclass()` guard for path-typed model configs |
| `pyproject.toml` | New deps: `pin-pink`, `qpsolvers`, `daqp` (manipulation extra); `lerobot>=0.5.1; python_version >= '3.12'` (manipulation extra); perception/manipulation marked mutually exclusive; `manipulation` removed from `all` |

### Deleted files

| File | Replaced by |
|---|---|
| `dimos/teleop/quest/data_collection.py` (66L) | `dimos/manipulation/data_collection/recorder.py` + `dimos/manipulation/data_collection/quest_episode_boundary.py` |
| `dimos/teleop/quest/test_data_collection.py` (55L) | `test_recorder.py` + `test_quest_episode_boundary.py` + `test_piper_integration.py` (all under `dimos/manipulation/data_collection/`) |

## Test plan

- [ ] `uv run pytest dimos/control/tasks/test_pink_teleop_task.py dimos/control/test_control.py`
- [ ] `uv run pytest dimos/manipulation/data_collection/` (recorder typed-camera + lifecycle + piper integration + episode boundary)
- [ ] `uv run pytest dimos/teleop/quest/test_blueprints.py`
- [ ] `uv run pytest dimos/manipulation/policy/` (full policy package suite — 109 tests passing on cc/learning)
- [ ] `uv run pytest dimos/hardware/manipulators/piper/test_adapter.py`
- [ ] `uv run pytest dimos/core/test_global_config.py dimos/robot/test_get_all_blueprints.py`
- [ ] Top-level import smoke: `uv run python -c "from dimos.manipulation.policy import PolicyModule, PolicyModuleConfig, QuestRolloutToggle"`
- [ ] Mock smoke: `uv run dimos run teleop-quest-piper` — Quest connects and drives the arm via Pink IK
- [ ] Mock smoke: `uv run dimos run teleop-quest-piper-data-collection` — `.rrd` files appear under the recording dir on each B-button toggle; Rerun preview shows live camera + joint plots
- [ ] xArm7 mock: `uv run dimos run teleop-quest-xarm7`
- [ ] Policy smoke (test backend): `uv run dimos run teleop-quest-piper-policy-test` — joint commands flow on `joint_command` at the configured `policy_rate`; pressing `left_secondary` toggles rollout; right teleop engage preempts and stops publication
- [ ] Dataset converter: `uv run python scripts/datasets/rrd_to_lerobot.py --input <session_dir_or_ep.rrd> --contract piper --task "pick the block" --output /tmp/lerobot_ep` — produces a LeRobotDataset v2 directory
- [ ] (On hardware) Piper gripper opens to commanded width; arm zeros + gripper deactivates cleanly on Ctrl-C
- [ ] (Optional, requires checkpoint) Policy smoke (lerobot backend): `uv run dimos run teleop-quest-piper-policy -o policymodule.backend_config.policy_path=<path>` — trained policy drives the arm; teleop engage preempts cleanly
