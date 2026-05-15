# Robot imitation learning stack: teleop + recording + policy

> Draft — cherry-picked from `cc/teleop-override` (1-week trial) onto `cc/learning`.
> Excludes openspec docs, agent-skill scaffolding, deprecated visualizers, and personal tooling from the source branch.

## Summary

This PR lands the three pillars of an imitation-learning loop on Dimos, wired end-to-end on the Piper:

1. **Teleop** — Pink-IK driver fed by Quest VR or keyboard, with safety guards and per-robot config presets.
2. **Recording** — Rerun-native, per-episode `.rrd` recorder with an operator-driven episode boundary on a Quest button.
3. **Policy learning** — `PolicyNode` that runs a learned manipulation policy on the same command surface as teleop, with a LeRobot inference backend, a per-robot `RobotContract` abstraction, and an `.rrd → LeRobotDataset v2` converter.

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

- **Recorder** (`dimos/visualization/rerun/recorder.py`, new — 307L):
  - `RerunDataRecorder` is a standalone Module that owns its own `rr.RecordingStream` + `rr.FileSink`, independent of the live-viewer bridge. It writes **one `.rrd` per episode**.
  - State machine: starts **IDLE** (no disk writes). `toggle_recording()` flips between `RECORDING` ↔ `IDLE`. `_open_episode()` opens a fresh `.rrd`; `_close_episode()` flushes, closes, and removes empty files.
  - Subscribes to the same pubsubs as the live bridge, applies the same converter contracts → guaranteed schema alignment.
  - `_log_episode_metadata()` writes `/meta/*` entities (start time, contract id, robot name, etc.) so downstream consumers don't need out-of-band metadata.
  - Bridge failures don't affect recording; recorder failures don't affect viewing.
- **Episode boundary** (`dimos/teleop/quest/episode_boundary.py`, new — 106L):
  - `EpisodeBoundary` is a Quest-specific Module that watches a configurable button field (default: `right_secondary`, i.e. the B button) and dispatches edges to `recorder.toggle_recording()` with 500 ms debounce.
  - Keeps teleop-specific UX out of the generic recorder — the recorder remains reusable for non-teleop blueprints (e.g. a future autonomous-rollout recorder).
- **Live preview** (`dimos/teleop/quest/data_collection_vis.py`, new):
  - `joint_state_to_rerun_scalars()` — per-joint scalar time-series, measured vs. commanded overlay.
  - `piper_data_collection_rerun_blueprint()` — camera left / joint plots right.
  - **The preview and the recorder share the same `piper_data_collection_rerun_config()`** — single source of truth for what's logged.
- **Quest blueprints** (`dimos/teleop/quest/blueprints.py`):
  - `teleop_quest_piper_data_collection` chains `CameraModule.blueprint()` → `RerunDataRecorder` → live viewer → `EpisodeBoundary`.
  - `teleop_quest_xarm7` / `teleop_quest_piper` include hardware viz via `ManipulationModule.blueprint()` (xArm7 gated on `is_xarm7_mock_preview`; Piper always on).
- **README** (`dimos/teleop/quest/README.md`): documents three runtime modes — mock Meshcat (default), real hardware IP, MuJoCo (`--simulation`).
- **Removed**: the old `PiperDataRecorder` (`dimos/teleop/quest/data_collection.py`) and its test. It manually multiplexed `color_image`/`joint_state`/`desired_joint_action` into a `TimedSensorStorage` directory; the new design delegates recording to the generic Rerun recorder and pulls episode boundaries out into its own module.
- **Tests**: `test_recorder.py`, `test_episode_boundary.py`, `test_data_collection_integration.py`, `test_data_collection_vis.py`, `test_blueprints.py`.

### Pillar 3 — Policy learning

A runtime policy node that swaps the teleop driver for a trained model, plus the dataset pipeline needed to train it.

- **Policy node** (`dimos/manipulation/policy/node.py`, new — 401L):
  - `PolicyNode` is a Module that assembles multi-camera images, joint state, task description, and Quest button streams into a `PolicyObservation`, runs inference on its own thread at `PolicyNodeConfig.policy_rate`, and publishes `JointState` on the `joint_command` output — **same port shape as teleop**, so the rest of the stack doesn't know which driver is in control.
  - Key methods: `assemble_observation()`, `_inference_loop()`, `_tick_once()` (calls `backend.select_action()` → `PolicyCommand`).
  - **Teleop preempt**: whenever any button in `teleop_engage_buttons` goes high, the backend is reset and publication pauses — buffered policy actions are dropped. Lets the operator yank control instantly.
- **Backend registry** (`dimos/manipulation/policy/registry.py`):
  - Public API: `register_backend()`, `create_backend()`, `available_backends()`, `is_registered()`.
  - Built-ins (auto-registered on `backends` import):
    - `"test"` (`backends/test.py`) — `TestPolicy`, a dependency-free sinusoidal stand-in for wiring checks.
    - `"lerobot"` (`backends/lerobot.py`) — `LeRobotBackend` wraps the LeRobot inference path. Lazy-imports `lerobot` only in `initialize()` so the module is importable without the `manipulation` extra installed. Accepts either a `RobotContract` instance or a registered contract name (e.g. `"piper"`).
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
- **Dataset converter** (`scripts/datasets/rrd_to_lerobot.py`, new — 515L):
  - CLI that converts per-episode `.rrd` files (written by `RerunDataRecorder`) into LeRobot v2 dataset directories. One `.rrd` becomes exactly one LeRobot episode.
  - Frames gate on camera arrival (~30 Hz); joint state and action use Rerun's latest-at semantics.
  - Configurable: contract, FPS, gripper binarization, output location.
- **Docs** (user-facing, new):
  - `docs/usage/datasets.md` (158L) — end-to-end `.rrd → LeRobotDataset v2` guide, gripper binarization options, contract selection.
  - `docs/usage/visualization.md` (114L) — recording semantics (entity layout, IDLE/RECORDING states, B-button toggle, debounce, empty-file removal) and training workflows (converter CLI, experimental Rerun dataloader, Rerun catalog server for dataset review).
- **Dependency**: `lerobot >= 0.5.1; python_version >= '3.12'` added to the `manipulation` extra. See **Install notes** above for the perception/manipulation conflict.
- **Tests** (~1900 lines under `dimos/manipulation/policy/`):
  - `test_node.py` (326L), `test_node_blueprint_wiring.py` (114L)
  - `test_blueprint.py`, `test_contract.py`, `test_contracts_registry.py`
  - `test_piper_contract.py` (298L), `test_registry.py`, `test_test_policy.py`
  - `test_rrd_to_lerobot_cli.py` (171L), `test_rrd_to_lerobot_integration.py` (211L)

### Supporting — Piper hardware adapter fixes

Long-tail correctness fixes on `dimos/hardware/manipulators/piper/adapter.py` that block reliable teleop + recording. Confirmed against the Piper SDK demo flow.

- **Gripper stroke units** — read/write paths now use the SDK's native units (0.001 mm, `GRIPPER_STROKE_UNITS_PER_M = 1_000_000`) instead of percentage. Values are clamped to max opening. This was the root cause of the gripper not reaching commanded positions.
- **Gripper-init sequencing** — `_gripper_initialized` flag tracks the SDK two-step (status `0x02` clear + `0x01` enable) so the gripper is properly armed before first command.
- **Graceful shutdown** — `_move_to_zero_position()` polls until within `PIPER_SHUTDOWN_JOINT_TOLERANCE_RAD` (0.03 rad) of zero, then `_deactivate_gripper()` sends status `0x02` before `DisablePiper()`. Prevents the arm from collapsing on Ctrl-C.
- **Tests**: `dimos/hardware/manipulators/piper/test_adapter.py` (161L).

### Supporting — Module registry plumbing

- `dimos/core/global_config.py` (+ `test_global_config.py`) — `simulation_backend: SimulationBackend = "mujoco"` field (replaces retired viser config fields).
- `dimos/robot/all_blueprints.py` (+ `test_get_all_blueprints.py`) — registers `policy-node`, `rerun-data-recorder`, `episode-boundary`, `teleop-quest-piper-data-collection`, and the `PIPER_ARM_FK_MODEL` rename.
- `dimos/robot/cli/dimos.py` — `inspect.isclass()` guard so path-typed model configs survive the arg-help/parser path.
- `dimos/robot/config.py` — `RobotConfig.get_task_config()` accepts an `end_effector_frame` override.

## Out of scope (deliberately not in this PR)

- **Rerun URDF visualizer** (`dimos/visualization/rerun/urdf_robot.py`) — superseded by `ManipulationModule(enable_viz=True)`, dropped from this PR.
- **Openspec / agent-skill artifacts** from the trial branch — kept on `cc/teleop-override` for reference, not merged.

## Next steps (improving demo performance)

The teleop loop, recording pipeline, and policy node are functional end-to-end on the Piper, but three concrete improvements should land before this becomes a daily-driver demo. Listed in suggested order of impact.

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

## File map

### New files

| File | Lines | Purpose |
|---|---|---|
| **Pillar 1 — Pink IK** | | |
| `dimos/control/tasks/pink_teleop_task.py` | (large) | Pink IK task hierarchy — unified `SingleArmPinkIKTask` + per-robot config presets |
| `dimos/control/tasks/test_pink_teleop_task.py` | new | Pink IK task unit tests |
| **Pillar 2 — Recording** | | |
| `dimos/visualization/rerun/recorder.py` | 307 | `RerunDataRecorder` (per-episode `.rrd`) |
| `dimos/visualization/rerun/test_recorder.py` | 372 | Recorder lifecycle + I/O tests |
| `dimos/teleop/quest/episode_boundary.py` | 106 | Quest button → `toggle_recording()` |
| `dimos/teleop/quest/test_episode_boundary.py` | 127 | Debounce + edge-detection tests |
| `dimos/teleop/quest/data_collection_vis.py` | ~200 | Live Rerun preview blueprint |
| `dimos/teleop/quest/test_data_collection_vis.py` | ~170 | Preview blueprint tests |
| `dimos/teleop/quest/test_data_collection_integration.py` | 281 | End-to-end recording integration |
| `dimos/teleop/quest/test_blueprints.py` | 271 | Quest blueprint integration tests |
| **Pillar 3 — Policy** | | |
| `dimos/manipulation/policy/node.py` | 401 | `PolicyNode` runtime module |
| `dimos/manipulation/policy/contracts/piper.py` | 302 | `PiperRobotContract` + `GripperBinarization` |
| `dimos/manipulation/policy/backends/lerobot.py` | 293 | `LeRobotBackend` |
| `dimos/manipulation/policy/contract.py` | 160 | `RobotContract` ABC |
| `dimos/manipulation/policy/backends/test.py` | 124 | `TestPolicy` (sinusoidal stand-in) |
| `dimos/manipulation/policy/blueprint.py` | 100 | Policy node blueprint |
| `dimos/manipulation/policy/config.py` | 93 | `PolicyNodeConfig` |
| `dimos/manipulation/policy/__init__.py` | 87 | Public surface |
| `dimos/manipulation/policy/backend.py` | 72 | Backend protocol |
| `dimos/manipulation/policy/registry.py` | 69 | Backend registry |
| `dimos/manipulation/policy/command.py` | 68 | `PolicyCommand` / `JointPositionCommand` |
| `dimos/manipulation/policy/observation.py` | 59 | `PolicyObservation` |
| `dimos/manipulation/policy/contracts/registry.py` | 50 | Contract registry |
| `dimos/manipulation/policy/backends/__init__.py` | 45 | Auto-register built-ins |
| `dimos/manipulation/policy/test_node.py` | 326 | Policy node lifecycle + preempt tests |
| `dimos/manipulation/policy/test_piper_contract.py` | 298 | Contract round-trip tests |
| `dimos/manipulation/policy/test_rrd_to_lerobot_integration.py` | 211 | End-to-end converter test |
| `dimos/manipulation/policy/test_rrd_to_lerobot_cli.py` | 171 | Converter CLI tests |
| `dimos/manipulation/policy/test_blueprint.py` | 133 | Blueprint wiring |
| `dimos/manipulation/policy/test_node_blueprint_wiring.py` | 114 | Node + blueprint integration |
| `dimos/manipulation/policy/test_test_policy.py` | 99 | TestPolicy backend |
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
| `dimos/teleop/quest/blueprints.py` | New data-collection blueprint, episode boundary wiring, hw viz, right-controller mapping |
| `dimos/teleop/quest/quest_teleop_module.py` | Match new blueprint contract |
| `dimos/teleop/quest/README.md` | Document runtime modes |
| `dimos/teleop/README.md` | Reference Quest teleop modes from the top-level teleop docs |
| `dimos/web/robot_web_interface.py` | Configurable Quest bind host |
| `dimos/hardware/sensors/camera/module.py` | Nested `WebcamConfig` |
| `docs/usage/visualization.md` | Recording semantics + training workflows |
| **Supporting** | |
| `dimos/hardware/manipulators/piper/adapter.py` | Gripper unit fix, graceful shutdown, init sequencing |
| `dimos/core/global_config.py` | `simulation_backend` field |
| `dimos/core/test_global_config.py` | `simulation_backend` + listen-host defaults |
| `dimos/robot/all_blueprints.py` | Register policy node, recorder, episode boundary, data-collection blueprint |
| `dimos/robot/test_get_all_blueprints.py` | Registry smoke test |
| `dimos/robot/cli/dimos.py` | `inspect.isclass()` guard for path-typed model configs |
| `pyproject.toml` | New deps: `pin-pink`, `qpsolvers`, `daqp`, `lerobot` (manipulation extra); perception/manipulation marked mutually exclusive |

### Deleted files

| File | Replaced by |
|---|---|
| `dimos/teleop/quest/data_collection.py` (66L) | `dimos/visualization/rerun/recorder.py` + `dimos/teleop/quest/episode_boundary.py` |
| `dimos/teleop/quest/test_data_collection.py` (55L) | `test_recorder.py` + `test_episode_boundary.py` + `test_data_collection_integration.py` |

## Test plan

- [ ] `uv run pytest dimos/control/tasks/test_pink_teleop_task.py dimos/control/test_control.py`
- [ ] `uv run pytest dimos/teleop/quest/test_blueprints.py dimos/teleop/quest/test_episode_boundary.py dimos/teleop/quest/test_data_collection_vis.py dimos/teleop/quest/test_data_collection_integration.py`
- [ ] `uv run pytest dimos/visualization/rerun/test_recorder.py`
- [ ] `uv run pytest dimos/manipulation/policy/` (full policy package suite)
- [ ] `uv run pytest dimos/hardware/manipulators/piper/test_adapter.py`
- [ ] `uv run pytest dimos/core/test_global_config.py dimos/robot/test_get_all_blueprints.py`
- [ ] Mock smoke: `uv run dimos run teleop-quest-piper` — Quest connects and drives the arm via Pink IK
- [ ] Mock smoke: `uv run dimos run teleop-quest-piper-data-collection` — `.rrd` files appear under the recording dir on each B-button toggle; Rerun preview shows live camera + joint plots
- [ ] xArm7 mock: `uv run dimos run teleop-quest-xarm7 -o is_xarm7_mock_preview=true`
- [ ] Policy smoke (test backend): `uv run dimos run policy-node -o backend=test` — joint commands flow on `joint_command` at the configured `policy_rate`; pressing the engage button on Quest preempts and stops publication
- [ ] Dataset converter: `uv run python scripts/datasets/rrd_to_lerobot.py <ep.rrd> --contract piper --out /tmp/lerobot_ep` — produces a LeRobotDataset v2 directory
- [ ] (On hardware) Piper gripper opens to commanded width; arm zeros + gripper deactivates cleanly on Ctrl-C
- [ ] (Optional, requires checkpoint) Policy smoke (lerobot backend): `uv run dimos run policy-node -o backend=lerobot -o backend_kwargs.pretrained_path=<path>` — trained policy drives the arm; teleop engage preempts cleanly
