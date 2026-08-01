## Context

DimOS teleoperation currently has device-specific modules such as Quest teleop that publish directly to coordinator-facing streams (`PoseStamped`, `Twist`, and `Buttons`). Manipulator control flows through `ControlCoordinator`, whose stable generic inputs include `joint_command: In[JointState]`, `coordinator_cartesian_command: In[PoseStamped]`, `twist_command: In[Twist]`, and `teleop_buttons: In[Buttons]`.

OpenArm Mini is a physical leader-arm teleoperator. For v1 it will only teleoperate OpenArm followers, so DimOS does not need to introduce a new coordinator array-command input or make OpenArm Mini robot-agnostic. The OpenArm Mini integration should still avoid duplicating common DimOS teleop lifecycle, publishing, and structural safety logic.

LeRobot's OpenArm Mini implementation is a useful behavioral reference, but DimOS should not depend on LeRobot at runtime. The OpenArm Mini adapter will depend on the lower-level Feetech motor communication library and implement the small amount of OpenArm Mini mapping/calibration logic directly.

## Goals / Non-Goals

**Goals:**

- Add a reusable `TeleopModule` shell that can host device-specific teleop adapters and publish to the existing `ControlCoordinator` inputs.
- Add a `TeleopAdapter` contract with `connect()`, `disconnect()`, and `get_current_command()`.
- Represent adapter output with a command envelope that distinguishes active commands, no command/no authority, and explicit stop commands.
- Enforce one primary motion output per adapter instance: `JointState`, `PoseStamped`, or `Twist`.
- Implement OpenArm Mini → OpenArm joint mirror teleop by emitting follower `JointState` commands.
- Keep OpenArm Mini runtime startup non-interactive and fail fast when required calibration artifacts are missing.
- Provide a manual calibration/demo script for OpenArm Mini leader setup.
- Keep existing Quest teleop implementation unchanged in v1.

**Non-Goals:**

- Do not migrate Quest teleop to the new `TeleopModule` in v1.
- Do not add a LeRobot runtime dependency.
- Do not add a new `ControlCoordinator` array-command input in v1.
- Do not make OpenArm Mini a generic robot-agnostic leader in v1; the adapter owns OpenArm-specific mapping.
- Do not make calibration a required method on all teleop adapters.
- Do not start follower OpenArm hardware or `ControlCoordinator` from the calibration/demo script.

## Decisions

### Introduce `TeleopModule` plus `TeleopAdapter`

`TeleopModule` is the DimOS module shell. It owns module lifecycle, periodic command retrieval, structural safety, and publishing to coordinator-facing outputs. `TeleopAdapter` is the device-specific bridge from a human teleoperation source into coordinator-native command objects.

The adapter interface is intentionally small:

```python
class TeleopAdapter(Protocol):
    primary_output: TeleopPrimaryOutput

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def get_current_command(self) -> TeleopCommand | None: ...
```

`get_current_command()` is used instead of `poll()` because input sources vary: serial leader arms are sampled, WebSocket controllers are event-updated, keyboard state may be callback-maintained, and gamepads are often polled. The module asks for the current command on each control tick; the adapter may return the same command repeatedly while authority is active, or `None` when no command should be published.

Alternative considered: a pure backend that emits normalized samples and a separate profile/binding layer. This was more reusable but too much abstraction for v1, especially because OpenArm Mini will initially only target OpenArm.

### Use a command envelope instead of raw messages or `None` overloading

Adapters return either `None` or a `TeleopCommand`:

```python
@dataclass(frozen=True)
class TeleopCommand:
    command: JointState | PoseStamped | Twist
    stop: bool = False
    metadata: TeleopCommandMetadata | None = None
```

`None` means no command/no authority. `TeleopCommand(command=...)` means an active command. `TeleopCommand(command=..., stop=True)` means an explicit stop command. The module must not treat a missing command as a stop signal.

Alternative considered: returning raw coordinator messages. This loses the ability to distinguish no authority from explicit stop without overloading message contents.

### Publish to existing coordinator streams

`TeleopModule` exposes the stable superset of coordinator-facing outputs:

- `joint_command: Out[JointState]`
- `coordinator_cartesian_command: Out[PoseStamped]`
- `twist_command: Out[Twist]`
- optional non-motion outputs such as `teleop_buttons` and leader/debug/status streams as needed

The adapter declares one primary motion output. The module publishes each active command only to the matching output and rejects adapters that try to mix primary motion abstractions.

Alternative considered: dynamic module IO based on configuration. DimOS supports configuration-resolved IO, but a stable superset of outputs is simpler because `ControlCoordinator` already has predefined inputs.

### Split generic and adapter-specific safety

`TeleopModule` owns structural safety:

- do not publish when command is `None`
- enforce max publish rate
- enforce stale command timeout
- handle explicit stop commands
- enforce one primary motion output per adapter
- stop safely during module shutdown

`OpenArmMiniTeleopAdapter` owns safety and validation requiring device or robot meaning:

- calibration artifact validity
- OpenArm Mini raw encoder tick → radians conversion around calibrated zero offsets
- calibration-defined leader joint assignment and per-joint flip handling
- OpenArm follower joint names
- sender-side OpenArm follower-limit clamping before publishing commands

Alternative considered: putting all safety in the generic module. This would force generic code to understand robot-specific units and joint semantics.

### Implement OpenArm Mini directly on the Feetech library

DimOS will implement OpenArm Mini teleop directly using the lower-level Feetech motor communication library. It will not import LeRobot at runtime. The implementation should mirror the relevant OpenArm Mini arm-joint behavior discovered from LeRobot: two physical serial buses, saved calibration, raw encoder ticks interpreted around calibrated zero offsets, and arm-joint commands emitted in radians. Gripper teleop is out of scope for v1 because the OpenArm follower gripper path is not yet a formal coordinator-controllable API.

The dependency should live in a narrow optional extra for OpenArm Mini teleop rather than broad `manipulation`, so users who do not use this device do not install serial servo dependencies.

Alternative considered: wrapping LeRobot's `OpenArmMini`. This would reduce code, but it would make DimOS depend on LeRobot packaging and all of its teleoperator assumptions for a small device-specific bridge.

### Keep calibration outside normal blueprint startup

Runtime OpenArm Mini teleop startup is non-interactive:

- resolve side-specific calibration directories
- load calibration artifacts
- connect/configure Feetech buses
- fail fast with a clear message if calibration is missing or invalid

Calibration is a special OpenArm Mini maintenance workflow, not part of the generic `TeleopAdapter` contract. A manual script such as `dimos/teleop/openarm_mini/demo_calibrate_openarm_mini.py` calibrates the leader arm joints only, writes calibration artifacts, and may optionally print calibrated leader readings.

The v1 calibration UX is zero-capture, not a live range dashboard. For one side at a time, the operator places the OpenArm Mini leader in its designed natural pose corresponding to the OpenArm follower all-zero arm pose, then runs the calibration script. The script reads the raw Feetech position for each semantic arm joint and stores it as that joint's `homing_offset`.

The calibration artifact is strict arm-only and minimal. It contains exactly `joint_1` through `joint_7`; each entry contains the physical Feetech motor id, the captured homing offset, and a flip boolean. The semantic joint name plus motor id is the leader joint assignment, so physical wrist motor ordering differences are represented by assigning the desired motor id to the desired semantic joint instead of hardcoding a runtime joint_6/joint_7 remap.

The runtime conversion is:

```text
radians = (raw_ticks - homing_offset) * 2π / FEETECH_ENCODER_SPAN
if flip:
  radians = -radians
```

The adapter then clamps each follower arm-joint command to the OpenArm follower's legal joint limits before publishing a `JointState`. Existing coordinator/follower-side defensive handling remains in place. V1 does not implement an automatic first-command follower-state gate; operators must perform startup alignment by placing the follower near the leader-implied command before enabling live authority.

Gripper motor 8 is not touched by the main calibration path, not required in the calibration artifact, and not emitted in v1 commands. Gripper endpoint calibration and follower gripper actuation should be added later when a formal OpenArm follower gripper API/path exists.

Default calibration directories are side-specific and under DimOS state storage:

```text
STATE_DIR / "teleop" / "openarm_mini" / "left"
STATE_DIR / "teleop" / "openarm_mini" / "right"
```

The OpenArm Mini config uses side-specific names:

```python
port_left: str = "/dev/ttyUSB1"
port_right: str = "/dev/ttyUSB0"
left_calibration_path: Path | None = None
right_calibration_path: Path | None = None
```

Alternative considered: storing calibration in cache or using a `teleop_id` directory. Calibration is persistent device state, not cache, and v1 does not need a user-defined teleop identity.

## Risks / Trade-offs

- Feetech library packaging/name may differ from LeRobot internals → verify the package and import surface before implementation; keep imports localized to the OpenArm Mini adapter with a clear missing-extra error.
- Directly owning OpenArm Mini transforms can drift from upstream LeRobot behavior → document the transform rules in code and add unit tests for raw tick conversion, flip handling, leader joint assignment, and follower-limit clamping.
- Hardcoding OpenArm Mini → OpenArm mapping limits reuse → acceptable for v1; introduce profiles/bindings or ordered array commands only when a second follower or behavior needs them.
- Calibration mistakes can cause unsafe leader/follower jumps → default runtime must refuse missing/invalid calibration, the calibration script should make captured zero offsets visible, the adapter should clamp outgoing commands to follower limits, and startup alignment remains an explicit operator responsibility until an automatic follower-state gate exists.
- Adding a generic `TeleopModule` without migrating Quest may leave two teleop patterns temporarily → acceptable to keep v1 focused and avoid destabilizing Quest.

## Migration Plan

1. Add the teleop adapter runtime types and module without changing existing Quest modules.
2. Add OpenArm Mini adapter, config, calibration script, tests, and blueprint.
3. Add the narrow optional dependency extra for OpenArm Mini teleop.
4. Regenerate the blueprint registry after adding the new blueprint.
5. Validate with unit tests and non-hardware startup/error-path tests; hardware validation requires calibrated OpenArm Mini and OpenArm devices.

Rollback is straightforward: remove the new blueprint from use. Existing Quest and keyboard teleop paths remain unchanged.

## Open Questions

- Exact Feetech Python package name and import surface must be verified during implementation.
- Exact future gripper command path should be revisited once OpenArm follower gripper control is exposed through a formal coordinator-compatible API.
