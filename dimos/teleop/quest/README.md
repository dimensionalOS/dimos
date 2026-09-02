# Quest Teleop

Teleoperation via Meta Quest 3 VR controllers. Dual-hand tracking with WebXR.

## Architecture

```
Quest Browser  ──WebSocket──→  Embedded HTTPS Server  ──→  ArmTeleopModule
(WebXR poses + Joy)             (port 8443)                  (absolute PoseStamped)
                                                                  │ left/right
                                                                  ▼
                                                     TeleopControlCoordinator
                                                                  │ by task name
                                                                  ▼
                                                        TeleopIKTask
                                                        (relative targets + Pink)
```

## Running

```bash
dimos run teleop-quest-rerun    # Quest teleop + Rerun viz
dimos run teleop-quest-xarm7   # XArm7
dimos run teleop-quest-hand-xarm7  # XArm7 hand tracking; pinch to toggle
dimos run teleop-quest-piper   # Piper
dimos run teleop-quest-a1z     # A1Z with mock hardware
dimos run teleop-quest-dual    # Mixed XArm6 + Piper, one task per arm
dimos run teleop-quest-openarm # OpenArm, bimanual IK + planner/Viser + mock hardware
```

Select a CAN interface explicitly to control real A1Z hardware:

```bash
dimos --can-port a1zcan run teleop-quest-a1z
```

Open `https://<host-ip>:8443/teleop` on Quest browser. Accept cert, tap Connect.

For hand teleop, remove the controllers. Pinch the thumb and index finger on
the selected hand to engage it, move the wrist to control the arm, then pinch
again to disengage. Pinch the thumb and middle finger to close the gripper;
release it to open the gripper. Hand tracking must be enabled for the Quest
browser.

`teleop-quest-openarm` is safe by default: it always uses the in-memory
`mock_whole_body` adapter, regardless of the global simulation setting. It does
not select physical OpenArm hardware implicitly. The mock and bimanual model
start at the canonical all-zero pose. Since that pose places both joint-4
coordinates at their lower limits, the OpenArm planner and Quest task share a
Pink joint-limit posture margin that supplies a deterministic inward direction
without changing the measured seed. No random retry runs in the control loop.

Specify both CAN interfaces to select real OpenArm hardware. Supplying only one
is rejected:

```bash
dimos run teleop-quest-openarm --left-can-port can1 --right-can-port can0
```

The blueprint also includes `ManipulationModule` with the same bimanual model
and Viser visualization. Its coordinator has a joint-trajectory task over both
arms at priority 20; planned execution therefore preempts the priority-10 Quest
task through normal arbitration and clears the teleoperation engagement state.

## Arm task bindings

Arm teleoperation uses one `TeleopIKTask` configured with one or two hand
bindings. Each binding names the controller (`left` or `right`), a frame in the
task's `RobotModelConfig`. The task's top-level `joint_names` explicitly select
the joints Pink may update. Gripper triggers publish normalized per-hand streams
to dedicated gripper tasks; gripper joints are not owned by the IK task.

Single-arm and mixed-arm setups use one binding per task. A bimanual robot such
as OpenArm uses one task, two bindings, and one bimanual model, so Pink solves
both frame targets in one control tick.

For a two-binding task, both controller grips must be held. Engagement captures
both controller and robot references together. Releasing either grip,
receiving stale input from either controller, preemption, or E-stop clears the
entire session; both hands must engage again before commands resume.

For controller-based arm teleoperation, the middle-finger grip is the deadman:
hold the relevant grip to engage and release it to disengage. Face buttons are
available for application lifecycle controls; the OpenYAM learning rollout
uses **A** to toggle policy execution. The index-finger trigger remains the
analog gripper command and is forwarded only while that hand's grip is held.

`teleop_buttons` publishes raw button levels. `button_pressed` and
`button_released` publish digital-only edges after 50 ms of stable input;
disconnecting the control client releases held buttons immediately. Lifecycle
consumers should subscribe to the edge streams instead of detecting edges from
raw levels independently.

## Subclassing

| Method | Purpose |
|--------|---------|
| `_handle_engage()` | Customize engage/disengage logic |
| `_should_publish()` | Add conditions for publishing |
| `_get_output_pose()` | Customize pose computation (ArmTeleop publishes absolute poses) |
| `_publish_msg()` | Change output format |

`self._lock` is already held — don't acquire it in overrides.

## Joy Message Format

**Axes**: thumbstick X, thumbstick Y, trigger (analog), grip (analog)

**Buttons**: trigger, grip, touchpad, thumbstick, X/A, Y/B, menu

## File Structure

```
quest/
├── quest_teleop_module.py   # Base module
├── quest_extensions.py      # ArmTeleop, TwistTeleop
├── quest_types.py           # QuestControllerState, Buttons
├── blueprints.py
└── web/static/index.html    # WebXR client
```
