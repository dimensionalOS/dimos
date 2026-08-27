# WebXR Teleop

Spatial teleoperation through browser WebXR input sources. Supports tracked
controllers and hands on compatible headsets, including Meta Quest and PICO.

## Architecture

```
WebXR Browser  ──WebSocket──→  Embedded HTTPS Server  ──→  ArmTeleopModule
(poses + Joy)                    (port 8443)                  (absolute PoseStamped)
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
dimos run teleop-webxr-rerun    # WebXR teleop + Rerun viz
dimos run teleop-webxr-xarm7   # XArm7
dimos run teleop-webxr-hand-xarm7  # XArm7 hand tracking; pinch to toggle
dimos run teleop-webxr-piper   # Piper
dimos run teleop-webxr-a1z     # A1Z with mock hardware
dimos run teleop-webxr-dual    # Mixed XArm6 + Piper, one task per arm
dimos run teleop-webxr-openarm # OpenArm, bimanual IK + planner/Viser + mock hardware
```

Select a CAN interface explicitly to control real A1Z hardware:

```bash
dimos --can-port a1zcan run teleop-webxr-a1z
```

Open `https://<host-ip>:8443/teleop` in a WebXR-capable headset browser. Accept
the certificate, then tap Connect.

### G1 SONIC full-body simulation

Calibrate the PICO Motion Trackers, then start the MuJoCo blueprint:

```bash
dimos --simulation mujoco run unitree-g1-sonic-webxr-teleop
```

Use `--viewer none` to skip Rerun and inspect the live simulation in the
native MuJoCo window.

Open `https://<host-ip>:8443/teleop` on the PICO and tap Connect. Hold X and A
together to guide the G1 with your body. Release either button to return SONIC
to planner control. While engaged, body motion supplies the whole-body
reference; the thumbsticks cannot command translation, but the right stick can
adjust heading.

Tracking loss ends engagement. After tracking returns, release and hold X+A
again. Partial body frames keep the last complete pose for at most 150 ms.

For hand teleop, remove the controllers. Pinch the thumb and index finger on
the selected hand to engage it, move the wrist to control the arm, then pinch
again to disengage. Pinch the thumb and middle finger to close the gripper;
release it to open the gripper. Hand tracking must be enabled in the headset
browser.

`teleop-webxr-openarm` is safe by default: it always uses the in-memory
`mock_whole_body` adapter, regardless of the global simulation setting. It does
not select physical OpenArm hardware implicitly. The mock and bimanual model
start at the canonical all-zero pose. Since that pose places both joint-4
coordinates at their lower limits, the OpenArm planner and teleoperation task
share a Pink joint-limit posture margin that supplies a deterministic inward
direction without changing the measured seed. No random retry runs in the
control loop.

Specify both CAN interfaces to select real OpenArm hardware. Supplying only one
is rejected:

```bash
dimos run teleop-webxr-openarm --left-can-port can1 --right-can-port can0
```

The blueprint also includes `ManipulationModule` with the same bimanual model
and Viser visualization. Its coordinator has a joint-trajectory task over both
arms at priority 20; planned execution therefore preempts the priority-10
teleoperation task through normal arbitration and clears the engagement state.

## Arm task bindings

Arm teleoperation uses one `TeleopIKTask` configured with one or two hand
bindings. Each binding names the controller (`left` or `right`), a frame in the
task's `RobotModelConfig`. The task's top-level `joint_names` explicitly select
the joints Pink may update. Gripper triggers publish normalized per-hand streams
to dedicated gripper tasks; gripper joints are not owned by the IK task.

Single-arm and mixed-arm setups use one binding per task. A bimanual robot such
as OpenArm uses one task, two bindings, and one bimanual model, so Pink solves
both frame targets in one control tick.

For a two-binding task, both primary buttons must be held. Engagement captures
both controller and robot references together. Releasing either button,
receiving stale input from either controller, preemption, or E-stop clears the
entire session; both hands must engage again before commands resume.

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

**Buttons**: trigger, grip, touchpad, thumbstick, X/A, Y/B, optional menu. WebXR
omits a platform-reserved menu button on devices such as PICO controllers.

## Body Tracking Messages

The WebSocket carries two frame formats. Controller poses and joystick state use
binary LCM messages. When body tracking is enabled, the browser sends JSON text
frames containing every joint resolved by the headset. A `null` joint map means
the body source is unavailable; an empty map means no joints resolved for that
frame.

## File Structure

```
webxr/
├── module.py             # Base module
├── extensions.py         # ArmTeleop, TwistTeleop
├── controller_types.py   # WebXRControllerState, Buttons
├── blueprints.py
└── web/static/index.html # WebXR client
```
