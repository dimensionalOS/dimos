# Dual OpenYAM Leader Teleop

Two YAM leader arms (the uynitsuj `yam_active_leader` design: seven Feetech
STS3215 servos per arm, IDs 1 to 6 for the joints and 7 for the gripper
trigger) drive the two OpenYAM followers. The leader module reads the Feetech
buses directly and streams follower joint targets into the coordinator's
`joint_trajectory` task, plus a normalized opening per gripper. The follower is
the dual OpenYAM connection module described in
[connection testing and CAN selection](/docs/capabilities/manipulation/damiao-connections.md).

## Install

```bash
uv sync --extra manipulation --extra openarm-mini-teleop --inexact
```

## Motor IDs and calibration

Each arm is one serial bus. Write the IDs one motor at a time with
`dimos hardware feetech-leader setup-motor-id` (joint 1 to 6, then 7 for the
trigger). Calibrate each arm with the leader held in the follower's zero pose,
which for the YAM is the folded rest pose on its supports, the same pose the
Damiao motors are zeroed in. Calibration files live under
`STATE_DIR/teleop/yam_leader/<side>/calibration.json`:

```bash
dimos hardware feetech-leader calibrate --side left \
  --port-left /dev/ttyACM1 --baudrate 1000000 \
  --left-calibration-path ~/.local/state/dimos/teleop/yam_leader/left \
  --left-flips joint_1,joint_2,joint_3,joint_4,joint_5,joint_6
```

Watch the readout while moving one joint at a time; every joint that turns the
wrong way goes into the flip list, then recalibrate. The trigger needs no flip,
its absolute angle is the closure. Read its full travel in radians off the TUI
with the trigger squeezed; that is the `gripper_travel_radians` module setting
(1.5 rad on the Dimensional leaders):

```bash
dimos hardware feetech-leader joint-tui --side left --port /dev/ttyACM1 \
  --baudrate 1000000 --calibration-path ~/.local/state/dimos/teleop/yam_leader/left
```

The serial ports can swap between boots. Pin them with a udev rule on the
board serial, or check which port moves which arm in Viser before enabling the
CAN connection.

## Run

One machine, mock follower, Viser on 127.0.0.1:8080:

```bash
dimos --simulation run teleop-dual-openyam-leader \
  --port-left /dev/ttyACM1 --port-right /dev/ttyACM0 \
  --yamleaderteleopmodule.gripper-travel-radians 1.5
```

Hardware follower on the same machine (both CAN selectors are required; startup
calibrates the grippers, so clear the jaws):

```bash
dimos run teleop-dual-openyam-leader \
  --port-left /dev/ttyACM1 --port-right /dev/ttyACM0 \
  --yamleaderteleopmodule.gripper-travel-radians 1.5 \
  --left-can-port follower_l --right-can-port follower_r
```

Split deployment, leaders with the operator and the follower on the robot
(zenoh router on the robot; the follower dials `tcp/127.0.0.1:7447`, override
with `--zenoh-connect`; expose Viser with `--manipulationmodule.visualization.host`):

```bash
# robot
dimos run teleop-dual-openyam-leader-follower --left-can-port follower_l --right-can-port follower_r
# operator machine
dimos run teleop-dual-openyam-leader-operator --robot-ip <robot-ip> \
  --port-left /dev/ttyACM1 --port-right /dev/ttyACM0
```

The ports given select the sides; one leader blueprint serves one or two arms.
Leader joint N drives follower joint N, clamped to the `dual_openyam.urdf`
limits before publishing. A mapped jump above 0.75 rad between two reads drops
that command, which is the intended response to a leader segment flipping
past a limit while being handled. The follower trajectory task caps the arm
joints at 3 rad/s and the wrist joints at 4 rad/s. Motor gains are the
connection module's.

## Smooth streaming

On one machine the leader module is linked to the coordinator and streams
each tick as a two-point trajectory from the last target to the new one over
`trajectory_horizon_s` (default 0.04 s, two ticks). A single streamed point
completes within a tick whenever the hand moves slowly, and the next point
then restarts from the measured position, which lags the command by the
motors' tracking error; that restart every 20 ms is a ratchet the operator
feels. With two points the task interpolates from its own last command while
it is executing, so there is no restart. A rejected anchor (the task went
idle and the measured start drifted past its 0.05 rad tolerance) falls back
to a single point for that tick. The mapped targets also pass a first-order
low-pass at `command_filter_cutoff_hz` (default 6 Hz, `None` to disable):

```bash
dimos run teleop-dual-openyam-leader --port-right /dev/ttyACM0 \
  --yamleaderteleopmodule.trajectory-horizon-s 0.04 \
  --yamleaderteleopmodule.command-filter-cutoff-hz 6
```

Without a coordinator link (the split operator blueprint) or with
`trajectory_horizon_s` set to `None`, targets go out on `joint_command`.

## Record a session

`dual-openyam-leader-collection` is the leader graph plus the episode monitor,
three D405 cameras (overhead and both wrists, colour only) and the profile
recorder. The dataset gets three images, the 14 measured joints as
`observation.state` and the coordinator's joint targets
(`coordinator_joint_command`) as `action`. Camera serials and the RealSense
binary come from the run config, never from source:

```json
{
  "overhead_camera": {"serial_number": "..."},
  "left_wrist_camera": {"serial_number": "..."},
  "right_wrist_camera": {"serial_number": "..."}
}
```

```bash
dimos run dual-openyam-leader-collection --config workcell.json \
  --port-left /dev/ttyACM1 --port-right /dev/ttyACM0 \
  --yamleaderteleopmodule.gripper-travel-radians 1.5 \
  --left-can-port follower_l --right-can-port follower_r \
  --recorder.recording recordings/leader-001 \
  --episodes.task "stack the cups"
dimos imitation control
```

The leaders have no buttons, so episodes are started, saved and discarded from
the control panel in the second terminal. Prepare and inspect the session as in
[Imitation Learning for Manipulation](/docs/capabilities/manipulation/imitation-learning.md).

## Before enabling the follower

- Start with the followers at their zero pose so the first streamed target is
  within the trajectory task's 0.05 rad start tolerance.
- Hold the leaders still and in their calibrated zero pose when the blueprint
  starts; read a leader in its straight pose after handling it.
- Keep the e-stop in reach for the first motions and move one joint at a time.
