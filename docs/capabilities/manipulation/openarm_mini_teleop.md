# OpenArm Mini Teleop

Two Feetech leader arms drive the bimanual OpenArm through the generic leader
teleop module (`dimos/teleop/leader`). The OpenArm folder supplies only the
motor-to-joint mapping, the joint limits and the blueprints.

## Install

```bash
uv sync --extra leader-teleop --extra manipulation
```

## Set motor IDs and calibrate

Each leader carries seven arm motors, `joint_1` to `joint_7` at IDs 1 to 7, and
a gripper trigger servo at ID 8. Write IDs one motor at a time:

```bash
dimos hardware feetech-leader setup-motor-id --port <feetech-port> --old-id 1 --new-id 3
```

Hold the leader in the pose that matches the follower's all-zero joints, trigger
released, and record the zero offsets. Flips name the motors that read backwards
on your printed arm; the default is none.

```bash
dimos hardware feetech-leader calibrate --side left --port-left <feetech-port> \
  --left-flips joint_1,joint_3,joint_4,joint_5,joint_6,joint_7
```

The file lands at `~/.local/state/dimos/teleop/leader/<side>/calibration.json`.
The trigger is recorded when a servo answers at ID 8. The OpenArm mapping names
the trigger, so a leader calibrated without one does not start; recalibrate with
the trigger connected. Check a side without starting any robot control:

```bash
dimos hardware feetech-leader joint-tui --side left --port <feetech-port>
```

## Bring-up in Viser

`teleop-openarm-mini` runs the leaders, the OpenArm coordinator, its connection
and Viser on one machine. The leader ports you pass select the sides; the other
arm holds. `--simulation` swaps the CAN connection for a mock.

```bash
dimos --simulation run teleop-openarm-mini --leaderteleopmodule.port-left <feetech-port>
```

Leader joint N drives follower joint N of the same side, clamped to the OpenArm
limits before publishing, read every 10 ms and low-passed at 6 Hz
(`--leaderteleopmodule.tick-period-s`, `--leaderteleopmodule.command-filter-cutoff-hz` to change). The targets stream into the coordinator's `joint_trajectory` task
with human-speed velocity limits; the trigger drives that side's gripper task,
1 = open.

## Driving the real OpenArm

Bring the CAN interfaces up with `dimos hardware can setup <interface>`, pose
each leader like its arm, then:

```bash
dimos run teleop-openarm-mini \
  --leaderteleopmodule.port-left <left-feetech-port> \
  --leaderteleopmodule.port-right <right-feetech-port> \
  --left-can-port can1 --right-can-port can2
```

Both CAN ports are required even with one leader connected.

## Leaders on a separate machine

Run the two halves as separate blueprints on one zenoh bus. The robot runs a
zenoh router on port 7447 (`dimos host install --start`, or override the address
with `DIMOS_OPENARM_ROUTER` or `--zenoh-connect`); the follower dials it and
serves Viser on all interfaces; the leaders run as zenoh clients and leave the
bus-wide `Coordinator` name to the robot.

```bash
# robot computer
dimos run teleop-openarm-mini-follower --left-can-port can1 --right-can-port can2

# operator laptop
dimos run teleop-openarm-mini-leader --robot-ip <robot-ip> \
  --leaderteleopmodule.port-left <left-feetech-port> \
  --leaderteleopmodule.port-right <right-feetech-port>
```

Start the router before the follower. If the link drops, the follower holds its
last target.

## Joy-Cons instead of trigger servos

`teleop-openarm-mini-joycon` and `teleop-openarm-mini-joycon-leader` add a
Nintendo Joy-Con per leader (L on the left, R on the right) beside the leader
module, with the leaders mapped as arm joints only. The Joy-Cons pair over
Bluetooth; the kernel `hid-nintendo` driver registers each as an input device
and the module reads those devices directly, no extra dependency.

| Joy-Con | does |
|---|---|
| ZL / ZR held | close the left / right jaw (`--joyconteleopmodule.gripper-mode toggle` flips per press instead) |
| B / Y | start or save / discard an episode (the episode monitor's defaults) |
| plus | pause and resume following (`LeaderTeleopModule.set_authority`) |
| left stick | base velocity on `cmd_vel` when `publish_cmd_vel` is on; stick pressed turns it into yaw |

With only the L Joy-Con, point the episode monitor at its buttons:
`--episodes.button-map '{"toggle": "MENU_L", "discard": "LG"}'` (minus and L).

Pair once per Joy-Con, holding the SYNC button on its rail until the LEDs
chase. bluez refuses a Joy-Con's input session by default because the
controller opens it before the bond completes, so allow that first:

```bash
sudo sed -i 's/^#ClassicBondedOnly=true/ClassicBondedOnly=false/' /etc/bluetooth/input.conf
sudo systemctl restart bluetooth
bluetoothctl
power on; agent on; scan on
pair <MAC>; trust <MAC>; connect <MAC>
```

A Joy-Con that was refused once goes to sleep and does not reconnect by
itself; connect it from the Bluetooth settings panel or pair it again.

Confirm the key and axis codes the driver reports, pressing each button and
moving each stick:

```bash
python -m dimos.teleop.joycon.probe 60
```

The defaults in `dimos.teleop.joycon.mapping` are `hid-nintendo`'s codes (ZL
is key 312, B is 304, the left stick is axes 0 and 1, the right stick 3 and
4); when a code differs, pass the table per side, for example
`--joyconteleopmodule.right-buttons '{"ZR": 313, "B": 304, "Y": 308, "plus": 315, "A": 305, "X": 307, "R": 311, "stick": 318}'`.
The published opening slews at `gripper_slew_per_s` (3 per second) so a jaw
never slams. A Joy-Con that drops off Bluetooth keeps its last opening until
it is back; the other side keeps working.

A leader calibrated with its trigger still loads: the mapping names no
trigger, so pass an arm-only calibration with
`--leaderteleopmodule.left-calibration-path`, or recalibrate with the servo
removed.

## Record demonstrations

`openarm-mini-collection` wraps the one-machine stack in the collection
recorder. It records the measured follower joints (`coordinator_joint_state`: 14
arm joints and 2 finger joints under `arm/`), the coordinator's position
commands (`arm_position_command`, the same 16 joints: arm targets from the
leaders, jaw openings from the triggers) and three 640x480 cameras.

```bash
dimos run openarm-mini-collection --daemon \
  --leaderteleopmodule.port-left <feetech-port> \
  --leaderteleopmodule.port-right <feetech-port> \
  --left-can-port can1 --right-can-port can2 \
  --front.hardware.camera-index /dev/video8 \
  --left-wrist.hardware.camera-index /dev/video0 \
  --right-wrist.hardware.camera-index /dev/video2 \
  --recorder.recording recordings/openarm-001 \
  --episodes.task "pick up the cube"
dimos imitation control
```

With the leaders on another machine, run `openarm-mini-follower-collection` on
the robot with the options above and `teleop-openarm-mini-leader` on the
operator machine. The episode panel controls takes; the arms keep following the
leaders between episodes.

```bash
dimos imitation inspect recordings/openarm-001
dimos imitation prepare recordings/openarm-001 --output datasets/openarm-001
```
