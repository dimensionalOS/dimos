# OpenArm Mini Teleop

Two Feetech leader arms drive the bimanual OpenArm. `OpenArmMiniConnection`
(`dimos/robot/manipulators/openarm/connection.py`) reads the leaders and
publishes their joint angles; the generic leader teleop module
(`dimos/teleop/leader`) maps them onto the arms. The OpenArm folder supplies the
leader robot data, the joint-to-joint mapping, the joint limits and the
blueprints.

## Install

```bash
uv sync --extra leader-teleop --extra manipulation
```

## Calibration

Each leader carries seven arm motors, `joint_1` to `joint_7` at IDs 1 to 7, and
a gripper trigger servo at ID 8. The leader connection reads one calibration file per side
from `~/.local/state/dimos/teleop/leader/<side>/calibration.json`: each motor's
zero offset, taken with the leader in the pose that matches the follower's
all-zero joints and the trigger released, and which motors read backwards. The
OpenArm mapping names the trigger, so a calibration without it does not start.
dimOS has no tool to write these files yet.

## Bring-up in Viser

`teleop-openarm-mini` runs the leaders, the OpenArm coordinator, its connection
and Viser on one machine. The leader ports you pass select the sides; the other
arm holds. `--simulation` swaps the CAN connection for a mock.

```bash
dimos --simulation run teleop-openarm-mini --left-serial-port <feetech-port>
```

Leader joint N drives follower joint N of the same side, clamped to the OpenArm
limits before publishing, read every 10 ms and low-passed at 6 Hz
(`--openarmminiconnection.state-rate-hz` with a matching `--leaderteleopmodule.reading-period-s`,
and `--leaderteleopmodule.command-filter-cutoff-hz`, to change). The targets stream into the coordinator's `joint_trajectory` task
with human-speed velocity limits; the trigger drives that side's gripper task,
1 = open.

## Driving the real OpenArm

Bring the CAN interfaces up with `dimos hardware can setup <interface>`, pose
each leader like its arm, then:

```bash
dimos run teleop-openarm-mini \
  --left-serial-port <left-feetech-port> \
  --right-serial-port <right-feetech-port> \
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
  --left-serial-port <left-feetech-port> \
  --right-serial-port <right-feetech-port>
```

Start the router before the follower. If the link drops, the follower holds its
last target.

## Record demonstrations

`openarm-mini-collection` wraps the one-machine stack in the collection
recorder. It records the measured follower joints (`coordinator_joint_state`: 14
arm joints and 2 finger joints under `arm/`), the coordinator's position
commands (`arm_position_command`, the same 16 joints: arm targets from the
leaders, jaw openings from the triggers) and three 640x480 cameras.

```bash
dimos run openarm-mini-collection --daemon \
  --left-serial-port <feetech-port> \
  --right-serial-port <feetech-port> \
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
