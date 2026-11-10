# Dual OpenYAM Leader Teleop

Two YAM leader arms (seven Feetech STS3215 servos each, IDs 1 to 6 on the
joints and 7 on the gripper trigger) drive the two OpenYAM followers joint for
joint. The robot side is a mapping table in
`dimos/robot/manipulators/dual_openyam/blueprints/leader_teleop.py`; the reading,
calibration and publishing live in the shared
[leader teleop module](/dimos/teleop/README.md). The follower is the dual OpenYAM
connection described in
[connection testing and CAN selection](/docs/capabilities/manipulation/damiao-connections.md).

## Install

```bash
uv sync --extra manipulation --extra leader-teleop --inexact
```

## Calibration

Calibrate each leader held in the follower's zero pose, the folded rest pose on
its supports, and name the joints that read backwards in the flip list. Files
land under `~/.local/state/dimos/teleop/leader/<side>/calibration.json`, or
point the module at another file with `--leaderteleopmodule.left-calibration-path`.

```bash
dimos hardware feetech-leader calibrate --side left --port-left /dev/ttyACM1 \
  --left-flips joint_1,joint_2,joint_3,joint_4,joint_5,joint_6
dimos hardware feetech-leader joint-tui --side left --port /dev/ttyACM1
```

The trigger needs no flip; its absolute angle is the closure. Its full travel
is `DUAL_OPENYAM_LEADER_TRIGGER_TRAVEL_RADIANS` (1.5 rad on the Dimensional
leaders). Serial ports can swap between boots: check which port moves which
arm in Viser before enabling the CAN connection.

## Run

Mock leaders and mock follower, Viser on 127.0.0.1:8080:

```bash
dimos --simulation run dual-openyam-leader-teleop
```

Real leaders, mock follower:

```bash
dimos --simulation run dual-openyam-leader-teleop \
  --leaderteleopmodule.port-left /dev/ttyACM1 --leaderteleopmodule.port-right /dev/ttyACM0
```

Hardware (both CAN selectors are required; startup calibrates the grippers,
so clear the jaws):

```bash
dimos run dual-openyam-leader-teleop \
  --leaderteleopmodule.port-left /dev/ttyACM1 --leaderteleopmodule.port-right /dev/ttyACM0 \
  --left-can-port follower_l --right-can-port follower_r
```

The ports given select the sides. The coordinator's `joint_trajectory` task
reads the leader's port as a guarded stream, so the leader owns the joints while
it publishes and releases them when it stops.

## Record demonstrations

`learning-collect-leader-dual-openyam` adds the recorder, the episode monitor
and three D405 color streams (overhead, left wrist, right wrist). Camera
serials and native binaries come from a run config, never from source.

```bash
dimos run learning-collect-leader-dual-openyam --config workcell.json \
  --recorder.recording ~/recordings/leader_001 --episodes.task "stack the cups" \
  --leaderteleopmodule.port-left /dev/ttyACM1 --leaderteleopmodule.port-right /dev/ttyACM0 \
  --left-can-port follower_l --right-can-port follower_r
```

The leaders have no buttons, so mark episodes from a second terminal with
`dimos imitation control`, then build the LeRobot dataset with
`dimos imitation prepare ~/recordings/leader_001 --output ~/datasets/leader_001`.
