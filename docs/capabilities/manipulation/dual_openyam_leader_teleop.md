# Dual OpenYAM Leader Teleop

Two YAM leader arms (seven Feetech STS3215 servos each, IDs 1 to 6 on the
joints and 7 on the gripper trigger) drive the two OpenYAM followers joint for
joint. The leaders are read by `DualOpenYamLeaderConnection`
(`dimos/robot/manipulators/dual_openyam/connection.py`), a Feetech connection.
Each leader motor names the follower joint it drives and that joint's limits
(`DUAL_OPENYAM_LEADERS` in `dimos/robot/manipulators/dual_openyam/config.py`),
so the connection publishes the followers' joint targets and jaw openings
directly. The follower is the dual OpenYAM
connection described in
[connection testing and CAN selection](/docs/capabilities/manipulation/damiao-connections.md).

## Install

```bash
uv sync --extra manipulation --extra leader-teleop --inexact
```

## Calibration

The leader connection reads one calibration file per side from
`~/.local/state/dimos/teleop/leader/<side>/calibration.json`, or from the file
given with `--left-calibration-path` / `--right-calibration-path`. Each file holds every
motor's zero offset, taken with the leader in the follower's zero pose (the
folded rest pose on its supports), and the joints that read backwards. dimOS
has no tool to write these files yet.

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
  --left-serial-port /dev/ttyACM1 --right-serial-port /dev/ttyACM0
```

Hardware (both CAN selectors are required; startup calibrates the grippers,
so clear the jaws):

```bash
dimos run dual-openyam-leader-teleop \
  --left-serial-port /dev/ttyACM1 --right-serial-port /dev/ttyACM0 \
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
  --left-serial-port /dev/ttyACM1 --right-serial-port /dev/ttyACM0 \
  --left-can-port follower_l --right-can-port follower_r
```

The recorder refuses an existing recording directory, so give every run a
new name. `observation.state` is the coordinator's measured joint state and
`action` is `arm_position_command`, the coordinator's command stream to the
connection (the coordinator publishes no `coordinator_joint_command` for
connection robots). The leaders have no buttons, so mark episodes from a
second terminal with `dimos imitation control`, then build the LeRobot
dataset with `dimos imitation prepare ~/recordings/leader_001 --output
~/datasets/leader_001`.

Preparation runs the schema saved with the recording. Its default quality mode
is `strict`, which rejects an episode on a single dropped camera frame; the
three D405s on shared USB hubs do drop frames. If `dimos imitation inspect`
shows rejected episodes, set `quality.mode` to `fill` in the recording's
`schema.json` and prepare again.

## Tuning, measured on Anvil

Leader command against measured joint state, 100 Hz, `arm_position_command` vs
`coordinator_joint_state` from a collection recording. The coordinator runs alone on its
worker and the command cadence is 10 ms flat, so what remains is the follower itself.

| change | arm joints 1-3 lag | wrist joints 4-6 lag | wrist feel |
|---|---|---|---|
| main gains (arm kd 5.0, wrist kp 10) | 90-120 ms | 160-190 ms | slight teeth |
| filter off | same | same | rougher |
| wrist velocity limit 6.0 | same | same | rougher |
| arm kd 1.0 (shipped) | 40-50 ms | unchanged | slight teeth |
| arm kd 1.0 + wrist kp 20 or 40 | 40-50 ms | 100 / 60 ms | teeth, unusable |

The arm damping is the one change that moved the lag, so it ships. A stiffer wrist tracks faster
but reproduces the 10 ms one-point jog restarts of the trajectory task as teeth, which the soft
wrist blurs; that restart is a platform item, and the wrist gains stay at main's values until it
lands. `--simulation` replaces the leaders with mock leaders parked at the home pose unless serial
ports are given, so the real boards can drive the simulated arms.
