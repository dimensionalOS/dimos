# Alfred

## Pillar bring-up

Install the small Alfred hardware extra rather than the platform-wide `all`
extra:

```bash
uv sync --extra alfred
```

Find the Nano's stable device path, then start the standalone pillar stack:

```bash
ls -l /dev/serial/by-id/
dimos --device-path /dev/serial/by-id/<nano> run alfred-pillar
```

Opening the serial port resets the board, and the firmware does not retain its
home reference across a reset. The connection waits out the reset, re-applies
`set echo 0` and `set rate` on every `ready` event, and refuses to move until
homed. Home it explicitly from another terminal:

```python
# dimos shell
pillar = app.PillarConnection
pillar.get_status()
pillar.home()
pillar.get_status()  # repeat until homed=True and phase="idle"

# Down: positions become more negative away from the top switch.
pillar.set_position(-0.10)
pillar.get_status()  # wait for phase="idle"

# Up: return to the post-home parking height.
pillar.set_position(-0.05)
pillar.get_status()  # wait for phase="idle"

# Controlled ramped stop, if needed.
pillar.stop_motion()
```

Do not home until the physical UP direction, normally-closed top switch, and
SSR brake polarity have been commissioned. The present firmware has no bottom
limit switch.

Once `get_status()` reports `homed=True` and `phase="idle"`, command the single
linear joint in metres:

```bash
dimos topic send /pillar/joint_command \
  'JointState(name=["pillar/lift"], position=[-0.10])'

dimos topic echo /pillar/joints
```

The current safe command range is `-0.500` to `-0.002` metres, with zero at
the top switch and `-0.050` metres as the post-home parking position. Motion
commands preempt each other, so a streamed target takes effect on the next step
rather than queueing behind the active move. The rail still rides its existing
ramp down to a standstill before reversing, so a reversal costs `v^2/2a` of
run-out — 2.7 mm at the default 40 mm/s. Prefer a lower speed or a higher
acceleration over sending targets more often.

`app.PillarConnection.stop_motion()` maps to the firmware's ramped `stop`.
It is not an emergency stop. A future firmware e-stop should stop step pulses,
engage the SSR brake, abort homing, and invalidate the position reference.

## Whole-robot URDF, sim, and navigation

The robot description (FlowBase, pillar lift, bimanual OpenArm v2.0, Mid-360, D455, D435)
is the LFS archive `alfred_description`: `alfred_v1.urdf` (casters welded) and
`alfred_v2.urdf` (eight steer/drive caster joints, drawn but not driven), built from the Onshape CAD by the
bundled `build_alfred_urdf.py`; its README has the frame table. `alfred_model.py` wraps it
with the coordinator joint names. `pillar/lift` is zero at the top limit switch, positive
up, range -0.500 .. -0.002 m, the same convention as the firmware.

```bash
uv sync --extra misc --extra alfred --extra manipulation

# Simulation: viser planner on alfred_v2 with mock lift and arms.
dimos run alfred-sim

# Robot. Point-LIO reads the host address on the lidar link; arms are real only with both
# CAN ports (Alfred: left can2, right can3, `dimos hardware can setup <if>` first).
export DIMOS_POINTLIO_HOST_IP=192.168.1.100
OPENARM_LEFT_CAN=can2 OPENARM_RIGHT_CAN=can3 dimos --rerun-host 0.0.0.0 run alfred-nav \
    --pillarconnection.device-path /dev/serial/by-id/<nano>

# Laptop viewer (click to navigate, keyboard teleop) and viser through a tunnel.
dimos-viewer --connect rerun+http://<robot>:9877/proxy --ws-url ws://<robot>:3030/ws
ssh -L 8095:127.0.0.1:8095 <robot>

# Tools that attach from another terminal must use the blueprint's transport.
DIMOS_TRANSPORT=lcm dimos shell     # app.PillarConnection.home() before planning the lift
```

`alfred-nav` keeps the base out of the coordinator: `AlfredHighLevel` is the only FlowBase
writer and `MovementManager` muxes teleop over navigation. `AlfredMountTf` publishes the
whole urdf mount tree, re-rooted with `root_frame=mid360_link`: Point-LIO owns the lidar's
parent edge, so `base_link -> mid360_link` is inverted and everything else hangs off the
lidar unchanged. Jeff's dimSLAM blueprints (`alfred-mls-nav*`) are untouched.
