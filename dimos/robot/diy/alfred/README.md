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

The current firmware resets when the serial port opens and does not retain its
home reference. Home it explicitly from another terminal:

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
the top switch and `-0.050` metres as the post-home parking position. The Nano
accepts one point-to-point goal at a time; the connection coalesces streamed
commands so the most recent target runs after the active move completes.

`app.PillarConnection.stop_motion()` maps to the firmware's ramped `x` stop.
It is not an emergency stop. A future firmware e-stop should stop step pulses,
engage the SSR brake, abort homing, and invalidate the position reference.
