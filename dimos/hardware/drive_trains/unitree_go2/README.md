# Unitree Go2 drive-train adapter

[`adapter.py`](adapter.py) — `UnitreeGo2TwistAdapter` (high-level): Twist `(vx, vy, wz)` via SportClient, with optional Rage Mode (`rage_mode=True`, ~2.5 m/s forward envelope). Auto-registered as `"unitree_go2"` and used by blueprints like `unitree-go2-keyboard-teleop`. This is the one you want for teleop, navigation, or anything velocity-commanded.

---

## Running

Install the `unitree-dds` extra (pulls `unitree-sdk2py-dimos` and
`cyclonedds`, which has wheels for x86_64 Linux and macOS; on Linux arm64,
build CycloneDDS first as in
[`docs/usage/transports/dds.md`](../../../../docs/usage/transports/dds.md)):

```bash
uv pip install -e ".[unitree-dds]"
```

Set the robot IP and launch a blueprint:

```bash
export ROBOT_IP=192.168.123.161
dimos run unitree-go2-keyboard-teleop         # direct DDS, FreeWalk default
```

Keyboard controls (pygame window must be focused):

| Key     | Action                        |
|---------|-------------------------------|
| `W / S` | Forward / Backward            |
| `Q / E` | Strafe Left / Right           |
| `A / D` | Turn Left / Right             |
| `Shift` | 2× speed boost                |
| `Ctrl`  | 0.5× slow mode                |
| `Space` | Emergency stop                |
| `ESC`   | Quit                          |

Troubleshooting:

| Symptom                               | Fix                                                         |
|---------------------------------------|-------------------------------------------------------------|
| `ModuleNotFoundError: unitree_sdk2py` | `uv pip install -e ".[unitree-dds]"`                        |
| `Could not locate cyclonedds`         | See [`docs/usage/transports/dds.md`](../../../../docs/usage/transports/dds.md) |
| DDS discovery failures                | Verify `ping $ROBOT_IP` succeeds; only one DDS domain active |
| `StandUp()` / `FreeWalk()` fails      | Power-cycle the Go2 on flat ground and retry                |
| Robot ignores velocity commands       | Wait ~5s for `[Go2] Locomotion ready` after startup       |
