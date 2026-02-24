# Unitree Go2 — DimOS Integration

Velocity control for the Unitree Go2 quadruped via the ControlCoordinator.

## Prerequisites

- **CycloneDDS** — Required for DDS communication with the Go2. See [docs/usage/transports/dds.md](../../../docs/usage/transports/dds.md) for installation instructions.
- **DimOS with the `unitree-dds` extra:**
  ```bash
  uv pip install -e ".[unitree-dds]"
  ```

## Network Setup

Connect your machine to the same network as the Go2, then export the robot IP:

```bash
export ROBOT_IP=192.168.123.161
```

## Running

### Real Hardware

```bash
dimos run unitree-go2-keyboard-teleop
```

### MuJoCo Simulation

No hardware or CycloneDDS required. Requires the `sim` extra:

```bash
uv pip install -e ".[sim]"
dimos --simulation run unitree-go2-keyboard-teleop
```

### Controls

| Key | Action |
|-----|--------|
| `W / S` | Forward / Backward |
| `Q / E` | Strafe Left / Right |
| `A / D` | Turn Left / Right |
| `Shift` | 2x speed boost |
| `Ctrl` | 0.5x slow mode |
| `Space` | Emergency stop |
| `ESC` | Quit |

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `ModuleNotFoundError: unitree_sdk2py` | Run `uv pip install -e ".[unitree-dds]"` |
| `Could not locate cyclonedds` | See [DDS install docs](../../../docs/usage/transports/dds.md) |
| Can't connect / DDS errors | Verify `ping $ROBOT_IP` succeeds and only one DDS domain is active |
| `StandUp()` or `FreeWalk()` fails | Power cycle the Go2 on flat ground and retry |
| Robot ignores velocity commands | Check logs for `Go2 locomotion ready` — allow ~5s after startup |
