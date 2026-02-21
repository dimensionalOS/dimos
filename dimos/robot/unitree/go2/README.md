# Unitree Go2 — DimOS Integration

Velocity control for the Unitree Go2 quadruped via Unitree SDK2 over DDS
(CycloneDDS).

## Architecture

```
Keyboard / Agent
      ↓  Twist (/cmd_vel)
ControlCoordinator  (100 Hz tick loop)
      ↓  [vx, vy, wz]
UnitreeGo2Adapter
      ↓  DDS (CycloneDDS)
Go2 SportClient  →  Move(vx, vy, wz)
```

---

## Installation

### Step 1 — Install CycloneDDS

The recommended approach is via pip — the wheel bundles the C++ library:

```bash
uv pip install "cyclonedds>=0.10.5"
```

If the prebuilt wheel doesn't work on your system (e.g. unsupported
architecture), fall back to building from source:

```bash
sudo apt-get update
sudo apt-get install -y cmake build-essential git libssl-dev bison

git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds
mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/cyclonedds/install
cmake --build . --target install
cd ../..

export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
echo 'export CYCLONEDDS_HOME="$HOME/cyclonedds/install"' >> ~/.bashrc
source ~/.bashrc
```

### Step 2 — Install DimOS with the `unitree` extra

```bash
uv pip install -e ".[unitree]"
```

This installs `unitree-sdk2py` along with the rest of the DimOS unitree
dependencies.

---

## Network Setup

The Go2 creates a WiFi access point when powered on.

1. Connect your machine to the Go2's WiFi:
   - SSID: `Unitree_Go2_XXXXX`
   - Password: `00000000` (8 zeros)

2. Verify connectivity:
   ```bash
   ping 192.168.12.1
   ```

---

## Running

### Keyboard Teleop (real hardware)

```bash
dimos run unitree-go2-keyboard-teleop
```

Controls:

| Key | Action |
|-----|--------|
| `W / S` | Forward / Backward |
| `Q / E` | Strafe Left / Right |
| `A / D` | Turn Left / Right |
| `Shift` | 2× speed boost |
| `Ctrl` | 0.5× slow mode |
| `Space` | Emergency stop |
| `ESC` | Quit |

### Keyboard Teleop (MuJoCo simulation)

No hardware required. Requires the `sim` extra:

```bash
uv pip install -e ".[sim]"
dimos --simulation run unitree-go2-keyboard-teleop
```

> **Display requirement:** MuJoCo and pygame both need an X11 display.
> If running over SSH, enable X11 forwarding (`ssh -X`) or run:
> ```bash
> xhost +local:
> export DISPLAY=:0
> ```

---

## Initialization Sequence

On `connect()` the adapter runs:

1. `ChannelFactoryInitialize(0)` — initialize DDS transport
2. `SportClient.StandUp()` — stand the robot up (waits ~3 s)
3. `SportClient.FreeWalk()` — activate locomotion mode (waits ~2 s)
4. `SportClient.Move(vx, vy, wz)` — velocity commands now accepted

Allow ~5 seconds after startup before sending velocity commands.

---

## Troubleshooting

**`ModuleNotFoundError: No module named 'unitree_sdk2py'`**
→ Run Step 2 above.

**`Could not locate cyclonedds` during install**
→ Try `uv pip install "cyclonedds>=0.10.5"` first. If building from source,
ensure `CYCLONEDDS_HOME` is exported and retry.

**DDS errors / can't connect to Go2**
→ Verify `ping 192.168.12.1` succeeds and only one DDS domain is active.

**`StandUp()` or `FreeWalk()` fails**
→ Power cycle the Go2, ensure it is on flat ground, and retry.

**Robot ignores velocity commands**
→ Check logs for `✓ Go2 locomotion ready`. The coordinator calls `write_enable()`
automatically; if it is missing, ensure the `auto_enable` field is set in the
`HardwareComponent`.

**X11 / display errors (sim mode)**
→ Run `xhost +local:` and `export DISPLAY=:0` before launching.
