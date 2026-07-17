# Spot

Boston Dynamics Spot control for DimOS: velocity teleop + camera/odometry streaming.

## Install

```bash
uv sync --extra spot
```

## Connect to Spot

Pick one:

- **WiFi (easiest):** join Spot's WiFi AP, robot is at `192.168.80.3`.
- **Ethernet:** plug into Spot's rear port. Robot defaults to `10.0.0.3/24`; set your
  interface to a static IP on that subnet (e.g. `10.0.0.20/24`, **no gateway**).

The rear-port IP is configurable, so if ethernet won't connect, check the actual
address in the Spot Admin Console (`https://192.168.80.3` → Network Setup → Ethernet).

## Run

```bash
dimos run spot \
  -o spothighlevel.username=<user> \
  -o spothighlevel.password=<password>
```

The IP auto-detects (WiFi then Ethernet). Force one with `-o spothighlevel.ip=<addr>`.

Keyboard teleop: WASD move/turn, QE strafe, Space soft-stop, ESC quit. A Rerun
viewer opens with the fisheye/depth cameras and odometry.

## Layout

- `config.py` — constants + pure address/credential helpers (no `bosdyn` import).
- `effectors/high_level.py` — `SpotHighLevel`: the single Spot module —
  lease/E-stop/power/stand + velocity commands plus the five fisheye + five depth
  cameras and body odometry.
- `recorder.py` — `SpotRecorder`: records every Spot stream to disk.
- `blueprints/spot.py` — the runnable `spot` blueprint (click/teleop + sensors + Rerun).
