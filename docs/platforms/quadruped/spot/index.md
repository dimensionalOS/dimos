# Boston Dynamics Spot

Boston Dynamics Spot control for dimOS: velocity teleop plus fisheye/depth camera and
odometry streaming.

## Install

Set up dimOS with the [official installer](/docs/installation/index.md) in developer mode, then add the Spot extra to that checkout:

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
  --spothighlevel.username=<user> \
  --spothighlevel.password=<password>
```

Both `spot` and `spot-record` require a username and password before any modules start.
The sticker inside Spot's battery bay lists the credentials. You can also supply them
through `SPOTHIGHLEVEL__USERNAME` and `SPOTHIGHLEVEL__PASSWORD`. Use `spot-replay`
to play a recording without robot credentials.

The IP auto-detects (WiFi then Ethernet). Force one with `--spothighlevel.ip=<addr>`.

Keyboard teleop: WASD move/turn, QE strafe, Space soft-stop, ESC quit. A Rerun
viewer opens with the fisheye/depth cameras and odometry.

## Available Blueprints

| Blueprint | Description |
|-----------|-------------|
| `dimos run spot` | Velocity teleop + camera/depth/odometry streaming + Rerun |
| `dimos run spot-record` | Same as `spot`, plus records every stream to a `.db` |

## Layout

Spot support is experimental and lives in `dimos/experimental/robot/bosdyn/spot/`.

- `config.py`: constants + pure address/credential helpers (no `bosdyn` import).
- `effectors/high_level.py`: `SpotHighLevel`, the single Spot module. It handles
  lease/E-stop/power/stand + velocity commands plus the five fisheye + five depth
  cameras and body odometry.
- `recorder.py`: `SpotRecorder` records every Spot stream to disk.
- `blueprints/spot.py`: the runnable `spot` blueprint (click/teleop + sensors + Rerun).

## Dimos Tools

- [Visualization](/docs/usage/visualization.md): Rerun, performance tuning
- [Data Streams](/docs/usage/data_streams/index.md): RxPY streams, backpressure, quality filtering
- [Transports](/docs/usage/transports/index.md): Zenoh, LCM, DDS
- [Blueprints](/docs/usage/blueprints.md): composing modules
