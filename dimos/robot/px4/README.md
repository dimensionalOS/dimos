# PX4 drone

Fly a PX4 multicopter from dimOS over MAVLink Offboard: takeoff, go-to, land and keyboard
teleop, with an RC pilot who can always take over. Diagrams and an overview of the whole
package: [PX4 drone](/docs/platforms/drone/px4/index.md).

## Modules

Every module but the connection is optional and binds to its streams by name.

| Module | Does | Reads | Writes | In |
|---|---|---|---|---|
| `Px4DroneConnection` | MAVLink bridge, flight supervisor | cmd_vel | odometry (twist: linear in `odom`, angular in `base_link`), odom, tf, imu, gps, battery, gimbal_attitude, vehicle_status, statustext, supervisor_state | all |
| `RtspCamera` | H.264/H.265 stream in; passthrough video, decoded frames, small JPEG out | none (video and JPEG rate are RPCs) | video, color_image, color_jpeg | `px4-a8` |
| `SiyiA8Gimbal` | Gimbal tf chain, camera intrinsics, aim requests | gimbal_attitude, target_los | tf, camera_info, gimbal_target | `px4-a8` |
| `Px4SkillContainer` | The flight commands as agent skills | none (calls the connection's RPCs) | none | `px4-agentic`, `px4-sitl-agentic` |

## Install

```bash
uv sync --extra px4
uv sync --extra px4 --extra agents        # for px4-agentic
```

## Run

| Blueprint | Runs |
|---|---|
| `px4` | connection, viewer and its keyboard: any PX4 vehicle |
| `px4-sitl` | the same against PX4 SITL, with the RC enable switch faked |
| `px4-a8` | `px4` plus the SIYI A8 camera and gimbal |
| `px4-agentic`, `px4-sitl-agentic` | `px4` and `px4-sitl` with the skills, the MCP server and the LLM agent |

```bash
dimos run px4                                             # router on PX4's companion port
dimos run px4 --px4droneconnection.link=radio             # router on a telemetry radio
dimos --record sqlite run px4                             # every stream to recordings/<run-id>/memory.db
dimos --rerun-open none --rerun-host 0.0.0.0 run px4-a8   # on the aircraft, viewer on a laptop
```

With the last form, run the `dimos-viewer --connect ...` line it logs on the laptop.

Each module also runs alone under its kebab-case name and binds to whatever else is
running: `dimos run px4-drone-connection`, `dimos run siyi-a8-gimbal`.

## Commands

From `dimos shell`:

```python
drone = app.Px4DroneConnection
drone.sitl_enable(True)                      # SITL only: stands in for the RC enable switch
drone.takeoff(2.0)                           # metres above ground; default 3.0
drone.go_to(north_m=-2, altitude_m=3)        # 2 m south of here, 3 m above the takeoff point
drone.go_to(relative=False, heading_deg=90)  # back over the takeoff point, facing east
drone.set_guidance_mode("TELEOP")            # or "HOVER"
drone.land()
drone.estop()                                # Hold and latch; estop_clear() works in IDLE
drone.status()
```

- Every command returns `{"accepted": bool, "rejection": str | None, "state": str}` at once;
  `drone.status()["state"]` shows the flight, `HOVER` once a takeoff or go-to is done.
- `takeoff` runs preflight first: enable switch on, 3D GPS fix with a horizontal accuracy
  (the receiver's `h_acc`, not HDOP) under 1.5 m, valid position estimate, a battery reading
  of 40 % or more (`SupervisorLimits.require_battery=False` for a vehicle without a power
  module), disarmed, on the ground. In flight, PX4 dropping the position estimate aborts:
  the setpoints stop and the supervisor asks for Hold, which PX4 may refuse without a
  position; its own failsafe then decides.
- `go_to` flies at 1 m/s or less, ends in `HOVER`, and gives up into `HOVER` after 75 s.
  It is refused unless the goal is 2 m inside the fence (30 m) and the ceiling (15 m).
  `set_guidance_mode("HOVER")` stops it.
- Limits are in `config.py` (`SupervisorLimits`, `GotoConfig`). Override them as JSON; the
  rest keep their defaults: `--px4droneconnection.limits '{"enable_channel": 8}'`.

## Keyboard

1. `dimos run px4` (or `px4-sitl`, `px4-a8`).
2. `drone.takeoff(2.0)` (SITL: `drone.sitl_enable(True)` first), then
   `drone.set_guidance_mode("TELEOP")`. The mode starts once the hover does.
3. Click the keyboard overlay in the viewer.
4. W/S forward and back, Q/E strafe, A/D turn, Shift faster, Space stop.

Keys are ignored outside `TELEOP`. Speeds are clamped to 1.5 m/s (W+Q together too) and
0.8 rad/s. Altitude stays where `TELEOP` began. When keys stop for 0.5 s the vehicle holds
position.

## Agent

1. `uv sync --extra px4 --extra agents` and set `OPENAI_API_KEY`.
2. `dimos run px4-agentic` (or `px4-sitl-agentic`, with QGroundControl open and
   `drone.sitl_enable(True)` in `dimos shell` first, as in Test).
3. Talk to it in `dimos humancli`, or send commands:

```bash
dimos agent-send "take off to 2 meters"
dimos agent-send "go to 2 meters south at 3 m altitude"
dimos mcp call go_to --arg north_m=-2 --arg altitude_m=3 --timeout 120   # the same skill, no LLM; it blocks until the manoeuvre ends
```

Skills: `takeoff`, `go_to`, `land`, `set_guidance_mode`, `flight_status`. A skill is the
connection RPC plus a wait for the outcome. The supervisor refuses a skill exactly as it
refuses the RPC.

## Test

Unit tests, no hardware and no simulator:

```bash
uv sync --extra px4
uv run pytest dimos/robot/px4 dimos/hardware/gimbal/siyi dimos/hardware/sensors/camera/rtsp \
    dimos/msgs/px4_msgs
```

In PX4 SITL:

1. In a PX4-Autopilot checkout: `make px4_sitl gz_x500`.
2. Open QGroundControl. PX4 SITL will not arm without a ground station.
3. `dimos run px4-sitl`
4. In `dimos shell`: `drone = app.Px4DroneConnection`, `drone.sitl_enable(True)`,
   `drone.takeoff(2.0)`. Once `drone.status()["state"]` is `HOVER`:
   `drone.go_to(north_m=-2, altitude_m=3)`, then `drone.land()`. The keyboard as above.

## Aircraft setup

dimOS reads PX4 from a mavlink-router endpoint on its own machine,
`udpin:127.0.0.1:14556`. The router owns the link to the aircraft and hands QGroundControl
its own endpoint, so the two never fight over a port. Only the router's far side and the
connection's `link` differ:

| Setup | dimOS and the router run on | The router faces | `link` | Blueprint |
|---|---|---|---|---|
| Bare kit | the laptop | the telemetry radio on USB (PX4 TELEM1, 57600) | `radio` | `px4`, `px4-agentic` |
| Companion computer | the computer on the aircraft | PX4 TELEM2 in Onboard mode, 921600 | `onboard` | `px4`, `px4-agentic`, `px4-a8` |

`link` sets the message rates the connection asks PX4 for at start (`LINK_STREAM_HZ` in
`config.py`). On TELEM1 PX4 sends local position at 1 Hz, below what the supervisor accepts,
so `radio` asks for 10 Hz. `onboard` asks for nothing.

### Laptop and telemetry radio

1. Build mavlink-router once:

   ```bash
   sudo apt install meson ninja-build pkg-config g++
   git clone --recurse-submodules https://github.com/mavlink-router/mavlink-router.git
   cd mavlink-router && meson setup build . && ninja -C build && sudo ninja -C build install
   ```

2. Save as `radio.conf`, with the radio's name from `ls /dev/serial/by-id/`, and run
   `mavlink-routerd -c radio.conf`:

   ```ini
   [General]
   TcpServerPort = 0

   [UartEndpoint radio]
   Device = /dev/serial/by-id/usb-FTDI_FT231X_USB_UART_XXXXXXXX-if00-port0
   Baud = 57600

   [UdpEndpoint qgc]
   Mode = Normal
   Address = 127.0.0.1
   Port = 14550

   [UdpEndpoint dimos]
   Mode = Normal
   Address = 127.0.0.1
   Port = 14556
   ```

3. In QGroundControl, Application Settings > AutoConnect: untick SiK Radio (and the other
   serial devices), keep UDP. QGC then connects through the router on 14550 instead of
   taking the radio's port.
4. PX4 parameters for TELEM1: `MAV_0_RATE=0` and `COM_OF_LOSS_T=1.0` in place of the 0.5
   below (a radio drops half a second now and then). `MAV_0_RATE=0` makes the port's budget
   baud / 20, 2880 B/s: local position at about 8 Hz, about 130 ms old. At the default 1200
   B/s it is about 3 Hz and 200-470 ms old, close to the supervisor's limits. PX4 sends no
   HIGHRES_IMU on TELEM1, so `imu` stays silent; the odometry's body rates come from ATTITUDE.
5. `dimos run px4 --px4droneconnection.link=radio`, or `px4-agentic` with the same flag and
   `dimos humancli`.

The supervisor aborts when PX4 goes silent for 1 s; the Hold it sends needs the link, so
PX4's Offboard-loss failsafe (`COM_OBL_RC_ACT`) is what catches a longer outage. The RC pilot
or QGC finishes that flight: the connection never commands a vehicle that has left
Offboard. PX4 forgets the requested rates when it reboots (a battery swap); the connection
asks again when it sees the new boot. That is not confirmed on hardware yet: restart dimOS
after a battery swap.

### Companion computer

1. PX4 TELEM2 in Onboard mode at 921600 (`MAV_1_CONFIG=102`, `MAV_1_MODE=2`,
   `SER_TEL2_BAUD=921600`), wired to the computer.
2. Add a mavlink-router endpoint on the computer and restart the router:

   ```ini
   [UdpEndpoint dimos]
   Mode = Normal
   Address = 127.0.0.1
   Port = 14556
   ```

3. QGC reaches the aircraft over Wi-Fi or cellular through another endpoint of the same
   router. `dimos shell` runs on the aircraft (the zenoh bus is local).
4. With the A8, pass the site's values to `dimos run px4-a8`:

   | Flag | Unset | Set it to |
   |---|---|---|
   | `--rtspcamera.url` | pinned by the blueprint to the A8's factory address | the camera's URL, if it moved |
   | `--siyia8gimbal.ip` | SIYI SDK off: no zoom poll | the A8's address |
   | `--siyia8gimbal.mount-xyz` | the camera sits at `base_link`; the gimbal warns at start | `[x,y,z]` measured from `base_link` to the gimbal base, metres, FLU |

### Both setups

1. Set these PX4 parameters in QGC and reboot. Check the names against your PX4 version.

   `COM_OBL_RC_ACT=5`, `COM_OF_LOSS_T=0.5`, `COM_RC_OVERRIDE=3`, `NAV_RCL_ACT=2`,
   `COM_RC_LOSS_T=0.5`, `GF_ACTION=2`, `GF_MAX_HOR_DIST=50`, `GF_MAX_VER_DIST=30`,
   `COM_DISARM_LAND=2`, `MPC_XY_VEL_MAX=3`, `MPC_Z_VEL_MAX_UP=1.5`, `MPC_Z_VEL_MAX_DN=1.0`,
   `COM_ARM_WO_GPS=0`, `COM_HOME_EN=1`, `COM_RCL_EXCEPT=0` (RC loss stays a failsafe in
   Offboard: a receiver that holds its last values keeps the enable switch reading high).

2. Put the enable switch on an RC channel, 7 by default (`enable_channel`). High allows
   the software to fly. Low, or a channel the receiver does not deliver, refuses takeoff
   and, in flight, aborts to Hold. RC loss itself is PX4's failsafe (`NAV_RCL_ACT`); the
   supervisor only reads the switch.
3. Leave the RC arm switch off. The supervisor arms over MAVLink and refuses if the vehicle
   is already armed.
4. Keep QGC connected. With the data-link-loss failsafe on (`NAV_DLL_ACT`), PX4 will not
   arm without a ground station, and the connection is not one.
5. Sync the clock of the machine running dimOS (NTP/chrony) before takeoff: staleness
   checks and the setpoint stream run on it.
6. Bench test with props off before flying: refusal with the enable switch low, a full
   takeoff-to-land sequence with it high, and pilot takeover by moving the mode switch.

Only one Offboard writer may run. The connection binds UDP 5610 as a lock and refuses to
start while another process holds it.

## Safety rules

1. The RC pilot always wins. When PX4 leaves Offboard the supervisor stops sending, goes to
   `IDLE` (`PILOT_OVERRIDE`) and never sends a mode command. A disarm stops it the same way.
2. One writer of setpoints: the connection's tick thread. If it stops, PX4's Offboard-loss
   failsafe takes over.
3. E-STOP is Hold plus a latch. `estop_clear` only works in `IDLE`.
4. Takeoff altitudes and go-to goals are checked against the fence, the ceiling and
   `min_alt_m` before anything moves. `GOTO` obeys the same abort rules as every armed
   state.
5. `sitl=True` (the enable switch faked by `sitl_enable`, RC checks skipped) is refused
   unless `mav_url` is PX4 SITL's port, 14540.
