# PX4 drone

Fly a PX4 quadcopter from dimOS over MAVLink Offboard.

Status: the SITL gate passes on PX4 v1.16.2. An earlier revision was bench-tested on a
Pixhawk 6C with props off. Not flown outdoors yet.

## How it works

- `Px4DroneConnection` is the only module that talks to PX4, over one MAVLink UDP socket.
- It publishes the vehicle as streams: odometry, imu, gps, battery, gimbal attitude,
  status.
- It runs the flight supervisor, a 20 Hz state machine that streams Offboard setpoints:
  `IDLE > PREFLIGHT > STREAMING > OFFBOARD_REQ > ARMING > TAKEOFF > HOVER > LANDING > IDLE`.
  From `HOVER` the operator selects a guidance mode or sends a go-to (`GOTO`). A selected
  mode ends with its flight: the next takeoff hovers.
- Commands are RPCs on the connection. There is no arm, mode or raw-setpoint RPC.
- Every other module is optional and binds to these streams by name. Without one you lose
  what it provides; everything else still starts.

"In" is where a module runs; `px4-drone` and `px4-sitl` stand for every blueprint built on
them (see Run).

| Module | Does | Reads | Writes | In |
|---|---|---|---|---|
| `Px4DroneConnection` | MAVLink bridge, flight supervisor | cmd_vel | odometry (twist: linear in `odom`, angular in `base_link`), odom, tf, imu, gps, battery, gimbal_attitude, vehicle_status, statustext, supervisor_state | all |
| `RtspCamera` | H.265 stream in; passthrough video, decoded frames, small JPEG out | none (video and JPEG rate are RPCs) | video, color_image, color_jpeg | `px4-drone`, `px4-sitl` (generated clip) |
| `SiyiA8Gimbal` | Gimbal tf chain, camera intrinsics, aim requests | gimbal_attitude, target_los | tf, camera_info, gimbal_target | `px4-drone`, `px4-sitl` |

## Install

```bash
uv sync --extra px4
```

## Run

| Blueprint | Runs |
|---|---|
| `px4-basic` | connection, viewer |
| `px4-drone` | the aircraft: connection, viewer and every module listed for it above |
| `px4-sitl` | the same against PX4 SITL |
| `px4-teleop`, `px4-sitl-teleop` | those two with the viewer's keyboard on `cmd_vel` |

```bash
dimos run px4-drone
dimos --record sqlite run px4-drone                          # every stream to recordings/<run-id>/memory.db
dimos --rerun-open none --rerun-host 0.0.0.0 run px4-drone   # on the aircraft, viewer on a laptop
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

- Every command returns `{"accepted": bool, "rejection": str | None, "state": str}`.
- `takeoff` runs preflight first: enable switch on, 3D GPS fix with a horizontal accuracy
  (the receiver's `h_acc`, not HDOP) under 1.5 m, valid position estimate, battery at 40 % or
  more, disarmed, on the ground.
- `go_to` flies at 1 m/s or less, ends in `HOVER`, and gives up into `HOVER` after 75 s.
  It is refused unless the goal is 2 m inside the fence (30 m) and the ceiling (15 m).
  `set_guidance_mode("HOVER")` stops it.
- Limits are in `config.py` (`SupervisorLimits`, `GotoConfig`).

## Keyboard

1. `dimos run px4-teleop` (or `px4-sitl-teleop`).
2. `drone.takeoff(2.0)`, then `drone.set_guidance_mode("TELEOP")`.
3. Click the keyboard overlay in the viewer.
4. W/S forward and back, Q/E strafe, A/D turn, Shift faster, Space stop.

Keys are ignored outside `TELEOP`. Speeds are clamped to 1.5 m/s (W+Q together too) and
0.8 rad/s. Altitude
stays where `TELEOP` began. When keys stop for 0.5 s the vehicle holds position.

## Test

### 1. Unit tests

No hardware, no simulator.

```bash
uv sync --extra px4
uv run pytest dimos/robot/px4 dimos/hardware/gimbal/siyi dimos/hardware/sensors/camera/rtsp \
    dimos/msgs/px4_msgs
```

### 2. SITL gate

1. Close QGroundControl. The gate binds UDP 14550 to send the ground-station heartbeat PX4
   needs before it arms.
2. In a PX4-Autopilot checkout: `HEADLESS=1 make px4_sitl gz_x500`. Wait for
   `Startup script returned successfully`.
3. `uv run python dimos/robot/px4/tool_sitl_gate.py --fly`
4. The last line must be `GATE PASS`.

It runs `px4-sitl` and checks:

- odometry at 25 distinct vehicle samples a second or more
- video frames arriving
- the flight: a takeoff cancelled by the enable switch before arming, takeoff to 2 m, go
  2 m south at 3 m, a go-to past the fence refused, a held key moving the vehicle at a
  locked altitude, land

The gate prints its numbers and ends with `GATE PASS` or `GATE FAIL`. Do not run it next to
another dimOS process: they share one zenoh bus. `ZENOH_SCOUT_ADDR=224.0.0.231:7461` in the
environment gives a gate its own.

### 3. Fly SITL by hand

1. `make px4_sitl gz_x500` in the PX4 checkout.
2. Open QGroundControl. PX4 will not arm without a ground station.
3. `dimos run px4-sitl-teleop`
4. `dimos shell`, then the commands above, starting with `drone.sitl_enable(True)`.

## Aircraft setup

1. Add a mavlink-router endpoint on the companion computer and restart the router:

   ```ini
   [UdpEndpoint dimos]
   Mode = Normal
   Address = 127.0.0.1
   Port = 14556
   ```

2. Set these PX4 parameters in QGC and reboot. Check the names against your PX4 version;
   they have not been verified on a vehicle yet.

   `COM_OBL_RC_ACT=5`, `COM_OF_LOSS_T=0.5`, `COM_RC_OVERRIDE=3`, `NAV_RCL_ACT=2`,
   `COM_RC_LOSS_T=0.5`, `GF_ACTION=2`, `GF_MAX_HOR_DIST=50`, `GF_MAX_VER_DIST=30`,
   `COM_DISARM_LAND=2`, `MPC_XY_VEL_MAX=3`, `MPC_Z_VEL_MAX_UP=1.5`, `MPC_Z_VEL_MAX_DN=1.0`,
   `COM_ARM_WO_GPS=0`, `COM_HOME_EN=1`.

3. Put the enable switch on RC channel 7 (`SupervisorLimits.enable_channel`). High allows
   the software to fly. Low, or a channel the receiver does not deliver, refuses takeoff
   and, in flight, aborts to Hold. RC loss itself is PX4's failsafe (`NAV_RCL_ACT`); the
   supervisor only reads the switch.
4. Leave the RC arm switch off. The supervisor arms over MAVLink and refuses if the vehicle
   is already armed.
5. Pass the site's values to `dimos run px4-drone`:

   | Flag | Unset | Set it to |
   |---|---|---|
   | `--rtspcamera.url` | pinned by the blueprint to `A8_RTSP_URL` (`config.py`), the A8's factory address | the camera's URL, if it moved |
   | `--siyia8gimbal.ip` | SIYI SDK off: no zoom poll | the A8's address |
   | `--siyia8gimbal.mount_xyz` | pinned by the blueprint to `GIMBAL_MOUNT_XYZ_UNMEASURED` (`config.py`), a placeholder; nothing warns | `[x,y,z]` measured from `base_link` to the gimbal base, metres, FLU |

6. Sync the companion computer's clock (NTP/chrony) before takeoff: staleness checks and
   the setpoint stream run on it.
7. Bench test with props off before flying: refusal with the enable switch low, a full
   takeoff-to-land sequence with it high, and pilot takeover by moving the mode switch.

Only one Offboard writer may run. The connection binds UDP 5610 as a lock and refuses to
start while another process holds it.

## Safety rules

1. The RC pilot always wins. When PX4 leaves Offboard the supervisor stops sending, goes to
   `IDLE` (`PILOT_OVERRIDE`) and never sends a mode command.
2. One writer of setpoints: the connection's tick thread. If it stops, PX4's Offboard-loss
   failsafe takes over.
3. E-STOP is Hold plus a latch. `estop_clear` only works in `IDLE`.
4. Takeoff altitudes and go-to goals are checked against the fence, the ceiling and
   `min_alt_m` before anything moves. `GOTO` obeys the same abort rules as every armed
   state.
