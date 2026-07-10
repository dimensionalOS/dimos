# R1 Pro DiMOS Integration — Setup & Connection Guide

## Overview

This directory contains diagnostic scripts and the rerun-bridge launcher for
validating DiMOS connectivity to the Galaxea R1 Pro humanoid robot over
ethernet. The robot runs ROS2 Humble on a Jetson Orin (Ubuntu 22.04 / L4T).
The laptop runs DiMOS inside a **Humble-on-22.04 docker container** so the
ROS2 stack matches the robot exactly (no cross-version DDS issues).

**Current status (2026-07-09)**: Migrated to the **new-gen R1 Pro**
(firmware **V2.3.0**, `ROS_DOMAIN_ID=1`). V2.3.0 replaced the standalone
mobiman control nodes with a unified `moca_adapter` that refuses external
chassis commands — we bypass it and run the standalone
`chassis_control_node` (which still ships) via the `~/galaxea-dimos` tree.
Chassis verified driving smoothly. See §14 for the full migration story.
Connection module + coordinator blueprint run end-to-end; sensor streams
flow through LCM into the rerun bridge. Manipulation (planner-coordinator)
blueprint researched but **not yet implemented** (§12.E). Stream-drop
elimination on the new gen in progress —
[NEWGEN_DROP_ELIMINATION.md](NEWGEN_DROP_ELIMINATION.md).

**2026-07-10 (§15): on-robot docker image verified end-to-end once**
(coordinator + all transports + rerun bridge on the robot), then a
stream-rate collapse was diagnosed (3 causes, see §15) and fixes were
committed — **but the post-reboot re-deploy is NOT currently working**:
as of end-of-session the coordinator process starts but spawns no workers
and publishes nothing on LCM port 7300, while something (likely a DDS
participant again) holds 7667. Pickup point: confirm the running container
was built from the current Dockerfile (`docker exec dimos-r1pro env | grep
LCM_DEFAULT_URL` must show port 7300; empty = old image → rebuild), then
re-run and check the coordinator's own terminal output for where it stalls.

---

## Network Setup

### Physical Connection
- Connect laptop to robot via ethernet cable
- Robot ethernet port: `eth1` on the robot

### Robot IP (persistent after netplan config)
- **New-gen robot**: `192.168.1.88` (laptop ethernet: `192.168.1.100`)
- Old-gen robot `eth1`: `192.168.123.150/24` (laptop ethernet
  `enxf8e43bb7046c`: `192.168.123.100/24`)

### Set laptop ethernet IP (if not already set)
```bash
sudo ip addr add 192.168.1.100/24 dev enxf8e43bb7046c     # new-gen robot
sudo ip addr add 192.168.123.100/24 dev enxf8e43bb7046c   # old-gen robot
```

### SSH into robot
```bash
ssh nvidia@192.168.1.88        # new-gen robot
ssh nvidia@192.168.123.150     # old-gen robot
# password: nvidia
```

### Make robot IP persistent across reboots (old-gen robot — already done)
Edit `/etc/netplan/50-cloud-init.yaml` on the robot, add `192.168.123.150/24`
to eth1 addresses:
```yaml
eth1:
  dhcp4: true
  addresses: [192.168.2.150/24, 192.168.123.150/24]
```
Then: `sudo netplan apply`

---

## Robot Startup Procedure — NEW-GEN robot (V2.3.0 — updated 2026-07-09)

The new-gen R1 Pro runs firmware **V2.3.0** on **`ROS_DOMAIN_ID=1`** and
starts CAN via **`canfd.sh`** (the current script name on the r1pro — older
docs referenced `can.sh`). Same dual-tree pattern as the old gen — boot ONE
tree at a time (they share tmux session names; `./robot_startup.sh kill`
switches between them):

| Tree | Behavior |
|---|---|
| `~/galaxea` | 100% factory stock (V2.3.0). The RC teleop drives the chassis via `moca_adapter`. Never modified. |
| `~/galaxea-dimos` | Byte-for-byte copy of stock + the 4-item diff below. Bypasses moca: standalone control stack + chassis gatekeeper, RC teleop disabled. Chassis accepts external commands. |

```bash
# Step 1: CAN bus driver
bash ~/canfd.sh

# Step 2: Boot the Dimensional tree (kill first if the other tree is up)
cd ~/galaxea-dimos/install/startup_config/share/startup_config/script
./robot_startup.sh kill
./robot_startup.sh boot ../sessions.d/ATCStandard/R1PROBody.d/

# Step 3: Wait ~30s for HDAS (arms open/close = healthy). The gatekeeper
#         opens all chassis gates automatically — no --controller-mode
#         flags, no relaunch script, no manual gatekeeper step.

# Step 4: Smoke-test the chassis with a plain drive (no gate flags needed):
python3 ~/chassis_poke.py --vx 0.12 --duration 4.0
```

```bash
# Verify everything is publishing (run on robot or laptop)
source ~/galaxea-dimos/install/setup.bash
export ROS_DOMAIN_ID=1
ros2 topic list --no-daemon | grep -E 'hdas|lidar' | head -20

# Spot-check rates on the streams the connection consumes:
ros2 topic hz /hdas/camera_wrist_left/color/image_raw/compressed          # ~15 Hz
ros2 topic hz /hdas/camera_wrist_right/color/image_raw/compressed         # ~15 Hz
ros2 topic hz /hdas/camera_head/left_raw/image_raw_color/compressed       # ~15 Hz
```

### What the `~/galaxea-dimos` tree changes (the entire diff vs stock)

After a firmware reflash, re-rsync `~/galaxea` → `~/galaxea-dimos` and
re-apply these (same recovery pattern as §13):

- **`sessions.d/.../mobiman.yaml`** — moca replaced with the standalone
  stack (chassis, joint-PID, gripper, EE-pose nodes) + a gatekeeper pane
  running `start_chassis_gatekeeper.sh`.
- **`mobiman/.../r1_pro_chassis_control_launch.py`** — `/controller` →
  `/controller_unused` remap on the chassis node (Gate 2 workaround).
- **New `~/chassis_gatekeeper.py` + `start_chassis_gatekeeper.sh`** —
  holds `mode=5` on `/controller_unused` + `brake_mode=False` + a nonzero
  `acc_limit`. All gate handling now lives on the robot.
- **`sessions.d/.../teleop.yaml` → `.disabled`** — stock V2.3.0's
  `homogeneous_teleoperation` co-publishes zeros onto
  `/motion_target/target_speed_chassis` at 200 Hz, out-shouting external
  commands. Disabled so there's no RC competitor.

Notes for the new gen:

- **Base has NO surround chassis cameras** (lidars only); sensors are the
  head/top cameras + 2 wrist D405s. `enable_chassis_cameras` stays `False`
  in `R1ProConnectionConfig` (the default).
- **Lidar is not auto-started** by this tree yet. `start_livox_lidar.sh`
  ships stock — add one line in `hdas.yaml` when you want it.
- Arms/torso/grippers run on the standalone nodes but were **not
  re-verified** in the 2026-07-09 session (chassis was).

If a wrist camera reports `RS2_USB_STATUS_BUSY` or repeatedly disconnects
(check `~/.ros/log/realsense2_camera_node_*_*.log`), it's a USB-layer fault
— reseat the cable on that camera and rerun
`start_realsense_camera_r1pro.sh`.

### Old-gen robot (V2.2.1) startup — HISTORICAL RECORD

Kept for reference / if the old-gen robot comes back into service. The
old-gen trees were `~/galaxea` (stock V2.2.1) and `~/dimos_galaxea`
(Dimensional copy), on **`ROS_DOMAIN_ID=41`**. The CAN script was
documented as `can.sh` at the time; the r1pro's updated script is
`canfd.sh` — use whichever exists in `~`:

```bash
# Step 1: CAN bus driver
bash ~/can.sh        # (updated script: ~/canfd.sh)

# Step 2: Boot the Dimensional tree (or ~/galaxea/... for stock)
cd ~/dimos_galaxea/install/startup_config/share/startup_config/script
./robot_startup.sh boot ../sessions.d/ATCStandard/R1PROBody.d/

# Step 3: Wait ~30s for HDAS, then drive from the laptop via /cmd_vel.
#         The tree started lidar, chassis cams, and gatekeeper itself.
# Verify: source ~/dimos_galaxea/install/setup.bash; export ROS_DOMAIN_ID=41
```

The old `~/dimos_galaxea` diff vs stock was four files:

- **`mobiman/.../r1_pro_chassis_control_launch.py`** — `/controller` →
  `/controller_unused` remap (Gate 2 workaround, one line).
- **`sessions.d/.../mobiman.yaml`** — 5th pane runs
  `start_chassis_gatekeeper.sh` (gatekeeper at `~/dimos_galaxea/chassis_gatekeeper.py`;
  also does `ip link set lo multicast on`).
- **`sessions.d/.../hdas.yaml`** — ZED pane (no ZED on that robot) replaced
  with `start_livox_lidar.sh`; 5th pane runs `start_chassis_cameras_v4l2.sh`
  (v4l2 launch at `~/dimos_galaxea/chassis_cameras_v4l2.launch.py`).

Key differences vs the new gen: V2.2.1 had **no moca** — the standalone
`chassis_control_node` was already the control path, so mobiman.yaml only
*added* a gatekeeper pane rather than swapping the whole control stack;
HDAS flooded `/controller` with **mode 2** (V2.3.0 floods mode 3 — same
lock, same remap fix); there was no `teleop.yaml` RC competitor to disable;
the base had 5 surround chassis cams (started via v4l2 launch) which the
new base doesn't have. Old-gen rate expectations included
`ros2 topic hz /hdas/lidar_chassis_left` at ~10 Hz.

On a clean boot HDAS (both gens) also brings up the **RealSense wrist
cameras** (left + right D405) via `start_realsense_camera_r1pro.sh` and the
**head signal camera** via `start_signal_camera_head.sh`.

### Robot tmux sessions
| Session | Purpose |
|---|---|
| `ros_discovery` | FastDDS discovery server on port 11811 (for VR/WiFi, not needed for ethernet) |
| `mobiman` | Main motion control stack |
| `hdas` | Hardware abstraction — arms, chassis, torso, grippers |
| `tools` | Utilities |

Check session health: `tmux attach -t hdas` (Ctrl+B D to detach)

---

## Laptop Setup (every session)

DiMOS runs inside the dev docker container. Open a shell into it via the
`./bin/dev` wrapper or `docker compose exec`. Inside the container:

```bash
cd /app
source .venv/bin/activate                      # Python 3.10 venv (see §6 + §12)
source /opt/ros/humble/setup.bash              # Humble's rclpy on PYTHONPATH
export ROS_DOMAIN_ID=1                         # new-gen robot (old gen was 41)
```

The domain ID is the **only** laptop-side change between old-gen and
new-gen robots — nothing in the dimos code hardcodes it; `R1ProConnection`
picks it up from the environment.

**One-time on a fresh checkout** — opt into the r1pro direnv template
(see §12.A for the rationale):

```bash
ln -sf .envrc.r1pro .envrc                     # exports UV_PYTHON=3.10
direnv allow                                   # auto-activates per-dir
rm -rf .venv
uv sync --all-extras --no-extra dds --no-extra unitree-dds
```

`requires-python = ">=3.10"` in `pyproject.toml`, with `onnxruntime<1.24`
and `onnxruntime-gpu>=1.17.1,<1.24` upper-bounded so cp310 wheels are
available (PyPI's 1.24.x dropped Python 3.10 support on 2026-02-05).
The repo's `.python-version` stays at `3.12` (project default); the
direnv template overrides it only inside this checkout for opted-in devs.

The XML profile `fastdds_r1pro.xml` is no longer required — the docker
container uses Humble's default FastDDS multicast over the `--network=host`
ethernet link. The XML stays in this directory as a fallback for networks
that block multicast.

---

## Chassis Gatekeeper (Key Concept)

The R1 Pro `chassis_control_node` has **three internal gates** that all must be
unlocked simultaneously for chassis movement to work. On the new gen
(V2.3.0) the on-robot gatekeeper owns **all** gate handling — a plain
`TwistStamped` on `/motion_target/target_speed_chassis` drives the chassis;
the laptop side needs no gate logic. (`R1ProConnection` still opens Gate 1
and Gate 3 itself from the old-gen days — harmless/redundant now, see §12.)

Note the gates live in the **standalone** `chassis_control_node`. V2.3.0's
stock control path is the new unified `moca_adapter`, which refuses external
chassis commands entirely (no gate we could find unlocks it) — the
`~/galaxea-dimos` tree bypasses moca and runs the standalone node instead
(see §14).

### The 3 Gates

| Gate | What blocks it | Where it's opened |
|---|---|---|
| **Gate 1**: Subscriber count on `/motion_control/chassis_speed` | Node skips IK if nobody subscribes | Gatekeeper subscribes; `R1ProConnection._on_chassis_speed` also subscribes (drives odom dead-reckoning) |
| **Gate 2**: `breaking_mode_` flag from `/controller` | HDAS floods `/controller` with a lock mode at 200 Hz — **mode 2 on V2.2.1, mode 3 on V2.3.0**; unlock is mode 5. Fighting HDAS directly = jerky. | On-robot: launch remaps `/controller` → `/controller_unused`, gatekeeper holds `mode=5` there uncontested |
| **Gate 3**: `acc_limit` defaults to zero | `calculateNextVelocity` uses `acc_limit * dt` which stays 0 | Gatekeeper publishes a nonzero acc_limit (new gen); `R1ProConnection._on_cmd_vel` also publishes it per Twist command |

The gatekeeper also publishes `brake_mode=False` (the "4th gate" —
`/motion_target/brake_mode`, see §13).

### Prerequisites (baked into the `~/galaxea-dimos` tree)
The `/controller` → `/controller_unused` remap, the standalone-stack swap,
and the gatekeeper auto-start all live in the Dimensional tree's
launch/session files. Launch files live only in the **install** tree
(vendor firmware ships no `src/`) — edit them directly, no rebuild.
Booting `~/galaxea-dimos` starts the gatekeeper for you; no manual
`python3 ~/chassis_gatekeeper.py` step.

### Running
```bash
# From laptop container — direct LCM publish to test the chassis path:
dimos run r1pro-coordinator         # in one shell
# Then: publish a Twist on LCM /cmd_vel from another shell or use the keyboard
# teleop blueprint below.
```

---

## DiMOS Integration Architecture (current — 2026-05-09)

### One Module owns the ROS 2 connection

```
                     coordinator-side                                  robot-side
   ┌────────────────────────────────────────────────┐    ┌────────────────────────────┐
   │  ControlCoordinator                            │    │  R1ProConnection (Module)  │
   │   ├─ TransportWholeBodyAdapter (hw="r1pro")    │◄──►│  In[MotorCommandArray]     │
   │   │     publishes /r1pro/motor_command         │    │  Out[JointState]   (18-DOF)│
   │   │     subscribes /r1pro/motor_states + /imu  │    │  Out[Imu] x2               │
   │   │                                            │    │                            │
   │   └─ TransportTwistAdapter      (hw="chassis") │◄──►│  In[Twist]                 │
   │         publishes /chassis/cmd_vel             │    │  Out[PoseStamped] (odom)   │
   │         subscribes /chassis/odom               │    │                            │
   └────────────────────────────────────────────────┘    │  Out[Image] x9             │
                                                         │  Out[PointCloud2]          │
                                                         └────────────┬───────────────┘
                                                                      │ ROS 2 (FastDDS)
                                                          ┌───────────▼──────────┐
                                                          │  Galaxea R1Pro robot │
                                                          └──────────────────────┘
```

`R1ProConnection` runs one control RawROS node + a separate isolated
`rclpy.Context` for sensor subscriptions (so heavy fragmented camera/lidar
UDP doesn't starve the control DDS receive thread). All 10 ROS subscriptions
are inside this Module.

### Files

| Path | Purpose |
|---|---|
| `dimos/robot/galaxea/r1pro/connection.py` | `R1ProConnection` Module + `R1PRO_UPPER_BODY_JOINTS` constant + `R1ProConnectionConfig` |
| `dimos/robot/galaxea/r1pro/blueprints/basic/r1pro_coordinator.py` | Coordinator blueprint: `autoconnect(R1ProConnection, ControlCoordinator)` with WHOLE_BODY + BASE hardware components, both wired through `transport_lcm` |
| `dimos/robot/galaxea/r1pro/blueprints/basic/r1pro_keyboard_teleop.py` | Composes `r1pro_coordinator` + `KeyboardTeleop` for chassis Twist (display throttled to 10 Hz to survive X11 forwarding — see §12.D) |
| `dimos/robot/catalog/galaxea.py` | `R1PRO_MODEL_PATH`, `R1PRO_COLLISION_EXCLUSIONS`, `r1pro_arm()` factory (planner / Drake side only) |
| `dimos/robot/all_blueprints.py` | Registry — `r1pro-coordinator`, `r1pro-keyboard-teleop`, `r1-pro-connection` |

The Rerun viewer layout function (`_r1pro_rerun_blueprint`) lives **inline**
in `r1pro_coordinator.py`, conditionally composed via
`RerunBridgeModule.blueprint(...)` when `global_config.viewer.startswith("rerun")`.
This matches the established Go2 / G1 pattern
([unitree_go2_basic.py](../../dimos/robot/unitree/go2/blueprints/basic/unitree_go2_basic.py),
[uintree_g1_primitive_no_nav.py](../../dimos/robot/unitree/g1/blueprints/primitive/uintree_g1_primitive_no_nav.py))
— no standalone bridge script needed.

**Removed in this refactor** (the old per-segment monolithic adapters from
`task/mustafa/r1pro-dual-arm-testing`):

- `dimos/hardware/manipulators/r1pro/{__init__.py,adapter.py}` — `R1ProArmAdapter` / `R1ProTorsoAdapter` / `R1ProUpperBodyAdapter`
- `dimos/hardware/drive_trains/r1pro/{__init__.py,adapter.py}` — `R1ProChassisAdapter`
- `dimos/hardware/whole_body/r1pro/{__init__.py,adapter.py}` — `R1ProWholeBodyAdapter`
- `dimos/hardware/r1pro_ros_env.py` — Humble↔Jazzy bridging is no longer needed (docker container = Humble end-to-end)
- `dimos/robot/humanoids/r1pro/blueprints.py` — old blueprint zoo (`r1pro-full`, `r1pro-dual-mock`, `r1pro-planner-full`, `r1pro-upper-body-full`, `r1pro-whole-body-full`)

The new pattern mirrors `unitree/g1/wholebody_connection.py` + `unitree/go2/connection.py` — one connection Module, generic `transport_lcm` adapters from `dimos/hardware/whole_body/transport/` and `dimos/hardware/drive_trains/transport/`, no robot-specific adapter code on the coordinator side.

### Joint layout

`R1PRO_UPPER_BODY_JOINTS` (URDF-faithful names, prefixed with `r1pro/`):

| Index range | Segment | Names |
|---|---|---|
| 0–3 | torso | `r1pro/torso_joint1` … `r1pro/torso_joint4` |
| 4–10 | left arm | `r1pro/left_arm_joint1` … `r1pro/left_arm_joint7` |
| 11–17 | right arm | `r1pro/right_arm_joint1` … `r1pro/right_arm_joint7` |

Chassis joint names are `chassis/vx`, `chassis/vy`, `chassis/wz` (3-DOF
holonomic Twist).

### Blueprints

| Registry name | What it composes |
|---|---|
| `r1pro-coordinator` | `R1ProConnection` + `ControlCoordinator` (WHOLE_BODY + BASE hardware, `servo_r1pro` + `vel_chassis` tasks) |
| `r1pro-keyboard-teleop` | `r1pro_coordinator` + `KeyboardTeleop` (display 10 Hz) — drives chassis only |

```bash
dimos run r1pro-coordinator
dimos run r1pro-keyboard-teleop
```

The bare `r1pro-coordinator` exposes a public LCM `/cmd_vel` Twist bus
(both `cmd_vel` and `twist_command` transports point at `/cmd_vel`), so any
external Twist publisher can drive the chassis without a new blueprint —
e.g. phone teleop, joystick, an autonomous policy, or `lcm-spy`-style
test scripts.

---

## Key Topics (current naming)

| LCM topic | Type | Direction | Owner |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | external publisher → coordinator | KeyboardTeleop, others |
| `/chassis/cmd_vel` | `geometry_msgs/Twist` | coordinator → connection | TransportTwistAdapter (hw=chassis) |
| `/chassis/odom` | `geometry_msgs/PoseStamped` | connection → coordinator | dead-reckoned from `/motion_control/chassis_speed` |
| `/r1pro/motor_command` | `sensor_msgs/MotorCommandArray` | coordinator → connection | TransportWholeBodyAdapter (hw=r1pro) |
| `/r1pro/motor_states` | `sensor_msgs/JointState` | connection → coordinator | aggregated 18-DOF feedback |
| `/r1pro/imu` | `sensor_msgs/Imu` | connection → coordinator | chassis IMU (the WholeBody adapter expects `/imu`) |
| `/r1pro/imu_torso` | `sensor_msgs/Imu` | connection → consumers | torso IMU (separate stream) |
| `/r1pro/head_color` | `sensor_msgs/Image` | connection → consumers | head RGB |
| `/r1pro/head_depth` | `sensor_msgs/Image` | connection → consumers | head 32FC1 depth |
| `/r1pro/chassis_front_left` … `_rear` | `sensor_msgs/Image` | connection → consumers | 5 surround chassis cams |
| `/r1pro/lidar` | `sensor_msgs/PointCloud2` | connection → consumers | Livox MID360 |
| `/r1pro/wrist_left_color`, `_depth` | `sensor_msgs/Image` | connection → consumers | left wrist D405 |
| `/r1pro/wrist_right_color`, `_depth` | `sensor_msgs/Image` | connection → consumers | right wrist D405 |
| `/coordinator/joint_state` | `sensor_msgs/JointState` | coordinator → consumers | aggregated coordinator joint state |
| `/r1pro/joint_command` | `sensor_msgs/JointState` | external → coordinator | streaming joint commands (planner, etc.) |

ROS topic names used by `R1ProConnection` (subscribed/published on the
robot side):

| ROS topic | Direction |
|---|---|
| `/hdas/feedback_arm_{left,right}` | robot → connection |
| `/hdas/feedback_torso` | robot → connection |
| `/hdas/imu_chassis`, `/hdas/imu_torso` | robot → connection |
| `/hdas/lidar_chassis_left` | robot → connection |
| `/hdas/camera_head/...`, `/hdas/camera_chassis_*/...`, `/hdas/camera_wrist_{left,right}/...` | robot → connection |
| `/motion_target/target_joint_state_{torso,arm_left,arm_right}` | connection → robot |
| `/motion_target/target_speed_chassis`, `/motion_target/chassis_acc_limit` | connection → robot |
| `/motion_target/brake_mode` | connection → robot |
| `/motion_control/chassis_speed` | robot → connection (Gate 1 + odom) |

---

## Rerun bridge

The bridge is composed into the coordinator blueprint via `--viewer`. One
process, one command:

```bash
dimos --viewer rerun run r1pro-coordinator           # native window
dimos --viewer rerun-web run r1pro-coordinator       # http://localhost:9090
dimos --viewer rerun-connect run r1pro-coordinator   # connect to existing viewer
```

Same flag works for any composing blueprint:

```bash
dimos --viewer rerun run r1pro-keyboard-teleop       # teleop + rerun
```

The layout (inline in `r1pro_coordinator.py` as `_r1pro_rerun_blueprint`)
has two tabs:

- **Main**: wrist_left_color, wrist_right_color, head_color (left column) +
  3D world view (lidar + odom).
- **Surround + depth**: 3×2 grid of the 5 surround chassis cams + head_depth.

Entity paths are `world/r1pro/<stream_name>` (the bridge prefixes `world/`
to the LCM topic name). Default ports `9877` (gRPC) / `9090` (web) come
from `RerunBridgeModule.Config` — override via `--grpc-port` / `--web-port`
if they collide.

---

## Verification Tests (legacy)

The original test scripts (`test_01_topic_discovery.py` ... `test_06_torso_command.py`)
on the dual-arm-testing branch still validate the **robot-side** ROS 2 layer
— they don't depend on DiMOS. They were not ported to this branch but can
be run directly from a copy of the dual-arm branch if needed for hardware
bring-up debugging.

For the DiMOS-side, the canonical verification is now:

1. `dimos run r1pro-coordinator` → boot logs show all 21 transports, the
   connection module gets through `Starting R1ProConnection control RawROS...`
   to `Waiting up to 5s for first feedback from torso/left_arm/right_arm...`
   (or `R1ProConnection started` once the robot is talking).
2. `dimos --viewer rerun-web run r1pro-coordinator` → rerun web viewer at
   `http://localhost:9090`, wrist + head + 3D + surround cams populate
   (see §11 for sensor-dropout troubleshooting if any pane stays blank).
3. `dimos run r1pro-keyboard-teleop` → press W, robot moves forward.

---

## Robot Architecture Notes

- **Platform**: Jetson Orin (aarch64), Ubuntu 22.04, L4T (Jetpack)
- **ROS2**: Humble, FastDDS (rmw_fastrtps_cpp)
- **ROS_DOMAIN_ID**: **1** on the new-gen robot (V2.3.0); the old-gen robot was 41
- **CAN bus**: arms and torso communicate via CAN (`~/canfd.sh` starts the
  driver — older docs called it `can.sh`)
- **HDAS**: Hardware abstraction layer — publishes all sensor feedback, receives
  all motion commands
- **mobiman**: Motion manager — handles kinematics, IK, safety limits. On
  stock V2.3.0 this is the unified `moca_adapter`; the `~/galaxea-dimos`
  tree swaps it for the standalone nodes (chassis, joint-PID, gripper,
  EE-pose) — see §14
- **Custom message package**: `hdas_msg` — used for motor control, BMS, LED,
  version info. Standard ROS2 types used for joint states and geometry
- **Chassis type**: W1 (3-wheel swerve drive), from `/opt/galaxea/body/hardware.json`

---

## Next Steps

- [x] Topic discovery and DDS connectivity over ethernet
- [x] Arm feedback reading
- [x] Arm joint movement
- [x] Chassis movement (via gatekeeper)
- [x] DiMOS adapters (chassis + arms) — original monolithic adapters
- [x] Sensor stream integration (wrist cameras, chassis cameras, LiDAR, IMUs)
- [x] Whole-body adapter
- [x] Sensor dropout under coordinator load — IP fragment buffer too small (resolved 2026-05-08)
- [x] **Architectural refactor: monolithic adapters → R1ProConnection Module + transport_lcm bridges (2026-05-09)**
- [x] **Coordinator blueprint registered as `r1pro-coordinator`**
- [x] **Keyboard teleop blueprint (chassis Twist) — `r1pro-keyboard-teleop`**
- [x] **Phase 1 hardware verification: connection boots, sensors flow, rerun renders all panes (2026-05-09)**
- [x] **Rerun bridge refactored to team-standard pattern — inline blueprint factory in coordinator, gated on `--viewer rerun` (§12.G)**
- [x] **`.envrc.r1pro` direnv template — opt-in `UV_PYTHON=3.10` without committing repo-wide `.python-version=3.10` (§12.H)**
- [x] **Test scripts restored from `task/mustafa/r1pro-dual-arm-testing` (test_01..06, run_all_tests.py, SENSOR_DROP_RUNBOOK.md)**
- [x] **Dual-tree robot setup (`~/galaxea` stock vs `~/dimos_galaxea`) — one-shot boot, reflash-proof (2026-06-13, §13)**
- [x] **New-gen robot (V2.3.0) chassis control — moca bypass via `~/galaxea-dimos` tree, verified driving smoothly (2026-07-09, §14)**
- [ ] Boot-test `~/galaxea-dimos` from a clean boot (recipe so far only exercised via the manual relaunch path)
- [ ] Re-verify arms/torso/grippers on the new gen's standalone nodes (chassis verified; the rest not re-checked since the moca bypass)
- [ ] Add lidar auto-start to the new-gen tree (`start_livox_lidar.sh` ships stock — one line in `hdas.yaml`)
- [ ] New-gen stream-drop elimination (T0–T5) — see [NEWGEN_DROP_ELIMINATION.md](NEWGEN_DROP_ELIMINATION.md)
- [ ] Manipulation blueprint (planner-coordinator) — researched, three paths identified, decision deferred (see §12.E)
- [ ] Push the URDF + meshes (`data/r1_pro_description/`) to git LFS so manipulation-blueprint paths can run on a fresh clone
- [ ] Torso planning approach — fold torso into bimanual planner (richer reachability) vs hold-fixed (simpler). Open question.
- [ ] Keyboard teleop felt "heavy" on hardware — `display_rate_hz` patch reverted, real root cause still TBD (see §12.D)
- [ ] Re-test `test_05_dimos_ros_layer.py` from the restored scripts — it referenced now-deleted monolithic adapters; needs updating to the new `R1ProConnection` imports or marking as historical
- [ ] Phase 2/3/4 hardware verification: ROS topic counts, motor_states / cmd_vel flow under load, brake-mode sanity

---

## Challenges & How We Solved Them

(Sections 1–11 are preserved from the dual-arm-testing branch — all still
factually valid for the robot side and the docker container setup. Section 12
is today's work.)

### 1. Finding the robot's IP
Robot had no known IP when connected via ethernet. Used `tcpdump` and `arp -a`
to discover it. Robot's `eth1` had no IPv4 assigned by default — manually
assigned `192.168.123.150/24` with `sudo ip addr add`, then made it persistent
via netplan.

### 2. ROS2 topic discovery failing across machines
**Root causes found (in order):**

**a) `ROS_LOCALHOST_ONLY=1` set in robot's `~/.bashrc`**
The robot was configured to only accept local DDS connections. Changed to
`ROS_LOCALHOST_ONLY=0` in `~/.bashrc` so tmux sessions (which source bashrc)
inherit the correct setting.

**b) CycloneDDS ↔ FastDDS EDP incompatibility**
Tried CycloneDDS on the laptop (ROS2 Jazzy default) thinking it would
interoperate with FastDDS on the robot (ROS2 Humble). Peer discovery (PDP)
worked — tcpdump confirmed packets flowing both ways — but endpoint discovery
(EDP) failed silently. Topics never appeared.

Fix: switch laptop to FastDDS (`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`) to match
the robot. The docker-container migration (§6) makes both sides Humble, so
this is no longer relevant for new sessions — left here as historical context.

**c) FastDDS using wrong network interface on laptop**
Laptop has WiFi, ethernet, and Tailscale interfaces. FastDDS multicast was
going out the wrong one. Fix: `fastdds_r1pro.xml` profile binding to the
ethernet interface and setting the robot's IP as an explicit unicast peer.
Also no longer needed in docker-container setup — `--network=host` plus
default multicast over the direct ethernet link works without XML.

**d) `interfaceWhiteList` renamed in FastDDS 3.x (Jazzy)**
The original XML used `<interfaceWhiteList>` (FastDDS 2.x) which Jazzy's
FastDDS 3.x silently ignored (renamed to `allowlist`). Switched to
locator-based config that works in both.

**e) Robot's FastDDS discovery server (port 11811)**
Initially thought `ROS_DISCOVERY_SERVER` was needed. Investigation showed
mobiman/hdas use multicast directly; the discovery server is for VR/WiFi
remote control only. Using `ROS_DISCOVERY_SERVER` broke topic visibility.

**f) HDAS process crashing (exit code -9)**
HDAS needs ~30 seconds to initialize on boot. If you check topics too early,
only chassis topics appear. The arm open/close cycle confirms hardware is
healthy — wait for it.

### 3. FastDDS 2.x/3.x DDS participant corruption
Running test scripts back-to-back with separate `rclpy.init()`/`rclpy.shutdown()`
cycles created new FastDDS 3.x participants each time. The
`ParticipantEntitiesInfo` wire format differs between FastDDS 2.x (Humble)
and 3.x (Jazzy), corrupting the robot's DDS state.

Fix: `run_all_tests.py` calls `rclpy.init()` once. Same docker-container
migration in §6 also retired this concern (everything is FastDDS 2.x now).

### 4. Chassis control node ignoring commands (the 3-gate problem)
See "Chassis Gatekeeper" section above. Took multiple sessions including
binary disassembly to identify the three gates.

### 5. ROS2 daemon unreliable on robot
The robot's ros2 daemon often shows only 2 topics (`/parameter_events`,
`/rosout`) even when 70+ are active. Always use `ros2 topic list --no-daemon`
on the robot.

### 6. Docker migration (Humble container on the laptop)

We moved the laptop-side DimOS runtime into a Docker container so the
environment matches the robot exactly (Ubuntu 22.04 + ROS2 Humble).

**a) Python 3.10 ↔ Python 3.12 mismatch with Humble's rclpy** — see §12.A,
which recapitulates this issue (it bit us *again* on this branch after a
rebase regenerated `uv.lock` with newer wheels).

**b) Docker dropped the DDS-XML unicast workaround** — Humble↔Humble plus
`--network=host` multicast works without XML. `FASTRTPS_DEFAULT_PROFILES_FILE`
removed from compose; XML kept as fallback.

**c) `ros2` daemon cache** — in the container, `ros2 topic list` returns
0–2 topics if it cached an empty discovery from earlier (e.g. while a bad
XML pinned bad locators). Run `ros2 daemon stop` and use `--no-daemon` after
any RMW change. `rclpy` in DimOS adapters is unaffected.

**d) Host-only kernel config the container can't apply itself** — DimOS's
system_configurator wants loopback multicast, a `224.0.0.0/4` route via `lo`,
and a higher `net.core.rmem_max`. The container can't modify host sysctls.
Apply once on the host:
```bash
sudo ip link set lo multicast on
sudo ip route add 224.0.0.0/4 dev lo   # ignore "exists" errors
sudo sysctl -w net.core.rmem_max=67108864
sudo sysctl -w net.core.rmem_default=67108864
```
Persist via `/etc/sysctl.d/60-r1pro-ros2.conf`.

**e) Python version pinning file (`.python-version`)** — historically pinned
to 3.12 (correct for old Jazzy host setup), broke the Humble container.
Updated to `3.10` on this branch (§12.A).

### 7. Rerun visualization with X11/Wayland forwarding from the container

(Detail preserved from the dual-arm branch — still valid.)

**Host one-time setup (run on the laptop):**

```bash
xhost +local:                                  # allow container windows
touch ~/.Xauthority                            # avoid Docker creating a dir
sudo ip link set lo multicast on
sudo ip route add 224.0.0.0/4 dev lo 2>/dev/null || true
sudo sysctl -w net.core.rmem_max=67108864
sudo sysctl -w net.core.rmem_default=67108864
```

**Inside the container — verify the display socket, GPU, and venv:**

```bash
echo "DISPLAY=$DISPLAY  WAYLAND_DISPLAY=$WAYLAND_DISPLAY  XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR"
ls -l /tmp/.X11-unix/                          # expect X0 (or X1) socket
ls -l /dev/dri/                                # expect renderD128 / card0

apt-get update && apt-get install -y x11-apps mesa-utils libvulkan1 vulkan-tools
xeyes &                                         # window should pop up
glxinfo -B | head -10                           # OpenGL renderer
vulkaninfo --summary 2>&1 | head -20            # Rerun's wgpu prefers Vulkan

cd /app
[ -d .venv ] || uv venv --python 3.10 .venv
source .venv/bin/activate
python -c "import rerun, rclpy; print('rerun', rerun.__version__, 'rclpy ok')"
```

If `xeyes` opens, X11 forwarding works.

**One-or-two-terminal workflow (all inside the same container):**

```bash
# Terminal 1 — coordinator + rerun viewer in one process
dimos --viewer rerun run r1pro-coordinator
# or for headless: dimos run r1pro-coordinator

# Terminal 2 (optional) — keyboard teleop / REPL / manipulation client
# Note: r1pro-keyboard-teleop already includes the coordinator
dimos --viewer rerun run r1pro-keyboard-teleop
```

**Troubleshooting** (preserved from the dual-arm branch):

| Symptom | Likely cause | Fix |
|---|---|---|
| `Authorization required, but no authorization protocol specified` | Host X server rejecting container UID | `xhost +local:` on host |
| `xeyes` works but Rerun blank/crashes | wgpu can't find a working GPU backend | Install `libvulkan1 mesa-utils`; for NVIDIA, add `runtime: nvidia` + `NVIDIA_DRIVER_CAPABILITIES=all` |
| Rerun opens but no panels populate | LCM bridge not receiving messages | Confirm `dimos run r1pro-coordinator` is in the same container; check connection logs |
| `ModuleNotFoundError: rerun` | Wrong venv (3.12) — see §12.A | `rm -rf .venv && uv venv --python 3.10 && uv sync` |
| Window appears but laggy/tearing under Wayland | Going through XWayland | Set `WAYLAND_DISPLAY=` (empty) to force pure X path |
| `~/.Xauthority` becomes a directory on host | Docker auto-created it | Stop container, `rmdir ~/.Xauthority`, `touch ~/.Xauthority`, restart |

### 8. Why `rr.spawn()` silently fails inside the dev container (historical)

VS Code extension hosts (`urdf-visualizer`, `rde-ros-2`, etc.) cache TCP
ports they've previously seen Rerun bind, and re-bind them on `127.0.0.1` on
subsequent VS Code launches. With `network_mode: host`, those collide with
any viewer subprocess.

**No longer relevant on this branch** — the standalone `run_rerun_bridge.py`
script that hit this issue was retired during the §12.G refactor. The
in-process `RerunBridgeModule` deployed via `dimos --viewer rerun run ...`
uses default ports `9877`/`9090` (not the `9876` VS Code typically caches).
If a port collision does happen, override via `--grpc-port` / `--web-port`
flags on `RerunBridgeModule.blueprint(...)`.

Kept here for the next person who runs into a phantom-port issue with
`rr.spawn()` in a similar setup.

### 9. Manipulation blueprint blocked on missing deps (Drake + trimesh)

(Old session log entry — still valid for when manipulation comes online.)

`dimos run r1pro-planner-full` (the dual-arm-branch blueprint) surfaced two
missing dependencies:

**a) Drake** — declared under `[project.optional-dependencies].manipulation`,
not base. Fix: `uv sync --python 3.10 --extra manipulation`.

**b) `uv pip install` bypassing the venv** — Dockerfile sets
`UV_SYSTEM_PYTHON=1`. Fix: `unset UV_SYSTEM_PYTHON` per session, or use
`uv pip install --python /app/.venv/bin/python <pkg>`.

**c) `trimesh` not declared anywhere → STL→OBJ conversion silent-skips →
Drake rejects URDF** — Drake 1.40+ dropped `.STL` for collision geometry.
DimOS's `mesh_utils.py` does STL→OBJ via trimesh but soft-fails when missing.
Fix: `pip install trimesh` + `rm -rf /tmp/dimos_urdf_cache`. To-do upstream:
add `trimesh` to the `manipulation` extra in `pyproject.toml`.

### 10. Sensor spin loop hot-looping on an invalidated rclpy context

Per-adapter sensor spin loops (in the OLD adapter files) wrapped
`spin_once()` in a blanket `try/except Exception: log; continue`. When
SIGINT or another module's shutdown invalidated the sensor's isolated
Context, every `spin_once()` raised "context is not valid" and the recovery
loop spun forever at wall-clock speed.

Fix carried into the new `R1ProConnection._sensor_spin` at
[connection.py](../../dimos/robot/galaxea/r1pro/connection.py):

```python
while not self._sensor_stop.is_set() and ctx.ok():
    try:
        executor.spin_once(timeout_sec=0.1)
    except Exception as exc:
        if not ctx.ok() or "context is not valid" in str(exc):
            logger.warning(f"Sensor context invalid, exiting spin: {exc}")
            break
        logger.warning(f"sensor spin_once raised (continuing): {exc}", exc_info=True)
```

Clean shutdowns now produce one exit line per sensor thread instead of a
flood. Critical for the new architecture too — same pattern carried over.

### 11. Chassis sensors missing from Rerun tree → IP fragment buffer

**Symptom** — wrist + head cameras render in Rerun, but
`world/r1pro/chassis/lidar`, `chassis_front_left/right`, `chassis_left/right`,
`chassis_rear`, `head_depth` entries are missing or stop after a few seconds.

**Root cause** — Linux `net.ipv4.ipfrag_high_thresh` defaults to 4 MB.
6 chassis cameras × ~100 KB JPEGs × ~30 Hz = ~18 MB/s of fragmented UDP,
plus PointCloud2 lidar. Reassembly pool fills in <300 ms; partially-assembled
datagrams evict each other; kernel drops them silently. Small messages (IMU)
pass because they fit in a single packet. This is *separate* from
`net.core.rmem_max`.

**Fix — apply on the laptop host** (network namespace inheritance via
`network_mode: host`):

```bash
sudo sysctl -w net.ipv4.ipfrag_high_thresh=67108864
sudo sysctl -w net.ipv4.ipfrag_low_thresh=50331648
sudo sysctl -w net.ipv4.ipfrag_time=60
```

Persist via `/etc/sysctl.d/60-r1pro-ros2.conf`:
```
net.core.rmem_max = 67108864
net.core.rmem_default = 67108864
net.ipv4.ipfrag_high_thresh = 67108864
net.ipv4.ipfrag_low_thresh = 50331648
net.ipv4.ipfrag_time = 60
```

Diagnostic that pinned it (in case it recurs):

```bash
nstat -az | grep -iE 'Reasm|FragOK|FragFail'
```
`IpReasmFails` climbing while `IpReasmOKs` flat ⇒ same problem, same fix.

---

### 12. Architectural refactor + bring-up (2026-05-09 — this branch)

The current branch (`mustafa/task/r1pro-coordinator-integration`) replaces
the monolithic per-segment adapters with one connection Module + generic
`transport_lcm` adapters, mirroring Go2/G1. New code under
`dimos/robot/galaxea/r1pro/`. Old code (per §12 of the architecture
section) deleted. Several issues surfaced during bring-up:

**A. Python 3.10 ↔ 3.12 mismatch (round 2: rebase regenerated uv.lock)**

After rebasing this branch onto a more recent `dev`, `uv sync` silently
provisioned a Python 3.12 venv (project's `.python-version` was `3.12` at the
time, and `requires-python = ">=3.10"` permits both). `dimos run
r1pro-coordinator` then crashed at `R1ProConnection.start()`:

```
ImportError: rclpy is not installed. ROS pubsub requires ROS 2.
```

The path in the trace was `cpython-3.12.13-linux-x86_64-gnu`. Humble's
rclpy is built for **Python 3.10** (Ubuntu 22.04 system Python) — its
compiled `.so` files won't load under 3.12.

**Fix — `.envrc.r1pro` direnv template** (the standing solution on this
branch — matches the existing `.envrc.nix` / `.envrc.venv` pattern):

```bash
# One-time setup for any r1pro dev:
ln -sf .envrc.r1pro .envrc
direnv allow

# Then the usual:
rm -rf .venv
uv sync                                        # picks UV_PYTHON=3.10 from .envrc.r1pro
source .venv/bin/activate
source /opt/ros/humble/setup.bash
python -c "import rclpy; print(rclpy.__file__)"
# /opt/ros/humble/lib/python3.10/site-packages/rclpy/__init__.py
```

The `.envrc.r1pro` template exports `UV_PYTHON=3.10` and activates the
local venv. Symlinking it to `.envrc` opts you into the pin
**per-checkout**, only inside this repo, only for devs who opt in.
`.python-version` stays at `3.12` (the project default — unchanged), so
non-r1pro work elsewhere in dimos picks the highest Python as before.

**Why a direnv template instead of committing `.python-version=3.10`**:
the repo-wide `.python-version` would force 3.10 on every dimos dev/CI,
including Jazzy / Python 3.12 work that has nothing to do with r1pro.
Direnv's per-directory activation lets only opted-in r1pro devs pick up
the override.

**One-shot fallback** if you can't use direnv:

```bash
uv sync --python 3.10 --all-extras --no-extra dds --no-extra unitree-dds
```

Every time. `.python-version` left untouched.

**Why it kept happening before the template**: `uv sync` honours the
project's `.python-version` over the `--python` flag passed to `uv venv`.
Without setting `UV_PYTHON=3.10` (via direnv) or passing `--python 3.10`
to `uv sync` itself, every subsequent sync regenerates the venv at the
highest installed Python (3.12 in our uv install).

**B. `onnxruntime` 1.24.1 dropped cp310 wheels**

After the Python pin, `uv sync --all-extras ...` failed with:

```
error: Distribution `onnxruntime==1.24.1` ... only has wheels with the
following Python implementation tags: cp311, cp312, cp313, cp314
```

PyPI released `onnxruntime` and `onnxruntime-gpu` 1.24.1 on **2026-02-05**
and dropped Python 3.10 wheel builds. The project's `cpu` extra in
`pyproject.toml` had `"onnxruntime"` (no upper bound), so a fresh
`uv lock` after that date resolves to 1.24.1 — incompatible with Humble.

The previous lock had 1.23.2 (released 2025-10-22) which still ships
cp310 wheels. The `db05bf0cb` commit ("Create DDS Transport Protocol
(#1174)", 2026-02-13) regenerated `uv.lock` and was the moment the bump
landed on this branch's lineage.

**Fix — `pyproject.toml`** (committed on this branch):

```toml
cpu = [
    # CPU inference backends.
    # Upper bound: onnxruntime 1.24+ dropped cp310 wheels. Lift when ROS 2
    # Humble (Python 3.10) is no longer supported.
    "onnxruntime<1.24",
    "ctransformers==0.2.27",
]

cuda = [
    "cupy-cuda12x==13.6.0; platform_machine == 'x86_64'",
    "nvidia-nvimgcodec-cu12[all]; platform_machine == 'x86_64'",
    # Upper bound: onnxruntime-gpu 1.24+ dropped cp310 wheels. Lift when ROS 2
    # Humble (Python 3.10) is no longer supported. Lower bound covers both
    # cuda11 and cuda12.
    "onnxruntime-gpu>=1.17.1,<1.24; platform_machine == 'x86_64'",
    "ctransformers[cuda]==0.2.27",
    "xformers>=0.0.20; platform_machine == 'x86_64'",
]
```

Then: `uv lock --python 3.10 && uv sync --python 3.10 --all-extras --no-extra dds --no-extra unitree-dds`.
Sanity check: `grep -A1 '^name = "onnxruntime"$' uv.lock | head -2`
should show `version = "1.23.x"`. **Lift these bounds when the project
moves off Humble (Jazzy = Python 3.12).**

**C. Keyboard teleop publishing into the void — missing `cmd_vel ↔ twist_command` LCM bridge**

After the venv was working, `r1pro-keyboard-teleop` booted clean but
keypresses never moved the chassis. Investigation:

- `KeyboardTeleop` declares `cmd_vel: Out[Twist]`.
- `ControlCoordinator` declares `twist_command: In[Twist]`.
- They're different stream names. `autoconnect` doesn't auto-bridge them
  in-process; it relies on **shared LCM topics** to route across names.

The Go2 coordinator had this transport pair:

```python
("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
```

— both pointing at `/cmd_vel`. KeyboardTeleop publishes to the topic on
its `cmd_vel` Out side; ControlCoordinator subscribes on its
`twist_command` In side. Same topic, different stream names, both bridged.

Our `r1pro_coordinator` was missing those entries, so the coordinator's
`twist_command` defaulted to `/twist_command` and KeyboardTeleop's
`cmd_vel` had no transport at all. No wires.

**Fix** — added two transports to `r1pro_coordinator.py`:

```python
# Public Twist bus on /cmd_vel — `cmd_vel` covers any module's Out
# (KeyboardTeleop, phone teleop, etc.); `twist_command` is the
# ControlCoordinator's matching In. Both pinned to the same LCM
# topic so any Twist publisher drives the chassis.
("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
```

Living on the **base coordinator** (not the keyboard-teleop blueprint
specifically) means any future Twist publisher (phone teleop, joystick,
autonomous policy) drives the chassis without a new blueprint.

**D. Keyboard teleop felt heavy on the dev container (open question)**

After fix C, teleop "barely sent any commands". First hypothesis was
`pygame.display.flip()` stalling on X11 round-trips (a known issue for
remote / docker / X11-forwarded sessions). A patch was prototyped adding
a `display_rate_hz` knob to `KeyboardTeleop` that decoupled display
refresh from the publish loop, with the r1pro blueprint passing
`display_rate_hz=10.0`.

**That patch was reverted** — it didn't fix the actual lag on hardware,
and adding a knob to a shared module to compensate for a problem that
turned out to live somewhere else wasn't worth the cross-robot surface
area. `KeyboardTeleop` is back to its upstream defaults; the r1pro
blueprint uses `KeyboardTeleop.blueprint()` with no overrides.

**Real root cause is still TBD.** End-of-session note from the user:
"the r1 connection is just so much heavy". Likely candidates to
investigate next session:

- LCM RPC hops across worker boundaries (KeyboardTeleop deploys on one
  worker, ControlCoordinator on another, R1ProConnection on a third —
  every Twist makes ~3 hops).
- The 100 Hz coordinator tick + `_on_cmd_vel` lock contention inside
  `R1ProConnection` (chassis publishes acc + brake + speed under
  `self._lock`, shared with motor_command, publish_loop, and 3 feedback
  callbacks).
- `pygame.event.get()` itself stalling under X11 (independent of `flip()`).

To-do when picking up: re-test `r1pro-keyboard-teleop` on hardware,
measure where time is actually going (cProfile on the KeyboardTeleop
worker, or just `lcm-spy` the rate of `/cmd_vel` to see whether the
keyboard side is slow at all). Don't re-introduce the display_rate_hz
patch without that measurement.

**E. Manipulation blueprint — researched, not yet built**

Goal: dual-arm planner-coordinator that lets the planner generate joint
trajectories for the two arms and execute them through the existing
whole-body adapter. Researched in detail; three viable paths identified,
none yet implemented:

| Path | Approach | Status |
|---|---|---|
| **A** | Two per-arm `HardwareComponent`s (mock adapters), one trajectory task per arm | Cleanest mirror of OpenArm dual-arm pattern. Doesn't exercise the new whole-body adapter. |
| **B** | Override `joint_prefix="r1pro/"` on both `r1pro_arm()` RobotConfigs so coordinator joint names match the WHOLE_BODY adapter directly | Risks Drake model-name collision when both arms load the same URDF. Needs verification. |
| **C** | Single 18-DOF `r1pro_upper_body()` RobotConfig (torso + both arms) | Drake's Cartesian IK works on a single end-effector — bimanual targeting needs custom planner, more work. |

The critical wiring detail: `ManipulationModule` plans against **bare URDF
joint names** (e.g. `left_arm_joint1`); the whole-body adapter expects
**prefixed coordinator names** (`r1pro/left_arm_joint1`). The existing
translation in `_translate_trajectory_to_coordinator()` uses
`RobotConfig.joint_prefix` — Path B forces this mapping to align with the
whole-body adapter's namespace.

**Decision deferred**. When picking up: read the manipulation research
notes from this session (transcript) before re-running the analysis.

**F. URDF in LFS**

The R1Pro URDF + meshes live at `data/r1_pro_description/` on the user's
laptop but are **not in git LFS yet**. `LfsPath` short-circuits when the
directory exists locally (`get_data` returns immediately if `file_path.
exists()` at [dimos/utils/data.py:230](../../dimos/utils/data.py)), so
manipulation works locally but breaks on a fresh clone.

**To-do** when manipulation is implemented:
```bash
./bin/lfs_push                              # tars data/r1_pro_description/ → data/.lfs/
git add data/.lfs/r1_pro_description.tar.gz
git commit -m "lfs: add r1_pro_description (urdf + meshes)"
```

**G. Rerun bridge refactored to the team-standard pattern**

The original v1 of this branch shipped a standalone bridge launcher
(`scripts/r1pro_test/run_rerun_bridge.py`) plus a separate layout file
(`dimos/robot/galaxea/r1pro/rerun.py`). Worked, but diverged from how
Go2 and G1 do it. 

### 13. V2.2.1 firmware reflash silently reverted everything (2026-06-13)

**Symptom** — after a firmware reflash the chassis wouldn't move even with the
gatekeeper running; arms were fine.

**Root cause 1 — robot-local DDS discovery broken.** `lo` was missing its
MULTICAST flag (boot-dependent). FastDDS announcements went out every
interface but were never delivered locally; only ~4 nodes showed in
`ros2 node list --no-daemon` (unicast fallback covers participant indices 0–3)
while dozens ran, so the gatekeeper couldn't see `chassis_control_node`. Arms
still worked because laptop↔robot discovery over eth1 is unaffected. Telltale:
`ROS_LOCALHOST_ONLY=1` pub/echo works, `=0` doesn't.
**Fix:** `sudo ip link set lo multicast on` (non-persistent; now applied by
`start_chassis_gatekeeper.sh` on every Dimensional boot).

**Root cause 2 — the reflash wiped all our hooks.** Gone: the `/controller`
remap (so HDAS's 200 Hz mode=2 re-locked Gate 2), the entire `~/galaxea/src`
tree (launch files now live only in the install tree — editable directly, no
rebuild), the lidar/chassis-cam session entries, and
`start_chassis_cameras_v4l2.sh`. V2.2.1 also adds a `vr_controller` node
publishing `/motion_target/target_speed_chassis` (a competing chassis
commander) and a `/motion_target/brake_mode` (Bool) subscription on the
chassis node — nothing publishes it, default permits motion; it's the 4th gate
to check if a future firmware flips it.

**Resolution — the dual-tree setup** (see Robot Startup Procedure). Vendor tree
stays byte-for-byte stock, all Dimensional changes live in `~/dimos_galaxea`,
so reflashes can never silently revert us again. To recover after a reflash:
re-rsync `~/galaxea` → `~/dimos_galaxea` and re-apply the 4-file diff.
(An intermediate runtime takeover/restore approach lives at
`~/dimos_ws/src/dimos_r1pro_bringup` — superseded but still functional.)

Still laptop-side (unchanged from §11): if depth/chassis cams render on the
robot but not in rerun, it's the laptop's IP-fragmentation sysctls. Head depth
will NEVER render — no ZED on this robot, the topic has no producer.

### 14. New-gen robot migration — V2.3.0 moca bypass (2026-07-09)

**Situation** — moved from the old-gen robot to the **new-gen R1 Pro**:
firmware **V2.3.0** (old was V2.2.1), `ROS_DOMAIN_ID=1` (old was 41), CAN
via `canfd.sh`, base has **lidars only** (no surround chassis cams).
V2.3.0 replaced the standalone mobiman control nodes with a
new unified **`moca_adapter`** — that architectural change is what broke
the old setup: none of our old hooks existed and the control path moved.

**Why the chassis wouldn't move from the laptop (two stacked blockers on
stock):**

1. **RC teleop competes** — `homogeneous_teleoperation` co-publishes zeros
   onto `/motion_target/target_speed_chassis` at 200 Hz, out-shouting
   external commands.
2. **moca refuses external chassis commands** — with teleop killed, moca
   still output zero to the wheels while happily driving the arms. It's a
   chassis-specific refusal inside moca we couldn't unlock (not brake, not
   acc, not QoS, not RC mode).

**Decision — bypass moca.** It's new in V2.3.0 and isn't what
`R1ProConnection` was built for. The old robot had no moca — we drove the
standalone `chassis_control_node` directly, and that node still ships in
V2.3.0. So we went back to it.

**The fix (verified driving smoothly):** on `chassis_control_node`, Gate 2
(see "Chassis Gatekeeper" above) is real on this firmware too — HDAS floods
`/controller` with **mode 3** (V2.2.1 used mode 2); it needs mode 5 to
unlock, and fighting HDAS directly is jerky. So:

- Remap `/controller` → `/controller_unused` on the chassis node → HDAS
  can't reach it.
- Gatekeeper holds `mode=5` on `/controller_unused` (uncontested) +
  `brake_mode=False` + a nonzero `acc_limit`.
- Then a plain `TwistStamped` on `/motion_target/target_speed_chassis`
  drives smoothly.

**What was built — `~/galaxea-dimos/`** (byte-for-byte copy of stock +
these changes only; `~/galaxea` stays 100% stock so reflashes can't
silently revert us — same insurance as §13):

- `mobiman.yaml`: moca → standalone stack (chassis, joint-PID, gripper,
  EE-pose) + gatekeeper pane
- chassis launch: `/controller` → `/controller_unused` remap
- new `chassis_gatekeeper.py` + `start_chassis_gatekeeper.sh`
- `teleop.yaml` → `.disabled` (no RC competitor)

**Laptop side:** point `R1ProConnection` at
`/motion_target/target_speed_chassis` as before — **no gate handling needed
on the laptop anymore**; the on-robot gatekeeper owns it. The only
laptop-side change is `export ROS_DOMAIN_ID=1`.

**Still open** (mirrored in Next Steps): clean-boot test of
`~/galaxea-dimos`; arms/torso/grippers not re-verified on the standalone
nodes; lidar auto-start (`start_livox_lidar.sh`, one line in `hdas.yaml`).

### 15. On-robot docker deployment + stream-rate collapse diagnosis (2026-07-10)

The one-stop on-robot docker image ([docker/](docker/)) was brought up
end-to-end on the new gen: build → run → `R1ProConnection started` → all
transports wired → rerun bridge serving gRPC on 9877. Every trap hit on the
way (build DNS, `ROS_DOMAIN_ID`, **LCM port 7667 colliding with FastDDS
domain-1 participant ports**, exec-shells not sourcing ROS, matplotlib
missing, host rmem sysctls unreachable from the container) is documented in
[docker/README.md](docker/README.md) "Known traps".

**Then: camera streams at 0.2–0.4 Hz in the viewer.** Diagnosis found THREE
stacked causes — measured, not guessed (raw-socket LCM channel-rate parser +
rclpy topic-rate probe):

1. **Wrist D405s failing at the hardware level.** ROS-level rates 0.0–0.3 Hz
   at the source; realsense logs flooding `Frames didn't arrived within 5
   seconds — Frames Timeout` on both wrists (USB3 enumeration fine at 5000M).
   The known USB fault class — reseat cables, rerun
   `start_realsense_camera_r1pro.sh`. No software fix applies.
2. **Triple JPEG codec on-robot (CPU).** Per color frame: connection
   `cv2.imdecode` → `JpegLcmTransport` turbojpeg re-encode → bridge turbojpeg
   decode → rerun logs RAW pixels. Head cams publish **30 Hz** on the new gen
   (~7 MB/s jpeg each); the connection sustained ~7 Hz, `maxsize=1` queues
   silently dropping the rest, load average 22 on 8 cores. Fixed the free
   part: `TurboJPEG()` was constructed **per message** on both encode and
   decode — now a thread-local cached instance
   ([Image.py](../../dimos/msgs/sensor_msgs/Image.py)). The structural fix is
   the Tier-2 JPEG passthrough (original camera bytes end-to-end,
   [NEWGEN_DROP_ELIMINATION.md](NEWGEN_DROP_ELIMINATION.md)) — NOT yet
   implemented; until it lands, head streams top out well below 30 Hz.
3. **rerun's gRPC stream is a reliable ordered log — it never drops frames.**
   The bridge logged raw `rr.Image` (~2.7 MB/frame ≈ >100 MB/s demand) into a
   ~5 MB/s WiFi viewer link, so the viewer lagged further and further behind
   (wlan0 TX counter: 246 GB). Fixed in
   [r1pro_coordinator.py](../../dimos/robot/galaxea/r1pro/blueprints/basic/r1pro_coordinator.py):
   `visual_override` JPEG-compresses color frames (q75, ~30–60× smaller)
   before logging, suppresses depth streams (largest raw payloads,
   viz-only), caps lidar at 5 Hz (was uncapped raw Points3D), color capped
   at 5 Hz until Tier-2.

**Also: hard reboot corrupted the git object DB.** 18 recently-written loose
objects truncated to 0 bytes (classic ext4 power-cut damage); one broke
`refs/remotes/origin/jeff/feat/dan_loc`. Recovery (worked cleanly): back up
uncommitted work (`git diff > backup.patch`), `find .git/objects -type f
-empty -delete`, `git update-ref -d <broken remote-tracking ref>`, `git
fetch origin` to restore. Local branches and the working tree were never
damaged. Can recur on any hard power-off with recent git activity.

---

## Session Log — Sensor Streams & Dual-Arm Coordinator Integration

(Preserved from the dual-arm-testing branch. Most details still apply
architecturally — the topic name prefixes have changed in the new
connection-module architecture, but the data flow and the kernel-level
fixes are identical.)

### What was built

**Sensor streams on adapters** (now consolidated into `R1ProConnection` —
see §12). All sensors physically attached to the robot are now subscribed
by a single Module's isolated DDS participant and decoded in dedicated
worker threads.

| Old per-adapter LCM topic | New connection-module LCM topic |
|---|---|
| `/r1pro/left_arm/wrist_color`, `/r1pro/left_arm/wrist_depth` | `/r1pro/wrist_left_color`, `/r1pro/wrist_left_depth` |
| `/r1pro/right_arm/wrist_color`, `/r1pro/right_arm/wrist_depth` | `/r1pro/wrist_right_color`, `/r1pro/wrist_right_depth` |
| `/r1pro/chassis/head` | `/r1pro/head_color` |
| `/r1pro/chassis/head_depth` | `/r1pro/head_depth` |
| `/r1pro/chassis/chassis_front_left` … | `/r1pro/chassis_front_left` … |
| `/r1pro/chassis/lidar` | `/r1pro/lidar` |
| `/r1pro/chassis/imu_chassis` | `/r1pro/imu` |
| `/r1pro/chassis/imu_torso` | `/r1pro/imu_torso` |

**Async worker pattern** (preserved verbatim in the new module):

1. ROS spin thread callback → enqueue raw `msg` (zero-copy)
2. Dedicated worker thread per sensor → `bytes(msg.data)` + decode + Out stream publish
3. All queues are `maxsize=1` (latest-frame semantics — stale frames replaced)

**Separate rclpy context for sensor subscriptions** — `R1ProConnection`'s
`_setup_sensor_streams()` creates a separate `rclpy.Context()` with its own
`MultiThreadedExecutor` and DDS participant. Same rationale as before:
prevents control traffic from saturating sensor DDS receive threads.

### The sensor dropout problem (resolved 2026-05-08)

(See §11 — IP fragment reassembly buffer too small. Same fix applies
on the new architecture; nothing changed at the kernel-buffer level.)
