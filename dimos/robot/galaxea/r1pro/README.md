# Galaxea R1 Pro

18-DOF upper body (torso 4 + arm 7 + arm 7) over ROS 2 / FastDDS, holonomic
chassis, head stereo + wrist cameras, chassis lidar, 2 IMUs.

## Robot-side setup

Everything in this section runs **on the robot's onboard computer**, over ssh,
not on your workstation. The paths below are the robot's, and are the same on
every R1 Pro.

`galaxea-dimos` is Galaxea's own ROS 2 driver with local bug fixes applied; it
is not part of dimos. `canfd.sh` ships with the robot and brings up the CAN FD
interfaces the driver needs. Boot the driver with the standalone stack, which
bypasses the stock `moca_adapter` and runs the chassis gatekeeper on-robot:

```bash
bash ~/canfd.sh
cd ~/galaxea-dimos/install/startup_config/share/startup_config/script
./robot_startup.sh kill
./robot_startup.sh boot ../sessions.d/ATCStandard/R1PROBody.d/
```

## Environment

- `ROS_DOMAIN_ID=1` (new-gen V2.3.0), `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.
- ROS 2 Humble ships `rclpy` built for CPython 3.10, so the venv must use the
  robot's system interpreter. `.python-version` says 3.12, so pass `--python`
  explicitly rather than relying on direnv:

  ```bash
  sudo apt-get install -y libturbojpeg   # pyturbojpeg needs the native lib
  uv sync --python /usr/bin/python3.10 --python-preference only-system \
          --no-default-groups --extra base --extra manipulation --extra cpu
  ```

  `--all-extras` does not work on the robot's arm64 board: the `scene` extra
  needs `usd-core` (x86_64/macOS wheels only) and `mapping` needs
  `gtsam-extended` (arm64 Linux wheels start at cp311). `mapping` is also
  reachable through `unitree` and `all`, so excluding it by name is not enough.
- The planning model fetches the vendor URDF from the pinned upstream repo.

## Blueprints

```bash
dimos run r1pro-coordinator     # connection + coordinator + viewer
dimos run r1pro-teleop          # + chassis teleop from the viewer
dimos run r1pro-nav             # + click-to-drive nav (costmap + A*)
dimos run r1pro-manipulation    # + dual-arm planning (experimental)
dimos run r1pro-planner-coordinator  # planar-base planning with fake hardware
```

## Hosted teleoperation

`r1pro-hosted-teleop-quest` and `r1pro-hosted-teleop-pico` dial out to the
dimensional-teleop broker; the operator drives from a WebXR headset at
[teleop.dimensionalos.com](https://teleop.dimensionalos.com) with no inbound
ports on the robot. Both names run the same stack — Quest and PICO deliver the
same WebXR poses and Joy — and exist so each headset has its own entry point.

```bash
TRANSPORTS__BROKER__API_KEY=dtk_live_... \
TRANSPORTS__BROKER__ROBOT_NAME=r1pro \
dimos run r1pro-hosted-teleop-quest
```

| Operator input | Effect |
|---|---|
| Side grip (hold) | Engage. One deadman for the whole robot: arms track, base drives, torso jogs. Release and everything stops. |
| Left stick | Drive the chassis: forward, back, strafe (grips held) |
| Right stick X | Yaw the chassis (grips held) |
| Right stick Y | Jog the torso, with the right stick clicked in |
| Trigger | Gripper opening on that hand, analog |
| A (hold) | Walk both arms back to the tray pose |

The broker transport needs the `webrtc` extra, which the `uv sync` line above
does not install:

```bash
uv sync --python /usr/bin/python3.10 --python-preference only-system \
        --no-default-groups --extra base --extra manipulation --extra cpu \
        --extra webrtc
```

Known gaps:

- `R1ProConnection` has no gripper port, so trigger commands reach the
  coordinator and stop there. Grasping needs the vendor gripper wired first.
- The torso jog rides the teleoperation IK task's head target, so it only
  moves while both arms are engaged. Base driving is always live.
- The operator console renders the `arm` view: arms, video, and E-STOP have
  controls, base and torso do not. They work from the sticks regardless.
- Head-left and right-wrist colour feed the two video mux inputs. The R1 Pro
  publishes them compressed, so each is decoded on the robot first.
