# Galaxea R1 Pro

18-DOF upper body (torso 4 + arm 7 + arm 7) over ROS 2 / FastDDS, holonomic
chassis, head stereo + wrist cameras, chassis lidar, 2 IMUs.

## Robot-side setup

Everything in this section runs **on the robot's onboard computer**, over ssh,
not on your workstation. The paths below are the robot's, and are the same on
every R1 Pro.

`galaxea-dimos` is Galaxea's own ROS 2 driver with local bug fixes applied; it
is not part of dimos. `R1ProConnection` boots it, when it is not running, with
[`vendor_setup.sh`](/dimos/robot/galaxea/r1pro/vendor_setup.sh): the standalone stack
(no `moca_adapter`, chassis gatekeeper on-robot) without the vendor head camera and
Livox drivers. HDAS brings up CAN FD itself, so `~/canfd.sh` is not needed. Pick a
vendor profile instead with `--r1proconnection.vendor-profile ATCStandard/R1PROVRTeleop`.

## Environment

- `ROS_DOMAIN_ID=1` (new-gen V2.3.0), `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.
- ROS 2 Humble ships `rclpy` built for CPython 3.10, so the venv must use the
  robot's system interpreter. `.python-version` says 3.12, so pass `--python`
  explicitly rather than relying on direnv:

  ```bash
  sudo apt-get install -y libturbojpeg   # pyturbojpeg needs the native lib
  uv sync --python /usr/bin/python3.10 --python-preference only-system \
          --no-default-groups --extra base --extra manipulation --extra cpu
  uv pip install python-socketio         # runtime dep of the viewer, only declared in the lint group
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
dimos run r1pro-planar-preview   # planar-base planning preview with fake hardware
```
