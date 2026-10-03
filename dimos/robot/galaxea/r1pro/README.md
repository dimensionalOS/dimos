# Galaxea R1 Pro

18-DOF upper body (torso 4 + arm 7 + arm 7) over ROS 2 / FastDDS, holonomic
chassis, head stereo + wrist cameras, chassis lidar, 2 IMUs.

## Robot-side setup

Everything in this section runs **on the robot's onboard computer**, over ssh,
not on your workstation. The paths below are the robot's, and are the same on
every R1 Pro.

The robot runs `galaxea-dimos`: Galaxea's own ROS 2 driver (firmware V2.3.0)
with the changes dimos needs. It is not part of dimos. It is stored in dimos
cloud as one tarball, `galaxea-dimos-v2.3.0-dimos.1.tar.zst`, upload id
`56a9468a113444c5afa792a9c7877cd1`. Install and start it:

```bash
dimos login   # once per robot
dimos data pull 56a9468a113444c5afa792a9c7877cd1 --dest ~/galaxea-dimos.tar.zst
tar -I zstd -xf ~/galaxea-dimos.tar.zst -C ~
~/galaxea-dimos/start.sh
```

The tree must end up at `~/galaxea-dimos`. `start.sh` runs `~/canfd.sh`, which
ships with the robot and brings up the CAN FD interfaces, then restarts the
driver and waits until it publishes. Restarting ends every tmux session on the
robot, so the script lists them and stops unless you pass `--yes`.
`~/galaxea-dimos/README-dimos.md` lists every change from stock and why, and
`dimos-changes.diff` holds the exact diff.

To restart the driver by hand:

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
  uv pip install python-socketio         # runtime dep of the viewer, only declared in the lint group
  ```

  `--all-extras` does not work on the robot's arm64 board: the `scene` extra
  needs `usd-core` (x86_64/macOS wheels only) and `mapping` needs
  `gtsam-extended` (arm64 Linux wheels start at cp311). `mapping` is also
  reachable through `unitree` and `all`, so excluding it by name is not enough.
- The planning model fetches the vendor URDF from the pinned upstream repo.

## Wrist cameras

dimos reads the two wrist D405s straight off V4L2, not through Galaxea's
RealSense driver. Both cameras share one USB 3.0 controller (a Renesas
uPD720201 PCIe card), and with colour and depth on both wrists at 1280x720@30
that controller stalls every stream. That is what stalls Galaxea's wrist
driver too.

**Stop Galaxea's wrist driver first.** It starts at boot, holds the cameras,
and while it runs the dimos wrist modules only log `cannot open` and retry. It
runs in the `hdas` tmux session, in the pane running
`start_realsense_camera_r1pro.sh`. Press Ctrl-C there, or:

```bash
tmux send-keys -t hdas:1.3 C-c   # pane index as found on our robot; check with tmux list-panes -a
```

What the blueprints open:

| Blueprint | Per wrist | Topics |
| --- | --- | --- |
| `r1pro-coordinator`, `-teleop`, `-nav` | colour 848x480 @ 30 | `wrist_{left,right}_color` |
| `r1pro-manipulation` | colour 640x480 + depth 848x480 @ 30 | `+ wrist_{left,right}_depth` |

Depth is `DEPTH16` in millimetres, in the depth imager's own pixel grid (not
aligned to colour). Both streams use the frame id `wrist_{left,right}_optical`.

### Full resolution at 15 fps

With the stock `uvcvideo` driver, 640x480 colour + 848x480 depth at 30 fps is
the most that fits. The driver's `FIX_BANDWIDTH` quirk (`0x80`) makes each
stream reserve USB bandwidth for its actual frame rate. With it, both wrists
run colour and depth at 1280x720 @ 15 fps. It does not help at 30 fps.

Load the quirk now (no dimos run may hold the cameras, and Galaxea's wrist
driver must be stopped):

```bash
sudo modprobe -r uvcvideo && sudo modprobe uvcvideo quirks=0x80
cat /sys/module/uvcvideo/parameters/quirks   # 128
```

Keep it across reboots:

```bash
echo "options uvcvideo quirks=0x80" | sudo tee /etc/modprobe.d/uvcvideo-r1pro.conf
```

The two wrist D405s are the only devices on `uvcvideo`, so nothing else is
affected. Then select the mode at run time:

```bash
dimos run r1pro-manipulation \
  --wristleftcolordepth.color-width 1280 --wristleftcolordepth.color-height 720 \
  --wristleftcolordepth.depth-width 1280 --wristleftcolordepth.depth-height 720 \
  --wristleftcolordepth.fps 15 \
  --wristrightcolordepth.color-width 1280 --wristrightcolordepth.color-height 720 \
  --wristrightcolordepth.depth-width 1280 --wristrightcolordepth.depth-height 720 \
  --wristrightcolordepth.fps 15
```

## Blueprints

```bash
dimos run r1pro-coordinator     # connection + coordinator + viewer
dimos run r1pro-teleop          # + chassis teleop from the viewer
dimos run r1pro-nav             # + click-to-drive nav (costmap + A*)
dimos run r1pro-manipulation    # + dual-arm planning (experimental)
dimos run r1pro-planar-preview   # planar-base planning preview with fake hardware
```
