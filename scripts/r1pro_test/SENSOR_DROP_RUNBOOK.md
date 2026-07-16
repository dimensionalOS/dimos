# R1 Pro Sensor Drop Diagnostic Runbook (adapted to current implementation)

> **⚠️ 2026-07-03 — READ FIRST:** the root cause of the recurring stream
> collapses was found and it is **upstream of everything in this runbook**:
> the robot's CSI/VI camera-capture subsystem melts down when multiple
> chassis cameras stream simultaneously (capture timeout→reset loop + kernel
> SMMU/IRQ fault storm that kills all streams and crashes driver nodes).
> Before running any procedure here, check `dmesg -Tw` on the robot for
> `tegra-camrtc-capture-vi: uncorr_err` / `arm-smmu` faults — if present,
> this runbook's network/DDS phases will only measure symptoms. See
> [STREAM_RELIABILITY_DIAGNOSTIC.md](STREAM_RELIABILITY_DIAGNOSTIC.md)
> (CURRENT UNDERSTANDING header + status log) and use
> [stream_dashboard.py](stream_dashboard.py) for live per-topic flow.
> Known-healthy interim config: head + wrist colors + lidar; skip chassis
> surround cams pending Galaxea/TZTEK escalation.

**Purpose:** Find the layer at which camera/LiDAR frames are being dropped when the ControlCoordinator tick loop starts, apply the cheapest fix that resolves it, and capture enough data that if the first round doesn't resolve it the next session has everything it needs to decide on architectural changes.

**What this runbook assumes about the code** (verified against the current repo; if any of this is wrong, stop and re-read the adapter):

- Both `R1ProArmAdapter` and `R1ProChassisAdapter` create a **separate `rclpy.Context()`** for sensor subscriptions, with `MultiThreadedExecutor(num_threads=2)` for the arm and `num_threads=4` for the chassis.
- Sensor callbacks **enqueue the raw `msg` object** (zero-copy) into a `queue.Queue(maxsize=1)`. Decode (`bytes(msg.data)` + `cv2.imdecode` or `ros_to_dimos()`) happens in a dedicated worker thread per sensor.
- **No subscriptions use `ReentrantCallbackGroup`** — all sensor subscriptions run on the default mutually-exclusive group. Multi-threading comes only from having multiple subscriptions dispatching to different threads in the executor.
- `ensure_r1pro_ros_env()` sets `ROS_DOMAIN_ID=41`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, and (if the XML exists) `FASTRTPS_DEFAULT_PROFILES_FILE`. It does **not** tune kernel sockets.
- Adapters are instantiated by `ControlCoordinator._setup_hardware()` and run in the **same Python process** as the coordinator's tick loop. Separate rclpy context = separate DDS participant/sockets, but the same interpreter and GIL.

**Guiding principle:** every phase either (a) closes the problem, or (b) narrows the hypothesis. Don't skip the data-capture steps even if a fix works — we want the evidence for the PR.

---

## Quick reference card

| Want to know... | Command |
|---|---|
| Current kernel UDP buffer ceiling | `sysctl net.core.rmem_max net.core.rmem_default` |
| UDP receive drops (system) | `nstat -az \| grep -i udp` |
| UDP drops delta over 10s | `nstat -n; sleep 10; nstat \| grep -i udp` |
| NIC-level drops | `ip -s link show <iface>` / `ethtool -S <iface> \| grep -iE 'drop\|miss\|fifo\|overrun'` |
| Softirq drops | `awk '{print $2}' /proc/net/softnet_stat \| paste -sd+ \| bc` |
| UDP sockets for adapter | `ss -unap \| grep <pid>` |
| Per-thread CPU | `top -H -p <pid>` |
| ROS CLI message rate | `ros2 topic hz <topic>` |
| Publisher QoS | `ros2 topic info -v <topic>` |
| Active RMW | `echo $RMW_IMPLEMENTATION` |

Keep one terminal running `watch -n1 'nstat -az | grep -iE "UdpRcvbufErrors|UdpInDatagrams|UdpInErrors"'` for the whole session. When that counter climbs, you know the layer.

---

## Unbounded rerun-viewer lag when wrist DEPTH publishes (2026-07-17)

When the wrist cams are healthy, their aligned-depth streams put ~3 MB/s of
raw 16-bit frames on LCM — and the rerun bridge **fully decodes every LCM
message BEFORE its max_hz throttle and visual_override suppression run**
(LCMEncoderMixin decodes in subscribe). The gRPC link budget stays fine
(depth is suppressed, ~1.5 MB/s), but the bridge worker saturates on decode
and the 64 MB LCM socket buffers become a many-seconds FIFO: every stream
the bridge logs (heads included) queues behind the depth flood → viewer lag
grows without bound. Fix: `enable_wrist_depth: bool = False` in
R1ProConnectionConfig (connection.py) — wrist depth is not subscribed at
all unless explicitly enabled. Until Tier-2 passthrough (and
throttle-before-decode in the bridge), the budget does not fit wrist depth.

## Wrist D405 wedge (works at boot → random "Frames Timeout" forever)

Diagnosed 2026-07-17 on the new-gen R1 Pro; same signature previously seen on
robot #1's right wrist — it is the PLATFORM, not a camera/cable/port:

- Signature in `sudo dmesg -T`: `uvcvideo ... Non-zero status (-71) in video
  completion handler` (EPROTO, USB3 link-level errors) escalating to
  `Failed to resubmit video URB (-1)` — at that point uvcvideo's streaming
  loop is dead and never recovers; realsense floods `Frames Timeout` at
  0.0–0.3 Hz while `lsusb -t` still shows a healthy 5000M enumeration.
- **Both wrists fail in the same second** every time — they share one PCIe
  xHCI controller (`0004:01:00.0`, USB bus 3+4), so the fault is upstream of
  the cameras.
- Stack: librealsense 2.57.5 on the **V4L2 backend** over **stock, unpatched
  uvcvideo 1.1.1** on the 5.15 RT Jetson kernel — the exact combination Intel
  warns about (their guidance: patched kernel modules or an RSUSB build).
- USB autosuspend was active on both cams (suspend/resume on a marginal link
  is a classic EPROTO trigger). Disabled + persisted:
  `/etc/udev/rules.d/99-realsense-no-autosuspend.rules`.

**KEY refinement (2026-07-17): the fw reset only sticks when realsense
restarts ALONE.** `initial_reset` is baked into the start script, but on a
FULL tree boot the cams still frequently wedge — the reset fires (dmesg shows
the USB disconnect/reconnect) yet re-wedges because it races the boot storm
(HDAS + both head cams + realsense all enumerating at once = USB/CPU
contention). Confirmed: standalone realsense relaunch recovered them every
time (03:53, 03:57, 04:08); the 04:05 full-tree boot did not. So do NOT
"kill the tmux and re-run robot_startup" to fix dead wrists — that recreates
the storm. Instead bounce ONLY the realsense pane, after the rest has settled:

```bash
~/reset_wrist_cams.sh              # helper: C-c the realsense pane + relaunch
# or manually — in the realsense pane (hdas:0.3):
tmux send-keys -t hdas:0.3 C-c
cd ~/galaxea-dimos/install/startup_config/share/startup_config/script/boot/modules/hdas
./start_realsense_camera_r1pro.sh  # reset baked in; sudo pw 'nvidia'
```
Watch for `Resetting device...` → `RealSense Node Is Up!`, no more Timeouts
(~10 s). Verify: `python3 ~/dimos/scripts/r1pro_test/lcm_channel_rates.py 8 | grep wrist`.

Candidate boot fix (untested): add a `sleep 15` at the top of
start_realsense_camera_r1pro.sh so the cams enumerate AFTER the storm settles,
making boot reliable without the manual bounce.

Structural fix if wedges continue: rebuild librealsense with
`-DFORCE_RSUSB_BACKEND=ON` (userspace libusb, no uvcvideo dependency) or
install Intel's patched uvcvideo for this kernel.

---

## Topic reference (exact strings used by current adapters)

Pulled directly from `dimos/hardware/manipulators/r1pro/adapter.py` and `dimos/hardware/drive_trains/r1pro/adapter.py`. Use these exact strings with `ros2 topic hz` / `ros2 topic info -v`.

**Arm adapter (one per side, `side ∈ {left, right}`):**

| Direction | ROS topic | Type |
|---|---|---|
| sub (state) | `/hdas/feedback_arm_{side}` | `sensor_msgs/JointState` |
| pub (cmd) | `/motion_target/target_joint_state_arm_{side}` | `sensor_msgs/JointState` |
| pub (gripper) | `/motion_target/target_position_gripper_{side}` | `sensor_msgs/JointState` |
| pub (brake) | `/motion_target/brake_mode` | `std_msgs/Bool` |
| sub (sensor) | `/hdas/camera_wrist_{side}/color/image_raw/compressed` | `sensor_msgs/CompressedImage` |
| sub (sensor) | `/hdas/camera_wrist_{side}/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` |

**Chassis adapter:**

| Direction | ROS topic | Type |
|---|---|---|
| pub (cmd) | `/motion_target/target_speed_chassis` | `geometry_msgs/TwistStamped` |
| pub (acc limit) | `/motion_target/chassis_acc_limit` | `geometry_msgs/TwistStamped` |
| sub (Gate 1) | `/motion_control/chassis_speed` | `geometry_msgs/TwistStamped` |
| sub (sensor) | `/hdas/camera_head/left_raw/image_raw_color/compressed` | `sensor_msgs/CompressedImage` |
| sub (sensor) | `/hdas/camera_chassis_front_left/rgb/compressed` | `sensor_msgs/CompressedImage` |
| sub (sensor) | `/hdas/camera_chassis_front_right/rgb/compressed` | `sensor_msgs/CompressedImage` |
| sub (sensor) | `/hdas/camera_chassis_left/rgb/compressed` | `sensor_msgs/CompressedImage` |
| sub (sensor) | `/hdas/camera_chassis_right/rgb/compressed` | `sensor_msgs/CompressedImage` |
| sub (sensor) | `/hdas/camera_chassis_rear/rgb/compressed` | `sensor_msgs/CompressedImage` |
| sub (sensor) | `/hdas/camera_head/depth/depth_registered` | `sensor_msgs/Image` |
| sub (sensor) | `/hdas/lidar_chassis_left` | `sensor_msgs/PointCloud2` |
| sub (sensor) | `/hdas/imu_chassis` | `sensor_msgs/Imu` |
| sub (sensor) | `/hdas/imu_torso` | `sensor_msgs/Imu` |

LCM topics published by the adapter (what the rest of DimOS consumes) follow `/r1pro/{hardware_id}/<name>` — see adapter source for the full map.

---

## Adapter log pattern to watch

Decode workers print every 5 seconds:

```
R1 Pro left wrist_color: 150 callbacks, 148 frames broadcast in last 5.0s
R1 Pro chassis head: 150 callbacks, 148 frames broadcast in last 5.0s
```

Interpretation:

- `N callbacks, M frames` where `N ≈ M ≈ 150` at 30 Hz over 5 s → healthy.
- `0 callbacks, 0 frames` → DDS/kernel stopped delivering to this subscription.
- `N callbacks, 0 frames` → DDS is alive, decode or LCM broadcast is failing.
- `N callbacks, M frames` with `M << N` → decode queue is dropping (can only happen if the worker is too slow; `maxsize=1` means newer frames clobber older).

---

## Phase 0 — Pre-flight (5 min)

Capture starting state. Paste results into the log template at the bottom.

```bash
# System state
uname -a
lsb_release -a
sysctl net.core.rmem_max net.core.rmem_default net.core.wmem_max net.core.netdev_max_backlog

# ROS env (these are set by ensure_r1pro_ros_env at adapter.connect())
echo "ROS_DISTRO=$ROS_DISTRO"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
echo "FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE"

# Which interface carries robot traffic (look for 192.168.123.100)
ip route
ip -s link

# Sanity: does the robot see us at all
ros2 node list | head -20
```

**Stop conditions for phase 0:**
- If `ros2 node list` shows nothing from the robot side, this isn't a sensor problem — it's discovery/network. Fix that before proceeding (check `/opt/ros/jazzy/setup.bash` is sourced, `FASTRTPS_DEFAULT_PROFILES_FILE` points at `scripts/r1pro_test/fastdds_r1pro.xml`, robot is actually running).
- If `rmem_max` is already > 16 MB, the Phase 3 fix alone won't help and you should plan to combine it with Phase 4 or skip straight to Phase 4.

---

## Phase 1 — Baseline (no adapter running)

Confirm what "normal" looks like. This is the ceiling.

In a plain shell, **robot started, DimOS NOT running**:

```bash
# One terminal per topic, 15 s each
ros2 topic hz /hdas/camera_head/left_raw/image_raw_color/compressed
ros2 topic hz /hdas/camera_wrist_left/color/image_raw/compressed
ros2 topic hz /hdas/lidar_chassis_left
ros2 topic hz /hdas/imu_chassis
ros2 topic hz /hdas/feedback_arm_left

# Publisher QoS — capture once
ros2 topic info -v /hdas/camera_head/left_raw/image_raw_color/compressed
ros2 topic info -v /hdas/camera_wrist_left/color/image_raw/compressed
ros2 topic info -v /hdas/lidar_chassis_left
```

Record for each topic: observed Hz, publisher Reliability (`RELIABLE` / `BEST_EFFORT`), History depth.

**Why the QoS check matters:** if the publisher is `BEST_EFFORT`, you **cannot** flip the subscriber to `RELIABLE` — QoS won't match, the subscription will silently receive zero messages. This rules out Phase 5a before you try it. The adapters currently match the publisher's reliability by default via `RawROS`, so this only matters if you hand-tune QoS.

**Also capture idle drops** (no adapter running):
```bash
nstat -n; sleep 10; nstat | grep -iE "Udp|IpExt"
```

Idle `UdpRcvbufErrors` should be ~zero. Anything else is a pre-existing issue and skews all downstream conclusions.

---

## Phase 2 — Reproduce the regression (the critical phase)

Goal: reproduce the frame drop AND capture data at every layer during the drop.

**Four terminals, set up before launching the adapter:**

1. **Terminal A:** `dimos run r1pro-full` — adapter + ControlCoordinator + ManipulationModule. Keep logs visible. Watch for the 5-second `"R1 Pro … N callbacks, M frames broadcast"` lines.
2. **Terminal B:** `ros2 topic hz /hdas/camera_head/left_raw/image_raw_color/compressed` — the CLI's independent view of the same ROS publisher the adapter subscribes to.
3. **Terminal C:** `watch -n1 'nstat -az | grep -iE "UdpRcvbufErrors|UdpInErrors|UdpInDatagrams"'`
4. **Terminal D:** Free for probes. Have these ready:
   ```bash
   # Find the PID — dimos runs via workers, the adapter lives in a worker process
   PID=$(pgrep -f "dimos.*r1pro-full" | head -1); echo "dimos-main PID=$PID"
   # Since adapters live in the same process as the coordinator (see Phase 6 note),
   # look for the worker hosting the coordinator module:
   ps --ppid $PID -o pid,cmd
   WORKER_PID=<pick the one running the coordinator worker>

   # UDP sockets and queue depths
   ss -unap | grep $WORKER_PID
   # Per-thread CPU
   top -H -p $WORKER_PID -b -n1 | head -30
   # NIC drops (substitute your interface from Phase 0)
   ip -s link show <iface>
   ```

**Sequence:**

1. Launch `dimos run r1pro-full`. The blueprint ordering is: `ManipulationModule` + `ControlCoordinator` + adapters.
2. **Before the ControlCoordinator tick loop starts writing commands**, confirm:
   - Terminal A shows healthy 5-second `"N callbacks, N frames broadcast"` lines for every sensor.
   - Terminal B shows ~30 Hz (or whatever baseline was in Phase 1).
   - Terminal C counters are flat.
3. The ControlCoordinator tick loop starts at 100 Hz as soon as `start()` finishes. From this moment you have 30–60 s before anything is suspicious.
4. When frame drops appear (`0 callbacks` or `N callbacks, 0 frames` in Terminal A), record:

| Observation | Terminal | Normal | Regression |
|---|---|---|---|
| Adapter-reported camera Hz (from log line) | A | ~30 | ? |
| CLI `ros2 topic hz` same topic | B | ~30 | **? ← critical** |
| `UdpRcvbufErrors` rate of climb | C | 0 | ? |
| `UdpInDatagrams` rate | C | — | ? |
| Socket rx queue backlog | D (`ss`) | small | ? |
| Hottest Python thread CPU | D (`top -H`) | — | ?% (thread name: ?) |
| NIC rx_dropped delta | D (`ip -s`) | 0 | ? |

---

## The diagnostic fork — read carefully

Based on **Terminal B** (`ros2 topic hz` from CLI) during the regression:

### Fork A — CLI also sees ~0 Hz

The drop is **below rclpy**. Kernel or DDS layer. The adapter's Python code is irrelevant — frames never made it up the stack.

- If `UdpRcvbufErrors` is climbing → **kernel UDP buffer overflow**. Go to Phase 3.
- If `UdpRcvbufErrors` is flat but NIC shows `rx_dropped`/`rx_missed` climbing → **NIC ring buffer overflow**. Different fix (`ethtool -G`, IRQ affinity). Unlikely on a laptop at these rates but possible.
- If both counters flat but CLI still sees 0 Hz → **DDS-layer drop** (FastDDS history queue, fragment reassembly timeout). Go to Phase 4.

### Fork B — CLI sees ~30 Hz, adapter log shows `0 callbacks`

The drop is **inside our Python process**. Kernel is delivering fine; we're losing frames between socket read and the callback, or the executor has died.

- Typically means spin thread died, GIL contention, or executor starvation.
- Go to Phase 5.

### Fork C — CLI sees something in between (say 10 Hz) and adapter sees 0

Both problems at once. Fix Fork A first (kernel fills the ceiling), then Fork B (Python catches what's delivered).

### Fork D (worth checking) — adapter log shows `N callbacks, 0 frames broadcast`

DDS is delivering to the subscription, but the decode worker isn't broadcasting. Rare but possible. Check:
- Is the worker thread alive? `top -H` should show the `r1pro_{side}_color` / `r1pro_chassis_{name}` thread.
- Is the LCM `transport.broadcast()` raising? Check adapter logs for exceptions.
- Is `cv2.imdecode` returning `None`? That path is logged but worth grepping for.

This is a pure Python/process problem — the kernel and DDS are blameless. Go to Phase 5c.

---

## Phase 3 — Kernel UDP buffer fix (Fork A path)

Raise the ceiling and re-test. This is the single highest-probability fix.

```bash
# Temporary (lost on reboot) — good for testing
sudo sysctl -w net.core.rmem_max=33554432
sudo sysctl -w net.core.rmem_default=33554432

# Verify
sysctl net.core.rmem_max net.core.rmem_default
```

**Important:** `rmem_max` is the ceiling. The socket still has to request a larger buffer via `SO_RCVBUF`. FastDDS does this based on its own configuration. Raising `rmem_max` alone may not help if FastDDS is still asking for 208 KB. This is why Phase 4 exists and why Phase 3 + Phase 4 are usually done together.

**Restart the adapter** (existing sockets don't resize) and re-run Phase 2.

| Result | Next |
|---|---|
| CLI 30 Hz, adapter 30 Hz, `UdpRcvbufErrors` flat | **Done.** Make persistent (below) + add a warning in `ensure_r1pro_ros_env()` (below). |
| CLI 30 Hz, adapter still 0 | Crossed from Fork A to Fork B. Go to Phase 5. |
| CLI still 0, `UdpRcvbufErrors` still climbing | Kernel ceiling raised but FastDDS isn't requesting the larger buffer. Go to Phase 4. |
| CLI still 0, `UdpRcvbufErrors` flat | Not a kernel-buffer issue. Go to Phase 4 (DDS), then Phase 5. |

**Make persistent after confirmation:**
```bash
sudo tee /etc/sysctl.d/60-r1pro-ros2.conf <<'EOF'
net.core.rmem_max=33554432
net.core.rmem_default=33554432
EOF
sudo sysctl --system
```

**Code change: add a warning in `ensure_r1pro_ros_env()`.** File: [dimos/hardware/r1pro_ros_env.py](dimos/hardware/r1pro_ros_env.py). Right before the function returns, read `/proc/sys/net/core/rmem_max` and `log.warning(...)` if below ~16 MB. Don't attempt `sudo sysctl` from Python — just warn loudly so the next operator knows.

---

## Phase 4 — FastDDS transport tuning

If Phase 3 alone didn't resolve it and you're still on Fork A, FastDDS's own buffer request is too low.

**Before editing the XML**, run `ros2 topic info -v` on a problem topic while the adapter is running. Confirm you're looking at FastDDS and not something weirder.

Edit [scripts/r1pro_test/fastdds_r1pro.xml](scripts/r1pro_test/fastdds_r1pro.xml). The current file uses **locator-based config** (no `interfaceWhiteList`/`allowlist`) because that was broken across FastDDS 2.x (Humble) and 3.x (Jazzy); don't reintroduce it.

Add a transport descriptor:

```xml
<transport_descriptors>
    <transport_descriptor>
        <transport_id>udp_r1pro</transport_id>
        <type>UDPv4</type>
        <sendBufferSize>33554432</sendBufferSize>
        <receiveBufferSize>33554432</receiveBufferSize>
        <maxMessageSize>65500</maxMessageSize>
    </transport_descriptor>
</transport_descriptors>
```

And reference it in the participant:

```xml
<rtps>
    <userTransports>
        <transport_id>udp_r1pro</transport_id>
    </userTransports>
    <!-- Leave useBuiltinTransports at default (true). Disabling it will break
         discovery with the robot. Custom UDP sits alongside builtin UDP+SHM. -->
    ...existing locator config...
</rtps>
```

After restart, confirm:
- `ros2 topic list` still enumerates robot topics (if empty, you broke discovery — revert).
- Adapter sensor logs show callbacks again.

**Cheap alternative test: CycloneDDS.** If the FastDDS XML tuning doesn't fix it, swap RMW for 10 seconds to see if it's a FastDDS bug:

```bash
# One-shot test
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
dimos run r1pro-full
```

CycloneDDS handles large fragmented messages more gracefully out of the box. If this fixes it, you have two options:
1. Adopt Cyclone as the R1 Pro default — update `ensure_r1pro_ros_env()` to set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` **but only after verifying Humble/Jazzy Cyclone interop is clean**; the reason we're on FastDDS is documented in the README.
2. Keep FastDDS and commit to the XML tuning.

Check install first: `dpkg -l | grep cyclonedds` or `ros2 pkg list | grep cyclone`.

---

## Phase 5 — Python-layer fixes (Fork B / Fork D path)

You're here if the CLI sees 30 Hz but the adapter sees 0 callbacks, or if callbacks are happening but broadcasts aren't.

### 5a. QoS compatibility sanity check

Re-read the publisher QoS captured in Phase 1. If the publisher is `BEST_EFFORT`, do **not** set the subscriber to `RELIABLE`. `RawROS` matches by default so this is mostly a don't-break-what-works note. If you override QoS in the adapter at some point, check compatibility first.

### 5b. Add `ReentrantCallbackGroup` to sensor subscriptions

**This is the highest-probability Python-layer fix, and our code currently does not use it.** Both adapters create subscriptions without specifying a `callback_group`, which means rclpy defaults to `MutuallyExclusiveCallbackGroup`. Even with `MultiThreadedExecutor(num_threads=4)`, every callback on a single subscription serializes against itself, and callbacks on the same group serialize against each other. At 30 fps per camera × 6 cameras on the chassis, that's 180 callback-dispatches/sec all contending for the same mutex.

**Change:**

File: [dimos/hardware/drive_trains/r1pro/adapter.py](dimos/hardware/drive_trains/r1pro/adapter.py) (chassis has 6 cameras so hits this hardest).
File: [dimos/hardware/manipulators/r1pro/adapter.py](dimos/hardware/manipulators/r1pro/adapter.py).

Where the sensor subscriptions are created, add a shared `ReentrantCallbackGroup`:

```python
from rclpy.callback_groups import ReentrantCallbackGroup

# in _setup_sensor_streams / _start (sensor path)
self._sensor_cb_group = ReentrantCallbackGroup()

self._sensor_node.create_subscription(
    CompressedImage,
    topic,
    callback,
    qos,
    callback_group=self._sensor_cb_group,
)
```

Apply this to **every sensor subscription** (wrist cameras, chassis cameras, depth, lidar, IMUs). Do **not** apply it to `/hdas/feedback_arm_*` or `/motion_control/chassis_speed` — you want those serialized (they feed the control loop).

Rebuild, re-run Phase 2. Record the before/after.

### 5c. Confirm decode is actually off the spin thread

Our code *looks* like it decodes off-spin (queue.Queue + worker thread), but verify empirically during the regression:

```bash
WORKER_PID=<from Phase 2>
top -H -p $WORKER_PID
```

Thread names of interest:
- `r1pro_{side}_color`, `r1pro_{side}_depth` (arm)
- `r1pro_chassis_{head,chassis_front_left,...}_color` (chassis)
- rclpy executor threads (typically unnamed or generic)

What to look for:
- **Single rclpy executor thread pegged near 100%** → spin thread is doing more than it should. Possibly because callback contention (see 5b) means one thread is actually serializing everything.
- **A `r1pro_*_color` worker pegged at 100%** → decode is off-spin correctly but single-worker-per-camera is saturated. Each `cv2.imdecode` releases the GIL, so adding more workers helps; or move decode to a `ProcessPoolExecutor`.
- **All CPU split across many threads, none pegged** → CPU isn't the bottleneck. Look at `ss` queue depths and socket errors.

### 5d. py-spy live profile

```bash
sudo py-spy top --pid $WORKER_PID
# or a 30-second sample:
sudo py-spy record -o r1pro.svg --pid $WORKER_PID --duration 30
```

What hot functions suggest:
- `rclpy` executor internals dominate → executor starvation (likely fixed by 5b).
- `cv2.imdecode` dominates → need more decode parallelism.
- Lock/wait functions dominate → GIL contention. Consider ProcessPool or moving adapter to a separate worker (Phase 6).

---

## Phase 6 — Architectural questions (if nothing above is enough)

At this point you have evidence for an architectural change. The original runbook left two questions open; here's what we now know about the current code:

### Q1: Do adapters run in a separate process from the coordinator?

**No.** `R1ProArmAdapter` and `R1ProChassisAdapter` are instantiated by `ControlCoordinator._setup_hardware()` and live in the **same worker process** as the coordinator's tick loop. Separate `rclpy.Context()` gives them separate DDS participants and separate UDP sockets, but they share the Python interpreter and GIL.

**Implication:** the "tune n_workers in the coordinator" option doesn't separate sensor decode from coordinator tick under the current blueprint wiring. To actually separate them you'd need to restructure so the adapter lives in its own worker/module, publishes its sensor LCM topics from there, and exposes a minimal RPC surface to the coordinator for `read_state`/`write_command`.

### Q2: Does the separate `rclpy.Context()` actually produce separate UDP sockets?

Verify before restructuring:
```bash
WORKER_PID=<from Phase 2>
ss -unap | grep $WORKER_PID | wc -l
```

- **Multiple sockets (~2 per context: data + discovery)** → FastDDS honored the context isolation. Separate participants are real.
- **Single socket** → the isolation pattern is illusory at the socket level; moving to a separate process is your only real isolation.

### Decision tree

| Evidence | Fix |
|---|---|
| Phase 3 + Phase 4 got 30 Hz, sustained | Kernel/DDS tuning is sufficient. Persist sysctl, commit XML, add `ensure_r1pro_ros_env()` warning. |
| Phase 5b got 30 Hz, sustained | Callback-group serialization was the cause. Commit the change; no architecture work needed. |
| Phase 5c shows decode worker pegged even with 5b | One decode thread per camera isn't enough. Add `ProcessPoolExecutor` for decode, or multiple worker threads per camera. |
| Nothing moved the needle | Restructure: adapters as their own worker. Collect full diagnostics below before starting. |

---

## Log template — fill in as you go

Copy this into `scripts/r1pro_test/sensor_drop_log_<date>.md` (or similar) and fill in. If you hit us up with this filled out, we have everything we need for the next session.

```
== PHASE 0 STARTING STATE ==
Date/time: Wed Apr 22 21:47 UTC 2026 — container session
Laptop kernel: Linux mustafa-Zenbook 6.17.0-22-generic
Execution env: Docker container dimos-ros-dev, network_mode=host (rootful Docker),
               container PID namespace only; netns is shared with laptop host (verified
               via /proc/<pid>/ns/net inode match).
rmem_max (before): 212992  (Linux default, ~208 KB — too small for R1 Pro sensor load)
rmem_default (before): 212992
netdev_max_backlog: 1000
sysctl write gotcha: `sudo sysctl -w net.core.rmem_max=...` FAILS inside the container
    ("Read-only file system"). `net.core.*` isn't a namespaced sysctl so Docker
    can't expose it per-container, and --network=host doesn't make /proc/sys writable.
    Fix must be applied on the laptop host; the container inherits it via shared netns.
ROS_DISTRO (container): humble (robot is also humble — no RMW cross-version compat needed)
ROS_DOMAIN_ID: 41
RMW_IMPLEMENTATION: rmw_fastrtps_cpp
FASTRTPS_DEFAULT_PROFILES_FILE: <unset> — default multicast PDP works Humble↔Humble
    over --network=host on the direct ethernet link. The unicast XML at
    scripts/r1pro_test/fastdds_r1pro.xml is retained as a fallback only.
Gotcha: `ros2 daemon stop` before any CLI discovery test — the daemon's discovery
    cache persists across sessions and made discovery look broken for ~1 hour
    when it was actually working. See memory: ros2_daemon_staleness.
Interface carrying robot traffic: enxf8e43bb7046c (192.168.123.100/24, direct
    ethernet to robot at 192.168.123.150). Default route is wlp3s0 (wifi);
    192.168.123.0/24 is a more-specific route via the ethernet.
NIC baseline drops (before test): enxf8e43bb7046c — RX 201216 pkts, 0 errors,
    0 dropped, 0 missed. NIC layer is clean; all frame loss is above the NIC
    (kernel UDP buffer).

== PHASE 1 BASELINE (NO ADAPTER) ==
Run from container with FASTRTPS_DEFAULT_PROFILES_FILE unset (pure multicast).
Observed 51 sensor/control topics after `ros2 daemon stop` + `ros2 topic list --no-daemon`.

Topic                                                                      Hz      Reliability      Durability        Node
/hdas/camera_head/left_raw/image_raw_color/compressed                      ~15     RELIABLE         VOLATILE          /signal_camera
/hdas/camera_wrist_left/color/image_raw/compressed                         ~15     RELIABLE         TRANSIENT_LOCAL   /hdas/camera_wrist_left
/hdas/camera_chassis_front_left/rgb/compressed                             ___     ___              ___               not tested
/hdas/lidar_chassis_left                                                   N/A — topic does NOT exist in R1PROBody.d session; perception
                                                                                  session not running, chassis lidar publisher absent. Remove from
                                                                                  adapter expected-topics if session is authoritative.
/hdas/imu_chassis                                                          ~200    RELIABLE         TRANSIENT_LOCAL   HDAS
/hdas/feedback_arm_left                                                    ~200    RELIABLE         TRANSIENT_LOCAL   HDAS

NOTE: multi-second stalls (max 12.5–12.9 s) observed on EVERY topic during baseline
with just `ros2 topic hz` as a single subscriber (no DimOS adapter). After a stall
the rate average collapses because topic hz's window keeps accumulating the gap;
the underlying publisher rate stays correct. This is the signature of Phase 3
(kernel UDP buffer overflow) manifesting even at baseline.

Idle UDP drops (nstat cumulative during Phase 1 test, single `topic hz` subscriber):
    UdpInDatagrams:   235539
    UdpInErrors:       1498
    UdpRcvbufErrors:   1498    ← all errors are receive-buffer overflows
    → Signature: every packet lost is due to undersized SO_RCVBUF / rmem_max.
      Phase 3 fix (raise rmem_max to 32 MB) is the first thing to try.

== PHASE 3 (applied after Phase 1 — baseline re-test with raised rmem_max) ==
Fix applied on laptop host (not in container — sysctl net.core.* is non-namespaced):
    sudo sysctl -w net.core.rmem_max=33554432
    sudo sysctl -w net.core.rmem_default=33554432
Verified in container: rmem_max=33554432 (inherited via --network=host shared netns).

Re-run of `ros2 topic hz /hdas/camera_head/left_raw/image_raw_color/compressed` for ~10s:
    rate: 15 Hz steady
    max:  0.109 s (normal jitter; prior runs had 12.5 s stalls)
    std:  0.008 s
nstat delta during test:
    UdpInDatagrams:   4877
    UdpInErrors:      0  (nstat omits zero counters — verified absent)
    UdpRcvbufErrors:  0
    → Baseline with one subscriber now clean with raised rmem_max.

TODO: re-test under real load (dimos run r1pro-full with ControlCoordinator tick loop
+ 6 chassis cameras + 2 wrist cameras + IMUs) to confirm fix holds under full sensor
concurrency. If errors stay flat, Phase 3 is the full fix; make persistent via
/etc/sysctl.d/60-r1pro-ros2.conf and add rmem_max warning to ensure_r1pro_ros_env().

== PHASE 2 DURING REGRESSION ==
Blueprint: r1pro-full
Coordinator PID:
Worker PID (hosting adapters + coordinator):
Adapter log (copy one "N callbacks, M frames" line per sensor):
  left wrist_color:   ___ callbacks, ___ frames in 5s
  left wrist_depth:   ___ callbacks, ___ frames in 5s
  right wrist_color:  ___ callbacks, ___ frames in 5s
  chassis head:       ___ callbacks, ___ frames in 5s
  chassis lidar:      ___ callbacks, ___ frames in 5s
  chassis imu:        ___ callbacks, ___ frames in 5s

CLI ros2 topic hz (same topic as a failing adapter sensor):  ___ Hz  <-- CRITICAL
UdpRcvbufErrors rate of climb:  ___ /sec
UdpInDatagrams rate:            ___ /sec
Max ss socket rx queue depth:   ___
Hottest Python thread CPU:      ___% (name: ___)
Rclpy executor thread CPU:      ___%
Decode worker thread CPU:       ___% (which worker: ___)
NIC rx_dropped delta (30s):     ___
NIC rx_missed / overrun delta:  ___

FORK: A / B / C / D

== PHASE 3 (if Fork A) ==
rmem_max after:                 ___
Adapter frame count after restart: ___
CLI Hz after restart:           ___
UdpRcvbufErrors after:          climbing / flat
Result: resolved / partial / no change

== PHASE 4 (if Phase 3 insufficient) ==
FastDDS XML transport_descriptor added: yes / no
useBuiltinTransports left at default: yes / no
ros2 topic list still enumerates robot topics: yes / no
Adapter frame count after:      ___
CycloneDDS swap tested: yes / no    Result: ___

== PHASE 5 (if Fork B / D) ==
Publisher reliability (from Phase 1): ___
ReentrantCallbackGroup applied: yes / no
Adapter frame count after:      ___
py-spy hot function:            ___
Decode thread pegged: yes / no (which: ___)

== PHASE 6 (if still unresolved) ==
ss socket count for worker:     ___ (>1 = real participant isolation)
Decision:
  [ ] Process separation (adapter as own worker)
  [ ] ProcessPoolExecutor for decode
  [ ] Other: ___

== OUTCOME ==
Resolved by: phase ___, change ___
Time spent:
Persistent config changes committed: yes / no
Follow-ups for next session: ___
```

---

## Stop conditions

You can stop when:

1. All adapter sensors report ~30 Hz under tick load, sustained for 5 minutes without regression.
2. `UdpRcvbufErrors` stays flat during operation.
3. `ss -unap` shows all sensor socket rx queues near empty (backpressure not accumulating).
4. The fix is committed — in code (adapter callback groups, XML) or in an env-setup script (sysctl config file).

If (1) holds but (2)–(4) don't, you have a fragile fix. Worth digging more before calling it.

---

## What to bring back if this runbook doesn't resolve it

- Filled-in log template above
- A 30-second `py-spy record -o r1pro.svg --pid $WORKER_PID` during the regression
- Output of `ros2 doctor --report`
- `ss -unap | grep $WORKER_PID` snapshot during regression
- `/proc/$WORKER_PID/status` (especially `Threads`, `VmRSS`) during regression
- Full adapter log from launch through 60 seconds of regression (so we can see the 5-second summary lines cross the threshold)

With those in hand we can decide process separation vs. ProcessPool vs. deeper DDS work on evidence rather than intuition.

---

## Appendix — HDAS launch edit applied on the old-gen robot (2026-07-03)

State of the robot's HDAS tmux launch config as we left it after the VI/SMMU
meltdown diagnosis (see [STREAM_RELIABILITY_DIAGNOSTIC.md](STREAM_RELIABILITY_DIAGNOSTIC.md)):
chassis CSI camera panes disabled, everything else kept.

```yaml
  - shell_command:
      - sleep 1
      - ./start_livox_lidar.sh
  - shell_command:
      - sleep 1
      - ./start_realsense_camera_r1pro.sh
  # DIMENSIONAL 2026-07-03: CSI camera panes disabled. Since the V2.2.1 reflash,
  # ANY Tegra VI/CSI capture floods dmesg with "VPR violation" / SMMU context
  # faults (verified by staged bring-up: lidar=clean, realsense=clean,
  # head signal_camera=faults + 0 frames, chassis v4l2=faults but 30 Hz frames).
  # Re-enable by restoring the panes below.
  - shell_command:
      - sleep 1
      - ./start_signal_camera_head.sh
  # - shell_command:
  #     - sleep 1
  #     - ./start_chassis_cameras_v4l2.sh
```
