# R1 Pro on-robot deployment — pitfalls & fixes (field debugging, 2026-07-17)

Everything that bit us bringing the dockerized dimos coordinator back up on the
new-gen R1 Pro after a reboot, in the order the layers peeled off. Each entry:
symptom → root cause → fix → where the fix lives. Detailed writeups live in
[docker/README.md](docker/README.md) ("Known traps"),
[SENSOR_DROP_RUNBOOK.md](SENSOR_DROP_RUNBOOK.md), and the main
[README.md](README.md); this is the map.

---

## 1. No internet after reboot (recurring)

**Symptom:** `ping github.com` → Network unreachable; docker build fails.
**Cause:** the robot's WiFi NetworkManager profile (`ATT6J4xPZS`) is
**manual/static IP with no gateway configured**. A previous session fixed it
with a live `ip route add`, which does not survive reboot.
**Fix (persistent this time):**
```bash
sudo nmcli con modify ATT6J4xPZS ipv4.gateway 192.168.1.254 ipv4.dns "8.8.8.8 1.1.1.1"
```
Robot IP is 192.168.1.88 on wlan0; verify after any reboot with `ip route | head -1`.

## 2. The stock vendor tree resurrects itself

**Symptom:** `robot_startup.sh kill` works, but seconds later the **stock**
`~/galaxea` tree (no chassis gatekeeper) is running again.
**Cause:** the pick-place-echo booth demo installs user systemd services;
`pick-place-watchdog.service` health-checks the vendor tmux sessions and
auto-runs `bringup.sh local` (kill + boot of **stock**) within ~30 s.
**Fix:** stop the watchdog before any tree switch:
```bash
systemctl --user stop pick-place-watchdog.service   # returns on reboot (still `enabled`)
```
**Extra trap:** the watchdog's bringup boots a **partial** stack — its env
can't find `zed_wrapper` (no head_depth) and livox lidar never publishes. If
lidar/head_depth are missing, you're probably on a watchdog-booted stock tree.
Also note `skill_runner`/`mqtt_bridge` from the same demo burn ~50% CPU —
stop them too when the demo isn't needed.

## 3. Coordinator "starts but no workers publish" — TWO stacked causes

The post-reboot mystery from the last handoff. Both had to be fixed.

**3a. ROS never sourced in non-interactive shells.**
`rclpy is not installed. ROS pubsub requires ROS 2` → `ExceptionGroup:
safe_thread_map failed`. The image appended `source /opt/ros/humble/setup.bash`
to `/root/.bashrc` — but stock `.bashrc` early-returns for non-interactive
shells, so `docker exec bash -lc "dimos run ..."` (any scripted run) never
sourced it. Only interactive `-it` shells worked, which is why it "worked when
I did it by hand."
**Fix:** the Dockerfile now **prepends** the source line (`sed -i '1i'`).

**3b. Fast DDS shared-memory blackhole.**
`ros2 topic list` fine, endpoint QoS visible, coordinator wires everything —
and **zero sensor data arrives**; feedback discovery times out
(`torso=False left=False right=False`). With `--ipc host` the root container
and the nvidia-user host nodes match via SHM locators, but host publishers
can't write into root-owned SHM ports — and Fast DDS **never falls back to
UDP per-endpoint**. Silent total data loss with perfect-looking discovery.
**Fix:** UDP-only transport profile
([docker/fastdds_udp_only.xml](docker/fastdds_udp_only.xml)) baked via
`FASTRTPS_DEFAULT_PROFILES_FILE` in the Dockerfile. Verified head cams at the
full 30 Hz through the container afterward.

## 4. Wrong chassis test for the new gen

**Symptom:** `test_03_chassis_command.py` runs, robot doesn't move.
**Cause:** test_03 is the **old-gen** (V2.2.1, domain 41) contract — it
publishes `/cmd_vel` and trusts the gatekeeper to *forward* it. The new-gen
V2.3.0 gatekeeper only *holds gate preconditions*; it forwards nothing.
**Fix:** use `test_05_chassis_command_newgen.py` (domain 1, publishes directly
to the chassis controller). The tree must be `~/galaxea-dimos` (gatekeeper up).

## 5. Wrist D405s: works at boot → random "Frames Timeout" forever

The big one — full detail in
[SENSOR_DROP_RUNBOOK.md](SENSOR_DROP_RUNBOOK.md) "Wrist D405 wedge".

**Symptom:** wrist cams stream fine, then drop to 0–0.3 Hz with realsense
flooding `Frames didn't arrived within 5 seconds`; `lsusb -t` still shows a
healthy 5 Gbps enumeration. Seen on TWO different robots → not a camera,
cable, or port.
**Diagnosis:** `sudo dmesg -T` shows `uvcvideo: Non-zero status (-71)`
(EPROTO, USB3 link errors) escalating to `Failed to resubmit video URB (-1)`
— at which point uvcvideo's streaming loop is dead until the device is truly
power-cycled. **Both wrists fail in the same second every time**: they share
one PCIe xHCI controller (`0004:01:00.0`), so the fault is upstream of the
cameras. Stack is librealsense 2.57.5 on the **V4L2 backend over stock,
unpatched uvcvideo** on the RT Jetson kernel — the combination Intel warns
about. Likely trigger: CPU starvation (see §7) making the node miss frame
deadlines. USB autosuspend was also active (classic EPROTO trigger) — now
disabled and persisted (`/etc/udev/rules.d/99-realsense-no-autosuspend.rules`).

**What does NOT recover a wedge** (all tried):
- restarting the realsense node — firmware stays wedged
- `echo 0/1 > .../authorized` re-enumeration — no VBUS cut
- sysfs per-port `disable` — EACCES even as root on this kernel
- `uhubctl` — rejects the Tegra root hub despite advertised ppps
- `initial_reset:=true` (librealsense `hardware_reset()`) — worked sometimes
  standalone but is **unreliable and can re-wedge the cams**; do not rely on it

**What DOES work:** a real power cycle = PCI unbind/rebind of the xHCI
controller, then a **plain** relaunch (no initial_reset):
```bash
~/reset_wrist_cams.sh     # does all of it; ~25 s
```
(The cellular modem + an aux USB serial share that controller and blip ~10 s;
they auto-recover. Chassis/arms are on CAN — unaffected.)
**Boot is self-healing now:** the `~/galaxea-dimos` start script waits out the
boot storm, launches plain, **verifies frames actually flow**, and retries.
Don't kill + re-run the whole tree to fix dead wrists — that recreates the
boot storm; bounce only the realsense pane (or run the helper).

## 6. Coordinator dies minutes after "it's running" (session-tied lifetime)

**Symptom:** coordinator verified up, then a clean, orderly shutdown a few
minutes later with no error.
**Cause:** it was launched via `docker exec -i` — the coordinator's stdin is
tied to that client process. When the client dies (terminal closes, background
task reaped), `dimos run` sees stdin EOF and gracefully shuts down.
**Fix:** always launch **detached**:
```bash
sudo docker exec -d dimos-r1pro bash -lc 'printf "y\n" | dimos run r1pro-coordinator > /var/log/coordinator.log 2>&1'
sudo docker exec dimos-r1pro tail -f /var/log/coordinator.log
```
**Related footguns:** `pkill -f "dimos run"` **kills its own wrapper shell**
(the `bash -c` cmdline matches the pattern — hit this twice); use
`dimos stop` inside the container instead. And a new instance refuses to
start while an old one holds ports ("Ports in use ... Run 'dimos stop' first").

## 7. CPU is the real budget on the Orin

**Symptom:** everything degrades at once — stream rates sag, the D405s wedge,
the rerun bridge starves. Load average hit 32 on 8 cores.
**Cause:** the box is shared with HDAS *plus* leaked debug/demo processes:
two forgotten `lcmspy` instances (~25% each — one on the host as root, one
inside the container) and the pick-place `skill_runner` (~50%).
**Fix:** hunt strays whenever things get weird:
```bash
ps -eo pcpu,pid,comm --sort=-pcpu | head    # then pkill the leftovers
```
Killing one leaked `lcmspy` alone dropped load 32 → 10.

## 8. Rerun viewer lag grows unbounded (the "backpressure" issue)

**Symptom:** viewer latency climbs from ~1 s to 10-15 s and keeps growing;
LCM topics on the robot all healthy.
**Three interlocking causes:**

1. **Rerun's live gRPC stream is a reliable ordered log.** It never drops
   frames and has no latest-only mode. Once a viewer falls behind — for any
   reason — it **never catches up**; reconnecting fresh is the only recovery.
   `memory_limit` bounds the server's history store, NOT viewer lag.
2. **The bridge decodes every LCM message BEFORE its caps apply.** The
   `max_hz` throttle and `visual_override` suppression run after
   `LCMEncoderMixin` has already JPEG-decoded the frame. So suppressed/capped
   streams still cost full decode CPU. With 4 cams at 8-30 Hz that's ~45
   decodes/s on the bridge → pegged at 164% CPU → bursty gRPC sends
   (measured 28 s stalls) → viewer starves and backlogs. Raw wrist DEPTH
   (~3 MB/s, decoded then thrown away) was the worst offender.
3. **Oversized history buffer.** `memory_limit: 1GB` ballooned to 2.4 GB RSS
   with GC churn, and every viewer reconnect replays the whole buffer.

**Fixes (all producer-side — the only kind that works with a no-drop log):**
- `enable_wrist_depth: false` (R1ProConnectionConfig default) — raw wrist
  depth never enters LCM; flip on only for manipulation work, ideally after
  Tier-2.
- `color_publish_hz: 5.0` (R1ProConnectionConfig) — frames are skipped
  **before** `cv2.imdecode` in the connection's color loops, so only ~5 Hz
  per camera enters the pipeline at all (the viewer never showed more than
  5 Hz anyway — the bridge cap). Verified: rx=29.6/s, dec=4.6/s.
- rerun `memory_limit` 1 GB → **256 MB** (blueprint) — fast reconnects, no
  GC thrash.
- After any coordinator restart, **reconnect the viewer fresh** (each restart
  is a new recording; a stale viewer session sits on dead backlog).

**How to tell which hop is behind** (with a viewer connected):
```bash
# link throughput:      t1=$(cat /sys/class/net/wlan0/statistics/tx_bytes); sleep 10; ...
# robot->viewer TCP:    sudo ss -tinp | grep -A1 9877     # send-Q, delivery_rate, lastsnd
# producer health:      sudo docker exec dimos-r1pro ps -eo pcpu,rss,comm --sort=-pcpu | head
```
In our case: link 102 Mbit/s and idle (lastsnd 28 s!) — the producer was the
bottleneck, not WiFi, not the laptop.

## 9. Interim state & the structural fix

With all caps in place the steady state is: colors 4.6 Hz ×4, lidar 10 Hz,
motor/IMU ~85 Hz, odom 50 Hz, bounded ~1 s viewer latency. The caps exist
because of the **triple-JPEG codec path** (camera JPEG → connection decode →
LCM re-encode → bridge decode → viewer): ~80 ms CPU per frame per hop.
**Tier-2 JPEG passthrough** ([NEWGEN_DROP_ELIMINATION.md](NEWGEN_DROP_ELIMINATION.md))
carries the camera's original JPEG bytes end-to-end and logs
`rr.EncodedImage`, removing the decode tax entirely → head cams at 30 Hz,
massive CPU headroom, and room to re-enable wrist depth. ~1 day of work;
the single highest-leverage change left.

## Quick reference — the commands that matter

```bash
# stream health (LCM channel rates):
python3 ~/dimos/scripts/r1pro_test/lcm_channel_rates.py 10

# wrist cams wedged:
~/reset_wrist_cams.sh

# coordinator start (detached) / logs / stop:
sudo docker exec -d dimos-r1pro bash -lc 'printf "y\n" | dimos run r1pro-coordinator > /var/log/coordinator.log 2>&1'
sudo docker exec dimos-r1pro tail -f /var/log/coordinator.log
sudo docker exec dimos-r1pro bash -lc 'dimos stop'

# viewer (reconnect fresh after any coordinator restart):
rerun --connect rerun+http://192.168.1.88:9877/proxy

# chassis test (new gen):
export ROS_DOMAIN_ID=1 && cd ~/dimos && python3 scripts/r1pro_test/test_05_chassis_command_newgen.py

# tree switch (stop the demo watchdog first!):
systemctl --user stop pick-place-watchdog.service
cd ~/galaxea-dimos/install/startup_config/share/startup_config/script
./robot_startup.sh kill && ./robot_startup.sh boot ../sessions.d/ATCStandard/R1PROBody.d/
```
