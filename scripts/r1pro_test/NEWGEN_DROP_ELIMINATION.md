# New-gen R1 Pro — stream-drop elimination plan

Date: 2026-07-09 · Branch: `mustafa/task/r1pro-coordinator-integration`

**Situation:** moved from the old-gen base (5 chassis surround cams on the
broken CSI/V4L2 path) to the new-gen R1 Pro (base has lidars only; sensors are
head/top cameras + 2 wrist D405s). Streams still collapse, just ~10–40 s later
than the old gen. Goal today: identify the layer on the *new* robot, then ship
the on-robot docker deployment (untethered).

**Method:** same as always — one test per suspect, cheapest and most decisive
first. All tools already exist ([stream_dashboard.py](stream_dashboard.py),
[monitor_topic_hz.py](monitor_topic_hz.py)).

---

## Already eliminated on the old gen — do not re-test first

| Suspect | Verdict | Evidence |
|---|---|---|
| PC↔robot wire / LAN throughput | **Clean** | 938 Mbit/s TCP, 0 retransmits, 0 NIC drops, RTT 1.4 ms (STREAM_RELIABILITY 2026-07-03) |
| DDS engines (FastDDS / Cyclone, C level) | **Clean** | ≥700–2500 MiB/s loopback, 0 loss (DDS_PROFILING) |
| ROS discovery flapping | **Ruled out** | `pubs` stayed 1 through every stall (monitor_topic_hz, 2026-07-02) |
| PC kernel ingress (`rmem_max`) | **Fixed** | raised to 64 MB; `UdpRcvbufErrors` flat during pulls |
| Rerun/dimos as the cause of *ingress* stalls | **Exonerated (old gen)** | full collapse reproduced with **zero** dimos/rerun code running — monitor alone (2026-07-02 verdict) |

These transfer to the new robot **provisionally** — T2 re-checks the last one
in 15 minutes because the hardware changed.

## Carry-over facts that seed the suspect list

1. **The SMMU/VI fault storm was triggered by the HEAD camera, not the chassis
   cams.** Clean-boot trap (2026-07-03): first fault fires 1.4 ms after
   `signal_camera` makes its first NvMap allocation; chassis cams only *scaled*
   the fault rate (~2 faults/frame/camera) until meltdown. → "new base has no
   chassis cams" does **not** automatically clear the new robot. Fewer cameras
   = slower accumulation = "lives 10–20 s longer" is exactly what this bug
   would look like if the new gen ships the same TZTEK BSP + head unit.
   Compare DTB: old gen was `geac91_510jx0_r3_0_jp6.0_GA_v6.0.3` (logs.txt).
2. **D405 wrist cams have a known USB endpoint-stall loop** (`CLEAR_HALT` ×3 in
   dmesg); dead until replug + HDAS tmux restart.
3. **Robot egress fixes were identified but never applied:** eth qdisc
   tail-drop (`UdpSndbufErrors` == qdisc drops == 13689; fix `txqueuelen
   10000`) and FastDDS synchronous send cascading participant-wide.
4. **LCM subscription queue is back at 10000**
   ([lcmpubsub.py:125](../../dimos/protocol/pubsub/impl/lcmpubsub.py)) — the
   queue=4 experiment didn't survive the rebase. The rerun-bridge
   unbounded-lag mechanism (single drain thread, decode *before* throttle,
   10k FIFO replaying stale backlog) is still live in the code.
5. **`head_depth` never published on the old gen** — verify the topic exists
   at all on the new one before debugging its absence downstream.

---

## Suspect list (ranked by prior)

| # | Cause | Component | Test | Goes away with docker-on-robot? |
|---|---|---|---|---|
| 1 | Same BSP VI/SMMU syncpoint bug via head camera | Robot kernel / TZTEK camera driver | T1 | **No** |
| 2 | HDAS driver death / D405 USB stalls | Robot vendor stack / USB | T1 + which-topics-die pattern | No |
| 3 | Robot DDS egress (qdisc tail-drop, sync send) | Robot kernel net / FastDDS | T3 | **Yes** (loopback/SHM) |
| 4 | PC consumer backpressure — rerun bridge / LCM queue | PC dimos | T2 | Changes shape (bridge moves on-robot) |
| 5 | Robot CPU saturation / thermal throttle | Robot compute | T4 | No — gets *worse* (dimos adds load) |
| 6 | New-gen unknowns (different lidar count/topics, head unit) | Robot config | T0 | — |

Rerun the viewer itself hogging the PC is **not** a plausible cause of robot-side
0 Hz ingress (proven old-gen), but the bridge's lag mechanism (#4) is real and
still latent — it produces *growing lag / stale frames*, a different symptom
from hard drops. Keep the two symptoms separate in notes.

---

## Test sequence

### T0 — inventory the new robot (10 min, prerequisite)

On the robot:

```bash
uname -a; cat /etc/nv_tegra_release 2>/dev/null
dmesg | grep -m1 "DTB Build version"       # same geac91_…_v6.0.3 BSP as old gen?
nproc; free -h
ros2 node list                              # is /signal_camera still the head path?
ros2 topic list | grep -E "hdas|camera|lidar"
ros2 topic info -v <each camera + lidar>    # QoS + pub count
ros2 topic hz <head, wrists, lidar>         # robot-local baseline rates
```

Record: which head/top camera node exists, how many lidars, whether
`head_depth` publishes, whether topic names changed vs the
[runbook table](SENSOR_DROP_RUNBOOK.md#topic-reference-exact-strings-used-by-current-adapters).

### T1 — kernel fault storm check (10 min, most decisive)

Answers: *is the new gen sick at the capture layer like the old one?*

On the robot, from a clean boot, **before** starting anything:

```bash
dmesg -Tw | grep -E --line-buffered "smmu|camrtc|VPR|uncorr_err|CLEAR_HALT|xhci"
```

Then start HDAS, then start streaming (robot-local:
`python scripts/r1pro_test/stream_dashboard.py --stagger 10` on the robot).

- **Faults appear when the head camera starts** → same BSP bug, new gen not
  clean. Everything downstream is contaminated; escalate to Galaxea with the
  new-gen repro and run dimos with the head camera disabled to get a clean
  baseline for the rest of the battery.
- **dmesg quiet through full streaming** → biggest old-gen suspect cleared.
  Continue to T2.
- **`CLEAR_HALT`/xhci lines when a wrist dies** → suspect #2 confirmed for
  that drop; note whether the topic recovers or stays dead (dead-until-restart
  = driver death, not transport).

### T2 — the "is it rerun?" A/B/C (15 min, answers the direct question)

PC tethered, robot streaming. Three stages, ~5 min each; stop at the first
stage that reproduces the collapse:

1. **Monitor only, no dimos:** `python scripts/r1pro_test/monitor_topic_hz.py`
   (all topics). Collapse reproduces → PC consumers exonerated again; the
   problem is robot-side or egress (go T1/T3). This alone answers "is rerun
   the issue" — with no rerun process alive.
2. **dimos without rerun:** `dimos run r1pro-coordinator` (no `--viewer
   rerun` → no bridge composed at all). Clean in (1) but collapses here →
   the connection/coordinator side is implicated (watch its `sensor_stats`
   log lines: `rx` vs `decoded` isolates ingress vs decode).
3. **dimos + rerun viewer:** add `--viewer rerun`. Clean in (2) but collapses
   or lags here → bridge/viewer layer. Distinguish: 0 Hz at the *connection's*
   rx counters = real ingress drop; connection rx fine but viewer stale =
   the LCM 10k-queue lag mechanism (suspect #4, fix = bounded latest-wins
   mailbox in the bridge, throttle before decode).

While any stage runs, keep `nstat` deltas on the PC:
`nstat -n; sleep 10; nstat | grep -iE "UdpRcvbufErrors|UdpInErrors"`.

### T3 — robot egress counters during a tethered collapse (10 min)

Only meaningful while tethered (class disappears on-robot). On the robot,
while T2-stage-1 is collapsing:

```bash
nstat -n; sleep 10; nstat | grep -iE "SndbufErrors|UdpOutDatagrams"
tc -s qdisc show dev eth1        # or whichever iface carries PC traffic
ss -u -m | grep -A1 -E "tb|sb"   # actual SO_SNDBUF on DDS sockets
```

`SndbufErrors`/qdisc drops climbing → apply the never-applied fix and re-test:

```bash
sudo ip link set eth1 txqueuelen 10000    # instant, reversible
```

### T4 — robot compute / thermal (runs alongside T1–T3)

```bash
tegrastats --interval 1000    # log it: temps, throttling flags, per-core %
```

Look at the moment of collapse: CPU pegged / `throttling` / temp cliff vs
nothing-special. Also `free -h` before/after 10 min of streaming (leak check).

### T5 — robot-local full-stack soak (validates the docker plan)

Everything on the robot, no PC in the loop: run the dashboard (or dimos
itself, once containerized) robot-locally against the full blueprint sensor
set for ≥30 min. Old gen held indefinitely robot-local except for the CSI
storm — if the new gen soaks clean here, the untethered architecture is
validated and any remaining tethered drops are egress-class (which docker
eliminates anyway).

**Success criterion (same as runbook):** all sensors at rate for 30 min,
dmesg quiet, `tegrastats` steady, no counter climbing.

---

## Docker-on-robot: what it fixes, what it doesn't

**Eliminated by moving everything on-robot:** the entire egress class —
qdisc tail-drops, `wmem`/send-buffer limits, FastDDS synchronous-send
cascades, the wire itself, multi-NIC locator confusion. DDS goes
loopback/SHM (robot-local runs already proved this path tolerant).

**NOT eliminated:** the BSP capture bug (if T1 shows it), D405 USB stalls,
robot CPU/thermal budget — and dimos now *adds* its decode load to the same
box that runs HDAS.

**CPU budget is the real risk.** Current color path per frame:
robot JPEG → connection `cv2.imdecode` → raw over LCM? no — `JpegLcmTransport`
**re-encodes** JPEG → bridge decodes JPEG → `rr.log(raw)`. That was ~700% +
~400% CPU on a 16-core x86; the Orin shares ~8–12 weaker cores with HDAS.
The fix that makes on-robot viable is the long-planned **Tier 2 passthrough**
(STREAM_RELIABILITY remediation): carry the robot's original JPEG bytes
end-to-end, `rr.log(rr.EncodedImage(...))`, decode on the viewer GPU. This is
also what makes **WiFi viewing feasible** — `Image.to_rerun()` currently logs
raw pixels, so the rerun gRPC stream to an untethered laptop viewer would
carry uncompressed frames; EncodedImage makes it JPEG-sized.

**Container checklist (the known traps):**

- [ ] `--network=host` (DDS + LCM multicast both need it)
- [ ] LCM multicast loopback works in-container:
      `ip route add 239.255.76.67 dev lo` + `ip link set lo multicast on`
      (or `export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=0` with lo mcast on)
- [ ] `libturbojpeg` in the image — hidden native dep of `JpegLcmTransport`;
      bites as "decode error" on every color stream (2026-05-22 incident)
- [ ] `rerun_open="none"` on the robot (headless); PC connects with
      `rerun grpc://<robot-ip>:9876` — bridge already logs the URI. The new
      Popen spawn path in [bridge.py](../../dimos/visualization/rerun/bridge.py)
      degrades gracefully when no viewer binary exists — keep viewer binaries
      OUT of the robot image.
- [ ] rerun `memory_limit` stays "1GB" (blueprint) — the gRPC server buffers
      on the robot when the WiFi viewer lags; 25% of an Orin is too much.
- [x] If HDAS runs outside the container, FastDDS SHM between host and
      container is a TRAP: with `--ipc host` the SHM locators match but the
      uid mismatch (root container vs nvidia host) blackholes ALL user data —
      it does NOT fall back to UDP (disproven 2026-07-17). Force UDP-only via
      `FASTRTPS_DEFAULT_PROFILES_FILE` (docker/fastdds_udp_only.xml, baked).
- [ ] Persist robot sysctls (`rmem_max`/`wmem_max` 40 MB were manual — check
      `/etc/sysctl.d/`, they may not survive reboot; also `txqueuelen` udev
      rule if T3 confirms).

## Log

- 2026-07-09 — plan written. T0–T5 pending on the new-gen robot.
- 2026-07-09 — new blueprint ran against the new-gen robot with **no sensor
  drops / no crash** (tethered). Depth topics not arriving — robot-side
  publication to verify (head_depth never published on old gen either).
  Head stereo (left+right) streams + two-tab rerun layout added.
- 2026-07-10 — tethered via WiFi → low flowrate as expected (never a
  supported transport). Decision: go on-robot. **Docker kit written:
  [docker/](docker/)** (Dockerfile, entrypoint with lo-multicast LCM route,
  build/run scripts) — implements the container checklist above. arm64+py310
  dependency resolution validated with `uv pip compile`. Core-file changes
  (Image.py depth meter, ddsservice pydantic fix, bridge viewer spawn)
  stashed out of the branch for a separate PR (`git stash list`).
