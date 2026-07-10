# On-robot dimos container (R1 Pro)

Runs `dimos run r1pro-coordinator` on the robot itself. No bulk sensor data
crosses a physical link — the entire failure class documented in
[STREAM_RELIABILITY_DIAGNOSTIC.md](../STREAM_RELIABILITY_DIAGNOSTIC.md)
(qdisc tail-drops, send-buffer overflow, fragmented UDP over the wire/WiFi)
is structurally gone. The laptop's only job is viewing: a rerun viewer
connected over WiFi to the bridge's gRPC server, which carries JPEG-sized
frames after the `JpegLcmTransport` work.

HDAS (the vendor stack) stays on the host as-is; dimos talks to it over
local DDS (`ROS_DOMAIN_ID=1` on the new-gen V2.3.0 robot — old-gen was 41;
FastDDS, loopback/SHM).

> **Prerequisite (new-gen V2.3.0): boot the `~/galaxea-dimos` tree on the
> host first.** The container drives the chassis through the standalone
> `chassis_control_node` + on-robot gatekeeper that only the `~/galaxea-dimos`
> tree brings up (moca bypass — see the main README §14). If you launch the
> container against stock `~/galaxea`, sensors + arms may connect but the
> chassis will silently refuse to move. Boot order: `bash ~/canfd.sh` →
> `~/galaxea-dimos/.../robot_startup.sh boot ...` → wait ~30s for HDAS →
> then `run.sh`.

## Host prerequisites (once per robot)

The container inherits the host network namespace (`--network=host`) but
**cannot set global kernel sysctls itself** — the in-container `dimos` config
prompt will say "completed" but the write no-ops against the read-only host
`net.core.*`. Without a large UDP receive buffer, LCM floods
`kernel UDP receive buffer is very small` and drops large frames (JPEG /
pointcloud). Apply once on the host and persist (defaults are only ~208 KB):

```bash
sudo tee /etc/sysctl.d/60-r1pro-ros2.conf > /dev/null <<'EOF'
net.core.rmem_max = 67108864
net.core.rmem_default = 67108864
net.ipv4.ipfrag_high_thresh = 67108864
net.ipv4.ipfrag_low_thresh = 50331648
net.ipv4.ipfrag_time = 60
EOF
sudo sysctl --system
```

LCM sizes its socket buffer at creation, so (re)start the coordinator *after*
applying this.

## Build

On the robot (simplest, no qemu):

```bash
git clone <repo> && cd dimos      # or rsync your checkout
scripts/r1pro_test/docker/build.sh
```

`build.sh` passes `--network=host` to the build. The apt step needs DNS, and
the host resolves via the systemd-resolved stub (`127.0.0.53`) which lives on
the host loopback — unreachable from Docker's default bridge netns, so a plain
`docker build` fails with `Temporary failure resolving 'ports.ubuntu.com'`.
Host networking shares the working resolver.

Or cross-build from the laptop and ship it:

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64   # once
ROBOT=nvidia@192.168.1.88 scripts/r1pro_test/docker/build.sh --cross   # new-gen robot
```

## Run (on the robot)

`run.sh` starts the container with an **idle PID 1** — nothing auto-launches.
You exec in and run a blueprint by hand:

```bash
scripts/r1pro_test/docker/run.sh                 # container idles, stays up
docker exec -it dimos-r1pro bash
dimos run r1pro-coordinator                       # answer 'y' at the config prompt
```

The image's `/root/.bashrc` sources ROS, so the exec shell has rclpy. On an
older image built before that was baked in, source it yourself first — else
`R1ProConnection.start()` dies with "rclpy is not installed":

```bash
source /opt/ros/humble/setup.bash                # only needed on pre-bake images
```

Why not auto-launch: `dimos run` prompts interactively ("Apply system config
changes? [y/N]"), and a detached PID 1 has no stdin to answer it — it stalls.
Running by hand from `docker exec -it` gives it a TTY (and sources ROS via
.bashrc, which the ENTRYPOINT-only source skips). It also guarantees a single
coordinator instance; a second one collides on the LCM + rerun-gRPC ports
(`bind: Address already in use`).

To auto-run anyway (once you've made the system config persistent on the host
so there's nothing to prompt for), pass the command through:

```bash
scripts/r1pro_test/docker/run.sh dimos run r1pro-coordinator
docker logs -f dimos-r1pro
```

For on-robot development, mount your checkout over the baked-in code —
edits apply on container restart, no rebuild:

```bash
REPO=/home/<user>/dimos scripts/r1pro_test/docker/run.sh
```

## View from the laptop (untethered)

The bridge logs its gRPC URI at startup (`docker logs dimos-r1pro`). Then:

```bash
rerun --connect rerun+http://<robot-ip>:9877/proxy   # new-gen robot-ip: 192.168.1.88
```

`RERUN_HOST=0.0.0.0` is baked into the image — without it the server binds
127.0.0.1 and nothing can connect remotely.

## First-run verification (10 min)

1. `docker logs`: entrypoint prints `LCM multicast ... routed via lo`; the
   connection prints its per-stream stats every 10 s — every subscribed
   stream should show healthy `received/decoded` counts.
2. `dmesg -Tw | grep -E "smmu|camrtc|VPR"` on the robot host — must stay
   quiet (T1 of [NEWGEN_DROP_ELIMINATION.md](../NEWGEN_DROP_ELIMINATION.md)).
3. `tegrastats` for a few minutes — CPU headroom and temps under full load.
   The decode→re-encode color path is the known CPU hog; if the Orin is
   pegged, the Tier-2 JPEG passthrough (STREAM_RELIABILITY remediation) is
   the next lever.
4. Soak ≥30 min (T5): all streams at rate, no counter climbing.

## Knobs

Everything in `GlobalConfig` is a plain env var (pydantic-settings):
`docker run -e RERUN_OPEN=web ...` etc. Defaults baked into the image:
`ROS_DOMAIN_ID=1` (new-gen V2.3.0 — override `-e ROS_DOMAIN_ID=41` for the
old-gen robot; it MUST match the robot or the container discovers zero
topics), `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `RERUN_OPEN=none`,
`RERUN_HOST=0.0.0.0`, `N_WORKERS=3`,
`LCM_DEFAULT_URL=udpm://239.255.76.67:7300?ttl=0` (off the DDS port range —
see Known traps).

## Known traps (all pre-wired here, listed so they stay known)

- **LCM port 7667 collides with FastDDS on `ROS_DOMAIN_ID=1`.** DDS derives
  its ports from the domain (`7400 + 250*domain + offset`); domain 1 (base
  7650) hands out `7661/7663/7665/7667/...` to participants, and LCM's default
  is `udpm://239.255.76.67:7667`. Under `--network=host` a robot DDS node
  (seen: `signal_camera_node`) binds 7667 first, so every dimos worker dies
  with `bind: Address already in use` → `ExceptionGroup: safe_thread_map
  failed`. Pre-wired fix: the image sets `LCM_DEFAULT_URL=udpm://239.255.76.67:7300`
  (below the DDS base 7400). If you change the multicast **group**, update the
  entrypoint's loopback route to match. Diagnose a recurrence with
  `sudo ss -aunp | grep 7667`. (Old-gen domain 41 put DDS at ~17650 — no clash.)
- **`matplotlib` is a hidden dep of `RerunBridgeModule`** (depth/pointcloud
  colormaps) but pyproject files it under the `manipulation` extra, not
  `visualization` — so a base-deps install misses it and the bridge's
  `start()` dies with `No module named 'matplotlib'`. The Dockerfile installs
  it explicitly. (Upstream fix: move it into the `visualization` extra.)
- **`docker exec` shells don't source ROS.** The ENTRYPOINT sources
  `/opt/ros/humble/setup.bash` for PID 1 only; an `exec -it ... bash` shell
  starts fresh, so a hand-run `dimos run` can't import rclpy. The image sources
  ROS from `/root/.bashrc` to cover this.
- `libturbojpeg` (apt) is a hidden native dep of `JpegLcmTransport` — its
  absence presents as "decode error" on every color stream.
- LCM needs multicast routed via loopback; the entrypoint does it and needs
  `--cap-add NET_ADMIN`. Only the LCM group (`239.255.76.67/32`) is routed —
  a blanket `224.0.0.0/4 → lo` would break DDS discovery to a tethered PC.
- If the robot's `net.core.rmem_max/wmem_max` (40 MB) were set manually they
  do NOT survive reboot — persist via `/etc/sysctl.d/` on the host. Matters
  less on-robot (loopback), but HDAS itself still uses UDP.
