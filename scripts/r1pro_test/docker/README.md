# On-robot dimos container (R1 Pro)

Runs `dimos run r1pro-coordinator` on the robot itself. No bulk sensor data
crosses a physical link — the entire failure class documented in
[STREAM_RELIABILITY_DIAGNOSTIC.md](../STREAM_RELIABILITY_DIAGNOSTIC.md)
(qdisc tail-drops, send-buffer overflow, fragmented UDP over the wire/WiFi)
is structurally gone. The laptop's only job is viewing: a rerun viewer
connected over WiFi to the bridge's gRPC server, which carries JPEG-sized
frames after the `JpegLcmTransport` work.

HDAS (the vendor stack) stays on the host as-is; dimos talks to it over
local DDS (`ROS_DOMAIN_ID=41`, FastDDS, loopback/SHM).

## Build

On the robot (simplest, no qemu):

```bash
git clone <repo> && cd dimos      # or rsync your checkout
scripts/r1pro_test/docker/build.sh
```

Or cross-build from the laptop and ship it:

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64   # once
ROBOT=user@192.168.123.150 scripts/r1pro_test/docker/build.sh --cross
```

## Run (on the robot)

```bash
scripts/r1pro_test/docker/run.sh
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
rerun --connect rerun+http://<robot-ip>:9876/proxy
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
`ROS_DOMAIN_ID=41`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `RERUN_OPEN=none`,
`RERUN_HOST=0.0.0.0`, `N_WORKERS=3`.

## Known traps (all pre-wired here, listed so they stay known)

- `libturbojpeg` (apt) is a hidden native dep of `JpegLcmTransport` — its
  absence presents as "decode error" on every color stream.
- LCM needs multicast routed via loopback; the entrypoint does it and needs
  `--cap-add NET_ADMIN`. Only the LCM group (`239.255.76.67/32`) is routed —
  a blanket `224.0.0.0/4 → lo` would break DDS discovery to a tethered PC.
- If the robot's `net.core.rmem_max/wmem_max` (40 MB) were set manually they
  do NOT survive reboot — persist via `/etc/sysctl.d/` on the host. Matters
  less on-robot (loopback), but HDAS itself still uses UDP.
