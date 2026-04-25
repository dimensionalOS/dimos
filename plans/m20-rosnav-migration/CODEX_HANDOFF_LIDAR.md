# Handoff: M20 lidar PC2 not flowing post-OTA

## Status update — resolved 2026-04-25

The lidar blocker described below is resolved in the current deployed
`drdds_recv` binary. The working solution bypasses libdrdds
`DrDDSChannel` for sensor receive and uses a forked raw FastDDS child
for all five sensor topics:

- `rt/LIDAR/POINTS`
- `rt/LIDAR/POINTS2`
- `rt/IMU`
- `rt/LIDAR/IMU201`
- `rt/LIDAR/IMU202`

Verified on NOS for >60 seconds:

```
[drdds_recv] status: lidar_front_matched=2 lidar_front_msgs=241 lidar_rear_matched=1 lidar_rear_msgs=240
[drdds_recv] imu_status: imu_yesense_matched=1 imu_yesense_msgs=4665 imu_airy_front_matched=3 imu_airy_front_msgs=9375 imu_airy_rear_matched=3 imu_airy_rear_msgs=9375
...
[drdds_recv] status: lidar_front_matched=2 lidar_front_msgs=941 lidar_rear_matched=1 lidar_rear_msgs=940
[drdds_recv] imu_status: imu_yesense_matched=1 imu_yesense_msgs=18666 imu_airy_front_matched=3 imu_airy_front_msgs=37377 imu_airy_rear_matched=3 imu_airy_rear_msgs=37377
```

Services active during verification: `drdds-recv`, `rsdriver`,
`yesense`, `charge_manager`, `reflective_column`, `localization`.

Post-cleanup verification on the rebuilt binary with the old diagnostic
`DRDDS_RECV_DRDDS_PC2` fallback removed:

- NOS build: `make drdds_recv -B` passes; only the pre-existing
  `shm_transport.h` `ftruncate` warning remains.
- Fresh bridge watch after install/restart:
  `lidar_front_msgs=14 -> 664`, `lidar_rear_msgs=8 -> 658`,
  `imu_yesense_msgs=350 -> 13351`, Airy IMUs
  `1345 -> 27345` over the same run.
- Dock-safe smartnav run with `M20_NAV_ENABLED=0` and
  `M20_SKIP_STAND=1` consumed PC2 frames in FastLIO:
  `frame #0..#29 path=pc2` immediately after stationary preroll,
  `DrddsLidarBridge` reached `lidar #301`, and `nav_cmd_pub`
  published zero commands only.
- After the timed smartnav run, no `m20_smartnav_native`,
  `fastlio2_native`, `drdds_lidar_bridge`, `nav_cmd_pub`, or
  `airy_imu_bridge` processes were left running; all base services
  remained active.

The historical notes below remain useful for context, but the ranked
open hypotheses and quick-win actions are superseded by the raw
FastDDS all-sensor receive path.

## Mission

Make `/LIDAR/POINTS` and `/LIDAR/POINTS2` PointCloud2 messages flow from `rsdriver` → our `drdds_recv` subscriber on the M20 robot's NOS board, post system-software upgrade to V1.1.8.5. IMU paths already work; lidar is the last blocker for SLAM.

Definition of done: `/var/log/drdds_recv.log` shows `lidar_front_msgs` incrementing at ~10 Hz steady-state alongside `lidar_front_matched >= 1`, and the same for `lidar_rear_*`. After that, smartnav can be started and FAST-LIO2 will consume the data.

## Access

- Mac → NOS over Tailscale: `ssh -o ProxyJump=user@m20-770-gogo user@10.21.31.106`
- `sudo` password is a single `'` (literal single quote)
- Source repo on Mac: `/Users/afik_cohen/gt/dimos/crew/ace`
- Source repo on NOS: `/var/opt/robot/data/dimos`
- `scp` works through the same ProxyJump

Branch: `feat/deeprobotics-m20-nav`. Last two relevant commits:
- `7e7cbd51c` — Init overload fix (2-arg `Init(int, string)` like rslidar, instead of 6-arg multi-domain)
- `7fa66fc42` — Init network_name fix (`"eth0/eth1"` to bind both interfaces, matches rslidar's literal arg)

## What works (don't regress)

After committed fixes, IMU is fully functional:

```
[drdds_recv] status:
  imu_yesense_matched=1     imu_yesense_msgs=~200 Hz
  imu_airy_front_matched=2  imu_airy_front_msgs=~200 Hz
  imu_airy_rear_matched=2   imu_airy_rear_msgs=~200 Hz
```

`yesense` publishes on topic `/IMU` (sensor_msgs::msg::Imu, ~88 bytes per message). Airy IMUs publish on `/LIDAR/IMU201` and `/LIDAR/IMU202` (same `Imu` type). All three match and flow at ~200 Hz steady-state.

Files that drive this success path:
- `dimos/robot/deeprobotics/m20/drdds_bridge/cpp/drdds_recv.cpp` — the subscriber binary source
- `/etc/systemd/system/drdds-recv.service` — starts the binary, ordered before rsdriver
- `/etc/systemd/system/rsdriver.service.d/10-drdds-order.conf` — `After=drdds-recv.service Requires=drdds-recv.service`
- `/etc/systemd/system/m20-nav-net.service` — runs `ip route add 224.0.0.0/4 dev lo` (multicast → loopback for cross-board isolation; harmless on single-host)

## What's broken

LIDAR PC2 channels are intermittently matched but data never flows:

```
[drdds_recv] status:
  lidar_front_matched=1 (or 2)  lidar_front_msgs=0   ← NEVER increments
  lidar_rear_matched=0 (or 1)   lidar_rear_msgs=0    ← NEVER increments
```

Topics are `/LIDAR/POINTS` (front, sensor_msgs::msg::PointCloud2, ~1 MB per scan, 10 Hz) and `/LIDAR/POINTS2` (rear, same type/rate). rsdriver journal shows it actively publishes at 10 Hz with `error_code: 0` and `send success size: 25K-43K` (per-fragment).

## Repro

1. Confirm services up:
   ```bash
   ssh ... 'systemctl is-active drdds-recv rsdriver yesense'
   # all should be "active"
   ```
2. Check status log:
   ```bash
   ssh ... 'strings /var/log/drdds_recv.log | grep "drdds_recv\] status" | tail -3'
   ```
3. Confirm rsdriver is publishing:
   ```bash
   ssh ... 'journalctl -u rsdriver --since "10 sec ago" --no-pager | grep "send success" | tail -5'
   ```

If you need to restart everything cleanly (Finding #5 ordering):
```bash
ssh ... 'sudo bash -c "systemctl stop rsdriver drdds-recv yesense; killall -9 yesense_node rslidar 2>/dev/null; sleep 2; rm -f /dev/shm/drdds_bridge_* /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* /dev/shm/sem.fastrtps_*; systemctl start drdds-recv; sleep 3; systemctl start rsdriver; sleep 35; systemctl start yesense; sleep 10"'
```

Build + redeploy after edits:
```bash
scp -o ProxyJump=user@m20-770-gogo PATH/drdds_recv.cpp user@10.21.31.106:/var/opt/robot/data/dimos/dimos/robot/deeprobotics/m20/drdds_bridge/cpp/drdds_recv.cpp
ssh ... 'cd /var/opt/robot/data/dimos/dimos/robot/deeprobotics/m20/drdds_bridge/cpp/build && make drdds_recv -B && sudo cp drdds_recv /opt/drdds_bridge/lib/drdds_bridge/'
```

## What's already known (do not re-investigate)

### Eliminated hypotheses

1. **Callback backpressure on slow `on_lidar`** — stubbed `on_lidar` to a bare `lidar_count.fetch_add(1) + std::cout`, no SHM acquire, no memcpy. Still `msgs=0`. Data does not reach the callback at all.

2. **Data Sharing (zero-copy SHM) desync** — `grep -c datasharing /proc/<pid>/maps` returns 0 for both `rslidar` and our `drdds_recv`. Stale `/dev/shm/fast_datasharing_*` files exist but are leftovers — neither active process maps them. Data Sharing is not in use on this system.

3. **System-wide PC2 broken on V1.1.8.5** — `handler` reports `Cloud=0 Hz` in `/var/opt/robot/log/2026_*/handler.*.log` going back to 2026-04-20 (well before the 2026-04-23 OTA). `handler` simply doesn't subscribe to PC2; was never a regression signal.

4. **Topic name / prefix mismatch** — verified rslidar binary publishes on `/LIDAR/POINTS` (hardcoded in binary `strings`), not the config-driven `/lidar_points` (that field is dead config). Our default `topic_prefix="rt"` in DrDDSChannel matches rslidar's default. IMU works on the same prefix convention.

5. **Multicast → loopback route** — `m20-nav-net.service` adds `ip route add 224.0.0.0/4 dev lo`. Removed it temporarily; route became `dev eth1`; lidar still `msgs=0`. Restored.

6. **`use_shm` parameter on DrDDSChannel** — tried 5-arg ctor with `(use_shm=true, "rt")` on lidar only: got initial ~400 msg burst then completely stalled. With `use_shm=true` on all channels: even IMU broke. Default `use_shm=false` is what works for IMU; lidar broken in either mode.

7. **6-arg `DrDDSManager::Init` with `local_host=true`** — caused all channels to drop to `matched=0`. The 6-arg variant creates `mMultiParticipant_` which is a different participant from the `mLocalParticipant_` that rsdriver/localization/charge_manager use (Finding #3). Breaks all matching.

### Currently committed code state

`drdds_recv.cpp` has these critical lines (after `7fa66fc42`):

```cpp
// 2-arg Init with "eth0/eth1" matches rslidar exactly (verified via objdump on rslidar's
// rodata at offset 0xd2490 — literal string "eth0/eth1")
DrDDSManager::Init(0, "eth0/eth1");

// 3-arg DrDDSChannel — defaults (use_shm=false, topic_prefix="rt") matched pre-OTA
DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType> lidar_ch(
    on_lidar, "/LIDAR/POINTS", 0);
DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType> lidar2_ch(
    on_lidar2, "/LIDAR/POINTS2", 0);
DrDDSChannel<sensor_msgs::msg::ImuPubSubType> imu_ch(
    on_imu, "/IMU", 0);
DrDDSChannel<sensor_msgs::msg::ImuPubSubType> imu_airy_front_ch(
    on_imu_airy_front, "/LIDAR/IMU201", 0);
DrDDSChannel<sensor_msgs::msg::ImuPubSubType> imu_airy_rear_ch(
    on_imu_airy_rear, "/LIDAR/IMU202", 0);
```

DrDDSChannel ctor signature in `/usr/local/include/drdds/core/drdds_core.h`:
```cpp
DrDDSChannel(const CallBack &handler, std::string topic_name = "default",
             int domain_id = 0, bool use_shm = false, std::string topic_prefix = "rt");
```

`DrDDSManager::Init` overloads in `/usr/local/include/drdds/core/drdds_manager.h`:
```cpp
static void Init(std::vector<int> domain_id, std::string module_id, std::string node_name,
                 bool local_host = false, bool enable_config = false, bool enable_discovery = false);
static void Init(int domain_id, const std::string network_name = "");
```

## Key empirical observation

Three working PC2 consumers on the system map rsdriver's SHM port file `/dev/shm/fastrtps_port7419` directly **without having their own participant port file**:

```
$ sudo lsof /dev/shm/fastrtps_port7419
COMMAND       PID
rslidar     217980  ← owner
localization_dd 2986  ← MAPS rslidar's port, NO own port file
charge_manager  2837  ← MAPS rslidar's port, NO own port file
pcl_pass_grid   3121  ← MAPS rslidar's port, NO own port file
```

vs. our `drdds_recv`:
```
/dev/shm/fastrtps_port7411  ← OUR own port file (separate participant)
```

`rslidar` does NOT map our `port7411`. So the SHM-write-from-publisher-into-subscriber-port path between rslidar and us is not happening, even though discovery succeeds (`matched=1` or `2`).

These working consumers (`localization_dd`, `charge_manager`, `pcl_pass_grid`) all use the same `DrDDSManager::Init(int, std::string)` signature as us (verified via `nm -D`). They have identical loaded libraries (`libdrdds.so.1.1.7`, `libfastrtps.so.2.14.2`, `libfastcdr.so.2.2.5`) and identical environment (no `FASTRTPS_DEFAULT_PROFILES_FILE`). They're systemd services with similar unit files.

**The mechanism by which they attach directly to rsdriver's port 7419 is unknown.** This is the central mystery.

`localization_ddsnode` ELF shows it does call `DrDDSManager::Init(0, <string>)` once at startup (verified via `objdump -d /opt/robot/share/localization/bin/localization_ddsnode`). What string it passes was not extracted (couldn't trace the x19 register backwards on aarch64). Worth verifying — it might be a different network_name string than `"eth0/eth1"`.

## Open hypotheses (ranked)

1. **Hidden initialization sequence** — `localization`, `charge_manager`, `pcl_pass` may make additional libdrdds calls beyond `Init` + `DrDDSChannel` ctor that put their participant in "co-located reader" mode. Find this call by disassembling one of those binaries (probably `localization_ddsnode` since it's smaller) around the `DrDDSManager::Init` call site, and looking for any other libdrdds symbol references.

2. **Init network_name semantics** — even though we pass `"eth0/eth1"` exactly like rslidar, `localization` may pass something different that triggers a different behavior. Need to extract that string from `localization_ddsnode` rodata.

3. **Participant resource limits / FastDDS QoS deadline** — the active drqos.xml at `/opt/drdds/dr_qos/drqos.xml` declares a 50 ms deadline on PointCloud2 reader+writer with RELIABLE reliability. Maybe a fragmented 1MB UDP transmission from rslidar to our process violates the deadline and FastDDS silently drops the writer-reader binding. Try editing drqos.xml to BEST_EFFORT or remove the deadline for PointCloud2; restart and observe.

4. **Boot-time interface race** — at first boot, `drdds-recv` started 09:23:23 but only bound `127.0.0.1:7400` (no eth0/eth1) — same network_name `"eth0/eth1"` but interfaces weren't yet up. Manually restarting `drdds-recv` afterwards picked up eth0/eth1. We should add a `systemd-networkd-wait-online`-style guard to drdds-recv.service. Independent of the lidar bug but worth fixing as a hygiene step.

## Things to try

### Quick wins (~30 min each)

A. **Extract `localization_ddsnode`'s `Init` second arg.** Use `objdump -d /opt/robot/share/localization/bin/localization_ddsnode` and follow the disassembly backward from the call site to find the rodata string loaded into `x1`. The Init call site is at `0x95968`. Compare to rslidar (`"eth0/eth1"`).

B. **Change drqos.xml** — back up `/opt/drdds/dr_qos/drqos.xml`, then edit the `data_writer` and `data_reader` sections for `sensor_msgs::msg::dds_::PointCloud2_`:
- Drop the `<deadline>` block
- Change `<reliability><kind>RELIABLE</kind></reliability>` to `BEST_EFFORT`
Restart all DDS services with the standard dance, observe.

C. **`ExecStartPre` for boot-time interface race** — add to `/etc/systemd/system/drdds-recv.service`:
```
ExecStartPre=/bin/sh -c 'until ip -br addr show eth0 | grep -q UP && ip -br addr show eth1 | grep -q UP; do sleep 1; done'
```
Won't fix the lidar problem but reduces noise on next boot.

D. **Trace what library calls localization makes during init.** `strace -f -e trace=openat,mmap,bind /opt/robot/share/localization/bin/localization_ddsnode 2>&1 | head -200` will show the syscall sequence and SHM segment opens during the first second. Compare to drdds_recv's strace. Look for differences in which fastrtps_port files get opened.

### Deeper dive (multi-hour)

E. **Disassemble libLocalization.so or localization_ddsnode** with Ghidra/Cutter/IDA, find the function that calls `DrDDSChannel<PointCloud2PubSubType>::DrDDSChannel(...)`. Check if there are any other libdrdds API calls between Init and DrDDSChannel construction. Specifically look for symbols like `DrDDSManager::AttachLocal*`, `DrDDSManager::JoinChannel*`, `DrDDSPublisher::Attach*`, anything that suggests joining an existing participant's pub. List all libdrdds symbol references in localization_ddsnode via `nm -D --undefined-only $(readlink -f /proc/$(pgrep localizat)/exe) | c++filt | grep DrDDS`.

F. **Direct SHM read** — bypass FastDDS entirely. `mmap` `/dev/shm/fastrtps_port7419` directly and parse FastDDS internal SHM frame format. This is what `localization` etc seem to be effectively doing (they map the port but don't have FastDDS subscribers). FastDDS source is open (`https://github.com/eProsima/Fast-DDS` v2.14.2 — that's the version on disk per `ls -la /usr/local/lib/libfastrtps.so.2.14.2`). Reverse-engineer the SHM ring buffer format. Risky but might be the cleanest path if the libdrdds API is genuinely undocumented.

G. **Reach out to DeepRobotics' Jeff** for the missing API. He's on-site (Dimensional office). Question: "Three of your shipping NOS services receive PointCloud2 from rsdriver via a SHM mechanism that maps rsdriver's port file directly. They use `DrDDSManager::Init(int, std::string)` like a normal subscriber, but they don't appear to create a separate FastDDS participant. What's the libdrdds API to enable this co-located reader mode? Or is there an extra call we need to make beyond `DrDDSManager::Init` + `DrDDSChannel<T>` to attach to an existing publisher's SHM?" — see also `plans/m20-rosnav-migration/FASTLIO2_LOG.md` Finding #36 for full context.

## Useful diagnostic commands

```bash
# Status of all DDS-related processes and their SHM/socket state
ssh ... 'echo "=== SHM port owners ==="; \
  for f in /dev/shm/fastrtps_port*; do \
    echo -n "$f -> "; \
    sudo lsof "$f" 2>/dev/null | tail -n +2 | awk "{print \$2,\$1}" | sort -u | tr "\n" ","; \
    echo; \
  done; \
  echo; echo "=== drdds_recv UDP binds ==="; \
  sudo ss -ntulp | grep drdds_recv'

# Verify our binary actually contains the eth0/eth1 fix (matches the deployed location)
ssh ... 'strings /opt/drdds_bridge/lib/drdds_bridge/drdds_recv | grep eth0/eth1'
# expect: "eth0/eth1"

# Inspect rsdriver and localization Init call args via disassembly
ssh ... 'sudo objdump -d /opt/robot/share/node_driver/bin/rslidar | grep -B 8 "DrDDSManager4InitEi" | head -30'
ssh ... 'sudo objdump -d /opt/robot/share/localization/bin/localization_ddsnode | grep -B 8 "DrDDSManager4InitEi" | head -30'

# All DrDDS symbols used by a binary
ssh ... 'nm -D --undefined-only /opt/robot/share/localization/bin/localization_ddsnode | c++filt | grep DrDDS'
```

## Background reading

In the repo at `plans/m20-rosnav-migration/`:
- `FASTLIO2_LOG.md` — Finding #36 has the full saga of the post-OTA breakage and recovery so far. The "Resolution of matched=0" subsection covers what we already learned.
- `ROSNAV_MIGRATION_LOG.md` — Findings #1-#5 cover original DDS discovery investigation. **Finding #5 ("SHM Discovery Startup Ordering")** is critical — `drdds_recv` MUST start before `rsdriver`, enforced by the systemd dropin. `matched > 0 but msgs == 0` was the pre-OTA symptom of wrong ordering. Post-OTA we've verified ordering is correct but the symptom persists for lidar specifically.
- `NATIVE_NAV_LOG.md` — Finding #4 covers the `rt/` topic prefix convention. Relevant for understanding why DrDDSChannel's default `topic_prefix="rt"` works.
- `UPGRADE_TO_V1.1.8.5.md` — pre-OTA upgrade plan with backup details.

## Constraints

- Do NOT compile anything on NOS that takes more than ~30 seconds — it's an aarch64 board with limited CPU. Compile native dimos code on the Mac and `scp` to NOS, OR build in the existing CMake build dir on NOS (it's already configured, just `make drdds_recv -B`).
- Do NOT run `nix-collect-garbage` — it will wipe pinned SLAM module builds (kernel 5.10 workaround flakes).
- Do NOT modify `rsdriver_config.yaml` — it was carefully restored from backup post-OTA. The active config has `send_separately: true` (we want dual-channel front+rear).
- You have sudo on NOS (password is single quote). Use it sparingly and don't run anything destructive without confirmation from the user.
- Avoid running on-NOS `docker run` / `docker build` — that crashes the OOM-prone NOS.

If you reach a state where lidar `msgs > 0` and growing, that's the win. Verify it persists for at least 60 seconds and survives a smartnav startup cycle, then update FASTLIO2_LOG.md with a "Resolution of lidar msgs=0" section and commit.
