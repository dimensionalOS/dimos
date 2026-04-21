# M20 FAST-LIO2 Native Port Log

**Goal**: Get FAST-LIO2 running as a dimos NativeModule with LCM input (instead of Livox SDK) on M20 NOS, as the click-to-goal unblocker while Option C (full 4-node ARISE native port) runs in parallel.

**Date started**: 2026-04-17
**Branch**: `integration/m20-rosnav-migration`
**Bead**: `di-857dn`

---

## Context

Native ARISE SLAM (single-pass laser-mapping port at `dimos/navigation/smart_nav/modules/arise_slam/`) is broken for indoor yaw tracking — `useIMU=true` is vestigial, no IMU preintegration linked. Measured: 352° physical rotation → 6.6° reported odometry yaw. Click-to-goal blocked.

FAST-LIO2 has tight IMU coupling baked in by design. OG FAST-LIO (1.x) already worked with this exact hardware in the Docker era (ROSNAV_MIGRATION_LOG Findings #10–#12: 10Hz odometry, zero drops on RSAIRY). Goal here: bring FAST-LIO2 up natively with LCM input, re-using Jeff's `FAST-LIO-NON-ROS/dimos-integration` fork that already has the LIO core + PCL + Eigen + Boost dependencies wired up in nix.

Parallel path (di-ony5x) ports the full 4-node ARISE stack as the architecturally-correct multi-floor-capable fix. FAST-LIO2 is the tactical unblocker.

---

## Finding #1: Existing `FastLio2` NativeModule is Livox-Only (2026-04-17)

Starting recon revealed `dimos/hardware/sensors/lidar/fastlio2/cpp/main.cpp` is already ~800 lines of working FAST-LIO2 wrapper — but the input layer binds directly to Livox SDK:

- `SetLivoxLidarPointCloudCallBack(on_point_cloud, ...)` feeds raw SDK packets
- `SetLivoxLidarImuDataCallback(on_imu_data, ...)` feeds raw SDK IMU
- Livox-specific accumulation: per-packet points buffered and emitted as a frame every `1/frequency` seconds
- Config: Livox IP/ports hardcoded via `host_ip`, `lidar_ip` + SDK port struct

Everything downstream (FAST-LIO core, LCM publishing of `registered_scan` + `odometry`, init_pose transform, frame IDs) is already plumbed. Only the INPUT layer needs to change.

**Plan**: add `--input_mode lcm` that bypasses the Livox SDK and instead subscribes to LCM `raw_points` + `imu` from the existing `DrddsLidarBridge`. Reuse everything else.

---

## Finding #2: nix Build on Kernel 5.10 — fchmodat2 Strikes Again (2026-04-17/18)

The same kernel 5.10 / glibc 2.42 fchmodat2 pain from the drdds_bridge + ARISE builds resurfaced with extra teeth:

1. **`fetchFromGitHub` dies on its own internal chmod** — my unpackPhase override only affects the outer derivation; the source derivation runs INSIDE nix's sandbox with nix's coreutils and fails. Fix: swap to `pkgs.fetchurl` (download-only tarball, no internal chmod).
2. **`/usr/bin/cp` + `/usr/bin/chmod` are not in the sandbox on NOS.** drdds_bridge's builds used these and worked — they were just cached from before this became a problem. Fix: use nix's `cp` with `--no-preserve=mode`; skip the chmod pass entirely with `dontFixup = true` (tarball perms are already usable).
3. **Flake inputs `path:../../livox/cpp` rejected as "mutable lock"** by nix ≥ 2.20. Fix: inline the `livox-sdk2` derivation directly into the fastlio2 flake (makes it self-contained, no sibling flake input).
4. **`livox-common` headers** (`dimos_native_module.hpp`, `livox_sdk_config.hpp`) live at `../../common/` — escape the copied store path under `path:.` URL. Fix: vendor a local copy into `dimos/hardware/sensors/lidar/fastlio2/cpp/livox_common_vendor/` and fall back to it in CMakeLists.txt when `LIVOX_COMMON_DIR` isn't set externally.
5. **nix git flake only sees committed files.** All my in-flight edits had to be `git add && git commit`ed on NOS (local-only commit, never pushed) for `git+file://.../dimos?dir=...&ref=HEAD` to pick them up.
6. **Stale `flake.lock` on Mac kept re-syncing to NOS** even after I deleted it there — needed to delete on Mac then re-sync.

Net: ~2 hours of nix/build-system yak shaving before any actual code ran.

---

## Finding #3: First Binary Runs, Input Layer Ready (2026-04-17 late)

After the nix fixes, `fastlio2_native --input_mode lcm --raw_points ... --imu ... --registered_scan ... --odometry ... --config_path mid360.yaml` starts cleanly:

```
[fastlio2] Starting FAST-LIO2 native module (input_mode=lcm)
[fastlio2] raw_points topic: /raw_points#sensor_msgs.PointCloud2
[fastlio2] imu topic: /imu#sensor_msgs.Imu
[fastlio2] Initializing FAST-LIO...
[fastlio2] FAST-LIO initialized.
[fastlio2] LCM subscribers active, waiting for data...
```

Full M20 blueprint (`m20_smartnav_native.py`) updated with `M20_SLAM_BACKEND` env var: `arise` (default, preserves Bex's parallel work) or `fastlio2`. No rip-out of the existing AriseSLAM path.

---

## Finding #4: FAST-LIO2's EKF Diverges — First Measurement (2026-04-17 night)

Initial on-hardware test, M20 stationary:

- Odometry rate: 0.07 Hz (should be ~10 Hz)
- 15s later: position reported at **(−9200, −9465, −3039)** meters — complete EKF blowup
- Viewer shows scans aligned (room stays pinned visually)
- IMU sensor data is clean: accel magnitude 9.84 m/s², gyro ≈ 0 rad/s, orientation quat normalized

Combination of "visually aligned scans + wildly wrong pose" is the fingerprint of a broken EKF. Time to dig.

---

## Finding #5: drdds Bridge Emits Duplicate + Out-of-Order Scans (2026-04-17 night)

Instrumented the LCM callback to log `timebase`, `max_offset`, `imu_latest`, point counts, per-frame. First 5 frames reveal:

```
frame #0  timebase=474.399987  max_offset=0.100s  imu_vs_frame_end=+0.321s  npts=81753
frame #1  timebase=474.399987  ← identical to #0!                         npts=81753
frame #2  timebase=474.499876  (normal 100ms increment)                   npts=78988
frame #3  timebase=472.899996  ← 1.6s BACKWARDS!                          npts=81742
frame #4  timebase=474.599999                                             npts=82270
```

The drdds bridge occasionally re-emits the same scan (identical stamp) or delivers a scan from the past (RSAIRY dual-lidar front/back merge artifacts). FAST-LIO's EKF seeing a scan from 1.6 seconds ago with newer IMU data produces instant divergence.

**Fix applied**: monotonic guard on `pc->header.stamp` in the LCM callback. Drop duplicates and anything older than last-accepted. Reduced drift from 9km/15s to 73m/20s (100× better but still wrong).

---

## Finding #6: `cp.line` Ring Pass-Through Starves Livox Preprocess (2026-04-18, codex review #1)

My handler was passing RSAIRY's ring index (0–63 after drdds bridge remap) into `CustomPoint.line`, thinking that was the "right" thing. But FAST-LIO-NON-ROS's Livox preprocess (`utils.cpp:12–26`) filters points by low line IDs — passing `line = 63` caused those points to be dropped entirely, starving the EKF of measurements.

**Fix**: force `cp.line = 0` like the Livox-native handler does. Livox Mid-360 is non-repetitive and reports a single "line" anyway.

---

## Finding #7: Clock-Domain Mismatch between Lidar and IMU Streams (2026-04-18, codex review #1–#2)

Codex identified the #1 remaining suspect: FAST-LIO2's `syncPackage()` compares lidar `header.stamp` directly against IMU `header.stamp`, assuming they share a clock domain. In our bridge:
- Lidar `header.stamp`: drdds sensor-hardware time, ~0.8s behind wall clock
- IMU `header.stamp`: yesense wall-clock time, ~0.225s behind wall clock

Comparing across two 0.6-second-apart clock domains in an EKF's IMU-preintegration gate produces catastrophic mis-alignment of IMU data with lidar scans.

**Fix applied**: at LCM arrival time, **wall-clock-stamp both streams** (`std::chrono::system_clock::now()`) before feeding FAST-LIO. Per-point `offset_time` stays relative; only the anchor shifts.

Produced ambiguous result at first — new diagnostic showed `imu_vs_frame_end` negative (lidar extended past IMU). Turned out I was stamping lidar at "wall_now", so scan_end = wall_now + max_offset, meaning IMU had to reach into the future. Corrected in Finding #8.

---

## Finding #8: Anchor `frame_ts` at `imu_latest - max_offset_s` (2026-04-18)

To make `scan_end = imu_latest` (FAST-LIO's syncPackage gate satisfied by construction), anchor the lidar scan's start time at `imu_latest - max_offset_s`. Produces `imu_vs_frame_end = +0.000s` consistently. First order of business: FAST-LIO stopped stalling on sync.

---

## Finding #9: Variable `max_offset_s` Causes Rewritten-Stamp Regression (2026-04-18, codex review #3)

Scans from drdds bridge alternate between ~100ms span (single lidar emission) and ~200ms span (both lidars merged). When anchoring at `imu_latest - max_offset`, a 200ms scan arriving right after a 100ms scan will have `frame_ts` **100ms EARLIER** than the previous one — even though the source `sensor_ts` was monotonically newer.

Example from logs:
```
frame #3  max_offset=0.100s  timebase=605.163
frame #4  max_offset=0.200s  timebase=605.063  ← 100ms back in time
```

Codex confirmed this signature by reading `syncPackage()` source: non-monotonic rewritten stamps silently corrupt EKF state.

**Fix applied**: second monotonic guard on the REWRITTEN `frame_ts` (separate from the source `sensor_ts` guard). Drops any scan whose rewritten stamp ≤ last accepted.

After this fix: pose[0] went from (−280, −119, −60)m → **(0.4, 32.8, −0.4)m** — initialization is now clean. FAST-LIO's IMU_init captures a proper stationary reference.

---

## Finding #10: FAST-LIO-NON-ROS Has No `imu_init_num` YAML Knob (2026-04-19)

Codex's H1 hypothesis: FAST-LIO's default 20-sample (100ms @ 200Hz) IMU_init window catches M20 servo flutter during robot startup, poisoning gravity/bias estimates permanently.

Tried adding `imu_init_num: 400` to `mid360.yaml`. Binary strings dump:
```
strings fastlio2_native | grep -iE 'imu_init|init_num|init_sample'
# (no hits)
```

The leshy/FAST-LIO-NON-ROS/dimos-integration fork doesn't expose this as a YAML key — hardcoded to ~20 samples internally. YAML change is a no-op.

**Fix applied instead**: stationary preroll gate in our C++ wrapper. Buffer IMU samples (and drop lidar) until `PREROLL_STATIONARY_S` (2.0s) of IMU shows gyro < 0.02 rad/s AND |accel| within 0.5 m/s² of 9.81. Only then flip `g_seeded` and start feeding lidar to FAST-LIO. This ensures FAST-LIO's own 20-sample init runs on guaranteed-stationary data.

Logs confirm it fires:
```
[fastlio2] first IMU sample received (ts=1776563084.141) — starting preroll
[fastlio2] stationary preroll complete after 2.00s — releasing lidar scans to FAST-LIO
```

After this fix: continuous exponential drift signature replaced by **step-jumps between stable plateaus** — the IMU bias problem is fixed. What's left is scan-matching error.

---

## Finding #11: Monotonic Guard Too Strict → Starves Data (2026-04-19)

Early strict guard dropped scans where `sensor_ts <= last_accepted_ts`. In practice, drdds emits many scans with `sensor_ts` just slightly below the latest accepted (<100ms back), which are still valid lidar data. Dropping them all left FAST-LIO getting only 5 scans per 15s — EKF could only do IMU-only prediction between them, which drifts.

**Fix**: relax to drop ONLY exact duplicates (`sensor_ts == last_accepted_ts`). The rewritten-stamp guard from Finding #9 is the strict one — it operates on `frame_ts` in FAST-LIO's domain, which we control. Source `sensor_ts` is only used for dedup.

Effect: accepted scan rate went from 5/15s → ~15/15s (much more data for FAST-LIO).

---

## Current Status (2026-04-19)

**Indoor stationary 15-second benchmark (M20 still, on carpet):**

| Metric | Before fixes | Current |
|---|---|---|
| EKF divergence pattern | exponential (9km in 15s) | **step-jumps between plateaus** |
| pose[0] (first publish) | (−280, −119, −60) m | **(+0.4, +32.8, −0.4) m** |
| 15s drift | ~1200 m | **~82 m (y-axis dominant)** |
| Odometry rate | 0.07 Hz | **1.9 Hz** |
| imu_vs_frame_end | +0.43s → -0.24s (chaos) | **+0.000s (bracketed)** |
| Scans dropped | >95% | ~30% |

**Visual (dimos-viewer):** registered_scan points stay pinned to the room during small motions. Scan-match ICP is broadly working; occasional big corrections produce the step-jumps.

**Summary of behavior change:** from "EKF totally broken" to "EKF sometimes wrong" — orders of magnitude improvement but not yet production-grade.

## Remaining Hypotheses

1. **ICP finding wrong local minima** in symmetric indoor scenes (same problem native ARISE has — insufficient yaw discrimination from single-bin features)
2. **IMU extrinsic wrong** — currently `extrinsic_T: [-0.011, -0.02329, 0.04412]` in mid360.yaml, which is Livox Mid-360-specific. M20 RSAIRY + yesense IMU have a different physical offset that isn't plumbed in.
3. **Fix 3 unapplied** — codex's "warmup N publishes before exposing pose" safety net not yet in. Protects against garbage first publish but doesn't address continuous wrong ICP.
4. **Noise covariances `acc_cov: 0.1, gyr_cov: 0.1`** are Livox-tuned. yesense may need different values.
5. **Scan rate too low** — only ~15 scans per 15s reaching FAST-LIO. More data = tighter state estimate.

---

## Commits on NOS (local only, not pushed)

```
d656cb54  wip: stationary preroll gate               ← Fix 2 (codex)
db259605  wip: relax dup guard + imu_init_num + diag ← Finding #11 + #10
18eedd62  wip: guard regressed rewritten frame_ts   ← Fix 1 (codex)
ce6b8d82  wip: anchor at imu_latest                  ← Finding #8
e7c53201  wip: fix build — hoist pc_data decl
ea3d120d  wip: wall-clock stamp both lidar + imu    ← Finding #7
9bfcae9e  wip: drop stale flake.lock (again)
9ff74abb  wip: livox-sdk2 unpack no-chmod
bd509716  wip: livox-sdk2 via fetchurl (kernel 5.10)
8389a2f9  wip: drop stale fastlio2 flake.lock
64b97e76  wip: fastlio2_native unpackPhase without /usr/bin
def27164  wip: line=0 + timebase diagnostics        ← Finding #6
9bfcae9e  wip: flake self-contained + vendored common
bd509716  wip: fastlio2_native via fetchurl
```

Mac side is synced to the same state but not committed to git (deliberate — all wip).

---

## Key Files

| File | Purpose |
|------|---------|
| `dimos/hardware/sensors/lidar/fastlio2/cpp/main.cpp` | Native wrapper. `on_lcm_point_cloud` + `on_lcm_imu` = the ported input layer. |
| `dimos/hardware/sensors/lidar/fastlio2/cpp/flake.nix` | Self-contained nix build; includes inlined livox-sdk2 derivation. |
| `dimos/hardware/sensors/lidar/fastlio2/cpp/CMakeLists.txt` | Build config; LIVOX_COMMON_DIR falls back to vendored copy. |
| `dimos/hardware/sensors/lidar/fastlio2/cpp/livox_common_vendor/` | Vendored `dimos_native_module.hpp` + `livox_sdk_config.hpp` (copies of `hardware/sensors/lidar/common/`). |
| `dimos/hardware/sensors/lidar/fastlio2/config/mid360.yaml` | FAST-LIO2 runtime YAML. |
| `dimos/navigation/smart_nav/modules/fastlio2/fastlio2.py` | SmartNav-facing Python wrapper; points at the hardware module's build via absolute path. |
| `dimos/robot/deeprobotics/m20/blueprints/nav/m20_smartnav_native.py` | Selects `arise` vs `fastlio2` backend via `M20_SLAM_BACKEND` env var. |

---

## Codex Reviews (3 rounds)

All three reviews delegated to `codex-rescue` agent with structured context dumps. Each round grounded in source reading; ranked Top-3 fixes with specific diagnostics.

- Review #1 (Finding #5–#7): "clock domain mismatch is THE smoking gun; fix lidar stamp in wall-clock domain."
- Review #2 (Finding #8): "variable max_offset causes rewritten-stamp regression; add separate guard."
- Review #3 (Finding #9–#10): "apply Fix 1 (rewritten guard) first. If pose[0] still wrong → abandon FAST-LIO2 LCM port" (user vetoed abandon recommendation).

Codex concerns on abandoning:
- "Remaining unknowns are inside the fetched FAST-LIO-NON-ROS core which is not in-tree. Debugging that core without source is blind."
- "FAST-LIO 1.x worked with this hardware per the migration log and has a simpler IMU propagation model that is less sensitive to timestamp precision."

User preference: push through — we want FAST-LIO2 (newer, actively maintained, modular stack with pgo/localizer/hba).

---

## Next Investigations (prioritized)

1. **IMU-lidar extrinsic** — compute `T_imu_lidar` from M20 URDF + DeepRobotics docs; update `extrinsic_T` in mid360.yaml. Possibly the single biggest remaining drift source.
2. **Codex review #4** — with the new step-jump signature and preroll working, get fresh take on what's happening now that IMU bias is fixed.
3. **Fix 3 (warmup guard)** — discard first N publishes as codex suggested. Trivial safety net.
4. **Tune `acc_cov` / `gyr_cov`** for yesense specifically.
5. **Test click-to-goal end-to-end** even with current drift — might still be usable for short trajectories if map quality is OK.

---

## Effort Accounting

- Total time: ~10 hours across 2 days (2026-04-17 → 2026-04-19)
- ~2 hours: nix build-system fights (Finding #2)
- ~3 hours: identifying + fixing timestamp/clock/sync issues (Findings #5–#9)
- ~2 hours: IMU init / preroll (Finding #10)
- ~1 hour: scan-rate tuning (Finding #11)
- ~2 hours: iteration cycles (rebuild × test × measure)

Original bead estimate: 1–3 days. Actual to this point: ~10 hours, still not production-ready. Suggests the bead scope was optimistic — FAST-LIO2-via-LCM is harder than "just swap the input layer."

---

## Finding #12: leshy Fork is Livox-Only by Design (2026-04-19, codex review #4)

Symptom after all timing fixes: pose[0] near zero (IMU init healthy), but stationary drift still 20–80 m / 15s with step-jumps between stable plateaus. Codex review #4 diagnosed: **point-to-plane EKF measurement-update degeneracy** from ICP's ikd-Tree finding wrong local minima.

Root-cause amplifier: we were feeding RSAIRY structured-scan data through FAST-LIO's Livox non-repetitive path via a `cp.line = 0` hack in `CustomMsg` conversion. Livox preprocess corrupts per-point timestamps for ring-structured data and kills per-return observability.

**Source archaeology in the leshy fork** (`/nix/store/.../FAST-LIO-NON-ROS-source/src/`):

- `Preprocess::process(PC2ConstPtr)` DOES exist — routes to `velodyne_handler`, `oust64_handler`, etc.
- But `FastLio::feed_lidar()` hardcoded to call `LaserMapping::livox_pcl_cbk()` — no dispatch on `lidar_type`
- `LaserMapping` doesn't even declare `standard_pcl_cbk(PC2ConstPtr)` — dead code path from upstream hku-mars/FAST_LIO

This is by design, not a bug. lesh (Dimensional engineer) simplified FAST-LIO2 for Livox-only testing. RoboSense wasn't in scope. Makes sense for their needs; breaks ours.

---

## Finding #13: Forked & Patched the Fork (2026-04-19)

Forked `leshy/FAST-LIO-NON-ROS` → `aphexcx/FAST-LIO-NON-ROS`, branch `dimos-integration-velodyne`. Minimal surgical patch (one commit, +48 lines):

- `LaserMapping::standard_pcl_cbk(const PC2ConstPtr &msg)` — mirrors `livox_pcl_cbk` but calls `p_pre->process(pc2_msg, ptr)` which dispatches to the existing `velodyne_handler` / `oust64_handler`
- `FastLio::feed_lidar_pc2(const PC2ConstPtr &lidar_data)` — public API router

Existing Livox users unaffected (`feed_lidar(CustomMsg)` still routes to `livox_pcl_cbk`).

Repo: https://github.com/aphexcx/FAST-LIO-NON-ROS/tree/dimos-integration-velodyne
Plan: upstream back to leshy once validated on hardware.

---

## Finding #14: Wrapper rewritten for PC2 Feed; Velodyne Path Active (2026-04-19)

`main.cpp::on_lcm_point_cloud` rewritten:

- Removed CustomMsg conversion (no more `cp.line = 0` hack, no more `CustomPoint` packing loop)
- Builds `custom_messages::PointCloud2` directly: copies header, fields (by name — FAST-LIO matches by name), and packed data bytes
- Calls `g_fastlio->feed_lidar_pc2(lidar_msg)` on our patched API
- Keeps all monotonicity guards + stationary preroll + wall-clock anchor

Frame diagnostic log now prints `path=pc2` to confirm the route.

`m20_smartnav_native.py` blueprint switched from `mid360.yaml` → `velodyne.yaml`.

`velodyne.yaml` tuned:
- `lidar_type: 2` (Velodyne PC2 path)
- `scan_line: 64` (matches current drdds bridge 192→64 remap)
- `scan_rate: 10`
- `timestamp_unit: 0` (seconds — drdds bridge publishes f32 relative seconds)
- `blind: 0.5` (M20 body 820×430mm + lidar mount margin)

---

## Finding #15: First Velodyne-Path Measurement (2026-04-19)

Stationary 15s benchmark after all fixes through #14:

| Metric | Livox path (pre-#14) | **Velodyne path (post-#14)** |
|---|---|---|
| pose[0] | (+0.4, +32.8, -0.4) | (-25.7, -33.0, -0.5) |
| 15s drift x | 9 m | 28.7 m |
| 15s drift y | 82 m | **19.9 m** ← improved |
| 15s drift z | 1.2 m | **0.65 m** ← improved |
| Settles (step-jump) | never, continuous drift | sample #5, stays plateau |

Y + Z drift improved, X slightly worse. Still 20–30m over 15s stationary — not production-ready for indoor nav (room is 3–5m), but the pattern is now "jumps to a stable wrong plateau" vs "continuously diverging." Each step-jump is an ICP wrong-local-minimum event.

---

## Finding #16: drdds Bridge Jitter is the Next Big Rock (2026-04-19)

Read-through of `drdds_bridge/cpp/main.cpp` + `drdds_recv.cpp` identified 5 concrete jitter sources driving ICP into bad local minima. Ranked by impact:

1. **Variable scan span (100 ms vs 200 ms) from rsdriver merge**. M20 has two RSAIRY lidars rotating independently; rsdriver (`send_separately: false`) merges whatever's available every emit cycle. Sometimes both → 200 ms span of points. Sometimes one → 100 ms span. Our EKF and our anchor math both assume constant span and jitter when it changes.

2. **Duplicate scans (identical sensor_ts)**. Likely drdds multicast redelivery or rsdriver publishing on multiple paths. Our SHM writer accepts everything.

3. **Out-of-order (1.6s old) scans**. SHM ring buffer = 16 slots × 4 MB = 1.6s of buffer. When our bridge reader stalls briefly (GC, CPU contention), writer overtakes us; our reader catches up on old entries producing backward-timestamp scans.

4. **Ring remap `ring * 64 / 192`** in the bridge. Collapses 3 physical beams into 1 logical ring; FAST-LIO's Velodyne preprocess treats neighboring points-in-same-ring as spatially close, but our "same bin" points are from 3 different vertical angles → garbage neighbors → bad curvature/normal estimation.

5. **Time relativization order dependency**. `first_t = first point's time` assumes known emit ordering. If rsdriver's order changes, relative times become inconsistent across scans.

### Recommended fix order (not yet applied)

**A. Remove 192→64 ring remap** — let rings pass through natively at 192. Set `scan_line: 192` in velodyne.yaml. Biggest geometry win for minimal code. Risk: FAST-LIO's internal N_SCANS arrays may overflow at 192 (upstream tested to 128) — verify + bounds-check before deploying.

**B. Push monotonic + dedup guards upstream into DrddsLidarBridge itself.** Currently all the guards live in our fastlio2_native LCM handler; move them to the bridge so there's ONE source of truth and any future consumer (not just fastlio2) gets clean data.

**C. Try `send_separately: true` in rsdriver + subscribe to front lidar only** (96 rings). Less data but clean. Eliminates merge-origin jitter entirely. Quick test, decide if 96 rings of stable data beats 192 of jittery merged data.

**D. Buffer-and-complete in bridge** (last resort, trickiest).

---

## Finding #17: Codex Review #4 — Ranked Next Moves + Abandonment Re-eval (2026-04-19)

Codex's review #4 after all Findings #1–#15 landed:

**Ranked top 3 remaining fixes** (besides review-#3's Fix 1/2/3 already in):

1. **Measure real yesense↔RSAIRY extrinsic from URDF** → replace `extrinsic_T: [0, 0, 0.28]` (current Velodyne default) + `extrinsic_R: identity` in velodyne.yaml. Wrong extrinsic alone = 0.05–0.3 m/min continuous drift AND pushes EKF toward local-minimum traps. Addresses both.

2. **Stop routing RSAIRY through Livox path** — DONE in Findings #13/#14.

3. **Fix drdds_bridge scan formation** — see Finding #16 for concrete plan.

**Abandonment update**: codex retroactively rescinded the "abandon FAST-LIO2" recommendation once Fix 1 (regressed-rewritten-stamp guard) demonstrably fixed pose[0]. Remaining problem is Livox-config-induced ICP degeneracy, not structural blocker. **Do not abandon.** User also explicitly vetoed abandonment regardless.

**Codex's diagnostic command** to confirm ICP degeneracy is the failure mode:
```
grep 'NO Effective Points' /tmp/smartnav_native.log
```
If those timestamps align with observed pose jumps → direct evidence of scan-match degeneracy from FAST-LIO's correspondence-selection stage.

---

## Current State (end of 2026-04-19 session)

**Working:**
- Native FAST-LIO2 binary running on M20 NOS
- LCM input path via Velodyne feed (`feed_lidar_pc2` on patched fork)
- IMU stationary preroll gate catches clean bias init
- Monotonic + regressed-rewritten scan guards prevent EKF corruption
- Wall-clock stamping resolves clock-domain mismatch
- Odometry publishing at 1.5 Hz, registered_scan visually aligned

**Not working:**
- Stationary drift 20–30 m / 15s — step-jumps to wrong plateaus
- Extrinsic in YAML is Velodyne default, not M20 reality
- drdds bridge jitter (see Finding #16) pushes EKF into bad ICP matches

**Pose[0] achievement:** went from (−9200, −9465, −3039) m (catastrophic EKF blowup) → (−25.7, −33.0, −0.5) m. 3 orders of magnitude improvement. Step-jumps are now the remaining class of error.

---

## Commits on NOS (local, not pushed — all WIP)

```
e62ba2c9  wip: velodyne PC2 path + aphexcx fork + velodyne.yaml RSAIRY tuning
d656cb54  wip: stationary preroll gate
db259605  wip: relax dup guard + imu_init_num + more diag
18eedd62  wip: guard regressed rewritten frame_ts
ce6b8d82  wip: anchor at imu_latest
e7c53201  wip: fix build — hoist pc_data decl
ea3d120d  wip: wall-clock stamp both lidar + imu
[earlier: nix build-system fights, see Finding #2]
```

**Pushed (fork):**
```
aphexcx/FAST-LIO-NON-ROS  branch: dimos-integration-velodyne
  3ecc60e  feat: restore Velodyne PointCloud2 input path for non-Livox lidars
```

---

## Next Session — Start Here

**Context to reload:** read this log + `dimos/hardware/sensors/lidar/fastlio2/cpp/main.cpp` + `dimos/hardware/sensors/lidar/fastlio2/config/velodyne.yaml`. The aphexcx/FAST-LIO-NON-ROS fork is referenced in the flake.nix and already compiled on NOS.

**Assets available (durable):**
- `plans/m20-rosnav-migration/M20_high_res.urdf` — M20 URDF, copied from `~/Downloads/`. Source of truth for the IMU↔lidar extrinsic.
- `plans/m20-rosnav-migration/FASTLIO2_LOG.md` — this log.
- `plans/m20-rosnav-migration/ARISE_SLAM_LOG.md` — ARISE journey (sibling context).
- `plans/m20-rosnav-migration/NATIVE_NAV_LOG.md` — overall native nav migration.

### Priority-ordered next steps

**1. Extract yesense↔RSAIRY extrinsic from M20_high_res.urdf** (~30 min)
- Parse URDF with `rosidl` or a Python XML parser (urdf_parser_py is NOT needed — just walk the `<link>` / `<joint>` tree)
- Locate the yesense IMU link and the RSAIRY lidar link (or the front/back lidars if separate)
- Compute T_base_imu and T_base_lidar (chain transforms through joints)
- Compute T_imu_lidar = T_base_imu⁻¹ · T_base_lidar
- Format as `extrinsic_T: [x, y, z]` and `extrinsic_R: [...]` (3×3 row-major or column-major — check FAST-LIO2 convention)
- Update `dimos/hardware/sensors/lidar/fastlio2/config/velodyne.yaml` with the measured values
- Also set `extrinsic_est_en: true` so FAST-LIO2 refines online as a safety net
- **No binary rebuild needed — YAML change only**
- Test: restart smartnav, sample 15s stationary, compare drift to current baseline (28.7m x, 19.9m y, 0.65m z)

**2. Apply drdds bridge Fix A: remove the 192→64 ring remap** (~20 min + rebuild)
- Edit `dimos/robot/deeprobotics/m20/drdds_bridge/cpp/main.cpp` lines 155–162 — delete the `ring * 64 / 192` block
- Update velodyne.yaml: `scan_line: 192`
- BEFORE BUILDING: verify the leshy fork's preprocess.cpp + laserMapping.cpp don't have hardcoded arrays sized on N_SCANS that would overflow. Grep `N_SCANS` in `/nix/store/znm8qkahgiar29c86ivb6gxr7fbr4q3z-source/src/` (or the new fork's store path after build). Upstream FAST-LIO tested to 128 per the ARISE_SLAM_LOG; 192 might need patching or may panic.
- Rebuild drdds_bridge only (fast, no need to rebuild fastlio2_native): `cd dimos/robot/deeprobotics/m20/drdds_bridge/cpp && nix build ...`
- Test: same 15s stationary measurement

**3. Apply drdds bridge Fix B: push guards upstream** (~30 min)
- Move the monotonicity + dup guards from `fastlio2/cpp/main.cpp::on_lcm_point_cloud` into `drdds_bridge/cpp/main.cpp::lidar_loop` before publishing.
- Rationale: clean data at the SOURCE, all downstream consumers benefit.
- Remove the corresponding guards from the fastlio2 wrapper once bridge guards are in.

**4. Enable Fix 3 (publish-ready warmup guard in fastlio2_native)** (~10 min)
- Add: discard first N publishes as warmup (codex review #3 suggestion, never applied).
- Trivial safety net — prevents any uninitialized FastLio state from leaking out.

**5. Run codex's ICP-degeneracy diagnostic**
- `grep 'NO Effective Points' /tmp/smartnav_native.log`
- If hit counts align with step-jump events in sample data, confirms the remaining failure class.
- If no hits → something else is causing the plateau-to-plateau jumps; need investigation.

**6. Consider single-lidar fallback (Fix C)**
- Only if #1–#4 don't get us under 5 m / 15s drift.
- Test `send_separately: true` in rsdriver config, subscribe front lidar only (96 rings of clean data vs 192 of jittery merged).

**7. Run click-to-goal end-to-end** once drift is acceptable
- Indoor living-room scenario, click ~2 m target
- Acceptance: robot reaches within 0.3 m of clicked point, no oscillation, no obstacle collisions

---

## Finding #18: Airy Integrated IMU Path — Built and Partially Working (2026-04-19 late)

Pivoted from yesense to the RoboSense Airy's integrated IMU (one per lidar, 200 Hz, PTP-UTC locked to the Airy lidar's own hardware clock). Motivation: the yesense↔RSAIRY extrinsic was unknown and cross-clock-domain timestamps drove the Findings #7/#8 fragility.

**What's built and verified:**

- `dimos/robot/deeprobotics/m20/drdds_bridge/cpp/airy_imu_bridge.cpp` — native module that binds UDP multicast `224.10.10.201:6681` (front) or `.202:6682` (rear), parses the 51-byte RSAIRY IMU packet per rs_driver's `decoder_RSAIRY.hpp`, applies FSR-aware unit conversion (raw × `2^(FSR+1)/32768` g for accel, `250·2^FSR/32768 · π/180` rad/s for gyro), applies a PTP-lock sanity gate (drop packets with UTC seconds < 2024-01-01), rotates accel+gyro from Airy sensor frame into base_link, and publishes LCM `sensor_msgs/Imu`. Running at a solid 200.0–200.4 Hz, `pkt_vs_wall = −0.001s` typical (PTP is tight).

- Empirically-derived rotations (rsdriver's euler convention decoded as neither ZYX nor XYZ Tait-Bryan — verified from stationary gravity vector):
  - `R_base_from_front = [[0,0,1], [-1,0,0], [0,-1,0]]` — Airy X (cable) → base +Z, Y → base -Z (down), Z (dome) → base +X (forward)
  - `R_base_from_rear  = [[0,0,-1], [1,0,0], [0,-1,0]]` — same form, dome → base -X (backward)
  Sanity check: stationary rotated accel = (0.03, −0.1, 9.85) m/s² ≈ (0, 0, g). ✓

- `fastlio2` wrapper: added `--native_clock` CLI flag (threaded through the Python `FastLio2` config as `native_clock: bool`). When true, bypasses the legacy wall-clock-stamp + `anchor-at-imu_latest` workarounds (Findings #7, #8) and enforces a monotonic floor on `frame_ts`: `max(sensor_ts, prev_frame_ts + 1e-4)`. Trust-sensor-timestamps path.

- `velodyne.yaml`: extrinsic `T=[0,0,0]`, `R=identity`, `extrinsic_est_en: true`. Valid only with the Airy IMU path — both lidar cloud (rsdriver-merged in base_link) and IMU (bridge-rotated to base_link) are in the same frame.

- Blueprint `M20_FASTLIO2_IMU` env var: `airy` (default) or `yesense` (fallback).

- rsdriver config `ts_first_point: true → false` on NOS. Codex recommended this to reduce first-point-of-interleaved-lidars cloud-timestamp jitter across merged scans.

**What's NOT working (the remaining blocker):**

Stationary-robot drift on the Airy IMU path:
- With old workarounds active: 965 m / 75 s (catastrophic — the wall-clock anchor wrongly rewrote already-PTP-consistent timestamps).
- With `native_clock=true` and monotonic floor: **initial ~50 s show near-perfect drift (Keyframes 1–2 within 1.5 m of origin), then drift explodes.** Keyframe 10 reaches hundreds to thousands of meters in X/Y/Z.
- With `ts_first_point: false` + all above: same pattern — short good period then runaway.

**Diagnostic smoking gun:** `imu_latest` in fastlio2 stalls 2–4+ seconds while lidar frame timebase marches forward. Example sequence:
```
frame #25  timebase=1776634594.600  imu_latest=1776634593.223  imu_vs_frame_end=-1.577s
frame #26  timebase=1776634594.700  imu_latest=1776634593.223  imu_vs_frame_end=-1.677s
frame #27  timebase=1776634595.900  imu_latest=1776634593.223  imu_vs_frame_end=-2.877s
frame #28  timebase=1776634596.300  imu_latest=1776634593.223  imu_vs_frame_end=-3.277s
frame #29  timebase=1776634597.600  imu_latest=1776634593.223  imu_vs_frame_end=-4.577s
```

Meanwhile `airy_imu_bridge` logs steady 200 Hz with `pkt_vs_wall = -0.001s`. So the bridge is publishing fine; fastlio2's LCM subscriber is backlogged. Per codex review #4 (Q3): the classic signature of a single-threaded LCM handle loop where a slow lidar callback (`feed_lidar_pc2` with 100k-160k points per cloud) blocks IMU deliveries; IMU messages queue in the OS socket buffer until lidar callback returns, then get processed in a burst at their (now stale) sensor timestamps.

**Y-axis accel bias interlude (attempted fix, reverted):** Noticed +0.27 m/s² persistent Y bias on stationary base_link accel. Tried subtracting 0.19 from sensor-frame Y in the bridge. **Wrong axis** — under `R_base_from_front`, sensor Y maps to base Z, not base Y. Base Y bias corresponds to sensor X. The subtraction inflated base Z to 10.05 instead of zeroing base Y. Reverted. Correct fix is to calibrate sensor-X offset (or bias-subtract in base frame after rotation).

---

## Current State (end of 2026-04-19 session)

**Working:**
- Native `airy_imu_bridge` → LCM `airy_imu_front` at 200 Hz, base_link frame, PTP-synced with Airy lidar
- `fastlio2_native --native_clock true` uses sensor timestamps directly + monotonic floor
- rsdriver `ts_first_point: false` on NOS
- Drift for the first ~50 s after `preroll complete`: within ~2 m of origin. That's production-grade for the stationary case.

**Not working:**
- After the initial good window, the LCM IMU subscriber in fastlio2 stalls for seconds while lidar keeps arriving. During stalls FAST-LIO's state estimator propagates on stale IMU + scan-matches to an already-drifted state → compounds catastrophically.
- Y-axis accel bias (~0.3 m/s² in base frame) unfixed.

**Hypothesis for the stall (codex Q3, needs verification):** fastlio2's LCM handle loop runs lidar and IMU callbacks on a single thread. When the lidar callback's `feed_lidar_pc2` takes >5ms for a 100k-point cloud, IMU messages queue in the socket buffer. At 200 Hz IMU + a slow lidar callback, the subscriber never catches up.

---

## Next Session — Start Here

**Priority 1: fix the LCM IMU subscriber stall.**

Options ranked:

1. **Split the LCM subscriber into separate threads for lidar and IMU.** The cleanest fix. Inside fastlio2's main.cpp, instead of one `lcm.handle` main loop, spawn a dedicated thread for the IMU subscription and let the main thread handle lidar. LCM's `LCM` class is thread-safe for publish but subscriber callbacks are serialized; running two separate `lcm::LCM` instances (one per thread) on the same URL with different `Subscribe` calls should deliver IMU and lidar independently. Verify against LCM docs first.

2. **Drop-old IMU queue.** Instead of serial processing, accept that we'll skip some IMU messages under load. Only store the latest IMU timestamp + state; discard any delivered IMU whose stamp is older than `g_last_imu_ts`. This doesn't fix the underlying stall but prevents the "process 40 seconds of stale IMU in a burst" pathology.

3. **Reduce lidar callback cost.** The 100k-160k point cloud per scan is big. Could voxel-downsample in the bridge before publishing — but that loses fidelity and codex Finding #2 argued against it. Alternative: do the PointCloud2→CustomMsg conversion off the LCM callback thread.

4. **Alternative entirely: `time_sync_en: true` in velodyne.yaml** — tells FAST-LIO2 to estimate lidar↔IMU time offset online. Documented in upstream FAST_LIO config. Worth testing because our current situation may be exactly what this flag was designed for.

**Priority 2: fix the base-frame Y accel bias.**

Either:
- Add a sensor-frame X offset in `airy_imu_bridge` (which maps to base -Y under rotation). Measure stationary Y residual, compute `sensor_X_offset = -base_Y_bias` (since base Y = -sensor X). Apply per lidar side.
- Or subtract the bias in base frame AFTER rotation. Simpler.

**Priority 3 (if above don't fix): revisit architecture.**

Codex's ongoing critique (earlier in this session, agentIds `abd9d86ead7d6d158`, `a6ba752b0b5e3e063`, `ac126b5325c55c1c0`, `a67517fbd9002d7e7`, `af02bd4ffd42e20f7` — codex thread continuation via SendMessage was not used; each agent is fresh) pointed at a structural mismatch: FAST-LIO2 assumes one rigid lidar + one IMU. The merged-dual-Airy cloud violates that. Even with a good IMU source, the input model is wrong. Full solution may need:

- Switch to FAST_LIO_MULTI (async mode, single shared IMU, per-lidar observation updates) OR
- Pre-deskew + rigid-transform rear lidar into front-lidar frame in our bridge (LOCUS-style preprocess), use front-Airy IMU with identity extrinsic.

These are bigger lifts. Try the Priority 1/2 stall fixes first.

---

## Commits on NOS (local, not pushed)

```
(recent)
wip: revert bad Y bias subtraction
wip: codex fixes (monotonic floor + Y bias + diag pkt_vs_wall)
wip: parse native_clock manually (arg_bool not in lidar common)
wip: native_clock flag
wip: skip nav_cmd_pub in nix
wip: fix airy rotation (empirical base_link map)
wip: airy_imu_bridge on NOS
(earlier) — all prior session wip from Findings #1–#17
```

**Untracked but promoted from prior session:**
- `dimos/robot/deeprobotics/m20/drdds_bridge/cpp/airy_imu_bridge.cpp`
- `dimos/robot/deeprobotics/m20/drdds_bridge/cpp/include/dimos_native_module.hpp` (vendored from `dimos/navigation/smart_nav/common/` — keep in sync)

---

## Finding #19: rsdriver separate mode works — discovery ordering was the gate (2026-04-21)

SHM discovery ordering (ROSNAV_MIGRATION_LOG Finding #5) blocked the whole
earlier investigation. When I first flipped `send_separately: true`, the
probe showed `matched=1` on `/LIDAR/POINTS` / `/LIDAR/POINTS2` but 0 msgs
— indistinguishable from "separate mode broken." After a clean sequence
(stop rsdriver/yesense, `pkill -9 drdds_recv`, clean `/dev/shm/fastrtps_*`,
start drdds-bridge first, then rsdriver + yesense), both streams flowed.

Empirically confirmed in separate mode (rsdriver's DeepRobotics-custom
flag, not upstream):

| Topic              | Content                      | Rate | Frame                |
|---|---|---|---|
| `/LIDAR/POINTS`    | Front Airy cloud             | 10 Hz| base_link (extrinsic applied) |
| `/LIDAR/POINTS2`   | Rear Airy cloud              | 10 Hz| base_link (extrinsic applied) |
| `/LIDAR/IMU201`    | Front Airy integrated IMU    | ~60 Hz| raw sensor frame, g-units    |
| `/LIDAR/IMU202`    | Rear Airy integrated IMU     | ~60 Hz| raw sensor frame, g-units    |
| `/IMU`             | Yesense IMU                  | 200 Hz| m/s², base-ish frame          |

Airy IMUs via rsdriver are too low-rate and in wrong units for FAST-LIO —
`airy_imu_bridge` (direct multicast tap at 224.10.10.20{1,2}:668{1,2})
stays as the IMU source, at 200 Hz m/s² base_link after rotation.

PTP sync across the two Airys is sub-microsecond (stamps on adjacent
front/rear pairs differ by ~3 µs).

`drdds_recv` extended with 5 subscriptions (existing lidar + IMU + 3 new
channels). Each writes to its own POSIX SHM segment for downstream
independent consumption. Built via the local-cmake path (not nix —
depends on host FastDDS at `/usr/local`).

---

## Finding #20: LCM Transport Split Killed the 50s-Explode Pattern (2026-04-21)

Finding #18 left the blocker at "fastlio2's LCM subscriber stalls 2-4s
during heavy lidar callbacks." Codex review pinned the pattern: single
`lcm.handle()` loop serializes lidar and IMU callbacks. The lidar callback
did all the heavy work synchronously (max-offset scan, ring remap, time
relativize, 2.6MB byte-copy into FAST-LIO's message type, `feed_lidar_pc2`),
which blocked ~20 IMU messages per cycle in the socket buffer. When the
queue spilled, `imu_latest` stalled for seconds and FAST-LIO propagated
on stale IMU → runaway drift after ~50s.

Fix in `dimos/hardware/sensors/lidar/fastlio2/cpp/main.cpp`:

- Subscriber callbacks now only copy raw bytes into a simple struct and
  push onto bounded `std::deque` (lidar cap 3, IMU cap 400 ≈ 2s at 200 Hz).
  Drop-oldest on overflow.
- New `fastlio_owner_loop` thread drains queues with IMU priority: all
  pending IMU is drained before each lidar frame so `g_last_imu_ts` tracks
  actual publish timestamps even during heavy processing.
- All former callback-body logic (preroll gate, native-clock monotonic
  floor, ring remap, `feed_lidar_pc2`) moved to the owner thread.

Commit `84b91fa0e`.

---

## Finding #21: Accel Bias Subtraction is a Bandaid — Proper Fix is DIFOP C.17 (2026-04-21, superseded by #22)

**Superseded by Finding #22.** When this entry was written we thought the
DIFOP C.17 quaternion direction or convention was the blocker. It wasn't
— the real blocker was that the empirical `R_BASE_LIDAR_FRONT` matrix
was actually `R_base_from_IMU` (collapsed the lidar-housing-to-IMU-chip
rotation into the mount rotation), so composing DIFOP on top double-
applied it. Keep this entry for history; see #22 for the real fix.



With the LCM split in place, 5-min stationary drift dropped from the
earlier 2000 m runaway to ~2000 m (still runaway) because the IMU now
delivered on time but was still biased. Base-frame stationary output:
`acc=(0.13, 0.35, 10.03)` m/s². Integrated twice over 180 s the 0.35 m/s²
Y bias alone produces ~5700 m of drift — matched the observation.

**First attempt: hardcode the bias.** Subtracted `(0.13, 0.35, 0.22)` from
the rotated base-frame accel in `airy_imu_bridge`. With the LCM split +
this subtraction, 5+ minute stationary holds at **1 keyframe at origin,
no additional keyframes**. Commit `84b91fa0e`. Producible and durable
for stationary testing but a known bandaid.

**Why bandaid:** hardcoded constants from one stationary snapshot.
Doesn't generalize across robots, temperatures, post-impact bias shifts,
or robot orientation (we measured upright on carpet; tilt the body and
the "bias" direction rotates). FAST-LIO2's own bias-random-walk state
can track slow accel drift, but not a fixed rotation error masquerading
as bias.

**Real fix:** RSAIRY publishes a factory IMU-to-lidar rigid transform in
DIFOP register C.17 (`IMU_CALIB_DATA`). Per the Airy user guide + rs_driver
source (`decoder_RSAIRY.hpp`), it's 7 `uint32_t` fields big-endian at
offset 1092 in the 1248-byte DIFOP packet, bit-cast to float32:

```
q_x, q_y, q_z, q_w, x, y, z   (quaternion + lever-arm translation)
```

Codex review confirmed: (1) C.17 is geometric extrinsic, not bias; (2) a
1-2° tilt between the nominal sensor frame and the factory-mounted IMU
axes is the most likely source of our residual; (3) quaternion direction
is almost certainly IMU→lidar (inferred — not documented publicly); (4)
translation is lever-arm between IMU and lidar origins, used for
centripetal correction during motion (skip for v1); (5) each Airy has
per-unit calibration, must parse DIFOP separately for front and rear.

**Plan for v1 (front Airy only, stationary validation first):**

1. **DIFOP reader at startup in `airy_imu_bridge`**
   - Bind second UDP multicast socket on 224.10.10.201:7781 (front) or
     `:7782` (rear) per `--which`. DIFOP is ~1 Hz so a blocking `recv`
     with a 5-second timeout is fine.
   - Validate magic `0xA5 0xFF 0x00 0x5A 0x11 0x11 0x55 0x55` at offset 0
     and tail `0x0F 0xF0` at offset 1246.
   - Extract offsets 1092..1119 (28 bytes).
   - Parse 7 × `uint32_t` big-endian with `ntohl`, bit-cast each to float32.
   - Normalize the quaternion (defensive — factory values might have
     tiny denormalization).

2. **Build `R_imu_to_lidar` (3×3 rotation matrix) from quaternion**
   - Standard xyzw → rotation-matrix formula.
   - **Renormalize defensively** before converting to matrix: rs_driver's
     `decodeDifopPkt()` does no normalization, and a slightly non-unit
     quaternion produces a non-orthonormal matrix — not just scale drift
     but shear / axis-coupling. Reject + log if `||q| − 1| > 0.01`.
   - Log the derived matrix + its frobenius-norm distance from identity
     at startup. Expect small rotation (≤ few degrees); if it's larger,
     direction convention (IMU→lidar vs lidar→IMU) becomes more load-
     bearing. Disambiguation test: quasi-static in-place yaw — pick the
     convention whose corrected gyro lands on a single base axis with
     expected sign and minimum cross-axis energy.

3. **Precompute the composed `R_imu_to_base = R_base_lidar · R_imu_to_lidar`**
   - `R_base_lidar` stays hardcoded per lidar side — geometrically
     determined from the physical mount on M20:
     - front: `[[0,0,1],[-1,0,0],[0,-1,0]]` (dome→+X_base, cable→-Z_base)
     - rear:  `[[0,0,-1],[1,0,0],[0,-1,0]]` (dome→-X_base, cable→-Z_base)
   - rsdriver config's `(yaw, pitch, roll)` uses a non-standard Euler
     encoding we couldn't reverse-engineer; none of ZYX/XYZ/ZXY/YZX/ZYZ/
     XZX decompositions of `(−π, −π/2, 0)` produce the observed matrix.
     Physical mount is unambiguous so we skip the reverse-engineering.

4. **Apply `R_imu_to_base` per sample in `process_airy_imu_sample`**
   - Replaces both the existing `R_base_from_sensor` rotate step AND the
     hardcoded `(0.13, 0.35, 0.22)` subtraction.
   - Delete the subtraction block.

5. **Fallback if DIFOP doesn't arrive — degraded startup, not blocking**
   - Main IMU path starts immediately with `R_imu_to_lidar = identity`.
     Log loudly (`[airy_imu_bridge] DIFOP not yet received — running
     DEGRADED (no factory IMU-lidar calibration applied). Drift may be
     large.`) and publish a health flag (a new status line in the 5s
     status report: `cal=PENDING | cal=OK`).
   - Background thread keeps retrying DIFOP every 2 s. When a valid C.17
     is received, hot-swap the rotation matrix via atomic pointer load
     on the next IMU sample and log `cal=OK`.
   - Never block startup indefinitely — factory pilot needs robustness
     over correctness at boot. The 1 Hz DIFOP race against our startup
     would be a flaky boot under degraded network conditions.
   - Do NOT fall back to the hardcoded (0.13, 0.35, 0.22) subtraction.

6. **Skip the lever-arm translation `(x, y, z)` for v1**
   - Correct formula is `a_imu = a_body + ω × (ω × r) + α × r` where
     `r = lever arm from IMU to body origin`. At `|ω| = 2 rad/s` and
     `r = 0.32 m`, the term is ~1.3 m/s² — not negligible during motion.
     But stationary test doesn't exercise it. Revisit after motion
     drift appears.

7. **Validation**
   - Stationary test: base-frame accel should read approximately
     `(ε, ε, 9.81)` with |ε| < 0.05 m/s² (vs today's 0.35 Y / 0.22 Z
     residual). If still 0.3+ m/s² off, our assumption that the
     residual is frame tilt was wrong — the fix didn't help and we
     need to revisit.
   - 5-min stationary holds at ≤ 1 keyframe at origin (current baseline
     from the bandaid path).
   - Motion test (walk the robot): position drift should not grow with
     orientation — the bandaid would break when the bias vector rotates
     with the robot; the proper fix should not.
   - **If a ≥0.2 m/s² residual persists after applying C.17**, rank
     candidate causes (per codex review):
     1. `R_base_lidar` off by a few degrees (mount not perfectly aligned).
        Test: quasi-static ±pitch / ±roll — mount-frame error flips
        predictably with tilt direction, true bias does not.
     2. True accel bias (temperature / mount-stress). Test: hold still
        several minutes cold-to-warm; persists or drifts with temperature,
        not pose sign.
     3. FSR mismatch. Test: log live `accelFsr` / `gyroFsr` bytes from
        every Airy IMU packet; wrong FSR shows as uniform scale error
        across all tilts, `|a|` consistently wrong.
     4. FAST-LIO2 `extrinsic_est_en` state absorbing something — lowest
        probability; the residual is visible at the `airy_imu_bridge`
        topic BEFORE FAST-LIO2 ingests it, so FAST-LIO2 can't cause it.

8. **Rear Airy (follow-up)**
   - Same machinery: `--which rear` reads DIFOP at `:7782`, applies its
     own per-unit `Q`, composes with the rear-specific `R_base_lidar`.

9. **Hardcoded-matrix caveat, documented**
   - Hardcoding `R_base_lidar` per front/rear is acceptable for the M20
     pilot where mount geometry is invariant. Real failure modes if we
     reuse this code elsewhere: different Airy bracket revision, front/
     rear swapped, physical rework of mount orientation. Mitigation:
     document the assumption in `airy_imu_bridge.cpp` header comment.
     Reading from URDF / env var would buy portability but doesn't solve
     rsdriver's non-standard Euler encoding problem — it's a separate
     concern. Revisit if we ever ship to a second robot model.

---

## Finding #22: Mount Matrix Was Wrong — Fixed, DIFOP Now Works End-to-End (2026-04-21)

Shipping Finding #21 revealed the rotation pipeline was broken: base-frame
stationary accel landed on **-Y_base** instead of **+Z_base**. Codex ranked
the candidates (H1 most likely: the original empirical `R_BASE_LIDAR_FRONT`
was actually `R_base_from_IMU`, not `R_base_from_lidar` — double-applied
the IMU-lidar rotation once we added DIFOP).

**Root cause was the mount geometry assumption.** User confirmed: "Airy
mounted horizontally, dome forward, cable exits upward through the top of
the housing into the robot." That pins:

```
lidar Z (dome)  → base +X  (forward)
lidar X (cable) → base +Z  (up)
lidar Y         → base -Y  (right, by right-hand rule)
```

The OLD matrix `[[0,0,1],[-1,0,0],[0,-1,0]]` had lidar X → base -Y, which
was consistent with "cable pointing SIDEWAYS" — wrong. It worked pre-DIFOP
only because it was implicitly also absorbing the IMU-chip rotation.

**Derivation back from observation**: DIFOP Q × raw_imu → gravity on
`+X_lidar` (≈9.86). Right-hand rotation with `+X_lidar → +Z_base` and
`+Z_lidar → +X_base` forces col_1 = col_2 × col_0 = `(1,0,0) × (0,0,1) =
(0,-1,0)`. Codex verified the cross-product, the `Rz(180°)·FRONT = REAR`
identity, and the composition order `R_base_from_imu = R_base_lidar ·
R_imu_to_lidar`.

**Corrected matrices:**

```cpp
R_BASE_LIDAR_FRONT = {{0,0,1}, {0,-1,0}, {1,0,0}};  // det = +1
R_BASE_LIDAR_REAR  = {{0,0,-1},{0, 1,0}, {1,0,0}};  // det = +1 (front rotated 180° about +Z_base)
```

**Result — stationary base-frame accel (front Airy, M20 upright):**

```
(0.101, -0.066, 9.831) m/s²
(0.119, -0.071, 9.845) m/s²
(0.114, -0.058, 9.844) m/s²
```

Residuals ≤ 0.15 m/s² on X/Y (IMU noise floor), gravity ~9.84 on +Z.
**Better than the old bandaid + no bandaid needed.** Per-unit factory
correct, tilt-invariant in principle (bias no longer masquerades as
rotation).

---

## Finding #23: Dual drdds_recv Processes Saturated NOS, Hid the Real Drift State (2026-04-22)

After shipping the DIFOP + corrected-mount fix, the initial smartnav
restart showed catastrophic runaway drift — 83 keyframes in 3 min,
positions at `(-5192, -9205, -9377)` m. Base-frame stationary accel
looked fine (`(0.12, -0.07, 9.84)`), so the 180m/s "velocity" the
keyframes implied had to come from somewhere else.

Root cause: **two systemd services were running `drdds_recv` simultaneously.**

- `drdds-bridge.service` (original, from upstream rig provisioning).
  ExecStart: `/opt/drdds_bridge/lib/drdds_bridge/drdds_recv`.
- `drdds-recv.service` (installed by our `deploy.sh provision`).
  ExecStart: `/opt/drdds_bridge/lib/drdds_bridge/drdds_recv`. Same binary.

Both were enabled + active, both competing for the same FastDDS SHM
subscriptions and writing to the same `/dev/shm/drdds_bridge_*` segments.
`top` showed two `drdds_recv` PIDs each consuming ~28% CPU, and the
5-minute load average was **38 on an 8-core NOS** (system thrashing).

Symptom in fastlio2: `imu_vs_frame_end` grew from ~-0.1 s to **-0.7 s**
as the lidar/IMU pipeline couldn't drain fast enough. FAST-LIO2
propagated its EKF on increasingly stale IMU data, treated the IMU
residual as motion, and emitted keyframes at runaway rates.

**Fix:** `sudo systemctl disable drdds-bridge` (keeping the deploy.sh-
provisioned `drdds-recv`). Load immediately dropped 38 → 1.2.
`imu_vs_frame_end` now stays in the -0.05 to -0.09 s band with a single
`drdds_recv`. Keyframes hold at 1 at origin when stationary (matching
the pre-compaction bandaid baseline, minus the bandaid).

**Secondary finding — deploy.sh symlink loop:** the `for store_path in
/nix/store/*-drdds_lidar_bridge-*/bin/airy_imu_bridge; do ln -sf ...;
done` loop in `deploy.sh sync` iterates over all matching nix store
paths alphabetically, so the symlink ends up pointing at whichever hash
sorts LAST, not the most recently built one. On a host with many stale
builds this silently reverts to an old binary after every `sync`.
Workaround for now: manually `ln -sfn <fresh-store-path>` after every
rebuild; long-term fix is to either (a) use `nix profile` to track the
latest output, (b) parse `result` symlink that `nix build` creates, or
(c) sort globs by mtime in the deploy script. Noted as TODO.

---

## Open Questions for Next Session / Jeff

- Does FAST-LIO2's preprocess.cpp + ikd-Tree actually support `scan_line: 192`? Upstream hku-mars tested up to 128 per README.
- Any known issues with scan_line counts above 64 we should watch for?
- Is the N_SCANS value ever used to allocate fixed arrays anywhere that would segfault at 192?
- Would Jeff accept a PR upstreaming the Velodyne input restoration to `leshy/FAST-LIO-NON-ROS`?
- Should the PGO duplicate-key crash (`RuntimeError: key "33", already exists` in `isam2.update`) be a separate bead? Saw it crash smartnav once during this session — likely upstream fastlio2 outputting multiple poses per scan when frames have near-identical timebases.

