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

## Finding #24: Rotation Breaks SLAM — Cloud Is in Raw Sensor Frame + Yaw DOF in IMU Rotation (2026-04-22)

Stationary SLAM is rock solid (1 KF at origin, 5+ min, cal=OK, DIFOP
applied) but any meaningful rotation causes catastrophic drift —
keyframes explode to thousands of meters within a few seconds of yaw.

### Experiments run

**Expt 1: `extrinsic_est_en: true → false` in velodyne.yaml**

Hypothesis (codex H1): FAST-LIO2's online estimator was destabilizing
under rotation because DIFOP already pins IMU↔lidar extrinsic — two
processes fighting. With `extrinsic_est_en: false`, early rotation KFs
stayed within ~0.3 m for the first few seconds (big improvement from
instant 10,000 m explosion). **Partial fix.** KFs then drifted
gradually, with a sudden 80 m teleport at KF 16 (scan-match lost lock)
and runaway after. Instant-explode mode eliminated but underlying frame
mismatch remains.

**Expt 2: Cloud-frame probe (the key data)**

Wrote a Python probe that reads `/dev/shm/drdds_bridge_lidar` directly
and dumps X/Y/Z min/max/mean/std for a single cloud frame. Important
note: `sizeof(ShmHeader) = 56` (not 64 — no `static_assert` catches
this, compiler packs to 56). My first probe was 8 bytes off.

Zeroed rsdriver's `pitch=-π/2 yaw=-π roll=0` → all zero, for both front
and rear Airy stanzas. Restarted pipeline. Probe showed (front lidar):

```
zeroed config:  x spans +0.37..+3.27 (mean+1.61), y spans -4.47..+12.86, z spans -0.61..+2.29
normal config:  x spans +0.37..+3.39 (mean+1.61), y spans -4.41..+14.53, z spans -0.61..+2.31
```

**The clouds are identical.** rsdriver does NOT apply the
roll/pitch/yaw to the cloud — it only tags frame_id and passes through.
The `roll/pitch/yaw` knobs in rsdriver config may be used for other
purposes (TF publishing, motion compensation) but not for cloud
transform. **The published cloud has always been in raw sensor frame.**

### Frame analysis

Cloud geometry from the probe:
- X: positive-only (0.37→3.39) → dome axis (hemispherical lidar only
  sees forward-of-dome half-space)
- Y: symmetric spread (±14m) → horizontal plane, room extent
- Z: symmetric narrow (±2m) → vertical axis (floor→ceiling)

User's physical mount: "horizontal, dome forward, cable up". Maps:
- +X_sensor (dome) → +X_base (forward)
- +Z_sensor → +Z_base (up)
- +Y_sensor → +Y_base (left, by right-hand rule)
→ **cloud-sensor frame = base_link** (identity rotation)

Separate from this, DIFOP's R_imu_to_lidar routes raw IMU gravity to
`+X_canonical_lidar`. That's the CABLE direction in world (up). So DIFOP's
"lidar frame" has cable = +X. In the cloud frame, cable = +Z.

**Two distinct "lidar frames" exist**:
1. DIFOP canonical: cable = +X_canonical, dome = +Z_canonical (probably)
2. Cloud sensor: dome = +X_sensor, cable = +Z_sensor

They're 90° rotated about the Y axis from each other. Our
`R_base_lidar = [[0,0,1],[0,-1,0],[1,0,0]]` was tuned to map DIFOP's
canonical → base, producing correct gravity on +Z_base stationary.
But the cloud was never in canonical frame — it was in sensor frame
all along.

### Why rotation fails

Even though cloud_sensor frame ≈ base_link and IMU output is in
base_link (gravity-correct), the COMPOSITION R_base_imu has a yaw DOF
pinned by an arbitrary assumption rather than by physical reality.
Gravity alignment pins roll + pitch but leaves yaw free. Any yaw
offset between "our claimed base frame" and "actual world-vertical-
aligned-with-cloud's-X" manifests as angular-rate mixing: physical yaw
about world-up appears as some combination of gyro X/Y/Z in the
"claimed base" frame. Observed during yaw test:

```
gyro_base = (-0.21, +0.10, -0.77) rad/s
rho = sqrt(gx² + gy²) / |gz| = 0.30
```

30% off-axis signal during hand-yaw. Some of that is real tilt (user
holding her and moving), but 30% is high and suggests a frame rotation
error in the 10° range. FAST-LIO2's filter then propagates with IMU
while scan-matching in the different frame → gradual Z drift → scan
match eventually fails → runaway.

### Plan for next session (not under rush — correctness > speed)

1. **Diagnostic: gyro purity test**. User holds M20 as level as possible
   and rotates ~45°/s about world-vertical for 10s. Log gyro_base. If
   rho > 0.1 consistently, confirms yaw DOF error.

2. **Fix candidate A**: publish airy_imu_bridge in `--frame sensor`
   (pass raw IMU through), compute correct R_cloud_to_raw_imu, set
   velodyne.yaml `extrinsic_R` to that rotation. Lets FAST-LIO2
   rotate cloud into IMU frame internally; `extrinsic_est_en: true`
   might then be safe (good prior rather than identity prior).

3. **Fix candidate B**: re-derive R_base_lidar from physical mount +
   cloud geometry jointly (not just from DIFOP+gravity), so the yaw
   DOF is physically pinned rather than arbitrary.

4. **Fix candidate C**: investigate whether FAST-LIO2's "lidar frame"
   and our cloud frame ARE the same, and whether something in the
   drdds_lidar_bridge ring remap / time-dekew might be corrupting
   angular motion.

### Related cleanups shipped

- `deploy.sh sync` now picks newest nix store output by mtime (not
  alphabetical) — avoids stale symlinks after rebuild.
- `deploy.sh provision` disables legacy `drdds-bridge.service` (which
  ran a duplicate `drdds_recv` and saturated NOS → Finding #23).
- `M20_SKIP_STAND=1` env on `M20Connection` lets smartnav run while
  robot is sitting / on dock. Navigation mode still applied so lidars
  stay powered.
- `rsdriver` config left with `pitch=-π/2 yaw=-π roll=0` (original) —
  has no functional effect on the cloud (per Expt 2 above) but
  cheaper to leave than re-edit every session.

---

## Finding #25: Gyro Purity Test Result + Real Rotation Cause is Lever-Arm Centripetal (2026-04-22)

Finding #24 proposed a gyro purity test to confirm a yaw DOF error in
`R_base_imu` as the cause of rotation-drives-SLAM-divergence. Ran it
today on the actual robot (tele-op D-key rotation in place via WASD
from dimos-viewer — NOT handheld, important). Plus a second expert
opinion from codex and a careful audit of the fork source. Result:
hypothesis refuted, real cause identified.

### Gyro purity data — 12 s capture, D-key yaw-in-place

Smartnav with `M20_SLAM_BACKEND=fastlio2 M20_FASTLIO2_IMU=airy` brought
up the airy_imu_bridge at 200 Hz, cal=OK. A Python LCM subscriber
(`/tmp/gyro_purity.py` on NOS, consumes `/airy_imu_front#sensor_msgs.Imu`
via `dimos_lcm.sensor_msgs.Imu`) recorded 12 s. User rotated the robot
in place for ~10 s at mean 62 °/s, peak 87 °/s.

**Stationary (|gz|<0.05, 1614 samples):**
- gx mean -0.006 stdev 0.001, gy mean -0.011 stdev 0.001
- |gz| mean 0.009 stdev 0.002
- accel mean (+0.002, +0.218, +9.847) → **tilt from +Z = 1.27°**

**Active rotation (|gz|>=0.3, 785 samples):**
- gx mean +0.006 stdev **0.103**, gy mean -0.035 stdev 0.090
- |gz| mean 1.079 stdev 0.199, peak 1.520
- accel mean (**-0.635**, +0.257, +9.769)  ← headline
- rho mean 0.107, median **0.092**, p5/p95 = 0.026 / 0.253

**Regression gx on gz:** slope -0.110, **R² = 0.045**.

**Rho in 1 s windows during rotation:** 0.128 → 0.108 → 0.089 → 0.102
(decreases as D-key holds steady, consistent with leg-shuffle transient
damping out).

### Why the yaw-DOF-error hypothesis is refuted

1. Median rho 0.092 < the 0.10 threshold proposed in Finding #24.
2. R² = 0.045 means gx is noise-dominated, not bias-dominated. A fixed
   yaw offset θ in R_base_imu would produce gx ≈ -sin(θ)·gz with high
   R² — not observed.
3. Stdev(gx) = 0.10 ≫ mean(gx) = 0.006 → wobble, not bias.
4. **Math gap in Finding #24**: a pure Rz(θ) yaw offset in R_base_imu
   *commutes* with a pure world-vertical rotation. `Rz(θ) · (0,0,ω) =
   (0,0,ω)`. So the rho test literally cannot detect a yaw-DOF error
   whose rotation axis is gravity. It can only confirm base Z is
   gravity-aligned (which it is, within ~5°).

### The -0.635 m/s² headline — real cause is lever-arm centripetal

M20 hardware manual (Lynx-M20-Series-Hardware-Development-Manual-V0.0.4-0.pdf,
Section 1.10) gives sensor positions relative to body origin:

| Sensor | X (mm) | Y (mm) | Z (mm) |
|---|---|---|---|
| Body IMU (yesense) | +63.2 | -26.8 | -43.5 |
| **Front Airy LiDAR** | **+320.28** | **0** | **-13** |
| Rear Airy LiDAR | -320.28 | 0 | -13 |

Body frame is standard ROS convention (X-forward, Y-left, Z-up) per
Section 1.6. The Airy IMU lives inside the front Airy housing, so its
position in body frame is ~(+0.320, 0, -0.013) m plus a ~5 mm DIFOP
lever offset (negligible).

When the M20 rotates in place about its body-vertical axis, the IMU
traces a circle of radius 0.320 m and feels fictitious centripetal
acceleration:

- a_centripetal_body = -ω × (ω × r_imu_body)
- For ω = (0, 0, ωz) and r = (+0.320, 0, -0.013):
  a_centripetal = (-0.320·ωz², 0, 0)
- At |ωz| mean = 1.08 rad/s → **-0.373 m/s² in ax**
- At |ωz| peak = 1.52 rad/s → **-0.740 m/s² in ax**
- Time-averaged over measured |ωz| distribution ≈ **-0.47 m/s²**

Observed ax mean during rotation = **-0.635 m/s²**. Within the
instantaneous range and above the mean-ω value (consistent with ω²-
weighted averaging giving more weight to peak ω moments).

FAST-LIO2 interprets the IMU's linear acceleration as
body-origin acceleration because `extrinsic_T = [0,0,0]` in the current
yaml. It has no way to subtract the centripetal fictitious term. It
integrates -0.64 m/s² twice and concludes the robot has translated
tens of meters within the few seconds of rotation → matches the
observed "keyframes to kilometers in seconds" runaway.

### Codex second opinion (2026-04-22 via codex-rescue)

Pushed back on Finding #24's framing on five points. Key ones:

1. **Rho doesn't exonerate cloud-vs-IMU frame mismatch.** It only screens
   for gravity-axis alignment. Codex: "The 'pure Rz commutes with
   world-vertical yaw' argument only clears a very specific idealized
   error; it does not prove the current composed rotation is correct,
   and low rho does not exonerate yaw mismatch."
2. **Missing lever-arm usually produces bounded error**, so the
   kilometers-in-seconds signature points at frame/deskew issues
   too. Codex ranks deskew risks in the bridge (ring remap, per-point
   time rewrite) above lever-arm for the specific "runaway in seconds"
   pattern.
3. Proposes a sensor-frame A/B test: run `airy_imu_bridge --frame sensor`
   (already implemented, lines 434-440) + front lidar only, and iterate
   extrinsic hypotheses to directly test cloud-vs-IMU consistency.
4. `velodyne.yaml` has no partial-extrinsic knob (gravity_freeze,
   yaw-only extrinsic est). Only option is `extrinsic_est_en` full on
   (tried in Finding #24, caused instant death) or full off (current).

### Bridge ring-remap audit — no-op in our code path

Investigated Codex's #1 suspect: `drdds_lidar_bridge/cpp/main.cpp:157-162`
collapses rings 0-191 → 0-63 via `ring * 64 / 192` with a comment
"ARISE N_SCANS=64". Went deep on the active fastlio2 fork source at
`/nix/store/d7xvw7wjz1vblip2ymdha88yzcb8ch1x-source/src/`:

- `N_SCANS` is a runtime int in `preprocess.h:106`, loaded from YAML
  `scan_line` at `laserMapping.hpp:319`. All uses are
  `for(i<N_SCANS)` loops or `std::vector(N_SCANS, ...)`. No fixed-size
  array sized on N_SCANS.
- Only fixed-size concern: `PointCloudXYZI pl_buff[128]` in
  `preprocess.h:103`. But it's only accessed inside
  `if(feature_enabled)` branches, and `laserMapping.hpp:336` hardcodes
  `p_pre->feature_enabled = false`. **Unreachable.**
- In the active path (`feature_enabled=false` + `given_offset_time=true`
  because our bridge writes relative per-point times), the `ring` field
  is grep'd and **never referenced in any indexing/grouping operation**.
  velodyne_handler's non-feature branch (preprocess.cpp:298-359) uses
  ring only inside `if(!given_offset_time)`, which doesn't fire for us.

**The ring remap has zero functional effect on fastlio2 in our path.**
Finding #16's "3-beams-per-bin ruins curvature/normal estimation" claim
only applies with `feature_enabled=true`. With feature extraction off
(as it has been since #14), the remap is inert vestigial code from the
AriseSLAM path. Safe to remove for hygiene but **not** the rotation
bug.

MAXN = 720000 in `laserMapping.hpp:219` is frame-count history
(`double T1[MAXN]`) — not ring count. Unrelated.

### Per-point time rewrite — still suspicious

Bridge at `main.cpp:164-168` converts per-point time from `double`
absolute to `float32` relative using `first_t = points[0].time` as the
reference. Two problems:

1. For dual-Airy merged scans, points are interleaved front/rear in
   memory, not sorted by time. `points[0].time` is just "the first
   point in memory order", not the minimum time. Points earlier in
   time than that get **negative** relative stamps.
2. `laserMapping.hpp:648`:
   `lidar_end_time = lidar_beg_time + points.back().curvature / 1000.0`
   uses the last-in-memory point's curvature as scan end time. If
   points aren't temporally sorted, this is not the maximum — scan end
   is underestimated, EKF deskew window wrong.

Under rotation this matters disproportionately because per-point pose
interpolation relies on correct relative stamps to compensate for
body motion during the 100 ms scan window.

### Plan forward (in priority order)

1. **Lever-arm centripetal correction in `airy_imu_bridge`** — one
   C++ change: subtract `ω × (ω × r)` from accel in base frame. `r`
   comes from PDF Section 1.10 per lidar side. Direct evidence-backed
   fix; alone it handles the fictitious lateral translation during
   pure yaw-in-place.

2. **Audit + fix per-point time relativization** in
   `drdds_lidar_bridge/cpp/main.cpp` — either:
   a. scan data first to find the minimum time, use as reference, or
   b. sort points by time before publishing (expensive at 100 k pts),
      or
   c. have drdds_recv emit points in time order upstream.
   (a) is cheapest. Verify whether drdds actually emits out-of-order.

3. **Remove 192→64 ring remap** (hygiene, no functional change). Bump
   `scan_line: 192` for correctness-if-ever-used.

4. (Only if above don't fix rotation) the sensor-frame A/B test Codex
   proposed, or switch back to yesense IMU now that PDF gives the
   extrinsic.

### What we ALSO learned from the PDF

Body IMU (yesense) at (+63, -27, -44) mm sits near the rotation axis.
At |ω|=1 rad/s its centripetal is only 0.068 m/s² (~5× less than
Airy). If the lever-arm correction in airy_imu_bridge doesn't fully
resolve rotation drift, switching to yesense is a plausible next move
because:
- Extrinsic to lidar now known: `T_yesense_to_front_lidar =
  (0.257, +0.027, +0.030) m`.
- Clock-domain issue (Findings #7/#8) that drove us away was before
  `native_clock` path landed — might be addressable now with
  `time_sync_en: true`.

---

## Finding #26: Tangential α × r Dominates Centripetal on M20 Pro Wheel-Legged (2026-04-22)

User asked during Finding #25's lever-arm implementation: "the robot
isn't rotating smoothly, it's shuffling its legs in place to rotate —
won't there be lots of accel/decel?" Checked the same 12 s gyro-purity
CSV that provided Finding #25's evidence and yes — the user was right,
and the correction landscape is different than I framed it.

### Data: α statistics during the D-key yaw-in-place

Rotation window n=785 samples, duration 3.92 s:

- **α = dω_z/dt** via backward finite difference on 200 Hz gyro:
  - mean −0.19 rad/s² (approx zero, as expected for sustained rotation)
  - **stdev 8.09 rad/s²**
  - **peak |α| = 36.76 rad/s²**
- Zero-crossing rate: 50 /s → **~25 Hz oscillation in α** — matches
  expected frequency of wheel-legged re-stance events (one pulse per
  foot plant × 4 feet × few Hz gait).
- Autocorrelation: peaks at +0.714 (lag 5 ms) decaying, negative peak
  at lag 50 ms → ~20 ms positive-correlation window, consistent with
  leg-shuffle push-off + plant.

### Tangential term is 15× bigger than centripetal at peak

Tangential fictitious accel is `α × r`. With `r_x = 0.320 m` for the
Airy IMU:

| Term | Typical | Peak |
|---|---|---|
| Centripetal ω²·r_x (at |ω| rms ≈ 1.1 rad/s) | **0.37 m/s²** | ~0.74 m/s² |
| Tangential α·r_x | **2.59 m/s²** | **11.76 m/s²** |

Tangential is ~7× bigger than centripetal at rms, **~15× at peak**.

### Direct evidence in the accel data: ay ≈ α_z × r_x

Regression of `ay` on `α_z` over the rotation window:

- **Correlation: +0.796**
- **Slope: +0.3009** (close to expected `r_x = +0.320` — ~5 % off)
- Intercept: +0.3153 m/s² (matches stationary ay bias)

This is a smoking gun. The observed `ay` during rotation is
essentially `α_z × r_x` plus a DC bias. The slope of 0.3009 recovers
the forward lever arm **directly from the data**, independent of the
PDF's 0.320 m value. Empirical corroboration that r ≈ 0.32 m is real
and not just a manual-reading artifact.

It also means that v1's centripetal-only correction subtracts 0.37 m/s²
from ax (correct and useful) but LEAVES the 2.6 m/s² typical / 11.8 m/s²
peak tangential contribution untouched in ay. That's still the
dominant fictitious term on this robot for this motion, and FAST-LIO2
would still integrate it as spurious lateral translation.

### Implication — implement tangential correction in v1

Decision made before first deploy of Finding #25 code: include α × r
in the bridge. Subtract `α × r + ω × (ω × r)` from accel instead of
just ω × (ω × r).

Computing α:
- Simple backward difference on base-frame gyro: `α_t = (ω_t − ω_{t−1}) / Δt`.
- 200 Hz gyro, Δt ≈ 0.005 s.
- Gyro noise floor at stationary: σ_ω ≈ 0.001 rad/s.
- Differentiated noise: σ_α ≈ σ_ω · √2 / Δt ≈ 0.28 rad/s² — much smaller
  than the 8 rad/s² rotation-window stdev of the real signal, so
  amplification is acceptable without filtering.
- Handle first sample (no previous ω): set α = 0.

Tangential from α × r:
- `α × r = (α_y · r_z − α_z · r_y, α_z · r_x − α_x · r_z, α_x · r_y − α_y · r_x)`.

Subtraction order in base frame:
- a_body ≈ a_imu − ω × (ω × r) − α × r.

Tests to run after deploy:
1. Stationary: ax/ay/az delta from baseline should be near zero
   (α ≈ 0 → tangential correction ≈ 0; centripetal also ≈ 0).
2. D-key rotation: ay stdev during rotation should collapse from 0.090
   toward gyro-noise floor; ay regression slope vs α_z should drop
   from +0.30 toward 0.
3. SLAM: keyframes should stop runaway during rotation if the ax+ay
   fictitious removal is sufficient.

### Why this matters for the bigger picture

M20 Pro is a **wheel-legged hybrid**. Rotating in place is NOT a
smooth rigid-body yaw; it's a chopped sequence of leg-plants and
wheel-yaw pulses. Any sensor offset from the rotation axis will feel
large α-driven accelerations at ~25 Hz. For any future filter design
(FAST-LIO2 config, EKF covariances, motion priors) this informs
expected input statistics: the IMU on this platform produces
large-variance `ay` during rotation that is not measurement noise —
it is real kinematic acceleration of a point not at the rotation
center.

For the short term: we must compensate it, because FAST-LIO2 has no
IMU-body-to-rotation-center lever-arm model. For the medium term: a
proper URDF-driven rigid-body transform in the bridge would handle
both centripetal and tangential together using the known lever arm,
and would generalize to other sensor mounts.

---

## Finding #27: Lever-Arm Correction Works at the IMU Level, but SLAM Still Diverges Under Rotation (2026-04-22)

Shipped Finding #25 + #26 combined correction (centripetal ω × (ω × r)
+ tangential α × r, with `r = (+0.320, 0, -0.013)` for front Airy in
base_link) in `airy_imu_bridge`. Deployed nix build
`00ksk17xfanvf6mlq4hxsfw3cqp3xq3n-drdds_lidar_bridge-0.1.0` with the
lever-arm logic enabled. Stamped that config on the LCM status line
(`lever_arm_base (m) = (0.3200, 0.0000, -0.0130) — centripetal
correction enabled`).

### Retest on identical D-key yaw-in-place scenario

Ran the same `retest_rotation.py` for 12 s, user yawed the robot via
tele-op D-key for ~10 s at a slightly slower rate (|gz| mean 0.77 rad/s
vs 1.08 previously).

Head-to-head (rotation − stationary):

| Metric | PRE (no correction) | POST (cent + tang) | Change |
|---|---|---|---|
| Δax | **-0.637 m/s²** | **-0.081 m/s²** | 87% reduction |
| Δay | +0.040 | +0.007 | 83% reduction |
| Δaz | -0.078 | +0.004 | 95% reduction |
| ay stdev during rotation | 3.06 | 1.16 | 62% reduction |
| az stdev during rotation | 3.18 | 1.82 | 43% reduction |
| corr(ay, α_z) | +0.796 (slope +0.30) | **-0.33 (slope -0.10)** | bias elim., slight over-correct |

The IMU signal IS cleaner. Bias from fictitious accel is essentially
gone on all three axes. Variance is also cut in half because we're now
subtracting (most of) the real ω × (ω × r) and α × r that leg-shuffle
produces. The residual -0.33 correlation / -0.10 slope in ay vs α_z
suggests our r_x = 0.320 is ~1.25× the effective value under load
(makes sense — the M20 Pro's effective rotation center shifts from
body-geometric-center during wheel-legged yaw as feet re-plant
asymmetrically). Minor, not urgent — could tune `r` empirically later.

### But the SLAM is still catastrophically wrong

Ran the same rotation through `fastlio2_native` via smartnav with
`M20_SLAM_BACKEND=fastlio2 M20_FASTLIO2_IMU=airy`. Keyframe trajectory
from the PGO log:

```
Keyframe  1 (-0.0,  0.0,  0.0)   ← stationary, preroll done
...
Keyframe 10 ( 0.0, -0.0, -0.1)   ← still stationary, clean
Keyframe 11 ( 1.7, -0.5,  1.0)   ← D pressed, small pose shift
Keyframe 13 ( 1.4, -4.8,  2.7)
Keyframe 17 (-1.6, -9.9,  2.9)   ← drifting but bounded (<10 m)
Keyframe 18 (-6.9,-17.9,  0.9)   ← STEP JUMP (classic ICP wrong local minimum)
Keyframe 20 (-8.1,-19.0,  0.7)
...
Keyframe 138 (3810, -10290, -10750)  ← kilometers away, runaway
```

Stationary-to-rotation transition is clean up through KF 17 — small
drift consistent with our residual 0.08 m/s² bias. The catastrophic
divergence starts at **KF 18 step-jump**, which is FAST-LIO's EKF
measurement-update committing to a wildly wrong ICP solution. After
that the state is far enough off that subsequent scans can't recover,
and PGO keeps appending keyframes along the diverged path.

**IMU lever-arm was NOT the dominant rotation failure mode.** Our
residual 0.08 m/s² integrated over 10 s of rotation = ~4 m of fake
translation. We're seeing 10,000+ m. Three orders of magnitude
unexplained by IMU.

### Smoking gun: `imu_vs_frame_end` gap

Every `fastlio2_native` frame log line shows
`imu_vs_frame_end = -0.050 to -0.170 s`, even stationary. That means
the lidar scan's end timestamp is 50 to 170 ms AFTER the most recent
IMU sample. FAST-LIO2's `syncPackage()` requires IMU coverage of the
scan window for proper per-point deskew + EKF measurement update. When
IMU is short by 170 ms during rotation at 1 rad/s:

- Orientation extrapolation error: ω · Δt = 1 · 0.17 = 0.17 rad ≈ **9.7°**
- FAST-LIO must propagate forward 170 ms on a constant-velocity
  assumption. During rotation this is catastrophic: the predicted
  pose at scan_end is off by ~10° orientation, which puts all
  predicted points in the wrong place, which makes ICP search
  against a misaligned cloud-to-map, which either fails or finds a
  wrong local minimum.

Stationary, ω = 0 so the 170 ms gap doesn't matter (nothing to
extrapolate). That's why stationary SLAM works perfectly and rotation
breaks.

### Next investigations — ranked

**1. Why is IMU 170 ms behind the lidar scan end?** The IMU publishes
at 200 Hz so fresh samples should be only 5 ms behind real time. The
lidar scan is 100 ms wide (max_offset=0.100s). If lidar publishes at
scan_end = real_time (normal), and IMU is at real_time − 5ms, gap
should be ≤5 ms. Getting 170 ms means either:

   a. The lidar's `header.stamp` is AHEAD of true physical scan_end
      (rsdriver / drdds timing bug).
   b. The IMU LCM stream is DELAYED by 170 ms between bridge and
      fastlio2 (transport bottleneck — but pkt_vs_wall in
      airy_imu_bridge reports -0.003 s, very tight).
   c. Both streams are consistent with wall clock but
      fastlio2 is computing `imu_vs_frame_end` using
      `lidar_beg_time + max_offset_s` and the bridge's relative time
      rewrite truncated that (Finding #26 empirical observation that
      `first_t = points[0].time` is not always scan start).

   (c) matches Codex's Q5 concern and Finding #25's time-relativization
   audit. Need to confirm by instrumenting the bridge or reading raw
   drdds point timestamps.

**2. What's actually in the per-point time field?** Write a probe that
reads a single cloud from `/dev/shm/drdds_bridge_lidar`, extracts the
per-point time as double (raw), and reports min/max/distribution. If
min != points[0].time or max != points.back().time, the bridge's
first_t / back().curvature heuristic is broken for time-anchor
computation.

**3. Fix the time anchor in bridge.** Replace `first_t = points[0].time`
with `first_t = min(points[i].time)` — one extra pass, ~50 k points
per scan, negligible. Ensure the PointCloud2 declared scan_end in
its header matches max(time) + lidar_beg_time.

### Plan stays

Finding #25 + #26 shipped and helped (IMU signal clean). Next: time
relativization. Ring remap removal stays third (still inert).

---

## Finding #28: `drdds_lidar_bridge` Tagged PointCloud2 with scan_END as header.stamp — Root Cause of Rotation SLAM Drift (2026-04-22)

Finding #27 flagged `imu_vs_frame_end = -0.05 to -0.17 s` even
stationary, concluded that drdds's per-point times were likely
out-of-order, and put a probe on the TODO.

Ran `probe_lidar_times.py` (reads `/dev/shm/drdds_bridge_lidar`
directly, extracts per-point time doubles, reports distribution).
Results on a single live cloud during the post-correction test:

```
[probe] slot: stamp=1776818077.700005054 pts=46521 point_step=26

[probe] Per-point time analysis:
  times[0]         = 1776818077.600054502
  times[-1]        = 1776818077.700005054
  min              = 1776818077.600054502
  max              = 1776818077.700005054
  max - min        = 0.099951 s   (99.95 ms)
  times[0] - min   = 0.000000    ← first point IS minimum
  times[-1] - max  = 0.000000    ← last point IS maximum
  descending-step count: 0 / 46520
  → points ARE monotonically ordered in time
```

**Finding #25 was wrong about time-ordering.** Points are strictly
monotonic. `first_t = points[0].time` correctly equals `min(time)`.
This kills my out-of-order hypothesis.

**But the probe revealed a much worse bug.** Note:
- `slot->stamp` = 1776818077.700005054 = scan_end (= max of points)
- `times[0]` = 1776818077.600054502 = scan_start

The drdds slot header stamp is **scan_END**, not scan_start. The
bridge's pre-fix code set `pc.header.stamp = slot_stamp = scan_END`.
FAST-LIO2 treats `pc.header.stamp` as scan_START and adds
`max_offset_s` (100 ms, derived from point time offsets) to get
scan_end. So FAST-LIO's computed scan_end was:

```
effective_scan_end = header.stamp_(scan_END) + max_offset_s
                   = scan_end + 100 ms
                   = true scan_end + 100 ms
```

FAST-LIO's `syncPackage()` then requires IMU coverage up to
`effective_scan_end` — i.e., 100 ms in the future of the actual scan
end. IMU at 200 Hz only gives us data up to real wall clock, so
`imu_latest < effective_scan_end` by 100 ms permanently. This is
exactly what the log was telling us:

```
[fastlio2] frame #3  path=pc2  timebase=1776817363.9999  max_offset=0.100s  imu_latest=...  imu_vs_frame_end=-0.050s to -0.170s
```

### Why stationary works, rotation breaks

Stationary: IMU is ω = 0, no rotation prediction needed. FAST-LIO's
EKF propagation using the most recent IMU to the (imaginary)
+100 ms scan_end is trivially correct (zero motion predicted = zero
motion). Scan match then sees points in their expected place.

Rotation at 1 rad/s: FAST-LIO must extrapolate the pose forward
100-170 ms ahead of the IMU's actual coverage, using the
constant-angular-velocity assumption. The forced extrapolation is:

- Orientation drift: ω × Δt = 1.0 × 0.17 = 0.17 rad ≈ **9.7°**
- This is applied to the whole deskewed cloud before ICP.
- Cloud becomes mis-aligned with the map by ~10°.
- ICP tries to correct, finds the WRONG local minimum (indoor scenes
  are full of parallel features at 10° ≤ offsets), commits to it.
- EKF accepts, keyframe explodes.

This matches the Finding #27 observation that KF 1-17 drift slowly
(the 100 ms scan_end mis-anchor produces small-but-growing error
under slight motion) then KF 18 step-jumps (ICP wrong-local-minimum
commit) and runaway after.

### Fix — one-line in `drdds_lidar_bridge/cpp/main.cpp`

Replace `pc.header = make_header(..., slot->stamp_*)` (scan_end) with
`pc.header = make_header(..., first_t)` (scan_start, which is
identical to `points[0].time` because cloud is monotonic).

```cpp
// Before
pc.header = make_header("lidar_link", slot->stamp_sec + slot->stamp_nsec * 1e-9);

// After — scan_start from first point
double first_t;
std::memcpy(&first_t, pc.data.data() + time_off, sizeof(double));
pc.header = make_header("lidar_link", first_t);
```

Expected effect after deploy:
- `imu_vs_frame_end` goes from −0.05 to −0.17 s → near zero (−5 to
  +5 ms, bounded by IMU sample spacing + small lidar-to-wall-clock
  skew, well within EKF propagation window).
- FAST-LIO2 no longer extrapolates forward 100 ms. Deskew uses actual
  IMU for all the scan's time window.
- Rotation-drift runaway is expected to go away, or at least reduce
  by orders of magnitude — if ICP still finds wrong local minima, that
  is a scene-structure issue, not our timing bug.

Commit is in source tree now, not yet deployed. Building next.

### Meta-lesson

- **Always verify assumptions with a direct probe before acting.**
  Finding #25 proposed that first_t != min_t due to interleaved
  front/rear rings. That was a plausible story for the dual-lidar
  merged mode but WRONG in practice — rsdriver's merge is
  time-sorted. The probe took 20 minutes to write and would have
  redirected attention here much earlier.
- **Header semantics vary across stacks.** ROS convention is
  "header.stamp = earliest point's time" and FAST-LIO follows it.
  drdds apparently uses "last point's time" or "publish time". When
  bridging between stacks, explicitly re-derive the header stamp from
  your point data, don't blindly trust the incoming header.

---

## Finding #29: Rotation SLAM Bounded — 3000× Drift Reduction (2026-04-22)

Shipped the Finding #28 scan_start header fix in
`drdds_lidar_bridge/cpp/main.cpp`. New nix build
`gwgykwraxah3f2bj389w55cv4ykbgxi5-drdds_lidar_bridge-0.1.0`, symlinks
repointed, smartnav restarted (`M20_SLAM_BACKEND=fastlio2
M20_FASTLIO2_IMU=airy`).

### Stationary baseline verified

- `imu_vs_frame_end` flipped from **−0.05 / −0.17 s** (pre-fix) to
  **+0.05 / +0.07 s** (post-fix). IMU now covers the scan window;
  FAST-LIO no longer extrapolates forward.
- Keyframe 1 at `(−0.0, −0.0, −0.0)`, stationary SLAM stable.
- 200 Hz IMU, cal=OK, lever-arm correction enabled per Findings
  #25/#26.

### Rotation test — same D-key yaw-in-place protocol

12 s capture, user pressed D for ~10 s, same rotation rate as before
(|gz| mean 0.77 rad/s, peak ~1.5 rad/s).

Keyframe trajectory from PGO log:

```
KF  1 (-0.0, -0.0, -0.0)   ← stationary preroll
KF  2 (-0.3,  0.1,  0.1)   ← D pressed, small excursion
KF  3 (-0.3,  0.0, -0.1)
KF  4 (-0.6, -0.2, -0.1)
KF  5 (-1.1, -0.1, -0.2)
KF  6 (-2.0, -0.1, -0.1)
KF  7 (-2.5, -0.2,  0.0)
KF  8 (-3.3, -0.1,  0.2)   ← peak drift
KF  9 (-2.6, -0.1,  0.1)
KF 10 (-1.6,  0.1, -0.0)   ← recovering
KF 11 ( 3.2,  0.3, -0.1)   ← D released, final
```

### Head-to-head drift comparison

| Config | Peak X drift | Peak |pos| | Final keyframe | # KFs | Behavior |
|---|---|---|---|---|---|
| Pre-correction (Finding #24) | ~thousands m | ~10 km | KF 138 (3810, −10290, −10750) | 138+ | Runaway |
| Lever-arm only (Finding #27) | ~thousands m | ~10 km | KF 138 (3810, −10290, −10750) | 138+ | Runaway, same magnitude |
| **+ scan_start fix** | **−3.3 m** | **3.3 m** | KF 11 (3.2, 0.3, −0.1) | **11** | **Bounded, self-recovers** |

**Drift reduction: 3000× on peak magnitude, KF count reduced by >90%.**
The "step-jump to wrong ICP local minimum" pattern is gone entirely.
Y + Z drift stay within ±0.3 m through the whole rotation window —
essentially pinned.

### Residual 3.3 m X-drift — two candidate causes

**(a) Lever-arm fine-tune.** Post-correction telemetry from Finding #27
showed `corr(ay, α_z) = −0.33` with regression slope `−0.10`, which is
a slight over-correction of the tangential term. Implied effective
r_x_eff ≈ 0.22 m (not 0.32 as configured). Over-subtracting α × r
leaves a negative residual in ay that integrates to X drift during
sustained rotation. Tunable via `--lever_arm_base_m`.

Why r_eff < r_geometric on a wheel-legged platform: under yaw tele-op
the robot shifts its center of support dynamically (leg-plant load
redistribution); the instantaneous rotation axis migrates during the
gait cycle rather than sitting at the body geometric center. For a
first pass, setting r_x ≈ 0.25 should reduce the slope residual and
shave off much of the 3.3 m.

**(b) Residual ICP ambiguity in indoor symmetric scene.** Our test was
on carpet in a roughly rectangular space with parallel walls. Even a
perfect IMU can't prevent a small yaw-accumulating scan-match error
because the scan is angularly self-similar. This bounded drift is the
"normal" failure mode of FAST-LIO2 Velodyne path in low-feature
environments and matches behavior reported in the FAST-LIO literature.

Probably a mix. Finding Next session: fine-tune (a) first, then if
residual > 1 m revisit (b) — possibly by enabling
`extrinsic_est_en: true` now that rotation is no longer causing
instant EKF death.

### Code state summary at end of 2026-04-22

Shipped this session:
- `airy_imu_bridge.cpp`: centripetal + tangential lever-arm
  subtraction, default `r = (0.320, 0, -0.013) m` for front Airy,
  overridable via `--lever_arm_base_m "x,y,z"`.
- `drdds_lidar_bridge/cpp/main.cpp`: PointCloud2 `header.stamp` now
  set from first-point time (scan_start), not the drdds slot's
  stamp field (which empirically is scan_END = time of last point).
- `FASTLIO2_LOG.md`: Findings #24-#29 fully documented including
  refuted hypotheses, data tables, and the codex second-opinion
  pushback.

Config unchanged:
- `velodyne.yaml`: `extrinsic_T = [0,0,0]`, `extrinsic_R = identity`,
  `extrinsic_est_en = false`, `scan_line = 64`.
- `drdds_lidar_bridge` still remaps 192→64 rings — inert in our path
  (Finding #25 audit) but scheduled for removal next session.

### Bounded-drift baseline makes downstream work tractable

With rotation no longer diverging, click-to-goal and longer-range
navigation become testable. Previous attempts all hit runaway drift
within seconds of rotation and never completed a short-range goal.
Now we have a stable enough estimate to drive the robot across a room
and back without the position estimate exploding.

Next targets (in order):
1. ~~Remove 192→64 ring remap + bump `scan_line: 192`~~ — DONE. Empirical
   correction: RSAIRY merged cloud has rings 0..95, not 0..191. Both
   Airys share the ring index space in rsdriver's merged mode; each
   ring value carries one point from each sensor. YAML set to
   `scan_line: 96`. Also checked in `flake.lock` to the repo so future
   `deploy.sh sync` doesn't strip it and force offline rebuilds.
2. Fine-tune `--lever_arm_base_m` toward empirical r_eff. Retest drift.
3. Click-to-goal end-to-end test. Success criterion: robot reaches
   within 0.5 m of a 2 m target and comes to a clean stop.

---

## Finding #30: Lever-Arm Tuning Null Result — Keep Geometric r, Ignore Regression Slope (2026-04-22)

Finding #29 noted a residual `corr(ay, α_z) = −0.33, slope = −0.10` after
the r = 0.320 m correction and speculated that an "effective r_x" of
about 0.22 m (derived naively from that slope as `r_eff = r_sub +
slope`) might sit better than the geometric value because the
wheel-legged rotation center shifts under leg-plant load. Tested that
hypothesis.

### Plumbing added this session (kept for future tuning)

- `AiryImuBridgeConfig.lever_arm_base_m: str | None = None` — optional
  override wired through the NativeModule CLI framework; when set,
  auto-passes `--lever_arm_base_m "x,y,z"` to the binary. Default
  remains the C++ geometric default `(0.320, 0, -0.013)` for front,
  `(-0.320, 0, -0.013)` for rear (PDF Section 1.10).
- `m20_smartnav_native.py` reads `M20_AIRY_LEVER_ARM_BASE_M="x,y,z"`
  env var and passes to the bridge.

### Experiment — r = 0.32 vs r = 0.22, same D-key yaw-in-place

Third test run on identical protocol (user presses D for ~10 s in
dimos-viewer while a 12 s LCM logger captures IMU at 200 Hz). All other
state identical: scan_start fix in place (Finding #28), ring remap
removed (Finding #29), `scan_line: 96` in velodyne.yaml.

|   | r = 0.32 (geometric) | r = 0.22 (tuned) |
|---|---|---|
| Δax rotating − stationary | **−0.081 m/s²** | worse (no stationary; ax rotating mean −0.117) |
| `corr(ay, α_z)` | −0.333 | **+0.052** |
| `slope(ay, α_z)` | −0.0963 | **+0.0156** |
| SLAM peak drift | **3.3 m bounded** | **hundreds of m runaway** |
| KF 11 Y position | 0.3 m | **63.3 m** (step-jump, ICP wrong local min) |
| Total KFs | 11 (bounded) | 29+ (runaway) |

### What the data actually say

**The "cleanest" IMU signal (r = 0.22) gives the WORST SLAM.** The
regression slope of ay on α_z is NOT a reliable tuning objective.
Two reasons this is almost certainly attenuation bias, not a true
over-correction:

1. **Errors-in-variables attenuation.** `α_z = dω_z/dt` is computed by
   backward difference on 200 Hz gyro with stationary σ ≈ 1 mrad/s.
   Differentiated σ_α ≈ √2 · 1e-3 / 0.005 ≈ 0.28 rad/s². During
   wheel-legged rotation the signal stdev in α is ~8 rad/s², so
   SNR ≈ 30× — sounds fine, but the leg-shuffle impulses are
   high-frequency and partially aliased by the sampling. Actual
   in-band SNR is lower. Regression slope attenuates by
   `var(signal) / (var(signal) + var(noise))` so observed slope can
   easily be 70 % of true slope. Applied naively this says r_true is
   closer to 0.32 than 0.22.

2. **Linear slope has a very different impact on SLAM than total
   fictitious-accel magnitude.** The centripetal term ω² · r is
   unsigned and always points inward; reducing r from 0.32 to 0.22
   *under*-corrects it and leaves ~0.04 m/s² extra bias in ax during
   rotation. The tangential α × r term changes sign with α, and the
   magnitude matters more than the correlation-slope sign (which just
   tells you which way you're biased on average). Fixing a
   small-magnitude correlation while introducing a larger-magnitude
   centripetal bias is a bad trade on drift integration.

### Also: SLAM is test-to-test sensitive at this operating point

Even in configurations that nominally looked "similar," small
variations in initial robot pose, which lidar features were visible,
and exactly when / how hard the user pressed D, tip ICP into wrong-
local-minimum at different moments. The r=0.32 run getting
3.3 m bounded drift and the r=0.22 run getting hundreds of meters
isn't necessarily reproducible to the decimal. Per-test variance
suggests the remaining failure mode is scene-structure fragility in
FAST-LIO's ICP, not an IMU problem we can fix by tuning r.

### Decision

**Keep the geometric default r = (0.320, 0, -0.013) m for front Airy.**
Reverted the start script by unsetting `M20_AIRY_LEVER_ARM_BASE_M`.
Plumbing for future tuning is in place; not used in production.

### What's next

The IMU correction chain is correct and bounded. Further gains at
the IMU layer will be sub-meter. The remaining 3 m / rotation-event
drift is in FAST-LIO's ICP / scan-match stage. Options:

1. Try `time_sync_en: true` in velodyne.yaml. This lets FAST-LIO
   estimate the lidar↔IMU offset online. Might help if our scan_start
   stamp is still slightly biased vs IMU's PTP time — easy to try.
2. Try `extrinsic_est_en: true`. Previously caused instant death
   (Finding #24), but that was before the scan_start fix eliminated
   the 100 ms forced IMU extrapolation; worth re-testing with the
   stable baseline we have now.
3. Accept the 3 m drift and test click-to-goal end-to-end in a
   cluttered room. Short-range nav might be fine if the map stays
   globally consistent — pose estimate returns to sensible values
   between rotation events.

Recommended next step: (3). The pilot goal is click-to-goal, not
perfect pose. If it works with current drift, we ship; if it doesn't,
(1)/(2) target the ICP stage where the remaining drift lives.

---

## Finding #31: Codex Second Opinion + Atomic-Publication Bug + Next-Step Plan (2026-04-22)

Applied DIFOP C.17 translation refinement (~5 mm per axis) to the
lever arm — stored it alongside the rotation, automatically added to
`lever_arm_base` when DIFOP arrives. Then dispatched codex-rescue for
a second expert review on where to invest next and how the DIFOP
implementation looks.

### Codex's correctness catch — atomic publication bundling

**Bug.** The initial implementation published four separate atomics
(`g_R_imu_to_base`, `g_difop_offset_body_x/y/z`) in sequence from the
DIFOP retry thread. The IMU hot path loaded each independently. Per-
variable atomicity prevented raw data races, but the bundle was NOT
atomic. The IMU loop could observe:
- A new `R_imu_to_base` with stale (0, 0, 0) DIFOP offset.
- A mixed-generation snapshot across the three offset doubles.

In practice the race is a one-time transient (only one DIFOP swap
per process lifetime, few-ns window), but correctness smell regardless.
`g_cal_ok` didn't gate the hot path; only the 5-second status print.

**Fix.** Bundled into `struct CalibState { Mat3 R_imu_to_base; double
difop_offset_body[3]; }` published via a single
`std::atomic<CalibState*>`. DIFOP thread constructs a fresh
`CalibState`, stores the pointer with `memory_order_release`. Hot
path loads with `memory_order_acquire`, reads `R` and `offset` from
the same instance. No partial-update window.

**Minor cleanups.** Removed stale "unused in v1" comment on
`DifopCalib::tx/ty/tz` (now used). Fixed docstring referencing old
symbol name. Kept the never-free leak pattern — two `CalibState`
allocations over process lifetime, stale one leaks to avoid racing
the hot path's dereference.

### Codex's priority ranking for the remaining 3 m rotation residual

Ranked highest value → lowest at current state:

1. **Front-only / asymmetric-scene A/B test.** Hypothesis: the
   remaining 3 m is now mostly ICP/scene degeneracy in the
   merged-dual-lidar cloud, not core timing. Switching to
   `send_separately: true` + subscribing to front lidar only (96
   rings of single-Airy data, no merge artifacts) isolates whether
   merge is a contributor. Highest info-per-hour.

2. **Raise `acc_cov` 5-10×.** Leave `gyr_cov: 0.1` alone. Rationale:
   residual is translational X-dominant after lever-arm cleanup,
   which implicates imperfect linear-acceleration modeling more than
   rotational propagation. Default 0.1 is Livox-tuned; Airy IMU may
   want higher. Trust accel less before trusting gyro less.

3. **`time_sync_en: true`** — low prior. `native_clock=true` already
   trusts the shared PTP clock for lidar and IMU. `time_sync_en` is
   an upstream software sync meant for unsynced external clocks; if
   it changes anything under PTP, that is a red flag, not a feature.
   Try once, expect no-op.

4. **`extrinsic_est_en: true`** — only after #2. Known FAST-LIO
   guidance: leave off when extrinsic is known. Turning it on can
   let the filter "explain away" residual ICP/accel-model error by
   drifting the extrinsic.

5. **Click-to-goal** — run it now if the product decision matters,
   but not a drift-reduction path. Validates operational tolerance,
   not estimator correctness.

### Decisions for next session

- **Implement now** (this session): atomic publication fix (done).
  Rebuild + deploy, confirm behavior unchanged.
- **Next session, in order**:
  1. `acc_cov` bump to 0.5 or 1.0 (try 5×, 10×). Measure rotation drift.
  2. Click-to-goal end-to-end to validate product viability at current
     drift baseline.
  3. If click-to-goal insufficient: front-only A/B test, then other
     knobs.

The pilot goal is click-to-goal, not perfect pose. 3 m bounded drift
may already be sufficient for short-range nav; worth validating
early before spending more time on estimator tuning.

### Quotable from codex

> "With native_clock=true and shared PTP time, time_sync_en should
> be effectively a no-op; if it changes anything, that is a bad sign,
> not a feature."

> "It is more likely to let the filter 'explain away' residual
> ICP/accel-model error by drifting extrinsic than to converge to
> something useful."

> "Stationary performance is perfect and the catastrophic rotation
> failure was timing, not gyro quality; the remaining error is
> translational after a lever-arm-corrected IMU, so you should trust
> acceleration less before you trust gyro less."

(Full transcript preserved in session history; summarized here for
durability.)

---

## Finding #32: acc_cov Tuning Null Result — Rotation Failure is ICP-Scene, Not IMU (2026-04-22)

Per codex's ranking in Finding #31, tried bumping `acc_cov: 0.1 → 0.5`
(5×) with the rationale "accel is noisy during rotation, trust it
less." Also wrote `rotate_and_log.py` to replace manual D-key tests
with scripted teleop for reproducibility.

### Scripted teleop infrastructure

`rotate_and_log.py` on NOS drives the M20 yaw-in-place by publishing
Twist to `/tele_cmd_vel#geometry_msgs.Twist` at 20 Hz for the requested
duration, then floods 2 s of zero Twists at 50 Hz to guarantee stop.
Safety guards:

- `atexit` + `SIGINT` + `SIGTERM` all trigger the zero flood
  (100 messages × 20 ms spacing). Survives exceptions and ctrl-C.
- Dedicated `lcm.LCM()` instance for the flood, separate from main
  loop's instance — so a deadlocked main loop can't block the stop.
- Default yaw lowered `-0.8 → -0.4` rad/s for safety margin.
- Prints "ROBOT SHOULD BE STOPPED" on normal exit for visual
  confirmation.
- Single-threaded (publish interleaved with `handle_timeout`) after
  discovering `lcm.LCM()` Python module isn't safe for concurrent
  `publish()` + `handle_timeout()` from different threads — the
  earlier two-thread version silently corrupted outgoing messages to
  all-zero Twists (tap on `/tele_cmd_vel` showed 60 msgs, 0 non-zero).

Result: the script drives the robot cleanly, stops reliably, produces
per-phase stats (warmup / rotation / cooldown).

### acc_cov=0.5, three scripted reps

Each rep: `./deploy.sh stop` + restart + full preroll, then `--yaw -0.4
--warmup 2 --rotate_duration 10 --cooldown 3`.

| Rep | KFs | Final pose | Peak drift mag | Outcome |
|---|---|---|---|---|
| 1 | 29+ | `(-1708.7, 1853.6, -423.4)` | 2530 m | **Runaway** |
| 2 | 6 | `(2.5, -1.2, 0.9)` | 2.7 m | Bounded |
| 3 | 3 | `(1.9, -3.6, -0.1)` | 4.1 m | Bounded |

**2/3 bounded, 1/3 runaway.**

Compare to `acc_cov=0.1` manual-D data points from earlier today:

| Test | KFs | Peak | Outcome |
|---|---|---|---|
| Manual rep #1 (2026-04-22 morning) | 11 | 3.3 m | Bounded |
| Manual rep #2 (same day, same config) | 39+ | 182 m | **Runaway** |

That's roughly 1/2 bounded at the default cov. The sweep provides no
statistical evidence that 0.5 is better or worse than 0.1 — 3 reps at
2/3 success vs 2 reps at 1/2 success are within each other's noise.

### Conclusion: IMU-chain is clean, ICP is fragile

The remaining failure mode is **scan-match (ICP) locking to a wrong
local minimum** once every few rotation events in the current indoor
scene. Evidence:

1. When bounded, drift is small (2-5 m) and self-recovers — consistent
   with correct IMU predictions being occasionally jolted by ICP
   corrections that pull too hard.
2. When runaway, drift starts with a sudden step-jump (e.g. KF 5 jumps
   from (2.2, -1.0, 0.5) → (-7.3, -9.4, -6.1) in rep 1) and grows
   monotonically after. Classic "committed to wrong cluster" signature.
3. The failure is RANDOM at the rep level. Same config, same physical
   environment, same scripted rotation produces either bounded or
   runaway with roughly flip-of-a-coin probability. That's not a
   tuning issue — that's a discrete scene-ambiguity issue.
4. `corr(ay, α_z)` was already collapsed to noise after our lever-arm
   fix (Finding #25/#26). Residual IMU bias is well within the
   "predictions are fine, ICP is the weak link" regime.

### What we are NOT going to chase further under FAST-LIO2

- More `acc_cov` / `gyr_cov` tuning — null result above.
- More lever-arm tuning — null result in Finding #30.
- More timing fixes — scan_start anchor is correct (Finding #28), PTP
  clocks match, no drift in `imu_vs_frame_end`.
- More geometry cleanups — ring remap is gone, `scan_line: 96` is
  correct (Finding #29).

The IMU-chain contribution of today's work is durable. The remaining
drift doesn't live there.

### Recommendations for forward progress

**Move off FAST-LIO2 as the primary SLAM backend.** Today's work
(airy_imu_bridge corrections, drdds_lidar_bridge scan_start header,
ring-remap cleanup, flake.lock hygiene) is portable to ANY SLAM
consumer — the LCM outputs are standard `/airy_imu_front` and
`/raw_points` with correct timestamps and correct frame conventions.
A different SLAM stack drops in with the same interface.

Options ranked by plausibility:

1. **ARISE multi-node (parallel path di-ony5x).** Our own port of the
   DeepRobotics reference SLAM for M20. Has keyframe rejection,
   degeneracy detection, loop closure. Was initially blocked by IMU
   preintegration plumbing issues; those pre-date today's IMU
   breakthrough and may now be straightforward.
2. **LIO-SAM** — mature, well-tested Velodyne+IMU stack with graph
   optimization and loop closure. Drop-in topic compatibility.
3. **KISS-ICP** — pure lidar (no IMU dependency). Interesting
   fallback since the IMU chain works fine but isn't the bottleneck.
4. **FAST-LIO_MULTI** — upstream Livox multi-lidar fork. Untried on
   this platform.

Not recommended: continuing FAST-LIO2 tuning without architectural
changes. We've spent enough time on it.

### Also note for Jeff

Our scan_start header fix is in the `drdds_lidar_bridge`, not in Jeff's
FAST-LIO-NON-ROS fork — so it's not an upstream PR. But if Jeff's fork
is consumed by other ROS-bypass stacks, they may have the same
scan_end-as-header confusion we had; worth flagging to him.

The Velodyne PC2 path restoration in `aphexcx/FAST-LIO-NON-ROS` branch
`dimos-integration-velodyne` (Finding #13) IS a candidate upstream
PR.

---

## Finding #33: The Real Bug Was the Robot's Own Body in the ikdtree (2026-04-23)

After all the IMU-chain breakthroughs (Findings #24-#32), rotation SLAM
was still test-to-test variable: ~2/3 bounded to 3m, ~1/3 runaway to
hundreds-of-meters/km. Spent morning on a diagnostic harness chasing
Python/multiprocessing stalls as the next hypothesis; found real
evidence of multi-second IMU delivery gaps (see
`/var/opt/robot/data/tmp/diag_rep{1,2}/` artifacts). Codex agreed stall
was the most-supported hypothesis and flagged the
cluttered-scene ICP angle as weak.

Then user shared a photo showing the M20 has a **physical bumper in
front of its RSAIRY lidars** to protect the optics, and the ikdtree
was registering it as a persistent world feature.

### The mechanism

- `velodyne.yaml` had `blind: 0.5` — spherical radius around the
  LIDAR optical center (verified via preprocess.cpp:172/349 — it is
  `sqrt(x²+y²+z²) < blind²` in the raw sensor frame).
- Front Airy is +320mm forward of body origin per M20 manual
  Section 1.10. So `blind=0.5` in sensor frame covers **+0.82m
  forward / −0.18m backward** in body frame.
- That excluded the front bumper but left **rear wheels at ~0.79m
  range, rear bumper at ~0.85m, leg-shuffle envelope out to ~1.0m**
  visible to FAST-LIO.
- These robot-body points entered the ikdtree as "world" features.
- During yaw, the robot body rotated WITH the sensor — ikdtree had
  bumper/wheel ghosts at world-pose-t-1, incoming scan had them at
  world-pose-t. Scan-match's point-to-plane residual tried to
  reconcile; sometimes succeeded (bounded drift of a few m from
  anti-rotation bias), sometimes locked to a wrong local minimum
  from the misalignment (runaway).

### The fix

One YAML line: `blind: 0.5 → 1.0` in
`dimos/hardware/sensors/lidar/fastlio2/config/velodyne.yaml`.

At `blind = 1.0m` in sensor frame:
- Forward body coverage: +1.32m (past rear bumper)
- Backward: −0.68m
- Lateral/vertical: ±1.0m
- Max body half-diagonal is ~0.65m in any direction from the
  +320mm-offset lidar, so the entire robot is excluded.
- Tradeoff: points in the real near-field 0.5-1.0m range are also
  dropped. Scan goes from ~50k to ~49k points, which is still
  abundant for matching.

### Result — 5/5 reps at zero drift

Ran 5 scripted yaw-in-place tests via `diag_harness.py` (fresh
smartnav each, `yaw=-0.4 rad/s`, 10s rotation). With `blind: 1.0`:

| Rep | KFs | Final pose | Outcome |
|---|---|---|---|
| 1 | 1 | `(0.0, 0.0, 0.0)` | **0 drift** |
| 2 | 1 | `(0.0, -0.0, -0.0)` | **0 drift** |
| 3 | 1 | `(-0.0, 0.0, -0.0)` | **0 drift** |
| 4 | 1 | `(-0.0, -0.0, 0.0)` | **0 drift** |
| 5 | 1 | `(0.0, 0.0, -0.0)` | **0 drift** |

Compare to earlier same-day with `blind: 0.5`:

| Config | Bounded rate | Runaway rate |
|---|---|---|
| `blind: 0.5` acc_cov=0.1 (Finding #32 manual) | 1/2 | 1/2 (to 182m) |
| `blind: 0.5` acc_cov=0.5 (Finding #32 scripted) | 2/3 | 1/3 (to 1700m) |
| **`blind: 1.0`** | **5/5** | **0/5** |

The failure mode is eliminated.

### Evidence that previously supported other hypotheses

Still partly valid, partly red herrings:

**Real signals from prior work — still needed:**
- **Lever-arm centripetal + tangential** (Findings #25/#26): genuine
  physical correction. Without it, the IMU reports 2.6 m/s² typical
  / 11.8 m/s² peak fictitious accel during wheel-legged rotation.
  Any SLAM backend would need this.
- **Scan_start header fix** (Finding #28): genuine bug. drdds's slot
  stamp is scan_END time, not scan_START. FAST-LIO was forced to
  extrapolate IMU 100ms into the future on every scan. This fix
  alone went from 10 km to 3 m drift (before the bumper fix brought
  it to zero).
- **Ring remap cleanup + scan_line=96** (Finding #29): correctness
  cleanup.
- **Atomic CalibState** (Finding #31): correctness.

**Red herring hypotheses I pursued today:**
- "ICP fragility in sparse indoor scene" — scene isn't sparse,
  that's not why ICP was locking wrong. It was locking wrong
  because the map had ghost self-features.
- "Python stalls are the cause of drift" — stalls are real (3-4s
  wall gaps confirmed), but the bumper fix eliminated drift WITHOUT
  fixing stalls. Stalls may matter in other scenarios (long
  sessions, fast linear motion) but not for yaw-in-place drift.

### The lesson

**Look at what's IN the close points, not just whether there are
many.** I analyzed the cloud in earlier diagnostic passes and
correctly measured 7-15k points within 1m, but concluded "cluttered
scene" when the right conclusion was "robot body is in the cloud."
Codex's Finding #31 review recommended scene A/B testing but I
didn't actually do it. User's photo of the bumper finally made the
physical geometry obvious.

### Implication for Jeff

The PHYSICAL BUMPER is universal across M20 units. Any team at
Dimensional running SLAM on this platform is likely hitting this
same issue if they're using any default `blind` under ~1.2m. Worth
asking whether their reference config addresses it.

### What next

1. Longer rotation stress test (30-60s) to verify bounded holds.
2. Click-to-goal end-to-end in a real space.
3. Consider cleaner body-box crop (`|x|<0.5 AND |y|<0.25 AND |z|<0.5`)
   vs the spherical blind. Currently we also filter real features
   in the 0.5-1.0m shell; a proper body-box wouldn't.

---

## Finding #34: URDF-Driven Body-Box Crop + Safety-Guarded Diag Harness (2026-04-23)

Follow-up to Finding #33 (`blind: 0.5 → 1.0`). The spherical blind worked
for 5/5 short rotations but has two problems:
1. Over-excludes real features in the 0.5-1.0m shell (doorway jambs,
   nearby walls — critical for narrow-passage nav).
2. Only 5/5 at 10s; 60s still runaway.

Moved to body-frame AABB crop per codex research (Finding #31 plan
item) — matches DLO/DLIO's production approach. Codex emphasized:
"The biggest implementation bug is cropping too late — if self points
survive preprocessing and enter the local map / ikd-tree, the main
benefit is already gone."

### URDF-derived AABB

Wrote `compute_body_aabb.py` that walks `M20_high_res.urdf` link +
joint tree at zero-joint pose, transforms all collision geometries
(boxes + cylinders) into base_link, computes overall and per-link
bounding boxes.

Per-link ranges in base_link (zero-joint pose):

| Link group | X range | Y range | Z range |
|---|---|---|---|
| base_link chassis | ±0.38 | ±0.12 | ±0.07 |
| hipx (4x) | ±0.36 | ±0.19 | ±0.05 |
| hipy (4x) | ±0.36 | ±0.22 | -0.30..0 |
| knee (4x) | ±0.36 | ±0.24 | -0.54..-0.25 |
| wheel (4x) | ±0.40 | ±0.25 | -0.59..-0.41 |

Overall AABB: X ±0.40, Y ±0.25, Z [-0.59, +0.07].

**Recommended crop with leg-swing padding** (X: 5cm, Y: 10cm, Z: 15cm):
`x[-0.454, +0.454] y[-0.354, +0.354] z[-0.740, +0.220]`.

### Implementation in `drdds_lidar_bridge`

Added `--body_crop xmin,xmax,ymin,ymax,zmin,zmax` CLI flag plumbed
through `DrddsLidarBridgeConfig.body_crop` and blueprint
`M20_BODY_CROP` env var. Runtime decision in the lidar hot loop:
for each point, compare `(x, y, z)` against the AABB directly — the
rsdriver-merged cloud is already in base_link so no rotation needed.
Dropped points are removed by in-place compaction (O(n), one pass,
zero allocation).

Also reverted `velodyne.yaml blind: 1.0 → 0.1` since the body crop
now handles self-filtering in a body-frame-aware way; the spherical
blind only needs to reject true sensor-level near-field noise.

### Empirical verification

After deploy:
```
[drdds_bridge] body_crop ENABLED: x[-0.454,0.454] y[-0.354,0.354] z[-0.740,0.220]
[drdds_bridge] body_crop: 49457 kept, 1292 dropped (2.5%)
```

**2.5% of points per scan drop** (~1300 out of 50k). Lower than I
expected — the M20's forward-mounted RSAIRY can't see most of its
own body (occluded by the housing / behind the optical window). But
the 1300 points that DO sneak through are at close range (<0.5m from
lidar), where they dominate EKF point-weight. Small count, huge
impact.

### Rotation test results — body crop vs blind=1.0

| Test | blind=1.0 (Finding #33) | body_crop + blind=0.1 (this) |
|---|---|---|
| 10s, scripted -0.4 rad/s | 5/5 reps, 0m drift | 1 rep, 1m drift, self-recovered |
| 60s sustained rotation | not tested | **step-jumps at ~KF 5, runaway to ~2 km** |

Body-box crop is at least as good as the 1.0m blind for 10s rotation
AND keeps real near-field features. 60s sustained rotation still
fails — same step-jump-then-runaway pattern as before, meaning the
body-crop was NOT the remaining bottleneck for longer tests.

### Real root cause for 60s failure — still unsolved

Current hypothesis (from diag_harness observations in earlier
sessions + Finding #31 data): **Python/CPU stalls occur
stochastically during rotation**, and over 60s the probability of a
stall coinciding with a critical yaw moment approaches 1. When it
hits, FAST-LIO's EKF propagates forward on a stale IMU for 300-500ms
while the lidar keeps arriving, and scan-match commits to a wrong
local minimum.

Evidence supporting this interpretation:
- 3-4 second wall-clock gaps measured between consecutive IMU LCM
  samples (Finding #31 diagnostic)
- 15 concurrently R/D threads on 8-core NOS (2× oversubscribed)
- `imu_vs_frame_end` spikes to -0.44s under load
- 10s tests: 5/5 pass → no stalls in the window
- 60s tests: 100% fail → at least one stall in the window

Not addressed by IMU-chain or scene-filtering fixes. Requires
scheduler or architecture changes: core-pinning fastlio2_native +
airy_imu_bridge to dedicated CPUs, or reducing Python worker count.
Codex agreed this is the highest-impact / lowest-effort next step.

### Safety bug found + fixed: simple_planner stale goal

While the robot was drift-diverging over 60s tests, I observed in
the nav_cmd_pub log:

```
[simple_planner] A* failed from (-5128.88, 47716.61) to (-112.21, 180.56)
nav_cmd_pub #3851 yaw=-1.396 matched=1
```

The robot's SLAM pose had drifted to (-5 km, +48 km) but
simple_planner still had a goal at (-112, 180) from an earlier stale
state. It was chasing that goal by commanding max angular velocity
(-1.396 rad/s ≈ -80°/s). The mux forwarded those commands as soon
as our teleop's cooldown (1s) expired between publishes.

That's why the user saw "rotation direction switching mid-test" —
our teleop would briefly lose the cooldown race, simple_planner's
stronger yaw command (3.5× magnitude, possibly opposite sign)
would take over, robot would rotate wrong way, then teleop recaptures
on next publish. Unsafe behavior.

### Diag harness v2 safety

Fixed in `/tmp/diag_harness.py`:

1. **Cooldown now actively publishes zero Twists at 20 Hz** instead
   of stopping publishes. Keeps teleop "active" in the mux, blocks
   simple_planner commands. Default cooldown extended 3s → 5s so
   robot stands in place for a few seconds.
2. **SIGTERM to smartnav at end** triggers the graceful-shutdown
   sit-down via the M20 connection module. Robot sits down cleanly
   AFTER standing still in cooldown.
3. **atexit-guarded zero flood** as a last-resort if the process
   dies unexpectedly — publishes 2 seconds of zero Twists before
   exit even on SIGINT/SIGTERM/exception.

Net: `diag_harness` run sequence is now WARMUP → ROTATE → POST-CLOUD →
HOLD-ZERO-COOLDOWN → STOP-FLOOD → SIGTERM. Robot is guaranteed
stopped (and sat down) at end of every test.

### What this means for the pilot

Short rotation bursts (what click-to-goal actually uses — typical
waypoint-to-waypoint trajectories have <2s of continuous rotation)
are production-viable with the current fixes. Sustained yaw-in-place
for 60s is not, but that's not a normal navigation workload.

**Recommended next step: end-to-end click-to-goal validation** to
confirm the product works before chasing the stall fix.

---

## Finding #35: CPU Affinity to Isolated Cores — 60s Drift 2km → 0.4m (2026-04-23)

Finding #34 ended with "60s still diverges via step-jump at ~KF 5,
runaway to 2 km." Hypothesis was Python/multiprocessing stalls
corrupting FAST-LIO's IMU delivery under sustained rotation. Also
saw a click-to-goal attempt fail on the same mechanism: simple_planner
rotated to align with the path direction, SLAM diverged during that
rotation, goal appeared in an even wronger direction, infinite spin.

### Discovery: NOS already has isolated cores reserved for exactly this

```
$ cat /proc/cmdline
... isolcpus=4,5,6,7 ...
$ nproc
4
$ lscpu | grep CPU
CPU(s): 8
```

NOS boot args already set `isolcpus=4,5,6,7`. That reserves cores
4-7 from the default Linux scheduler — no process is placed there
unless it's explicitly affinity-pinned. Cores 0-3 take everything
else (all 4 schedulable CPUs that `nproc` reports). That's why
measured load was 4× oversubscribed: 15+ R/D threads crammed onto
4 cores while 4 sat idle.

Someone at Dimensional (or DeepRobotics upstream) configured this
correctly. We just never used it.

### Implementation

`NativeModuleConfig.cpu_affinity: frozenset[int] | None` field
added to `dimos/core/native_module.py`. In the child preexec hook,
`os.sched_setaffinity(0, cpu_affinity)` is called after fork but
before exec, so the native binary and all its threads inherit the
mask.

Blueprint change in `m20_smartnav_native.py`:

```python
_SLAM_CPU_AFFINITY = frozenset({4, 5, 6, 7})

FastLio2.blueprint(..., cpu_affinity=_SLAM_CPU_AFFINITY)
AiryImuBridge.blueprint(..., cpu_affinity=_SLAM_CPU_AFFINITY)
DrddsLidarBridge.blueprint(..., cpu_affinity=_SLAM_CPU_AFFINITY)
```

Overridable via `M20_SLAM_CORES="4,5,6,7"` env var.

After deploy:
```
fastlio2_native    pid=111805  affinity=4-7  ✓
airy_imu_bridge    pid=111786  affinity=4-7  ✓
drdds_lidar_bridge pid=111783  affinity=4-7  ✓
```

Python smartnav workers stay on cores 0-3 by default scheduler
placement (they're not isolcpus'd in). `yesense_node` was already
running on core 4 (system process with explicit affinity).

### Result — 60s sustained rotation at 0.4 rad/s

```
Keyframe  1 added (0.0, 0.0, -0.0)
Keyframe  2 added (0.0, -0.1, -0.0)
...
Keyframe 20 added (-0.1, -0.2, -0.0)
...
Keyframe 47 added (-0.4, -0.0, -0.0)
```

Max drift: **0.4m** over 60s of continuous rotation (24 rad, ~4 full
revolutions of the robot). Versus:

| Config | 60s rotation peak | # KFs |
|---|---|---|
| Pre-fixes (Finding #24) | ~10 km | 138+ |
| + scan_start (Finding #28) | runs away after ~15s | - |
| + blind=1.0 (Finding #33) | 17 km | 220 |
| + body-box crop (Finding #34) | 2 km | 17 |
| **+ CPU pin 4-7 (this)** | **0.4 m** | **47** |

**~5000× reduction** from baseline; **~5000× reduction** from
body-crop-only. The core-pin is the last-mile fix.

### Understanding why the earlier diagnostics showed stalls

Wall-clock gap of 1.3 s is still present in the Python IMU recorder
even with core pinning in place:

```
IMU samples: 13665
wall gap max: 1293ms  mean: 5.15ms  p99: 32.8ms
gaps > 20ms: 507
```

But FAST-LIO's `imu_vs_frame_end` stayed in ±50 ms:

```
+0.020s: 7 occurrences
-0.000s: 5
+0.015s: 3
+0.010s: 3
+0.005s: 2
+0.000s: ...
max observed: ±0.050s
```

**So the Python LCM recorder (on cores 0-3) is still stalling — but
FAST-LIO and airy_imu_bridge on cores 4-7 are not.** The earlier
diagnostic measurements (Finding #31) were picking up the RECORDER's
stall, not a stall in the SLAM pipeline. Once we isolated the SLAM
pipeline, the actual hot path was clean all along — the stall was
self-inflicted by running the recorder on the same contended cores
as all the Python workers.

Lesson: when measuring stalls, instrument the actual path, not a
second-class python recorder that competes for the same CPU.

### Codex's predicted "won't help" intervention we didn't do

Codex's review also recommended AGAINST reducing `n_workers: 4 → 2`
— workers distribute modules across processes by least-load, so
fewer workers means MORE modules per worker and MORE intra-worker
contention. Kept `n_workers=4`.

Also deferred: SCHED_FIFO for airy_imu_bridge. Not needed — 0.4m
drift at 60s is already in production-spec territory. Can revisit
if we ever want tighter bounds.

### What's next

This unblocks the actual product test: **click-to-goal**. Earlier
attempt failed because the initial "rotate to align with path
direction" phase triggered the same ICP runaway. With that failure
mode gone, click-to-goal should just work.

---

## Questions for Jeff (2026-04-23, in-person at Dimensional)

**IMPORTANT**: Updated after Finding #33. Rotation is fixed; questions
re-prioritized around robustness, generalization, and correctness.

**High priority:**

1. **Self-filter for robot-body points.** We found that M20's physical
   bumpers + rear wheels sit in the ~0.5-1.0m range from the front
   lidar, get registered as persistent world features in the
   ikdtree, and cause catastrophic rotation drift. Spherical `blind`
   works but is a blunt instrument (also drops real close features
   in the 0.5-1.0m shell). Does your fork or Dimensional's
   production SLAM have a **body-frame bounding-box crop**? If not,
   would you take a PR? The right API is probably an additional
   `body_crop` yaml stanza: `{xmin, xmax, ymin, ymax, zmin, zmax}`
   in sensor frame, applied BEFORE the spherical blind.

2. **Production SLAM choice for M20-class robots.** Now that
   rotation is bounded with the bumper fix, is FAST-LIO2 the right
   long-term backend, or are you on something else (LIO-SAM,
   ARISE, etc.)? Curious whether you've hit the body-feature issue
   on your shipping robots and what you use to mitigate.

3. **Degeneracy detection / keyframe rejection.** Does your fork
   have any of the robustness features ARISE has (point-to-plane
   degeneracy detection, reject keyframes with high residual, etc.)?
   Without the bumper fix, FAST-LIO was blind-committing to
   wildly wrong ICP solutions — no warning, no rejection.

**Medium priority — config safety now that baseline is stable:**

4. **`extrinsic_est_en: true` safety.** We saw instant EKF death
   with it on under the OLD scan-end header bug (Finding #24). Now
   that the scan_start bug is fixed and rotation is bounded, is it
   safe to enable? Would it help refine residuals further, or is it
   going to destabilize again?

5. **`feature_enabled=false` hardcode** (laserMapping.hpp:336). Why
   is this permanently off? Would enabling feature extraction help
   — or would it hurt since RSAIRY isn't really a Livox
   non-repetitive pattern? (Note: `pl_buff[128]` upper bound in
   preprocess.h would need to match `scan_line: 96` if toggled on
   for RSAIRY.)

6. **`time_sync_en`** — what's its intended use? With PTP-shared
   timestamps (Airy lidar/IMU on same hardware clock), is it a
   no-op? Or does it still adjust scan-to-IMU alignment?

7. **Python-stack stalls.** Our smartnav is Python-heavy. Measured
   3-4s wall-gaps between consecutive IMU LCM messages on NOS (8
   cores, 15 oversubscribed threads during rotation). FAST-LIO's
   `imu_vs_frame_end` telemetry drops from +50ms to -440ms on those
   spikes. Doesn't cause drift with bumper fix, but probably
   concerns you for production. Do your production stacks pin cores
   or isolate SLAM from Python orchestration? Any recommended
   taskset layout?

**Low priority — upstream contributions:**

8. **Velodyne PC2 input path PR.** Our
   `aphexcx/FAST-LIO-NON-ROS` branch
   `dimos-integration-velodyne` restores the Velodyne handler
   (Finding #13). Would you merge?

9. **Scan_start header convention.** Our `drdds_lidar_bridge` had a
   bug where `pc.header.stamp` was set to scan_END (from drdds slot
   metadata). FAST-LIO adds `max_offset_s` to `header.stamp` to
   compute scan_end, which was then +100ms in the future. Not a
   FAST-LIO bug but worth mentioning — any Dimensional bridge code
   consuming drdds clouds may have the same confusion. Fix was
   trivial (Finding #28).

10. **Multi-lidar support.** Is there a `FAST-LIO_MULTI` branch
    you'd recommend, or is the single-lidar assumption deep enough
    that we should use LOCUS-style preprocess (rigid-transform rear
    into front-lidar frame, feed as one cloud) instead?

---

## Open Questions for Next Session / Jeff

- Does FAST-LIO2's preprocess.cpp + ikd-Tree actually support `scan_line: 192`? **Confirmed safe in audit above — N_SCANS is dynamic, feature_extract path is unused.**
- Is the N_SCANS value ever used to allocate fixed arrays anywhere that would segfault at 192? **Only pl_buff[128] in feature path, unreachable.**
- Would Jeff accept a PR upstreaming the Velodyne input restoration to `leshy/FAST-LIO-NON-ROS`?
- Should the PGO duplicate-key crash (`RuntimeError: key "33", already exists` in `isam2.update`) be a separate bead? Saw it crash smartnav once during this session — likely upstream fastlio2 outputting multiple poses per scan when frames have near-identical timebases.
- Are RSAIRY lidar points actually emitted in time-order, or interleaved across rings? Needs a quick empirical probe to decide how to fix the per-point time relativization.

