# Point-LIO replay file (L0)

One flat little-endian file per recording. Both the C++ golden harness
(`../cpp/harness.cpp`) and the Rust replay/tests read it, so neither depends on
dimos messages or a transport. Produced from a Mid-360 pcap by
`pointlio_replay dump` using the Rust driver's own pcap parser and frame
assembler, so the frames are exactly what the live `Mid360` module publishes.

```
header:  magic "PLIO"  u32 version = 1  f64 frame_hz
records until EOF:
  u8 kind            0 = imu, 1 = lidar
  imu:    u64 ts_ns   f64 gyro[3] (rad/s)   f64 acc[3] (m/s^2)      -- as the driver publishes
  lidar:  u64 start_ns  u32 n   n × { f32 x y z (m); f32 intensity [0,1]; u32 offset_ns; u8 tag }
```

Point record is 21 bytes, packed, no padding.

## Order = feed order

Records are sorted by **availability time**: `ts_ns` for IMU, `start_ns + max(offset_ns)`
(frame end) for lidar; IMU first on ties. That is when a live consumer would see the
message, so a reader feeds records strictly in file order and never reorders.

Consumer rule (harness and Rust replay both): feed each record; after every lidar record
call the estimator's `process()` repeatedly until it reports nothing left to do. Given the
file, the whole run is a fixed call sequence and the C++ core is bit-reproducible.

## Units at the estimator boundary

The estimator wants accel in g (`acc_norm = 1.0` in `PointLioConfig`). Both consumers
divide `acc` by `9.80665` (`GRAVITY_MS2` in the livox crate) before feeding. Gyro is rad/s
as-is. Point `offset_ns` is relative to `start_ns` (the frame's `timebase`).

## Golden output (`harness` writes, Rust tests read)

Directory per dataset, written by `pointlio_harness --replay f.plio --config c.json --out dir
[--frames K] [--stats]` (built by `nix build -L .#pointlio_native` in `../cpp`, installed as
`result/bin/pointlio_harness`; `result/` is a cached symlink, rerun `nix build` after changes):

- `trajectory.tum` — every processed frame: `ts x y z qx qy qz qw` (`%.9f`; ts = the
  estimator's odometry stamp, i.e. `lidar_end_time` in sensor seconds, not wall time).
- `frames.bin` — first `K` frames (default 30) of intermediates for L1/L2. Per frame:
  `u32 idx  f64 lidar_ts (same ts as the .tum line)  u32 n_down  n_down × f32 xyz (feats_down_body)`
  `u32 n_eff (effct_feat_num)  f64 state[42] (kf_output.x_ after update)  f64 P[30*30] row-major`
  then per downsampled point `u8 selected  u8 n_nbr  n_nbr × f32 xyz (Nearest_Points)`
  `f32 plane[4] (esti_plane result, zeros if not selected)`.
  `state[42]` is the `state_output` manifold with rotations as full 3×3 matrices, column-major
  as Eigen stores them: `pos[3] rot[9] offset_R_L_I[9] offset_T_L_I[3] vel[3] omg[3] acc[3]
  gravity[3] bg[3] ba[3]`. (30 is the tangent dimension; the raw matrices avoid a log-map
  convention.)
  The core keeps plane coefficients only for the last point group (`normvec` is per-group), so
  the harness recomputes them from the retained `Nearest_Points` with the same
  `esti_plane(·, plane_thr)` call; `selected` is the core's own `point_selected_surf`.
  `maximum_iter` is 1 in this fork, so neighbours/selection are from the single update pass.
- `config.json` — the exact config the run used: a copy of `--config`. It has the keys of the
  Python `PointLioConfig` as the coordinator sends them (the harness parses it with the same
  `dimos::native::Config::parse<PointLioConfig>` as `main.cpp`), so dump it with

  ```
  python -c 'import json; from dimos.hardware.sensors.lidar.pointlio.module import PointLioConfig as C; print(json.dumps(C(lidar_ip="0.0.0.0", host_ip="0.0.0.0").to_config_dict(), indent=1))'
  ```

Golden must be generated with `-ffp-contract=off` (GCC contracts FMA on aarch64 by default,
not on x86; the flag makes desktop and Jetson goldens agree). The CMake target pins it.

## Rust side (`pointlio_replay`, crate `dimos-pointlio`)

- `run --replay f.plio --config c.json --out dir [--frames K=30] [--max-frames N] [--stats]` —
  same CLI, feed order, drain rule and output layout as the harness (`src/golden.rs` writes
  it); `--max-frames` stops after N processed frames (tests). The core applies the
  `custom_messages::Time` fromSec→toSec round-trip itself (`laser_mapping::ros_time`).
- `compare <golden_dir> <candidate_dir> [--tol-state 1e-9] [--tol-plane-ulp 4]` — per frame,
  in data-flow order: `n_down`, `feats_down_body` (exact), `selected`/neighbours (exact), plane
  ULPs, `n_eff`, state per block, `P`; prints the first frame breaking each check, then the
  unaligned trajectory error (per frame, APE RMSE/max, final pose). Non-zero exit on any break.
  Outputs belong under `data/pointlio_golden/<name>_rust/`, never /tmp.
