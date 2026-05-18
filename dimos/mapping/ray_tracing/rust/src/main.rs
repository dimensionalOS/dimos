// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Native Rust voxel-map module with raycast clearing.
//
// Algorithm (v1):
//   * Insert the voxel of every point into the global hash set.
//   * For every point, walk the 3D-DDA ray from the sensor origin
//     (latest odometry pose) toward the point, removing every
//     intermediate voxel from the map.  The endpoint voxel itself
//     is kept (it just got inserted as a hit).
//
// Inputs (LCM topics, set by the dimos NativeModule coordinator):
//   * `lidar`    : sensor_msgs::PointCloud2  (world frame)
//   * `odometry` : nav_msgs::Odometry        (world frame)
//
// Output:
//   * `global_map` : sensor_msgs::PointCloud2  (world frame)
//
// PointCloud2 input is expected in the standard FastLio2 layout
// (xyz at offsets 0/4/8 as little-endian f32, point_step >= 12).

mod dynamic_cloud;

use ahash::{AHashMap, AHashSet};
use dimos_module::{run, Input, LcmTransport, Module, Output};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::Time;
use serde::Deserialize;

use dynamic_cloud::DynamicCloud;

type VoxelKey = (i32, i32, i32);

#[derive(Debug, Default, Clone, Copy)]
struct VoxelState {
    health: i32,
    timestamp_nanos: u64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    voxel_size: f32,
    max_range: f32,
    ray_subsample: u32,
    shadow_depth: f32,
    min_health: i32,
    max_health: i32,
    /// Seconds between sequence-counter increments. Defaults to 1.0
    /// (one tick per second — "slow clock").
    #[serde(default = "default_sequence_period_secs")]
    sequence_period_secs: f32,
}

fn default_sequence_period_secs() -> f32 {
    1.0
}

#[derive(Default)]
struct VoxelMap {
    voxels: AHashMap<VoxelKey, VoxelState>,
}

#[derive(Default)]
struct SlowClock {
    /// Quantized nanosecond timestamp of the current slow tick.
    current_nanos: u64,
    /// Wall-clock seconds (from msg timestamps) when the next tick fires.
    /// `None` until the first lidar message arrives.
    next_tick_secs: Option<f64>,
}

impl SlowClock {
    /// Advance the clock to the given time, snapping `current_nanos`
    /// forward whenever a `period_secs` boundary is crossed.
    fn advance(&mut self, now_secs: f64, period_secs: f32) {
        let period = period_secs.max(f32::EPSILON) as f64;
        match self.next_tick_secs {
            None => {
                // First sample primes the schedule. Stamp anything observed
                // before the first boundary crossing with this initial time.
                self.current_nanos = secs_to_nanos(now_secs);
                self.next_tick_secs = Some(now_secs + period);
            }
            Some(mut t) => {
                while now_secs >= t {
                    self.current_nanos = secs_to_nanos(t);
                    t += period;
                }
                self.next_tick_secs = Some(t);
            }
        }
    }

    fn reset_to(&mut self, timestamp_nanos: u64, now_secs: f64, period_secs: f32) {
        self.current_nanos = timestamp_nanos;
        self.next_tick_secs = Some(now_secs + period_secs.max(f32::EPSILON) as f64);
    }
}

fn secs_to_nanos(s: f64) -> u64 {
    if s <= 0.0 {
        0
    } else {
        (s * 1e9) as u64
    }
}

fn time_to_secs(t: &Time) -> f64 {
    t.sec as f64 + t.nsec as f64 * 1e-9
}

#[derive(Module)]
struct RayTracingVoxelMap {
    #[input(decode = PointCloud2::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,

    #[input(decode = Odometry::decode, handler = on_odometry)]
    odometry: Input<Odometry>,

    #[input(decode = decode_dynamic_cloud, handler = on_map_override)]
    map_override: Input<DynamicCloud>,

    #[output(encode = encode_dynamic_cloud)]
    global_map: Output<DynamicCloud>,

    #[config]
    config: Config,

    map: VoxelMap,
    last_origin: Option<(f32, f32, f32)>,
    last_lidar_secs: Option<f64>,
    clock: SlowClock,
}

impl RayTracingVoxelMap {
    async fn on_odometry(&mut self, msg: Odometry) {
        self.last_origin = Some((
            msg.pose.pose.position.x as f32,
            msg.pose.pose.position.y as f32,
            msg.pose.pose.position.z as f32,
        ));
    }

    async fn on_lidar(&mut self, msg: PointCloud2) {
        let Some(origin) = self.last_origin else {
            // Need at least one odometry sample before we can raycast.
            return;
        };

        let voxel_size = self.config.voxel_size;
        if voxel_size <= 0.0 {
            eprintln!("voxel_ray_tracing: voxel_size must be > 0, got {voxel_size}");
            return;
        }

        let points = match extract_xyz(&msg) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("voxel_ray_tracing: bad cloud, dropped: {e}");
                return;
            }
        };
        if points.is_empty() {
            return;
        }

        let now_secs = time_to_secs(&msg.header.stamp);
        self.last_lidar_secs = Some(now_secs);
        self.clock
            .advance(now_secs, self.config.sequence_period_secs);
        let timestamp_nanos = self.clock.current_nanos;

        let inv = 1.0_f32 / voxel_size;
        let mut live: AHashSet<VoxelKey> = AHashSet::with_capacity(points.len());
        for &(x, y, z) in &points {
            live.insert(world_to_voxel(x, y, z, inv));
        }

        update_map(
            &mut self.map,
            origin,
            &points,
            &self.config,
            timestamp_nanos,
        );

        // Echo the input cloud's frame; the global map lives in the same
        // world frame as the upstream lidar/odometry.
        let cloud = build_dynamic_cloud(
            &self.map,
            &live,
            voxel_size,
            &msg.header.frame_id,
            msg.header.stamp,
        );
        if let Err(e) = self.global_map.publish(&cloud).await {
            eprintln!("voxel_ray_tracing: publish failed: {e}");
        }
    }

    async fn on_map_override(&mut self, msg: DynamicCloud) {
        self.map.voxels.clear();
        self.map.voxels.reserve(msg.voxels.len());

        let mut per_voxel_ts = vec![0u64; msg.voxels.len()];
        let mut max_ts: u64 = 0;
        for (i, &idx) in msg.event_indices.iter().enumerate() {
            let idx = idx as usize;
            let t = msg.event_timestamps[i];
            if idx < per_voxel_ts.len() && t > per_voxel_ts[idx] {
                per_voxel_ts[idx] = t;
            }
            if t > max_ts {
                max_ts = t;
            }
        }

        for (i, &(x, y, z)) in msg.voxels.iter().enumerate() {
            let health = (msg.quantity[i] as i32).min(self.config.max_health);
            let timestamp_nanos = per_voxel_ts[i];
            self.map.voxels.insert(
                (x, y, z),
                VoxelState {
                    health,
                    timestamp_nanos,
                },
            );
        }

        // Reset the slow clock. Prefer the last lidar timestamp; fall back
        // to the override's own message timestamp if no lidar has been seen yet.
        let now_secs = self
            .last_lidar_secs
            .unwrap_or(msg.timestamp_nanos as f64 * 1e-9);
        self.clock
            .reset_to(max_ts, now_secs, self.config.sequence_period_secs);
    }
}

fn decode_dynamic_cloud(buf: &[u8]) -> std::io::Result<DynamicCloud> {
    DynamicCloud::decode(buf).map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
}

fn encode_dynamic_cloud(msg: &DynamicCloud) -> Vec<u8> {
    msg.encode()
        .expect("DynamicCloud::encode: frame_id exceeds 65535 bytes")
}

fn update_map(
    map: &mut VoxelMap,
    origin: (f32, f32, f32),
    points: &[(f32, f32, f32)],
    cfg: &Config,
    timestamp_nanos: u64,
) {
    let inv = 1.0_f32 / cfg.voxel_size;
    let max_range_sq = if cfg.max_range > 0.0 {
        cfg.max_range * cfg.max_range
    } else {
        f32::INFINITY
    };

    let mut hits: AHashSet<VoxelKey> = AHashSet::with_capacity(points.len());
    for &(x, y, z) in points {
        hits.insert(world_to_voxel(x, y, z, inv));
    }

    let mut misses: AHashSet<VoxelKey> = AHashSet::new();
    let origin_voxel = world_to_voxel(origin.0, origin.1, origin.2, inv);
    let step = cfg.ray_subsample.max(1) as usize;
    for (i, &p) in points.iter().enumerate() {
        if i % step != 0 {
            continue;
        }
        let dx = p.0 - origin.0;
        let dy = p.1 - origin.1;
        let dz = p.2 - origin.2;
        if dx * dx + dy * dy + dz * dz > max_range_sq {
            continue;
        }
        let endpoint = world_to_voxel(p.0, p.1, p.2, inv);
        walk_ray(
            &mut misses,
            origin,
            p,
            cfg.voxel_size,
            cfg.shadow_depth,
            origin_voxel,
            endpoint,
        );
    }

    // Apply hits first: a voxel that is both a hit and a miss this scan
    // counts as a hit (the lidar return is the stronger signal).
    //
    // Sequence stamping: when an observation lands on a voxel whose
    // current health <= 0 (i.e. still uncertain), stamp it with the
    // current slow-clock value. The check is against PRE-update health,
    // so the confirmation event itself (uncertain -> confirmed) also
    // gets stamped — its sequence captures the moment of confirmation.
    // Subsequent hits on already-confirmed voxels (pre-health > 0) leave
    // the stamp frozen.
    for v in &hits {
        let state = map.voxels.entry(*v).or_insert(VoxelState {
            health: cfg.min_health,
            timestamp_nanos,
        });
        let was_uncertain = state.health <= 0;
        state.health = (state.health + 1).min(cfg.max_health);
        if was_uncertain {
            state.timestamp_nanos = timestamp_nanos;
        }
    }
    for v in misses.difference(&hits) {
        if let Some(state) = map.voxels.get_mut(v) {
            let was_uncertain = state.health <= 0;
            state.health -= 1;
            if state.health <= cfg.min_health {
                map.voxels.remove(v);
            } else if was_uncertain {
                state.timestamp_nanos = timestamp_nanos;
            }
        }
    }
}

#[inline]
fn world_to_voxel(x: f32, y: f32, z: f32, inv: f32) -> VoxelKey {
    (
        (x * inv).floor() as i32,
        (y * inv).floor() as i32,
        (z * inv).floor() as i32,
    )
}

/// Amanatides & Woo 3-D DDA. Records every voxel strictly between
/// `origin_voxel` and `endpoint` into `misses`, then continues past
/// `endpoint` for `shadow_depth` meters and records those voxels too.
/// The endpoint voxel itself is not added (it is a hit, handled by the
/// caller).
fn walk_ray(
    misses: &mut AHashSet<VoxelKey>,
    origin: (f32, f32, f32),
    end: (f32, f32, f32),
    voxel_size: f32,
    shadow_depth: f32,
    origin_voxel: VoxelKey,
    endpoint: VoxelKey,
) {
    if origin_voxel == endpoint {
        return;
    }

    let (ox, oy, oz) = origin;
    let dx = end.0 - ox;
    let dy = end.1 - oy;
    let dz = end.2 - oz;

    let (mut x, mut y, mut z) = origin_voxel;

    let step_x = dx.signum() as i32;
    let step_y = dy.signum() as i32;
    let step_z = dz.signum() as i32;

    let t_max_init = |p: f32, d: f32, vox: i32, step: i32| -> f32 {
        if step == 0 {
            return f32::INFINITY;
        }
        let next_boundary = if step > 0 {
            (vox + 1) as f32 * voxel_size
        } else {
            vox as f32 * voxel_size
        };
        (next_boundary - p) / d
    };

    let mut tx = t_max_init(ox, dx, x, step_x);
    let mut ty = t_max_init(oy, dy, y, step_y);
    let mut tz = t_max_init(oz, dz, z, step_z);

    let dt_x = if step_x == 0 {
        f32::INFINITY
    } else {
        voxel_size / dx.abs()
    };
    let dt_y = if step_y == 0 {
        f32::INFINITY
    } else {
        voxel_size / dy.abs()
    };
    let dt_z = if step_z == 0 {
        f32::INFINITY
    } else {
        voxel_size / dz.abs()
    };

    let half = voxel_size * 0.5;
    let endpoint_center = (
        endpoint.0 as f32 * voxel_size + half,
        endpoint.1 as f32 * voxel_size + half,
        endpoint.2 as f32 * voxel_size + half,
    );
    let shadow_sq = shadow_depth.max(0.0).powi(2);

    // FIXME: I don't know if we really need this
    let max_iter = 4096;
    let mut past_endpoint = false;
    for _ in 0..max_iter {
        if tx < ty {
            if tx < tz {
                x += step_x;
                tx += dt_x;
            } else {
                z += step_z;
                tz += dt_z;
            }
        } else if ty < tz {
            y += step_y;
            ty += dt_y;
        } else {
            z += step_z;
            tz += dt_z;
        }

        // FIXME: I don't like how this is written, come back and change this.
        // It would be more clear to do this in two loops, one for the normal tracing
        // and a second for the shadow clearing
        if (x, y, z) == endpoint {
            past_endpoint = true;
            continue;
        }

        if past_endpoint {
            let cx = x as f32 * voxel_size + half;
            let cy = y as f32 * voxel_size + half;
            let cz = z as f32 * voxel_size + half;
            let ddx = cx - endpoint_center.0;
            let ddy = cy - endpoint_center.1;
            let ddz = cz - endpoint_center.2;
            if ddx * ddx + ddy * ddy + ddz * ddz > shadow_sq {
                return;
            }
        }

        misses.insert((x, y, z));
    }
}

struct ExtractError(&'static str);
impl std::fmt::Display for ExtractError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.0)
    }
}

fn extract_xyz(msg: &PointCloud2) -> Result<Vec<(f32, f32, f32)>, ExtractError> {
    let mut x_off: Option<usize> = None;
    let mut y_off: Option<usize> = None;
    let mut z_off: Option<usize> = None;
    for f in &msg.fields {
        if f.datatype != PointField::FLOAT32 as u8 {
            continue;
        }
        match f.name.as_str() {
            "x" => x_off = Some(f.offset as usize),
            "y" => y_off = Some(f.offset as usize),
            "z" => z_off = Some(f.offset as usize),
            _ => {}
        }
    }
    let xo = x_off.ok_or(ExtractError("missing float32 x field"))?;
    let yo = y_off.ok_or(ExtractError("missing float32 y field"))?;
    let zo = z_off.ok_or(ExtractError("missing float32 z field"))?;

    let n = (msg.width as usize) * (msg.height as usize);
    let step = msg.point_step as usize;
    if step == 0 {
        return Err(ExtractError("point_step is 0"));
    }
    if msg.data.len() < n * step {
        return Err(ExtractError(
            "data buffer shorter than width*height*point_step",
        ));
    }
    if msg.is_bigendian {
        return Err(ExtractError("big-endian point data not supported"));
    }

    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let base = i * step;
        let x = read_f32_le(&msg.data, base + xo);
        let y = read_f32_le(&msg.data, base + yo);
        let z = read_f32_le(&msg.data, base + zo);
        if x.is_finite() && y.is_finite() && z.is_finite() {
            out.push((x, y, z));
        }
    }
    Ok(out)
}

#[inline]
fn read_f32_le(buf: &[u8], off: usize) -> f32 {
    let bytes: [u8; 4] = buf[off..off + 4]
        .try_into()
        .expect("bounds checked by caller");
    f32::from_le_bytes(bytes)
}

fn build_dynamic_cloud(
    map: &VoxelMap,
    live: &AHashSet<VoxelKey>,
    voxel_size: f32,
    frame_id: &str,
    stamp: Time,
) -> DynamicCloud {
    // Include all voxels currently considered "live": those with health > 0
    // (confirmed) plus any voxel hit this scan (even if still uncertain).
    // Live voxels were just inserted by update_map, so they're guaranteed
    // in the map.
    //
    // Emit one event per published voxel — the sparse format permits
    // tighter packings (e.g. only timestamping changed voxels) but this
    // dense layout is simplest and lets downstream consumers always look
    // up a per-voxel timestamp directly.
    let mut voxels = Vec::with_capacity(map.voxels.len());
    let mut quantity = Vec::with_capacity(map.voxels.len());
    let mut event_indices = Vec::with_capacity(map.voxels.len());
    let mut event_timestamps = Vec::with_capacity(map.voxels.len());

    for (&key, &state) in &map.voxels {
        if state.health > 0 || live.contains(&key) {
            let idx = voxels.len() as u32;
            voxels.push(key);
            quantity.push(state.health.max(0) as u32);
            event_indices.push(idx);
            event_timestamps.push(state.timestamp_nanos);
        }
    }

    let timestamp_nanos = (stamp.sec as i64 as u64)
        .wrapping_mul(1_000_000_000)
        .wrapping_add(stamp.nsec.max(0) as u64);

    DynamicCloud {
        timestamp_nanos,
        voxel_size,
        frame_id: frame_id.to_string(),
        voxels,
        quantity,
        event_indices,
        event_timestamps,
    }
}

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new()
        .await
        .expect("failed to create LCM transport");
    run::<RayTracingVoxelMap, _>(transport)
        .await
        .expect("voxel_ray_tracing run failed");
}

#[cfg(test)]
mod tests {
    use super::*;

    fn basic_config() -> Config {
        Config {
            voxel_size: 1.0,
            max_range: 100.0,
            ray_subsample: 1,
            shadow_depth: 2.0,
            min_health: 0,
            max_health: 1,
            sequence_period_secs: 1.0,
        }
    }

    fn health_of(map: &VoxelMap, key: VoxelKey) -> Option<i32> {
        map.voxels.get(&key).map(|s| s.health)
    }

    fn insert_health(map: &mut VoxelMap, key: VoxelKey, health: i32) {
        map.voxels.insert(
            key,
            VoxelState {
                health,
                timestamp_nanos: 0,
            },
        );
    }

    #[test]
    fn walk_ray_hits_correct_voxels_1() {
        let voxel_size = 1.0;
        let shadow_depth = 2.0;
        let origin = (0.5, 0.5, 0.5);
        let end = (5.5, 0.5, 0.5);
        let inv = 1.0 / voxel_size;
        let origin_voxel = world_to_voxel(origin.0, origin.1, origin.2, inv);
        let endpoint = world_to_voxel(end.0, end.1, end.2, inv);

        let mut misses: AHashSet<VoxelKey> = AHashSet::new();
        walk_ray(
            &mut misses,
            origin,
            end,
            voxel_size,
            shadow_depth,
            origin_voxel,
            endpoint,
        );

        let expected: AHashSet<VoxelKey> = [
            (1, 0, 0),
            (2, 0, 0),
            (3, 0, 0),
            (4, 0, 0),
            (6, 0, 0),
            (7, 0, 0),
        ]
        .into_iter()
        .collect();
        assert_eq!(misses, expected);
    }

    #[test]
    fn walk_ray_hits_correct_voxels_2() {
        let voxel_size = 1.0;
        let shadow_depth = 2.0;
        let origin = (0.5, 0.5, 0.5);
        let end = (3.5, 2.5, 1.5);
        let inv = 1.0 / voxel_size;
        let origin_voxel = world_to_voxel(origin.0, origin.1, origin.2, inv);
        let endpoint = world_to_voxel(end.0, end.1, end.2, inv);

        let mut misses: AHashSet<VoxelKey> = AHashSet::new();
        walk_ray(
            &mut misses,
            origin,
            end,
            voxel_size,
            shadow_depth,
            origin_voxel,
            endpoint,
        );

        let expected: AHashSet<VoxelKey> = [
            (1, 0, 0),
            (1, 1, 0),
            (1, 1, 1),
            (2, 1, 1),
            (2, 2, 1),
            (4, 2, 1),
            (4, 3, 1),
            (4, 3, 2),
        ]
        .into_iter()
        .collect();
        assert_eq!(misses, expected);
    }

    #[test]
    fn hits_insert_voxels() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        update_map(
            &mut map,
            (0.0, 0.0, 0.0),
            &[(5.5, 0.5, 0.5), (0.5, 5.5, 0.5)],
            &cfg,
            0,
        );
        assert_eq!(health_of(&map, (5, 0, 0)), Some(1));
        assert_eq!(health_of(&map, (0, 5, 0)), Some(1));
        assert_eq!(map.voxels.len(), 2);
    }

    #[test]
    fn voxels_on_ray_are_removed() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        insert_health(&mut map, (3, 0, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg, 0);
        // make sure the initial point got cleared by the new update
        assert!(!map.voxels.contains_key(&(3, 0, 0)));
        assert_eq!(health_of(&map, (5, 0, 0)), Some(1));
    }

    #[test]
    fn voxels_not_on_ray_survive() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        insert_health(&mut map, (3, 5, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg, 0);
        assert_eq!(health_of(&map, (3, 5, 0)), Some(1));
        assert_eq!(health_of(&map, (5, 0, 0)), Some(1));
    }

    #[test]
    fn voxels_within_shadow_region_are_removed() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        insert_health(&mut map, (6, 0, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg, 0);
        // point within the shadow is no longer included, new point is included
        assert!(!map.voxels.contains_key(&(6, 0, 0)));
        assert_eq!(health_of(&map, (5, 0, 0)), Some(1));
    }

    #[test]
    fn voxels_beyond_shadow_region_survive() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        insert_health(&mut map, (8, 0, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg, 0);
        assert_eq!(health_of(&map, (8, 0, 0)), Some(1));
        assert_eq!(health_of(&map, (5, 0, 0)), Some(1));
    }

    #[test]
    fn hit_caught_by_other_ray_is_not_removed() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        update_map(
            &mut map,
            (0.0, 0.0, 0.0),
            &[(3.5, 0.5, 0.5), (5.5, 0.5, 0.5)],
            &cfg,
            0,
        );
        assert_eq!(health_of(&map, (3, 0, 0)), Some(1));
        assert_eq!(health_of(&map, (5, 0, 0)), Some(1));
    }

    #[test]
    fn point_beyond_max_range_does_not_clear() {
        let cfg = Config {
            max_range: 3.0,
            ..basic_config()
        };
        let mut map = VoxelMap::default();
        insert_health(&mut map, (3, 0, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg, 0);
        assert_eq!(health_of(&map, (3, 0, 0)), Some(1));
    }

    #[test]
    fn two_hits_needed_when_min_health_is_negative() {
        let cfg = Config {
            min_health: -1,
            ..basic_config()
        };
        let mut map = VoxelMap::default();
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg, 0);
        assert_eq!(health_of(&map, (5, 0, 0)), Some(0));

        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg, 0);
        assert_eq!(health_of(&map, (5, 0, 0)), Some(1));
    }

    #[test]
    fn two_misses_needed_when_max_health_is_two() {
        let cfg = Config {
            max_health: 2,
            ..basic_config()
        };
        let mut map = VoxelMap::default();
        update_map(&mut map, (0.0, 0.0, 0.0), &[(3.5, 0.5, 0.5)], &cfg, 0);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(3.5, 0.5, 0.5)], &cfg, 0);
        assert_eq!(health_of(&map, (3, 0, 0)), Some(2));

        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg, 0);
        assert_eq!(health_of(&map, (3, 0, 0)), Some(1));

        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg, 0);
        assert!(!map.voxels.contains_key(&(3, 0, 0)));
    }

    #[test]
    fn unconfirmed_voxels_get_timestamp_stamp() {
        // With min_health=-1, a fresh hit lands at health=0 — still
        // uncertain — and so must be stamped with the supplied timestamp.
        let cfg = Config {
            min_health: -1,
            ..basic_config()
        };
        let mut map = VoxelMap::default();
        update_map(
            &mut map,
            (0.0, 0.0, 0.0),
            &[(5.5, 0.5, 0.5)],
            &cfg,
            42_000_000_000,
        );
        let state = map.voxels.get(&(5, 0, 0)).copied().unwrap();
        assert_eq!(state.health, 0);
        assert_eq!(state.timestamp_nanos, 42_000_000_000);
    }

    #[test]
    fn confirmed_voxels_freeze_their_timestamp() {
        // With min_health=-1, max_health=1: two hits to reach health=1.
        //   hit #1 (ts=10): -1 -> 0, pre-health was -1 (≤0), stamp 10
        //   hit #2 (ts=99): 0 -> 1, pre-health was 0 (≤0), stamp 99
        //                   -- voxel now confirmed --
        //   hit #3 (ts=1000): pre-health is 1 (>0), no stamp
        let cfg = Config {
            min_health: -1,
            ..basic_config()
        };
        let mut map = VoxelMap::default();
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg, 10);
        assert_eq!(map.voxels[&(5, 0, 0)].timestamp_nanos, 10);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg, 99);
        let confirming = map.voxels[&(5, 0, 0)];
        assert_eq!(confirming.health, 1);
        assert_eq!(confirming.timestamp_nanos, 99);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg, 1000);
        let frozen = map.voxels[&(5, 0, 0)];
        assert_eq!(frozen.health, 1);
        assert_eq!(frozen.timestamp_nanos, 99);
    }

    #[test]
    fn slow_clock_ticks_at_period() {
        let mut clock = SlowClock::default();
        clock.advance(100.0, 1.0);
        // First call primes the schedule and stamps with `now` itself.
        assert_eq!(clock.current_nanos, secs_to_nanos(100.0));
        clock.advance(100.5, 1.0);
        // Not yet a period elapsed.
        assert_eq!(clock.current_nanos, secs_to_nanos(100.0));
        clock.advance(101.0, 1.0);
        // Crossed first scheduled boundary (101.0).
        assert_eq!(clock.current_nanos, secs_to_nanos(101.0));
        clock.advance(103.5, 1.0);
        // Crossed boundaries at 102.0 and 103.0 — most recent wins.
        assert_eq!(clock.current_nanos, secs_to_nanos(103.0));
    }

    #[test]
    fn slow_clock_reset_snaps_backwards() {
        let mut clock = SlowClock::default();
        clock.advance(100.0, 1.0);
        clock.advance(110.0, 1.0);
        let big = clock.current_nanos;
        // Override is authoritative even if smaller.
        clock.reset_to(42, 110.0, 1.0);
        assert_eq!(clock.current_nanos, 42);
        assert!(clock.current_nanos < big);
        // Next tick still fires at the scheduled time.
        clock.advance(111.0, 1.0);
        assert_eq!(clock.current_nanos, secs_to_nanos(111.0));
    }
}
