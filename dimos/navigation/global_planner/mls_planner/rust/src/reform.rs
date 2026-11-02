// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Keyframe pose corrections from a pose-graph optimizer, turned into a warp
//! of world points with fast-reform.
//!
//! Two PointCloud2 record topics carry them, little-endian with no padding:
//! `pose_graph`, one record per keyframe (x, y, z, node_id, ts, qx..qw), and
//! `loop_closure_event`, one per keyframe a closure moved (the pre-closure
//! pose as in pose_graph, then dx, dy, dz, dqx..dqw with post = delta * pre).
//!
//! The map holds each keyframe at the pose it had when the map last moved for
//! it, its applied pose. A closure warps by post * applied⁻¹ for the keyframes
//! that moved at least `min_shift_m`. The others keep their applied pose, so
//! the moves too small to warp for are carried into the next closure, not lost.

use ahash::{AHashMap, AHashSet};
use fast_reform::graph::BlendDeltas;
use fast_reform::{warp_positions, Quat, Vec3};
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use rayon::prelude::*;

/// Gaussian widths past which an anchor's weight counts as nothing.
const REACH_SIGMAS: f64 = 3.0;

/// Time sigmas past which an anchor is left out of a point's blend.
const TIME_REACH_SIGMAS: f64 = 4.0;

/// Tiles searched outward for an anchor around a tile that has none near.
const MAX_FALLBACK_RINGS: i32 = 16;

/// A keyframe pose: world-frame rotation and translation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub rotation: Quat,
    pub translation: Vec3,
}

impl Pose {
    pub const IDENTITY: Pose = Pose {
        rotation: Quat::IDENTITY,
        translation: Vec3::ZERO,
    };

    /// self * other.
    pub fn compose(&self, other: &Pose) -> Pose {
        let rotated = self.rotation.rotate(other.translation);
        Pose {
            rotation: quat_mul(self.rotation, other.rotation).normalized_or_identity(),
            translation: Vec3::new(
                rotated.x + self.translation.x,
                rotated.y + self.translation.y,
                rotated.z + self.translation.z,
            ),
        }
    }

    pub fn inverse(&self) -> Pose {
        let rotation = self.rotation.conjugate();
        let t = rotation.rotate(self.translation);
        Pose {
            rotation,
            translation: Vec3::new(-t.x, -t.y, -t.z),
        }
    }

    pub fn apply(&self, p: Vec3) -> Vec3 {
        let r = self.rotation.rotate(p);
        Vec3::new(
            r.x + self.translation.x,
            r.y + self.translation.y,
            r.z + self.translation.z,
        )
    }
}

fn quat_mul(a: Quat, b: Quat) -> Quat {
    Quat::new(
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    )
}

/// Rotation angle of a unit quaternion, in radians.
fn angle(q: Quat) -> f64 {
    2.0 * q.w.abs().min(1.0).acos()
}

/// One pose_graph record.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KeyframeRecord {
    pub id: u32,
    pub ts: f64,
    pub pose: Pose,
}

/// One loop_closure_event record.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MovedRecord {
    pub keyframe: KeyframeRecord,
    pub delta: Pose,
}

impl MovedRecord {
    pub fn post(&self) -> Pose {
        self.delta.compose(&self.keyframe.pose)
    }
}

#[derive(Debug, PartialEq)]
pub struct DecodeError(pub String);

impl std::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

/// Field offsets by name, each checked for its datatype.
struct Fields<'a> {
    msg: &'a PointCloud2,
    step: usize,
    count: usize,
}

impl<'a> Fields<'a> {
    fn new(msg: &'a PointCloud2) -> Result<Self, DecodeError> {
        if msg.is_bigendian {
            return Err(DecodeError("big-endian records are not supported".into()));
        }
        let step = msg.point_step as usize;
        let count = (msg.width.max(0) as usize) * (msg.height.max(0) as usize);
        if count > 0 && (step == 0 || msg.data.len() < count * step) {
            return Err(DecodeError(format!(
                "{} bytes cannot hold {count} records of {step}",
                msg.data.len()
            )));
        }
        Ok(Self { msg, step, count })
    }

    fn offset(&self, name: &str, datatype: u8, size: usize) -> Result<usize, DecodeError> {
        let field = self
            .msg
            .fields
            .iter()
            .find(|f| f.name == name)
            .ok_or_else(|| DecodeError(format!("missing field {name}")))?;
        if field.datatype != datatype {
            return Err(DecodeError(format!(
                "field {name} has datatype {}, want {datatype}",
                field.datatype
            )));
        }
        let offset = field.offset as usize;
        if offset + size > self.step {
            return Err(DecodeError(format!("field {name} overruns the record")));
        }
        Ok(offset)
    }

    fn f32s<const N: usize>(&self, names: [&str; N]) -> Result<[usize; N], DecodeError> {
        let mut out = [0; N];
        for (slot, name) in out.iter_mut().zip(names) {
            *slot = self.offset(name, PointField::FLOAT32 as u8, 4)?;
        }
        Ok(out)
    }

    fn record(&self, i: usize) -> &[u8] {
        &self.msg.data[i * self.step..(i + 1) * self.step]
    }
}

fn read<const N: usize>(record: &[u8], offset: usize) -> [u8; N] {
    record[offset..offset + N]
        .try_into()
        .expect("offset checked against the step")
}

fn pose_at(record: &[u8], t: &[usize; 3], q: &[usize; 4]) -> Pose {
    let f = |o: usize| f32::from_le_bytes(read(record, o)) as f64;
    Pose {
        translation: Vec3::new(f(t[0]), f(t[1]), f(t[2])),
        rotation: Quat::new(f(q[0]), f(q[1]), f(q[2]), f(q[3])).normalized_or_identity(),
    }
}

/// Decode a pose_graph cloud, one record per keyframe.
pub fn decode_pose_graph(msg: &PointCloud2) -> Result<Vec<KeyframeRecord>, DecodeError> {
    let fields = Fields::new(msg)?;
    let t = fields.f32s(["x", "y", "z"])?;
    let q = fields.f32s(["qx", "qy", "qz", "qw"])?;
    let id = fields.offset("node_id", PointField::UINT32 as u8, 4)?;
    let ts = fields.offset("ts", PointField::FLOAT64 as u8, 8)?;
    Ok((0..fields.count)
        .map(|i| {
            let r = fields.record(i);
            KeyframeRecord {
                id: u32::from_le_bytes(read(r, id)),
                ts: f64::from_le_bytes(read(r, ts)),
                pose: pose_at(r, &t, &q),
            }
        })
        .collect())
}

/// Decode a loop_closure_event cloud, one record per keyframe it moved.
pub fn decode_loop_closure_event(msg: &PointCloud2) -> Result<Vec<MovedRecord>, DecodeError> {
    let keyframes = decode_pose_graph(msg)?;
    let fields = Fields::new(msg)?;
    let dt = fields.f32s(["dx", "dy", "dz"])?;
    let dq = fields.f32s(["dqx", "dqy", "dqz", "dqw"])?;
    Ok(keyframes
        .into_iter()
        .enumerate()
        .map(|(i, keyframe)| MovedRecord {
            keyframe,
            delta: pose_at(fields.record(i), &dt, &dq),
        })
        .collect())
}

/// Field layout of pose_graph, for building test and replay clouds.
pub const POSE_GRAPH_FIELDS: &[(&str, u8)] = &[
    ("x", PointField::FLOAT32 as u8),
    ("y", PointField::FLOAT32 as u8),
    ("z", PointField::FLOAT32 as u8),
    ("node_id", PointField::UINT32 as u8),
    ("ts", PointField::FLOAT64 as u8),
    ("qx", PointField::FLOAT32 as u8),
    ("qy", PointField::FLOAT32 as u8),
    ("qz", PointField::FLOAT32 as u8),
    ("qw", PointField::FLOAT32 as u8),
];

/// pose_graph's fields then the delta's.
pub const LOOP_EVENT_FIELDS: &[(&str, u8)] = &[
    ("x", PointField::FLOAT32 as u8),
    ("y", PointField::FLOAT32 as u8),
    ("z", PointField::FLOAT32 as u8),
    ("node_id", PointField::UINT32 as u8),
    ("ts", PointField::FLOAT64 as u8),
    ("qx", PointField::FLOAT32 as u8),
    ("qy", PointField::FLOAT32 as u8),
    ("qz", PointField::FLOAT32 as u8),
    ("qw", PointField::FLOAT32 as u8),
    ("dx", PointField::FLOAT32 as u8),
    ("dy", PointField::FLOAT32 as u8),
    ("dz", PointField::FLOAT32 as u8),
    ("dqx", PointField::FLOAT32 as u8),
    ("dqy", PointField::FLOAT32 as u8),
    ("dqz", PointField::FLOAT32 as u8),
    ("dqw", PointField::FLOAT32 as u8),
];

/// A record cloud over raw packed records with the given layout.
pub fn record_cloud(layout: &[(&str, u8)], data: Vec<u8>) -> PointCloud2 {
    let mut offset = 0;
    let fields: Vec<PointField> = layout
        .iter()
        .map(|&(name, datatype)| {
            let field = PointField {
                name: name.into(),
                offset,
                datatype,
                count: 1,
            };
            offset += if datatype == PointField::FLOAT64 as u8 {
                8
            } else {
                4
            };
            field
        })
        .collect();
    let n = data.len() as i32 / offset.max(1);
    PointCloud2 {
        height: 1,
        width: n,
        fields,
        is_bigendian: false,
        point_step: offset,
        row_step: offset * n,
        data,
        is_dense: true,
        ..Default::default()
    }
}

/// How a closure turns into a warp.
#[derive(Clone, Copy, Debug)]
pub struct ReformParams {
    /// Spatial blend width, m. 0 picks twice the mean keyframe spacing.
    pub blend_sigma_m: f64,
    /// Temporal blend width, s. 0 picks twice the mean keyframe interval.
    pub blend_time_sigma_s: f64,
    /// A keyframe whose correction moves its surroundings less than this
    /// waits for a later closure.
    pub min_shift_m: f64,
    /// Past this fraction of the map's voxels moving, the map is rebuilt
    /// from the warped voxels rather than repaired around them all.
    pub rebuild_fraction: f64,
}

/// A keyframe as the map holds it.
#[derive(Clone, Copy, Debug)]
struct Applied {
    ts: f64,
    pose: Pose,
}

/// The keyframes' applied poses, kept from pose_graph and closures.
#[derive(Default)]
pub struct ReformState {
    applied: AHashMap<u32, Applied>,
}

impl ReformState {
    pub fn len(&self) -> usize {
        self.applied.len()
    }

    pub fn is_empty(&self) -> bool {
        self.applied.is_empty()
    }

    /// Take the pose of each keyframe not seen before. A known keyframe keeps
    /// its applied pose: the map has not moved for its newer pose yet.
    pub fn add_keyframes(&mut self, records: &[KeyframeRecord]) {
        for r in records {
            self.applied.entry(r.id).or_insert(Applied {
                ts: r.ts,
                pose: r.pose,
            });
        }
    }

    /// The warp a closure asks for, or None when no keyframe moved enough.
    /// The keyframes it warps for take their post pose.
    pub fn closure(&mut self, moved: &[MovedRecord], params: &ReformParams) -> Option<Warp> {
        // A keyframe new to us arrives at its pre pose, as the map holds it.
        for m in moved {
            self.applied.entry(m.keyframe.id).or_insert(Applied {
                ts: m.keyframe.ts,
                pose: m.keyframe.pose,
            });
        }
        let sigma = if params.blend_sigma_m > 0.0 {
            params.blend_sigma_m
        } else {
            2.0 * mean_spacing(self.applied.values().map(|a| a.pose.translation))
        }
        .max(1e-3);
        let time_sigma = if params.blend_time_sigma_s > 0.0 {
            params.blend_time_sigma_s
        } else {
            2.0 * mean_interval(self.applied.values().map(|a| a.ts))
        }
        .max(1e-6);
        let reach = REACH_SIGMAS * sigma;

        let mut corrections: AHashMap<u32, Pose> = AHashMap::new();
        for m in moved {
            let applied = self.applied[&m.keyframe.id];
            let correction = m.post().compose(&applied.pose.inverse());
            let at = applied.pose.translation;
            let to = correction.apply(at);
            let shift = ((to.x - at.x).powi(2) + (to.y - at.y).powi(2) + (to.z - at.z).powi(2))
                .sqrt()
                + angle(correction.rotation) * reach;
            if shift >= params.min_shift_m {
                corrections.insert(m.keyframe.id, correction);
            }
        }
        if corrections.is_empty() {
            return None;
        }
        let anchors: Vec<Anchor> = self
            .applied
            .iter()
            .map(|(id, a)| Anchor {
                position: a.pose.translation,
                ts: a.ts,
                correction: corrections.get(id).copied().unwrap_or(Pose::IDENTITY),
                moved: corrections.contains_key(id),
            })
            .collect();
        for (id, correction) in &corrections {
            let a = self.applied.get_mut(id).expect("inserted above");
            a.pose = correction.compose(&a.pose);
        }
        let mut warp = Warp::new(anchors, sigma, time_sigma, reach);
        warp.rebuild_fraction = params.rebuild_fraction;
        Some(warp)
    }
}

/// Mean distance from each point to its nearest other, on a hash grid.
fn mean_spacing(points: impl Iterator<Item = Vec3>) -> f64 {
    let points: Vec<Vec3> = points.collect();
    if points.len() < 2 {
        return 0.5;
    }
    let cell = 1.0;
    let key = |p: &Vec3| {
        (
            (p.x / cell).floor() as i64,
            (p.y / cell).floor() as i64,
            (p.z / cell).floor() as i64,
        )
    };
    let mut grid: AHashMap<(i64, i64, i64), Vec<usize>> = AHashMap::new();
    for (i, p) in points.iter().enumerate() {
        grid.entry(key(p)).or_default().push(i);
    }
    let nearest: Vec<f64> = (0..points.len())
        .into_par_iter()
        .map(|i| {
            let p = points[i];
            let (kx, ky, kz) = key(&p);
            for rings in 1..=8_i64 {
                let mut best = f64::INFINITY;
                for dx in -rings..=rings {
                    for dy in -rings..=rings {
                        for dz in -rings..=rings {
                            let Some(ids) = grid.get(&(kx + dx, ky + dy, kz + dz)) else {
                                continue;
                            };
                            for &j in ids {
                                if j == i {
                                    continue;
                                }
                                let q = points[j];
                                let d = ((p.x - q.x).powi(2)
                                    + (p.y - q.y).powi(2)
                                    + (p.z - q.z).powi(2))
                                .sqrt();
                                best = best.min(d);
                            }
                        }
                    }
                }
                // Within the searched cube, anything nearer than `rings` cells is exact.
                if best <= (rings as f64) * cell {
                    return best;
                }
            }
            f64::NAN
        })
        .collect();
    let found: Vec<f64> = nearest.into_iter().filter(|d| d.is_finite()).collect();
    if found.is_empty() {
        return 0.5;
    }
    found.iter().sum::<f64>() / found.len() as f64
}

/// Mean gap from each time to its nearest other.
fn mean_interval(times: impl Iterator<Item = f64>) -> f64 {
    let mut times: Vec<f64> = times.collect();
    if times.len() < 2 {
        return 1.0;
    }
    times.sort_by(f64::total_cmp);
    let n = times.len();
    let total: f64 = (0..n)
        .map(|i| {
            let before = if i > 0 {
                times[i] - times[i - 1]
            } else {
                f64::INFINITY
            };
            let after = if i + 1 < n {
                times[i + 1] - times[i]
            } else {
                f64::INFINITY
            };
            before.min(after)
        })
        .sum();
    total / n as f64
}

#[derive(Clone, Copy, Debug)]
struct Anchor {
    position: Vec3,
    ts: f64,
    correction: Pose,
    moved: bool,
}

/// Square tiles of `reach` meters in the ground plane.
pub type Tile = (i32, i32);

/// One closure's warp: every keyframe as an anchor carrying its correction
/// (identity for the unmoved), blended per point by fast-reform. Points only
/// see the anchors within reach of their tile, and a tile with no moved
/// anchor among them is the identity.
pub struct Warp {
    anchors: Vec<Anchor>,
    by_tile: AHashMap<Tile, Vec<u32>>,
    affected: AHashSet<Tile>,
    sigma: f64,
    time_sigma: f64,
    reach: f64,
    rebuild_fraction: f64,
}

impl Warp {
    fn new(anchors: Vec<Anchor>, sigma: f64, time_sigma: f64, reach: f64) -> Self {
        let mut by_tile: AHashMap<Tile, Vec<u32>> = AHashMap::new();
        for (i, a) in anchors.iter().enumerate() {
            by_tile
                .entry(tile_of(a.position.x, a.position.y, reach))
                .or_default()
                .push(i as u32);
        }
        let mut warp = Self {
            anchors,
            by_tile,
            affected: AHashSet::new(),
            sigma,
            time_sigma,
            reach,
            rebuild_fraction: 1.0,
        };
        // A moved anchor reaches the tiles around its own. A tile too far from
        // every anchor falls back to its nearest ones, see affects_tile.
        for a in warp.anchors.iter().filter(|a| a.moved) {
            let (tx, ty) = tile_of(a.position.x, a.position.y, reach);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    warp.affected.insert((tx + dx, ty + dy));
                }
            }
        }
        warp
    }

    pub fn rebuild_fraction(&self) -> f64 {
        self.rebuild_fraction
    }

    pub fn sigma(&self) -> f64 {
        self.sigma
    }

    pub fn time_sigma(&self) -> f64 {
        self.time_sigma
    }

    pub fn moved_keyframes(&self) -> usize {
        self.anchors.iter().filter(|a| a.moved).count()
    }

    pub fn keyframes(&self) -> usize {
        self.anchors.len()
    }

    /// How far the moved keyframes' own positions move: (median, max) in meters.
    pub fn keyframe_shift_m(&self) -> (f64, f64) {
        let mut shifts: Vec<f64> = self
            .anchors
            .iter()
            .filter(|a| a.moved)
            .map(|a| {
                let to = a.correction.apply(a.position);
                ((to.x - a.position.x).powi(2)
                    + (to.y - a.position.y).powi(2)
                    + (to.z - a.position.z).powi(2))
                .sqrt()
            })
            .collect();
        if shifts.is_empty() {
            return (0.0, 0.0);
        }
        shifts.sort_by(f64::total_cmp);
        (shifts[shifts.len() / 2], shifts[shifts.len() - 1])
    }

    /// The farthest a warped point may sensibly land from where it was: the
    /// largest keyframe shift plus a rotation's swing over a tile and its neighbors.
    pub fn max_plausible_move_m(&self) -> f64 {
        let (_, max_shift) = self.keyframe_shift_m();
        let max_angle = self
            .anchors
            .iter()
            .filter(|a| a.moved)
            .map(|a| angle(a.correction.rotation))
            .fold(0.0, f64::max);
        max_shift + max_angle * 3.0 * self.reach + self.reach
    }

    /// The same correction as one fast-reform graph over every keyframe, with
    /// no tiling or time windows, the reference the tiled warp approximates.
    pub fn graph_delta(&self) -> fast_reform::GraphDelta3D {
        fast_reform::GraphDelta3D {
            nodes: self
                .anchors
                .iter()
                .enumerate()
                .map(|(i, a)| fast_reform::Node {
                    id: i as u64,
                    ts: a.ts,
                    position: a.position,
                })
                .collect(),
            transforms: self
                .anchors
                .iter()
                .map(|a| fast_reform::Transform {
                    translation: a.correction.translation,
                    rotation: a.correction.rotation,
                })
                .collect(),
            blend_sigma: self.sigma,
            blend_time_sigma: self.time_sigma,
        }
    }

    /// The tile a ground position falls in.
    pub fn tile(&self, x: f32, y: f32) -> Tile {
        tile_of(x as f64, y as f64, self.reach)
    }

    /// Whether the warp can move a point in this tile.
    pub fn affects_tile(&self, tile: Tile) -> bool {
        if self.affected.contains(&tile) {
            return true;
        }
        // Far from every anchor, a point takes its nearest anchors' blend.
        self.near_anchors(tile).is_none()
            && self
                .fallback_anchors(tile)
                .iter()
                .any(|&i| self.anchors[i as usize].moved)
    }

    /// Whether the warp can move a point at this ground position.
    pub fn affects(&self, x: f32, y: f32) -> bool {
        self.affects_tile(self.tile(x, y))
    }

    /// Anchors in the 3x3 tiles around `tile`, None when there are none.
    fn near_anchors(&self, (tx, ty): Tile) -> Option<Vec<u32>> {
        let mut out = Vec::new();
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(ids) = self.by_tile.get(&(tx + dx, ty + dy)) {
                    out.extend_from_slice(ids);
                }
            }
        }
        (!out.is_empty()).then_some(out)
    }

    /// The anchors of the nearest ring of tiles holding any.
    fn fallback_anchors(&self, (tx, ty): Tile) -> Vec<u32> {
        for ring in 2..=MAX_FALLBACK_RINGS {
            let mut out = Vec::new();
            for dx in -ring..=ring {
                for dy in -ring..=ring {
                    if dx.abs() != ring && dy.abs() != ring {
                        continue;
                    }
                    if let Some(ids) = self.by_tile.get(&(tx + dx, ty + dy)) {
                        out.extend_from_slice(ids);
                    }
                }
            }
            if !out.is_empty() {
                return out;
            }
        }
        Vec::new()
    }

    fn blend_of(&self, ids: &[u32]) -> BlendDeltas {
        let anchors: Vec<&Anchor> = ids.iter().map(|&i| &self.anchors[i as usize]).collect();
        BlendDeltas {
            positions: anchors.iter().map(|a| a.position).collect(),
            times: anchors.iter().map(|a| a.ts).collect(),
            rotations: anchors.iter().map(|a| a.correction.rotation).collect(),
            translations: anchors.iter().map(|a| a.correction.translation).collect(),
            sigma: self.sigma,
            time_sigma: self.time_sigma,
        }
    }

    fn any_moved(&self, ids: &[u32]) -> bool {
        ids.iter().any(|&i| self.anchors[i as usize].moved)
    }

    /// Warp one tile's points, or None when no moved anchor reaches them.
    /// Timed points go in runs spanning one time sigma, each against only
    /// the anchors within TIME_REACH_SIGMAS of it in time: at a place passed
    /// many times, a point blends against its own pass, not all of them.
    fn warp_tile(
        &self,
        tile: Tile,
        ids: Vec<usize>,
        points: &[[f32; 3]],
        times: &[f64],
    ) -> Option<(Vec<usize>, Vec<[f32; 3]>)> {
        let mut anchors = self
            .near_anchors(tile)
            .unwrap_or_else(|| self.fallback_anchors(tile));
        if !self.any_moved(&anchors) {
            return None;
        }
        anchors.sort_by(|&a, &b| {
            self.anchors[a as usize]
                .ts
                .total_cmp(&self.anchors[b as usize].ts)
        });
        let anchor_ts: Vec<f64> = anchors
            .iter()
            .map(|&i| self.anchors[i as usize].ts)
            .collect();
        let (mut timed, untimed): (Vec<usize>, Vec<usize>) =
            ids.into_iter().partition(|&i| times[i].is_finite());
        timed.sort_by(|&a, &b| times[a].total_cmp(&times[b]));

        // Runs spanning one time sigma, each warped against the anchors near it in time,
        // in parallel.
        let mut runs: Vec<&[usize]> = Vec::new();
        let mut run_start = 0;
        while run_start < timed.len() {
            let t0 = times[timed[run_start]];
            let mut run_end = run_start + 1;
            while run_end < timed.len() && times[timed[run_end]] <= t0 + self.time_sigma {
                run_end += 1;
            }
            runs.push(&timed[run_start..run_end]);
            run_start = run_end;
        }
        let reach = TIME_REACH_SIGMAS * self.time_sigma;
        let warped_runs: Vec<(&[usize], Vec<[f32; 3]>)> = runs
            .into_par_iter()
            .filter_map(|run| {
                let (t0, t1) = (times[run[0]], times[run[run.len() - 1]]);
                let lo = anchor_ts.partition_point(|&t| t < t0 - reach);
                let hi = anchor_ts.partition_point(|&t| t <= t1 + reach);
                // No anchor near in time leaves fast-reform's own fallback, the
                // strongest anchor of all.
                let near = if lo < hi {
                    &anchors[lo..hi]
                } else {
                    &anchors[..]
                };
                if !self.any_moved(near) {
                    return None;
                }
                let pts: Vec<[f32; 3]> = run.iter().map(|&i| points[i]).collect();
                let ts: Vec<f64> = run.iter().map(|&i| times[i]).collect();
                Some((run, warp_positions(&pts, Some(&ts), &self.blend_of(near))))
            })
            .collect();
        let mut order = Vec::with_capacity(timed.len() + untimed.len());
        let mut out = Vec::with_capacity(order.capacity());
        for (run, warped) in warped_runs {
            order.extend_from_slice(run);
            out.extend(warped);
        }
        if !untimed.is_empty() {
            let pts: Vec<[f32; 3]> = untimed.iter().map(|&i| points[i]).collect();
            out.extend(warp_positions(&pts, None, &self.blend_of(&anchors)));
            order.extend(untimed);
        }
        Some((order, out))
    }

    /// Warp world points, each with the time it was observed, NaN for unknown
    /// (which blends by distance alone). Unaffected points come back as is.
    pub fn apply(&self, points: &[[f32; 3]], times: &[f64]) -> Vec<[f32; 3]> {
        assert_eq!(points.len(), times.len());
        let mut by_tile: AHashMap<Tile, Vec<usize>> = AHashMap::new();
        for (i, p) in points.iter().enumerate() {
            by_tile
                .entry(tile_of(p[0] as f64, p[1] as f64, self.reach))
                .or_default()
                .push(i);
        }
        let groups: Vec<(Tile, Vec<usize>)> = by_tile.into_iter().collect();
        let warped: Vec<(Vec<usize>, Vec<[f32; 3]>)> = groups
            .into_par_iter()
            .filter_map(|(tile, ids)| self.warp_tile(tile, ids, points, times))
            .collect();
        let mut result = points.to_vec();
        for (order, out) in warped {
            for (i, p) in order.into_iter().zip(out) {
                result[i] = p;
            }
        }
        result
    }
}

fn tile_of(x: f64, y: f64, edge: f64) -> Tile {
    ((x / edge).floor() as i32, (y / edge).floor() as i32)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn yaw(rad: f64) -> Quat {
        Quat::new(0.0, 0.0, (rad / 2.0).sin(), (rad / 2.0).cos())
    }

    fn put_pose(out: &mut Vec<u8>, id: u32, ts: f64, pose: &Pose) {
        let t = pose.translation;
        for v in [t.x, t.y, t.z] {
            out.extend_from_slice(&(v as f32).to_le_bytes());
        }
        out.extend_from_slice(&id.to_le_bytes());
        out.extend_from_slice(&ts.to_le_bytes());
        let q = pose.rotation;
        for v in [q.x, q.y, q.z, q.w] {
            out.extend_from_slice(&(v as f32).to_le_bytes());
        }
    }

    fn at(x: f64, y: f64) -> Pose {
        Pose {
            rotation: Quat::IDENTITY,
            translation: Vec3::new(x, y, 0.0),
        }
    }

    fn event(moves: &[(u32, f64, Pose, Pose)]) -> PointCloud2 {
        let mut data = Vec::new();
        for (id, ts, pre, delta) in moves {
            put_pose(&mut data, *id, *ts, pre);
            let t = delta.translation;
            for v in [t.x, t.y, t.z] {
                data.extend_from_slice(&(v as f32).to_le_bytes());
            }
            let q = delta.rotation;
            for v in [q.x, q.y, q.z, q.w] {
                data.extend_from_slice(&(v as f32).to_le_bytes());
            }
        }
        record_cloud(LOOP_EVENT_FIELDS, data)
    }

    fn params() -> ReformParams {
        ReformParams {
            blend_sigma_m: 1.0,
            blend_time_sigma_s: 1.0,
            min_shift_m: 0.02,
            rebuild_fraction: 1.0,
        }
    }

    #[test]
    fn records_decode_at_the_gsc_offsets() {
        let pose = Pose {
            rotation: yaw(0.3),
            translation: Vec3::new(1.0, -2.0, 0.5),
        };
        let mut data = Vec::new();
        put_pose(&mut data, 7, 12.5, &pose);
        let msg = record_cloud(POSE_GRAPH_FIELDS, data);
        assert_eq!(msg.point_step, 40);
        let got = decode_pose_graph(&msg).unwrap();
        assert_eq!(got.len(), 1);
        assert_eq!((got[0].id, got[0].ts), (7, 12.5));
        assert!((got[0].pose.translation.y + 2.0).abs() < 1e-6);
        assert!((got[0].pose.rotation.z - yaw(0.3).z).abs() < 1e-6);

        let delta = Pose {
            rotation: yaw(0.1),
            translation: Vec3::new(0.2, 0.0, 0.0),
        };
        let msg = event(&[(3, 1.0, pose, delta)]);
        assert_eq!(msg.point_step, 68);
        let offsets: Vec<i32> = msg.fields.iter().map(|f| f.offset).collect();
        assert_eq!(offsets[9..], [40, 44, 48, 52, 56, 60, 64]);
        let got = decode_loop_closure_event(&msg).unwrap();
        assert_eq!(got[0].keyframe.id, 3);
        assert!((got[0].delta.translation.x - 0.2).abs() < 1e-6);
        let post = got[0].post();
        let want = delta.compose(&pose);
        assert!((post.translation.x - want.translation.x).abs() < 1e-5);
    }

    #[test]
    fn decode_rejects_a_short_buffer_and_a_wrong_type() {
        let mut msg = record_cloud(POSE_GRAPH_FIELDS, vec![0; 40]);
        msg.width = 2;
        assert!(decode_pose_graph(&msg).is_err());
        let mut msg = record_cloud(POSE_GRAPH_FIELDS, vec![0; 40]);
        msg.fields[3].datatype = PointField::FLOAT32 as u8;
        assert!(decode_pose_graph(&msg).is_err());
    }

    /// Keyframes along x at 1 m, the second half shifted +0.5 m in y.
    fn half_shifted() -> (ReformState, Vec<MovedRecord>) {
        let mut state = ReformState::default();
        let kfs: Vec<KeyframeRecord> = (0..20)
            .map(|i| KeyframeRecord {
                id: i,
                ts: i as f64,
                pose: at(i as f64, 0.0),
            })
            .collect();
        state.add_keyframes(&kfs);
        let shift = Pose {
            rotation: Quat::IDENTITY,
            translation: Vec3::new(0.0, 0.5, 0.0),
        };
        let moved = decode_loop_closure_event(&event(
            &(10..20)
                .map(|i| (i, i as f64, at(i as f64, 0.0), shift))
                .collect::<Vec<_>>(),
        ))
        .unwrap();
        (state, moved)
    }

    #[test]
    fn a_closure_moves_points_by_their_keyframes_and_leaves_far_ones() {
        let (mut state, moved) = half_shifted();
        let warp = state.closure(&moved, &params()).expect("keyframes moved");
        assert_eq!(warp.moved_keyframes(), 10);
        let points = [[15.0, 0.3, 0.0], [2.0, 0.3, 0.0], [-30.0, 0.0, 0.0]];
        let out = warp.apply(&points, &[15.0, 2.0, f64::NAN]);
        assert!((out[0][1] - 0.8).abs() < 0.01, "{:?}", out[0]);
        assert!((out[1][1] - 0.3).abs() < 0.01, "{:?}", out[1]);
        assert_eq!(out[2], points[2]);
        assert!(!warp.affects(-30.0, 0.0));
        assert!(warp.affects(15.0, 0.0));
    }

    #[test]
    fn time_picks_the_segment_a_point_was_seen_from_at_a_seam() {
        // Keyframes 0..10 go out along x at y=0, 10..20 come back over the
        // same line. Only the return leg is corrected.
        let mut state = ReformState::default();
        let kfs: Vec<KeyframeRecord> = (0..20)
            .map(|i| {
                let x = if i < 10 { i } else { 19 - i } as f64;
                KeyframeRecord {
                    id: i,
                    ts: i as f64 * 10.0,
                    pose: at(x, 0.0),
                }
            })
            .collect();
        state.add_keyframes(&kfs);
        let shift = Pose {
            rotation: Quat::IDENTITY,
            translation: Vec3::new(0.0, 1.0, 0.0),
        };
        let moved: Vec<MovedRecord> = kfs[10..]
            .iter()
            .map(|k| MovedRecord {
                keyframe: *k,
                delta: shift,
            })
            .collect();
        let p = ReformParams {
            blend_time_sigma_s: 10.0,
            ..params()
        };
        let warp = state.closure(&moved, &p).unwrap();
        // The same place, seen on the way out and on the way back.
        let out = warp.apply(&[[5.0, 0.0, 0.0], [5.0, 0.0, 0.0]], &[50.0, 140.0]);
        assert!(out[0][1].abs() < 0.05, "outbound moved: {:?}", out[0]);
        assert!(
            (out[1][1] - 1.0).abs() < 0.05,
            "return did not: {:?}",
            out[1]
        );
    }

    #[test]
    fn a_shift_too_small_to_warp_carries_into_the_next_closure() {
        let mut state = ReformState::default();
        state.add_keyframes(&[KeyframeRecord {
            id: 0,
            ts: 0.0,
            pose: at(0.0, 0.0),
        }]);
        let nudge = |pre: Pose| MovedRecord {
            keyframe: KeyframeRecord {
                id: 0,
                ts: 0.0,
                pose: pre,
            },
            delta: Pose {
                rotation: Quat::IDENTITY,
                translation: Vec3::new(0.015, 0.0, 0.0),
            },
        };
        assert!(state.closure(&[nudge(at(0.0, 0.0))], &params()).is_none());
        // The optimizer now holds it at 0.015 and nudges it again.
        let warp = state
            .closure(&[nudge(at(0.015, 0.0))], &params())
            .expect("the moves add up past min_shift");
        let out = warp.apply(&[[0.0, 0.0, 0.0]], &[0.0]);
        assert!((out[0][0] - 0.03).abs() < 1e-4, "{:?}", out[0]);
    }

    #[test]
    fn a_rotation_turns_points_about_the_world_origin_as_post_is_delta_pre() {
        let mut state = ReformState::default();
        let pre = Pose {
            rotation: Quat::IDENTITY,
            translation: Vec3::new(10.0, 0.0, 0.0),
        };
        state.add_keyframes(&[KeyframeRecord {
            id: 0,
            ts: 0.0,
            pose: pre,
        }]);
        let delta = Pose {
            rotation: yaw(std::f64::consts::FRAC_PI_2),
            translation: Vec3::ZERO,
        };
        let warp = state
            .closure(
                &[MovedRecord {
                    keyframe: KeyframeRecord {
                        id: 0,
                        ts: 0.0,
                        pose: pre,
                    },
                    delta,
                }],
                &params(),
            )
            .unwrap();
        let out = warp.apply(&[[10.0, 0.0, 0.0]], &[0.0]);
        assert!(
            out[0][0].abs() < 1e-4 && (out[0][1] - 10.0).abs() < 1e-4,
            "{:?}",
            out[0]
        );
    }

    #[test]
    fn mean_spacing_matches_brute_force() {
        let pts: Vec<Vec3> = (0..50)
            .map(|i| {
                Vec3::new(
                    (i as f64 * 0.37).sin() * 5.0,
                    (i as f64 * 0.91).cos() * 5.0,
                    0.0,
                )
            })
            .collect();
        let brute: f64 = pts
            .iter()
            .enumerate()
            .map(|(i, p)| {
                pts.iter()
                    .enumerate()
                    .filter(|(j, _)| *j != i)
                    .map(|(_, q)| ((p.x - q.x).powi(2) + (p.y - q.y).powi(2)).sqrt())
                    .fold(f64::INFINITY, f64::min)
            })
            .sum::<f64>()
            / pts.len() as f64;
        assert!((mean_spacing(pts.into_iter()) - brute).abs() < 1e-9);
    }
}
