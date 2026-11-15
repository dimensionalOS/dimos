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

//! Lazy loop-closure reform: GscPGO's keyframes (`new_keyframe`) and versioned
//! closures (`loop_closure_event`), the map's version layers, and the reverse
//! lookup from a current-frame area to the version 0 tiles stored there.
//!
//! Every voxel and cell is anchored to a keyframe. A layer holds what was last
//! written or reformed at one closure version, all in that version's frame;
//! only the current layer is computed in. A stale part moves into the current
//! layer when something near it is touched, by its anchor's pose then and now.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use lcm_msgs::sensor_msgs::{PointCloud2, PointField};

use crate::adjacency::{CellId, SurfaceLookup};
use crate::columns::ColumnIz;
use crate::sharded_map::ShardedMap;
use crate::terrain::{PlaneMap, SurfacePlane};
use crate::voxel::VoxelKey;

/// A closure version: 0 before any closure, then GscPGO's event count.
pub type Version = u32;

/// The anchor of an element written before any keyframe was known; it never moves.
pub const NO_ANCHOR: u32 = u32::MAX;

/// A loop_closure_event record's `closed_with` when the keyframe closed with none.
pub const CLOSED_WITH_NONE: u32 = u32::MAX;

/// How the planner follows GscPGO's loop closures (its `new_keyframe` and `loop_closure_event` inputs).
#[derive(Clone, Copy, Debug, serde::Deserialize, serde::Serialize, validator::Validate)]
#[serde(deny_unknown_fields)]
pub struct ReformConfig {
    /// Follow closures; off ignores new_keyframe and loop_closure_event.
    pub enabled: bool,
    /// Edge of a version 0 tile, the extent of a submesh, m.
    #[validate(range(exclusive_min = 0.0))]
    pub tile_m: f32,
    /// Extra tiles around each area a lookup maps to version 0.
    #[validate(range(min = 0))]
    pub tile_pad: i32,
    /// Edge of the coarse grid keyframe boxes are filed in, m.
    #[validate(range(exclusive_min = 0.0))]
    pub box_grid_m: f32,
    /// How far each keyframe box grows before it is filed: the most a reform moves a point off its keyframe's rigid motion, m.
    #[validate(range(min = 0.0))]
    pub box_pad_m: f32,
    /// Past layers whose keyframe poses are kept, for clouds stamped with an older version.
    #[validate(range(min = 1))]
    pub version_history: u32,
    /// A closure starts a new layer when it closes a loop or moves some keyframe's box corner this far from the current layer's frame, m (0 = half a voxel).
    #[validate(range(min = 0.0))]
    pub min_shift_m: f32,
    /// Stale elements (voxels and cells) migrated per region update, from the submeshes around the update first and
    /// then those inside a closure's closing boxes, so no one update carries a whole merge (0 = all at once).
    pub migrate_per_update: u32,
}

impl ReformConfig {
    /// Closures ignored, for tests and tools that never reform.
    pub fn off() -> Self {
        Self {
            enabled: false,
            tile_m: 10.0,
            tile_pad: 1,
            box_grid_m: 10.0,
            box_pad_m: 1.0,
            version_history: 4,
            min_shift_m: 0.0,
            migrate_per_update: 4000,
        }
    }

    /// The shift that starts a new layer: min_shift_m, or half a voxel when 0.
    pub fn layer_shift_m(&self, voxel_size: f32) -> f64 {
        if self.min_shift_m > 0.0 {
            self.min_shift_m as f64
        } else {
            0.5 * voxel_size as f64
        }
    }

    pub fn table_params(&self) -> TableParams {
        TableParams {
            box_grid_m: self.box_grid_m,
            box_pad_m: self.box_pad_m,
            version_history: self.version_history as usize,
        }
    }

    pub fn layer_params(&self) -> LayerParams {
        LayerParams {
            tile_m: self.tile_m,
            tile_pad: self.tile_pad,
        }
    }
}

/// A rigid pose: unit quaternion (x, y, z, w) and translation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub rotation: [f64; 4],
    pub translation: [f64; 3],
}

impl Pose {
    pub const IDENTITY: Pose = Pose {
        rotation: [0.0, 0.0, 0.0, 1.0],
        translation: [0.0; 3],
    };

    /// A pose from a translation and a quaternion, the quaternion normalized.
    pub fn new(translation: [f64; 3], rotation: [f64; 4]) -> Pose {
        let n = rotation.iter().map(|c| c * c).sum::<f64>().sqrt();
        let rotation = if n > 0.0 && n.is_finite() {
            rotation.map(|c| c / n)
        } else {
            Self::IDENTITY.rotation
        };
        Pose {
            rotation,
            translation,
        }
    }

    /// A pose turned `yaw` radians about z.
    pub fn from_yaw(translation: [f64; 3], yaw: f64) -> Pose {
        let half = 0.5 * yaw;
        Pose::new(translation, [0.0, 0.0, half.sin(), half.cos()])
    }

    /// The rotation alone applied to `v`.
    pub fn rotate(&self, v: [f64; 3]) -> [f64; 3] {
        let [qx, qy, qz, qw] = self.rotation;
        // v + 2w (q × v) + 2 q × (q × v)
        let t = [
            2.0 * (qy * v[2] - qz * v[1]),
            2.0 * (qz * v[0] - qx * v[2]),
            2.0 * (qx * v[1] - qy * v[0]),
        ];
        [
            v[0] + qw * t[0] + (qy * t[2] - qz * t[1]),
            v[1] + qw * t[1] + (qz * t[0] - qx * t[2]),
            v[2] + qw * t[2] + (qx * t[1] - qy * t[0]),
        ]
    }

    pub fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        let r = self.rotate(p);
        [
            r[0] + self.translation[0],
            r[1] + self.translation[1],
            r[2] + self.translation[2],
        ]
    }

    /// self * other.
    pub fn compose(&self, other: &Pose) -> Pose {
        let [ax, ay, az, aw] = self.rotation;
        let [bx, by, bz, bw] = other.rotation;
        let rotation = [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ];
        Pose::new(self.apply(other.translation), rotation)
    }

    pub fn inverse(&self) -> Pose {
        let [x, y, z, w] = self.rotation;
        let conjugate = Pose {
            rotation: [-x, -y, -z, w],
            translation: [0.0; 3],
        };
        let t = conjugate.rotate(self.translation);
        Pose {
            rotation: conjugate.rotation,
            translation: [-t[0], -t[1], -t[2]],
        }
    }
}

/// One `new_keyframe` record: the keyframe's raw odometry pose and its box, in its own frame, around every scan since the previous keyframe.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KeyframeRecord {
    pub node_id: u32,
    pub ts: f64,
    pub raw: Pose,
    /// min xyz then max xyz; +inf then -inf when no scan gave it points.
    pub scan_box: [f32; 6],
}

/// One `loop_closure_event` record: a keyframe's pose before the solve and the change, post = delta * pre.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EventRecord {
    pub node_id: u32,
    pub pre: Pose,
    pub delta: Pose,
    /// The event's count, when GscPGO sends one.
    pub version: Option<Version>,
    /// The keyframe the newest closure since the previous event paired this one with.
    pub closed_with: Option<u32>,
}

impl EventRecord {
    pub fn post(&self) -> Pose {
        self.delta.compose(&self.pre)
    }
}

#[derive(Debug, PartialEq)]
pub struct DecodeError(pub String);

impl std::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

/// A record cloud's field offsets by name, each checked for its datatype.
pub(crate) struct Fields<'a> {
    msg: &'a PointCloud2,
    step: usize,
    pub count: usize,
}

impl<'a> Fields<'a> {
    pub fn new(msg: &'a PointCloud2) -> Result<Self, DecodeError> {
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

    /// The offset of a field, None when the cloud has no field of that name.
    pub fn optional(
        &self,
        name: &str,
        datatype: u8,
        size: usize,
    ) -> Result<Option<usize>, DecodeError> {
        let Some(field) = self.msg.fields.iter().find(|f| f.name == name) else {
            return Ok(None);
        };
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
        Ok(Some(offset))
    }

    pub fn offset(&self, name: &str, datatype: u8, size: usize) -> Result<usize, DecodeError> {
        self.optional(name, datatype, size)?
            .ok_or_else(|| DecodeError(format!("missing field {name}")))
    }

    fn f32s<const N: usize>(&self, names: [&str; N]) -> Result<[usize; N], DecodeError> {
        let mut out = [0; N];
        for (slot, name) in out.iter_mut().zip(names) {
            *slot = self.offset(name, PointField::FLOAT32 as u8, 4)?;
        }
        Ok(out)
    }

    pub fn record(&self, i: usize) -> &[u8] {
        &self.msg.data[i * self.step..(i + 1) * self.step]
    }
}

pub(crate) fn read<const N: usize>(record: &[u8], offset: usize) -> [u8; N] {
    record[offset..offset + N]
        .try_into()
        .expect("offset checked against the step")
}

fn f32_at(record: &[u8], offset: usize) -> f32 {
    f32::from_le_bytes(read(record, offset))
}

fn u32_at(record: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(read(record, offset))
}

fn pose_at(record: &[u8], t: &[usize; 3], q: &[usize; 4]) -> Pose {
    let f = |o: usize| f32_at(record, o) as f64;
    Pose::new(
        [f(t[0]), f(t[1]), f(t[2])],
        [f(q[0]), f(q[1]), f(q[2]), f(q[3])],
    )
}

/// An LCM-encoded PointCloud2, as recorded or sent on the wire.
pub fn decode_cloud(bytes: &[u8]) -> Result<PointCloud2, DecodeError> {
    PointCloud2::decode(bytes).map_err(|e| DecodeError(e.to_string()))
}

/// Decode a `new_keyframe` cloud, one record per keyframe.
pub fn decode_new_keyframe(msg: &PointCloud2) -> Result<Vec<KeyframeRecord>, DecodeError> {
    let fields = Fields::new(msg)?;
    let t = fields.f32s(["x", "y", "z"])?;
    let q = fields.f32s(["qx", "qy", "qz", "qw"])?;
    let b = fields.f32s(["min_x", "min_y", "min_z", "max_x", "max_y", "max_z"])?;
    let id = fields.offset("node_id", PointField::UINT32 as u8, 4)?;
    let ts = fields.offset("ts", PointField::FLOAT64 as u8, 8)?;
    Ok((0..fields.count)
        .map(|i| {
            let r = fields.record(i);
            KeyframeRecord {
                node_id: u32_at(r, id),
                ts: f64::from_le_bytes(read(r, ts)),
                raw: pose_at(r, &t, &q),
                scan_box: b.map(|o| f32_at(r, o)),
            }
        })
        .collect())
}

/// Decode a `loop_closure_event` cloud, one record per keyframe; `version` and `closed_with` are read when present.
pub fn decode_loop_closure_event(msg: &PointCloud2) -> Result<Vec<EventRecord>, DecodeError> {
    let fields = Fields::new(msg)?;
    let t = fields.f32s(["x", "y", "z"])?;
    let q = fields.f32s(["qx", "qy", "qz", "qw"])?;
    let dt = fields.f32s(["dx", "dy", "dz"])?;
    let dq = fields.f32s(["dqx", "dqy", "dqz", "dqw"])?;
    let id = fields.offset("node_id", PointField::UINT32 as u8, 4)?;
    let version = fields.optional("version", PointField::UINT32 as u8, 4)?;
    let closed_with = fields.optional("closed_with", PointField::UINT32 as u8, 4)?;
    Ok((0..fields.count)
        .map(|i| {
            let r = fields.record(i);
            EventRecord {
                node_id: u32_at(r, id),
                pre: pose_at(r, &t, &q),
                delta: pose_at(r, &dt, &dq),
                version: version.map(|o| u32_at(r, o)),
                closed_with: closed_with
                    .map(|o| u32_at(r, o))
                    .filter(|&k| k != CLOSED_WITH_NONE),
            }
        })
        .collect())
}

/// An xy rectangle, (x0, x1, y0, y1), in meters.
pub type Rect = [f32; 4];

/// Whether two rectangles overlap.
pub fn overlaps(a: Rect, b: Rect) -> bool {
    a[0] <= b[1] && b[0] <= a[1] && a[2] <= b[3] && b[2] <= a[3]
}

/// What the keyframe table needs from the config.
#[derive(Clone, Copy, Debug)]
pub struct TableParams {
    /// Edge of the coarse grid keyframe boxes are filed in, m.
    pub box_grid_m: f32,
    /// How far each box is grown before it is filed, m.
    pub box_pad_m: f32,
    /// How many past layers' frames are kept, for clouds stamped with an older version.
    pub version_history: usize,
}

/// The keyframes as GscPGO sends them: raw pose and box once, the current pose kept from each closure.
pub struct KeyframeTable {
    params: TableParams,
    node_ids: Vec<u32>,
    raw: Vec<Pose>,
    now: Vec<Pose>,
    scan_box: Vec<[f32; 6]>,
    slot_of: BTreeMap<u32, u32>,
    version: Version,
    /// now * raw^-1 of the newest keyframe at the last closure, which places keyframes made since.
    correction: Pose,
    /// The frame the current layer is built in: the version it started at, every keyframe's pose then, and the correction then.
    frame: Frame,
    /// Past layers' frames, newest last, for clouds stamped with an older version.
    frames: VecDeque<Frame>,
    /// Coarse grid cell -> keyframes whose padded current box reaches it.
    box_grid: BTreeMap<(i32, i32), Vec<u32>>,
    /// Per slot, its padded box in the current frame and its pose from there to version 0, both fixed until the next frame.
    filed: Vec<(Option<Rect>, Pose)>,
}

/// A layer's frame: the version it was fixed at, the keyframes' poses then, and the correction that placed keyframes made since.
#[derive(Clone, Debug)]
struct Frame {
    version: Version,
    poses: Vec<Pose>,
    correction: Pose,
}

impl Default for Frame {
    fn default() -> Self {
        Self {
            version: 0,
            poses: Vec::new(),
            correction: Pose::IDENTITY,
        }
    }
}

impl Frame {
    /// A keyframe's pose in this frame; one made after it was fixed rides its correction.
    fn pose(&self, slot: u32, table: &KeyframeTable) -> Pose {
        self.poses
            .get(slot as usize)
            .copied()
            .unwrap_or_else(|| self.correction.compose(&table.raw(slot)))
    }
}

/// What a closure changed: its version and the keyframes that closed, by slot.
#[derive(Clone, Debug, PartialEq)]
pub struct Closure {
    pub version: Version,
    pub closing: Vec<u32>,
}

impl KeyframeTable {
    pub fn new(params: TableParams) -> Self {
        Self {
            params,
            node_ids: Vec::new(),
            raw: Vec::new(),
            now: Vec::new(),
            scan_box: Vec::new(),
            slot_of: BTreeMap::new(),
            version: 0,
            correction: Pose::IDENTITY,
            frame: Frame::default(),
            frames: VecDeque::new(),
            box_grid: BTreeMap::new(),
            filed: Vec::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.raw.len()
    }

    pub fn is_empty(&self) -> bool {
        self.raw.is_empty()
    }

    pub fn version(&self) -> Version {
        self.version
    }

    /// The newest keyframe, which new data is anchored to.
    pub fn latest(&self) -> Option<u32> {
        (!self.raw.is_empty()).then(|| self.raw.len() as u32 - 1)
    }

    pub fn slot(&self, node_id: u32) -> Option<u32> {
        self.slot_of.get(&node_id).copied()
    }

    pub fn node_id(&self, slot: u32) -> u32 {
        self.node_ids[slot as usize]
    }

    pub fn raw(&self, slot: u32) -> Pose {
        self.raw[slot as usize]
    }

    pub fn now(&self, slot: u32) -> Pose {
        self.now[slot as usize]
    }

    pub fn scan_box(&self, slot: u32) -> [f32; 6] {
        self.scan_box[slot as usize]
    }

    /// raw * frame^-1: a point near the keyframe in the current layer's frame to where it was at version 0.
    pub fn to_v0(&self, slot: u32) -> Pose {
        self.filed[slot as usize].1
    }

    /// Take a new keyframe, placed by the current correction; a known one is ignored.
    pub fn add(&mut self, record: &KeyframeRecord) -> u32 {
        if let Some(slot) = self.slot(record.node_id) {
            return slot;
        }
        let slot = self.raw.len() as u32;
        self.node_ids.push(record.node_id);
        self.raw.push(record.raw);
        self.now.push(self.correction.compose(&record.raw));
        self.scan_box.push(record.scan_box);
        self.slot_of.insert(record.node_id, slot);
        self.file_box(slot);
        slot
    }

    /// Apply a closure: each listed keyframe takes its post pose. Returns None when it lists no known keyframe.
    pub fn apply_event(&mut self, records: &[EventRecord]) -> Option<Closure> {
        let known: Vec<(u32, &EventRecord)> = records
            .iter()
            .filter_map(|r| Some((self.slot(r.node_id)?, r)))
            .collect();
        if known.is_empty() {
            return None;
        }
        let mut closing: BTreeSet<u32> = BTreeSet::new();
        for &(slot, record) in &known {
            self.now[slot as usize] = record.post();
            if let Some(partner) = record.closed_with.and_then(|id| self.slot(id)) {
                closing.insert(slot);
                closing.insert(partner);
            }
        }
        let newest = self.raw.len() as u32 - 1;
        self.correction = self.now(newest).compose(&self.raw(newest).inverse());
        let next = self.version + 1;
        self.version = known
            .iter()
            .find_map(|(_, r)| r.version)
            .map_or(next, |v| v.max(next));
        Some(Closure {
            version: self.version,
            closing: closing.into_iter().collect(),
        })
    }

    /// The version the current layer's frame was fixed at.
    pub fn frame_version(&self) -> Version {
        self.frame.version
    }

    /// Fix the current poses as the frame of a layer starting now; the old frame is kept for older clouds.
    pub fn start_frame(&mut self) {
        let next = Frame {
            version: self.version,
            poses: self.now.clone(),
            correction: self.correction,
        };
        self.frames
            .push_back(std::mem::replace(&mut self.frame, next));
        while self.frames.len() > self.params.version_history {
            self.frames.pop_front();
        }
        self.refile_boxes();
    }

    /// A keyframe's pose in the current layer's frame.
    pub fn frame_pose(&self, slot: u32) -> Pose {
        self.frame.pose(slot, self)
    }

    /// A keyframe's pose in the frame of the layer `version` falls in, None when that layer's frame is no longer kept.
    pub fn pose_at(&self, version: Version, slot: u32) -> Option<Pose> {
        if version >= self.frame.version {
            return Some(self.frame.pose(slot, self));
        }
        let frame = self.frames.iter().rev().find(|f| f.version <= version)?;
        Some(frame.pose(slot, self))
    }

    /// The farthest any keyframe's box corner has moved between the current layer's frame and now, m.
    pub fn shift_since_frame(&self) -> f64 {
        let mut worst: f64 = 0.0;
        for slot in 0..self.raw.len() as u32 {
            let (then, now) = (self.frame_pose(slot), self.now(slot));
            let b = self.scan_box(slot);
            let boxed = b[0] <= b[3] && b[1] <= b[4] && b[2] <= b[5];
            let corners = if boxed { 8 } else { 1 };
            for corner in 0..corners {
                let pick = |axis: usize| {
                    if !boxed {
                        0.0
                    } else if corner >> axis & 1 == 0 {
                        b[axis] as f64
                    } else {
                        b[axis + 3] as f64
                    }
                };
                let local = [pick(0), pick(1), pick(2)];
                let (a, c) = (then.apply(local), now.apply(local));
                let d =
                    ((a[0] - c[0]).powi(2) + (a[1] - c[1]).powi(2) + (a[2] - c[2]).powi(2)).sqrt();
                worst = worst.max(d);
            }
        }
        worst
    }

    /// The keyframe's padded box in the current layer's frame, as an xy rectangle; None for an empty box.
    pub fn current_rect(&self, slot: u32) -> Option<Rect> {
        self.filed[slot as usize].0
    }

    /// current_rect from the box and the frame pose, for filing.
    fn frame_rect(&self, slot: u32) -> Option<Rect> {
        let b = self.scan_box(slot);
        if !(b[0] <= b[3] && b[1] <= b[4] && b[2] <= b[5]) {
            return None;
        }
        let pose = self.frame_pose(slot);
        let mut rect = [
            f32::INFINITY,
            f32::NEG_INFINITY,
            f32::INFINITY,
            f32::NEG_INFINITY,
        ];
        for corner in 0..8 {
            let pick = |axis: usize| {
                if corner >> axis & 1 == 0 {
                    b[axis]
                } else {
                    b[axis + 3]
                }
            } as f64;
            let p = pose.apply([pick(0), pick(1), pick(2)]);
            rect[0] = rect[0].min(p[0] as f32);
            rect[1] = rect[1].max(p[0] as f32);
            rect[2] = rect[2].min(p[1] as f32);
            rect[3] = rect[3].max(p[1] as f32);
        }
        let pad = self.params.box_pad_m;
        Some([rect[0] - pad, rect[1] + pad, rect[2] - pad, rect[3] + pad])
    }

    fn grid_cells(&self, rect: Rect) -> impl Iterator<Item = (i32, i32)> {
        let g = self.params.box_grid_m;
        let cell = |v: f32| (v / g).floor() as i32;
        let (x0, x1, y0, y1) = (cell(rect[0]), cell(rect[1]), cell(rect[2]), cell(rect[3]));
        (x0..=x1).flat_map(move |x| (y0..=y1).map(move |y| (x, y)))
    }

    fn file_box(&mut self, slot: u32) {
        let rect = self.frame_rect(slot);
        let to_v0 = self.raw(slot).compose(&self.frame_pose(slot).inverse());
        let at = slot as usize;
        if at < self.filed.len() {
            self.filed[at] = (rect, to_v0);
        } else {
            self.filed.push((rect, to_v0));
        }
        if let Some(rect) = rect {
            let cells: Vec<(i32, i32)> = self.grid_cells(rect).collect();
            for cell in cells {
                self.box_grid.entry(cell).or_default().push(slot);
            }
        }
    }

    fn refile_boxes(&mut self) {
        self.box_grid.clear();
        for slot in 0..self.raw.len() as u32 {
            self.file_box(slot);
        }
    }

    /// The keyframes whose padded current box overlaps `rect`, ascending.
    pub fn candidates(&self, rect: Rect) -> Vec<u32> {
        let mut out: Vec<u32> = Vec::new();
        for cell in self.grid_cells(rect) {
            for &slot in self.box_grid.get(&cell).into_iter().flatten() {
                if self.current_rect(slot).is_some_and(|r| overlaps(r, rect)) {
                    out.push(slot);
                }
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }
}

/// Where an element sits: its anchor keyframe and the version 0 tile it is filed under.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Element {
    pub anchor: u32,
    pub tile: (i32, i32),
}

/// A layer no longer written to: its voxels, planes and surface by key in its own version's frame, and the poses its anchors had then. Its cells stay in the cell store, indexed as stale.
#[derive(Default)]
pub struct FrozenLayer {
    pub by_col: ColumnIz,
    pub planes: PlaneMap,
    pub voxels: ShardedMap<VoxelKey, Element>,
    pub surface_lookup: SurfaceLookup,
    pub anchor_poses: BTreeMap<u32, Pose>,
}

impl FrozenLayer {
    /// The pose to move an element into the current layer by: its anchor's pose in the current frame times its pose in this layer's.
    pub fn carry(&self, anchor: u32, table: &KeyframeTable) -> Pose {
        match self.anchor_poses.get(&anchor) {
            Some(then) if anchor != NO_ANCHOR => table.frame_pose(anchor).compose(&then.inverse()),
            _ => Pose::IDENTITY,
        }
    }
}

/// The elements of one layer filed under one version 0 tile, and how many each anchor holds.
#[derive(Default, Debug)]
pub struct Submesh {
    pub voxels: BTreeSet<VoxelKey>,
    pub cells: BTreeSet<CellId>,
    pub counts: BTreeMap<u32, u32>,
}

impl Submesh {
    pub(crate) fn is_empty(&self) -> bool {
        self.voxels.is_empty() && self.cells.is_empty()
    }

    pub(crate) fn count(&mut self, anchor: u32, delta: i64) {
        let entry = self.counts.entry(anchor).or_default();
        *entry = (*entry as i64 + delta).max(0) as u32;
        if *entry == 0 {
            self.counts.remove(&anchor);
        }
    }
}

/// What the layers need from the config.
#[derive(Clone, Copy, Debug)]
pub struct LayerParams {
    /// Edge of a version 0 tile, the extent of a submesh, m.
    pub tile_m: f32,
    /// Tiles added around each translated area.
    pub tile_pad: i32,
}

/// The version layers: the current one's element bookkeeping, the frozen ones, and the submeshes of all by version 0 tile.
pub struct Layers {
    pub params: LayerParams,
    pub current: Version,
    /// The current layer's voxels' elements.
    pub voxels: ShardedMap<VoxelKey, Element>,
    /// Every live cell's element, any layer.
    pub cells: ShardedMap<CellId, Element>,
    /// Anchors the current layer holds elements of.
    pub anchors: BTreeSet<u32>,
    pub stale: BTreeMap<Version, FrozenLayer>,
    pub submeshes: BTreeMap<(Version, (i32, i32)), Submesh>,
    /// Version 0 tile -> the layers with a submesh there.
    pub tiles: BTreeMap<(i32, i32), BTreeSet<Version>>,
}

impl Layers {
    pub fn new(params: LayerParams) -> Self {
        Self {
            params,
            current: 0,
            voxels: ShardedMap::new(),
            cells: ShardedMap::new(),
            anchors: BTreeSet::new(),
            stale: BTreeMap::new(),
            submeshes: BTreeMap::new(),
            tiles: BTreeMap::new(),
        }
    }

    pub fn tile_of(&self, x: f64, y: f64) -> (i32, i32) {
        let t = self.params.tile_m as f64;
        ((x / t).floor() as i32, (y / t).floor() as i32)
    }

    /// The element of a current-layer point: anchored to `anchor`, filed by where it sat at version 0.
    pub fn element(&self, table: &KeyframeTable, anchor: Option<u32>, p: [f64; 3]) -> Element {
        match anchor {
            Some(a) => {
                let v0 = table.to_v0(a).apply(p);
                Element {
                    anchor: a,
                    tile: self.tile_of(v0[0], v0[1]),
                }
            }
            None => Element {
                anchor: NO_ANCHOR,
                tile: self.tile_of(p[0], p[1]),
            },
        }
    }

    fn submesh(&mut self, layer: Version, tile: (i32, i32)) -> &mut Submesh {
        self.tiles.entry(tile).or_default().insert(layer);
        self.submeshes.entry((layer, tile)).or_default()
    }

    fn drop_if_empty(&mut self, layer: Version, tile: (i32, i32)) {
        if self
            .submeshes
            .get(&(layer, tile))
            .is_some_and(Submesh::is_empty)
        {
            self.submeshes.remove(&(layer, tile));
            if let Some(layers) = self.tiles.get_mut(&tile) {
                layers.remove(&layer);
                if layers.is_empty() {
                    self.tiles.remove(&tile);
                }
            }
        }
    }

    /// File a current-layer voxel.
    pub fn add_voxel(&mut self, k: VoxelKey, el: Element) {
        if let Some(old) = self.voxels.insert(k, el) {
            self.unfile_voxel(k, old);
        }
        self.anchors.insert(el.anchor);
        let current = self.current;
        let sub = self.submesh(current, el.tile);
        sub.voxels.insert(k);
        sub.count(el.anchor, 1);
    }

    fn unfile_voxel(&mut self, k: VoxelKey, el: Element) {
        let current = self.current;
        if let Some(sub) = self.submeshes.get_mut(&(current, el.tile)) {
            if sub.voxels.remove(&k) {
                sub.count(el.anchor, -1);
            }
        }
        self.drop_if_empty(current, el.tile);
    }

    /// Forget a current-layer voxel.
    pub fn remove_voxel(&mut self, k: VoxelKey) {
        if let Some(el) = self.voxels.remove(&k) {
            self.unfile_voxel(k, el);
        }
    }

    /// File a cell of `layer`.
    pub fn add_cell(&mut self, id: CellId, layer: Version, el: Element) {
        if let Some(old) = self.cells.insert(id, el) {
            self.unfile_cell_in(id, old, layer);
        }
        if layer == self.current {
            self.anchors.insert(el.anchor);
        }
        let sub = self.submesh(layer, el.tile);
        sub.cells.insert(id);
        sub.count(el.anchor, 1);
    }

    fn unfile_cell_in(&mut self, id: CellId, el: Element, layer: Version) {
        if let Some(sub) = self.submeshes.get_mut(&(layer, el.tile)) {
            if sub.cells.remove(&id) {
                sub.count(el.anchor, -1);
            }
        }
        self.drop_if_empty(layer, el.tile);
    }

    /// Forget a cell that lived in `layer`.
    pub fn remove_cell(&mut self, id: CellId, layer: Version) {
        if let Some(el) = self.cells.remove(&id) {
            self.unfile_cell_in(id, el, layer);
        }
    }

    /// The stale submeshes filed under the version 0 tiles that the area `rect` (current frame) maps to through each candidate keyframe, ascending.
    pub fn lookup(&self, table: &KeyframeTable, rect: Rect) -> Vec<(Version, (i32, i32))> {
        self.lookup_all(table, &[rect])
    }

    /// lookup over each of `rects`, the union once, ascending.
    pub fn lookup_all(&self, table: &KeyframeTable, rects: &[Rect]) -> Vec<(Version, (i32, i32))> {
        let mut tiles: ahash::AHashSet<(i32, i32)> = ahash::AHashSet::new();
        let pad = self.params.tile_pad;
        let add_square = |tiles: &mut ahash::AHashSet<(i32, i32)>, corners: &[[f64; 3]; 4]| {
            let (mut lo, mut hi) = ((i32::MAX, i32::MAX), (i32::MIN, i32::MIN));
            for c in corners {
                let t = self.tile_of(c[0], c[1]);
                lo = (lo.0.min(t.0), lo.1.min(t.1));
                hi = (hi.0.max(t.0), hi.1.max(t.1));
            }
            for x in lo.0 - pad..=hi.0 + pad {
                for y in lo.1 - pad..=hi.1 + pad {
                    tiles.insert((x, y));
                }
            }
        };
        for &rect in rects {
            let corners = [
                [rect[0] as f64, rect[2] as f64, 0.0],
                [rect[1] as f64, rect[2] as f64, 0.0],
                [rect[0] as f64, rect[3] as f64, 0.0],
                [rect[1] as f64, rect[3] as f64, 0.0],
            ];
            for slot in table.candidates(rect) {
                let to_v0 = table.to_v0(slot);
                add_square(&mut tiles, &corners.map(|c| to_v0.apply(c)));
            }
        }
        let mut found = Vec::new();
        for tile in tiles {
            for &layer in self.tiles.get(&tile).into_iter().flatten() {
                if layer < self.current {
                    found.push((layer, tile));
                }
            }
        }
        found.sort_unstable();
        found
    }
}

/// A plane carried by `carry`: its normal turned, its height re-measured at the destination voxel's center.
pub fn carry_plane(
    plane: SurfacePlane,
    from: VoxelKey,
    to: VoxelKey,
    carry: &Pose,
    voxel_size: f32,
    min_normal_z: f32,
) -> Option<SurfacePlane> {
    let center = |k: VoxelKey| {
        [
            (k.0 as f64 + 0.5) * voxel_size as f64,
            (k.1 as f64 + 0.5) * voxel_size as f64,
            (k.2 as f64 + 0.5) * voxel_size as f64,
        ]
    };
    let (gx, gy, z0) = plane.parts();
    let len = ((gx * gx + gy * gy + 1.0) as f64).sqrt();
    let normal = carry.rotate([-gx as f64 / len, -gy as f64 / len, 1.0 / len]);
    let c = center(from);
    let on_plane = carry.apply([c[0], c[1], c[2] + z0 as f64]);
    if normal[2] < min_normal_z as f64 {
        return None;
    }
    let d = center(to);
    // The plane's height at the destination column, from the carried point and normal.
    let z = on_plane[2]
        - (normal[0] * (d[0] - on_plane[0]) + normal[1] * (d[1] - on_plane[1])) / normal[2];
    Some(SurfacePlane::from_parts(
        (-normal[0] / normal[2]) as f32,
        (-normal[1] / normal[2]) as f32,
        (z - d[2]) as f32,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-9)
    }

    fn params() -> TableParams {
        TableParams {
            box_grid_m: 10.0,
            box_pad_m: 1.0,
            version_history: 2,
        }
    }

    fn keyframe(node_id: u32, raw: Pose) -> KeyframeRecord {
        KeyframeRecord {
            node_id,
            ts: node_id as f64,
            raw,
            scan_box: [-2.0, -2.0, -1.0, 2.0, 2.0, 1.0],
        }
    }

    #[test]
    fn a_pose_and_its_inverse_cancel() {
        let pose = Pose::new([1.0, -2.0, 0.5], [0.1, 0.2, 0.3, 0.9]);
        let p = [3.0, 4.0, -1.0];
        assert!(close(pose.inverse().apply(pose.apply(p)), p));
        assert!(close(pose.compose(&pose.inverse()).apply(p), p));
    }

    #[test]
    fn an_element_round_trips_through_its_anchor() {
        let mut table = KeyframeTable::new(params());
        let slot = table.add(&keyframe(7, Pose::from_yaw([5.0, 0.0, 0.0], 0.3)));
        // A point in the anchor's frame, placed now, then brought back.
        let local = [1.0, 2.0, 0.0];
        let world = table.now(slot).apply(local);
        assert!(close(table.now(slot).inverse().apply(world), local));
        // Before any closure version 0 and now agree.
        assert!(close(table.to_v0(slot).apply(world), world));
    }

    #[test]
    fn the_table_follows_closures_and_places_later_keyframes_by_the_correction() {
        let mut table = KeyframeTable::new(params());
        let a = table.add(&keyframe(10, Pose::from_yaw([0.0, 0.0, 0.0], 0.0)));
        let b = table.add(&keyframe(11, Pose::from_yaw([10.0, 0.0, 0.0], 0.0)));
        let shift = Pose::from_yaw([0.0, 1.0, 0.0], 0.0);
        let closure = table
            .apply_event(&[
                EventRecord {
                    node_id: 10,
                    pre: table.now(a),
                    delta: Pose::IDENTITY,
                    version: Some(1),
                    closed_with: Some(11),
                },
                EventRecord {
                    node_id: 11,
                    pre: table.now(b),
                    delta: shift,
                    version: Some(1),
                    closed_with: None,
                },
            ])
            .expect("known keyframes");
        assert_eq!(
            closure,
            Closure {
                version: 1,
                closing: vec![a, b]
            }
        );
        assert!(close(table.now(b).translation, [10.0, 1.0, 0.0]));
        assert!(close(table.now(a).translation, [0.0; 3]));
        // A keyframe made after the closure rides the newest keyframe's correction.
        let c = table.add(&keyframe(12, Pose::from_yaw([20.0, 0.0, 0.0], 0.0)));
        assert!(close(table.now(c).translation, [20.0, 1.0, 0.0]));
        // Version 0's frame stays the current layer's until a new one starts, then is kept.
        assert!(close(
            table.pose_at(1, b).unwrap().translation,
            [10.0, 0.0, 0.0]
        ));
        assert!(table.shift_since_frame() >= 1.0);
        table.start_frame();
        assert!(close(
            table.pose_at(0, b).unwrap().translation,
            [10.0, 0.0, 0.0]
        ));
        assert!(close(
            table.pose_at(1, b).unwrap().translation,
            [10.0, 1.0, 0.0]
        ));
        assert!(table.shift_since_frame() < 1e-9);
        assert!(close(
            table.to_v0(b).apply([10.0, 1.0, 0.0]),
            [10.0, 0.0, 0.0]
        ));
    }

    #[test]
    fn a_box_far_from_its_keyframe_is_a_candidate_there() {
        let mut table = KeyframeTable::new(params());
        let mut record = keyframe(1, Pose::IDENTITY);
        record.scan_box = [-1.0, 14.0, -1.0, 4.0, 18.0, 1.0];
        let slot = table.add(&record);
        assert_eq!(table.candidates([0.0, 2.0, 15.0, 16.0]), vec![slot]);
        assert!(table.candidates([0.0, 2.0, -5.0, -4.0]).is_empty());
    }

    #[test]
    fn a_carried_level_plane_stays_level_at_the_same_height() {
        let plane = SurfacePlane::from_parts(0.0, 0.0, 0.02);
        let carry = Pose::from_yaw([0.8, 0.0, 0.0], 0.4);
        let out = carry_plane(plane, (0, 0, 0), (10, 4, 0), &carry, 0.1, 0.5).unwrap();
        let (gx, gy, z0) = out.parts();
        assert!(
            gx.abs() < 1e-5 && gy.abs() < 1e-5 && (z0 - 0.02).abs() < 1e-5,
            "{out:?}"
        );
    }
}
