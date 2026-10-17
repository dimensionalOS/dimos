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

//! Port of the motion2 target planner spec (planners/target.py + se2_search):
//! obstacle xy -> 2D distance field -> SE(2) lattice search -> shortcut
//! smoothing -> densified (x, y, yaw) path. Deterministic by construction.
//! The intake is planar; which returns are obstacles is `module/obstacles.rs`'s call.

use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::f64::consts::PI;

/// Fine distance-field pitch: half the map's voxel.
const FINE: f64 = 0.04; // VOXEL / 2
const PAD: f64 = 1.5;
/// Lattice pitch: three fine samples.
const CELL: f64 = 0.12; // 3 * FINE
/// Lattice, fine field and voxel grid are commensurate at this pitch; working
/// areas snap to it in the world frame, so a far point never moves a sample.
const PERIOD: f64 = 0.24; // 2 * CELL == 3 * VOXEL == 6 * FINE
/// Free space kept around the working area, in whole periods.
const GRID_PAD: f64 = 0.72; // 3 * PERIOD
const YAW_BINS: usize = 16;
const OFFSET_STEP: f64 = 0.05;
/// Worst-case distance between the fine-grid snaps of two coincident points.
const SNAP: f64 = FINE * std::f64::consts::SQRT_2;
/// Side of the point-index bucket, in metres.
const BUCKET: f64 = 0.2;

/// Pitch at which a route is priced along its own arc. `se2.py::COST_STEP`.
const COST_STEP: f64 = FINE;

/// `se2.py::COMMIT_MARGIN`; tests only, python owns the number.
pub const COMMIT_MARGIN: f64 = 3.0;

/// The governor curve, read off the embodiment (`Emb::governor`).
#[derive(Clone, Copy, Debug, PartialEq)]
struct Governor {
    pub max_speed: f64,
    pub min_speed: f64,
    /// Room at which full speed is granted (m).
    pub speed_clearance: f64,
    /// The precision floor (m); below it clearance is fiction.
    pub floor: f64,
}

impl Governor {
    /// Clearance -> the speed the follower is held to.
    #[inline]
    fn speed(&self, clearance: f64) -> f64 {
        self.min_speed
            + (self.max_speed - self.min_speed)
                * ((clearance - self.floor) / (self.speed_clearance - self.floor)).clamp(0.0, 1.0)
    }

    /// What a metre at this clearance costs in open-space metres.
    #[inline]
    fn tight(&self, clearance: f64) -> f64 {
        self.max_speed / self.speed(clearance)
    }

    /// The multiplier at contact, `max_speed / min_speed`.
    #[inline]
    fn tight_max(&self) -> f64 {
        self.max_speed / self.min_speed
    }
}

/// `ControllerConfig`, field for field: a law's tuning, as searched on one body.
#[derive(Clone, Debug, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct Tuning {
    pub lookahead: f64,
    pub k_pos: f64,
    pub k_yaw: f64,
    pub fan_yaw_per_m: f64,
    pub fan_yaw_done: f64,
    pub speed_lookahead: f64,
}

/// `embodiment/base.py::Embodiment`, field for field.
#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct Emb {
    pub length: f64,
    pub width: f64,
    pub center_off: f64,
    pub comfort: f64,
    pub precision: f64,
    /// Governor curve: cruise at `speed_clearance` of room, creep at `precision`.
    pub max_speed: f64,
    pub min_speed: f64,
    pub speed_clearance: f64,
    pub max_yaw_rate: f64,
    /// Gait plant, measured; read by the follower, not the planner.
    pub command_slew: [f64; 3],
    pub gait_band: [f64; 2],
    pub walk_gain: f64,
    pub walk_slip: f64,
    pub walk_slip_ramp: f64,
    /// Vertical geometry off the surface the feet stand on (`obstacles.rs`).
    pub steppable: f64,
    pub height: f64,
    pub base_height: f64,
    /// Follower tuning (`ControllerConfig`); the planner never reads it.
    pub control: Tuning,
    pub strafe: f64,
    pub reverse: f64,
    pub yaw_w: f64,
    /// One row per |drift| deg: `(deg, length, width, off_x, off_y)`, `off_y`
    /// mirrored by drift sign. Empty = the union at every heading.
    pub envelope: Vec<[f64; 5]>,
    /// Extra swept WIDTH per rad-per-metre of curvature (edge dyaw / length).
    pub arc_inflate: f64,
}

impl Emb {
    /// A test body; not any robot. A deployed module is configured with its own.
    #[cfg(any(test, feature = "test-bodies"))]
    pub fn fixture() -> Self {
        Emb {
            length: 0.883,
            width: 0.593,
            center_off: 0.002,
            comfort: 0.4,
            precision: 0.05,
            max_speed: 0.5,
            min_speed: 0.2,
            speed_clearance: 0.35,
            max_yaw_rate: 1.4,
            command_slew: [2.5, 2.0, 5.0],
            gait_band: [0.45, 0.95],
            walk_gain: 0.964,
            walk_slip: 0.132,
            walk_slip_ramp: 0.08,
            steppable: 0.2,
            height: 0.45,
            base_height: 0.29,
            control: Tuning {
                lookahead: 0.35,
                k_pos: 2.0,
                k_yaw: 2.0,
                fan_yaw_per_m: 3.0,
                fan_yaw_done: 0.25,
                speed_lookahead: 2.0,
            },
            strafe: 1.8,
            reverse: 1.5,
            yaw_w: 0.25,
            envelope: vec![
                [0.0, 0.819, 0.416, -0.023, 0.0],
                [26.6, 0.802, 0.436, -0.032, -0.008],
                [45.0, 0.788, 0.472, -0.035, -0.018],
                [63.4, 0.781, 0.5, -0.039, -0.016],
                [90.0, 0.781, 0.507, -0.039, -0.009],
                [116.6, 0.781, 0.497, -0.039, 0.0],
                [135.0, 0.781, 0.463, -0.039, -0.001],
                [153.4, 0.781, 0.422, -0.039, -0.003],
                [180.0, 0.781, 0.416, -0.039, 0.0],
            ],
            arc_inflate: 0.0334,
        }
    }

    /// The pricing curve, read off the body.
    fn governor(&self) -> Governor {
        Governor {
            max_speed: self.max_speed,
            min_speed: self.min_speed,
            speed_clearance: self.speed_clearance,
            floor: self.precision,
        }
    }

    /// The all-gait union; the fallback for an unmeasured embodiment.
    fn union_box(&self) -> [f64; 4] {
        [self.length, self.width, self.center_off, 0.0]
    }

    /// The standing body: the largest box nested in every envelope row.
    /// `embodiment/base.py::stand_box`.
    fn stand_box(&self) -> [f64; 4] {
        if self.envelope.is_empty() {
            return self.union_box();
        }
        let mut lo = f64::NEG_INFINITY;
        let mut hi = f64::INFINITY;
        let mut half_w = f64::INFINITY;
        for r in &self.envelope {
            lo = lo.max(r[3] - r[1] / 2.0);
            hi = hi.min(r[3] + r[1] / 2.0);
            // Mirroring folds a row's y interval onto |off_y| .. w/2 - |off_y|.
            half_w = half_w.min(r[2] / 2.0 - r[4].abs());
        }
        [hi - lo, 2.0 * half_w, (lo + hi) / 2.0, 0.0]
    }

    /// `(length, width, off_x, off_y)` for a body-frame drift angle in rad.
    fn envelope_at(&self, drift: f64) -> [f64; 4] {
        if self.envelope.is_empty() {
            return self.union_box();
        }
        let rel = rem_2pi(drift);
        let deg = rel.abs().to_degrees();
        // `min_by` keeps the FIRST of equal minima, as python's `min(key=)` does.
        let row = self
            .envelope
            .iter()
            .min_by(|a, b| (a[0] - deg).abs().total_cmp(&(b[0] - deg).abs()))
            .expect("non-empty");
        [
            row[1],
            row[2],
            row[3],
            if rel >= 0.0 { row[4] } else { -row[4] },
        ]
    }
}

fn arange(start: f64, stop: f64, step: f64) -> Vec<f64> {
    (0..arange_len(start, stop, step))
        .map(|k| start + k as f64 * step)
        .collect()
}

fn arange_len(start: f64, stop: f64, step: f64) -> usize {
    let n = ((stop - start) / step).ceil();
    if n > 0.0 {
        n as usize
    } else {
        0
    }
}

/// Footprint sample points of one swept box `(length, width, off_x, off_y)`.
fn offsets(b: &[f64; 4]) -> Vec<(f64, f64)> {
    let (hl, hw) = (b[0] / 2.0, b[1] / 2.0);
    let xs = arange(-hl, hl + OFFSET_STEP / 2.0, OFFSET_STEP);
    let ys = arange(-hw, hw + OFFSET_STEP / 2.0, OFFSET_STEP);
    // Coarse-to-fine order: the clearance scan stops at the first blocked sample.
    let (nxs, nys) = (xs.len(), ys.len());
    let mut seen = vec![false; nxs * nys];
    let mut out = Vec::with_capacity(nxs * nys);
    let mut step = 1;
    while step * 2 < nxs.max(nys) {
        step *= 2;
    }
    loop {
        let mut xi = 0;
        while xi < nxs {
            let mut yi = 0;
            while yi < nys {
                if !seen[xi * nys + yi] {
                    seen[xi * nys + yi] = true;
                    out.push((xs[xi] + b[2], ys[yi] + b[3]));
                }
                yi += step;
            }
            // The far edge of each axis is a footprint corner: take it early.
            if !seen[xi * nys + nys - 1] {
                seen[xi * nys + nys - 1] = true;
                out.push((xs[xi] + b[2], ys[nys - 1] + b[3]));
            }
            xi += step;
        }
        if step == 1 {
            break;
        }
        step /= 2;
    }
    out
}

/// Swept boxes interned by geometry: id 0 the union, then the envelope rows
/// in both drift signs.
struct Fps {
    keys: Vec<[i64; 4]>,
    offs: Vec<Vec<(f64, f64)>>,
    /// The static body the seed is witnessed on; id 0 for an unmeasured embodiment.
    stand: usize,
}

/// Box identity, rounded exactly as the python reference rounds it.
fn fp_key(b: &[f64; 4]) -> [i64; 4] {
    let q = |v: f64| (v * 1e9).round_even_i64();
    [q(b[0]), q(b[1]), q(b[2]), q(b[3])]
}

impl Fps {
    fn new(emb: &Emb) -> Self {
        let mut f = Fps {
            keys: Vec::new(),
            offs: Vec::new(),
            stand: 0,
        };
        f.intern(&emb.union_box()); // id 0: the veto shape and every fallback
        f.stand = f.intern(&emb.stand_box());
        for row in &emb.envelope {
            for s in [1.0f64, -1.0] {
                f.intern(&[row[1], row[2], row[3], s * row[4]]);
            }
        }
        f
    }

    fn intern(&mut self, b: &[f64; 4]) -> usize {
        let k = fp_key(b);
        match self.keys.iter().position(|x| *x == k) {
            Some(i) => i,
            None => {
                self.keys.push(k);
                self.offs.push(offsets(b));
                self.keys.len() - 1
            }
        }
    }

    /// Box for this body-frame drift angle; `None` = the union.
    fn id(&self, emb: &Emb, drift: Option<f64>) -> usize {
        let b = match drift {
            None => emb.union_box(),
            Some(d) => emb.envelope_at(d),
        };
        let k = fp_key(&b);
        self.keys
            .iter()
            .position(|x| *x == k)
            .expect("every reachable box is interned by Fps::new")
    }

    fn union(&self) -> &[(f64, f64)] {
        &self.offs[0]
    }

    /// The standing body's samples, and its box id for the lattice caches.
    fn stand(&self) -> &[(f64, f64)] {
        &self.offs[self.stand]
    }
}

/// `x.round_ties_even() as i64` via the 1.5 * 2^52 trick, no libm call. Exact
/// for |x| < 2^51; every caller clamps into the grid anyway.
trait RoundEvenI64 {
    fn round_even_i64(self) -> i64;
}

impl RoundEvenI64 for f64 {
    #[inline(always)]
    fn round_even_i64(self) -> i64 {
        const MAGIC: f64 = 6_755_399_441_055_744.0; // 1.5 * 2^52
        (self + MAGIC).to_bits() as i64 - MAGIC.to_bits() as i64
    }
}

fn rem_2pi(x: f64) -> f64 {
    x - (x / (2.0 * PI)).round() * (2.0 * PI)
}

/// Radius of the footprint sample set about the body origin.
fn reach_of(offs: &[(f64, f64)]) -> f64 {
    offs.iter().fold(0.0f64, |m, &(ox, oy)| m.max(ox.hypot(oy)))
}

/// Uniform-bucket point index for nearest-neighbour queries, flat CSR.
struct PointBuckets {
    b: f64,
    x0: f64,
    y0: f64,
    nx: i64,
    ny: i64,
    off: Vec<u32>,
    px: Vec<f64>,
    py: Vec<f64>,
}

impl PointBuckets {
    fn new(pts: &[(f64, f64)]) -> Self {
        let b = BUCKET;
        let (mut x0, mut y0) = (f64::INFINITY, f64::INFINITY);
        let (mut x1, mut y1) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
        for &(x, y) in pts {
            x0 = x0.min(x);
            y0 = y0.min(y);
            x1 = x1.max(x);
            y1 = y1.max(y);
        }
        let nx = (((x1 - x0) / b).floor() as i64 + 1).max(1);
        let ny = (((y1 - y0) / b).floor() as i64 + 1).max(1);
        let n = (nx * ny) as usize;
        let cell_of = |x: f64, y: f64| -> usize {
            let i = (((x - x0) / b).floor() as i64).clamp(0, nx - 1);
            let j = (((y - y0) / b).floor() as i64).clamp(0, ny - 1);
            (i * ny + j) as usize
        };
        let mut off = vec![0u32; n + 1];
        for &(x, y) in pts {
            off[cell_of(x, y) + 1] += 1;
        }
        for k in 0..n {
            off[k + 1] += off[k];
        }
        let mut fill = off.clone();
        let mut buf = vec![(0.0f64, 0.0f64); pts.len()];
        for &(x, y) in pts {
            let c = cell_of(x, y);
            buf[fill[c] as usize] = (x, y);
            fill[c] += 1;
        }
        // Collapse coincident points per bucket; `nearest` takes a min, so
        // repeats cannot change it.
        let mut off2 = vec![0u32; n + 1];
        let mut fx = Vec::with_capacity(pts.len());
        let mut fy = Vec::with_capacity(pts.len());
        for c in 0..n {
            let (a, z) = (off[c] as usize, off[c + 1] as usize);
            let bucket = &mut buf[a..z];
            bucket.sort_unstable_by(|p, q| p.0.total_cmp(&q.0).then(p.1.total_cmp(&q.1)));
            let mut last = (f64::NAN, f64::NAN);
            for &(x, y) in bucket.iter() {
                if x != last.0 || y != last.1 {
                    fx.push(x);
                    fy.push(y);
                    last = (x, y);
                }
            }
            off2[c + 1] = fx.len() as u32;
        }
        PointBuckets {
            b,
            x0,
            y0,
            nx,
            ny,
            off: off2,
            px: fx,
            py: fy,
        }
    }

    #[inline]
    /// Smallest squared distance over buckets `jlo..=jhi` of column `i`.
    fn row(&self, i: i64, jlo: i64, jhi: i64, best: &mut f64, qx: f64, qy: f64) {
        if i < 0 || i >= self.nx {
            return;
        }
        let a = self.off[(i * self.ny + jlo) as usize] as usize;
        let z = self.off[(i * self.ny + jhi) as usize + 1] as usize;
        let mut m = *best;
        for (&x, &y) in self.px[a..z].iter().zip(&self.py[a..z]) {
            let (dx, dy) = (x - qx, y - qy);
            let d = dx * dx + dy * dy;
            m = if d < m { d } else { m };
        }
        *best = m;
    }

    /// Nearest-point distance, exact below `cap`.
    fn nearest(&self, qx: f64, qy: f64, cap: f64) -> f64 {
        let qi = ((qx - self.x0) / self.b).floor() as i64;
        let qj = ((qy - self.y0) / self.b).floor() as i64;
        let max_ring = qi
            .abs()
            .max((self.nx - 1 - qi).abs())
            .max(qj.abs())
            .max((self.ny - 1 - qj).abs());
        // Squared throughout, so the ring stop-conditions square too.
        let mut best = f64::INFINITY;
        let cap2 = cap * cap;
        for r in 0..=max_ring {
            let edge = ((r - 1).max(0) as f64) * self.b;
            let floor = edge * edge;
            if best <= floor || cap2 <= floor {
                break;
            }
            let jlo = (qj - r).max(0);
            let jhi = (qj + r).min(self.ny - 1);
            if jlo > jhi {
                continue;
            }
            if r == 0 {
                self.row(qi, jlo, jhi, &mut best, qx, qy);
                continue;
            }
            self.row(qi - r, jlo, jhi, &mut best, qx, qy);
            self.row(qi + r, jlo, jhi, &mut best, qx, qy);
            let (jm, jp) = (qj - r, qj + r);
            for i in (qi - r + 1)..(qi + r) {
                if i < 0 || i >= self.nx {
                    continue;
                }
                if jm >= 0 && jm < self.ny {
                    self.row(i, jm, jm, &mut best, qx, qy);
                }
                if jp >= 0 && jp < self.ny {
                    self.row(i, jp, jp, &mut best, qx, qy);
                }
            }
        }
        best.sqrt()
    }
}

struct World {
    /// Absolute fine-grid index of the field's origin; positions are `fkx * FINE`.
    fkx: i64,
    fky: i64,
    nfx: usize,
    nfy: usize,
    sdf: Vec<f64>,
    pts: Option<PointBuckets>,
    cap: f64,
    pub bounds: (f64, f64, f64, f64),
    /// Absolute LATTICE index of the working area's low corner, the same way.
    pub kx: i64,
    pub ky: i64,
    /// Lattice dims and price multiplier per cell: 1.0 near a ground return,
    /// `unseen_cost` elsewhere; empty = all explored.
    nlx: usize,
    nly: usize,
    unseen: Vec<f64>,
}

impl World {
    /// The unexplored-terrain multiplier at lattice cell (i, j).
    #[inline]
    fn mult(&self, i: usize, j: usize) -> f64 {
        if self.unseen.is_empty() {
            1.0
        } else {
            self.unseen[i * self.nly + j]
        }
    }

    /// `mult` at a world position, snapped the way the lattice is.
    fn mult_at(&self, x: f64, y: f64) -> f64 {
        if self.unseen.is_empty() {
            return 1.0;
        }
        let i = ((x / CELL).round_even_i64() - self.kx).clamp(0, self.nlx as i64 - 1) as usize;
        let j = ((y / CELL).round_even_i64() - self.ky).clamp(0, self.nly as i64 - 1) as usize;
        self.unseen[i * self.nly + j]
    }

    /// `at` for a caller that holds only the flat index (the miss path).
    #[inline]
    fn at_k(&mut self, k: usize) -> f64 {
        let (i, j) = (k / self.nfy, k % self.nfy);
        self.at(i, j, k)
    }

    #[inline]
    fn at(&mut self, i: usize, j: usize, k: usize) -> f64 {
        let v = self.sdf[k];
        if v >= 0.0 {
            return v;
        }
        let d = match &self.pts {
            Some(b) => b.nearest(
                (self.fkx + i as i64) as f64 * FINE,
                (self.fky + j as i64) as f64 * FINE,
                self.cap,
            ),
            None => f64::INFINITY,
        };
        self.sdf[k] = d;
        d
    }

    /// x-only half of the flat fine-field index; `ypart` is the other.
    #[inline]
    fn xpart(&self, i: usize) -> usize {
        i * self.nfy
    }

    #[inline]
    fn ypart(j: usize) -> usize {
        j
    }

    /// Fine-field value at a world position, snapped on the absolute grid.
    #[inline]
    fn lookup(&mut self, px: f64, py: f64) -> f64 {
        let i = ((px / FINE).round_even_i64() - self.fkx).clamp(0, self.nfx as i64 - 1) as usize;
        let j = ((py / FINE).round_even_i64() - self.fky).clamp(0, self.nfy as i64 - 1) as usize;
        let k = self.xpart(i) + Self::ypart(j);
        self.at(i, j, k)
    }
}

#[cfg(test)]
fn build_world(points: &[[f64; 2]], pose: (f64, f64, f64), goal: (f64, f64), cap: f64) -> World {
    build_world_explored(points, &[], 1.0, pose, goal, cap)
}

/// `build_world` pricing lattice cells with no `ground` return under them or
/// their 8 neighbours at `unseen_cost`; `<= 1.0` turns the layer off.
fn build_world_explored(
    points: &[[f64; 2]],
    ground: &[[f64; 2]],
    unseen_cost: f64,
    pose: (f64, f64, f64),
    goal: (f64, f64),
    cap: f64,
) -> World {
    let band: Vec<(f64, f64)> = points.iter().map(|p| (p[0], p[1])).collect();
    // Working area over {pose, goal, cloud} padded by `PAD`, low corner snapped
    // onto the absolute lattice.
    let mut x0 = goal.0.min(pose.0);
    let mut y0 = goal.1.min(pose.1);
    let mut x1 = goal.0.max(pose.0);
    let mut y1 = goal.1.max(pose.1);
    for &(x, y) in &band {
        x0 = x0.min(x);
        y0 = y0.min(y);
        x1 = x1.max(x);
        y1 = y1.max(y);
    }
    let (px, py) = (
        ((x0 - PAD) / PERIOD).floor() as i64,
        ((y0 - PAD) / PERIOD).floor() as i64,
    );
    let (x0, y0) = (px as f64 * PERIOD, py as f64 * PERIOD);
    let (x1, y1) = (x1 + PAD, y1 + PAD);
    let (fkx, fky) = ((px - 3) * 6, (py - 3) * 6); // GRID_PAD = 3 PERIODs = 18 fine
    let (fx0, fy0) = (fkx as f64 * FINE, fky as f64 * FINE);
    let nfx = arange_len(fx0, x1 + GRID_PAD, FINE);
    let nfy = arange_len(fy0, y1 + GRID_PAD, FINE);
    let ncells = nfx * nfy;
    let (sdf, pts) = if band.is_empty() {
        (vec![f64::INFINITY; ncells], None)
    } else {
        (vec![-1.0; ncells], Some(PointBuckets::new(&band)))
    };
    let (kx, ky) = (px * 2, py * 2);
    let nlx = arange_len(x0, x1 + CELL, CELL);
    let nly = arange_len(y0, y1 + CELL, CELL);
    let unseen = if unseen_cost <= 1.0 {
        Vec::new()
    } else {
        let mut u = vec![unseen_cost; nlx * nly];
        // ponytail: one-cell dilation so a voxel-sparse floor does not flicker
        // holes into the explored set; a proper coverage estimate if it does.
        for g in ground {
            let i = (g[0] / CELL).round_even_i64() - kx;
            let j = (g[1] / CELL).round_even_i64() - ky;
            for di in -1..=1 {
                for dj in -1..=1 {
                    let (a, b) = (i + di, j + dj);
                    if a >= 0 && b >= 0 && a < nlx as i64 && b < nly as i64 {
                        u[a as usize * nly + b as usize] = 1.0;
                    }
                }
            }
        }
        u
    };
    World {
        fkx,
        fky,
        nfx,
        nfy,
        sdf,
        pts,
        cap,
        bounds: (x0, y0, x1, y1),
        kx,
        ky,
        nlx,
        nly,
        unseen,
    }
}

fn gcd(a: i64, b: i64) -> i64 {
    if b == 0 {
        a
    } else {
        gcd(b, a % b)
    }
}

struct Move {
    di: i64,
    dj: i64,
    /// `di * ny + dj`: what this move adds to a flat state index in any yaw plane.
    dk: i64,
    base: f64,
    mids: Vec<(i64, i64)>,
}

/// Min-heap node; `f` is the cost's bit pattern, which orders like `total_cmp`
/// over non-negative doubles.
#[derive(PartialEq, Eq)]
struct Node {
    f: u64,
    k: u32,
}

impl Ord for Node {
    fn cmp(&self, o: &Self) -> Ordering {
        o.f.cmp(&self.f).then(o.k.cmp(&self.k))
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

/// Clearance per (yaw bin, cell, swept box); the union is the hot path, rows
/// are cached lazily.
struct Clear<'a> {
    w: &'a mut World,
    /// Union clearance per (bin, cell): 0.0 = unevaluated, -1.0 = does not fit,
    /// else the minimum clearance (`speed_clearance` when certified without a scan).
    t: Vec<f64>,
    nx: usize,
    ny: usize,
    gx: Vec<f64>,
    gy: Vec<f64>,
    rot: Vec<(f64, f64)>,
    noff: usize,
    /// `(sin, cos)` per yaw bin, for the row scans.
    cs: Vec<(f64, f64)>,
    /// Footprint samples per interned box; id 0 is the union.
    fp_offs: Vec<Vec<(f64, f64)>>,
    nfp: usize,
    /// Row clearance per (bin, box), same encoding as `t`, allocated on first use.
    rowc: Vec<Vec<f64>>,
    /// Fine-field index halves per (bin, lattice line, sample); `dx` / `dy`
    /// mark filled lines.
    ix: Vec<u32>,
    iy: Vec<u32>,
    dx: Vec<bool>,
    dy: Vec<bool>,
    margin: f64,
    certify: f64,
    /// The standing body's box id; `free`'s shape only.
    stand: usize,
    gov: Governor,
}

/// The lattice's yaw bins, in radians.
fn yaw_bins() -> Vec<f64> {
    (0..YAW_BINS)
        .map(|k| -PI + k as f64 * (2.0 * PI / YAW_BINS as f64))
        .collect()
}

impl<'a> Clear<'a> {
    fn new(
        w: &'a mut World,
        fps: &Fps,
        margin: f64,
        gx: Vec<f64>,
        gy: Vec<f64>,
        gov: Governor,
    ) -> Self {
        let (nx, ny) = (gx.len(), gy.len());
        let noff = fps.union().len();
        let mut rot = Vec::with_capacity(YAW_BINS * noff);
        let mut cs = Vec::with_capacity(YAW_BINS);
        let mut reach: f64 = 0.0;
        for th in yaw_bins() {
            let (s, c) = th.sin_cos();
            cs.push((s, c));
            for &(ox, oy) in fps.union() {
                let (rx, ry) = (c * ox - s * oy, s * ox + c * oy);
                reach = reach.max(rx.hypot(ry));
                rot.push((rx, ry));
            }
        }
        let nfp = fps.offs.len();
        Clear {
            w,
            t: vec![0.0f64; YAW_BINS * nx * ny],
            nx,
            ny,
            gx,
            gy,
            rot,
            noff,
            cs,
            fp_offs: fps.offs.clone(),
            nfp,
            rowc: vec![Vec::new(); YAW_BINS * nfp],
            ix: vec![0u32; YAW_BINS * nx * noff],
            iy: vec![0u32; YAW_BINS * ny * noff],
            dx: vec![false; YAW_BINS * nx],
            dy: vec![false; YAW_BINS * ny],
            margin,
            // Above `speed_clearance` a cell is not worth scanning.
            certify: gov.speed_clearance + reach + SNAP,
            stand: fps.stand,
            gov,
        }
    }

    /// Fine-grid x indices of this yaw bin's footprint at lattice column `i`.
    fn fill_x(&mut self, b: usize, i: usize, rx: usize) {
        let x = self.gx[i];
        let (base, o) = (b * self.noff, rx * self.noff);
        let (fkx, hi) = (self.w.fkx, self.w.nfx as i64 - 1);
        let nfy = self.w.nfy;
        for (d, r) in self.ix[o..o + self.noff]
            .iter_mut()
            .zip(&self.rot[base..base + self.noff])
        {
            let fi = (((x + r.0) / FINE).round_even_i64() - fkx).clamp(0, hi) as usize;
            *d = (fi * nfy) as u32;
        }
        self.dx[rx] = true;
    }

    /// Fine-grid y indices of this yaw bin's footprint at lattice row `j`.
    fn fill_y(&mut self, b: usize, j: usize, ry: usize) {
        let y = self.gy[j];
        let (base, o) = (b * self.noff, ry * self.noff);
        let (fky, hi) = (self.w.fky, self.w.nfy as i64 - 1);
        for (d, r) in self.iy[o..o + self.noff]
            .iter_mut()
            .zip(&self.rot[base..base + self.noff])
        {
            let fj = (((y + r.1) / FINE).round_even_i64() - fky).clamp(0, hi) as usize;
            *d = World::ypart(fj) as u32;
        }
        self.dy[ry] = true;
    }

    fn eval(&mut self, k: usize, b: usize, i: usize, j: usize) -> f64 {
        if self.w.lookup(self.gx[i], self.gy[j]) >= self.certify {
            self.t[k] = self.gov.speed_clearance;
            return self.gov.speed_clearance;
        }
        let rx = b * self.nx + i;
        if !self.dx[rx] {
            self.fill_x(b, i, rx);
        }
        let ry = b * self.ny + j;
        if !self.dy[ry] {
            self.fill_y(b, j, ry);
        }
        let (ox, oy) = (rx * self.noff, ry * self.noff);
        let n = self.noff;
        // Slices, so the bounds checks collapse and a miss can write through `w`.
        let Clear {
            w, ix, iy, margin, ..
        } = self;
        let (ixs, iys, margin) = (&ix[ox..ox + n], &iy[oy..oy + n], *margin);
        let mut m = f64::INFINITY;
        for (&a, &c) in ixs.iter().zip(iys.iter()) {
            let kk = a as usize + c as usize;
            let mut d = w.sdf[kk];
            if d < 0.0 {
                d = w.at_k(kk);
            }
            if d < m {
                m = d;
                if m <= margin {
                    self.t[k] = -1.0;
                    return -1.0;
                }
            }
        }
        self.t[k] = m;
        m
    }

    #[inline]
    fn clear(&mut self, b: usize, i: usize, j: usize) -> f64 {
        self.clear_at((b * self.nx + i) * self.ny + j, b, i, j)
    }

    /// `clear` for a caller that already holds the flat index.
    #[inline]
    fn clear_at(&mut self, k: usize, b: usize, i: usize, j: usize) -> f64 {
        let v = self.t[k];
        if v != 0.0 {
            return v;
        }
        self.eval(k, b, i, j)
    }

    /// Time price of one metre entering this cell, on the union clearance; a
    /// blocked cell prices at the floor.
    #[inline]
    fn price(&self, v: f64) -> f64 {
        if v < 0.0 {
            self.gov.tight_max()
        } else {
            self.gov.tight(v)
        }
    }

    /// Minimum clearance of swept box `fp` at (bin, cell), same encoding as `t`.
    fn row_clear(&mut self, b: usize, fp: usize, i: usize, j: usize) -> f64 {
        if fp == 0 {
            return self.clear(b, i, j);
        }
        let p = b * self.nfp + fp;
        if self.rowc[p].is_empty() {
            self.rowc[p] = vec![0.0; self.nx * self.ny];
        }
        let k = i * self.ny + j;
        let cached = self.rowc[p][k];
        if cached != 0.0 {
            return cached;
        }
        let (s, c) = self.cs[b];
        let (x, y) = (self.gx[i], self.gy[j]);
        let margin = self.margin;
        let Clear { w, fp_offs, .. } = self;
        let mut m = f64::INFINITY;
        for &(ox, oy) in &fp_offs[fp] {
            let d = w.lookup(x + c * ox - s * oy, y + s * ox + c * oy);
            if d < m {
                m = d;
                if m <= margin {
                    m = -1.0;
                    break;
                }
            }
        }
        self.rowc[p][k] = m;
        m
    }

    /// Does swept box `fp` clear `thresh` at (bin, cell)? Union first; every
    /// row is nested in it.
    #[inline]
    fn fits(&mut self, b: usize, fp: usize, i: usize, j: usize, thresh: f64) -> bool {
        if self.clear(b, i, j) > thresh {
            return true;
        }
        fp != 0 && self.row_clear(b, fp, i, j) > thresh
    }

    /// Can the robot stand here? Read on the standing box, so a cell a drift
    /// row threaded is not a wall on replan.
    #[inline]
    fn free(&mut self, b: usize, i: usize, j: usize) -> bool {
        self.fits(b, self.stand, i, j, self.margin)
    }
}

fn pose_clear(w: &mut World, offs: &[(f64, f64)], x: f64, y: f64, th: f64) -> f64 {
    let (s, c) = th.sin_cos();
    let mut m = f64::INFINITY;
    for &(ox, oy) in offs {
        let d = w.lookup(x + c * ox - s * oy, y + s * ox + c * oy);
        if d < m {
            m = d;
        }
    }
    m
}

/// Metres of `a -> b` over unexplored cells; zero when the layer is off.
fn dark_len(w: &World, a: &[f64; 3], b: &[f64; 3]) -> f64 {
    if w.unseen.is_empty() {
        return 0.0;
    }
    let len = (b[0] - a[0]).hypot(b[1] - a[1]);
    let n = ((len / (0.5 * CELL)).ceil() as usize).max(1);
    let h = len / n as f64;
    (0..n)
        .filter(|&q| {
            let t = (q as f64 + 0.5) / n as f64;
            w.mult_at(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])) > 1.0
        })
        .count() as f64
        * h
}

/// Is the straight SE(2) interpolation `a -> b` clear of `floor`, on its own
/// drift row widened by its curvature?
fn seg_free(w: &mut World, fps: &Fps, emb: &Emb, a: &[f64; 3], b: &[f64; 3], floor: f64) -> bool {
    let dyaw = rem_2pi(b[2] - a[2]);
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let span = dx.hypot(dy);
    let head = if span > 1e-9 {
        Some(dy.atan2(dx))
    } else {
        None
    };
    let pad = if span > 1e-9 {
        0.5 * emb.arc_inflate * dyaw.abs() / span
    } else {
        0.0
    };
    let steps = 2usize
        .max((span / 0.06) as usize)
        .max((dyaw.abs() / 0.15) as usize);
    for k in 0..=steps {
        let t = k as f64 / steps as f64;
        let th = a[2] + t * dyaw;
        let offs = &fps.offs[fps.id(emb, head.map(|h| h - th))];
        if pose_clear(w, offs, a[0] + t * dx, a[1] + t * dy, th) <= floor + pad {
            return false;
        }
    }
    true
}

/// The lattice the search walks: cell centres from their absolute index.
fn lattice_axes(w: &World) -> (Vec<f64>, Vec<f64>) {
    let (x0, y0, x1, y1) = w.bounds;
    let (kx, ky) = (w.kx, w.ky);
    (
        (0..arange_len(x0, x1 + CELL, CELL))
            .map(|i| (kx + i as i64) as f64 * CELL)
            .collect(),
        (0..arange_len(y0, y1 + CELL, CELL))
            .map(|j| (ky + j as i64) as f64 * CELL)
            .collect(),
    )
}

/// `se2_search` on a clearance table the caller owns (a pure memo, so shareable).
fn se2_search_in(
    cl: &mut Clear,
    fps: &Fps,
    start: (f64, f64, f64),
    goal: (f64, f64),
    emb: &Emb,
    margin: f64,
) -> Option<Vec<[f64; 3]>> {
    let offs = fps.union().to_vec();
    let stand_offs = fps.stand().to_vec();
    let (kx, ky) = (cl.w.kx, cl.w.ky);
    let (nx, ny) = (cl.nx, cl.ny);
    let thetas = yaw_bins();

    let cell_of = |px: f64, py: f64| -> (usize, usize) {
        (
            ((px / CELL).round_even_i64() - kx).clamp(0, nx as i64 - 1) as usize,
            ((py / CELL).round_even_i64() - ky).clamp(0, ny as i64 - 1) as usize,
        )
    };
    let mut sb = 0;
    let mut sbest = f64::INFINITY;
    for (k, &th) in thetas.iter().enumerate() {
        let d = rem_2pi(th - start.2).abs();
        if d < sbest {
            sbest = d;
            sb = k;
        }
    }
    let (si, sj) = cell_of(start.0, start.1);
    let (gi, gj) = cell_of(goal.0, goal.1);
    // The seed is witnessed at the true start pose on the standing box (nested
    // in every row, so a replan from a published route cannot refuse); below
    // the witness, the nearest standing-fit lattice state with a clear segment.
    let fit_bin = |cl: &mut Clear, i: usize, j: usize| -> Option<usize> {
        for d in 0..=(YAW_BINS / 2) {
            for b in [(sb + d) % YAW_BINS, (sb + YAW_BINS - d) % YAW_BINS] {
                if cl.free(b, i, j) {
                    return Some(b);
                }
            }
        }
        None
    };
    let witness = pose_clear(cl.w, &stand_offs, start.0, start.1, start.2) > margin;
    let (sb, si, sj) = match if witness {
        Some(sb)
    } else {
        fit_bin(cl, si, sj)
    } {
        Some(b) => (b, si, sj),
        None => {
            let mut cands: Vec<(usize, usize)> = Vec::new();
            for di in -1i64..=1 {
                for dj in -1i64..=1 {
                    let (i, j) = (si as i64 + di, sj as i64 + dj);
                    if (di, dj) != (0, 0) && i >= 0 && j >= 0 && i < nx as i64 && j < ny as i64 {
                        cands.push((i as usize, j as usize));
                    }
                }
            }
            // Nearest first; the (i, j) tail keeps the order total.
            cands.sort_by(|&(ai, aj), &(bi, bj)| {
                let da = (cl.gx[ai] - start.0).hypot(cl.gy[aj] - start.1);
                let db = (cl.gx[bi] - start.0).hypot(cl.gy[bj] - start.1);
                da.total_cmp(&db).then((ai, aj).cmp(&(bi, bj)))
            });
            let here = [start.0, start.1, start.2];
            let mut pick = None;
            for (i, j) in cands {
                if let Some(b) = fit_bin(cl, i, j) {
                    let there = [cl.gx[i], cl.gy[j], thetas[b]];
                    if seg_free(cl.w, fps, emb, &here, &there, margin) {
                        pick = Some((b, i, j));
                        break;
                    }
                }
            }
            pick?
        }
    };

    let mut moves = Vec::new();
    for di in -2i64..=2 {
        for dj in -2i64..=2 {
            if (di, dj) == (0, 0) || gcd(di.abs(), dj.abs()) == 2 {
                continue;
            }
            let mids = if di.abs().max(dj.abs()) == 2 {
                vec![
                    (
                        (di as f64 / 2.0).floor() as i64,
                        (dj as f64 / 2.0).floor() as i64,
                    ),
                    (
                        (di as f64 / 2.0).ceil() as i64,
                        (dj as f64 / 2.0).ceil() as i64,
                    ),
                ]
            } else {
                Vec::new()
            };
            moves.push(Move {
                di,
                dj,
                dk: di * ny as i64 + dj,
                base: ((di * di + dj * dj) as f64).sqrt() * CELL,
                mids,
            });
        }
    }
    // Gait-real cost per (yaw bin, move).
    let nmv = moves.len();
    let mut mcost = vec![0.0f64; YAW_BINS * nmv];
    // Swept box per (yaw bin, move), by that edge's drift angle.
    let mut fp_move = vec![0usize; YAW_BINS * nmv];
    for (b, &th) in thetas.iter().enumerate() {
        for (mi, mv) in moves.iter().enumerate() {
            let head = (mv.dj as f64).atan2(mv.di as f64);
            let rel = head - th;
            let (f, l) = (rel.cos(), rel.sin());
            mcost[b * nmv + mi] = mv.base
                * (1.0
                    + (emb.strafe - 1.0) * l.abs()
                    + if f < 0.0 { emb.reverse - 1.0 } else { 0.0 });
            fp_move[b * nmv + mi] = fps.id(emb, Some(rel));
        }
    }
    let yaw_step = 2.0 * PI / YAW_BINS as f64;
    // Extra half-width one bin of turn costs a blend edge, per unit edge length.
    let arc_pad: Vec<f64> = moves
        .iter()
        .map(|mv| 0.5 * emb.arc_inflate * yaw_step / mv.base)
        .collect();
    let yaw_cost = emb.yaw_w * yaw_step;
    // A body thinner than a diagonal cell gets every edge segment-checked.
    let thinnest = emb
        .envelope
        .iter()
        .map(|r| r[1].min(r[2]))
        .fold(emb.length.min(emb.width), f64::min);
    let dense_moves = thinnest < CELL * std::f64::consts::SQRT_2;

    // Heuristic: exact shortest path to the goal through a relaxed free space
    // (a disc of radius `r_in` fits, priced by an upper bound on clearance),
    // so it lower-bounds every SE(2) edge. Unsettled cells take
    // max(straight line, `tfin`), still admissible and consistent.
    let inradius = |o: &[(f64, f64)]| -> f64 {
        let (mut ax0, mut ax1) = (f64::INFINITY, f64::NEG_INFINITY);
        let (mut ay0, mut ay1) = (f64::INFINITY, f64::NEG_INFINITY);
        for &(ox, oy) in o {
            ax0 = ax0.min(ox);
            ax1 = ax1.max(ox);
            ay0 = ay0.min(oy);
            ay1 = ay1.max(oy);
        }
        (-ax0).min(ax1).min(-ay0).min(ay1).max(0.0)
    };
    // The union's disc prices; the smallest box's disc decides membership.
    let r_price = inradius(&offs);
    let r_pass = fps.offs.iter().map(|o| inradius(o)).fold(r_price, f64::min);

    let ncell = nx * ny;
    let mut d2 = vec![f64::INFINITY; ncell];
    // 0 = open, 1 = settled. Also the lazy-deletion filter for `heap2`.
    let mut done2 = vec![0u8; ncell];
    // Price of a relaxed metre per cell: 0.0 = untested, -1.0 = blocked.
    let mut mul = vec![0.0f64; ncell];
    let mut heap2: BinaryHeap<Node> = BinaryHeap::new();
    let kg = gi * ny + gj;
    let ks = si * ny + sj;
    // Backwards from the goal, stopping once the start cell is settled; the
    // goal is seeded untested.
    d2[kg] = 0.0;
    mul[kg] = 1.0;
    // `-0.0` encodes as the largest u64, so each push asserts the sign.
    let f0 = 0.0f64; // bound so the assert reads a value, not two equal literals (clippy::eq_op)
    debug_assert!(f0 >= 0.0 && f0.is_sign_positive());
    heap2.push(Node {
        f: f0.to_bits(),
        k: kg as u32,
    });
    let mut tfin = f64::INFINITY;
    while let Some(Node { f, k }) = heap2.pop() {
        let f = f64::from_bits(f);
        let k = k as usize;
        if done2[k] != 0 {
            continue;
        }
        done2[k] = 1;
        if k == ks {
            tfin = f;
            break;
        }
        let (i, j) = (k / ny, k % ny);
        // The forward edge is `kk -> k`; the search charges the entered cell, `k`.
        let mk = mul[k];
        for mv in &moves {
            let (ni, nj) = (i as i64 + mv.di, j as i64 + mv.dj);
            if ni < 0 || nj < 0 || ni >= nx as i64 || nj >= ny as i64 {
                continue;
            }
            let (ni, nj) = (ni as usize, nj as usize);
            let kk = ni * ny + nj;
            if done2[kk] != 0 {
                continue;
            }
            let nd = f + mv.base * mk;
            if nd >= d2[kk] {
                continue;
            }
            if mul[kk] == 0.0 {
                let (px, py) = (cl.gx[ni], cl.gy[nj]);
                // Upper bound on the footprint's minimum clearance here in any yaw bin.
                let mtop = cl.w.lookup(px, py) + 1.5 * SNAP;
                mul[kk] = if mtop - r_pass > margin {
                    cl.gov.tight(mtop - r_price)
                } else {
                    -1.0
                };
            }
            if mul[kk] < 0.0 {
                continue;
            }
            d2[kk] = nd;
            debug_assert!(nd >= 0.0 && nd.is_sign_positive());
            heap2.push(Node {
                f: nd.to_bits(),
                k: kk as u32,
            });
        }
    }
    let (d2, done2, tfin) = (d2, done2, tfin);

    // Settled cells: exact; the rest: max(straight line, `tfin`).
    let heur = |i: usize, j: usize| -> f64 {
        let k = i * ny + j;
        if done2[k] != 0 {
            return d2[k];
        }
        let di = i as f64 - gi as f64;
        let dj = j as f64 - gj as f64;
        (CELL * (di * di + dj * dj).sqrt()).max(tfin)
    };

    let n_states = YAW_BINS * nx * ny;
    let mut dist = vec![f64::INFINITY; n_states];
    // `from + 1`, 0 = no predecessor, so the vector is served zeroed.
    let mut prev = vec![0u32; n_states];
    let mut closed = vec![false; n_states];
    let mut heap: BinaryHeap<Node> = BinaryHeap::new();
    let s0 = (sb * nx + si) * ny + sj;
    dist[s0] = 0.0;
    // Infinite `h` proves unreachability: drop, do not push.
    let mut goal_state: Option<(usize, usize, usize)> = None;
    if heur(si, sj) < f64::INFINITY {
        debug_assert!(heur(si, sj) >= 0.0 && heur(si, sj).is_sign_positive());
        heap.push(Node {
            f: heur(si, sj).to_bits(),
            k: s0 as u32,
        });
    }
    while let Some(Node { f: _, k: from }) = heap.pop() {
        let from = from as usize;
        let b = from / (nx * ny);
        let i = (from / ny) % nx;
        let j = from % ny;
        let d = dist[from];
        // Stale pops: the lowest `f` for a state pops first and is expanded.
        if closed[from] {
            continue;
        }
        closed[from] = true;
        if (i, j) == (gi, gj) {
            goal_state = Some((b, i, j));
            break;
        }
        let hij = heur(i, j);
        let from = from as u32;
        for pass in 0..3usize {
            let (nb, extra) = match pass {
                0 => (b, 0.0),
                1 => ((b + 1) % YAW_BINS, 0.5 * yaw_cost),
                _ => ((b + YAW_BINS - 1) % YAW_BINS, 0.5 * yaw_cost),
            };
            let kbase = ((nb * nx + i) * ny + j) as i64;
            if pass > 0 {
                // A turn in place has no drift row; the union is its shape.
                let k = kbase as usize;
                let uv = cl.clear_at(k, nb, i, j);
                if uv > 0.0 {
                    let yc = yaw_cost * cl.price(uv) * cl.w.mult(i, j);
                    if d + yc < dist[k] {
                        dist[k] = d + yc;
                        prev[k] = from + 1;
                        debug_assert!(d + yc + hij >= 0.0 && (d + yc + hij).is_sign_positive());
                        heap.push(Node {
                            f: (d + yc + hij).to_bits(),
                            k: k as u32,
                        });
                    }
                }
            }
            let crow = b * nmv;
            for (mi, mv) in moves.iter().enumerate() {
                let (ni, nj) = (i as i64 + mv.di, j as i64 + mv.dj);
                if ni < 0 || nj < 0 || ni >= nx as i64 || nj >= ny as i64 {
                    continue;
                }
                let (ni, nj) = (ni as usize, nj as usize);
                // `(nb * nx + ni) * ny + nj` by displacement, see `Move::dk`.
                let k = (kbase + mv.dk) as usize;
                // Multiplier >= 1.0: if the cheapest edge does not improve, skip the scan.
                let cmin = mcost[crow + mi] + extra;
                if d + cmin >= dist[k] {
                    continue;
                }
                // A cell the relaxation cannot route to the goal never needs pricing.
                let hn = heur(ni, nj);
                if hn == f64::INFINITY {
                    continue;
                }
                // Price on the union, feasibility on the drift row.
                let uv = cl.clear_at(k, nb, ni, nj);
                let c = cmin * cl.price(uv) * cl.w.mult(ni, nj);
                if d + c >= dist[k] {
                    continue;
                }
                let fp = fp_move[nb * nmv + mi];
                let thresh = if pass > 0 {
                    margin + arc_pad[mi]
                } else {
                    margin
                };
                if !(uv > thresh || cl.row_clear(nb, fp, ni, nj) > thresh) {
                    continue;
                }
                let mut blocked = false;
                for &(mdi, mdj) in &mv.mids {
                    if !cl.fits(
                        nb,
                        fp,
                        (i as i64 + mdi) as usize,
                        (j as i64 + mdj) as usize,
                        thresh,
                    ) {
                        blocked = true;
                        break;
                    }
                }
                if blocked {
                    continue;
                }
                if dense_moves
                    && !seg_free(
                        cl.w,
                        fps,
                        emb,
                        &[cl.gx[i], cl.gy[j], thetas[b]],
                        &[cl.gx[ni], cl.gy[nj], thetas[nb]],
                        margin,
                    )
                {
                    continue;
                }
                dist[k] = d + c;
                prev[k] = from + 1;
                debug_assert!(d + c + hn >= 0.0 && (d + c + hn).is_sign_positive());
                heap.push(Node {
                    f: (d + c + hn).to_bits(),
                    k: k as u32,
                });
            }
        }
    }
    let (mut b, mut i, mut j) = goal_state?;
    let mut states = vec![(b, i, j)];
    while prev[(b * nx + i) * ny + j] != 0 && (b, i, j) != (sb, si, sj) {
        let p = prev[(b * nx + i) * ny + j] as usize - 1;
        b = p / (nx * ny);
        i = (p / ny) % nx;
        j = p % ny;
        states.push((b, i, j));
    }
    states.reverse();
    let raw: Vec<[f64; 3]> = states
        .iter()
        .map(|&(b, i, j)| [cl.gx[i], cl.gy[j], thetas[b]])
        .collect();

    let w = &mut *cl.w;
    let raw_clear: Vec<f64> = raw
        .iter()
        .map(|s| pose_clear(w, &offs, s[0], s[1], s[2]))
        .collect();
    // A shortcut may not get closer to the world than the raw detour it
    // replaces (capped at `comfort`).
    let chord_floor = |raw_clear: &[f64], a: usize, b: usize| -> f64 {
        let minc = raw_clear[a..=b]
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        margin.max(minc.min(emb.comfort) - 0.02)
    };
    // Anchored at the goal so anchors are a function of the suffix and a
    // replan reproduces them; each anchor then retreats `RETREAT_NUM` of the
    // chord's arc length (whole raw edges, re-cleared) so it does not commit
    // straight past a corner. Arc, not vertices: rotations carry no distance.
    const RETREAT_NUM: f64 = 0.2;
    let mut arc = vec![0.0f64; raw.len()];
    // A chord may not add dark metres beyond the raw span it replaces.
    let mut dark = vec![0.0f64; raw.len()];
    for m in 1..raw.len() {
        arc[m] = arc[m - 1] + (raw[m][0] - raw[m - 1][0]).hypot(raw[m][1] - raw[m - 1][1]);
        dark[m] = dark[m - 1] + dark_len(w, &raw[m - 1], &raw[m]);
    }
    let chord_dark = |w: &World, j: usize, k: usize| -> bool {
        dark_len(w, &raw[j], &raw[k]) <= dark[k] - dark[j] + CELL
    };
    let mut keep = vec![raw.len() - 1];
    while *keep.last().unwrap() > 0 {
        let k = *keep.last().unwrap();
        let mut j = 0usize;
        while j + 1 < k {
            let floor = chord_floor(&raw_clear, j, k);
            if chord_dark(w, j, k) && seg_free(w, fps, emb, &raw[j], &raw[k], floor) {
                break;
            }
            j += 1;
        }
        // Last vertex within the fraction, capped at `k - 1`; a pure yaw edge
        // stops the scan.
        let target = arc[j] + (arc[k] - arc[j]) * RETREAT_NUM;
        let mut r = j;
        while r + 1 < k && arc[r + 1] > arc[r] && arc[r + 1] <= target {
            r += 1;
        }
        while r > j {
            let floor = chord_floor(&raw_clear, r, k);
            if chord_dark(w, r, k) && seg_free(w, fps, emb, &raw[r], &raw[k], floor) {
                break;
            }
            r -= 1;
        }
        keep.push(r);
    }
    keep.reverse();
    // Start repair: a pure-rotation first chord is lengthened forward, bounded
    // by the second anchor.
    if keep.len() > 2 && raw[keep[1]][0] == raw[0][0] && raw[keep[1]][1] == raw[0][1] {
        let (lo, hi) = (keep[1], keep[2]);
        let mut f = hi;
        while f > lo {
            let floor = chord_floor(&raw_clear, 0, f);
            if chord_dark(w, 0, f) && seg_free(w, fps, emb, &raw[0], &raw[f], floor) {
                break;
            }
            f -= 1;
        }
        if f == hi {
            keep.remove(1);
        } else {
            keep[1] = f;
        }
    }
    Some(keep.iter().map(|&k| raw[k]).collect())
}

/// Largest yaw change one published waypoint may command. `planners/base.py::YAW_STEP`.
const YAW_STEP: f64 = 0.045;

/// Largest yaw window a station may carry, under the 0.5 rad cylinder threshold.
const MAX_STATION_YAW: f64 = 0.45;

/// Arc length between stations.
const SCORE_STRIDE_M: f64 = 0.3;

/// Published waypoints per scoring station at this resolution.
fn station_stride(res: f64) -> f64 {
    (SCORE_STRIDE_M / res).round().max(1.0)
}

/// Extra clearance the coarse tier needs: the swept station's worst excursion
/// plus `SNAP` and the search's own 0.05 margin.
fn sweep_slack(reach: f64) -> f64 {
    2.0 * reach * (0.5 * MAX_STATION_YAW).sin() + SNAP + 0.05
}

/// Does every pose of this segment clear `slack`? Sampled at the denser tier.
fn chord_clears(
    w: &mut World,
    offs: &[(f64, f64)],
    a: [f64; 3],
    b: [f64; 3],
    dyaw: f64,
    n: usize,
    slack: f64,
) -> bool {
    for k in 0..=n {
        let t = k as f64 / n as f64;
        let c = pose_clear(
            w,
            offs,
            a[0] + t * (b[0] - a[0]),
            a[1] + t * (b[1] - a[1]),
            a[2] + t * dyaw,
        );
        if c < slack {
            return false;
        }
    }
    true
}

fn densify(
    w: &mut World,
    offs: &[(f64, f64)],
    states: &[[f64; 3]],
    res: f64,
    slack: f64,
) -> Vec<[f64; 3]> {
    let stride = station_stride(res);
    let coarse = MAX_STATION_YAW / stride;
    let nseg = states.len() - 1;
    let mut nc = Vec::with_capacity(nseg);
    let mut nf = Vec::with_capacity(nseg);
    let mut roomy = Vec::with_capacity(nseg);
    for s in states.windows(2) {
        let (a, b) = (s[0], s[1]);
        let dyaw = rem_2pi(b[2] - a[2]);
        let nd = 1usize.max(((b[0] - a[0]).hypot(b[1] - a[1]) / res) as usize);
        let c = nd.max((dyaw.abs() / coarse).ceil() as usize);
        let f = nd.max((dyaw.abs() / YAW_STEP).ceil() as usize);
        // Where the distance term dominates both tiers there is nothing to decide.
        roomy.push(c == f || chord_clears(w, offs, a, b, dyaw, f, slack));
        nc.push(c);
        nf.push(f);
    }

    let mut dense = vec![states[0]];
    for (m, s) in states.windows(2).enumerate() {
        let (a, b) = (s[0], s[1]);
        let dyaw = rem_2pi(b[2] - a[2]);
        // A station looks back `stride` steps, so the next `stride - 1`
        // segments must be roomy too.
        let span = m + stride as usize;
        let ok = roomy[m..span.min(nseg)].iter().all(|&r| r);
        let n = if ok { nc[m] } else { nf[m] };
        for k in 1..=n {
            let t = k as f64 / n as f64;
            dense.push([
                a[0] + t * (b[0] - a[0]),
                a[1] + t * (b[1] - a[1]),
                a[2] + t * dyaw,
            ]);
        }
    }
    dense
}

/// Route cost on the follower's clock, in open-space metres, sampled by arc so
/// densifying changes nothing. `se2.py::path_cost`.
fn path_cost(w: &mut World, offs: &[(f64, f64)], emb: &Emb, states: &[[f64; 3]]) -> f64 {
    if states.len() < 2 {
        return 0.0;
    }
    let n = states.len() - 1;
    let gov = emb.governor();
    let mut span = vec![0.0f64; n];
    let mut dyaw = vec![0.0f64; n];
    let mut arcs = vec![0.0f64; n + 1];
    let mut total = 0.0;
    for m in 0..n {
        let (a, b) = (states[m], states[m + 1]);
        span[m] = (b[0] - a[0]).hypot(b[1] - a[1]);
        dyaw[m] = rem_2pi(b[2] - a[2]);
        let moving = span[m] > 1e-9;
        arcs[m + 1] = arcs[m] + if moving { span[m] } else { 0.0 };
        if !moving {
            // A rotation in place carries no arc, so it is priced here.
            let th = a[2] + 0.5 * dyaw[m];
            total += emb.yaw_w
                * dyaw[m].abs()
                * gov.tight(pose_clear(w, offs, a[0], a[1], th))
                * w.mult_at(a[0], a[1]);
        }
    }
    let mv: Vec<usize> = (0..n).filter(|&m| span[m] > 1e-9).collect();
    let length = arcs[n];
    if mv.is_empty() || length <= 0.0 {
        return total;
    }
    // Even sub-steps over the whole route, so an added vertex moves no sample.
    let nk = ((length / COST_STEP).ceil() as usize).max(1);
    let h = length / nk as f64;
    let mut p = 0usize;
    for q in 0..nk {
        let mid = (q as f64 + 0.5) * h;
        while p + 1 < mv.len() && arcs[mv[p + 1]] <= mid {
            p += 1;
        }
        let m = mv[p];
        let (a, b) = (states[m], states[m + 1]);
        let t = ((mid - arcs[m]) / span[m]).clamp(0.0, 1.0);
        let th = a[2] + t * dyaw[m];
        let rel = (b[1] - a[1]).atan2(b[0] - a[0]) - th;
        let gait = 1.0
            + (emb.strafe - 1.0) * rel.sin().abs()
            + if rel.cos() < 0.0 {
                emb.reverse - 1.0
            } else {
                0.0
            };
        // Turning while translating is a blend edge, and pays half the yaw price.
        let turn = 0.5 * emb.yaw_w * dyaw[m].abs() / span[m];
        let x = a[0] + t * (b[0] - a[0]);
        let y = a[1] + t * (b[1] - a[1]);
        total += (gait + turn) * h * gov.tight(pose_clear(w, offs, x, y, th)) * w.mult_at(x, y);
    }
    total
}

/// Head-trim a published route to its nearest waypoint; the true pose does not
/// replace the head. `se2.py::trim_to_pose`.
fn trim_to_pose(states: &[[f64; 3]], pose: (f64, f64, f64)) -> Vec<[f64; 3]> {
    if states.is_empty() {
        return vec![[pose.0, pose.1, pose.2]];
    }
    let mut best = 0usize;
    let mut bd = f64::INFINITY;
    for (i, s) in states.iter().enumerate() {
        let d = (s[0] - pose.0).hypot(s[1] - pose.1);
        if d < bd {
            bd = d;
            best = i;
        }
    }
    states[best..].to_vec()
}

/// The published route, trimmed to here and carried to the goal, or None when
/// this map no longer allows it.
fn committed(
    cl: &mut Clear,
    fps: &Fps,
    emb: &Emb,
    incumbent: &[[f64; 3]],
    pose: (f64, f64, f64),
    goal: (f64, f64),
    margin: f64,
) -> Option<(Vec<[f64; 3]>, bool)> {
    let mut route = trim_to_pose(incumbent, pose);
    if route.len() < 2 {
        return None;
    }
    // Re-validate first (cheap), carry second (a search); collision-free, not
    // the planning margin.
    for pair in route.windows(2) {
        if !seg_free(cl.w, fps, emb, &pair[0], &pair[1], 0.0) {
            return None;
        }
    }
    let end = *route.last().expect("len >= 2");
    let cell = |v: f64| (v / CELL).round_even_i64();
    // The goal moves between replans: carry the route on by chord, else by
    // search from the far end.
    let mut carried = false;
    if (cell(end[0]), cell(end[1])) != (cell(goal.0), cell(goal.1)) {
        carried = true;
        let tgt = [goal.0, goal.1, end[2]];
        if seg_free(cl.w, fps, emb, &end, &tgt, margin) {
            route.push(tgt);
        } else {
            let join = route.len() - 1;
            // On the clearance table the fresh search just filled.
            route.extend_from_slice(&se2_search_in(
                cl,
                fps,
                (end[0], end[1], end[2]),
                goal,
                emb,
                margin,
            )?);
            for pair in route[join..].windows(2) {
                if !seg_free(cl.w, fps, emb, &pair[0], &pair[1], 0.0) {
                    return None;
                }
            }
        }
    }
    Some((route, carried))
}

/// Both routes priced from where the robot actually is.
fn priced(pose: (f64, f64, f64), states: &[[f64; 3]]) -> Vec<[f64; 3]> {
    let here = [pose.0, pose.1, pose.2];
    let s = states[0];
    if (s[0] - here[0]).abs() < 1e-9
        && (s[1] - here[1]).abs() < 1e-9
        && (s[2] - here[2]).abs() < 1e-9
    {
        return states.to_vec();
    }
    let mut out = vec![here];
    out.extend_from_slice(states);
    out
}

/// `incumbent` is the route already published; it is trimmed to `pose`,
/// re-validated, carried to the goal, and kept unless the fresh search beats
/// it by `commit_margin`.
pub fn plan(
    points: &[[f64; 2]],
    pose: (f64, f64, f64),
    goal: (f64, f64),
    emb: &Emb,
    resolution: f64,
    incumbent: Option<&[[f64; 3]]>,
    commit_margin: f64,
) -> Option<Vec<[f64; 3]>> {
    plan_explored(
        points,
        &[],
        1.0,
        pose,
        goal,
        emb,
        resolution,
        incumbent,
        commit_margin,
    )
}

/// `plan`, pricing lattice cells with no `ground` return at `unseen_cost`; the
/// heuristic never sees it, so the search stays exact.
#[allow(clippy::too_many_arguments)]
pub fn plan_explored(
    points: &[[f64; 2]],
    ground: &[[f64; 2]],
    unseen_cost: f64,
    pose: (f64, f64, f64),
    goal: (f64, f64),
    emb: &Emb,
    resolution: f64,
    incumbent: Option<&[[f64; 3]]>,
    commit_margin: f64,
) -> Option<Vec<[f64; 3]>> {
    let fps = Fps::new(emb);
    let offs = fps.union().to_vec();
    let reach = reach_of(&offs);
    // Covers the certificate (`speed_clearance`) and the smoothing floor (`comfort`).
    let cap = emb.comfort.max(emb.speed_clearance) + reach + SNAP;
    let mut w = build_world_explored(points, ground, unseen_cost, pose, goal, cap);
    let margin = emb.precision;
    // World from {pose, goal, cloud}, never the incumbent; one clearance table for both.
    let (fresh, held) = {
        let (gx, gy) = lattice_axes(&w);
        let mut cl = Clear::new(&mut w, &fps, margin, gx, gy, emb.governor());
        let fresh = se2_search_in(&mut cl, &fps, pose, goal, emb, margin);
        let held = match incumbent {
            None => None,
            Some(inc) => committed(&mut cl, &fps, emb, inc, pose, goal, margin),
        };
        (fresh, held)
    };
    // A kept, uncarried route is already at resolution.
    let (states, dense) = match (fresh, held) {
        (fresh, None) => (fresh?, false),
        // A still-walkable route beats a stub.
        (None, Some((route, carried))) => (route, !carried),
        (Some(f), Some((route, carried))) => {
            let cf = path_cost(&mut w, &offs, emb, &priced(pose, &f));
            let cr = path_cost(&mut w, &offs, emb, &priced(pose, &route));
            if cf < cr - commit_margin {
                (f, false)
            } else {
                (route, !carried)
            }
        }
    };
    if dense {
        return Some(states);
    }
    Some(densify(
        &mut w,
        &offs,
        &states,
        resolution,
        sweep_slack(reach),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Unexplored terrain is priced, not forbidden.
    #[test]
    fn unexplored_terrain_is_detoured_not_crossed() {
        let emb = Emb::fixture();
        let mut ground = Vec::new();
        let mut y = 1.0;
        while y <= 2.0 {
            let mut x = -1.0;
            while x <= 5.0 {
                ground.push([x, y]);
                x += 0.1;
            }
            y += 0.1;
        }
        // The endpoints themselves are seen, so start and goal cells are cheap.
        for c in [[0.0, 0.0], [4.0, 0.0]] {
            ground.push(c);
        }
        let run = |cost: f64| {
            let p = plan_explored(
                &[],
                &ground,
                cost,
                (0.0, 0.0, 0.0),
                (4.0, 0.0),
                &emb,
                0.1,
                None,
                COMMIT_MARGIN,
            )
            .expect("open world routes");
            p.iter().map(|s| s[1]).fold(0.0f64, f64::max)
        };
        assert!(run(1.0) < 0.2, "straight without the layer");
        assert!(run(5.0) > 0.8, "bends into the seen corridor");
        // Forbidding is not the contract: a goal in the dark is still reached.
        assert!(plan_explored(
            &[],
            &[[0.0, 0.0]],
            5.0,
            (0.0, 0.0, 0.0),
            (4.0, 0.0),
            &emb,
            0.1,
            None,
            COMMIT_MARGIN
        )
        .is_some());
    }

    /// Ring of obstacle points (square outline), spacing `step`, half-size `h`.
    fn ring(cx: f64, cy: f64, h: f64, step: f64) -> Vec<[f64; 2]> {
        let mut pts = Vec::new();
        let mut t = -h;
        while t <= h {
            pts.push([cx - h, cy + t]);
            pts.push([cx + h, cy + t]);
            pts.push([cx + t, cy - h]);
            pts.push([cx + t, cy + h]);
            t += step;
        }
        pts
    }

    /// The inlined rounding is the libm one, bit for bit, over every value the
    /// planner can hand it.
    #[test]
    fn round_even_matches_round_ties_even() {
        let mut xs: Vec<f64> = Vec::new();
        // Step by an eighth so every tie and quarter-way case appears.
        let mut k = -80_000i64;
        while k <= 80_000 {
            xs.push(k as f64 / 8.0);
            k += 1;
        }
        for &e in &[1e-16, 1e-9, 1e-3, 0.25, 0.5, 1.0, 1e6, 1e9, 2.0f64.powi(50)] {
            xs.push(e);
            xs.push(-e);
            xs.push(e + 0.5);
            xs.push(-e - 0.5);
        }
        xs.push(0.0);
        xs.push(-0.0);
        // A deterministic spread of non-dyadic values across the useful range.
        let mut u: u64 = 0x2545_F491_4F6C_DD1D;
        for _ in 0..200_000 {
            u ^= u << 13;
            u ^= u >> 7;
            u ^= u << 17;
            let v = ((u >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * 4.0e6;
            xs.push(v);
        }
        for x in xs {
            assert_eq!(
                x.round_even_i64(),
                x.round_ties_even() as i64,
                "round_even_i64 disagrees at {x:?}"
            );
        }
    }

    /// No station window reaches the 0.5 rad cylinder threshold; both tiers
    /// must appear.
    #[test]
    fn published_yaw_never_reaches_the_cylinder_threshold() {
        let emb = Emb::fixture();
        // A slot barely wider than the body forces the fine tier.
        let mut tight = ring(2.6, 0.0, 1.2, 0.05);
        tight.retain(|p| !(p[1].abs() < 0.30 && p[0] > 3.0));
        for x in [3.0f64, 3.05, 3.1, 3.15, 3.2] {
            for s in [-1.0f64, 1.0] {
                tight.push([x, s * 0.30]);
            }
        }
        let worlds = [Vec::new(), ring(2.0, 0.0, 0.25, 0.05), tight];
        let mut steps: Vec<f64> = Vec::new();
        for res in [0.1f64, 0.075, 0.15] {
            let stride = station_stride(res) as usize;
            for pts in &worlds {
                let Some(path) = plan(
                    pts,
                    (0.0, 0.0, 0.0),
                    (4.0, 0.0),
                    &emb,
                    res,
                    None,
                    COMMIT_MARGIN,
                ) else {
                    continue;
                };
                for w in path.windows(2) {
                    steps.push(rem_2pi(w[1][2] - w[0][2]).abs());
                }
                for w in path.windows(stride + 1) {
                    let d = rem_2pi(w[stride][2] - w[0][2]).abs();
                    assert!(
                        d <= MAX_STATION_YAW + 1e-9,
                        "station window {d} at res {res} exceeds MAX_STATION_YAW"
                    );
                }
            }
        }
        // Coverage: a run that only ever took one tier would prove nothing.
        assert!(
            steps.iter().any(|&d| d > YAW_STEP + 1e-9),
            "no segment took the coarse tier: the gate is inert"
        );
        assert!(
            steps.iter().any(|&d| d > 1e-9 && d <= YAW_STEP + 1e-9),
            "no segment took the fine tier: the gate never refuses"
        );
    }

    /// `sweep_slack` covers the worst-case rotation excursion with the two
    /// discretisation terms left over.
    #[test]
    #[allow(clippy::assertions_on_constants)] // the constant bound IS the property under test
    fn sweep_slack_dominates_the_rotation_excursion() {
        assert!(
            MAX_STATION_YAW < 0.5,
            "station window reaches the cylinder threshold"
        );
        for reach in [0.1f64, reach_of(Fps::new(&Emb::fixture()).union()), 0.9] {
            let excursion = 2.0 * reach * (0.5 * MAX_STATION_YAW).sin();
            assert!(
                sweep_slack(reach) - excursion >= SNAP + 0.05 - 1e-12,
                "slack {} leaves under SNAP+0.05 over excursion {excursion} at reach {reach}",
                sweep_slack(reach)
            );
        }
        // The per-waypoint step must add up to the window at every resolution.
        for res in [0.05f64, 0.075, 0.1, 0.15, 0.3, 0.6] {
            let stride = station_stride(res);
            assert!(
                (stride * (MAX_STATION_YAW / stride) - MAX_STATION_YAW).abs() < 1e-12,
                "stride {stride} at res {res} does not reconstruct MAX_STATION_YAW"
            );
        }
    }

    #[test]
    fn determinism() {
        let pts = ring(2.0, 0.0, 0.25, 0.05);
        let emb = Emb::fixture();
        let a = plan(
            &pts,
            (0.0, 0.0, 0.0),
            (4.0, 0.0),
            &emb,
            0.1,
            None,
            COMMIT_MARGIN,
        )
        .unwrap();
        let b = plan(
            &pts,
            (0.0, 0.0, 0.0),
            (4.0, 0.0),
            &emb,
            0.1,
            None,
            COMMIT_MARGIN,
        )
        .unwrap();
        assert_eq!(a.len(), b.len());
        for (p, q) in a.iter().zip(&b) {
            for k in 0..3 {
                assert_eq!(p[k].to_bits(), q[k].to_bits());
            }
        }
    }

    #[test]
    fn thin_wall_not_hopped() {
        // A tiny body blocks one lattice column; without midpoint checks the
        // search hops through.
        let emb = Emb {
            length: 0.06,
            width: 0.06,
            center_off: 0.0,
            comfort: 0.4,
            precision: 0.01,
            strafe: 1.8,
            reverse: 1.5,
            yaw_w: 0.25,
            envelope: Vec::new(),
            arc_inflate: 0.0,
            ..Emb::fixture()
        };
        let mut pts = Vec::new();
        let mut y = -4.0;
        while y <= 4.0 {
            pts.push([2.0, y]);
            y += 0.02;
        }
        let path = plan(
            &pts,
            (0.0, 0.0, 0.0),
            (4.0, 0.0),
            &emb,
            0.1,
            None,
            COMMIT_MARGIN,
        )
        .unwrap();
        for w in path.windows(2) {
            let (a, b) = (w[0], w[1]);
            if (a[0] - 2.0) * (b[0] - 2.0) < 0.0 {
                let t = (2.0 - a[0]) / (b[0] - a[0]);
                let y = a[1] + t * (b[1] - a[1]);
                assert!(y.abs() > 3.8, "path hopped the wall at y={y:.2}");
            }
        }
    }

    #[test]
    fn sealed_box_refuses() {
        let pts = ring(0.0, 0.0, 1.0, 0.02);
        assert!(plan(
            &pts,
            (0.0, 0.0, 0.0),
            (4.0, 0.0),
            &Emb::fixture(),
            0.1,
            None,
            COMMIT_MARGIN
        )
        .is_none());
    }

    /// The governor price stays in [1.0, max_speed / min_speed].
    #[test]
    fn governor_price_is_never_below_one() {
        let emb = Emb::fixture();
        let fps = Fps::new(&emb);
        let pts = ring(2.0, 0.0, 0.6, 0.03);
        let cap = emb.comfort.max(emb.speed_clearance) + reach_of(fps.union()) + SNAP;
        let mut w = build_world(&pts, (0.0, 0.0, 0.0), (4.0, 0.0), cap);
        let (x0, y0, x1, y1) = w.bounds;
        let gx = arange(x0, x1 + CELL, CELL);
        let gy = arange(y0, y1 + CELL, CELL);
        let (nx, ny) = (gx.len(), gy.len());
        let mut cl = Clear::new(&mut w, &fps, emb.precision, gx, gy, emb.governor());
        let (mut blocked, mut charged) = (0usize, 0usize);
        for b in 0..YAW_BINS {
            for i in 0..nx {
                for j in 0..ny {
                    let uv = cl.clear(b, i, j);
                    let v = cl.price(uv);
                    assert!(
                        (1.0..=cl.gov.tight_max()).contains(&v),
                        "price {v} at bin {b}, cell ({i}, {j}) leaves the governor band"
                    );
                    if cl.clear(b, i, j) < 0.0 {
                        blocked += 1;
                    } else if v > 1.0 {
                        charged += 1;
                    }
                }
            }
        }
        // Not a vacuous pass: the fixture must exercise both branches.
        assert!(blocked > 0, "fixture saw no blocked state");
        assert!(charged > 0, "fixture saw no tightness-charged state");
    }

    /// Every envelope row is nested inside the union.
    #[test]
    fn every_row_is_nested_inside_the_union() {
        let emb = Emb::fixture();
        let u = emb.union_box();
        let (ux0, ux1) = (u[2] - u[0] / 2.0, u[2] + u[0] / 2.0);
        let (uy0, uy1) = (u[3] - u[1] / 2.0, u[3] + u[1] / 2.0);
        for row in &emb.envelope {
            for s in [1.0f64, -1.0] {
                let r = [row[1], row[2], row[3], s * row[4]];
                let (rx0, rx1) = (r[2] - r[0] / 2.0, r[2] + r[0] / 2.0);
                let (ry0, ry1) = (r[3] - r[1] / 2.0, r[3] + r[1] / 2.0);
                assert!(
                    rx0 >= ux0 && rx1 <= ux1 && ry0 >= uy0 && ry1 <= uy1,
                    "row at {} deg escapes the union: x [{rx0}, {rx1}] y [{ry0}, {ry1}]",
                    row[0]
                );
            }
        }
    }

    /// The standing box is nested inside every row and matches
    /// `embodiment/base.py::stand_box`.
    #[test]
    fn the_standing_box_is_nested_inside_every_row() {
        let emb = Emb::fixture();
        let b = emb.stand_box();
        let (bx0, bx1) = (b[2] - b[0] / 2.0, b[2] + b[0] / 2.0);
        let (by0, by1) = (b[3] - b[1] / 2.0, b[3] + b[1] / 2.0);
        assert_eq!(b[3], 0.0, "mirroring cannot leave a lateral offset behind");
        assert!(b[0] < emb.length && b[1] < emb.width, "still the union");
        for row in &emb.envelope {
            for s in [1.0f64, -1.0] {
                let r = [row[1], row[2], row[3], s * row[4]];
                let (rx0, rx1) = (r[2] - r[0] / 2.0, r[2] + r[0] / 2.0);
                let (ry0, ry1) = (r[3] - r[1] / 2.0, r[3] + r[1] / 2.0);
                assert!(
                    rx0 <= bx0 + 1e-12
                        && rx1 >= bx1 - 1e-12
                        && ry0 <= by0 + 1e-12
                        && ry1 >= by1 - 1e-12,
                    "the standing box escapes the {} deg row",
                    row[0]
                );
            }
        }
        // 0.781 x 0.416 at -0.039: the fixture's 180-degree row.
        assert!((b[0] - 0.781).abs() < 5e-4, "length {}", b[0]);
        assert!((b[1] - 0.416).abs() < 5e-4, "width {}", b[1]);
        assert!((b[2] + 0.039).abs() < 5e-4, "off_x {}", b[2]);
        // Nobody measured the others, so they stand in their union.
        let plain = Emb {
            envelope: Vec::new(),
            ..Emb::fixture()
        };
        assert_eq!(plain.stand_box(), plain.union_box());
        assert_eq!(Fps::new(&plain).stand, 0);
    }

    /// Lookup is exact at the measured rows, and `off_y` mirrors with drift sign.
    #[test]
    fn envelope_lookup_is_exact_at_the_lattice_drift_angles() {
        let emb = Emb::fixture();
        for row in &emb.envelope {
            let d = row[0].to_radians();
            for s in [1.0f64, -1.0] {
                let got = emb.envelope_at(s * d);
                assert_eq!(got[0], row[1], "length at {} deg", s * row[0]);
                assert_eq!(got[1], row[2], "width at {} deg", s * row[0]);
                assert_eq!(got[2], row[3], "off_x at {} deg", s * row[0]);
                // 0 and 180 fold onto themselves, so their off_y keeps its sign.
                let want = if s < 0.0 && row[0] > 0.0 && row[0] < 180.0 {
                    -row[4]
                } else {
                    row[4]
                };
                assert_eq!(got[3], want, "off_y at {} deg", s * row[0]);
            }
        }
        // An embodiment with no measured rows reads the union at every heading.
        let plain = Emb {
            envelope: Vec::new(),
            ..Emb::fixture()
        };
        for deg in [0.0f64, 37.0, 90.0, 180.0, -140.0] {
            assert_eq!(plain.envelope_at(deg.to_radians()), plain.union_box());
        }
    }

    /// A far point can add rows but never re-phase the field: bit-exact.
    #[test]
    fn a_far_point_cannot_move_the_answer() {
        let emb = Emb::fixture();
        let pts = ring(2.0, 0.0, 0.45, 0.05);
        let base = plan(
            &pts,
            (0.0, 0.0, 0.0),
            (4.0, 0.0),
            &emb,
            0.1,
            None,
            COMMIT_MARGIN,
        )
        .expect("route exists");
        for far in [[-11.7, -6.0], [-40.3, 0.0], [0.0, -17.9]] {
            let mut with = pts.clone();
            with.push(far);
            let got = plan(
                &with,
                (0.0, 0.0, 0.0),
                (4.0, 0.0),
                &emb,
                0.1,
                None,
                COMMIT_MARGIN,
            )
            .expect("route exists");
            assert_eq!(
                got.len(),
                base.len(),
                "a point at {far:?} changed the path length"
            );
            for (k, (p, q)) in base.iter().zip(&got).enumerate() {
                for c in 0..3 {
                    assert_eq!(
                        p[c].to_bits(),
                        q[c].to_bits(),
                        "a point at {far:?} moved pose {k} component {c}: {} vs {}",
                        p[c],
                        q[c]
                    );
                }
            }
        }
    }

    /// A whole-`PERIOD` translation translates the route (not bit-exact:
    /// `PERIOD` is not dyadic).
    #[test]
    fn a_whole_period_translation_translates_the_answer() {
        let emb = Emb::fixture();
        let pts = ring(2.0, 0.0, 0.45, 0.05);
        let base = plan(
            &pts,
            (0.0, 0.0, 0.0),
            (4.0, 0.0),
            &emb,
            0.1,
            None,
            COMMIT_MARGIN,
        )
        .expect("route exists");
        let d = 4.0 * PERIOD;
        let moved: Vec<[f64; 2]> = pts.iter().map(|p| [p[0] + d, p[1] + d]).collect();
        let got = plan(
            &moved,
            (d, d, 0.0),
            (4.0 + d, d),
            &emb,
            0.1,
            None,
            COMMIT_MARGIN,
        )
        .expect("route exists");
        let arc = |p: &[[f64; 3]]| -> f64 {
            p.windows(2)
                .map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]))
                .sum()
        };
        assert!(
            (arc(&base) - arc(&got)).abs() < CELL,
            "translation changed the route length: {} vs {}",
            arc(&base),
            arc(&got)
        );
        // Every pose of the translated answer sits on the untranslated one.
        for q in &got {
            let near = base
                .iter()
                .map(|p| (q[0] - d - p[0]).hypot(q[1] - d - p[1]))
                .fold(f64::INFINITY, f64::min);
            assert!(
                near < CELL,
                "translated pose {q:?} is {near:.3} m off the route"
            );
        }
    }

    /// The coarse-to-fine emission order is a permutation: same set, each once.
    #[test]
    fn footprint_sample_order_is_a_permutation() {
        for emb in [
            Emb::fixture(),
            Emb {
                width: 0.45,
                comfort: 0.5,
                ..Emb::fixture()
            },
            Emb {
                length: 2.0,
                width: 0.24,
                comfort: 0.3,
                ..Emb::fixture()
            },
        ] {
            let boxes = std::iter::once(emb.union_box())
                .chain(emb.envelope.iter().map(|r| [r[1], r[2], r[3], r[4]]));
            for bx in boxes {
                let got = offsets(&bx);
                let (hl, hw) = (bx[0] / 2.0, bx[1] / 2.0);
                let xs = arange(-hl, hl + OFFSET_STEP / 2.0, OFFSET_STEP);
                let ys = arange(-hw, hw + OFFSET_STEP / 2.0, OFFSET_STEP);
                let mut want: Vec<(u64, u64)> = Vec::new();
                for &x in &xs {
                    for &y in &ys {
                        want.push(((x + bx[2]).to_bits(), (y + bx[3]).to_bits()));
                    }
                }
                let mut have: Vec<(u64, u64)> = got
                    .iter()
                    .map(|&(x, y)| (x.to_bits(), y.to_bits()))
                    .collect();
                assert_eq!(have.len(), want.len(), "sample count changed");
                have.sort_unstable();
                let mut want_sorted = want.clone();
                want_sorted.sort_unstable();
                assert_eq!(have, want_sorted, "sample set changed");
            }
        }
    }

    /// The lazy clearance table matches an eager, uncapped full scan.
    #[test]
    #[allow(clippy::needless_range_loop)] // (b, i, j) index every array in the body
    fn lazy_clearance_matches_full_footprint_scan() {
        let emb = Emb::fixture();
        let fps = Fps::new(&emb);
        let pts = ring(2.0, 0.0, 0.6, 0.03);
        let offs = fps.union().to_vec();
        let cap = emb.comfort.max(emb.speed_clearance) + reach_of(&offs) + SNAP;
        let mut w = build_world(&pts, (0.0, 0.0, 0.0), (4.0, 0.0), cap);
        // Reference field: no cap, so every cell holds its exact distance.
        let mut wref = build_world(&pts, (0.0, 0.0, 0.0), (4.0, 0.0), f64::INFINITY);
        let (x0, y0, x1, y1) = w.bounds;
        let gx = arange(x0, x1 + CELL, CELL);
        let gy = arange(y0, y1 + CELL, CELL);
        let (nx, ny) = (gx.len(), gy.len());
        let thetas = yaw_bins();
        let noff = offs.len();
        let margin = emb.precision;
        let mut cl = Clear::new(&mut w, &fps, margin, gx.clone(), gy.clone(), emb.governor());
        for b in 0..YAW_BINS {
            let (s, c) = thetas[b].sin_cos();
            for i in 0..nx {
                for j in 0..ny {
                    for fp in 0..fps.offs.len() {
                        let got = cl.row_clear(b, fp, i, j);
                        let mut m = f64::INFINITY;
                        for &(ox, oy) in &fps.offs[fp] {
                            let d = wref.lookup(gx[i] + c * ox - s * oy, gy[j] + s * ox + c * oy);
                            if d < m {
                                m = d;
                            }
                        }
                        let want = if m > margin { m } else { -1.0 };
                        // Above `speed_clearance` every consumer has saturated.
                        if got >= emb.speed_clearance && want >= emb.speed_clearance {
                            continue;
                        }
                        assert_eq!(
                            got.to_bits(),
                            want.to_bits(),
                            "clearance mismatch at bin {b}, box {fp}, cell ({i}, {j})"
                        );
                    }
                }
            }
        }
        assert!(noff > 0);
    }
}
