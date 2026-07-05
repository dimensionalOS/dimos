// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Level-aware height-cost occupancy — a faithful port of the measured Python
//! semantics in `dimos/mapping/pointclouds/occupancy.py` (height_cost with
//! `flag_dropoffs` + `reference_z`), self-contained so the local planner owns
//! its costmap (no CostMapper module) and can run it at high resolution.
//!
//! Layer chain (each rule carries the failure it was measured against, see the
//! Python file for the full history):
//!   1. band slice around the robot's storey (overhead + far-below removed)
//!   2. min/max height maps; pass-under only for true overhangs (mid-band empty)
//!   3. gradient cost (Sobel; can_climb scales height-change-per-cell to 0-100)
//!   4. drop-off layers: rim flags (fall > max_safe_fall), deep-void lethal
//!      (landing > void_depth_lethal below the band floor), GRADED shallow
//!      voids traversable (stairs descend riser-by-riser; ledges do not),
//!      unknown lethal except the ascent frontier / interior holes / graded
//!      shallow voids, below-band landing gap-fill (3x3 max, no-data cells only)
//!   5. chamfer distance transform to the lethal set (the solver's clearance
//!      field), cached with the grid.

/// Costmap tunables. Field names/meanings mirror the Python HeightCostConfig.
#[derive(Clone, Debug)]
pub struct CostmapConfig {
    pub resolution: f32,
    pub can_pass_under: f32,
    pub can_climb: f32,
    pub ignore_noise: f32,
    pub max_safe_fall: f32,
    pub void_depth_lethal: f32,
    pub slice_below: f32,
    pub slice_above: f32,
    /// Half-extent (m) of the square window kept around the robot. The Python
    /// pipeline gridded whatever the terrain slice covered; the internal
    /// costmap bounds work explicitly so the solve window is predictable.
    pub half_extent: f32,
    /// Storey-reference hysteresis (m): the band only moves when the robot's z
    /// drifts further than this from the current reference (0.25 = one riser;
    /// 0.5 measurably broke the climb — the ascent frontier lagged).
    pub level_hysteresis: f32,
}

impl Default for CostmapConfig {
    fn default() -> Self {
        Self {
            // Matched to the terrain mapper's 0.1 m voxel output — a finer grid
            // is under-sampled by the input (hl58). Raise together with the
            // mapper's voxel size when pursuing Jeff's higher-res goal.
            resolution: 0.1,
            can_pass_under: 0.6,
            can_climb: 1.2,
            ignore_noise: 0.05,
            max_safe_fall: 0.5,
            void_depth_lethal: 2.5,
            slice_below: 1.1,
            slice_above: 1.5,
            half_extent: 8.0,
            level_hysteresis: 0.25,
        }
    }
}

pub const LETHAL: i8 = 100;
pub const UNKNOWN: i8 = -1;
pub const LETHAL_THRESHOLD: i8 = 50;

/// The grid + derived layers the solver consumes. `cost` uses the ROS
/// occupancy convention (0 free .. 100 lethal, -1 unknown); `distance` is
/// metres to the nearest lethal cell (chamfer approximation).
pub struct Costmap {
    pub width: usize,
    pub height: usize,
    pub resolution: f32,
    /// World coordinates of cell (row 0, col 0).
    pub origin: (f32, f32),
    pub cost: Vec<i8>,
    pub distance: Vec<f32>,
}

impl Costmap {
    #[inline]
    pub fn index(&self, row: usize, col: usize) -> usize {
        row * self.width + col
    }

    /// (row, col) for a world point, or None when outside the grid.
    #[inline]
    pub fn cell(&self, x: f32, y: f32) -> Option<(usize, usize)> {
        let col = ((x - self.origin.0) / self.resolution + 0.5).floor();
        let row = ((y - self.origin.1) / self.resolution + 0.5).floor();
        if col < 0.0 || row < 0.0 || col as usize >= self.width || row as usize >= self.height {
            return None;
        }
        Some((row as usize, col as usize))
    }

    #[inline]
    pub fn cell_center(&self, row: usize, col: usize) -> (f32, f32) {
        (
            self.origin.0 + col as f32 * self.resolution,
            self.origin.1 + row as f32 * self.resolution,
        )
    }
}

/// Storey-reference tracker (port of CostMapper._track_robot_z).
#[derive(Default)]
pub struct LevelTracker {
    reference: Option<f32>,
}

impl LevelTracker {
    pub fn update(&mut self, z: f32, hysteresis: f32) -> f32 {
        match self.reference {
            Some(reference) if (z - reference).abs() <= hysteresis => reference,
            _ => {
                self.reference = Some(z);
                z
            }
        }
    }
}

/// Build the costmap from a world-frame point cloud around the robot.
///
/// `points` is (x, y, z) triples; `robot` the current pose used for the window
/// and the storey band. Points outside the window are ignored.
pub fn build(points: &[[f32; 3]], robot: (f32, f32, f32), reference_z: f32, cfg: &CostmapConfig) -> Costmap {
    let res = cfg.resolution;
    let half = cfg.half_extent;
    let min_x = robot.0 - half;
    let min_y = robot.1 - half;
    let width = ((2.0 * half) / res).ceil() as usize;
    let height = width;
    let n = width * height;

    let z_lo = reference_z - cfg.slice_below;
    let z_hi = reference_z + cfg.slice_above;

    let mut min_h = vec![f32::NAN; n];
    let mut max_h = vec![f32::NAN; n];
    let mut mid_count = vec![0u16; n];
    let mut below_h = vec![f32::NEG_INFINITY; n];
    let mut above = vec![false; n];

    let inv_res = 1.0 / res;
    let cell_of = |x: f32, y: f32| -> Option<usize> {
        let col = ((x - min_x) * inv_res + 0.5).floor();
        let row = ((y - min_y) * inv_res + 0.5).floor();
        if col < 0.0 || row < 0.0 || col as usize >= width || row as usize >= height {
            return None;
        }
        Some(row as usize * width + col as usize)
    };

    // Pass 1: height extrema per cell, split by band.
    for p in points {
        let Some(i) = cell_of(p[0], p[1]) else { continue };
        let z = p[2];
        if z < z_lo {
            if z > below_h[i] {
                below_h[i] = z; // below-band LANDING = max return (first surface you'd land on)
            }
        } else if z > z_hi {
            above[i] = true;
        } else {
            if min_h[i].is_nan() || z < min_h[i] {
                min_h[i] = z;
            }
            if max_h[i].is_nan() || z > max_h[i] {
                max_h[i] = z;
            }
        }
    }
    // Pass 2: mid-band counts (overhang detection needs min_h from pass 1).
    for p in points {
        let Some(i) = cell_of(p[0], p[1]) else { continue };
        let z = p[2];
        if z < z_lo || z > z_hi || min_h[i].is_nan() {
            continue;
        }
        let floor = min_h[i];
        if z > floor + 0.15 && z < floor + cfg.can_pass_under {
            mid_count[i] += 1;
        }
    }

    // Effective surface: pass-under only for true overhangs (bridge/ceiling —
    // big min->max gap AND an empty robot-body band above the floor). A wall
    // has continuous returns and must keep its max (using the floor erases the
    // wall — the planner then routes through 3 m walls; recorded at wp4).
    let mut surface = vec![f32::NAN; n];
    for i in 0..n {
        if max_h[i].is_nan() {
            continue;
        }
        let overhang = (max_h[i] - min_h[i]) > cfg.can_pass_under && mid_count[i] == 0;
        surface[i] = if overhang { min_h[i] } else { max_h[i] };
    }

    // Hole fill (stands in for the Python gaussian-weighted smoothing): the
    // terrain mapper publishes VOXELIZED clouds (0.1 m), so a finer grid has
    // interior holes in every other cell — without filling, the Sobel taps
    // read the holes as cliffs, the lethal set blankets the map, and the
    // robot is 'boxed in' (hl58: empty plans at 60 Hz for the whole course).
    // Two passes of 3x3 weighted averaging fill one-cell holes per pass;
    // observed cells keep their measured value.
    for _ in 0..2 {
        let src = surface.clone();
        for row in 0..height as isize {
            for col in 0..width as isize {
                let i = row as usize * width + col as usize;
                if !src[i].is_nan() {
                    continue;
                }
                let mut sum = 0.0f32;
                let mut count = 0u32;
                for dr in -1..=1_isize {
                    for dc in -1..=1_isize {
                        let (r, c) = (row + dr, col + dc);
                        if r < 0 || c < 0 || r as usize >= height || c as usize >= width {
                            continue;
                        }
                        let v = src[r as usize * width + c as usize];
                        if !v.is_nan() {
                            sum += v;
                            count += 1;
                        }
                    }
                }
                if count >= 3 {
                    surface[i] = sum / count as f32;
                }
            }
        }
    }
    let observed: Vec<bool> = surface.iter().map(|v| !v.is_nan()).collect();

    // Gradient cost via a 3x3 Sobel on the observed surface (NaN treated as 0
    // but only cells whose 4-neighbourhood is fully observed keep a valid
    // gradient — boundary gradients against unknown read as false cliffs).
    let mut cost = vec![UNKNOWN; n];
    let at = |r: isize, c: isize, data: &[f32]| -> f32 {
        if r < 0 || c < 0 || r as usize >= height || c as usize >= width {
            return 0.0;
        }
        let v = data[r as usize * width + c as usize];
        if v.is_nan() {
            0.0
        } else {
            v
        }
    };
    let obs_at = |r: isize, c: isize, obs: &[bool]| -> bool {
        r >= 0 && c >= 0 && (r as usize) < height && (c as usize) < width && obs[r as usize * width + c as usize]
    };
    for row in 0..height as isize {
        for col in 0..width as isize {
            let i = row as usize * width + col as usize;
            if !observed[i] {
                continue;
            }
            // 4-neighbourhood erosion of the observed mask.
            if !(obs_at(row - 1, col, &observed)
                && obs_at(row + 1, col, &observed)
                && obs_at(row, col - 1, &observed)
                && obs_at(row, col + 1, &observed))
            {
                continue;
            }
            let gx = (at(row - 1, col + 1, &surface) + 2.0 * at(row, col + 1, &surface) + at(row + 1, col + 1, &surface)
                - at(row - 1, col - 1, &surface)
                - 2.0 * at(row, col - 1, &surface)
                - at(row + 1, col - 1, &surface))
                / (8.0 * res);
            let gy = (at(row + 1, col - 1, &surface) + 2.0 * at(row + 1, col, &surface) + at(row + 1, col + 1, &surface)
                - at(row - 1, col - 1, &surface)
                - 2.0 * at(row - 1, col, &surface)
                - at(row - 1, col + 1, &surface))
                / (8.0 * res);
            let mut dh_per_cell = (gx * gx + gy * gy).sqrt() * res;
            if dh_per_cell < cfg.ignore_noise {
                dh_per_cell = 0.0;
            }
            cost[i] = ((dh_per_cell / cfg.can_climb) * 100.0).clamp(0.0, 100.0) as i8;
        }
    }

    dropoff_layers(&mut cost, &surface, &observed, &mut below_h, &above, z_lo, cfg, width, height);

    let distance = chamfer_distance(&cost, width, height, res);

    Costmap {
        width,
        height,
        resolution: res,
        origin: (min_x, min_y),
        cost,
        distance,
    }
}

/// Drop-off layer chain (port of occupancy.py's flag_dropoffs branch — each
/// rule's measured story lives there).
#[allow(clippy::too_many_arguments)]
fn dropoff_layers(
    cost: &mut [i8],
    surface: &[f32],
    observed: &[bool],
    below_h: &mut [f32],
    above: &[bool],
    z_lo: f32,
    cfg: &CostmapConfig,
    width: usize,
    height: usize,
) {
    let n = width * height;

    // Interior-hole fill of the observed mask: only voids OUTSIDE the surface
    // count as drops. Flood-fill the complement from the border; anything not
    // reached is an interior hole.
    let mut outside = vec![false; n];
    let mut stack: Vec<usize> = Vec::with_capacity(width * 2 + height * 2);
    let visit = |i: usize, outside: &mut Vec<bool>, stack: &mut Vec<usize>| {
        if !outside[i] && !observed[i] {
            outside[i] = true;
            stack.push(i);
        }
    };
    for col in 0..width {
        visit(col, &mut outside, &mut stack);
        visit((height - 1) * width + col, &mut outside, &mut stack);
    }
    for row in 0..height {
        visit(row * width, &mut outside, &mut stack);
        visit(row * width + width - 1, &mut outside, &mut stack);
    }
    while let Some(i) = stack.pop() {
        let row = i / width;
        let col = i % width;
        if row > 0 {
            visit(i - width, &mut outside, &mut stack);
        }
        if row + 1 < height {
            visit(i + width, &mut outside, &mut stack);
        }
        if col > 0 {
            visit(i - 1, &mut outside, &mut stack);
        }
        if col + 1 < width {
            visit(i + 1, &mut outside, &mut stack);
        }
    }
    let obs_filled: Vec<bool> = (0..n).map(|i| !outside[i]).collect();

    // Below-band landing gap-fill: no-data cells take the 3x3 neighbourhood max
    // (fill-only — measured landings are never diluted; a dilate-everything
    // variant weakened real rims to 59% edge coverage).
    let below_orig = below_h.to_vec();
    for row in 0..height as isize {
        for col in 0..width as isize {
            let i = row as usize * width + col as usize;
            if below_orig[i].is_finite() {
                continue;
            }
            let mut best = f32::NEG_INFINITY;
            for dr in -1..=1_isize {
                for dc in -1..=1_isize {
                    let (r, c) = (row + dr, col + dc);
                    if r < 0 || c < 0 || r as usize >= height || c as usize >= width {
                        continue;
                    }
                    let v = below_orig[r as usize * width + c as usize];
                    if v > best {
                        best = v;
                    }
                }
            }
            below_h[i] = best;
        }
    }

    // dropped = void past the surface edge with an observed below-band landing
    // and no rising terrain (ascent frontier is not a cliff).
    let dropped: Vec<bool> = (0..n)
        .map(|i| !obs_filled[i] && below_h[i].is_finite() && !above[i])
        .collect();

    // Deep void: landing far below the band floor -> lethal itself (the backstop
    // for cliff rims with occlusion gaps; recorded as a 6 m free-fall).
    for i in 0..n {
        if dropped[i] && (z_lo - below_h[i]) > cfg.void_depth_lethal {
            cost[i] = LETHAL;
        }
    }

    // Graded shallow voids: walkable ONLY when the landing is within
    // max_safe_fall of what the robot would stand on next door (stairs step
    // riser-by-riser; a terrace rim carries the full drop — recorded 2.9 m
    // fall). Ungraded shallow voids are lethal; graded ones explicitly free.
    let standing: Vec<f32> = (0..n)
        .map(|i| {
            let s = if observed[i] { surface[i] } else { f32::NEG_INFINITY };
            s.max(below_h[i])
        })
        .collect();
    let mut graded = vec![false; n];
    for row in 0..height as isize {
        for col in 0..width as isize {
            let i = row as usize * width + col as usize;
            if !below_h[i].is_finite() {
                continue;
            }
            let mut nb = f32::NEG_INFINITY;
            for dr in -1..=1_isize {
                for dc in -1..=1_isize {
                    let (r, c) = (row + dr, col + dc);
                    if r < 0 || c < 0 || r as usize >= height || c as usize >= width {
                        continue;
                    }
                    let v = standing[r as usize * width + c as usize];
                    if v > nb {
                        nb = v;
                    }
                }
            }
            graded[i] = (nb - below_h[i]) <= cfg.max_safe_fall;
        }
    }
    for i in 0..n {
        if !dropped[i] || (z_lo - below_h[i]) > cfg.void_depth_lethal {
            continue;
        }
        if graded[i] {
            if cost[i] == UNKNOWN {
                cost[i] = 0; // descent preview treads read traversable
            }
        } else {
            cost[i] = LETHAL; // ledge rim
        }
    }

    // Unknown is lethal in level-aware mode (the planner would treat it as
    // free and route over voids that merely had no returns; recorded 1.5 m
    // east of the stair corridor over a 5 m drop) — EXCEPT the ascent frontier
    // (above-band returns), interior holes, and the graded shallow voids
    // (already freed above).
    for i in 0..n {
        if cost[i] == UNKNOWN && !obs_filled[i] && !above[i] {
            cost[i] = LETHAL;
        }
    }

    // Rim flags on observed cells (8-neighbour so the lethal line has no
    // diagonal gaps): fall from this surface to the neighbour's landing beyond
    // max_safe_fall = an edge the robot must not roll off; one riser = stairs.
    let mut rim = vec![false; n];
    for row in 0..height as isize {
        for col in 0..width as isize {
            let i = row as usize * width + col as usize;
            if !observed[i] {
                continue;
            }
            let s = surface[i];
            'neigh: for dr in -1..=1_isize {
                for dc in -1..=1_isize {
                    if dr == 0 && dc == 0 {
                        continue;
                    }
                    let (r, c) = (row + dr, col + dc);
                    if r < 0 || c < 0 || r as usize >= height || c as usize >= width {
                        continue;
                    }
                    let j = r as usize * width + c as usize;
                    if dropped[j] && below_h[j].is_finite() && (s - below_h[j]) > cfg.max_safe_fall {
                        rim[i] = true;
                        break 'neigh;
                    }
                }
            }
        }
    }
    for i in 0..n {
        if rim[i] {
            cost[i] = LETHAL;
        }
    }
}

/// Two-pass 3-4 chamfer distance (in metres) to the lethal set. Within ~2% of
/// a true EDT at 8-connectivity — plenty for a clearance penalty ramp.
pub fn chamfer_distance(cost: &[i8], width: usize, height: usize, resolution: f32) -> Vec<f32> {
    const ORTH: f32 = 3.0;
    const DIAG: f32 = 4.0;
    let inf = f32::MAX / 4.0;
    let mut d: Vec<f32> = cost
        .iter()
        .map(|&c| if c >= LETHAL_THRESHOLD { 0.0 } else { inf })
        .collect();
    // Forward pass.
    for row in 0..height {
        for col in 0..width {
            let i = row * width + col;
            if d[i] == 0.0 {
                continue;
            }
            let mut best = d[i];
            if col > 0 {
                best = best.min(d[i - 1] + ORTH);
            }
            if row > 0 {
                best = best.min(d[i - width] + ORTH);
                if col > 0 {
                    best = best.min(d[i - width - 1] + DIAG);
                }
                if col + 1 < width {
                    best = best.min(d[i - width + 1] + DIAG);
                }
            }
            d[i] = best;
        }
    }
    // Backward pass.
    for row in (0..height).rev() {
        for col in (0..width).rev() {
            let i = row * width + col;
            if d[i] == 0.0 {
                continue;
            }
            let mut best = d[i];
            if col + 1 < width {
                best = best.min(d[i + 1] + ORTH);
            }
            if row + 1 < height {
                best = best.min(d[i + width] + ORTH);
                if col + 1 < width {
                    best = best.min(d[i + width + 1] + DIAG);
                }
                if col > 0 {
                    best = best.min(d[i + width - 1] + DIAG);
                }
            }
            d[i] = best;
        }
    }
    let scale = resolution / ORTH;
    for v in d.iter_mut() {
        *v = if *v >= inf { f32::MAX } else { *v * scale };
    }
    d
}
