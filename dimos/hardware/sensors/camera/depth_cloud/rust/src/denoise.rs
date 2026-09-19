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

//! Taking the thorns off a stereo depth image.
//!
//! Many of the errors in a stereo depth cloud look like spikes: one pixel
//! disagreeing with every neighbour. Every filter here works on the rectified
//! depth image **in place, at full resolution**, so the pixel grid is never the
//! thing that changed. Two of them deliberately turn pixels into holes and one
//! deliberately fills holes in; a filter that wins by answering less is not a
//! filter that won, so the answered count is worth watching either way.
//!
//! Every filter is a pure function of a neighbourhood, so every row of the
//! output is independent of every other: they run one row per thread. That is
//! not a micro-optimisation -- single-threaded, the shipped chain cost 37 ms a
//! frame, which made the denoiser the most expensive stage in the pipeline.
//!
//! The chain and its parameters were chosen by a week of scoring every filter
//! against the lidar map of the same scene (the `depth_eval` study); the
//! numbers in the doc comments below come from that.
//!
//! NaN means the matcher gave up. Nothing here reads a NaN as a depth, and
//! nothing writes one except `speckle` and `steep`, which is the point of them.

use rayon::prelude::*;

/// Which filter, and its one parameter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Denoise {
    None,
    /// The median of a `(2r+1)^2` neighbourhood. The classic answer to a spike:
    /// a thorn is one pixel disagreeing with its neighbours, and a median of
    /// nine is unmoved by one of them being wrong.
    Median(usize),
    /// Blank a pixel that disagrees with its neighbourhood's median by more than
    /// this. Not smoothing -- removal. The thorn goes, and the hole it leaves is
    /// one more pixel the camera did not answer, which is a cheaper mistake for
    /// a map than a point in the wrong place.
    Speckle(f32),
    /// A depth-guided bilateral: average the neighbourhood, weighting each
    /// neighbour by how far it is in space AND how far it is in depth. Smooths
    /// the noise on a surface without dragging an edge across the gap, which a
    /// plain blur does and which turns one thorn into nine.
    Bilateral(f32),
    /// Median at 1/n resolution, then back up to the full grid: smooth and
    /// then upsample. Rejects a spike over a wider support than a 3x3 can
    /// reach, at the cost of the fine detail.
    Coarse(usize),
    /// The plain box average of a `(2r+1)^2` neighbourhood: really heavy
    /// smoothing over large patches. A median is an order statistic: it picks
    /// one neighbour and ignores the rest, so it takes a thorn off and leaves
    /// the surface's own noise exactly where it was. A mean divides that noise
    /// by the root of the window's count, which is the only filter here that
    /// can touch it -- and pays by dragging every depth edge across the gap.
    ///
    /// Computed from a summed-area table over the finite pixels, so a 49x49
    /// window costs the same as a 3x3 and `r` can be as large as the picture.
    Mean(usize),
    /// Fit a plane to the neighbourhood and pull the pixel onto it. Radius,
    /// then how far to pull (1.0 lands on the plane exactly). Worse at complex
    /// geometry, better at the simple geometry -- floors and walls -- that
    /// navigation mostly looks at.
    ///
    /// **Fitted in inverse depth, not in depth.** A plane in the world is
    /// `a*u + b*v + c = 1/Z` exactly, for any orientation, because the
    /// projection divides by Z. Fitting `Z` itself is only a plane for a wall
    /// square on to the camera, and on a floor -- which runs from 1 m at the
    /// bottom of the frame to 6 m at the top -- it is a hyperbola, so the fit
    /// would bend the floor rather than flatten it.
    ///
    /// Nine summed-area tables, so the fit is a constant six multiplies per
    /// pixel whatever the kernel is. Least squares, which means a thorn drags
    /// the plane: chain a `median` in front of it where that matters.
    Plane(usize, f32),
    /// Blank a pixel whose surface is too steeply tilted away from the camera.
    /// A matcher smears every depth edge into a ramp, so the scene looks
    /// wrapped in plastic; the ramps are probably wrong and are better unknown.
    ///
    /// The threshold is the **tilt**, not a depth gradient: how far the depth
    /// moves from one pixel to the next, in units of that pixel's own lateral
    /// footprint `Z/fx`. 1 is a surface at 45 degrees, 10 is 84 degrees. That
    /// is the scale-free form -- a raw `dZ/du` threshold would mean a different
    /// angle at 1 m than at 6, and a different one again at another downscale.
    ///
    /// **It cannot tell a smeared edge from a real one**, because a real depth
    /// discontinuity has an unbounded gradient too. It removes both, which is
    /// the right trade: a ramp the matcher invented across a doorway is a wall
    /// of phantom points that the raytracer will clear real obstacles through,
    /// and a genuine edge is a handful of pixels.
    Steep(f32),
    /// Fill a hole with the median of its finite neighbours, where it has
    /// enough of them. The one filter that ANSWERS MORE: a hole is a pixel the
    /// map learns nothing from, so filling it with an honest neighbour is a
    /// gain, and filling it with a thorn is not -- chain a `median` in front.
    Fill(usize),
}

/// One or more filters, applied in order.
///
/// `median:2+fill:2` is a different thing from either alone: the median takes
/// the thorns off, and the fill then has honest neighbours to fill a hole from.
/// Doing it the other way round fills from the thorns.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Chain(pub Vec<Denoise>);

impl Chain {
    pub fn parse(text: &str) -> Result<Self, String> {
        if text.is_empty() || text == "none" {
            return Ok(Chain(Vec::new()));
        }
        text.split('+')
            .map(Denoise::parse)
            .collect::<Result<_, _>>()
            .map(Chain)
    }

    /// `fx` is the rectified focal length in pixels. Only `steep` reads it --
    /// it is the only filter whose threshold is an angle rather than a length
    /// -- but it is passed to all of them rather than smuggled in through a
    /// field, so a filter cannot quietly depend on a camera it was not given.
    pub fn apply(&self, depth: &[f32], width: usize, height: usize, fx: f32) -> Vec<f32> {
        let mut held = depth.to_vec();
        for step in &self.0 {
            held = step.apply(&held, width, height, fx);
        }
        held
    }

    pub fn name(&self) -> String {
        if self.0.is_empty() {
            return "none".to_string();
        }
        self.0
            .iter()
            .map(|d| format!("{d:?}"))
            .collect::<Vec<_>>()
            .join("+")
    }
}

impl Denoise {
    /// `median:2`, `mean:8`, `plane:8:0.7`, `steep:4`, `speckle:0.1`,
    /// `bilateral:0.05`, `coarse:2`, `fill:2`.
    pub fn parse(text: &str) -> Result<Self, String> {
        let (name, value) = text.split_once(':').unwrap_or((text, ""));
        let number = |fallback: f64| -> Result<f64, String> {
            if value.is_empty() {
                Ok(fallback)
            } else {
                value
                    .parse()
                    .map_err(|_| format!("denoise={text:?}: {value:?} is not a number"))
            }
        };
        Ok(match name {
            "none" | "" => Denoise::None,
            "median" => Denoise::Median(number(1.0)? as usize),
            "speckle" => Denoise::Speckle(number(0.10)? as f32),
            "bilateral" => Denoise::Bilateral(number(0.05)? as f32),
            "coarse" => Denoise::Coarse(number(2.0)? as usize),
            "mean" => Denoise::Mean(number(2.0)? as usize),
            "steep" => Denoise::Steep(number(4.0)? as f32),
            // The one filter with two knobs: `plane:8` or `plane:8:0.7`.
            "plane" => {
                let (radius, weight) = match value.split_once(':') {
                    None => (number(8.0)?, 1.0),
                    Some((r, w)) => (
                        r.parse()
                            .map_err(|_| format!("denoise={text:?}: {r:?} is not a radius"))?,
                        w.parse()
                            .map_err(|_| format!("denoise={text:?}: {w:?} is not a weight"))?,
                    ),
                };
                Denoise::Plane(radius as usize, weight as f32)
            }
            "fill" => Denoise::Fill(number(2.0)? as usize),
            other => {
                return Err(format!(
                    "denoise={other:?} is not none, median, mean, plane, steep, \
                     speckle, bilateral, coarse or fill"
                ))
            }
        })
    }

    pub fn apply(self, depth: &[f32], width: usize, height: usize, fx: f32) -> Vec<f32> {
        match self {
            Denoise::None => depth.to_vec(),
            Denoise::Median(radius) => median(depth, width, height, radius.max(1)),
            Denoise::Speckle(tolerance) => speckle(depth, width, height, 1, tolerance),
            Denoise::Bilateral(sigma) => bilateral(depth, width, height, 2, sigma),
            Denoise::Coarse(factor) => coarse(depth, width, height, factor.max(2)),
            Denoise::Mean(radius) => mean(depth, width, height, radius.max(1)),
            Denoise::Steep(tilt) => steep(depth, width, height, fx, tilt),
            Denoise::Plane(radius, weight) => plane(depth, width, height, radius.max(1), weight),
            Denoise::Fill(radius) => fill(depth, width, height, radius.max(1)),
        }
    }
}

/// The finite depths in a `(2r+1)^2` window, and their median.
fn window_median(
    depth: &[f32],
    width: usize,
    height: usize,
    row: usize,
    column: usize,
    radius: usize,
    held: &mut Vec<f32>,
) -> Option<f32> {
    held.clear();
    let top = row.saturating_sub(radius);
    let left = column.saturating_sub(radius);
    for r in top..=(row + radius).min(height - 1) {
        for c in left..=(column + radius).min(width - 1) {
            let z = depth[r * width + c];
            if z.is_finite() {
                held.push(z);
            }
        }
    }
    if held.is_empty() {
        return None;
    }
    let middle = held.len() / 2;
    held.select_nth_unstable_by(middle, |a, b| a.partial_cmp(b).unwrap());
    Some(held[middle])
}

/// A window median in constant time per pixel, whatever the radius.
///
/// The naive form gathers the `(2r+1)^2` neighbours of every pixel and selects
/// the middle one, which for the 17x17 window the shipped chain uses is 289
/// values per pixel, 180k pixels a frame -- the single most expensive thing in
/// the pipeline, and on an Orin more than the whole 30 fps budget. This is
/// Huang's sliding window instead: the window's contents live in a histogram,
/// and moving one column right adds a column of `2r+1` values and removes one,
/// so the cost per pixel is `2(2r+1)` increments plus a short walk to the
/// median, independent of the window's area.
///
/// The histogram is over depth in MILLIMETRES, so a median comes back rounded
/// to the millimetre. That is three orders of magnitude under the centimetres
/// of noise this filter exists to take off, and it is what makes the histogram
/// finite. Two levels -- 256 coarse bins of 256 mm, 65536 fine -- keep the walk
/// to the median under 512 steps.
struct SlidingMedian {
    coarse: [u32; 256],
    fine: Vec<u32>,
    count: u32,
}

/// Depth in millimetres, as a fine bin. Beyond 65 m is clamped rather than
/// dropped: it still counts as an answer, just a far one.
fn millimetre_bin(z: f32) -> usize {
    (z * 1000.0).round().clamp(0.0, 65535.0) as usize
}

impl SlidingMedian {
    fn new() -> Self {
        Self {
            coarse: [0; 256],
            fine: vec![0; 65536],
            count: 0,
        }
    }

    fn add(&mut self, z: f32) {
        if z.is_finite() {
            let bin = millimetre_bin(z);
            self.coarse[bin >> 8] += 1;
            self.fine[bin] += 1;
            self.count += 1;
        }
    }

    fn remove(&mut self, z: f32) {
        if z.is_finite() {
            let bin = millimetre_bin(z);
            self.coarse[bin >> 8] -= 1;
            self.fine[bin] -= 1;
            self.count -= 1;
        }
    }

    /// The same element `select_nth(len / 2)` picks: the upper median.
    fn median(&self) -> Option<f32> {
        if self.count == 0 {
            return None;
        }
        let target = self.count / 2;
        let mut seen = 0u32;
        for (block, &n) in self.coarse.iter().enumerate() {
            if seen + n > target {
                for bin in block << 8..(block + 1) << 8 {
                    seen += self.fine[bin];
                    if seen > target {
                        return Some(bin as f32 / 1000.0);
                    }
                }
            }
            seen += n;
        }
        None
    }

    /// Run the window along one row, calling `at(column, count, median)` for
    /// every column. The histogram is empty again on return, so one instance
    /// serves every row a thread is handed without being cleared.
    fn sweep(
        &mut self,
        depth: &[f32],
        width: usize,
        height: usize,
        row: usize,
        radius: usize,
        mut at: impl FnMut(usize, u32, Option<f32>),
    ) {
        let top = row.saturating_sub(radius);
        let bottom = (row + radius).min(height - 1);
        let column_of = |c: usize| (top..=bottom).map(move |r| r * width + c);
        for c in 0..=radius.min(width - 1) {
            column_of(c).for_each(|i| self.add(depth[i]));
        }
        for column in 0..width {
            at(column, self.count, self.median());
            if column + radius + 1 < width {
                column_of(column + radius + 1).for_each(|i| self.add(depth[i]));
            }
            if column >= radius {
                column_of(column - radius).for_each(|i| self.remove(depth[i]));
            }
        }
    }
}

fn median(depth: &[f32], width: usize, height: usize, radius: usize) -> Vec<f32> {
    let mut out = depth.to_vec();
    out.par_chunks_mut(width).enumerate().for_each_init(
        SlidingMedian::new,
        |window, (row, line)| {
            window.sweep(depth, width, height, row, radius, |column, _, m| {
                // Only where the matcher answered: a median must not invent
                // depth in a hole, or the pixel count is no longer the thing
                // it was.
                if let (true, Some(m)) = (line[column].is_finite(), m) {
                    line[column] = m;
                }
            });
        },
    );
    out
}

fn speckle(depth: &[f32], width: usize, height: usize, radius: usize, tolerance: f32) -> Vec<f32> {
    let mut out = depth.to_vec();
    out.par_chunks_mut(width)
        .enumerate()
        .for_each(|(row, line)| {
            let mut held = Vec::with_capacity((2 * radius + 1).pow(2));
            for (column, slot) in line.iter_mut().enumerate() {
                let z = *slot;
                if !z.is_finite() {
                    continue;
                }
                match window_median(depth, width, height, row, column, radius, &mut held) {
                    Some(m) if (z - m).abs() > tolerance => *slot = f32::NAN,
                    _ => {}
                }
            }
        });
    out
}

fn bilateral(depth: &[f32], width: usize, height: usize, radius: usize, sigma: f32) -> Vec<f32> {
    let mut out = depth.to_vec();
    let space = (radius as f32).max(1.0);
    out.par_chunks_mut(width)
        .enumerate()
        .for_each(|(row, line)| {
            for (column, slot) in line.iter_mut().enumerate() {
                let here = *slot;
                if !here.is_finite() {
                    continue;
                }
                let (mut sum, mut weight) = (0.0f32, 0.0f32);
                let top = row.saturating_sub(radius);
                let left = column.saturating_sub(radius);
                for r in top..=(row + radius).min(height - 1) {
                    for c in left..=(column + radius).min(width - 1) {
                        let z = depth[r * width + c];
                        if !z.is_finite() {
                            continue;
                        }
                        let dr = r as f32 - row as f32;
                        let dc = c as f32 - column as f32;
                        let near = (-(dr * dr + dc * dc) / (2.0 * space * space)).exp();
                        // The depth term is what keeps an edge an edge: a neighbour
                        // on the far side of a discontinuity gets no vote.
                        let like = (-((z - here) * (z - here)) / (2.0 * sigma * sigma)).exp();
                        sum += z * near * like;
                        weight += near * like;
                    }
                }
                if weight > 0.0 {
                    *slot = sum / weight;
                }
            }
        });
    out
}

fn coarse(depth: &[f32], width: usize, height: usize, factor: usize) -> Vec<f32> {
    let (small_w, small_h) = (width.div_ceil(factor), height.div_ceil(factor));
    let mut small = vec![f32::NAN; small_w * small_h];
    let mut held = Vec::with_capacity(factor * factor);
    for row in 0..small_h {
        for column in 0..small_w {
            held.clear();
            for r in row * factor..((row + 1) * factor).min(height) {
                for c in column * factor..((column + 1) * factor).min(width) {
                    let z = depth[r * width + c];
                    if z.is_finite() {
                        held.push(z);
                    }
                }
            }
            if held.is_empty() {
                continue;
            }
            let middle = held.len() / 2;
            held.select_nth_unstable_by(middle, |a, b| a.partial_cmp(b).unwrap());
            small[row * small_w + column] = held[middle];
        }
    }
    // Back up, nearest-neighbour, and ONLY where the matcher answered. Bilinear
    // across a hole's edge would invent depth at a discontinuity, and filling
    // holes is `fill`'s job, where it can be measured on its own.
    let mut out = depth.to_vec();
    for row in 0..height {
        for column in 0..width {
            if !depth[row * width + column].is_finite() {
                continue;
            }
            let z = small[(row / factor) * small_w + column / factor];
            if z.is_finite() {
                out[row * width + column] = z;
            }
        }
    }
    out
}

/// The box average over the finite pixels of a `(2r+1)^2` window.
///
/// Two summed-area tables -- one of the depths, one of how many of them were
/// finite -- so every window is four lookups whatever its size. Done the naive
/// way, `mean:24` would read 2401 pixels for each of 184,320.
fn mean(depth: &[f32], width: usize, height: usize, radius: usize) -> Vec<f32> {
    // (width+1) x (height+1) so the corner arithmetic needs no first-row case.
    let stride = width + 1;
    let mut sums = vec![0.0f64; stride * (height + 1)];
    let mut counts = vec![0u32; stride * (height + 1)];
    for row in 0..height {
        for column in 0..width {
            let z = depth[row * width + column];
            let (value, one) = if z.is_finite() {
                (z as f64, 1)
            } else {
                (0.0, 0)
            };
            let here = (row + 1) * stride + column + 1;
            sums[here] = value + sums[here - 1] + sums[here - stride] - sums[here - stride - 1];
            counts[here] =
                one + counts[here - 1] + counts[here - stride] - counts[here - stride - 1];
        }
    }
    let mut out = depth.to_vec();
    out.par_chunks_mut(width)
        .enumerate()
        .for_each(|(row, line)| {
            let top = row.saturating_sub(radius);
            let bottom = (row + radius + 1).min(height);
            for (column, slot) in line.iter_mut().enumerate() {
                // Only where the matcher answered, like `median`: averaging into a
                // hole would invent depth, and filling holes is `fill`'s job.
                if !slot.is_finite() {
                    continue;
                }
                let left = column.saturating_sub(radius);
                let right = (column + radius + 1).min(width);
                let corners = |table: &dyn Fn(usize) -> f64| {
                    table(bottom * stride + right)
                        - table(top * stride + right)
                        - table(bottom * stride + left)
                        + table(top * stride + left)
                };
                let total = corners(&|i| sums[i]);
                let count = corners(&|i| counts[i] as f64);
                if count > 0.0 {
                    *slot = (total / count) as f32;
                }
            }
        });
    out
}

/// Least squares `a*u + b*v + c = 1/Z` over the window, then pull towards it.
///
/// Nine summed-area tables -- `n`, `u`, `v`, `uu`, `uv`, `vv`, `d`, `ud`, `vd`
/// -- give every one of the fit's sums in four lookups, so a 33x33 kernel costs
/// what a 3x3 does. The normal equations are then centred on the window's own
/// mean before they are solved: in raw pixel coordinates `uu` reaches 2e5 and
/// the 3x3 system is badly enough conditioned to matter in f64.
fn plane(depth: &[f32], width: usize, height: usize, radius: usize, weight: f32) -> Vec<f32> {
    let stride = width + 1;
    let area = stride * (height + 1);
    // In the order the solve reads them.
    let mut tables = vec![vec![0.0f64; area]; 9];
    for row in 0..height {
        for column in 0..width {
            let z = depth[row * width + column];
            let (u, v) = (column as f64, row as f64);
            // Inverse depth, and zero where the matcher gave up, so a hole
            // contributes nothing to any of the nine sums.
            let (d, n) = if z.is_finite() && z > 0.0 {
                (1.0 / z as f64, 1.0)
            } else {
                (0.0, 0.0)
            };
            let terms = [
                n,
                u * n,
                v * n,
                u * u * n,
                u * v * n,
                v * v * n,
                d,
                u * d,
                v * d,
            ];
            let here = (row + 1) * stride + column + 1;
            for (table, term) in tables.iter_mut().zip(terms) {
                table[here] =
                    term + table[here - 1] + table[here - stride] - table[here - stride - 1];
            }
        }
    }

    let mut out = depth.to_vec();
    out.par_chunks_mut(width)
        .enumerate()
        .for_each(|(row, line)| {
            let top = row.saturating_sub(radius);
            let bottom = (row + radius + 1).min(height);
            for (column, slot) in line.iter_mut().enumerate() {
                let z = *slot;
                if !z.is_finite() || z <= 0.0 {
                    continue;
                }
                let left = column.saturating_sub(radius);
                let right = (column + radius + 1).min(width);
                let mut sums = [0.0f64; 9];
                for (sum, table) in sums.iter_mut().zip(tables.iter()) {
                    *sum = table[bottom * stride + right]
                        - table[top * stride + right]
                        - table[bottom * stride + left]
                        + table[top * stride + left];
                }
                let [n, su, sv, suu, suv, svv, sd, sud, svd] = sums;
                if n < 3.0 {
                    continue;
                }
                let mean_d = sd / n;
                // Centred, so the constant term drops out of the 2x2 system.
                let cuu = suu - su * su / n;
                let cuv = suv - su * sv / n;
                let cvv = svv - sv * sv / n;
                let cud = sud - su * sd / n;
                let cvd = svd - sv * sd / n;
                let det = cuu * cvv - cuv * cuv;
                // Degenerate -- a window that is one row, one column, or a handful
                // of collinear survivors. The plane through it is the mean, which
                // is what a zero gradient gives, so there is nothing to special
                // case beyond not dividing by it.
                let (a, b) = if det.abs() < 1e-9 * (cuu * cvv).abs().max(1.0) {
                    (0.0, 0.0)
                } else {
                    ((cud * cvv - cvd * cuv) / det, (cvd * cuu - cud * cuv) / det)
                };
                let fitted = mean_d + a * (column as f64 - su / n) + b * (row as f64 - sv / n);
                let here = 1.0 / z as f64;
                let pulled = here + weight as f64 * (fitted - here);
                if pulled > 0.0 {
                    *slot = (1.0 / pulled) as f32;
                }
            }
        });
    out
}

/// Blank every pixel tilted further from the camera than `tilt` allows.
///
/// The gradient is a central difference where both neighbours answered and
/// one-sided where only one did, so an edge beside a hole is still measured
/// rather than silently passed. An axis with no finite neighbour at all
/// contributes nothing: a lone pixel is not steep, it is unsupported, and
/// `speckle` is the filter for that.
fn steep(depth: &[f32], width: usize, height: usize, fx: f32, tilt: f32) -> Vec<f32> {
    let mut out = depth.to_vec();
    out.par_chunks_mut(width)
        .enumerate()
        .for_each(|(row, line)| {
            for (column, slot) in line.iter_mut().enumerate() {
                let z = *slot;
                if !z.is_finite() || z <= 0.0 {
                    continue;
                }
                let at = |r: usize, c: usize| -> Option<f32> {
                    let v = depth[r * width + c];
                    (v.is_finite() && v > 0.0).then_some(v)
                };
                let difference = |low: Option<f32>, high: Option<f32>| match (low, high) {
                    (Some(a), Some(b)) => (b - a) / 2.0,
                    (Some(a), None) => z - a,
                    (None, Some(b)) => b - z,
                    (None, None) => 0.0,
                };
                let across = difference(
                    (column > 0).then(|| at(row, column - 1)).flatten(),
                    (column + 1 < width).then(|| at(row, column + 1)).flatten(),
                );
                let down = difference(
                    (row > 0).then(|| at(row - 1, column)).flatten(),
                    (row + 1 < height).then(|| at(row + 1, column)).flatten(),
                );
                // `z / fx` is what one pixel spans at this range, so the ratio is
                // the surface's tangent -- the same number at 1 m and at 6.
                let footprint = z / fx;
                if (across * across + down * down).sqrt() > tilt * footprint {
                    *slot = f32::NAN;
                }
            }
        });
    out
}

fn fill(depth: &[f32], width: usize, height: usize, radius: usize) -> Vec<f32> {
    let mut out = depth.to_vec();
    let enough = ((2 * radius + 1).pow(2) / 2) as u32;
    out.par_chunks_mut(width).enumerate().for_each_init(
        SlidingMedian::new,
        |window, (row, line)| {
            window.sweep(depth, width, height, row, radius, |column, count, m| {
                // Only where the neighbourhood is mostly answered: a lone
                // reading beside a big hole is not evidence about the hole.
                if let (false, true, Some(m)) = (line[column].is_finite(), count >= enough, m) {
                    line[column] = m;
                }
            });
        },
    );
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The histogram median against the gather-and-select one it replaced, on
    /// a picture with holes and at every border, for the shipped radius and a
    /// window wider than the picture. Agreement is to the millimetre the
    /// histogram rounds to.
    #[test]
    fn the_sliding_median_matches_the_naive_one_including_holes_and_borders() {
        let (w, h) = (23, 11);
        let mut depth: Vec<f32> = (0..w * h)
            .map(|i| 1.0 + ((i * 7919) % 97) as f32 * 0.037)
            .collect();
        for hole in [0, 5, w * 3 + 4, w * 7 + 20, w * h - 1] {
            depth[hole] = f32::NAN;
        }
        for radius in [1usize, 3, 8, 40] {
            let fast = median(&depth, w, h, radius);
            let mut held = Vec::new();
            for row in 0..h {
                for column in 0..w {
                    let want = if depth[row * w + column].is_finite() {
                        window_median(&depth, w, h, row, column, radius, &mut held).unwrap()
                    } else {
                        f32::NAN
                    };
                    let got = fast[row * w + column];
                    assert_eq!(got.is_nan(), want.is_nan(), "r={radius} at {row},{column}");
                    if want.is_finite() {
                        assert!(
                            (got - want).abs() <= 0.0005 + 1e-6,
                            "r={radius} at {row},{column}: {got} vs {want}"
                        );
                    }
                }
            }
        }
    }

    /// `fill` reads the same window: a hole with a mostly answered
    /// neighbourhood takes its median, a hole beside a big hole stays one.
    #[test]
    fn the_sliding_fill_matches_the_naive_rule() {
        let (w, h) = (15, 9);
        let mut depth: Vec<f32> = (0..w * h).map(|i| 2.0 + (i % 5) as f32 * 0.01).collect();
        depth[4 * w + 7] = f32::NAN; // a lone hole: filled
        for row in 0..h {
            for column in 0..4 {
                depth[row * w + column] = f32::NAN; // a big hole: its edge is not
            }
        }
        let out = fill(&depth, w, h, 2);
        assert!(out[4 * w + 7].is_finite());
        assert!(out[4 * w + 1].is_nan(), "deep inside the big hole");
        let enough = (5 * 5 / 2) as usize;
        let mut held = Vec::new();
        for row in 0..h {
            for column in 0..w {
                if depth[row * w + column].is_finite() {
                    continue;
                }
                let naive = window_median(&depth, w, h, row, column, 2, &mut held);
                let want = match naive {
                    Some(m) if held.len() >= enough => m,
                    _ => f32::NAN,
                };
                let got = out[row * w + column];
                assert_eq!(got.is_nan(), want.is_nan(), "at {row},{column}");
                if want.is_finite() {
                    assert!((got - want).abs() <= 0.0005 + 1e-6, "at {row},{column}");
                }
            }
        }
    }

    /// The R1's rectified focal length at downscale 4, so a tilt threshold in
    /// these tests means what it means on the robot.
    const FX: f32 = 253.3;

    /// A flat wall at 3 m with one thorn, and one hole.
    fn wall() -> (Vec<f32>, usize, usize) {
        let (w, h) = (9, 9);
        let mut depth = vec![3.0f32; w * h];
        depth[4 * w + 4] = 5.0; // the thorn
        depth[2 * w + 2] = f32::NAN; // the hole
        (depth, w, h)
    }

    fn answered(depth: &[f32]) -> usize {
        depth.iter().filter(|z| z.is_finite()).count()
    }

    #[test]
    fn a_median_takes_the_thorn_off_and_keeps_the_pixel_count() {
        let (depth, w, h) = wall();
        let out = Denoise::Median(1).apply(&depth, w, h, FX);
        assert!((out[4 * w + 4] - 3.0).abs() < 1e-6, "{}", out[4 * w + 4]);
        assert_eq!(
            answered(&out),
            answered(&depth),
            "no pixel invented or lost"
        );
    }

    #[test]
    fn speckle_removes_the_thorn_rather_than_moving_it() {
        let (depth, w, h) = wall();
        let out = Denoise::Speckle(0.10).apply(&depth, w, h, FX);
        assert!(out[4 * w + 4].is_nan());
        assert_eq!(answered(&out), answered(&depth) - 1);
    }

    #[test]
    fn a_bilateral_keeps_an_edge() {
        // Half at 3 m, half at 6 m. A plain blur would put the seam at 4.5.
        let (w, h) = (9, 9);
        let mut depth = vec![3.0f32; w * h];
        for row in 0..h {
            for column in 5..w {
                depth[row * w + column] = 6.0;
            }
        }
        let out = Denoise::Bilateral(0.05).apply(&depth, w, h, FX);
        assert!(
            (out[4 * w + 4] - 3.0).abs() < 1e-3,
            "near side: {}",
            out[4 * w + 4]
        );
        assert!(
            (out[4 * w + 5] - 6.0).abs() < 1e-3,
            "far side: {}",
            out[4 * w + 5]
        );
    }

    #[test]
    fn coarse_smooths_and_comes_back_to_the_same_grid() {
        let (depth, w, h) = wall();
        let out = Denoise::Coarse(3).apply(&depth, w, h, FX);
        assert_eq!(out.len(), depth.len());
        assert_eq!(answered(&out), answered(&depth));
        assert!((out[4 * w + 4] - 3.0).abs() < 1e-6);
    }

    #[test]
    fn fill_is_the_one_that_answers_more() {
        let (depth, w, h) = wall();
        let out = Denoise::Fill(2).apply(&depth, w, h, FX);
        assert_eq!(answered(&out), answered(&depth) + 1);
        assert!((out[2 * w + 2] - 3.0).abs() < 1e-6);
    }

    #[test]
    fn a_chain_applies_in_order() {
        let (depth, w, h) = wall();
        // median first, so the fill has honest neighbours; the thorn is gone
        // AND the hole is filled.
        let out = Chain::parse("median:1+fill:2")
            .unwrap()
            .apply(&depth, w, h, FX);
        assert!(
            (out[4 * w + 4] - 3.0).abs() < 1e-6,
            "the thorn: {}",
            out[4 * w + 4]
        );
        assert!(
            (out[2 * w + 2] - 3.0).abs() < 1e-6,
            "the hole: {}",
            out[2 * w + 2]
        );
        assert_eq!(answered(&out), answered(&depth) + 1);
    }

    /// The mean is not an order statistic: where a median leaves a surface's
    /// own noise alone, this divides it down. That is the whole reason to want
    /// a big patch, so it is worth a test rather than a comment.
    #[test]
    fn a_mean_divides_the_noise_a_median_leaves_alone() {
        // A wall at 3 m with alternating +-10 cm of noise and no thorn at all.
        let (w, h) = (9, 9);
        let mut depth = vec![3.0f32; w * h];
        for row in 0..h {
            for column in 0..w {
                depth[row * w + column] += if (row + column) % 2 == 0 { 0.10 } else { -0.10 };
            }
        }
        let middle = 4 * w + 4;
        let median = Denoise::Median(2).apply(&depth, w, h, FX);
        let mean = Denoise::Mean(2).apply(&depth, w, h, FX);
        assert!(
            (median[middle] - depth[middle]).abs() < 1e-6,
            "the median picks a neighbour"
        );
        assert!(
            (mean[middle] - 3.0).abs() < 0.01,
            "the mean averages: {}",
            mean[middle]
        );
        assert_eq!(
            answered(&mean),
            answered(&depth),
            "no pixel invented or lost"
        );
    }

    /// The summed-area table is the whole reason a 49x49 window is affordable,
    /// and it is also the easiest thing here to get subtly wrong at an edge.
    #[test]
    fn the_summed_area_mean_matches_the_naive_one_including_at_the_border() {
        let (w, h) = (17, 13);
        let mut depth: Vec<f32> = (0..w * h).map(|i| 2.0 + (i % 7) as f32 * 0.1).collect();
        depth[0] = f32::NAN;
        depth[5 * w + 9] = f32::NAN;
        for radius in [1usize, 3, 6, 40] {
            let fast = Denoise::Mean(radius).apply(&depth, w, h, FX);
            for row in 0..h {
                for column in 0..w {
                    if !depth[row * w + column].is_finite() {
                        assert!(fast[row * w + column].is_nan());
                        continue;
                    }
                    let (mut sum, mut count) = (0.0f64, 0usize);
                    for r in row.saturating_sub(radius)..=(row + radius).min(h - 1) {
                        for c in column.saturating_sub(radius)..=(column + radius).min(w - 1) {
                            let z = depth[r * w + c];
                            if z.is_finite() {
                                sum += z as f64;
                                count += 1;
                            }
                        }
                    }
                    let want = (sum / count as f64) as f32;
                    let got = fast[row * w + column];
                    assert!(
                        (got - want).abs() < 1e-4,
                        "r={radius} at {row},{column}: {got} vs {want}"
                    );
                }
            }
        }
    }

    /// The reason the fit is in inverse depth. A real floor's depth runs from
    /// 1 m at the bottom of the frame to 6 at the top, and that is a hyperbola
    /// in `Z` -- a filter that fits `Z` flat would bend it. In `1/Z` it is
    /// exactly a plane, so the filter should leave it alone.
    #[test]
    fn a_slanted_surface_survives_the_plane_fit_where_a_mean_bends_it() {
        let (w, h) = (21, 21);
        // 1/Z from 1/1.0 at the top row to 1/6.0 at the bottom: a floor.
        let inverse = |row: usize| 1.0 - (1.0 - 1.0 / 6.0) * row as f32 / (h - 1) as f32;
        let depth: Vec<f32> = (0..w * h).map(|i| 1.0 / inverse(i / w)).collect();
        let fitted = Denoise::Plane(5, 1.0).apply(&depth, w, h, FX);
        let averaged = Denoise::Mean(5).apply(&depth, w, h, FX);
        let middle = 10 * w + 10;
        assert!(
            (fitted[middle] - depth[middle]).abs() < 1e-3,
            "the plane fit moved a plane: {} vs {}",
            fitted[middle],
            depth[middle]
        );
        assert!(
            (averaged[middle] - depth[middle]).abs() > 1e-2,
            "a mean should bend it, so this test would not prove anything"
        );
    }

    /// What it is for: noise on a surface simple enough to have a plane.
    #[test]
    fn the_plane_fit_takes_the_noise_off_a_slanted_surface() {
        let (w, h) = (21, 21);
        let inverse = |row: usize| 1.0 - (1.0 - 1.0 / 6.0) * row as f32 / (h - 1) as f32;
        let clean: Vec<f32> = (0..w * h).map(|i| 1.0 / inverse(i / w)).collect();
        // Deterministic, zero-mean, alternating in both axes.
        let noisy: Vec<f32> = clean
            .iter()
            .enumerate()
            .map(|(i, z)| {
                z + if (i / w + i % w) % 2 == 0 {
                    0.05
                } else {
                    -0.05
                }
            })
            .collect();
        let fitted = Denoise::Plane(5, 1.0).apply(&noisy, w, h, FX);
        let before: f32 = noisy
            .iter()
            .zip(&clean)
            .map(|(a, b)| (a - b) * (a - b))
            .sum();
        let after: f32 = fitted
            .iter()
            .zip(&clean)
            .map(|(a, b)| (a - b) * (a - b))
            .sum();
        assert!(after < before / 4.0, "{after} against {before}");
        assert_eq!(answered(&fitted), answered(&noisy));
    }

    /// The weight is the "how aggressively": 0 changes nothing, 1 lands on the
    /// plane, and between them it is a straight line in inverse depth.
    #[test]
    fn the_plane_weight_is_how_far_it_pulls() {
        let (depth, w, h) = wall();
        let full = Denoise::Plane(3, 1.0).apply(&depth, w, h, FX);
        let half = Denoise::Plane(3, 0.5).apply(&depth, w, h, FX);
        let none = Denoise::Plane(3, 0.0).apply(&depth, w, h, FX);
        let thorn = 4 * w + 4;
        assert!((none[thorn] - 5.0).abs() < 1e-5, "weight 0 is the identity");
        let want = 1.0 / (0.5 * (1.0 / 5.0) + 0.5 * (1.0 / full[thorn]));
        assert!(
            (half[thorn] - want).abs() < 1e-3,
            "{} vs {want}",
            half[thorn]
        );
        assert!(full[thorn] < 5.0, "the thorn came down: {}", full[thorn]);
    }

    /// The threshold is an angle, so the same surface must read the same at
    /// any range -- that is the whole reason it is divided by `Z/fx`.
    #[test]
    fn the_steep_threshold_is_an_angle_not_a_gradient() {
        let (w, h) = (9, 9);
        // A 45 degree surface: depth grows by one pixel footprint per pixel.
        // Built by marching, so the step at each column is that column's own
        // footprint rather than the first one's.
        let ramp = |start: f32| -> Vec<f32> {
            let mut row = vec![start; w];
            for column in 1..w {
                row[column] = row[column - 1] * (1.0 + 1.0 / FX);
            }
            (0..w * h).map(|i| row[i % w]).collect::<Vec<f32>>()
        };
        for start in [1.0f32, 6.0] {
            let depth = ramp(start);
            let kept = Denoise::Steep(2.0).apply(&depth, w, h, FX);
            let cut = Denoise::Steep(0.5).apply(&depth, w, h, FX);
            assert_eq!(
                answered(&kept),
                answered(&depth),
                "45 deg survives at {start} m"
            );
            assert!(cut[4 * w + 4].is_nan(), "and goes at 0.5 at {start} m");
        }
    }

    /// What it is for: the ramp a matcher invents across a depth
    /// discontinuity, which every smoothing filter here makes longer.
    #[test]
    fn steep_removes_the_bridge_across_a_jump_and_leaves_the_walls() {
        let (w, h) = (11, 5);
        // 2 m on the left, 5 m on the right, joined by a three-pixel ramp.
        let profile: Vec<f32> = vec![2.0, 2.0, 2.0, 2.0, 2.75, 3.5, 4.25, 5.0, 5.0, 5.0, 5.0];
        let depth: Vec<f32> = (0..w * h).map(|i| profile[i % w]).collect();
        let out = Denoise::Steep(4.0).apply(&depth, w, h, FX);
        for column in [0usize, 1, 9, 10] {
            assert!(out[2 * w + column].is_finite(), "flat at {column} survived");
        }
        for column in [4usize, 5, 6] {
            assert!(out[2 * w + column].is_nan(), "the bridge at {column} went");
        }
    }

    #[test]
    fn none_is_the_identity() {
        let (depth, w, h) = wall();
        let out = Denoise::None.apply(&depth, w, h, FX);
        assert_eq!(out.len(), depth.len());
        for (a, b) in out.iter().zip(depth.iter()) {
            assert_eq!(a.is_nan(), b.is_nan());
            if a.is_finite() {
                assert_eq!(a, b);
            }
        }
    }
}
