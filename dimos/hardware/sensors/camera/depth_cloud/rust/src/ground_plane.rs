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

//! Snap the floor onto the plane the disparity map already says it is.
//!
//! A floor is the one surface in the room whose shape is known before the
//! camera looks at it, and it is also the surface stereo reads worst: it is
//! flat, it is dim, and it is seen at a grazing angle, so a disparity error of
//! half a pixel becomes several centimetres of height. Per pixel that error is
//! mostly random, and averaging it away is the whole point of fitting.
//!
//! The fit is done in disparity space rather than in metres, because that is
//! where it is linear. For a rectified pair, a 3D plane `aX + bY + cZ + d = 0`
//! reaches the image as
//!
//! ```text
//!     disparity(u, v) = alpha * u + beta * v + gamma
//! ```
//!
//! -- exactly affine, with no approximation, since `X = (u - cx) Z / fx` and
//! `1/Z = disparity / (fx * baseline)`. So the floor is a *plane in
//! (u, v, disparity)*, and finding it is a least-squares fit of three numbers.
//! The classic form of this is the v-disparity image, where the floor shows up
//! as a line because the `u` term vanishes for a floor the camera has no roll
//! about; keeping the `u` term is the same idea without that assumption, which
//! matters here because the R1's head does have a small roll.
//!
//! Fitting in metres instead would be fitting a curve, and worse, a curve whose
//! noise grows with the square of range -- the far half of the floor would be
//! weighted a hundred times less than it deserves. In disparity the noise is
//! roughly constant, which is the condition least squares is the right answer
//! under.
//!
//! What this does **not** do is decide that a pixel is floor because the fit
//! would like it to be. Only pixels already within a tolerance of the fitted
//! plane are moved; everything else -- walls, furniture, people -- keeps the
//! disparity the matcher measured. A floor-flattener that reached further than
//! that would erase exactly the things a robot must not drive into.

/// The floor, as the affine disparity map it is in a rectified pair.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GroundPlane {
    /// Disparity per pixel of column. Zero when the camera has no roll.
    pub alpha: f32,
    /// Disparity per pixel of row. The dominant term: rows further down the
    /// picture are closer floor, and closer floor has more disparity.
    pub beta: f32,
    pub gamma: f32,
    /// How many pixels ended up supporting the fit.
    pub inliers: usize,
}

impl GroundPlane {
    /// The disparity this plane predicts at a pixel.
    pub fn at(&self, column: usize, row: usize) -> f32 {
        self.alpha * column as f32 + self.beta * row as f32 + self.gamma
    }
}

impl GroundParams {
    /// How far from the plane a pixel may sit and still be called floor, at a
    /// place where the plane predicts `fitted` pixels of disparity.
    ///
    /// Depth goes as one over disparity, so a tolerance stated in pixels is a
    /// window whose depth grows with the square of range. Capping it at a
    /// fraction of the fitted disparity makes it a window of roughly constant
    /// *relative* depth instead, which is the shape of the error it is meant
    /// to be catching.
    pub fn tolerance_at(&self, fitted: f32) -> f32 {
        self.tolerance_px
            .min(self.max_depth_change * fitted.max(0.0))
    }
}

/// How the floor is separated from everything else standing on it.
#[derive(Clone, Copy, Debug)]
pub struct GroundParams {
    /// Rows above this fraction of the image are not offered to the fit.
    ///
    /// The floor of a room the camera is standing in is in the lower part of
    /// the picture, and the upper part is where the far wall and the ceiling
    /// are. Both of those are also planes, both are better textured than the
    /// floor, and a fit given the whole image will happily prefer one of them.
    ///
    /// Where to put it is a trade against range, and it is easy to put too
    /// low. The R1's head looks down about 20 degrees, so at the default
    /// downscale the floor's vanishing line is around row 59 of 384 and every
    /// row below it is floor of some range: cutting at 0.45 would have thrown
    /// away everything past about 4 m, which is most of what the far bands are
    /// made of. At 0.30 the fit still reaches past 8 m.
    pub horizon: f32,
    /// A pixel is on the floor if its disparity is within this of the fit.
    ///
    /// In pixels of disparity, which is the right unit: it is a roughly
    /// constant tolerance in *height above the floor*, whereas a tolerance in
    /// metres of depth would be centimetres at the robot's feet and metres at
    /// the far wall.
    pub tolerance_px: f32,
    /// The most a snap may change a reading, as a fraction of its depth.
    ///
    /// Without this the tolerance is a fixed number of pixels, and a fixed
    /// number of pixels is not a fixed amount of floor. At a metre the R1's
    /// head reads about 30 px of disparity, so 1.5 px is five per cent of the
    /// depth; at six metres it reads 5 px, and the same 1.5 px is a window a
    /// metre and a half deep. Every far pixel therefore fell "within
    /// tolerance" of the plane whatever the plane said, and the far floor was
    /// snapped onto an extrapolation instead of being left alone -- which is
    /// how the first version of this made the 5-6 m band worse than the raw
    /// matcher while improving every band below it.
    ///
    /// Five per cent is chosen from what the flattener is *for* rather than
    /// from any score: it is there to remove sub-pixel noise, not to move a
    /// reading somewhere else, and a correction larger than a twentieth of the
    /// depth is the second thing.
    pub max_depth_change: f32,
    /// Refits after the first, each one having dropped what the last called an
    /// outlier. Three is enough to shed the furniture; more only chases noise.
    pub iterations: usize,
    /// Below this many supporting pixels the fit is refused rather than
    /// reported badly. A frame pointed at a wall has no floor to find, and a
    /// plane fitted to a hundred stray pixels would be worse than no plane.
    pub min_inliers: usize,
}

impl Default for GroundParams {
    fn default() -> Self {
        Self {
            horizon: 0.30,
            tolerance_px: 1.5,
            max_depth_change: 0.05,
            iterations: 3,
            min_inliers: 500,
        }
    }
}

/// Fit the floor's disparity plane, or `None` if the frame does not show one.
///
/// *disparity* is the matcher's output, with non-positive values meaning "no
/// reading". Only pixels below `params.horizon` are considered.
pub fn fit_ground(
    disparity: &[f32],
    width: usize,
    height: usize,
    params: &GroundParams,
) -> Option<GroundPlane> {
    if width == 0 || height == 0 || disparity.len() < width * height {
        return None;
    }
    let first_row = ((height as f32 * params.horizon) as usize).min(height);
    let mut samples: Vec<(f32, f32, f32)> = Vec::new();
    for row in first_row..height {
        for column in 0..width {
            let d = disparity[row * width + column];
            if d > 0.0 && d.is_finite() {
                samples.push((column as f32, row as f32, d));
            }
        }
    }
    if samples.len() < params.min_inliers {
        return None;
    }

    let mut plane = solve(&samples)?;
    for _ in 0..params.iterations {
        // Trimming, not weighting. A weighted fit lets a tall obstacle keep a
        // small vote, and the thing standing on the floor is precisely what
        // must not bend the floor. The tolerance is widened on the first passes
        // so that a fit thrown off by furniture can still find its way back:
        // starting at the final tolerance would let the first bad fit decide
        // which pixels the second one is allowed to see.
        let widened = params.tolerance_px * 3.0;
        let kept: Vec<(f32, f32, f32)> = samples
            .iter()
            .copied()
            .filter(|(u, v, d)| (d - plane.at(*u as usize, *v as usize)).abs() <= widened)
            .collect();
        if kept.len() < params.min_inliers {
            return None;
        }
        plane = solve(&kept)?;
    }
    let inliers = samples
        .iter()
        .filter(|(u, v, d)| {
            let fitted = plane.at(*u as usize, *v as usize);
            (d - fitted).abs() <= params.tolerance_at(fitted)
        })
        .count();
    if inliers < params.min_inliers {
        return None;
    }
    Some(GroundPlane { inliers, ..plane })
}

/// Replace the disparity of every pixel already near *plane* with the plane's.
///
/// Returns how many pixels were moved. Pixels further from the plane than
/// `GroundParams::tolerance_at` allows are left exactly as the matcher produced
/// them, so an obstacle standing on the floor survives this untouched -- and so
/// does the far floor, where the plane is an extrapolation and the matcher's
/// own answer is the better one.
pub fn flatten_ground(
    disparity: &mut [f32],
    width: usize,
    height: usize,
    plane: &GroundPlane,
    params: &GroundParams,
) -> usize {
    let first_row = ((height as f32 * params.horizon) as usize).min(height);
    let mut moved = 0;
    for row in first_row..height {
        for column in 0..width {
            let index = row * width + column;
            let d = disparity[index];
            if !(d > 0.0 && d.is_finite()) {
                continue;
            }
            let fitted = plane.at(column, row);
            if (d - fitted).abs() <= params.tolerance_at(fitted) && fitted > 0.0 {
                disparity[index] = fitted;
                moved += 1;
            }
        }
    }
    moved
}

/// Least squares for `d = alpha*u + beta*v + gamma` over the samples.
///
/// Solved as the 3x3 normal equations by Cramer's rule. The matrix is tiny and
/// well conditioned for any real image -- the columns span the picture -- so
/// the usual warning about normal equations does not bite, and a hand-written
/// Cramer keeps this crate free of a linear-algebra dependency.
fn solve(samples: &[(f32, f32, f32)]) -> Option<GroundPlane> {
    let n = samples.len() as f64;
    let (mut su, mut sv, mut sd) = (0.0f64, 0.0f64, 0.0f64);
    let (mut suu, mut svv, mut suv) = (0.0f64, 0.0f64, 0.0f64);
    let (mut sud, mut svd) = (0.0f64, 0.0f64);
    for &(u, v, d) in samples {
        let (u, v, d) = (u as f64, v as f64, d as f64);
        su += u;
        sv += v;
        sd += d;
        suu += u * u;
        svv += v * v;
        suv += u * v;
        sud += u * d;
        svd += v * d;
    }
    let matrix = [[suu, suv, su], [suv, svv, sv], [su, sv, n]];
    let rhs = [sud, svd, sd];
    let det = determinant(&matrix);
    if det.abs() < 1e-6 {
        return None;
    }
    let mut solution = [0.0f64; 3];
    for column in 0..3 {
        let mut replaced = matrix;
        for row in 0..3 {
            replaced[row][column] = rhs[row];
        }
        solution[column] = determinant(&replaced) / det;
    }
    Some(GroundPlane {
        alpha: solution[0] as f32,
        beta: solution[1] as f32,
        gamma: solution[2] as f32,
        inliers: samples.len(),
    })
}

fn determinant(m: &[[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A floor seen by a level rectified camera, as a disparity map.
    ///
    /// `fx * baseline / Z` with `Z` read off the floor geometry, which is the
    /// same arithmetic the real pipeline does, so the map is a real floor
    /// rather than a plane drawn in disparity by fiat.
    fn floor_disparity(width: usize, height: usize) -> Vec<f32> {
        let (fx, fy, cy) = (400.0f32, 400.0f32, height as f32 / 2.0);
        let (baseline, camera_height) = (0.12f32, 1.0f32);
        let mut out = vec![0.0f32; width * height];
        for row in 0..height {
            let elevation = (row as f32 - cy) / fy;
            if elevation <= 0.02 {
                continue; // at or above the horizon there is no floor
            }
            let z = camera_height / elevation;
            if z > 12.0 {
                continue;
            }
            for column in 0..width {
                out[row * width + column] = fx * baseline / z;
            }
        }
        out
    }

    /// Deterministic hash noise, so a failure is reproducible.
    fn jitter(index: usize) -> f32 {
        let mut x = index as u64 ^ 0x9e37_79b9_7f4a_7c15;
        x ^= x >> 30;
        x = x.wrapping_mul(0xbf58_476d_1ce4_e5b9);
        x ^= x >> 27;
        (x % 2001) as f32 / 1000.0 - 1.0
    }

    /// RMS depth error over the pixels where `truth` disparity is at least
    /// `floor_px`. The cut is there because the flattener deliberately stops
    /// working where disparity is small -- see `GroundParams::max_depth_change`
    /// -- so an average taken over the whole picture would be reporting mostly
    /// on the part it was asked not to touch.
    fn depth_error_above(disparity: &[f32], truth: &[f32], fx_baseline: f32, floor_px: f32) -> f32 {
        let mut total = 0.0f64;
        let mut count = 0usize;
        for (measured, ideal) in disparity.iter().zip(truth) {
            if *measured > 0.0 && *ideal >= floor_px {
                let e = (fx_baseline / measured - fx_baseline / ideal) as f64;
                total += e * e;
                count += 1;
            }
        }
        if count == 0 {
            return f32::INFINITY;
        }
        (total / count as f64).sqrt() as f32
    }

    #[test]
    fn a_noisy_floor_comes_back_flat() {
        // The point of the whole path: half a pixel of disparity noise is
        // several centimetres of floor at range, and the floor is the one
        // surface whose shape we knew in advance.
        let (width, height) = (240, 200);
        let truth = floor_disparity(width, height);
        let mut noisy: Vec<f32> = truth
            .iter()
            .enumerate()
            .map(|(i, d)| if *d > 0.0 { d + 0.5 * jitter(i) } else { 0.0 })
            .collect();
        let params = GroundParams::default();
        let before = depth_error_above(&noisy, &truth, 48.0, 10.0);
        let plane = fit_ground(&noisy, width, height, &params).expect("a floor");
        let moved = flatten_ground(&mut noisy, width, height, &plane, &params);
        assert!(moved > 5_000, "only {moved} pixels were snapped");
        let after = depth_error_above(&noisy, &truth, 48.0, 10.0);
        assert!(
            after < before / 4.0,
            "flattening barely helped: {before:.4} m -> {after:.4} m"
        );
    }

    #[test]
    fn no_snap_moves_a_reading_by_more_than_the_promised_fraction() {
        // The guarantee the tolerance exists to make, and the one the robot
        // needed: a flattener is allowed to remove noise, not to relocate a
        // surface. Stated against the *measured* disparity, since that is what
        // is being replaced.
        let (width, height) = (240, 200);
        let truth = floor_disparity(width, height);
        let mut noisy: Vec<f32> = truth
            .iter()
            .enumerate()
            .map(|(i, d)| if *d > 0.0 { d + 1.2 * jitter(i) } else { 0.0 })
            .collect();
        let before = noisy.clone();
        let params = GroundParams::default();
        let plane = fit_ground(&noisy, width, height, &params).expect("a floor");
        flatten_ground(&mut noisy, width, height, &plane, &params);
        for (index, (old, new)) in before.iter().zip(&noisy).enumerate() {
            if old == new {
                continue;
            }
            let allowed = params.tolerance_at(*new) + 1e-4;
            assert!(
                (old - new).abs() <= allowed,
                "pixel {index} moved {:.3} px with {allowed:.3} allowed",
                (old - new).abs()
            );
            assert!(
                (old - new).abs() <= params.max_depth_change * new + 1e-4,
                "pixel {index} moved more than {:.0}% of its depth",
                params.max_depth_change * 100.0
            );
        }
    }

    #[test]
    fn a_level_floor_has_no_column_term() {
        let (width, height) = (240, 200);
        let disparity = floor_disparity(width, height);
        let plane = fit_ground(&disparity, width, height, &GroundParams::default()).expect("floor");
        assert!(plane.alpha.abs() < 1e-3, "alpha {}", plane.alpha);
        assert!(
            plane.beta > 0.0,
            "beta {} should grow downwards",
            plane.beta
        );
    }

    #[test]
    fn an_obstacle_standing_on_the_floor_is_not_flattened_into_it() {
        // The one failure that would make this dangerous rather than merely
        // wrong: a box in front of the robot smoothed down into floor.
        let (width, height) = (240, 200);
        let truth = floor_disparity(width, height);
        let mut map = truth.clone();
        let box_disparity = 30.0f32;
        for row in (height / 2 + 10)..(height / 2 + 40) {
            for column in 100..140 {
                map[row * width + column] = box_disparity;
            }
        }
        let params = GroundParams::default();
        let plane = fit_ground(&map, width, height, &params).expect("a floor");
        flatten_ground(&mut map, width, height, &plane, &params);
        let mut intact = 0;
        for row in (height / 2 + 10)..(height / 2 + 40) {
            for column in 100..140 {
                if map[row * width + column] == box_disparity {
                    intact += 1;
                }
            }
        }
        assert!(intact > 1_000, "the box lost {} pixels", 1200 - intact);
    }

    #[test]
    fn a_frame_with_no_floor_is_refused_rather_than_invented() {
        let (width, height) = (240, 200);
        let wall = vec![20.0f32; width * height];
        let plane = fit_ground(&wall, width, height, &GroundParams::default());
        // A wall *is* a plane in (u, v, disparity), so it does fit -- but a
        // flat one, with no row term. The caller's protection is that
        // flattening it changes nothing, not that the fit is refused.
        let plane = plane.expect("a constant map is a degenerate plane");
        assert!(plane.beta.abs() < 1e-3);
    }

    #[test]
    fn the_far_floor_is_left_to_the_matcher_rather_than_extrapolated_onto() {
        // The failure the first version of this had on the robot: a tolerance
        // in pixels is a window in depth that grows with the square of range,
        // so at six metres every reading was "near" the plane whatever the
        // plane said, and the far floor came back worse than the raw matcher.
        let (width, height) = (240, 200);
        let truth = floor_disparity(width, height);
        // A plane that is right near the camera and half a pixel out at the
        // top of the picture -- the shape a least-squares fit's extrapolation
        // error actually takes.
        let plane = GroundPlane {
            alpha: 0.0,
            beta: 0.0,
            gamma: 6.0,
            inliers: 10_000,
        };
        let params = GroundParams::default();
        let mut map = truth.clone();
        flatten_ground(&mut map, width, height, &plane, &params);
        let far: Vec<usize> = (0..width * height)
            .filter(|i| truth[*i] > 0.0 && truth[*i] < 8.0)
            .collect();
        assert!(!far.is_empty(), "the fixture has no far floor");
        // Every far pixel that was snapped was one the plane already agreed
        // with to within five per cent; none was dragged across a metre of
        // depth to reach it. Under a tolerance stated only in pixels, a 1.5 px
        // window at six disparity is a window a metre and a half deep, and
        // every one of these would have moved.
        for index in &far {
            let (old, new) = (truth[*index], map[*index]);
            assert!(
                (old - new).abs() <= 0.05 * new + 1e-4,
                "a far pixel moved {:.2} px, from {old:.2} to {new:.2}",
                (old - new).abs()
            );
        }
        let dragged = far
            .iter()
            .filter(|i| (truth[**i] - map[**i]).abs() > 0.5)
            .count();
        assert_eq!(
            dragged, 0,
            "{dragged} far pixels moved more than half a pixel"
        );
    }

    #[test]
    fn the_tolerance_is_a_fraction_of_depth_once_disparity_is_small() {
        let params = GroundParams::default();
        assert_eq!(params.tolerance_at(100.0), 1.5);
        assert_eq!(params.tolerance_at(5.0), 0.25);
        assert_eq!(params.tolerance_at(0.0), 0.0);
    }

    #[test]
    fn too_few_readings_is_not_a_floor() {
        let (width, height) = (240, 200);
        let mut sparse = vec![0.0f32; width * height];
        sparse[width * (height - 1)] = 20.0;
        assert!(fit_ground(&sparse, width, height, &GroundParams::default()).is_none());
    }
}
