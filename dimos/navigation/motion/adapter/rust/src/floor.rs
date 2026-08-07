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

//! Where the floor is, so the planner's body band can sit on it -- the rust
//! twin of `adapter/floor.py`, which is the specification.
//!
//! The target planner slices the cloud at an ABSOLUTE z of 0.05..0.45 m, which
//! is only the body's band if the map's z origin is the ground. On a LIO stack
//! it is not: odometry starts at the sensor, so the origin sits at base height
//! and the band reads a slab over the robot's head-room. A constant trim
//! cannot fix that -- one voxel too much and the floor's own slab lands inside
//! the band and walls the robot in -- so the floor is estimated per tick,
//! sanity bounded against tf's base height above ground, and the slab itself
//! is dropped before the band is taken.
//!
//! BOTH SIDES OF THE STACK READ THE SAME SLICE: the planner plans on the
//! anchored cloud and the follower measures its room hint off it, through the
//! same [`anchored_cloud`].

use std::borrow::Cow;

/// Neighbourhood the floor is read from (m). Wide enough to hold ground in a
/// cluttered room, narrow enough that a ramp or a stair flight is still
/// locally planar -- the planner's own operating assumption.
pub const FLOOR_RADIUS_M: f64 = 2.5;
/// Low quantile of that neighbourhood: the ground, not what stands on it.
pub const FLOOR_PERCENTILE: f64 = 5.0;
/// Fewer returns than this is not a floor sample, it is noise.
pub const FLOOR_MIN_POINTS: usize = 100;
/// How far the estimate may sit from the tf prior before the prior wins.
pub const FLOOR_TOLERANCE_M: f64 = 0.35;

/// The floor's z under `xy`, or the tf prior when the cloud cannot say.
pub fn estimate_floor(points: &[[f32; 3]], xy: (f64, f64), prior: Option<f64>) -> Option<f64> {
    estimate_floor_with(
        points,
        xy,
        prior,
        FLOOR_RADIUS_M,
        FLOOR_PERCENTILE,
        FLOOR_MIN_POINTS,
        FLOOR_TOLERANCE_M,
    )
}

/// `estimate_floor` with every constant open, so the tests can pin them.
pub fn estimate_floor_with(
    points: &[[f32; 3]],
    xy: (f64, f64),
    prior: Option<f64>,
    radius: f64,
    percentile_pct: f64,
    min_points: usize,
    tolerance: f64,
) -> Option<f64> {
    let mut near: Vec<f64> = points
        .iter()
        .filter(|p| {
            let (dx, dy) = (p[0] as f64 - xy.0, p[1] as f64 - xy.1);
            dx.hypot(dy) < radius
        })
        .map(|p| p[2] as f64)
        .collect();
    if near.len() < min_points {
        return prior;
    }
    let floor = percentile(&mut near, percentile_pct);
    match prior {
        Some(p) if (floor - p).abs() > tolerance => Some(p),
        _ => Some(floor),
    }
}

/// Cloud re-zeroed on the floor, with the ground slab within `margin` dropped.
pub fn anchor_to_floor(points: &[[f32; 3]], floor: f64, margin: f64) -> Vec<[f32; 3]> {
    // f32 throughout, as the python does: the planner's SDF and the room hint
    // both see the shifted numbers, and widening first would disagree with
    // them in the last bits.
    let floor = floor as f32;
    let margin = margin as f32;
    points
        .iter()
        .map(|p| [p[0], p[1], p[2] - floor])
        .filter(|p| margin <= 0.0 || p[2] > margin)
        .collect()
}

/// The cloud the body band is taken from: floor-anchored when configured, else
/// as it came. `FloorAnchor.anchor` in the python twin.
///
/// THE TF PRIOR IS REQUIRED, not optional. A low quantile of the cloud alone is
/// only the floor if the floor is in the cloud; hand it a wall and it will
/// happily anchor to the wall's base and delete the wall. `prior` is what says
/// where the ground is supposed to be, so without it the band stays exactly
/// where it was.
pub fn anchored_cloud<'a>(
    points: &'a [[f32; 3]],
    xy: (f64, f64),
    prior: Option<f64>,
    enabled: bool,
    margin: f64,
) -> Cow<'a, [[f32; 3]]> {
    if !enabled || prior.is_none() {
        return Cow::Borrowed(points);
    }
    match estimate_floor(points, xy, prior) {
        Some(f) => Cow::Owned(anchor_to_floor(points, f, margin)),
        None => Cow::Borrowed(points),
    }
}

/// numpy's linear-interpolation percentile, which the python twin calls.
fn percentile(values: &mut [f64], p: f64) -> f64 {
    values.sort_by(f64::total_cmp);
    let n = values.len();
    if n == 1 {
        return values[0];
    }
    let rank = (p / 100.0).clamp(0.0, 1.0) * (n - 1) as f64;
    let lo = rank.floor() as usize;
    let frac = rank - lo as f64;
    if frac == 0.0 {
        return values[lo];
    }
    values[lo] + frac * (values[lo + 1] - values[lo])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A flat floor at `z`, dense enough to be a floor sample.
    fn slab(z: f32, n: usize) -> Vec<[f32; 3]> {
        (0..n)
            .map(|i| {
                let a = i as f32 / n as f32 * std::f32::consts::TAU;
                (a.cos(), a.sin())
            })
            .map(|(c, s)| [c, s, z])
            .collect()
    }

    #[test]
    fn a_flat_floor_is_read_off_the_cloud() {
        let got = estimate_floor(&slab(-0.28, 400), (0.0, 0.0), None).expect("a floor");
        assert!((got - -0.28).abs() < 1e-6, "{got}");
    }

    #[test]
    fn a_sparse_neighbourhood_falls_back_to_the_prior() {
        assert_eq!(
            estimate_floor(&slab(-0.28, 10), (0.0, 0.0), Some(-0.24)),
            Some(-0.24)
        );
        assert_eq!(estimate_floor(&slab(-0.28, 10), (0.0, 0.0), None), None);
    }

    #[test]
    fn points_outside_the_radius_do_not_count() {
        // a floor 10 m away is another room's floor
        let far: Vec<[f32; 3]> = slab(-2.0, 400)
            .iter()
            .map(|p| [p[0] + 10.0, p[1], p[2]])
            .collect();
        let mut pts = slab(-0.28, 400);
        pts.extend(far);
        let got = estimate_floor(&pts, (0.0, 0.0), None).expect("a floor");
        assert!((got - -0.28).abs() < 1e-6, "{got}");
    }

    #[test]
    fn the_prior_bounds_a_wild_estimate() {
        // a hole in the floor (a stairwell edge) drags the low quantile down;
        // tf's base height is the sanity bound that keeps the band on the floor
        let mut pts = slab(-0.28, 400);
        pts.extend(slab(-3.0, 400));
        let got = estimate_floor(&pts, (0.0, 0.0), Some(-0.24)).expect("a floor");
        assert_eq!(got, -0.24);
    }

    #[test]
    fn a_floor_the_prior_agrees_with_is_kept() {
        // the cloud wins inside the tolerance: it is the measurement, the prior
        // is only the bound
        let got = estimate_floor(&slab(-0.28, 400), (0.0, 0.0), Some(-0.24)).expect("a floor");
        assert!((got - -0.28).abs() < 1e-6, "{got}");
    }

    #[test]
    fn anchoring_moves_the_band_onto_the_floor() {
        // a 0.20 m obstacle above a floor at -0.28 reads as 0.48 absolute --
        // outside the 0.05..0.45 band -- and as 0.20 once anchored
        let anchored = anchor_to_floor(&[[1.0, 0.0, -0.08]], -0.28, 0.08);
        assert_eq!(anchored.len(), 1);
        assert!((anchored[0][2] - 0.2).abs() < 1e-6, "{:?}", anchored[0]);
    }

    #[test]
    fn the_ground_slab_is_dropped_rather_than_walled_into_the_band() {
        // the +0.29 counterfactual in the diagnosis: quantisation puts the
        // floor's own layer just inside the band, and every tick then refuses
        let floor = slab(-0.28, 40);
        let upper: Vec<[f32; 3]> = floor.iter().map(|p| [p[0], p[1], p[2] + 0.08]).collect();
        let mut pts = floor;
        pts.extend(upper);
        let anchored = anchor_to_floor(&pts, -0.28, 0.08);
        assert!(
            anchored.is_empty(),
            "{} slab points survived",
            anchored.len()
        );
    }

    #[test]
    fn a_zero_margin_keeps_everything() {
        let anchored = anchor_to_floor(&[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]], 0.0, 0.0);
        assert_eq!(anchored.len(), 2);
    }

    #[test]
    fn a_floor_already_at_zero_is_a_no_op_shift() {
        // the referee's sim worlds put the plan poses on the ground, and the
        // anchoring has to leave those exactly where they were
        let pts = [[1.0, 2.0, 0.3], [-1.0, 0.5, 0.44]];
        let anchored = anchor_to_floor(&pts, 0.0, 0.08);
        assert_eq!(anchored, pts);
    }

    #[test]
    fn the_percentile_interpolates_like_numpy() {
        let mut v = vec![0.0, 1.0, 2.0, 3.0];
        assert_eq!(percentile(&mut v, 0.0), 0.0);
        assert_eq!(percentile(&mut v, 100.0), 3.0);
        assert_eq!(percentile(&mut v, 50.0), 1.5);
        // np.percentile([0,1,2,3], 5) == 0.15
        assert!((percentile(&mut v, 5.0) - 0.15).abs() < 1e-12);
    }
}
