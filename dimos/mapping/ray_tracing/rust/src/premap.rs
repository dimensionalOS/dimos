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

//! A loaded map kept apart from the live map, in its own frame, so a relocalizer can keep moving
//! it: live rays clear it through the latest world -> premap transform, and the emitted maps place
//! what is left of it in the world by that transform.

use ahash::AHashSet;
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion};

use crate::mapper::{Point, Pose};
use crate::voxel_ray_tracer::{
    clear_premap, emit_points_flags, emit_points_ungated_flags, partition_seed, seed_tile, Config,
    Cylinder, VoxelKey, VoxelMap,
};

pub struct Premap {
    pub frame_id: String,
    map: VoxelMap,
    /// The seed regions' cylinders, in the premap's frame.
    regions: Vec<Cylinder>,
    /// Corners of the premap's bounding box, which measure how far a new placement moves it.
    corners: Vec<Point3<f32>>,
    /// world <- premap, the latest transform the frame was traced under.
    placement: Option<Isometry3<f32>>,
    /// world <- premap the regions were last published under.
    regions_placement: Option<Isometry3<f32>>,
}

impl Premap {
    /// Seed a cloud given in its own frame, every voxel premap.
    pub fn load(frame_id: String, points: &[Point], cfg: &Config) -> Self {
        let mut map = VoxelMap::with_support_min(cfg.support_min);
        let part = partition_seed(points, cfg.voxel_size, (0.0, 0.0, 0.0), cfg.region_m);
        map.reserve_chunks(part.tile_count());
        let all_premap = AHashSet::new();
        for tile in part.tiles() {
            seed_tile(&mut map, tile, &all_premap, cfg);
        }
        let (lo, hi) = points.iter().fold(
            ([f32::INFINITY; 3], [f32::NEG_INFINITY; 3]),
            |(lo, hi), &(x, y, z)| {
                (
                    [lo[0].min(x), lo[1].min(y), lo[2].min(z)],
                    [hi[0].max(x), hi[1].max(y), hi[2].max(z)],
                )
            },
        );
        Self {
            frame_id,
            map,
            regions: part.regions.iter().map(|r| r.cylinder).collect(),
            corners: box_corners(lo, hi),
            placement: None,
            regions_placement: None,
        }
    }

    pub fn healthy_count(&self) -> usize {
        self.map.healthy_count()
    }

    /// Place the premap by a world <- premap pose, or unplace it with None.
    pub fn place(&mut self, world_from_premap: Option<Pose>) {
        self.placement = world_from_premap.map(isometry);
    }

    pub fn placed(&self) -> bool {
        self.placement.is_some()
    }

    /// Clear what one world-frame frame's rays pass through. Returns how many voxels it cleared.
    pub fn clear(&mut self, world_points: &[Point], world_origin: Point, cfg: &Config) -> usize {
        let Some(world_from_premap) = self.placement else {
            return 0;
        };
        let premap_from_world = world_from_premap.inverse();
        let points: Vec<Point> = world_points
            .iter()
            .map(|&p| apply(&premap_from_world, p))
            .collect();
        let origin = apply(&premap_from_world, world_origin);
        clear_premap(&mut self.map, origin, &points, cfg)
    }

    /// Its support-gated uncleared voxels within world `bounds`, as world (x, y, z, flags) quads,
    /// leaving out any whose world voxel `skip` says the live map already holds.
    pub fn local_points(
        &self,
        bounds: &Cylinder,
        voxel_size: f32,
        skip: impl Fn(f32, f32, f32) -> bool,
    ) -> Vec<f32> {
        let Some(world_from_premap) = self.placement else {
            return Vec::new();
        };
        let query = cylinder_through(&world_from_premap.inverse(), bounds).bounds();
        let empty = AHashSet::new();
        let points = emit_points_flags(&self.map, voxel_size, Some(&query), &empty);
        let world = bounds.bounds();
        place_quads(&world_from_premap, &points, |x, y, z| {
            world.contains(x, y, z) && !skip(x, y, z)
        })
    }

    /// Every uncleared voxel, as world (x, y, z, flags) quads.
    pub fn global_points(&self, voxel_size: f32) -> Vec<f32> {
        let Some(world_from_premap) = self.placement else {
            return Vec::new();
        };
        let empty: AHashSet<VoxelKey> = AHashSet::new();
        let points = emit_points_ungated_flags(&self.map, voxel_size, &empty);
        place_quads(&world_from_premap, &points, |_, _, _| true)
    }

    /// Its regions as world cylinders when its placement has moved more than `shift_m` since they
    /// were last taken (or never were), zero taking them only at the first placement.
    pub fn take_regions_due(&mut self, shift_m: f32) -> Vec<Cylinder> {
        let Some(now) = self.placement else {
            return Vec::new();
        };
        let due = match self.regions_placement {
            None => true,
            Some(_) if shift_m <= 0.0 => false,
            Some(then) => self
                .corners
                .iter()
                .any(|c| (now * c - then * c).norm() > shift_m),
        };
        if !due {
            return Vec::new();
        }
        self.regions_placement = Some(now);
        self.regions
            .iter()
            .map(|c| cylinder_through(&now, c))
            .collect()
    }
}

fn isometry(pose: Pose) -> Isometry3<f32> {
    let (px, py, pz) = pose.position;
    let (qx, qy, qz, qw) = pose.orientation;
    Isometry3::from_parts(
        Translation3::new(px, py, pz),
        UnitQuaternion::from_quaternion(Quaternion::new(qw, qx, qy, qz)),
    )
}

fn apply(iso: &Isometry3<f32>, (x, y, z): Point) -> Point {
    let p = iso * Point3::new(x, y, z);
    (p.x, p.y, p.z)
}

fn box_corners(lo: [f32; 3], hi: [f32; 3]) -> Vec<Point3<f32>> {
    let mut corners = Vec::with_capacity(8);
    for x in [lo[0], hi[0]] {
        for y in [lo[1], hi[1]] {
            for z in [lo[2], hi[2]] {
                corners.push(Point3::new(x, y, z));
            }
        }
    }
    corners
}

/// The upright cylinder around `cylinder`'s bounding box once moved by `iso`, so a query through
/// it misses nothing the moved cylinder holds.
fn cylinder_through(iso: &Isometry3<f32>, cylinder: &Cylinder) -> Cylinder {
    let r = cylinder.radius;
    let corners = box_corners(
        [cylinder.cx - r, cylinder.cy - r, cylinder.z_min],
        [cylinder.cx + r, cylinder.cy + r, cylinder.z_max],
    );
    let moved: Vec<Point3<f32>> = corners.iter().map(|c| iso * c).collect();
    let center = iso
        * Point3::new(
            cylinder.cx,
            cylinder.cy,
            (cylinder.z_min + cylinder.z_max) * 0.5,
        );
    let radius = moved
        .iter()
        .map(|c| (c.x - center.x).hypot(c.y - center.y))
        .fold(0.0, f32::max);
    let (z_min, z_max) = moved
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), c| {
            (lo.min(c.z), hi.max(c.z))
        });
    Cylinder {
        cx: center.x,
        cy: center.y,
        radius,
        z_min,
        z_max,
    }
}

/// (x, y, z, flags) quads moved by `iso`, keeping those `keep` takes.
fn place_quads(
    iso: &Isometry3<f32>,
    quads: &[f32],
    keep: impl Fn(f32, f32, f32) -> bool,
) -> Vec<f32> {
    let mut out = Vec::with_capacity(quads.len());
    for i in (0..quads.len() / 4).map(|n| 4 * n) {
        let (x, y, z) = apply(iso, (quads[i], quads[i + 1], quads[i + 2]));
        if keep(x, y, z) {
            out.extend_from_slice(&[x, y, z, quads[i + 3]]);
        }
    }
    out
}

/// Whether a world point lies in a voxel the live map holds or hit this frame.
pub fn live_holds<'a>(
    map: &'a VoxelMap,
    live: &'a AHashSet<VoxelKey>,
    voxel_size: f32,
) -> impl Fn(f32, f32, f32) -> bool + 'a {
    let inv = 1.0 / voxel_size;
    move |x, y, z| {
        let key = (
            (x * inv).floor() as i32,
            (y * inv).floor() as i32,
            (z * inv).floor() as i32,
        );
        map.voxels.is_healthy(&key) || live.contains(&key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const IDENTITY: (f32, f32, f32, f32) = (0.0, 0.0, 0.0, 1.0);

    fn config() -> Config {
        Config {
            voxel_size: 1.0,
            fine_divisor: 0,
            emit_fine: false,
            max_range: 100.0,
            pre_trace_subsample: 1,
            ray_subsample: 1,
            shadow_depth: 0.0,
            grace_depth: 0.0,
            min_health: -1,
            max_health: 5,
            graze_cos: 0.0,
            graze_recent_frames: u32::MAX,
            support_min: 0,
            emit_every: 1,
            global_emit_every: 1,
            region_percentile: 95.0,
            region_radius_window: 1,
            world_frame: "world".to_string(),
            tf_match_tolerance_s: 0.1,
            max_cloud_age_s: 0.0,
            tf_wait_timeout_s: 0.05,
            worker_threads: 1,
            region_m: 4.0,
            viz_cell_m: 4.0,
            viz_emit_every: 0,
            viz_sweep_cells: 0,
            separate_premaps: true,
            premap_regions_shift_m: 0.0,
        }
    }

    fn shifted(x: f32) -> Pose {
        Pose {
            position: (x, 0.0, 0.0),
            orientation: IDENTITY,
        }
    }

    fn everywhere() -> Cylinder {
        Cylinder {
            cx: 0.0,
            cy: 0.0,
            radius: 100.0,
            z_min: -10.0,
            z_max: 10.0,
        }
    }

    fn xs(quads: &[f32]) -> Vec<f32> {
        quads.iter().step_by(4).copied().collect()
    }

    /// A premap three voxels along +x, its frame placed 10 m along world +x.
    fn row() -> Premap {
        let mut premap = Premap::load(
            "premap".into(),
            &[(1.5, 0.5, 0.5), (2.5, 0.5, 0.5), (3.5, 0.5, 0.5)],
            &config(),
        );
        premap.place(Some(shifted(10.0)));
        premap
    }

    #[test]
    fn an_unplaced_premap_emits_and_clears_nothing() {
        let mut premap = row();
        premap.place(None);
        assert!(premap.global_points(1.0).is_empty());
        assert_eq!(
            premap.clear(&[(15.5, 0.5, 0.5)], (10.5, 0.5, 0.5), &config()),
            0
        );
    }

    #[test]
    fn points_are_placed_in_the_world_and_flagged_premap() {
        let premap = row();
        let points = premap.global_points(1.0);
        assert_eq!(xs(&points), vec![11.5, 12.5, 13.5]);
        assert!(points.iter().skip(3).step_by(4).all(|f| f.to_bits() == 1));
    }

    /// A world ray from x = 10.5 to 15.5 runs through the premap's frame x = 0.5 .. 5.5.
    #[test]
    fn world_rays_clear_through_the_placement_and_the_voxels_stay_marked() {
        let mut premap = row();
        let cleared = premap.clear(&[(15.5, 0.5, 0.5)], (10.5, 0.5, 0.5), &config());
        assert_eq!(cleared, 3);
        assert!(premap.global_points(1.0).is_empty());
        assert_eq!(
            premap.map.voxels.len(),
            3,
            "cleared voxels stay in the premap"
        );
        // Moving the premap afterwards does not bring them back.
        premap.place(Some(shifted(20.0)));
        assert!(premap.global_points(1.0).is_empty());
    }

    #[test]
    fn a_ray_short_of_the_premap_leaves_it() {
        let mut premap = row();
        assert_eq!(
            premap.clear(&[(11.5, 0.5, 0.5)], (10.5, 0.5, 0.5), &config()),
            0
        );
        assert_eq!(xs(&premap.global_points(1.0)), vec![11.5, 12.5, 13.5]);
    }

    #[test]
    fn local_points_keep_to_the_world_bounds_and_skip_what_live_holds() {
        let premap = row();
        let bounds = Cylinder {
            cx: 12.0,
            cy: 0.5,
            radius: 1.0,
            z_min: -1.0,
            z_max: 2.0,
        };
        assert_eq!(
            xs(&premap.local_points(&bounds, 1.0, |_, _, _| false)),
            vec![11.5, 12.5]
        );
        let live_at_12 = |x: f32, _, _| x.floor() == 12.0;
        assert_eq!(
            xs(&premap.local_points(&everywhere(), 1.0, live_at_12)),
            vec![11.5, 13.5]
        );
    }

    #[test]
    fn local_points_follow_a_rotated_placement() {
        let mut premap = row();
        // Yaw 90 deg: premap +x is world +y.
        let half = 2.0_f32.sqrt() / 2.0;
        premap.place(Some(Pose {
            position: (0.0, 0.0, 0.0),
            orientation: (0.0, 0.0, half, half),
        }));
        let bounds = Cylinder {
            cx: -0.5,
            cy: 3.5,
            radius: 0.5,
            z_min: 0.0,
            z_max: 1.0,
        };
        let points = premap.local_points(&bounds, 1.0, |_, _, _| false);
        assert_eq!(points.len(), 4);
        assert!((points[0] + 0.5).abs() < 1e-5 && (points[1] - 3.5).abs() < 1e-5);
    }

    #[test]
    fn regions_are_due_at_the_first_placement_and_after_a_big_enough_shift() {
        let mut premap = row();
        let first = premap.take_regions_due(0.5);
        assert!(!first.is_empty());
        assert!(first[0].cx > 10.0, "regions are in the world frame");
        assert!(premap.take_regions_due(0.5).is_empty());
        premap.place(Some(shifted(10.3)));
        assert!(premap.take_regions_due(0.5).is_empty());
        premap.place(Some(shifted(10.6)));
        assert!(!premap.take_regions_due(0.5).is_empty());

        let mut once = row();
        assert!(!once.take_regions_due(0.0).is_empty());
        once.place(Some(shifted(50.0)));
        assert!(once.take_regions_due(0.0).is_empty());
    }
}
