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

//! Which returns are obstacles: the rust twin of `motion/obstacles.py`, which is the specification.
//! A model is z-only over a cloud referenced to the surface the feet stand on; [`field_points`] joins it to the caller's frame.

use std::collections::{HashMap, HashSet};

use crate::planner::{Emb, Soft};

/// Edge of the local map's voxels (m); its points are voxel centers.
pub const VOXEL: f64 = 0.08;

/// Height the gait clears without contact (m). With 8 cm voxels this is the floor layer.
pub const FREE: f64 = 0.03;

/// How far around the robot the floor is read (m).
pub const GROUND_RADIUS: f64 = 1.5;

/// Floor voxels needed before the map's floor is trusted over the body height.
pub const GROUND_MIN_POINTS: usize = 20;

/// Columns taller than this many voxels near the floor are walls, not floor.
pub const FLOOR_COLUMN: usize = 2;

/// This share of floor columns also holding the layer above means the floor straddles the two.
pub const STRADDLE: f64 = 0.15;

/// The model names a config may carry, for the validation error message.
pub const MODELS: [&str; 1] = ["body_band"];

/// What a model made of one cloud, as indices into that cloud.
#[derive(Debug, Default, PartialEq)]
pub struct Field {
    /// Never traversable.
    pub hard: Vec<u32>,
    /// Traversable at a price: `(index, cost)`, the extra price per metre over it.
    pub soft: Vec<(u32, f32)>,
}

/// A z-rule over a cloud referenced to the surface the feet stand on.
pub trait ObstacleModel: Send + Sync {
    fn field(&self, cloud: &[[f32; 3]]) -> Field;
}

/// Obstacles by the body's own geometry, read on each voxel off the ground. The floor layer is
/// free, a voxel centered at or under `steppable` is soft, up to the top of the robot is hard.
pub struct BodyBand {
    steppable: f32,
    height: f32,
    step_cost: f32,
}

impl ObstacleModel for BodyBand {
    fn field(&self, cloud: &[[f32; 3]]) -> Field {
        let half = (VOXEL / 2.0) as f32;
        let mut field = Field::default();
        for (i, p) in cloud.iter().enumerate() {
            let bottom = p[2] - half;
            if bottom < FREE as f32 || bottom >= self.height {
                continue;
            }
            if p[2] <= self.steppable {
                field
                    .soft
                    .push((i as u32, self.step_cost * p[2] / self.steppable));
            } else {
                field.hard.push(i as u32);
            }
        }
        field
    }
}

/// The named model, built for this body, or `None` when the name is unknown.
pub fn load(name: &str, emb: &Emb, step_cost: f64) -> Option<Box<dyn ObstacleModel>> {
    match name {
        "body_band" => Some(Box::new(BodyBand {
            steppable: emb.steppable as f32,
            height: emb.height as f32,
            step_cost: step_cost as f32,
        })),
        _ => None,
    }
}

/// The floor the planner read off the map: its height and the voxels it was read from.
pub struct Ground {
    pub z: f64,
    pub support: Vec<[f32; 3]>,
}

/// The floor near the robot: the fullest voxel layer below `under_z` over wall-free columns.
pub fn ground_from_map(points: &[[f32; 3]], x: f64, y: f64, under_z: f64) -> Option<Ground> {
    let r2 = GROUND_RADIUS * GROUND_RADIUS;
    let window = under_z + 2.0 * VOXEL;
    let near: Vec<([i64; 3], [f32; 3])> = points
        .iter()
        .filter(|p| {
            let (dx, dy) = (p[0] as f64 - x, p[1] as f64 - y);
            dx * dx + dy * dy <= r2 && (p[2] as f64) < window
        })
        .map(|p| (voxel_key(p), *p))
        .collect();
    let mut height: HashMap<(i64, i64), usize> = HashMap::new();
    for (k, _) in &near {
        *height.entry((k[0], k[1])).or_default() += 1;
    }
    let mut layers: HashMap<i64, usize> = HashMap::new();
    let mut columns: HashSet<[i64; 3]> = HashSet::new();
    for (k, p) in &near {
        if height[&(k[0], k[1])] <= FLOOR_COLUMN && (p[2] as f64) < under_z {
            *layers.entry(k[2]).or_default() += 1;
            columns.insert(*k);
        }
    }
    let (&floor, &count) = layers
        .iter()
        .max_by_key(|&(&layer, &count)| (count, std::cmp::Reverse(layer)))?;
    if count < GROUND_MIN_POINTS {
        return None;
    }
    // a floor near a voxel boundary fills both layers in the same columns, and the ground is
    // their shared boundary; a platform's top has the floor under it hidden
    let under: Vec<&[i64; 3]> = columns.iter().filter(|k| k[2] == floor).collect();
    let both = under
        .iter()
        .filter(|k| columns.contains(&[k[0], k[1], floor + 1]))
        .count();
    let straddle = both as f64 >= STRADDLE * under.len() as f64;
    let top = if straddle { floor + 1 } else { floor };
    let support = near
        .iter()
        .filter(|(k, p)| {
            (k[2] == floor || k[2] == top)
                && height[&(k[0], k[1])] <= FLOOR_COLUMN
                && (p[2] as f64) < under_z
        })
        .map(|(_, p)| *p)
        .collect();
    Some(Ground {
        z: top as f64 * VOXEL,
        support,
    })
}

fn voxel_key(p: &[f32; 3]) -> [i64; 3] {
    let v = VOXEL as f32;
    [
        (p[0] / v).floor() as i64,
        (p[1] / v).floor() as i64,
        (p[2] / v).floor() as i64,
    ]
}

/// z off the support surface; f32 throughout, as the python does.
pub fn referenced(points: &[[f32; 3]], ground_z: f64) -> Vec<[f32; 3]> {
    let ground = ground_z as f32;
    points.iter().map(|p| [p[0], p[1], p[2] - ground]).collect()
}

/// The surface the cloud saw, as xy: voxels centered at or under `steppable`.
/// Lattice cells with none are priced as unseen (`Config::unseen_cost`).
pub fn ground_points(points: &[[f32; 3]], ground_z: f64, steppable: f64) -> Vec<[f64; 2]> {
    let ceiling = (ground_z + steppable) as f32;
    points
        .iter()
        .filter(|p| p[2] <= ceiling)
        .map(|p| [p[0] as f64, p[1] as f64])
        .collect()
}

/// What this model sees, referenced to the ground: the hard points the search plans
/// around, and the soft points as `(xy, cost)`. A soft voxel under a hard one is the
/// foot of a wall, not something to step on, so it is dropped.
pub fn field_points(
    model: &dyn ObstacleModel,
    points: &[[f32; 3]],
    ground_z: f64,
) -> (Vec<[f32; 3]>, Vec<Soft>) {
    let pts = referenced(points, ground_z);
    let field = model.field(&pts);
    let hard: Vec<[f32; 3]> = field.hard.iter().map(|&i| pts[i as usize]).collect();
    let walls: HashSet<(i64, i64)> = hard
        .iter()
        .map(|p| {
            let k = voxel_key(p);
            (k[0], k[1])
        })
        .collect();
    let soft = field
        .soft
        .iter()
        .filter(|&&(i, _)| {
            let k = voxel_key(&pts[i as usize]);
            !walls.contains(&(k[0], k[1]))
        })
        .map(|&(i, c)| {
            let p = pts[i as usize];
            ([p[0] as f64, p[1] as f64], c as f64)
        })
        .collect();
    (hard, soft)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> Emb {
        Emb::fixture()
    }

    fn ground_z(points: &[[f32; 3]], x: f64, y: f64, under_z: f64) -> Option<f64> {
        ground_from_map(points, x, y, under_z).map(|g| g.z)
    }

    fn model() -> Box<dyn ObstacleModel> {
        load("body_band", &fixture(), 5.0).expect("known model")
    }

    /// One voxel column at x, centers at `layers` voxel layers above a floor at `ground_z`.
    fn column(x: f32, layers: &[i32], ground_z: f32) -> Vec<[f32; 3]> {
        let v = VOXEL as f32;
        layers
            .iter()
            .map(|&k| [x, 0.0, ground_z + v / 2.0 + k as f32 * v])
            .collect()
    }

    #[test]
    fn the_floor_layer_is_free_and_a_low_object_is_soft() {
        let cloud = column(1.0, &[0, 1], -0.3);
        let (hard, soft) = field_points(model().as_ref(), &cloud, -0.3);
        assert!(hard.is_empty(), "{hard:?}");
        assert_eq!(soft.len(), 1);
        // the voxel is centered 0.12 m up, 0.6 of the way to steppable
        assert!((soft[0].1 - 5.0 * 0.12 / 0.2).abs() < 1e-5, "{soft:?}");
    }

    #[test]
    fn a_voxel_centered_past_steppable_is_hard() {
        let (hard, soft) = field_points(model().as_ref(), &column(1.0, &[0, 3], 0.0), 0.0);
        assert_eq!(hard.len(), 1);
        assert!((hard[0][2] - 0.28).abs() < 1e-6, "{hard:?}");
        assert!(soft.is_empty());
        // the layer under it is the last soft one, at the full price
        let (hard, soft) = field_points(model().as_ref(), &column(1.0, &[0, 2], 0.0), 0.0);
        assert!(hard.is_empty());
        assert!((soft[0].1 - 5.0).abs() < 1e-5, "{soft:?}");
    }

    #[test]
    fn the_foot_of_a_wall_is_wall_not_step() {
        let mut cloud = column(1.0, &[0, 1, 2, 3, 4], 0.0);
        cloud.extend(column(2.0, &[0, 1], 0.0));
        let (hard, soft) = field_points(model().as_ref(), &cloud, 0.0);
        assert_eq!(hard.len(), 2);
        assert_eq!(soft.len(), 1);
        assert_eq!(soft[0].0[0], 2.0);
    }

    #[test]
    fn the_band_reaches_the_top_of_the_robot_and_no_further() {
        let height = fixture().height;
        let v = VOXEL as f32;
        let under = [1.0, 0.0, height as f32 - 0.01 + v / 2.0];
        let over = [1.0, 0.0, height as f32 + 0.01 + v / 2.0];
        let (hard, _) = field_points(model().as_ref(), &[under, over], 0.0);
        assert_eq!(hard, vec![under]);
    }

    #[test]
    fn soft_voxels_count_as_seen_surface_and_hard_ones_do_not() {
        let cloud = column(1.0, &[0, 1, 2, 3], 0.0);
        assert_eq!(ground_points(&cloud, 0.0, fixture().steppable).len(), 3);
    }

    /// A floor patch of voxel centers over 1.2 m around the origin, at these layers each.
    fn floor(layers: &[i32]) -> Vec<[f32; 3]> {
        let v = VOXEL as f32;
        let mut pts = Vec::new();
        for i in -15..=15 {
            for j in -15..=15 {
                for &k in layers {
                    pts.push([
                        (i as f32 + 0.5) * v,
                        (j as f32 + 0.5) * v,
                        v / 2.0 + k as f32 * v,
                    ]);
                }
            }
        }
        pts
    }

    #[test]
    fn the_ground_is_the_floor_layer_not_the_block_on_it() {
        let mut cloud = floor(&[0]);
        for i in 0..4 {
            for j in 0..4 {
                cloud.extend(
                    column(0.5 + 0.08 * i as f32, &[1], 0.0)
                        .iter()
                        .map(|p| [p[0], 0.08 * j as f32, p[2]]),
                );
            }
        }
        assert_eq!(ground_z(&cloud, 0.0, 0.0, 0.2), Some(0.0));
        // the floor layer is the one under the body, wherever the robot stands
        let lifted: Vec<[f32; 3]> = cloud.iter().map(|p| [p[0], p[1], p[2] + 0.4]).collect();
        let got = ground_z(&lifted, 0.0, 0.0, 0.6).expect("floor");
        assert!((got - 0.4).abs() < 1e-6, "{got}");
    }

    #[test]
    fn a_floor_across_a_voxel_boundary_is_ground_at_the_boundary() {
        let got = ground_z(&floor(&[0, 1]), 0.0, 0.0, 0.2).expect("floor");
        assert!((got - VOXEL).abs() < 1e-9, "{got}");
        let (hard, soft) = field_points(model().as_ref(), &floor(&[0, 1]), got);
        assert!(hard.is_empty() && soft.is_empty());
    }

    #[test]
    fn a_broad_low_platform_is_not_a_floor_across_a_boundary() {
        // a strip a third of the patch wide, one layer up, with the floor under it hidden
        let v = VOXEL as f32;
        let cloud: Vec<[f32; 3]> = floor(&[0])
            .into_iter()
            .map(|p| {
                if p[0] > 0.2 && p[0] < 0.6 {
                    [p[0], p[1], p[2] + v]
                } else {
                    p
                }
            })
            .collect();
        assert_eq!(ground_z(&cloud, 0.0, 0.0, 0.2), Some(0.0));
    }

    #[test]
    fn the_support_is_the_floor_voxels_and_not_the_wall_or_the_block() {
        let mut cloud = floor(&[0]);
        let floor_n = cloud.len();
        for i in -15..=15 {
            cloud.extend(
                column((i as f32 + 0.5) * 0.08, &[0, 1, 2, 3, 4], 0.0)
                    .iter()
                    .map(|p| [p[0], 1.32, p[2]]),
            );
        }
        cloud.push([0.52, 0.52, 0.12]);
        let g = ground_from_map(&cloud, 0.0, 0.0, 0.2).expect("floor");
        assert!(g.support.iter().all(|p| (p[2] - 0.04).abs() < 1e-6));
        assert!(g.support.iter().all(|p| p[1] < 1.3));
        assert!(g.support.len() <= floor_n && g.support.len() > floor_n / 2);
        // across a voxel boundary both floor layers are support, out to the read radius
        let both = floor(&[0, 1]);
        let g = ground_from_map(&both, 0.0, 0.0, 0.2).expect("floor");
        let inside = both
            .iter()
            .filter(|p| (p[0] as f64).hypot(p[1] as f64) <= GROUND_RADIUS)
            .count();
        assert_eq!(g.support.len(), inside);
    }

    #[test]
    fn walls_do_not_vote_for_the_floor() {
        let mut cloud = floor(&[0]);
        for i in -15..=15 {
            cloud.extend(
                column(0.08 * i as f32, &[0, 1, 2, 3, 4], 0.0)
                    .iter()
                    .map(|p| [p[0], 0.6, p[2]]),
            );
        }
        assert_eq!(ground_z(&cloud, 0.0, 0.0, 0.2), Some(0.0));
    }

    #[test]
    fn too_little_floor_leaves_the_ground_to_the_caller() {
        let cloud = column(0.5, &[0], 0.0);
        assert_eq!(ground_z(&cloud, 0.0, 0.0, 0.2), None);
        let far: Vec<[f32; 3]> = (0..40).map(|i| [3.0, 0.03 * i as f32, 0.04]).collect();
        assert_eq!(ground_z(&far, 0.0, 0.0, 0.2), None);
    }

    #[test]
    fn an_unknown_model_is_refused_rather_than_defaulted() {
        assert!(load("floor_anchor", &fixture(), 5.0).is_none());
        for name in MODELS {
            assert!(
                load(name, &fixture(), 5.0).is_some(),
                "{name} is advertised but unknown"
            );
        }
    }
}
