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
//! A model is z-only over a cloud referenced to the surface the feet stand on; [`hard_points`] joins it to the caller's frame.

use std::collections::{HashMap, HashSet};

use crate::planner::Emb;

/// Ground exclusion (m): three voxel layers, since the floor reads a layer high at range (see obstacles.py).
pub const LOW: f64 = 0.24;

/// Edge of the local map's voxels (m); its points are voxel centers.
pub const VOXEL: f64 = 0.08;

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
    /// Traversable at a price: `(index, cost)`. Empty for now.
    pub soft: Vec<(u32, f32)>,
}

/// A z-rule over a cloud referenced to the surface the feet stand on.
pub trait ObstacleModel: Send + Sync {
    fn field(&self, cloud: &[[f32; 3]]) -> Field;
}

/// Obstacles by the body's own geometry: clear of the ground, under the top of the robot.
pub struct BodyBand {
    low: f32,
    high: f32,
}

impl ObstacleModel for BodyBand {
    fn field(&self, cloud: &[[f32; 3]]) -> Field {
        Field {
            hard: indices(cloud, |z| z > self.low && z <= self.high),
            soft: Vec::new(),
        }
    }
}

fn indices(cloud: &[[f32; 3]], keep: impl Fn(f32) -> bool) -> Vec<u32> {
    cloud
        .iter()
        .enumerate()
        .filter(|(_, p)| keep(p[2]))
        .map(|(i, _)| i as u32)
        .collect()
}

/// The named model, built for this body, or `None` when the name is unknown.
pub fn load(name: &str, emb: &Emb) -> Option<Box<dyn ObstacleModel>> {
    match name {
        "body_band" => Some(Box::new(BodyBand {
            low: LOW as f32,
            high: emb.height as f32,
        })),
        _ => None,
    }
}

/// The floor near the robot: the fullest voxel layer below `under_z` over wall-free columns.
pub fn ground_from_map(points: &[[f32; 3]], x: f64, y: f64, under_z: f64) -> Option<f64> {
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
    Some(top as f64 * VOXEL)
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

/// The floor the cloud saw, as xy; lattice cells with none are priced as unseen (`Config::unseen_cost`).
pub fn ground_points(points: &[[f32; 3]], ground_z: f64) -> Vec<[f64; 2]> {
    let ground = ground_z as f32;
    points
        .iter()
        .filter(|p| p[2] - ground <= LOW as f32)
        .map(|p| [p[0] as f64, p[1] as f64])
        .collect()
}

/// The obstacles this model sees: the cloud the search plans on.
pub fn hard_points(model: &dyn ObstacleModel, points: &[[f32; 3]], ground_z: f64) -> Vec<[f32; 3]> {
    let pts = referenced(points, ground_z);
    model
        .field(&pts)
        .hard
        .iter()
        .map(|&i| pts[i as usize])
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> Emb {
        Emb::fixture()
    }

    /// A ground slab under an obstacle just above LOW, lifted to `ground_z`.
    fn room(ground_z: f32) -> Vec<[f32; 3]> {
        let mut pts = Vec::new();
        for x in [-1.0f32, 0.0, 1.0] {
            for z in [0.0f32, 0.04, 0.08, 0.12] {
                pts.push([x, 0.0, z + ground_z]);
            }
        }
        for z in obstacle_heights() {
            pts.push([2.0, 0.0, z + ground_z]);
        }
        pts
    }

    /// Three heights just above the ground exclusion, so the fixture follows LOW.
    fn obstacle_heights() -> [f32; 3] {
        let low = LOW as f32;
        [low + 0.02, low + 0.08, low + 0.14]
    }

    #[test]
    fn the_body_band_drops_the_ground_slab_and_keeps_the_obstacle() {
        // the phantom regression: a quantised floor must not become a wall
        let model = load("body_band", &fixture()).expect("known model");
        let got = hard_points(model.as_ref(), &room(-0.28), -0.28);
        assert_eq!(got.len(), 3, "{got:?}");
        for (p, want) in got.iter().zip(obstacle_heights()) {
            assert!((p[2] - want).abs() < 1e-6, "{p:?} wanted {want}");
        }
    }

    #[test]
    fn the_body_band_reaches_the_top_of_the_robot_and_no_further() {
        let model = load("body_band", &fixture()).expect("known model");
        let cloud = [[1.0, 0.0, 0.3], [1.0, 0.0, 0.61]];
        let got = hard_points(model.as_ref(), &cloud, 0.0);
        assert_eq!(got, vec![[1.0, 0.0, 0.3]]);
    }

    /// One voxel column at x, centers at `layers` voxel layers above a floor at `ground_z`.
    fn column(x: f32, layers: &[i32], ground_z: f32) -> Vec<[f32; 3]> {
        let v = VOXEL as f32;
        layers
            .iter()
            .map(|&k| [x, 0.0, ground_z + v / 2.0 + k as f32 * v])
            .collect()
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
        assert_eq!(ground_from_map(&cloud, 0.0, 0.0, 0.2), Some(0.0));
        // the floor layer is the one under the body, wherever the robot stands
        let lifted: Vec<[f32; 3]> = cloud.iter().map(|p| [p[0], p[1], p[2] + 0.4]).collect();
        let got = ground_from_map(&lifted, 0.0, 0.0, 0.6).expect("floor");
        assert!((got - 0.4).abs() < 1e-6, "{got}");
    }

    #[test]
    fn a_floor_across_a_voxel_boundary_is_ground_at_the_boundary() {
        let got = ground_from_map(&floor(&[0, 1]), 0.0, 0.0, 0.2).expect("floor");
        assert!((got - VOXEL).abs() < 1e-9, "{got}");
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
        assert_eq!(ground_from_map(&cloud, 0.0, 0.0, 0.2), Some(0.0));
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
        assert_eq!(ground_from_map(&cloud, 0.0, 0.0, 0.2), Some(0.0));
    }

    #[test]
    fn too_little_floor_leaves_the_ground_to_the_caller() {
        let cloud = column(0.5, &[0], 0.0);
        assert_eq!(ground_from_map(&cloud, 0.0, 0.0, 0.2), None);
        let far: Vec<[f32; 3]> = (0..40).map(|i| [3.0, 0.03 * i as f32, 0.04]).collect();
        assert_eq!(ground_from_map(&far, 0.0, 0.0, 0.2), None);
    }

    #[test]
    fn an_unknown_model_is_refused_rather_than_defaulted() {
        assert!(load("floor_anchor", &fixture()).is_none());
        for name in MODELS {
            assert!(
                load(name, &fixture()).is_some(),
                "{name} is advertised but unknown"
            );
        }
    }

    #[test]
    fn the_soft_tier_is_empty_for_now() {
        let model = load("body_band", &fixture()).expect("known model");
        assert!(model.field(&room(0.0)).soft.is_empty());
    }
}
