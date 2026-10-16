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

//! Which returns are obstacles: a property of the body, not of the scene.
//! A model is z-only over a cloud referenced to the surface the feet stand on.
//! [`field_points`] joins it to the caller's frame.

use std::collections::HashSet;

use crate::module::ground::Plane;
use crate::planner::{Emb, Soft};

/// Edge of the local map's voxels (m). Its points are voxel centers.
pub const VOXEL: f64 = 0.08;

/// Height the gait clears without contact (m). With 8 cm voxels this is the floor layer.
pub const FREE: f64 = 0.03;

/// Voxel centers carry f32 rounding, so a height within this of a threshold is on it (m).
const ON: f32 = 1e-3;

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

/// Obstacles by the body's own geometry, read on each voxel off the ground. A voxel reaching
/// down to the floor is free, one centered at or under `steppable` is soft, and one starting
/// under the top of the robot is hard.
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
            if bottom < FREE as f32 || bottom >= self.height - ON {
                continue;
            }
            if p[2] <= self.steppable + ON {
                let share = (p[2] / self.steppable).min(1.0);
                field.soft.push((i as u32, self.step_cost * share));
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

/// The voxel holding `p`, as layer indices.
pub fn voxel_key(p: &[f32; 3]) -> [i64; 3] {
    let v = VOXEL as f32;
    [
        (p[0] / v).floor() as i64,
        (p[1] / v).floor() as i64,
        (p[2] / v).floor() as i64,
    ]
}

/// z off the floor under each point, in f32.
pub fn referenced(points: &[[f32; 3]], ground: &Plane) -> Vec<[f32; 3]> {
    points
        .iter()
        .map(|p| {
            let z = p[2] as f64 - ground.at(p[0] as f64, p[1] as f64);
            [p[0], p[1], z as f32]
        })
        .collect()
}

/// The surface the cloud saw, as xy: voxels centered at or under `steppable`.
/// Lattice cells with none are priced as unseen (`Config::unseen_cost`).
pub fn ground_points(points: &[[f32; 3]], ground: &Plane, steppable: f64) -> Vec<[f64; 2]> {
    let ceiling = steppable as f32 + ON;
    referenced(points, ground)
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
    ground: &Plane,
) -> (Vec<[f32; 3]>, Vec<Soft>) {
    let pts = referenced(points, ground);
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

    /// The voxel center the ray tracer emits for layer `k`, in its own f32 arithmetic.
    fn center(k: i64) -> f32 {
        let v = VOXEL as f32;
        k as f32 * v + v * 0.5
    }

    #[test]
    fn the_floor_layer_is_free_and_a_low_object_is_soft() {
        let cloud = column(1.0, &[0, 1], -0.3);
        let (hard, soft) = field_points(model().as_ref(), &cloud, &Plane::flat(-0.3));
        assert!(hard.is_empty(), "{hard:?}");
        assert_eq!(soft.len(), 1);
        // the voxel is centered 0.12 m up, 0.6 of the way to steppable
        assert!((soft[0].1 - 5.0 * 0.12 / 0.2).abs() < 1e-5, "{soft:?}");
    }

    #[test]
    fn a_voxel_centered_past_steppable_is_hard() {
        let (hard, soft) = field_points(
            model().as_ref(),
            &column(1.0, &[0, 3], 0.0),
            &Plane::flat(0.0),
        );
        assert_eq!(hard.len(), 1);
        assert!((hard[0][2] - 0.28).abs() < 1e-6, "{hard:?}");
        assert!(soft.is_empty());
        // the layer under it is the last soft one, at the full price
        let (hard, soft) = field_points(
            model().as_ref(),
            &column(1.0, &[0, 2], 0.0),
            &Plane::flat(0.0),
        );
        assert!(hard.is_empty());
        assert!((soft[0].1 - 5.0).abs() < 1e-5, "{soft:?}");
    }

    #[test]
    fn the_layer_centered_on_steppable_is_soft_on_any_floor() {
        // a ground on a voxel face puts the second layer's center on steppable, up to rounding
        for top in -40..=40 {
            let cloud = [[1.0, 0.0, center(top)], [1.0, 0.0, center(top + 2)]];
            let ground = Plane::flat(top as f64 * VOXEL);
            let (hard, soft) = field_points(model().as_ref(), &cloud, &ground);
            assert!(hard.is_empty(), "floor layer {top}: {hard:?}");
            assert_eq!(soft.len(), 1, "floor layer {top}");
            assert!(
                (soft[0].1 - 5.0).abs() < 1e-3,
                "floor layer {top}: {soft:?}"
            );
            assert_eq!(ground_points(&cloud, &ground, fixture().steppable).len(), 2);
        }
    }

    #[test]
    fn a_voxel_starting_at_the_top_of_the_robot_is_overhead_on_any_floor() {
        // a ground at a voxel center puts the eighth layer's bottom on the 0.60 m height
        for top in -40..=40 {
            let ground = Plane::flat(center(top) as f64);
            let cloud = [[1.0, 0.0, center(top + 7)], [1.0, 0.0, center(top + 8)]];
            let (hard, _) = field_points(model().as_ref(), &cloud, &ground);
            assert_eq!(hard.len(), 1, "floor layer {top}: {hard:?}");
            assert!(
                (hard[0][2] - 0.56).abs() < 1e-4,
                "floor layer {top}: {hard:?}"
            );
        }
    }

    #[test]
    fn heights_are_read_off_the_floor_under_each_voxel() {
        let tilted = Plane {
            x0: 0.0,
            y0: 0.0,
            z: 0.0,
            gx: 0.05,
            gy: 0.0,
        };
        // 3 m out the floor is 0.15 m up: a voxel there at 0.19 is floor, at 0.27 a step
        let cloud = [[3.0, 0.0, 0.19], [3.0, 1.0, 0.27], [3.0, 2.0, 0.55]];
        let (hard, soft) = field_points(model().as_ref(), &cloud, &tilted);
        assert_eq!(soft.len(), 1, "{soft:?}");
        assert_eq!(soft[0].0[1], 1.0);
        assert_eq!(hard.len(), 1, "{hard:?}");
        assert_eq!(hard[0][1], 2.0);
        assert_eq!(ground_points(&cloud, &tilted, fixture().steppable).len(), 2);
    }

    #[test]
    fn the_foot_of_a_wall_is_wall_not_step() {
        let mut cloud = column(1.0, &[0, 1, 2, 3, 4], 0.0);
        cloud.extend(column(2.0, &[0, 1], 0.0));
        let (hard, soft) = field_points(model().as_ref(), &cloud, &Plane::flat(0.0));
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
        let (hard, _) = field_points(model().as_ref(), &[under, over], &Plane::flat(0.0));
        assert_eq!(hard, vec![under]);
    }

    #[test]
    fn soft_voxels_count_as_seen_surface_and_hard_ones_do_not() {
        let cloud = column(1.0, &[0, 1, 2, 3], 0.0);
        assert_eq!(
            ground_points(&cloud, &Plane::flat(0.0), fixture().steppable).len(),
            3
        );
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
