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

use crate::planner::Emb;

/// Ground exclusion (m): three voxel layers, since the floor reads a layer high at range (see obstacles.py).
pub const LOW: f64 = 0.24;

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

/// Obstacles by the body's own geometry: clear of the ground, under the belly.
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
    fn the_body_band_looks_under_the_belly_not_over_it() {
        let model = load("body_band", &fixture()).expect("known model");
        let cloud = [[1.0, 0.0, 0.3], [1.0, 0.0, 0.46]];
        let got = hard_points(model.as_ref(), &cloud, 0.0);
        assert_eq!(got, vec![[1.0, 0.0, 0.3]]);
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
