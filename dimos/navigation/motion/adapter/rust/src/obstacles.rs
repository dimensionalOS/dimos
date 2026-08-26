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

//! Which returns are obstacles -- the rust twin of `motion/obstacles.py`,
//! which is the specification.
//!
//! The band is a property of the BODY, not of the scene: the base rides
//! `base_height` above the surface its feet stand on, so a cloud referenced to
//! that surface says directly what the body can hit. Nothing is estimated off
//! the scene, so nothing can be estimated wrong.
//!
//! A model is z-only and says nothing about xy. The frame is the caller's --
//! [`hard_points`] is where the two meet.

use dimos_motion2_target::planner::Emb;

/// Ground exclusion for the body-referenced band. TWO voxel layers, not one: a
/// floor whose true height sits near a voxel boundary quantises into both
/// layers either side of it, and one layer leaves the upper one standing as a
/// carpet the search cannot cross.
pub const LOW: f64 = 0.16;

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

/// The cloud in the frame a model reads: z off the support surface. f32
/// throughout, as the python does -- the planner's SDF and the room hint both
/// see the shifted numbers, and widening first would disagree with them in the
/// last bits.
pub fn referenced(points: &[[f32; 3]], ground_z: f64) -> Vec<[f32; 3]> {
    let ground = ground_z as f32;
    points.iter().map(|p| [p[0], p[1], p[2] - ground]).collect()
}

/// The obstacles this model sees -- the cloud the search plans on.
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

    fn go2() -> Emb {
        Emb::go2()
    }

    /// A ground slab 0..0.12 m thick under a 0.30 m obstacle, lifted to `base_z`.
    ///
    /// The recording's geometry: the map's z origin is base height, so absolute
    /// z says nothing until it is referenced to the surface the feet stand on.
    fn room(ground_z: f32) -> Vec<[f32; 3]> {
        let mut pts = Vec::new();
        for x in [-1.0f32, 0.0, 1.0] {
            for z in [0.0f32, 0.04, 0.08, 0.12] {
                pts.push([x, 0.0, z + ground_z]);
            }
        }
        for z in [0.18f32, 0.24, 0.30] {
            pts.push([2.0, 0.0, z + ground_z]);
        }
        pts
    }

    #[test]
    fn the_body_band_drops_the_ground_slab_and_keeps_the_obstacle() {
        // the phantom regression: a quantised floor must not become a wall
        let model = load("body_band", &go2()).expect("known model");
        let got = hard_points(model.as_ref(), &room(-0.28), -0.28);
        assert_eq!(got.len(), 3, "{got:?}");
        for (p, want) in got.iter().zip([0.18f32, 0.24, 0.30]) {
            assert!((p[2] - want).abs() < 1e-6, "{p:?} wanted {want}");
        }
    }

    #[test]
    fn the_body_band_looks_under_the_belly_not_over_it() {
        let model = load("body_band", &go2()).expect("known model");
        let cloud = [[1.0, 0.0, 0.3], [1.0, 0.0, 0.46]];
        let got = hard_points(model.as_ref(), &cloud, 0.0);
        assert_eq!(got, vec![[1.0, 0.0, 0.3]]);
    }

    #[test]
    fn an_unknown_model_is_refused_rather_than_defaulted() {
        assert!(load("floor_anchor", &go2()).is_none());
        for name in MODELS {
            assert!(
                load(name, &go2()).is_some(),
                "{name} is advertised but unknown"
            );
        }
    }

    #[test]
    fn the_soft_tier_is_empty_for_now() {
        let model = load("body_band", &go2()).expect("known model");
        assert!(model.field(&room(0.0)).soft.is_empty());
    }
}
