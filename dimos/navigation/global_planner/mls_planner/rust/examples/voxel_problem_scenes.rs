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

//! Plan across small synthetic worlds where whole voxels misread the surface,
//! once from whole voxels and once from the ray tracer's planes, and write
//! what each saw for `utils/map_pages.py scenes`.
//!
//!     cargo run --release --example voxel_problem_scenes -- <out.json>

#[path = "../src/mls_planner/voxel_problem.rs"]
mod voxel_problem;

use dimos_mls_planner::mls_planner::{Config, MapPoint, Planner};
use serde_json::{json, Value};
use voxel_problem::{center, plan, scenes, voxels, whole_voxels, Scene, LENGTH, VOXEL};

/// What the planner made of the voxels: its surface cells, and its path if it found one.
fn read(points: &[MapPoint], scene: &Scene) -> Value {
    let (planner, path) = plan(points, scene);
    let cells: Vec<[f32; 3]> = planner
        .surface_clearance()
        .into_iter()
        .map(|((ix, iy, iz), z, _)| {
            let (x, y, _) = center(ix, iy, iz);
            [x, y, z]
        })
        .collect();
    json!({
        "cells": cells,
        "path": path.map(|p| p.iter().map(|&(x, y, z)| [x, y, z]).collect::<Vec<_>>()),
    })
}

fn main() {
    let out = std::env::args()
        .nth(1)
        .expect("voxel_problem_scenes <out.json>");
    let result: Vec<Value> = scenes()
        .iter()
        .map(|scene| {
            let points = voxels(scene);
            let (whole, planes) = (read(&whole_voxels(&points), scene), read(&points, scene));
            assert_eq!(
                (whole["path"].is_null(), planes["path"].is_null()),
                (scene.planes_cross, !scene.planes_cross),
                "{}",
                scene.name
            );
            json!({
                "name": scene.name,
                "caption": scene.caption,
                "voxel_size": VOXEL,
                "step_threshold_m": scene.step_threshold_m,
                "voxels": points.iter().map(|&((x, y, z), _)| [x, y, z]).collect::<Vec<_>>(),
                "surface": (0..LENGTH).map(|ix| (scene.surface)(ix).0).collect::<Vec<_>>(),
                "whole_voxels": whole,
                "planes": planes,
            })
        })
        .collect();
    std::fs::write(&out, serde_json::to_string(&result).unwrap()).expect("write scenes");
}
