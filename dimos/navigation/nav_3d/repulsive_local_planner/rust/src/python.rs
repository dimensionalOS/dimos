// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! pyo3 bindings for offline parity tests against the Python reference planner.

use numpy::PyReadonlyArray2;
use pyo3::prelude::*;

use crate::costmap::{self, CostmapConfig};
use crate::solver::{self, SolverConfig};

/// Build a costmap from Nx3 points and plan; returns the (x, y, yaw) poses.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
fn plan_once(
    points: PyReadonlyArray2<f32>,
    robot: (f32, f32, f32),
    robot_z: f32,
    global_path: Vec<(f32, f32)>,
    tail: Vec<(f32, f32)>,
    speed: f32,
    resolution: f32,
) -> PyResult<Vec<(f32, f32, f32)>> {
    let pts: Vec<[f32; 3]> = points
        .as_array()
        .rows()
        .into_iter()
        .map(|r| [r[0], r[1], r[2]])
        .collect();
    let ccfg = CostmapConfig {
        resolution,
        ..CostmapConfig::default()
    };
    let scfg = SolverConfig::default();
    let map = costmap::build(&pts, (robot.0, robot.1, robot_z), robot_z, &ccfg);
    let ext = solver::carrot_extension(&global_path, &tail, &scfg);
    let plan = solver::plan(&map, &global_path, &ext, robot, speed, None, &scfg);
    Ok(plan.poses)
}

/// plan_once with a previous path for commitment/hysteresis chaining.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
fn plan_once_prev(
    points: PyReadonlyArray2<f32>,
    robot: (f32, f32, f32),
    robot_z: f32,
    global_path: Vec<(f32, f32)>,
    tail: Vec<(f32, f32)>,
    speed: f32,
    resolution: f32,
    previous: Vec<(f32, f32)>,
) -> PyResult<Vec<(f32, f32, f32)>> {
    let pts: Vec<[f32; 3]> = points
        .as_array()
        .rows()
        .into_iter()
        .map(|r| [r[0], r[1], r[2]])
        .collect();
    let ccfg = CostmapConfig {
        resolution,
        ..CostmapConfig::default()
    };
    let scfg = SolverConfig::default();
    let map = costmap::build(&pts, (robot.0, robot.1, robot_z), robot_z, &ccfg);
    let ext = solver::carrot_extension(&global_path, &tail, &scfg);
    let prev_opt = (previous.len() >= 2).then_some(previous.as_slice());
    let plan = solver::plan(&map, &global_path, &ext, robot, speed, prev_opt, &scfg);
    Ok(plan.poses)
}

#[pymodule]
fn dimos_repulsive_field(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(plan_once, m)?)?;
    m.add_function(wrap_pyfunction!(plan_once_prev, m)?)?;
    Ok(())
}
