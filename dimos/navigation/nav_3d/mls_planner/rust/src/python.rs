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

use numpy::ndarray::Array2;
use numpy::{IntoPyArray, PyArray2, PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use validator::Validate;

use crate::bridge::Bridge;
use crate::edges::edges_to_segments;
use crate::mls_planner::{Config, Planner, RegionBounds};
use crate::voxel::{surface_point_xyz, VoxelKey};

#[pyclass]
pub struct MLSPlanner {
    config: Config,
    planner: Planner,
}

/// Extract a (N, 3) float32 numpy array into xyz tuples, dropping any row with
/// a non-finite coordinate.
fn extract_points(points: &Bound<'_, PyAny>) -> PyResult<Vec<(f32, f32, f32)>> {
    let points: PyReadonlyArray2<'_, f32> = points
        .extract()
        .map_err(|_| PyValueError::new_err("points must be a (N, 3) float32 numpy array"))?;
    let shape = points.shape();
    if shape[1] != 3 {
        return Err(PyValueError::new_err(format!(
            "points must be (N, 3) float32, got shape {:?}",
            shape
        )));
    }
    let arr = points.as_array();
    let n = shape[0];
    Ok((0..n)
        .filter_map(|i| {
            let x = arr[[i, 0]];
            let y = arr[[i, 1]];
            let z = arr[[i, 2]];
            (x.is_finite() && y.is_finite() && z.is_finite()).then_some((x, y, z))
        })
        .collect())
}

/// Pack waypoints into a (W, 3) float32 numpy array.
fn waypoint_array(py: Python<'_>, waypoints: Vec<(f32, f32, f32)>) -> Bound<'_, PyArray2<f32>> {
    let mut flat: Vec<f32> = Vec::with_capacity(waypoints.len() * 3);
    for (x, y, z) in waypoints {
        flat.push(x);
        flat.push(y);
        flat.push(z);
    }
    let n = flat.len() / 3;
    Array2::from_shape_vec((n, 3), flat)
        .expect("3 elements pushed per waypoint")
        .into_pyarray(py)
}

#[pymethods]
impl MLSPlanner {
    #[new]
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (
        *,
        voxel_size,
        robot_height,
        max_overhead_m = 2.0,
        surface_closing_radius = 0.3,
        node_spacing_m = 1.0,
        wall_clearance_m = 0.1,
        wall_buffer_m = 0.75,
        wall_buffer_weight = 100.0,
        step_threshold_m = 0.16,
        step_penalty_weight = 4.0,
        bridge_max_hop_m = 1.0,
        bridge_max_grade = 0.85,
        bridge_cost_weight = 10.0,
        bridge_switch_margin = 0.25,
    ))]
    fn new(
        voxel_size: f32,
        robot_height: f32,
        max_overhead_m: f32,
        surface_closing_radius: f32,
        node_spacing_m: f32,
        wall_clearance_m: f32,
        wall_buffer_m: f32,
        wall_buffer_weight: f32,
        step_threshold_m: f32,
        step_penalty_weight: f32,
        bridge_max_hop_m: f32,
        bridge_max_grade: f32,
        bridge_cost_weight: f32,
        bridge_switch_margin: f32,
    ) -> PyResult<Self> {
        let config = Config {
            world_frame: String::new(),
            voxel_size,
            robot_height,
            max_overhead_m,
            surface_closing_radius,
            node_spacing_m,
            wall_clearance_m,
            wall_buffer_m,
            wall_buffer_weight,
            step_threshold_m,
            step_penalty_weight,
            bridge_max_hop_m,
            bridge_max_grade,
            bridge_cost_weight,
            bridge_switch_margin,
            // Unused here. Only the binary's replan loop reads goal_tolerance.
            goal_tolerance: 1.0,
            // Unused here. Only the binary's worker publishes viz artifacts.
            viz_publish_hz: 1.0,
        };
        config
            .validate()
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(Self {
            config,
            planner: Planner::default(),
        })
    }

    fn update_global_map(&mut self, py: Python<'_>, points: &Bound<'_, PyAny>) -> PyResult<()> {
        let pts = extract_points(points)?;
        let config = &self.config;
        let planner = &mut self.planner;
        py.allow_threads(move || planner.update_global_map(&pts, config));
        Ok(())
    }

    #[pyo3(signature = (points, origin, radius, z_min, z_max, sensor_z))]
    #[allow(clippy::too_many_arguments)]
    fn update_region(
        &mut self,
        py: Python<'_>,
        points: &Bound<'_, PyAny>,
        origin: (f32, f32),
        radius: f32,
        z_min: f32,
        z_max: f32,
        sensor_z: f32,
    ) -> PyResult<()> {
        let pts = extract_points(points)?;
        let bounds = RegionBounds::capped(
            origin.0,
            origin.1,
            radius,
            z_min,
            z_max,
            sensor_z,
            self.config.max_overhead_m,
        );
        let config = &self.config;
        let planner = &mut self.planner;
        py.allow_threads(move || planner.update_region(&pts, &bounds, config));
        Ok(())
    }

    fn surface_map<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f32>> {
        let voxel_size = self.config.voxel_size;
        let surface: Vec<VoxelKey> = self.planner.surface().collect();
        let positions: Vec<f32> = py.allow_threads(|| {
            let mut out: Vec<f32> = Vec::with_capacity(surface.len() * 3);
            for (ix, iy, iz) in surface {
                let (x, y, z) = surface_point_xyz(ix, iy, iz, voxel_size);
                out.push(x);
                out.push(y);
                out.push(z);
            }
            out
        });
        let n = positions.len() / 3;
        Array2::from_shape_vec((n, 3), positions)
            .expect("3 elements pushed per cell")
            .into_pyarray(py)
    }

    /// Surface cells as (M, 4) float32 rows of x, y, z, clearance, where
    /// clearance is the distance to the nearest untraversable edge.
    fn surface_clearance_map<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f32>> {
        let voxel_size = self.config.voxel_size;
        let cells = self.planner.surface_clearance();
        let values: Vec<f32> = py.allow_threads(|| {
            let mut out: Vec<f32> = Vec::with_capacity(cells.len() * 4);
            for ((ix, iy, iz), clearance) in cells {
                let (x, y, z) = surface_point_xyz(ix, iy, iz, voxel_size);
                out.push(x);
                out.push(y);
                out.push(z);
                out.push(clearance);
            }
            out
        });
        let n = values.len() / 4;
        Array2::from_shape_vec((n, 4), values)
            .expect("4 elements pushed per cell")
            .into_pyarray(py)
    }

    fn nodes<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f32>> {
        let graph = self.planner.graph();
        let positions: Vec<f32> = py.allow_threads(|| {
            let mut out: Vec<f32> = Vec::with_capacity(graph.nodes.len() * 3);
            for n in &graph.nodes {
                out.push(n.pos.0);
                out.push(n.pos.1);
                out.push(n.pos.2);
            }
            out
        });
        let n = positions.len() / 3;
        Array2::from_shape_vec((n, 3), positions)
            .expect("3 elements pushed per node")
            .into_pyarray(py)
    }

    /// Each row is `[x0, y0, z0, x1, y1, z1, cost]`.
    fn node_edges<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f32>> {
        let voxel_size = self.config.voxel_size;
        let graph = self.planner.graph();
        let values: Vec<f32> = py.allow_threads(|| {
            let segments = edges_to_segments(&graph.cells, &graph.cell_state, &graph.node_edges);
            let mut out: Vec<f32> = Vec::with_capacity(segments.len() * 7);
            for (a, b, cost) in segments {
                let pa = surface_point_xyz(a.0, a.1, a.2, voxel_size);
                let pb = surface_point_xyz(b.0, b.1, b.2, voxel_size);
                out.extend_from_slice(&[pa.0, pa.1, pa.2, pb.0, pb.1, pb.2, cost]);
            }
            out
        });
        let n = values.len() / 7;
        Array2::from_shape_vec((n, 7), values)
            .expect("7 elements pushed per segment")
            .into_pyarray(py)
    }

    /// Returns `(W, 3)` float32 waypoints or `None` if no full path exists.
    fn plan<'py>(
        &self,
        py: Python<'py>,
        start: (f32, f32, f32),
        goal: (f32, f32, f32),
    ) -> Option<Bound<'py, PyArray2<f32>>> {
        let waypoints = py.allow_threads(|| self.planner.plan(start, goal, &self.config))?;
        Some(waypoint_array(py, waypoints))
    }

    /// Plan with the robot's replan policy: a full plan, else the safe prefix
    /// of the cached path to the same goal. Returns `(W, 3)` float32 waypoints,
    /// empty when nothing ahead is traversable (stop).
    fn plan_or_truncate<'py>(
        &mut self,
        py: Python<'py>,
        start: (f32, f32, f32),
        goal: (f32, f32, f32),
    ) -> Bound<'py, PyArray2<f32>> {
        let config = &self.config;
        let planner = &mut self.planner;
        let waypoints = py.allow_threads(move || planner.plan_or_truncate(start, goal, config));
        waypoint_array(py, waypoints)
    }

    /// Optimistic bridge of the last plan_or_truncate call: `(route_end,
    /// approach_point)` world-frame xyz tuples, or `None` when the last plan
    /// reached the goal.
    fn frontier_bridge(&self) -> Option<Bridge> {
        self.planner.frontier_bridge()
    }

    fn voxel_count(&self) -> usize {
        self.planner.voxel_count()
    }

    /// Accumulated occupied voxel centers as (N, 3) float32, for visualization.
    fn voxel_map<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f32>> {
        let vs = self.config.voxel_size;
        let half = vs * 0.5;
        let keys: Vec<(i32, i32, i32)> = self.planner.voxel_keys().collect();
        let positions: Vec<f32> = py.allow_threads(|| {
            let mut out: Vec<f32> = Vec::with_capacity(keys.len() * 3);
            for (kx, ky, kz) in keys {
                out.push(kx as f32 * vs + half);
                out.push(ky as f32 * vs + half);
                out.push(kz as f32 * vs + half);
            }
            out
        });
        let n = positions.len() / 3;
        Array2::from_shape_vec((n, 3), positions)
            .expect("3 elements pushed per voxel")
            .into_pyarray(py)
    }

    fn clear(&mut self) {
        self.planner = Planner::default();
    }

    fn __repr__(&self) -> String {
        let graph = self.planner.graph();
        format!(
            "MLSPlanner(voxel_size={}, surface_cells={}, nodes={}, edges={})",
            self.config.voxel_size,
            self.planner.surface().count(),
            graph.nodes.len(),
            graph.node_edges.len(),
        )
    }
}

#[pymodule]
fn dimos_mls_planner(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Log planner tracing to stderr. Defaults to warn, override with RUST_LOG.
    let filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("dimos_mls_planner=warn"));
    let _ = tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_writer(std::io::stderr)
        .try_init();
    m.add_class::<MLSPlanner>()?;
    Ok(())
}
