// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use numpy::ndarray::Array2;
use numpy::{IntoPyArray, PyArray2, PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use validator::Validate;

use crate::edges::edges_to_segments;
use crate::mls_planner::{Config, Planner};
use crate::voxel::surface_point_xyz;

#[pyclass]
pub struct MLSPlanner {
    config: Config,
    planner: Planner,
}

#[pymethods]
impl MLSPlanner {
    #[new]
    #[pyo3(signature = (
        *,
        voxel_size,
        robot_height,
        surface_dilation_passes = 3,
        surface_erosion_passes = 3,
        node_spacing_m = 1.0,
        node_wall_buffer_m = 0.3,
        node_step_threshold_m = 0.25,
    ))]
    fn new(
        voxel_size: f32,
        robot_height: f32,
        surface_dilation_passes: u32,
        surface_erosion_passes: u32,
        node_spacing_m: f32,
        node_wall_buffer_m: f32,
        node_step_threshold_m: f32,
    ) -> PyResult<Self> {
        let config = Config {
            world_frame: String::new(),
            voxel_size,
            robot_height,
            surface_dilation_passes,
            surface_erosion_passes,
            node_spacing_m,
            node_wall_buffer_m,
            node_step_threshold_m,
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
        let pts: Vec<(f32, f32, f32)> = (0..n)
            .filter_map(|i| {
                let x = arr[[i, 0]];
                let y = arr[[i, 1]];
                let z = arr[[i, 2]];
                (x.is_finite() && y.is_finite() && z.is_finite()).then_some((x, y, z))
            })
            .collect();

        let config = &self.config;
        let planner = &mut self.planner;
        py.allow_threads(move || planner.update_global_map(&pts, config));
        Ok(())
    }

    fn surface_map<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f32>> {
        let voxel_size = self.config.voxel_size;
        let surface = self.planner.surface();
        let positions: Vec<f32> = py.allow_threads(|| {
            let mut out: Vec<f32> = Vec::with_capacity(surface.len() * 3);
            for &(ix, iy, iz) in surface {
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

    /// Returns `(W, 3)` float32 waypoints or `None` if no path exists.
    fn plan<'py>(
        &self,
        py: Python<'py>,
        start: (f32, f32, f32),
        goal: (f32, f32, f32),
    ) -> Option<Bound<'py, PyArray2<f32>>> {
        let waypoints = py.allow_threads(|| self.planner.plan(start, goal, &self.config))?;
        let mut flat: Vec<f32> = Vec::with_capacity(waypoints.len() * 3);
        for (x, y, z) in waypoints {
            flat.push(x);
            flat.push(y);
            flat.push(z);
        }
        let n = flat.len() / 3;
        Some(
            Array2::from_shape_vec((n, 3), flat)
                .expect("3 elements pushed per waypoint")
                .into_pyarray(py),
        )
    }

    fn clear(&mut self) {
        self.planner = Planner::default();
    }

    fn __repr__(&self) -> String {
        let graph = self.planner.graph();
        format!(
            "MLSPlanner(voxel_size={}, surface_cells={}, nodes={}, edges={})",
            self.config.voxel_size,
            self.planner.surface().len(),
            graph.nodes.len(),
            graph.node_edges.len(),
        )
    }
}

#[pymodule]
fn dimos_mls_planner(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<MLSPlanner>()?;
    Ok(())
}
