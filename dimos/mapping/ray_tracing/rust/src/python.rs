// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use numpy::ndarray::Array2;
use numpy::{IntoPyArray, PyArray2, PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::voxel_map::{iter_global_points, update_map, Config, LocalBounds, VoxelMap};

#[pyclass]
pub struct VoxelRayMap {
    config: Config,
    map: VoxelMap,
}

#[pymethods]
impl VoxelRayMap {
    #[new]
    #[pyo3(signature = (
        *,
        voxel_size,
        max_range,
        ray_subsample = 1,
        shadow_depth = 0.2,
        grace_depth = 0.2,
        min_health = -2,
        max_health = 1,
    ))]
    fn new(
        voxel_size: f32,
        max_range: f32,
        ray_subsample: u32,
        shadow_depth: f32,
        grace_depth: f32,
        min_health: i32,
        max_health: i32,
    ) -> PyResult<Self> {
        let config = Config {
            voxel_size,
            max_range,
            ray_subsample,
            shadow_depth,
            grace_depth,
            min_health,
            max_health,
        };
        config.validate().map_err(PyValueError::new_err)?;
        Ok(Self {
            config,
            map: VoxelMap::default(),
        })
    }

    fn add_frame(
        &mut self,
        py: Python<'_>,
        points: &Bound<'_, PyAny>,
        origin: (f32, f32, f32),
    ) -> PyResult<()> {
        let points: PyReadonlyArray2<'_, f32> = points
            .extract()
            .map_err(|_| PyValueError::new_err("points must be a (N, 3) float32 numpy array"))?;
        let shape = points.shape();
        if shape.len() != 2 || shape[1] != 3 {
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

        let cfg = &self.config;
        let map = &mut self.map;
        py.allow_threads(move || {
            update_map(map, origin, &pts, cfg);
        });
        Ok(())
    }

    fn global_map<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f32>> {
        let voxel_size = self.config.voxel_size;
        let map = &self.map;
        let positions: Vec<f32> = py.allow_threads(|| {
            let mut out: Vec<f32> = Vec::with_capacity(map.voxels.len() * 3);
            for (x, y, z) in iter_global_points(map, voxel_size) {
                out.push(x);
                out.push(y);
                out.push(z);
            }
            out
        });
        let n = positions.len() / 3;
        Array2::from_shape_vec((n, 3), positions)
            .expect("3 elements pushed per voxel")
            .into_pyarray(py)
    }

    fn local_map<'py>(
        &self,
        py: Python<'py>,
        origin: (f32, f32, f32),
        radius: f32,
        z_min: f32,
        z_max: f32,
    ) -> Bound<'py, PyArray2<f32>> {
        let bounds = LocalBounds {
            origin_x: origin.0,
            origin_y: origin.1,
            r_xy_max_sq: radius * radius,
            z_min,
            z_max,
        };
        let voxel_size = self.config.voxel_size;
        let map = &self.map;
        let positions: Vec<f32> = py.allow_threads(|| {
            let mut out: Vec<f32> = Vec::new();
            for (x, y, z) in iter_global_points(map, voxel_size) {
                if !bounds.contains(x, y, z) {
                    continue;
                }
                out.push(x);
                out.push(y);
                out.push(z);
            }
            out
        });
        let n = positions.len() / 3;
        Array2::from_shape_vec((n, 3), positions)
            .expect("3 elements pushed per voxel")
            .into_pyarray(py)
    }

    fn voxel_count(&self) -> usize {
        self.map.healthy_count()
    }

    fn clear(&mut self) {
        self.map.voxels.clear();
    }

    fn __len__(&self) -> usize {
        self.voxel_count()
    }

    fn __repr__(&self) -> String {
        format!(
            "VoxelRayMap(voxel_size={}, voxels={})",
            self.config.voxel_size,
            self.voxel_count(),
        )
    }
}

#[pymodule]
fn _dimos_voxel_ray_tracing(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<VoxelRayMap>()?;
    Ok(())
}
