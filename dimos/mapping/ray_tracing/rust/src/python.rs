// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use numpy::ndarray::Array2;
use numpy::{IntoPyArray, PyArray2, PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use validator::Validate;

use ahash::AHashSet;

use crate::voxel_ray_tracer::{
    batch_local_bounds, emit_points, iter_global_normals, update_map, Config, LocalBounds,
    VoxelKey, VoxelMap,
};

fn extract_tuples(arr: &Bound<'_, PyAny>, name: &str) -> PyResult<Vec<(f32, f32, f32)>> {
    let arr: PyReadonlyArray2<'_, f32> = arr.extract().map_err(|_| {
        PyValueError::new_err(format!("{name} must be a (N, 3) float32 numpy array"))
    })?;
    let shape = arr.shape();
    if shape[1] != 3 {
        return Err(PyValueError::new_err(format!(
            "{name} must be (N, 3) float32, got shape {:?}",
            shape
        )));
    }
    let view = arr.as_array();
    Ok((0..shape[0])
        .filter_map(|i| {
            let x = view[[i, 0]];
            let y = view[[i, 1]];
            let z = view[[i, 2]];
            (x.is_finite() && y.is_finite() && z.is_finite()).then_some((x, y, z))
        })
        .collect())
}

/// Local region a batch of frames observed, as (cx, cy, radius, z_min, z_max).
/// Non-finite points are ignored.
#[pyfunction]
fn local_bounds(
    points: &Bound<'_, PyAny>,
    origins: &Bound<'_, PyAny>,
    percentile: f32,
    margin: f32,
) -> PyResult<(f32, f32, f32, f32, f32)> {
    let pts = extract_tuples(points, "points")?;
    let origs = extract_tuples(origins, "origins")?;
    Ok(batch_local_bounds(&pts, &origs, percentile, margin))
}

#[pyclass]
pub struct VoxelRayMapper {
    config: Config,
    map: VoxelMap,
    // Voxels hit in current lidar frame
    live: AHashSet<VoxelKey>,
}

#[pymethods]
impl VoxelRayMapper {
    #[new]
    #[pyo3(signature = (
        *,
        voxel_size,
        max_range,
        ray_subsample = 1,
        shadow_depth = 0.1,
        grace_depth = 0.2,
        min_health = -1,
        max_health = 5,
        graze_cos = 0.7,
        support_min = 4,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        voxel_size: f32,
        max_range: f32,
        ray_subsample: u32,
        shadow_depth: f32,
        grace_depth: f32,
        min_health: i32,
        max_health: i32,
        graze_cos: f32,
        support_min: i32,
    ) -> PyResult<Self> {
        let config = Config {
            voxel_size,
            max_range,
            ray_subsample,
            shadow_depth,
            grace_depth,
            min_health,
            max_health,
            graze_cos,
            support_min,
            emit_every: 1,
            global_emit_every: 1,
            region_percentile: 95.0,
        };
        config
            .validate()
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(Self {
            config,
            map: VoxelMap::default(),
            live: AHashSet::new(),
        })
    }

    #[getter]
    fn voxel_size(&self) -> f32 {
        self.config.voxel_size
    }

    #[getter]
    fn shadow_depth(&self) -> f32 {
        self.config.shadow_depth
    }

    fn add_frame(
        &mut self,
        py: Python<'_>,
        points: &Bound<'_, PyAny>,
        origin: (f32, f32, f32),
    ) -> PyResult<()> {
        let pts = extract_tuples(points, "points")?;

        let cfg = &self.config;
        let map = &mut self.map;
        self.live = py.allow_threads(move || update_map(map, origin, &pts, cfg));
        Ok(())
    }

    fn global_map<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f32>> {
        let voxel_size = self.config.voxel_size;
        let map = &self.map;
        let live = &self.live;
        let positions: Vec<f32> = py.allow_threads(|| {
            let mut out: Vec<f32> = Vec::with_capacity(map.voxels.len() * 3);
            for (x, y, z) in emit_points(map, voxel_size, None, 0, live) {
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

    /// Healthy voxel centers and their surface normals, both (M, 3) float32 in
    /// matching order. The normal is the zero vector where there is no plane.
    fn global_map_normals<'py>(
        &self,
        py: Python<'py>,
    ) -> (Bound<'py, PyArray2<f32>>, Bound<'py, PyArray2<f32>>) {
        let voxel_size = self.config.voxel_size;
        let map = &self.map;
        let (positions, normals): (Vec<f32>, Vec<f32>) = py.allow_threads(|| {
            let mut positions: Vec<f32> = Vec::with_capacity(map.voxels.len() * 3);
            let mut normals: Vec<f32> = Vec::with_capacity(map.voxels.len() * 3);
            for ((x, y, z), n) in iter_global_normals(map, voxel_size) {
                positions.push(x);
                positions.push(y);
                positions.push(z);
                normals.extend_from_slice(&n);
            }
            (positions, normals)
        });
        let m = positions.len() / 3;
        let positions = Array2::from_shape_vec((m, 3), positions)
            .expect("3 elements pushed per voxel")
            .into_pyarray(py);
        let normals = Array2::from_shape_vec((m, 3), normals)
            .expect("3 elements pushed per voxel")
            .into_pyarray(py);
        (positions, normals)
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
        let support_min = self.config.support_min;
        let map = &self.map;
        let live = &self.live;
        let positions: Vec<f32> = py.allow_threads(|| {
            let mut out: Vec<f32> = Vec::new();
            for (x, y, z) in emit_points(map, voxel_size, Some(&bounds), support_min, live) {
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
        self.live.clear();
    }

    fn __len__(&self) -> usize {
        self.voxel_count()
    }

    fn __repr__(&self) -> String {
        format!(
            "VoxelRayMapper(voxel_size={}, voxels={})",
            self.config.voxel_size,
            self.voxel_count(),
        )
    }
}

#[pymodule]
fn dimos_voxel_ray_tracing(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<VoxelRayMapper>()?;
    m.add_function(wrap_pyfunction!(local_bounds, m)?)?;
    Ok(())
}
