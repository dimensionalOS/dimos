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

use crate::mapper::{Mapper, Pose};
use crate::voxel_ray_tracer::{batch_local_bounds, iter_global_normals, Config, LocalBounds};

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

fn points_to_array<'py>(py: Python<'py>, points: &[(f32, f32, f32)]) -> Bound<'py, PyArray2<f32>> {
    let mut out: Vec<f32> = Vec::with_capacity(points.len() * 3);
    for &(x, y, z) in points {
        out.push(x);
        out.push(y);
        out.push(z);
    }
    Array2::from_shape_vec((points.len(), 3), out)
        .expect("3 elements pushed per point")
        .into_pyarray(py)
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
    mapper: Mapper,
}

#[pymethods]
impl VoxelRayMapper {
    #[new]
    #[pyo3(signature = (
        *,
        voxel_size,
        max_range,
        fine_divisor = 0,
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
        fine_divisor: u32,
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
            fine_divisor,
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
            fine_emit_every: 0,
            region_percentile: 95.0,
        };
        config
            .validate()
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(Self {
            mapper: Mapper::new(config),
        })
    }

    #[getter]
    fn voxel_size(&self) -> f32 {
        self.mapper.config().voxel_size
    }

    #[getter]
    fn shadow_depth(&self) -> f32 {
        self.mapper.config().shadow_depth
    }

    /// Register a sensor-frame cloud by the pose, fold it into the map, and
    /// return the registered points as (N, 3) float32.
    fn add_frame<'py>(
        &mut self,
        py: Python<'py>,
        points: &Bound<'py, PyAny>,
        position: (f32, f32, f32),
        orientation: (f32, f32, f32, f32),
    ) -> PyResult<Bound<'py, PyArray2<f32>>> {
        let pts = extract_tuples(points, "points")?;
        let pose = Pose {
            position,
            orientation,
        };
        let mapper = &mut self.mapper;
        let world = py.allow_threads(move || mapper.add_frame(&pts, pose));
        Ok(points_to_array(py, &world))
    }

    /// Cylinder over the frames batched since the last call, as
    /// (cx, cy, radius, z_min, z_max). Consumes the batch.
    fn take_local_bounds(&mut self) -> (f32, f32, f32, f32, f32) {
        self.mapper.take_local_bounds()
    }

    fn global_map<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f32>> {
        let mapper = &self.mapper;
        let points = py.allow_threads(|| mapper.global_points());
        points_to_array(py, &points)
    }

    /// Healthy voxel centers and their surface normals, both (M, 3) float32 in
    /// matching order. The normal is the zero vector where there is no plane.
    fn global_map_normals<'py>(
        &self,
        py: Python<'py>,
    ) -> (Bound<'py, PyArray2<f32>>, Bound<'py, PyArray2<f32>>) {
        let mapper = &self.mapper;
        let (positions, normals): (Vec<f32>, Vec<f32>) = py.allow_threads(|| {
            let map = mapper.map();
            let mut positions: Vec<f32> = Vec::with_capacity(map.voxels.len() * 3);
            let mut normals: Vec<f32> = Vec::with_capacity(map.voxels.len() * 3);
            for ((x, y, z), n) in iter_global_normals(map, mapper.config().voxel_size) {
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
        let mapper = &self.mapper;
        let points = py.allow_threads(|| mapper.local_points(&bounds));
        points_to_array(py, &points)
    }

    /// Fine-cell centers inside the cylinder as (M, 3) float32.
    fn local_map_fine<'py>(
        &self,
        py: Python<'py>,
        origin: (f32, f32, f32),
        radius: f32,
        z_min: f32,
        z_max: f32,
    ) -> PyResult<Bound<'py, PyArray2<f32>>> {
        let bounds = LocalBounds {
            origin_x: origin.0,
            origin_y: origin.1,
            r_xy_max_sq: radius * radius,
            z_min,
            z_max,
        };
        let mapper = &self.mapper;
        let points = py
            .allow_threads(|| mapper.fine_points(&bounds))
            .ok_or_else(|| PyValueError::new_err("fine_divisor is not set"))?;
        Ok(points_to_array(py, &points))
    }

    fn voxel_count(&self) -> usize {
        self.mapper.map().healthy_count()
    }

    fn clear(&mut self) {
        self.mapper.clear();
    }

    fn __len__(&self) -> usize {
        self.voxel_count()
    }

    fn __repr__(&self) -> String {
        format!(
            "VoxelRayMapper(voxel_size={}, voxels={})",
            self.mapper.config().voxel_size,
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
