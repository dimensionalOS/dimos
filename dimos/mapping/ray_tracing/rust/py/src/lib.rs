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
use numpy::{IntoPyArray, PyArray1, PyArray2, PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use validator::Validate;

use dimos_voxel_ray_tracing::mapper::{Mapper, Pose};
use dimos_voxel_ray_tracing::reform::{
    decode_loop_closure_event, decode_pose_graph, record_cloud, ReformParams, ReformState,
    LOOP_EVENT_FIELDS, POSE_GRAPH_FIELDS,
};
use dimos_voxel_ray_tracing::voxel_ray_tracer::{
    partition_seed, Config, LocalBounds, SeedPartition,
};
use pyo3::types::PyDict;

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
    let finite = |x: f32, y: f32, z: f32| {
        (x.is_finite() && y.is_finite() && z.is_finite()).then_some((x, y, z))
    };
    // Contiguous input skips per-element bounds checks.
    if let Some(slice) = view.as_slice() {
        return Ok(slice
            .as_chunks::<3>()
            .0
            .iter()
            .filter_map(|p| finite(p[0], p[1], p[2]))
            .collect());
    }
    Ok((0..shape[0])
        .filter_map(|i| finite(view[[i, 0]], view[[i, 1]], view[[i, 2]]))
        .collect())
}

fn points_to_array<'py>(py: Python<'py>, points: &[(f32, f32, f32)]) -> Bound<'py, PyArray2<f32>> {
    let mut out: Vec<f32> = Vec::with_capacity(points.len() * 3);
    for &(x, y, z) in points {
        out.extend_from_slice(&[x, y, z]);
    }
    Array2::from_shape_vec((points.len(), 3), out)
        .expect("3 elements pushed per point")
        .into_pyarray(py)
}

/// Centers, normals, and smallest eigenvalues from the viz fit query.
type NormalFitArrays<'py> = (
    Bound<'py, PyArray2<f32>>,
    Bound<'py, PyArray2<f32>>,
    Bound<'py, PyArray1<f32>>,
);

/// Zero-copy hand-off of flat (x, y, z) triples to a (N, 3) numpy array.
fn flat_to_array(py: Python<'_>, points: Vec<f32>) -> Bound<'_, PyArray2<f32>> {
    let n = points.len() / 3;
    Array2::from_shape_vec((n, 3), points)
        .expect("emitters produce whole triples")
        .into_pyarray(py)
}

#[pyclass]
pub struct VoxelRayMapper {
    mapper: Mapper,
    // A region-by-region seed in progress, as the module applies one.
    seed: Option<(SeedPartition, usize)>,
    // Keyframes and their applied poses, for loop closures (reform_ray_tracing).
    reform: ReformState,
}

/// One region of a seed as the map now holds it: the cylinder covering its
/// chunks and the support-gated (M, 3) float32 points inside.
#[pyclass(frozen, get_all)]
pub struct SeededRegion {
    center: (f32, f32),
    radius: f32,
    z_min: f32,
    z_max: f32,
    points: Py<PyArray2<f32>>,
}

#[pymethods]
impl VoxelRayMapper {
    #[new]
    #[pyo3(signature = (
        *,
        voxel_size,
        max_range,
        fine_divisor = 3,
        ray_subsample = 1,
        shadow_depth = 0.1,
        grace_depth = 0.2,
        min_health = -1,
        max_health = 5,
        graze_cos = 0.7,
        support_min = 4,
        region_percentile = 95.0,
        worker_threads = 4,
        emit_every = 0,
        region_m = 4.0,
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
        region_percentile: f32,
        worker_threads: u32,
        emit_every: u32,
        region_m: f32,
    ) -> PyResult<Self> {
        // A nonzero emit_every batches frames for take_local_bounds. Callers
        // that never take must leave it 0 or the batch grows forever.
        // emit_fine only gates the module's publishing. This API queries fine
        // points explicitly, so it stays off.
        let config = Config {
            voxel_size,
            fine_divisor,
            emit_fine: false,
            max_range,
            ray_subsample,
            shadow_depth,
            grace_depth,
            min_health,
            max_health,
            graze_cos,
            support_min,
            emit_every,
            global_emit_every: 1,
            region_percentile,
            world_frame: "world".to_string(),
            tf_match_tolerance_s: 0.1,
            max_cloud_age_s: 0.0,
            tf_wait_timeout_s: 0.05,
            worker_threads,
            region_m,
            viz_cell_m: region_m,
            viz_emit_every: 0,
            viz_sweep_cells: 0,
        };
        config
            .validate()
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(Self {
            mapper: Mapper::new(config),
            seed: None,
            reform: ReformState::default(),
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

    /// Register a sensor-frame cloud by the pose and fold it into the map.
    fn add_frame(
        &mut self,
        py: Python<'_>,
        points: &Bound<'_, PyAny>,
        position: (f32, f32, f32),
        orientation: (f32, f32, f32, f32),
    ) -> PyResult<()> {
        let pts = extract_tuples(points, "points")?;
        let pose = Pose {
            position,
            orientation,
        };
        let mapper = &mut self.mapper;
        py.allow_threads(move || mapper.add_frame(pts, pose));
        Ok(())
    }

    /// Fold an already world-frame cloud into the map, raycasting from origin.
    fn add_frame_world(
        &mut self,
        py: Python<'_>,
        points: &Bound<'_, PyAny>,
        origin: (f32, f32, f32),
    ) -> PyResult<()> {
        let pts = extract_tuples(points, "points")?;
        let mapper = &mut self.mapper;
        py.allow_threads(move || mapper.add_frame_world(pts, origin));
        Ok(())
    }

    /// Record when each voxel lands, for loop closures, as reform_ray_tracing does.
    fn track_voxel_times(&mut self) {
        self.mapper.track_voxel_times();
    }

    /// The time (s) the next frames' new voxels are stamped with.
    fn set_clock(&mut self, t: f64) {
        self.mapper.set_clock(t);
    }

    /// Take keyframes from raw pose_graph records (40 bytes each). Keyframes
    /// already known keep the pose the map holds them at.
    fn add_keyframes(&mut self, records: Vec<u8>) -> PyResult<usize> {
        if !records.len().is_multiple_of(40) {
            return Err(PyValueError::new_err(
                "pose_graph records are 40 bytes each",
            ));
        }
        let msg = record_cloud(POSE_GRAPH_FIELDS, records);
        let records = decode_pose_graph(&msg).map_err(|e| PyValueError::new_err(e.0))?;
        self.reform.add_keyframes(&records);
        Ok(self.reform.len())
    }

    /// Warp the map through raw loop_closure_event records (68 bytes each), as
    /// reform_ray_tracing does. None when no keyframe moved past min_shift_m,
    /// else a dict of what the warp did and cost.
    #[pyo3(signature = (records, blend_sigma_m = 0.0, blend_time_sigma_s = 0.0, min_shift_m = 0.04, thin_cell_m = 0.0))]
    #[allow(clippy::too_many_arguments)]
    fn apply_loop_closure<'py>(
        &mut self,
        py: Python<'py>,
        records: Vec<u8>,
        blend_sigma_m: f64,
        blend_time_sigma_s: f64,
        min_shift_m: f64,
        thin_cell_m: f64,
    ) -> PyResult<Option<Bound<'py, PyDict>>> {
        if !records.len().is_multiple_of(68) {
            return Err(PyValueError::new_err(
                "loop_closure_event records are 68 bytes each",
            ));
        }
        let msg = record_cloud(LOOP_EVENT_FIELDS, records);
        let moved = decode_loop_closure_event(&msg).map_err(|e| PyValueError::new_err(e.0))?;
        let params = ReformParams {
            blend_sigma_m,
            blend_time_sigma_s,
            min_shift_m,
            thin_cell_m,
        };
        let Some(warp) = self.reform.closure(&moved, &params) else {
            return Ok(None);
        };
        let mapper = &mut self.mapper;
        let report = py.allow_threads(|| mapper.apply_warp(&warp));
        let out = PyDict::new(py);
        out.set_item("voxels_warped", report.voxels_warped)?;
        out.set_item("voxels_rebinned", report.voxels_rebinned)?;
        out.set_item("warp_ms", report.warp_ms)?;
        out.set_item("voxel_ms", report.voxel_ms)?;
        out.set_item("keyframes_moved", warp.moved_keyframes())?;
        out.set_item("keyframes_blended", warp.blended_keyframes())?;
        Ok(Some(out))
    }

    /// Every healthy voxel's center, (N, 3) float32: what reform_ray_tracing dumps.
    fn healthy_centers<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f32>> {
        flat_to_array(py, self.mapper.healthy_centers())
    }

    /// Bulk-seed a world-frame map cloud, creating only absent voxels.
    /// Returns how many voxels were created.
    fn seed_points(&mut self, py: Python<'_>, points: &Bound<'_, PyAny>) -> PyResult<usize> {
        let pts = extract_tuples(points, "points")?;
        let mapper = &mut self.mapper;
        Ok(py.allow_threads(move || mapper.seed_points(&pts)))
    }

    /// The last frame's registered points as (N, 3) float32.
    fn registered_points<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f32>> {
        points_to_array(py, self.mapper.registered_points())
    }

    /// Cylinder over the frames batched since the last call, as
    /// (cx, cy, radius, z_min, z_max). Consumes the batch.
    fn take_local_bounds(&mut self) -> (f32, f32, f32, f32, f32) {
        let c = self.mapper.take_local_bounds();
        (c.cx, c.cy, c.radius, c.z_min, c.z_max)
    }

    fn global_map<'py>(&self, py: Python<'py>) -> Bound<'py, PyArray2<f32>> {
        let mapper = &self.mapper;
        let points = py.allow_threads(|| mapper.global_points());
        flat_to_array(py, points)
    }

    /// Partition a world-frame map cloud into regions nearest `origin` first,
    /// ready for seed_next_region. Returns the region count.
    fn start_seed(
        &mut self,
        py: Python<'_>,
        points: &Bound<'_, PyAny>,
        origin: (f32, f32, f32),
    ) -> PyResult<usize> {
        let pts = extract_tuples(points, "points")?;
        let (voxel_size, region_m) = {
            let config = self.mapper.config();
            (config.voxel_size, config.region_m)
        };
        let part = py.allow_threads(|| partition_seed(&pts, voxel_size, origin, region_m));
        self.mapper.reserve_chunks(part.tile_count());
        let regions = part.regions.len();
        self.seed = Some((part, 0));
        Ok(regions)
    }

    /// Seed the next pending region and return it as the map now holds it.
    /// None once every region has landed.
    fn seed_next_region(&mut self, py: Python<'_>) -> Option<SeededRegion> {
        let (part, next) = self.seed.as_mut()?;
        let Some(region) = part.regions.get(*next) else {
            self.seed = None;
            return None;
        };
        *next += 1;
        let mapper = &mut self.mapper;
        let points = py.allow_threads(|| {
            for tile in &region.tiles {
                mapper.seed_tile(tile);
            }
            mapper.local_points(&region.cylinder.bounds())
        });
        let c = region.cylinder;
        Some(SeededRegion {
            center: (c.cx, c.cy),
            radius: c.radius,
            z_min: c.z_min,
            z_max: c.z_max,
            points: flat_to_array(py, points).unbind(),
        })
    }

    /// Healthy voxel centers and their surface normals, both (M, 3) float32 in
    /// matching order. The normal is the zero vector where there is no plane.
    fn global_map_normals<'py>(
        &self,
        py: Python<'py>,
    ) -> (Bound<'py, PyArray2<f32>>, Bound<'py, PyArray2<f32>>) {
        let mapper = &self.mapper;
        let (positions, normals) = py.allow_threads(|| mapper.normals());
        let m = positions.len() / 3;
        let positions = Array2::from_shape_vec((m, 3), positions)
            .expect("3 elements pushed per voxel")
            .into_pyarray(py);
        let normals = Array2::from_shape_vec((m, 3), normals)
            .expect("3 elements pushed per voxel")
            .into_pyarray(py);
        (positions, normals)
    }

    /// global_map_normals with freshly recomputed fits, plus each fit's
    /// smallest eigenvalue as (M,). Whole-map refit cost, visualization only.
    fn global_map_normal_fits<'py>(&self, py: Python<'py>) -> NormalFitArrays<'py> {
        let mapper = &self.mapper;
        let (positions, normals, eigs) = py.allow_threads(|| mapper.normal_fits());
        let m = eigs.len();
        let positions = Array2::from_shape_vec((m, 3), positions)
            .expect("3 elements pushed per voxel")
            .into_pyarray(py);
        let normals = Array2::from_shape_vec((m, 3), normals)
            .expect("3 elements pushed per voxel")
            .into_pyarray(py);
        let eigs = eigs.into_pyarray(py);
        (positions, normals, eigs)
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
        flat_to_array(py, points)
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
        Ok(flat_to_array(py, points))
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

/// One closure's fast-reform warp on its own, for scoring a map against a rebuild
/// placed by the same blend.
#[pyclass]
struct ReformWarp {
    warp: dimos_voxel_ray_tracing::reform::Warp,
}

#[pymethods]
impl ReformWarp {
    /// The warp that takes keyframes from their pose_graph records (40 bytes
    /// each) to the post poses of loop_closure_event records (68 bytes each).
    #[new]
    #[pyo3(signature = (keyframes, closure, blend_sigma_m = 0.0, blend_time_sigma_s = 0.0, min_shift_m = 0.0))]
    fn new(
        keyframes: Vec<u8>,
        closure: Vec<u8>,
        blend_sigma_m: f64,
        blend_time_sigma_s: f64,
        min_shift_m: f64,
    ) -> PyResult<Self> {
        if !keyframes.len().is_multiple_of(40) || !closure.len().is_multiple_of(68) {
            return Err(PyValueError::new_err(
                "pose_graph records are 40 bytes, loop_closure_event records 68",
            ));
        }
        let keyframes = decode_pose_graph(&record_cloud(POSE_GRAPH_FIELDS, keyframes))
            .map_err(|e| PyValueError::new_err(e.0))?;
        let moved = decode_loop_closure_event(&record_cloud(LOOP_EVENT_FIELDS, closure))
            .map_err(|e| PyValueError::new_err(e.0))?;
        let mut state = ReformState::default();
        state.add_keyframes(&keyframes);
        let params = ReformParams {
            blend_sigma_m,
            blend_time_sigma_s,
            min_shift_m,
            thin_cell_m: -1.0,
        };
        let warp = state
            .closure(&moved, &params)
            .ok_or_else(|| PyValueError::new_err("no keyframe moved past min_shift_m"))?;
        Ok(Self { warp })
    }

    /// Warp (N, 3) float32 points all observed at time t.
    fn apply<'py>(
        &self,
        py: Python<'py>,
        points: PyReadonlyArray2<'py, f32>,
        t: f64,
    ) -> PyResult<Bound<'py, PyArray2<f32>>> {
        let flat: Vec<[f32; 3]> = points
            .as_array()
            .rows()
            .into_iter()
            .map(|r| [r[0], r[1], r[2]])
            .collect();
        let times = vec![t; flat.len()];
        let warp = &self.warp;
        let out = py.allow_threads(|| warp.apply(&flat, &times));
        Ok(flat_to_array(py, out.into_iter().flatten().collect()))
    }
}

#[pymodule(name = "dimos_voxel_ray_tracing")]
fn dimos_voxel_ray_tracing_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<VoxelRayMapper>()?;
    m.add_class::<SeededRegion>()?;
    m.add_class::<ReformWarp>()?;
    Ok(())
}
