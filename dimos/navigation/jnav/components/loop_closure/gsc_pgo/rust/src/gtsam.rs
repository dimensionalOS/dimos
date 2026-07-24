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

//! Safe Rust wrappers over the gtsam C shim (shim/gtsam_shim.{h,cpp}).
//!
//! The surface mirrors exactly what the gsc_pgo C++ core uses: an `Isam2`
//! configured like `SimplePGO::SimplePGO` (relinearizeThreshold=0.01,
//! relinearizeSkip=1), a `FactorGraph` holding Pose3 prior/between factors,
//! `Values` for initial estimates and best-estimate readback, and the
//! diagonal / gaussian-covariance / robust-Huber noise models.
//!
//! All handles are RAII (`Drop` frees the C++ object). The raw pointers make
//! every type `!Send`/`!Sync`, which matches gtsam's thread-unsafety.

use std::os::raw::{c_char, c_void};

mod ffi {
    use std::os::raw::{c_char, c_void};

    extern "C" {
        pub fn gtsam_shim_symbol_key(chr: c_char, index: u64) -> u64;

        pub fn gtsam_shim_noise_diagonal_variances(variances: *const f64) -> *mut c_void;
        pub fn gtsam_shim_noise_gaussian_covariance(covariance: *const f64) -> *mut c_void;
        pub fn gtsam_shim_noise_robust_huber(k: f64, base: *const c_void) -> *mut c_void;
        pub fn gtsam_shim_noise_free(noise: *mut c_void);

        pub fn gtsam_shim_graph_create() -> *mut c_void;
        pub fn gtsam_shim_graph_destroy(graph: *mut c_void);
        pub fn gtsam_shim_graph_clear(graph: *mut c_void);
        pub fn gtsam_shim_graph_size(graph: *const c_void) -> usize;
        pub fn gtsam_shim_graph_add_prior_pose3(
            graph: *mut c_void,
            key: u64,
            r: *const f64,
            t: *const f64,
            noise: *const c_void,
        ) -> i32;
        pub fn gtsam_shim_graph_add_between_pose3(
            graph: *mut c_void,
            key1: u64,
            key2: u64,
            r: *const f64,
            t: *const f64,
            noise: *const c_void,
        ) -> i32;

        pub fn gtsam_shim_values_create() -> *mut c_void;
        pub fn gtsam_shim_values_destroy(values: *mut c_void);
        pub fn gtsam_shim_values_clear(values: *mut c_void);
        pub fn gtsam_shim_values_insert_pose3(
            values: *mut c_void,
            key: u64,
            r: *const f64,
            t: *const f64,
        ) -> i32;
        pub fn gtsam_shim_values_at_pose3(
            values: *const c_void,
            key: u64,
            out_r: *mut f64,
            out_t: *mut f64,
        ) -> bool;

        pub fn gtsam_shim_isam2_create() -> *mut c_void;
        pub fn gtsam_shim_isam2_destroy(isam2: *mut c_void);
        pub fn gtsam_shim_isam2_update(
            isam2: *mut c_void,
            graph: *const c_void,
            values: *const c_void,
            remove_indices: *const u64,
            n_remove: usize,
            out_new_factor_indices: *mut *mut u64,
            out_len: *mut usize,
        ) -> i32;
        pub fn gtsam_shim_isam2_update_empty(isam2: *mut c_void) -> i32;
        pub fn gtsam_shim_isam2_calculate_best_estimate(isam2: *const c_void) -> *mut c_void;
        pub fn gtsam_shim_indices_free(indices: *mut u64);
    }
}

/// Error from the gtsam boundary. The shim swallows C++ exceptions and
/// reports them as codes; `context` says which call failed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GtsamError {
    pub context: &'static str,
}

impl std::fmt::Display for GtsamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "gtsam call failed: {}", self.context)
    }
}

impl std::error::Error for GtsamError {}

fn check(code: i32, context: &'static str) -> Result<(), GtsamError> {
    if code == 0 {
        Ok(())
    } else {
        Err(GtsamError { context })
    }
}

/// Rigid transform: row-major rotation matrix + translation, the same layout
/// the C++ core's `M3D`/`V3D` (Eigen) pairs carry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose3 {
    pub r: [[f64; 3]; 3],
    pub t: [f64; 3],
}

impl Pose3 {
    pub fn identity() -> Pose3 {
        Pose3 {
            r: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            t: [0.0, 0.0, 0.0],
        }
    }

    pub fn from_translation(t: [f64; 3]) -> Pose3 {
        Pose3 {
            t,
            ..Pose3::identity()
        }
    }

    fn r_flat(&self) -> [f64; 9] {
        [
            self.r[0][0],
            self.r[0][1],
            self.r[0][2],
            self.r[1][0],
            self.r[1][1],
            self.r[1][2],
            self.r[2][0],
            self.r[2][1],
            self.r[2][2],
        ]
    }
}

/// `gtsam::Symbol(chr, index)` packed into a raw key — used for the
/// location-landmark variables (`Symbol('l', i)`). Keyframe indices are plain
/// integer keys and need no packing.
pub fn symbol_key(chr: char, index: u64) -> u64 {
    unsafe { ffi::gtsam_shim_symbol_key(chr as c_char, index) }
}

/// Opaque `gtsam::SharedNoiseModel`.
pub struct NoiseModel {
    handle: *mut c_void,
}

impl NoiseModel {
    /// `noiseModel::Diagonal::Variances` — tangent order [rot(3), trans(3)].
    pub fn diagonal_variances(variances: &[f64; 6]) -> NoiseModel {
        let handle = unsafe { ffi::gtsam_shim_noise_diagonal_variances(variances.as_ptr()) };
        assert!(
            !handle.is_null(),
            "gtsam diagonal noise construction failed"
        );
        NoiseModel { handle }
    }

    /// `noiseModel::Gaussian::Covariance` — row-major 6x6, tangent order.
    pub fn gaussian_covariance(covariance: &[f64; 36]) -> NoiseModel {
        let handle = unsafe { ffi::gtsam_shim_noise_gaussian_covariance(covariance.as_ptr()) };
        assert!(
            !handle.is_null(),
            "gtsam gaussian noise construction failed"
        );
        NoiseModel { handle }
    }

    /// `noiseModel::Robust(mEstimator::Huber(k), base)`. `base` stays usable.
    pub fn robust_huber(k: f64, base: &NoiseModel) -> NoiseModel {
        let handle = unsafe { ffi::gtsam_shim_noise_robust_huber(k, base.handle) };
        assert!(
            !handle.is_null(),
            "gtsam robust-Huber noise construction failed"
        );
        NoiseModel { handle }
    }
}

impl Drop for NoiseModel {
    fn drop(&mut self) {
        unsafe { ffi::gtsam_shim_noise_free(self.handle) }
    }
}

/// Opaque `gtsam::NonlinearFactorGraph`.
pub struct FactorGraph {
    handle: *mut c_void,
}

impl FactorGraph {
    pub fn new() -> FactorGraph {
        let handle = unsafe { ffi::gtsam_shim_graph_create() };
        assert!(!handle.is_null(), "gtsam graph construction failed");
        FactorGraph { handle }
    }

    /// `resize(0)` — drop all staged factors after an iSAM2 commit.
    pub fn clear(&mut self) {
        unsafe { ffi::gtsam_shim_graph_clear(self.handle) }
    }

    pub fn len(&self) -> usize {
        unsafe { ffi::gtsam_shim_graph_size(self.handle) }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn add_prior_pose3(
        &mut self,
        key: u64,
        pose: &Pose3,
        noise: &NoiseModel,
    ) -> Result<(), GtsamError> {
        let r = pose.r_flat();
        let code = unsafe {
            ffi::gtsam_shim_graph_add_prior_pose3(
                self.handle,
                key,
                r.as_ptr(),
                pose.t.as_ptr(),
                noise.handle,
            )
        };
        check(code, "add_prior_pose3")
    }

    pub fn add_between_pose3(
        &mut self,
        key1: u64,
        key2: u64,
        pose: &Pose3,
        noise: &NoiseModel,
    ) -> Result<(), GtsamError> {
        let r = pose.r_flat();
        let code = unsafe {
            ffi::gtsam_shim_graph_add_between_pose3(
                self.handle,
                key1,
                key2,
                r.as_ptr(),
                pose.t.as_ptr(),
                noise.handle,
            )
        };
        check(code, "add_between_pose3")
    }
}

impl Default for FactorGraph {
    fn default() -> FactorGraph {
        FactorGraph::new()
    }
}

impl Drop for FactorGraph {
    fn drop(&mut self) {
        unsafe { ffi::gtsam_shim_graph_destroy(self.handle) }
    }
}

/// Opaque `gtsam::Values` — both the initial-estimate container and the
/// best-estimate snapshot returned by `Isam2::calculate_best_estimate`.
pub struct Values {
    handle: *mut c_void,
}

impl Values {
    pub fn new() -> Values {
        let handle = unsafe { ffi::gtsam_shim_values_create() };
        assert!(!handle.is_null(), "gtsam values construction failed");
        Values { handle }
    }

    pub fn clear(&mut self) {
        unsafe { ffi::gtsam_shim_values_clear(self.handle) }
    }

    pub fn insert_pose3(&mut self, key: u64, pose: &Pose3) -> Result<(), GtsamError> {
        let r = pose.r_flat();
        let code = unsafe {
            ffi::gtsam_shim_values_insert_pose3(self.handle, key, r.as_ptr(), pose.t.as_ptr())
        };
        check(code, "insert_pose3")
    }

    /// `at<Pose3>(key)`; `None` when the key is absent.
    pub fn pose3(&self, key: u64) -> Option<Pose3> {
        let mut r = [0.0f64; 9];
        let mut t = [0.0f64; 3];
        let found = unsafe {
            ffi::gtsam_shim_values_at_pose3(self.handle, key, r.as_mut_ptr(), t.as_mut_ptr())
        };
        if !found {
            return None;
        }
        Some(Pose3 {
            r: [[r[0], r[1], r[2]], [r[3], r[4], r[5]], [r[6], r[7], r[8]]],
            t,
        })
    }
}

impl Default for Values {
    fn default() -> Values {
        Values::new()
    }
}

impl Drop for Values {
    fn drop(&mut self) {
        unsafe { ffi::gtsam_shim_values_destroy(self.handle) }
    }
}

/// Opaque `gtsam::ISAM2`, configured like the C++ core:
/// relinearizeThreshold=0.01, relinearizeSkip=1.
pub struct Isam2 {
    handle: *mut c_void,
}

impl Isam2 {
    pub fn new() -> Isam2 {
        let handle = unsafe { ffi::gtsam_shim_isam2_create() };
        assert!(!handle.is_null(), "gtsam ISAM2 construction failed");
        Isam2 { handle }
    }

    /// `update(graph, values, removeFactorIndices)`. Returns the
    /// `newFactorsIndices` gtsam assigned to the staged factors, in graph
    /// order — the C++ core uses these to revise (remove) location-constraint
    /// factors later.
    pub fn update(
        &mut self,
        graph: &FactorGraph,
        values: &Values,
        remove_indices: &[u64],
    ) -> Result<Vec<u64>, GtsamError> {
        let mut out_ptr: *mut u64 = std::ptr::null_mut();
        let mut out_len: usize = 0;
        let code = unsafe {
            ffi::gtsam_shim_isam2_update(
                self.handle,
                graph.handle,
                values.handle,
                remove_indices.as_ptr(),
                remove_indices.len(),
                &mut out_ptr,
                &mut out_len,
            )
        };
        check(code, "isam2_update")?;
        if out_ptr.is_null() {
            return Ok(Vec::new());
        }
        let indices = unsafe { std::slice::from_raw_parts(out_ptr, out_len) }.to_vec();
        unsafe { ffi::gtsam_shim_indices_free(out_ptr) };
        Ok(indices)
    }

    /// No-argument `update()` — extra relinearization pass after a closure.
    pub fn update_empty(&mut self) -> Result<(), GtsamError> {
        let code = unsafe { ffi::gtsam_shim_isam2_update_empty(self.handle) };
        check(code, "isam2_update_empty")
    }

    /// `calculateBestEstimate()` — a fresh `Values` snapshot.
    pub fn calculate_best_estimate(&self) -> Result<Values, GtsamError> {
        let handle = unsafe { ffi::gtsam_shim_isam2_calculate_best_estimate(self.handle) };
        if handle.is_null() {
            return Err(GtsamError {
                context: "calculate_best_estimate",
            });
        }
        Ok(Values { handle })
    }
}

impl Default for Isam2 {
    fn default() -> Isam2 {
        Isam2::new()
    }
}

impl Drop for Isam2 {
    fn drop(&mut self) {
        unsafe { ffi::gtsam_shim_isam2_destroy(self.handle) }
    }
}
