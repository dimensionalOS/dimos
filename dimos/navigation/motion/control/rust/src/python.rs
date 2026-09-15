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

//! One pyfunction per law. A new law gets a new entry point rather than a
//! flag on an existing one: the signatures differ (a law marshals only the
//! inputs it reads) and the baseline's binding must not shift when a
//! generation lands.

use numpy::{PyReadonlyArray1, PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use dimos_motion2_target::planner::Emb;

use crate::clearance;
use crate::emb::{base_params, governor, hinted_params};
use crate::laws::hinted::update as hinted_impl;
use crate::laws::seed::update as seed_impl;
use crate::stamps;

/// The body as `Embodiment` dumps it: one JSON dict, the same string the
/// planner crate and the native modules take.
fn emb_of(emb: &str) -> PyResult<Emb> {
    serde_json::from_str(emb)
        .map_err(|e| PyValueError::new_err(format!("emb is not an Embodiment: {e}")))
}

fn rows_of(path: &PyReadonlyArray2<'_, f64>) -> PyResult<Vec<[f64; 3]>> {
    if path.shape()[1] != 3 {
        return Err(PyValueError::new_err(format!(
            "path must be (N, 3) float64, got shape {:?}",
            path.shape()
        )));
    }
    let view = path.as_array();
    Ok((0..view.shape()[0])
        .map(|k| [view[[k, 0]], view[[k, 1]], view[[k, 2]]])
        .collect())
}

// copied rather than borrowed as a slice: a strided or non-contiguous view
// has no slice, and these are one float per waypoint
fn vec_of(a: Option<&PyReadonlyArray1<'_, f64>>) -> Option<Vec<f64>> {
    a.map(|v| v.as_array().iter().copied().collect())
}

/// One tick of the seed law. `path` is an (N, 3) float64 array of (x, y, yaw)
/// in the pose's frame; `clearance` is an optional length-N float64 room
/// annotation (any other length is ignored, as in the python); `emb` the
/// body as JSON. Returns the body-frame twist (vx, vy, wz).
#[pyfunction]
#[pyo3(signature = (pose, path, clearance, emb))]
fn update_seed(
    py: Python<'_>,
    pose: (f64, f64, f64),
    path: PyReadonlyArray2<'_, f64>,
    clearance: Option<PyReadonlyArray1<'_, f64>>,
    emb: &str,
) -> PyResult<(f64, f64, f64)> {
    let rows = rows_of(&path)?;
    let clr = vec_of(clearance.as_ref());
    let e = emb_of(emb)?;
    let cfg = base_params(&e, [e.min_speed, e.max_speed]);
    Ok(py.allow_threads(|| seed_impl(pose, &rows, clr.as_deref(), &cfg)))
}

/// One tick of the hinted law. As `update_seed`, plus `ts`, the optional
/// length-N vector of the path's own per-waypoint stamps carrying the
/// planner's required-precision profile (see `stamps::decode_ceilings`).
#[pyfunction]
#[pyo3(signature = (pose, path, clearance, ts, emb))]
fn update_hinted(
    py: Python<'_>,
    pose: (f64, f64, f64),
    path: PyReadonlyArray2<'_, f64>,
    clearance: Option<PyReadonlyArray1<'_, f64>>,
    ts: Option<PyReadonlyArray1<'_, f64>>,
    emb: &str,
) -> PyResult<(f64, f64, f64)> {
    let rows = rows_of(&path)?;
    let clr = vec_of(clearance.as_ref());
    let stamps = vec_of(ts.as_ref());
    let cfg = hinted_params(&emb_of(emb)?);
    Ok(py.allow_threads(|| hinted_impl(pose, &rows, clr.as_deref(), stamps.as_deref(), &cfg)))
}

/// The planner's side of the stamp dialect: per-waypoint timestamps carrying
/// the required-precision profile. Exposed for parity against
/// `profile.encode_precision` -- on the robot the planner module calls the
/// rust directly, with no python in the loop.
#[pyfunction]
#[pyo3(signature = (path, clearance, t0, emb))]
fn encode_precision(
    py: Python<'_>,
    path: PyReadonlyArray2<'_, f64>,
    clearance: Option<PyReadonlyArray1<'_, f64>>,
    t0: f64,
    emb: &str,
) -> PyResult<Vec<f64>> {
    let gov = governor(&emb_of(emb)?);
    let rows = rows_of(&path)?;
    // no clearance and a wrong-length clearance are the same case to the
    // encoder, so the empty vec stands in for both
    let clr = vec_of(clearance.as_ref()).unwrap_or_default();
    Ok(py.allow_threads(|| stamps::encode_precision(&rows, &clr, t0, &gov)))
}

/// Per-waypoint room hint. `xy` is (N, 2) float64 waypoints, `points` the
/// obstacle model's (M, 3) float32 hard set -- every row counts, z unread.
/// Exposed for parity against the python's `cKDTree` twins -- on the robot both
/// modules call the rust directly.
#[pyfunction]
#[pyo3(signature = (xy, points, half_width))]
fn path_clearance(
    py: Python<'_>,
    xy: PyReadonlyArray2<'_, f64>,
    points: PyReadonlyArray2<'_, f32>,
    half_width: f64,
) -> PyResult<Vec<f64>> {
    if xy.shape()[1] != 2 {
        return Err(PyValueError::new_err(format!(
            "xy must be (N, 2) float64, got shape {:?}",
            xy.shape()
        )));
    }
    if points.shape()[1] != 3 {
        return Err(PyValueError::new_err(format!(
            "points must be (M, 3) float32, got shape {:?}",
            points.shape()
        )));
    }
    let wv = xy.as_array();
    let waypoints: Vec<[f64; 2]> = (0..wv.shape()[0])
        .map(|k| [wv[[k, 0]], wv[[k, 1]]])
        .collect();
    let pv = points.as_array();
    let cloud: Vec<[f32; 3]> = (0..pv.shape()[0])
        .map(|k| [pv[[k, 0]], pv[[k, 1]], pv[[k, 2]]])
        .collect();
    Ok(py.allow_threads(|| clearance::path_clearance(&waypoints, &cloud, half_width)))
}

#[pymodule]
fn dimos_motion2_tc(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(path_clearance, m)?)?;
    m.add_function(wrap_pyfunction!(update_seed, m)?)?;
    m.add_function(wrap_pyfunction!(update_hinted, m)?)?;
    m.add_function(wrap_pyfunction!(encode_precision, m)?)?;
    Ok(())
}
