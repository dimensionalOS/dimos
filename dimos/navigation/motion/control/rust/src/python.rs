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
//! inputs it reads) and a track's binding must not shift when another track's
//! generation lands.

use numpy::{PyReadonlyArray1, PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::clearance;
use crate::geom::Params;
use crate::laws::blind::{update as blind_impl, BlindParams};
use crate::laws::hinted::{update as hinted_impl, HintedParams, Law as HintedLaw};
use crate::laws::seed::update as seed_impl;
use crate::stamps;

/// The numeric `ControllerConfig` fields, in declaration order.
type BaseParams = (f64, f64, f64, f64, f64, f64, f64, f64, f64, f64, f64);

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

/// (max_speed, min_speed, speed_clearance, floor, max_yaw_rate): the
/// embodiment's governor, as `embodiment/base.py` orders it.
type GovernorTuple = (f64, f64, f64, f64, f64);

fn governor_of(g: GovernorTuple) -> stamps::Governor {
    stamps::Governor {
        max_speed: g.0,
        min_speed: g.1,
        speed_clearance: g.2,
        floor: g.3,
        max_yaw_rate: g.4,
    }
}

fn base_params(p: BaseParams) -> Params {
    Params {
        lookahead: p.0,
        max_speed: p.1,
        max_yaw_rate: p.2,
        k_pos: p.3,
        k_yaw: p.4,
        fan_yaw_per_m: p.5,
        fan_yaw_done: p.6,
        min_speed: p.7,
        speed_clearance: p.8,
        speed_floor_clearance: p.9,
        speed_lookahead: p.10,
    }
}

/// One tick of the seed law. `path` is an (N, 3) float64 array of (x, y, yaw)
/// in the pose's frame; `clearance` is an optional length-N float64 room
/// annotation (any other length is ignored, as in the python). Returns the
/// body-frame twist (vx, vy, wz).
#[pyfunction]
#[pyo3(signature = (pose, path, clearance, params))]
fn update_seed(
    py: Python<'_>,
    pose: (f64, f64, f64),
    path: PyReadonlyArray2<'_, f64>,
    clearance: Option<PyReadonlyArray1<'_, f64>>,
    params: BaseParams,
) -> PyResult<(f64, f64, f64)> {
    let rows = rows_of(&path)?;
    let clr = vec_of(clearance.as_ref());
    let cfg = base_params(params);
    Ok(py.allow_threads(|| seed_impl(pose, &rows, clr.as_deref(), &cfg)))
}

/// One tick of the blind law. As `update_seed`, plus `ts`, the optional
/// length-N vector of the path's own per-waypoint stamps carrying the
/// planner's required-precision profile (see `stamps::decode_ceilings`), and
/// the three gait-calibration fields `BlindControllerConfig` adds after the
/// base ones (walk_gain, walk_slip, slip_ramp).
#[pyfunction]
#[pyo3(signature = (pose, path, clearance, ts, params, walk))]
fn update_blind(
    py: Python<'_>,
    pose: (f64, f64, f64),
    path: PyReadonlyArray2<'_, f64>,
    clearance: Option<PyReadonlyArray1<'_, f64>>,
    ts: Option<PyReadonlyArray1<'_, f64>>,
    params: BaseParams,
    walk: (f64, f64, f64),
) -> PyResult<(f64, f64, f64)> {
    let rows = rows_of(&path)?;
    let clr = vec_of(clearance.as_ref());
    let stamps = vec_of(ts.as_ref());
    let cfg = BlindParams {
        base: base_params(params),
        walk_gain: walk.0,
        walk_slip: walk.1,
        slip_ramp: walk.2,
    };
    Ok(py.allow_threads(|| blind_impl(pose, &rows, clr.as_deref(), stamps.as_deref(), &cfg)))
}

/// The six terms `HintedParams` adds after the base ones, in the order
/// `ControllerConfig.hinted_params` yields them.
type HintedExtra = (f64, f64, f64, f64, f64, f64);

fn hinted_params(p: BaseParams, h: HintedExtra) -> HintedParams {
    HintedParams {
        base: base_params(p),
        tangent_preview: h.0,
        escape_clearance: h.1,
        escape_preview: h.2,
        escape_speed: h.3,
        brake_accel: h.4,
        brake_margin: h.5,
        // the pure law never reads it; `step` sets the body's own
        ..HintedParams::default()
    }
}

/// The hinted law, which keeps one tick of memory (its own previous command
/// and the time it was issued) so it can ramp its output at the plant's own
/// command slew. A class rather than a free function for exactly that reason;
/// `reset()` clears the history.
#[pyclass(name = "HintedLaw")]
pub struct PyHintedLaw {
    inner: HintedLaw,
}

#[pymethods]
impl PyHintedLaw {
    #[new]
    fn new() -> Self {
        Self {
            inner: HintedLaw::new(),
        }
    }

    fn reset(&mut self) {
        self.inner.reset();
    }

    /// One tick, rate limited. As `update_seed`, plus the tick time `t`, the
    /// six extra `HintedParams` terms and the body's `command_slew`.
    #[pyo3(signature = (pose, path, clearance, t, params, hinted, slew))]
    // the binding is a marshalling boundary: every argument is one input the
    // law reads, and bundling them would only hide the count behind a tuple
    #[allow(clippy::too_many_arguments)]
    fn step(
        &mut self,
        py: Python<'_>,
        pose: (f64, f64, f64),
        path: PyReadonlyArray2<'_, f64>,
        clearance: Option<PyReadonlyArray1<'_, f64>>,
        t: f64,
        params: BaseParams,
        hinted: HintedExtra,
        slew: (f64, f64, f64),
    ) -> PyResult<(f64, f64, f64)> {
        let rows = rows_of(&path)?;
        let clr = vec_of(clearance.as_ref());
        let mut cfg = hinted_params(params, hinted);
        cfg.slew = [slew.0, slew.1, slew.2];
        let inner = &mut self.inner;
        Ok(py.allow_threads(move || inner.step(pose, &rows, clr.as_deref(), &cfg, t)))
    }
}

/// The hinted law WITHOUT its rate limiter -- the pure tick, for parity sweeps
/// and for anything that wants the request rather than the ramped command.
#[pyfunction]
#[pyo3(signature = (pose, path, clearance, params, hinted))]
fn update_hinted_raw(
    py: Python<'_>,
    pose: (f64, f64, f64),
    path: PyReadonlyArray2<'_, f64>,
    clearance: Option<PyReadonlyArray1<'_, f64>>,
    params: BaseParams,
    hinted: HintedExtra,
) -> PyResult<(f64, f64, f64)> {
    let rows = rows_of(&path)?;
    let clr = vec_of(clearance.as_ref());
    let cfg = hinted_params(params, hinted);
    Ok(py.allow_threads(|| hinted_impl(pose, &rows, clr.as_deref(), &cfg)))
}

/// The planner's side of the stamp dialect: per-waypoint timestamps carrying
/// the required-precision profile. Exposed for parity against
/// `profile.encode_precision` -- on the robot the planner module calls the
/// rust directly, with no python in the loop.
#[pyfunction]
#[pyo3(signature = (path, clearance, t0, governor))]
fn encode_precision(
    py: Python<'_>,
    path: PyReadonlyArray2<'_, f64>,
    clearance: Option<PyReadonlyArray1<'_, f64>>,
    t0: f64,
    governor: GovernorTuple,
) -> PyResult<Vec<f64>> {
    let gov = governor_of(governor);
    let rows = rows_of(&path)?;
    // no clearance and a wrong-length clearance are the same case to the
    // encoder, so the empty vec stands in for both
    let clr = vec_of(clearance.as_ref()).unwrap_or_default();
    Ok(py.allow_threads(|| stamps::encode_precision(&rows, &clr, t0, &gov)))
}

/// The dialect's inverse leg: decoded speed ceilings back to the clearance
/// that produced them. Exposed for parity against
/// `profile.ceilings_to_clearance` -- on the robot the follower module calls
/// the rust directly when it is on the hinted track with no cloud of its own.
#[pyfunction]
#[pyo3(signature = (ceilings, governor))]
fn ceilings_to_clearance(
    py: Python<'_>,
    ceilings: PyReadonlyArray1<'_, f64>,
    governor: GovernorTuple,
) -> Vec<f64> {
    let v: Vec<f64> = ceilings.as_array().iter().copied().collect();
    let gov = governor_of(governor);
    py.allow_threads(|| stamps::ceilings_to_clearance(&v, &gov))
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
    m.add_function(wrap_pyfunction!(update_blind, m)?)?;
    m.add_function(wrap_pyfunction!(update_hinted_raw, m)?)?;
    m.add_function(wrap_pyfunction!(encode_precision, m)?)?;
    m.add_function(wrap_pyfunction!(ceilings_to_clearance, m)?)?;
    m.add_class::<PyHintedLaw>()?;
    Ok(())
}
