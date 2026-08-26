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

use crate::planner::{plan as plan_impl, Emb, COMMIT_MARGIN};

/// The `Embodiment` fields the search reads, in declaration order: length,
/// width, center_off, comfort, precision, strafe, reverse, yaw_w, envelope,
/// arc_inflate. `envelope` is the measured per-heading rows,
/// `(deg, length, width, off_x, off_y)` each; an empty sequence asks for the
/// all-gait union at every heading, which is what an unmeasured embodiment
/// gets. A tuple rather than a class because `planners/target.py` marshals it
/// straight off the frozen dataclass, and the crossing is the spec.
/// `target.py`'s marshalling of an `Embodiment`; the governor rides as its own
/// tuple because pyo3 extracts at most twelve elements.
type EmbTuple = (
    f64,
    f64,
    f64,
    f64,
    f64,
    f64,
    f64,
    f64,
    Vec<[f64; 5]>,
    f64,
    (f64, f64, f64, f64),
    ((f64, f64, f64), (f64, f64), (f64, f64, f64)),
);

/// One plan call. points: (N, 2) float64 obstacle xy in world frame -- every
/// row is an obstacle, the caller's model already decided which (see
/// `planner.rs`). `incumbent` is the (M, 3) route the caller last published, or
/// None on the first plan and after a reset; `commit_margin` is
/// `scenarios.COMMIT_MARGIN`, which python owns and hands over here the way it
/// hands over the envelope. Returns an (M, 3) array of (x, y, yaw) at
/// `resolution`, or None to refuse.
#[pyfunction]
#[pyo3(signature = (points, pose, goal, emb, resolution, incumbent=None, commit_margin=COMMIT_MARGIN))]
// The argument list IS the boundary, and it is the spec: every one of these is
// a thing python owns and the crate is handed. Bundling them into a struct
// would only move the same seven names one indirection away from the call.
#[allow(clippy::too_many_arguments)]
fn plan<'py>(
    py: Python<'py>,
    points: PyReadonlyArray2<'py, f64>,
    pose: (f64, f64, f64),
    goal: (f64, f64),
    emb: EmbTuple,
    resolution: f64,
    incumbent: Option<PyReadonlyArray2<'py, f64>>,
    commit_margin: f64,
) -> PyResult<Option<Bound<'py, PyArray2<f64>>>> {
    if points.shape()[1] != 2 {
        return Err(PyValueError::new_err(format!(
            "points must be (N, 2) float64, got shape {:?}",
            points.shape()
        )));
    }
    let inc = match &incumbent {
        None => None,
        Some(a) => {
            if a.shape()[1] != 3 {
                return Err(PyValueError::new_err(format!(
                    "incumbent must be (M, 3) float64, got shape {:?}",
                    a.shape()
                )));
            }
            let v = a.as_array();
            Some(
                (0..v.shape()[0])
                    .map(|k| [v[[k, 0]], v[[k, 1]], v[[k, 2]]])
                    .collect::<Vec<[f64; 3]>>(),
            )
        }
    };
    let view = points.as_array();
    let pts: Vec<[f64; 2]> = (0..view.shape()[0])
        .map(|k| [view[[k, 0]], view[[k, 1]]])
        .collect();
    let emb = Emb {
        length: emb.0,
        width: emb.1,
        center_off: emb.2,
        comfort: emb.3,
        precision: emb.4,
        strafe: emb.5,
        reverse: emb.6,
        yaw_w: emb.7,
        envelope: emb.8,
        arc_inflate: emb.9,
        max_speed: emb.10 .0,
        min_speed: emb.10 .1,
        speed_clearance: emb.10 .2,
        max_yaw_rate: emb.10 .3,
        command_slew: [emb.11 .0 .0, emb.11 .0 .1, emb.11 .0 .2],
        gait_band: [emb.11 .1 .0, emb.11 .1 .1],
        walk_gain: emb.11 .2 .0,
        walk_slip: emb.11 .2 .1,
        walk_slip_ramp: emb.11 .2 .2,
    };
    let out = py.allow_threads(|| {
        plan_impl(
            &pts,
            pose,
            goal,
            &emb,
            resolution,
            inc.as_deref(),
            commit_margin,
        )
    });
    Ok(out.map(|states| {
        let mut arr = Array2::<f64>::zeros((states.len(), 3));
        for (k, s) in states.iter().enumerate() {
            arr[[k, 0]] = s[0];
            arr[[k, 1]] = s[1];
            arr[[k, 2]] = s[2];
        }
        arr.into_pyarray(py)
    }))
}

#[pymodule]
fn dimos_motion2_target(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(plan, m)?)?;
    Ok(())
}
