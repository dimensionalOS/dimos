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

use crate::planner::{plan_explored as plan_impl, Emb, COMMIT_MARGIN};

/// One plan call. points: (N, 2) float64 obstacle xy in world frame -- every
/// row is an obstacle, the caller's model already decided which (see
/// `planner.rs`). `emb` is the `Embodiment` as JSON -- the same dict the native
/// modules are configured with, dumped by `planners/target.py`; an empty
/// `envelope` asks for the all-gait union at every heading, which is what an
/// unmeasured embodiment gets. `incumbent` is the (M, 3) route the caller last
/// published, or None on the first plan and after a reset; `commit_margin` is
/// `se2.COMMIT_MARGIN`, which python owns and hands over here the way it hands
/// over the body. Returns an (M, 3) array of (x, y, yaw) at
/// `resolution`, or None to refuse. `ground` is the (N, 2) floor the cloud saw;
/// `unseen_cost` > 1 prices every metre over a cell with no floor under it
/// that many times higher (`planner.rs::plan_explored`).
#[pyfunction]
#[pyo3(signature = (points, pose, goal, emb, resolution, incumbent=None, commit_margin=COMMIT_MARGIN, ground=None, unseen_cost=1.0))]
// The argument list IS the boundary, and it is the spec: every one of these is
// a thing python owns and the crate is handed. Bundling them into a struct
// would only move the same seven names one indirection away from the call.
#[allow(clippy::too_many_arguments)]
fn plan<'py>(
    py: Python<'py>,
    points: PyReadonlyArray2<'py, f64>,
    pose: (f64, f64, f64),
    goal: (f64, f64),
    emb: &str,
    resolution: f64,
    incumbent: Option<PyReadonlyArray2<'py, f64>>,
    commit_margin: f64,
    ground: Option<PyReadonlyArray2<'py, f64>>,
    unseen_cost: f64,
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
    let xy = |a: &PyReadonlyArray2<'py, f64>| -> Vec<[f64; 2]> {
        let v = a.as_array();
        (0..v.shape()[0]).map(|k| [v[[k, 0]], v[[k, 1]]]).collect()
    };
    let pts = xy(&points);
    // (N, 2) ground returns; a cell with none under it prices at `unseen_cost`.
    let gnd = ground.as_ref().map(xy).unwrap_or_default();
    // the body as `Embodiment` dumps it: one dict, the native modules' own
    let emb: Emb = serde_json::from_str(emb)
        .map_err(|e| PyValueError::new_err(format!("emb is not an Embodiment: {e}")))?;
    let out = py.allow_threads(|| {
        plan_impl(
            &pts,
            &gnd,
            unseen_cost,
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
