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

use crate::planner::{plan as plan_impl, Emb};

/// One plan call. points: (N, 3) float64 cloud in world frame; emb: (length,
/// width, center_off, comfort, precision, strafe, reverse, yaw_w). Returns an
/// (M, 3) array of (x, y, yaw) at `resolution`, or None to refuse.
#[pyfunction]
#[pyo3(signature = (points, pose, goal, emb, resolution))]
fn plan<'py>(
    py: Python<'py>,
    points: PyReadonlyArray2<'py, f64>,
    pose: (f64, f64, f64),
    goal: (f64, f64),
    emb: (f64, f64, f64, f64, f64, f64, f64, f64),
    resolution: f64,
) -> PyResult<Option<Bound<'py, PyArray2<f64>>>> {
    if points.shape()[1] != 3 {
        return Err(PyValueError::new_err(format!(
            "points must be (N, 3) float64, got shape {:?}",
            points.shape()
        )));
    }
    let view = points.as_array();
    let pts: Vec<[f64; 3]> = (0..view.shape()[0])
        .map(|k| [view[[k, 0]], view[[k, 1]], view[[k, 2]]])
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
    };
    let out = py.allow_threads(|| plan_impl(&pts, pose, goal, &emb, resolution));
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
