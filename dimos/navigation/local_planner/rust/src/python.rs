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

use crate::planner::{plan_explored as plan_impl, Emb, COMMIT_MARGIN, VOXEL};

/// One plan call; python owns every argument (`search/target.py`). `points`/`ground`
/// are (N, 2) world xy, `emb` the `Embodiment` JSON, `incumbent` (M, 3); (M, 3) or None.
#[pyfunction]
#[pyo3(signature = (points, pose, goal, emb, resolution, incumbent=None, commit_margin=COMMIT_MARGIN, ground=None, unseen_cost=1.0, pointcloud_resolution=VOXEL))]
// the argument list is the boundary; a struct would only move the names one hop
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
    pointcloud_resolution: f64,
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
    let gnd = ground.as_ref().map(xy).unwrap_or_default();
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
            pointcloud_resolution,
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
fn dimos_local_planner(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(plan, m)?)?;
    Ok(())
}
