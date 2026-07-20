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

use lcm_msgs::sensor_msgs::PointCloud2;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict};

use crate::costmapper::{Config, CostmapCalculator};

#[pyclass]
pub struct CostMapper {
    config: Config,
    calculator: CostmapCalculator,
}

fn config_value(config: &Bound<'_, PyDict>) -> PyResult<serde_json::Value> {
    let json = config.py().import("json")?;
    let encoded: String = json.call_method1("dumps", (config,))?.extract()?;
    serde_json::from_str(&encoded)
        .map_err(|error| PyValueError::new_err(format!("config must contain JSON values: {error}")))
}

#[pymethods]
impl CostMapper {
    #[new]
    #[pyo3(signature = (*, algo, config, initial_safe_radius_meters))]
    fn new(
        algo: String,
        config: &Bound<'_, PyDict>,
        initial_safe_radius_meters: f64,
    ) -> PyResult<Self> {
        let config = Config {
            algo,
            config: config_value(config)?,
            initial_safe_radius_meters,
        };
        let calculator = CostmapCalculator::new(&config)
            .map_err(|error| PyValueError::new_err(error.to_string()))?;
        Ok(Self { config, calculator })
    }

    /// Convert an LCM-encoded PointCloud2 into an LCM-encoded OccupancyGrid.
    fn calculate_costmap<'py>(
        &self,
        py: Python<'py>,
        cloud: &[u8],
    ) -> PyResult<Bound<'py, PyBytes>> {
        let cloud = PointCloud2::decode(cloud)
            .map_err(|error| PyValueError::new_err(format!("invalid PointCloud2: {error}")))?;
        let grid = self
            .calculator
            .calculate(&cloud)
            .map_err(|error| PyValueError::new_err(error.to_string()))?;
        Ok(PyBytes::new(py, &grid.encode()))
    }

    #[getter]
    fn algo(&self) -> &str {
        &self.config.algo
    }

    #[getter]
    fn initial_safe_radius_meters(&self) -> f64 {
        self.config.initial_safe_radius_meters
    }

    fn __repr__(&self) -> String {
        format!(
            "CostMapper(algo={:?}, initial_safe_radius_meters={})",
            self.config.algo, self.config.initial_safe_radius_meters,
        )
    }
}

#[pymodule]
fn dimos_native_costmapper(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<CostMapper>()?;
    Ok(())
}
