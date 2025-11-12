use lcm::Lcm;
use pyo3::prelude::*;

#[pyclass(unsendable)]
pub struct LCMConfig {
    pub ttl: u8,
    pub url: Option<String>,
    pub autoconf: bool,
    pub lcm: Option<Lcm>,
}

#[pymethods]
impl LCMConfig {
    #[new]
    fn new() -> Self {
        Self {
            ttl: 0,
            url: None,
            autoconf: true,
            lcm: None,
        }
    }
}
