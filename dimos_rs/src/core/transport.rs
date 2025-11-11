use crate::core::colors::{blue, green};
use pyo3::prelude::*;
use std::fmt;

#[pyclass]
pub struct PubSubTransport {
    topic: String,
}

#[pymethods]
impl PubSubTransport {
    #[new]
    fn new(topic: String) -> Self {
        PubSubTransport { topic }
    }

    fn __str__(&self) -> String {
        format!(
            "{}{}{}",
            green("PubSubTransport("),
            blue(&self.topic),
            green(")")
        )
    }

    fn __repr__(&self) -> String {
        self.__str__()
    }
}

impl fmt::Display for PubSubTransport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.__str__())
    }
}
