use colored::Colorize;
use pyo3::prelude::*;
use pyo3::types::PyAny;

#[pyclass]
pub struct PubSubTransport {
    #[pyo3(get, set)]
    topic: Py<PyAny>,
}

#[pymethods]
impl PubSubTransport {
    #[new]
    fn new(topic: Py<PyAny>) -> Self {
        PubSubTransport { topic }
    }

    fn __str__(&self, py: Python<'_>) -> PyResult<String> {
        let topic_str = self.topic.bind(py).repr()?.to_string();
        Ok(format!(
            "{}{}{}",
            "PubSubTransport(".green(),
            topic_str.blue(),
            ")".green()
        ))
    }

    fn __repr__(&self, py: Python<'_>) -> PyResult<String> {
        self.__str__(py)
    }
}
