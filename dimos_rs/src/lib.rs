// use std::time::Duration;
// use zenoh::{Config, bytes::Encoding, key_expr::KeyExpr};

use pyo3::prelude::*;
use pyo3::types::{PyDict, PyTuple, PyType};
pub mod core;

#[pymodule]
fn dimos_rs(py: Python, m: &Bound<PyModule>) -> PyResult<()> {
    // Import Transport from dimos.core.stream
    let stream_module = py.import("dimos.core.stream")?;
    let transport_class = stream_module.getattr("Transport")?;

    // Get the Rust PubSubTransport type
    let rust_type = PyType::new::<core::transport::PubSubTransport>(py);

    // Create a new Python class that inherits from Transport[T] and the Rust type
    // This makes PubSubTransport inherit from Transport[T]
    let builtins = py.import("builtins")?;
    let type_func = builtins.getattr("type")?;

    let name = "PubSubTransport";
    // Inherit from Transport first (for isinstance checks and proper inheritance),
    // then the Rust implementation (for method resolution)
    let rust_type_any = rust_type.as_any().clone();
    let bases = PyTuple::new(py, &[transport_class, rust_type_any])?;
    let namespace = PyDict::new(py);

    // Create the new class that inherits from Transport[T] and the Rust type
    let new_class = type_func.call1((name, bases, namespace))?;

    m.add("PubSubTransport", new_class)?;
    Ok(())
}
