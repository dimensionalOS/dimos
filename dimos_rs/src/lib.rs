// use std::time::Duration;
// use zenoh::{Config, bytes::Encoding, key_expr::KeyExpr};

use pyo3::prelude::*;
pub mod core;

#[pyfunction]
fn double(x: i32) -> PyResult<i32> {
    Ok(x * 2)
}

#[pymodule]
fn dimos_rs(_py: Python, m: &Bound<PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(double, m)?)?;
    m.add_class::<core::transport::PubSubTransport>()?;
    Ok(())
}
