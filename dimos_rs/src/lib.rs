use pyo3::prelude::*;

#[pyfunction]
fn double(x: i32) -> PyResult<i32> {
    Ok(x * 2)
}

#[pymodule]
fn dimos_rs(_py: Python, m: &Bound<PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(double, m)?)?;
    Ok(())
}
