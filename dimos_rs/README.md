# dimos_rs

## Build the Rust extension

```bash
maturin develop -m dimos_rs/Cargo.toml
uv run python -c "import dimos_rs; print(dimos_rs.double(21))"

```
