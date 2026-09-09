# DIOS installation core

Selective import of DIOS installation code, with an independent Cargo workspace.
App, desktop, service, and publishing tooling is excluded; see [UPSTREAM.md](UPSTREAM.md).

```bash
cargo build --locked --manifest-path installer/Cargo.toml
installer/target/debug/create-dimos --mode dev --profile navigation --project-dir "$PWD"
```

This foundation prepares and verifies explicit dependency profiles. SDK project
scaffolding and the final public workflow are added in the next stack layers.
