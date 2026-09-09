# DimOS workspace creator

A one-shot initializer derived from DIOS installation logic. It creates an editable
SDK workspace or prepares a DimOS contributor checkout. It installs no permanent
`dim` command and contains no desktop, app, or service manager.

```bash
cargo build --locked --manifest-path installer/Cargo.toml
installer/target/debug/create-dimos my-robot --profile navigation
cd my-robot
source .dimos/activate.sh
dimos run my-robot.hello
dimos doctor
```

Use `--contributor` to prepare a DimOS checkout and `--restore` to restore a
workspace from its configuration and lockfiles. See the
[installation guide](../docs/installation/installer.md) and [provenance](UPSTREAM.md).
