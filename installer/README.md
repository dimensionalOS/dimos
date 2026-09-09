# DimOS workspace creator

Create an editable SDK project, or prepare a contributor checkout.

```bash
cargo build --locked --manifest-path installer/Cargo.toml
installer/target/debug/create-dimos my-robot --profile navigation
cd my-robot
source .dimos/activate.sh
dimos run my-robot.hello
```

Use --restore to restore the environment while preserving source and manifests.
Optional direnv uses the same activation script. See [UPSTREAM.md](UPSTREAM.md).
