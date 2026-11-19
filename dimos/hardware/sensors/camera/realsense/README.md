# RealSense

Capture runs in the Rust module in `rust/`. `camera.py` builds and launches it;
`dimos run real-sense-camera-vis` opens the camera streams in Rerun.

## macOS

Install the SDK and native build tools:

```bash
brew install librealsense rust pkgconf
```

From the DimOS checkout with its virtual environment activated:

```bash
pkg-config --modversion realsense2
dimos run real-sense-camera-vis
```

On macOS, DimOS builds with `cargo build --release --locked --target-dir target`
and launches `rust/target/release/realsense_native`. Cargo locates the SDK through
`pkg-config`; a custom SDK installation can set `PKG_CONFIG_PATH` to the directory
containing `realsense2.pc`. The first build downloads the Rust dependencies.

This sets up compilation. Camera USB access is separate; see the
[RealSense macOS instructions](https://github.com/realsenseai/librealsense/blob/master/doc/installation_osx.md).

## Linux

DimOS builds with `nix build -L path:.` and launches
`rust/result/bin/realsense_native`. The module has its own flake because
librealsense2 is not in the root shell. For development inside that shell:

```bash
cd dimos/hardware/sensors/camera/realsense/rust && nix develop path:. -c cargo build --release
```

Both platforms accept explicit `RealSenseCameraConfig.build_command` and
`executable` overrides for custom builds.
