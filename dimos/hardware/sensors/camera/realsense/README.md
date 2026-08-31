# RealSense

Capture is the rust module in `rust/`, a workspace member. It links the system librealsense2 (`librealsense2-dev` from the [RealSense apt repo](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md), or `pkgs.librealsense` in the nix shell):

```bash
cd rust && cargo build --release
```

`camera.py` launches it; `dimos run real-sense-camera-vis` shows color, depth and the cloud in Rerun.
