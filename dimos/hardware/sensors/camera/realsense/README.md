# RealSense

Capture is the rust module in `rust/`, built through its own flake (librealsense2 is not in the root shell):

```bash
cd dimos/hardware/sensors/camera/realsense/rust && nix develop path:. -c cargo build --release
```

`camera.py` launches it; `dimos run real-sense-camera-vis` shows color, depth and the cloud in Rerun.
