# RealSense

Capture is the rust module in `rust/`, built through its own flake (librealsense2 is not in the root shell):

```bash
cd dimos/hardware/sensors/camera/realsense/rust && nix build -L path:.
```

`camera.py` launches `result/bin/realsense_native` and runs that build by itself when the binary is missing. `dimos run real-sense-camera-vis --enable-pointcloud true` shows color, depth and the cloud in Rerun.
