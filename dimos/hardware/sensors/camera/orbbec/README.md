# Orbbec

Gemini 330-series color and depth camera. The SDK is not a dimos dependency, install it by hand:

```bash
uv pip install --no-deps pyorbbecsdk2==2.1.2
```

`dimos run orbbec-camera-vis --orbbeccamera.enable-pointcloud true` shows color, depth and the cloud in Rerun.

Set `rotate_180` when the camera is mounted upside down.
