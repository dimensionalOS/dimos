# R1Pro head stereo depth

`r1pro-head-stereo` runs the two existing head camera modules and `StereoDepth`.
It opens the cameras and configures the deserializer's FSYNC at 30 Hz. That
camera setting persists until reboot. It contains no motor, manipulation,
gripper, chassis or navigation modules. It does not record images by default.
Do not run another application that owns the same camera devices.

## Requirements

- The calibrated 1920 x 1536 UYVY head camera pair. Do not select a smaller V4L2
  capture mode. Rectification resizes images after capture.
- Factory calibration at `/opt/galaxea/body/stereo.yaml`, with translation in
  metres. `translation_scale_to_m` is explicit for other calibration units.
- A working DimOS environment and the native Jetson `v4l2_camera` executable.
- For the default CUDA backend, the installed NVIDIA VPI Python binding must
  match the interpreter. The robot used for validation has VPI 3.1.5 and Python
  3.10. Its SDK binding directory is
  `/opt/nvidia/vpi3/lib/aarch64-linux-gnu/python`.

Use a real Git checkout when resolving the native source directory. An extracted
source archive without `.git` can make DimOS fall back to its default repository.
To reuse an existing native binary without building, supply a JSON config with
`source_dir: null`, `build_command: null`, `auto_build: false` and the absolute
`executable` path for both `headleftcamera` and `headrightcamera`. Use the
executable's wrapper, which supplies the Jetson shared-library paths.

## Start and inspect

Activate the checkout's Python 3.10 environment. On a Jetson with the SDK layout
above, add only the binding directory to this process's import path:

```bash
export PYTHONPATH="/opt/nvidia/vpi3/lib/aarch64-linux-gnu/python${PYTHONPATH:+:$PYTHONPATH}"
python -c 'import vpi; print(vpi.__version__)'
dimos run r1pro-head-stereo
```

If using the explicit native-binary config described above:

```bash
dimos run r1pro-head-stereo --config /absolute/path/head-camera-config.json
```

Use the same environment and transport settings in another terminal:

```bash
dimos shell
```

```python
app.StereoDepth.get_status()
app.StereoDepth.sample_depth(480, 384)
```

Both RPCs only read state. `sample_depth(u, v)` returns the median valid depth
in a 5 x 5 rectified pixel patch. It returns unavailable for stale or invalid
depth. Pixels are in the output image, not the original distorted image.

The portable reference backend can be selected without VPI:

```bash
dimos run r1pro-head-stereo --stereodepth.backend opencv_sgbm
```

## Outputs and geometry

| Topic | Type | Meaning |
| --- | --- | --- |
| `head_left_color`, `head_right_color` | `CompressedImage` | Original JPEG frames with capture timestamps |
| `head_rgbd_color` | `Image`, RGB | Rectified left image |
| `head_rgbd_depth` | `Image`, DEPTH | Float32 optical-axis depth in metres; zero means invalid |
| `head_rgbd_info` | `CameraInfo` | Rectified intrinsics and projection at the output resolution |
| `head_rgbd_confidence` | `Image`, GRAY16 | Backend confidence; not a calibrated probability |

RGB-D outputs share the left capture timestamp and
`r1pro_head_left_rectified_optical` frame. Defaults are 960 x 768, 5 processed
pairs per second, capture skew at most 2 ms, output age at most 0.5 s, and valid
depth from 0.3 to 3 m. Frames that expire during processing are not published.
The queue holds only the latest complete pair. `backpressure_replacements`
counts intentional replacement while rate limited; `expired_pairs` counts pairs
discarded at the age/stop boundary. `dropped_pairs` is their sum. `dropped_images`
counts frame-level pairing rejections. `age_s` measures the age of the last
published capture timestamp when queried, not GPU inference duration.

Rectification uses the complete factory intrinsics, distortion, stereo rotation
and translation. It recomputes zero-disparity projections at the output size.
Metric reprojection uses `Q`, including any principal-point offset. VPI's signed
Q10.5 disparity is divided by 32; OpenCV's fixed-point disparity by 16. Nonfinite,
nonpositive, low-confidence and out-of-range results become zero. No hole
filling supplies missing surface geometry. OpenCV confidence is a binary valid
mask, whereas VPI returns its U16 confidence map.

## Verification and limits

A bounded camera-only run on the R1Pro produced approximately 30 fps per eye and
5 Hz RGB-D, with no stereo processing error. Roughly 40% of the whole image had
valid depth under the default confidence/range mask in that scene. These figures
describe one functional run, not physical accuracy or maximum throughput.
The existing native right-camera process exited with signal 11 during shutdown
in that run. Both native processes exited and all workers stopped. The native
shutdown fault is unresolved and must be checked before unattended use.

Before using depth for picking, place a textured stationary board at measured
distances across the workspace. Check rectified horizontal alignment, depth
error, missing regions and timestamp freshness. Reflective, transparent and
textureless objects can have no usable depth. Unit tests establish numerical
conversion and masking, not the camera's physical accuracy.

This blueprint publishes no robot-base TF. The factory and current model mount
extrinsics differ and have not been reconciled. `robot_mount_verified` therefore
remains false. The rectified optical frame also differs from the raw left optical
frame by the rectification rotation. Do not treat camera-frame points as arm
targets. Verify the mount and capture-time torso FK before integrating object
poses with manipulation. No independent neck actuator has been identified or
added.
