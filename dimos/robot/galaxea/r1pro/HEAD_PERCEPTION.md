# R1Pro head perception

`r1pro-head-perception` extends `r1pro-head-stereo` with the same scene registration
used by the real xArm grasp blueprint, plus a standalone Viser viewer. It uses
Moondream for text-conditioned detection and EdgeTAM for instance masks.

The graph contains exactly the two head cameras, stereo depth, scene registration
and perception viewer. It has no robot connection, coordinator, manipulation,
gripper, planner or motor command interface. Starting it opens cameras and
configures FSYNC as described in [Head stereo depth](HEAD_STEREO.md).

## Prepare and check

Use a separate environment and a real checkout containing this PR and its parent
head-stereo PR. Do not replace the working manipulation environment's Torch.
The default model stack needs CUDA-enabled PyTorch compatible with the robot's
JetPack, Open3D, SAM2/EdgeTAM, Transformers, Viser, and the existing VPI binding.
VPI CUDA working does not establish PyTorch CUDA support. Standard CPU Torch
cannot run EdgeTAM. Choose a JetPack-compatible Torch distribution before
resolving the remaining project dependencies; do not blindly overwrite it with a
CPU wheel from a generic sync.

The inspected robot environment has Torch 2.6.0+cpu. Open3D and SAM2 are present
under the distribution names `open3d-unofficial-arm` 0.19.0.post9 and
`edgetam-dimos` 1.0.1. The checked model-asset locations are not prepared. Full
real-model perception is not yet validated. No packages or weights were installed
by this PR's tests.

From the intended environment, this command checks prerequisites without loading
models, downloading weights or opening cameras:

```bash
export PYTHONPATH="/opt/nvidia/vpi3/lib/aarch64-linux-gnu/python${PYTHONPATH:+:$PYTHONPATH}"
python -m dimos.robot.galaxea.r1pro.head_perception_preflight
```

The blueprint repeats the dependency and cached-asset check before deploying
modules. A passing asset presence check is not a model load or integrity test.
After preparing compatible dependencies, explicitly prefetch the existing model
assets before camera startup:

```bash
python -c 'from dimos.utils.data import get_data; print(get_data("models_edgetam"))'
python -c 'from huggingface_hub import snapshot_download; snapshot_download("vikhyatk/moondream2")'
```

These commands download model files from the existing DimOS LFS and Hugging Face
sources. The Moondream implementation retains the repository's existing model
loading policy. Validate model loading separately, with cameras closed, before
running the graph. The prerequisite guard is for the pinned default backend
combination; changing detector/segmenter composition requires updating that guard.

## Run and inspect

Once prerequisites pass and the camera devices are free:

```bash
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
dimos run r1pro-head-perception
```

An explicit native-camera executable config can be supplied with `--config`, as
in the parent stereo guide. No vendor motor stack is needed.

The viewer binds only `127.0.0.1:8096`. From the Mac, use the existing authorized
SSH route to forward TCP, then open `http://localhost:8096`:

```bash
ssh -N -L 8096:127.0.0.1:8096 nvidia@ROBOT_HOST
```

If the Mac has no authorized SSH key, run the tunnel from an already-authorized
host or arrange access separately. This PR does not create credentials, change
firewalls, expose public listeners or deploy a relay. Viser uses ordinary HTTP
and WebSocket over this tunnel.

Enter comma-separated object prompts and click **Scan scene**. This runs
perception only. A second click while a scan is running is rejected. Inspect the
segmentation overlay, depth, scene cloud, per-object cloud and AABB. The
object selector displays its ID, visible-surface center, size and point count.
Depth support and rejection reasons are shown even when no 3D object survives.

The same operation is available from the same environment and transport:

```bash
dimos shell
```

```python
scene = app.ObjectSceneRegistrationModule
scene.scan_scene(["water bottle", "cup", "box"])
scene.get_scan_status()
scene.get_detected_objects()
scene.get_object_pointcloud_by_object_id("ID_FROM_SCAN")
```

## Quality and freshness

Input RGB-D must be at most 0.5 s old, in one optical frame, with matching image
dimensions and intrinsics, and capture skew at most 2 ms. Masks are eroded before
measuring depth support, matching the existing 3D projection's boundary erosion.
By default a mask needs at least 100 valid depth pixels and 30% valid support.
These are initial rejection settings, not physical accuracy guarantees.

Each scan replaces the previous camera-frame scene. IDs are scan-local, and
camera-frame clouds are not accumulated across scans or torso poses. Results
expire 15 s after capture, including time spent in inference. The viewer hides
expired geometry and images; object/cloud query APIs return no old geometry.
`inference_s` includes detection, segmentation and 3D postprocessing, rather than
GPU kernel time alone. This inspection lifetime is not a freshness allowance for
future robot motion.
If inference exceeds it, the scan fails instead of publishing late geometry.

Generic scene-registration defaults preserve existing consumers: freshness and
depth-support policies are opt-in, and persistent tracking remains the default.
The R1Pro blueprint explicitly enables snapshot-only and strict input policies.

## Acceptance before manipulation

Start with stationary, opaque textured cups or boxes at measured distances.
Check RGB/mask alignment, foreground depth coverage, center error, repeatability,
adjacent-object separation, and rejection when depth is absent. Cover a camera or
stop the camera-only graph normally to verify stale snapshots disappear. Do not
stop another user's process or move motors to perform these checks.

Only then test transparent or reflective bottles. Detecting an object in RGB does
not establish valid depth. Missing depth is not filled with guessed geometry.
An AABB encloses visible samples; its center is not a grasp target, and its
orientation is not a measured object 6D pose.

All results use `r1pro_head_left_rectified_optical`, with optical x right, y down,
z forward, in metres. There is no base-frame TF or torso reader in this graph.
Mount calibration and capture-time torso FK remain a separate perception step.
The parent PR's native-camera shutdown fault and measured-distance accuracy
validation remain open. Do not enable unattended operation based on mock tests.
