---
title: "GraspGenX grasp proposals"
---

GraspGenX runs directly in-process in a DimOS worker. There is no ZMQ transport,
sidecar, or runtime clone. The integration proposes robot TCP targets from one
already-segmented object point cloud; it does not plan or execute a grasp.

## Install

```bash
uv sync --extra graspgenx
```

Use a CUDA-capable NVIDIA GPU and a compatible PyTorch installation. The integration
resolves `torch>=2.1,<2.7` and `torchvision>=0.16,<0.22`, and pins GraspGenX to
`b9429097728cb1c430dd78b92edf17ba318aad03`.

The extra expresses installation intent. uv uses one universal lock, so these
constraints participate in the global resolution and can change the shared runtime.
This checkout selects shared `torch==2.6.0` and `torchvision==0.21.0`. The accepted
direct shared-lock trade-off means enabling this extra can regress global torch,
CUDA, and MuJoCo behavior; run the relevant regression suites after lock changes.
The validated uv overrides are `yourdfpy>=0.0.60`, `trimesh>=4.12`, `numpy>=2`,
`timm>=1.0.17`, `huggingface-hub>=0.30,<1`, `diffusers>=0.29`, and
`pyopengl>=3.1.5`.

When changing constraints, re-resolve the universal lock and run focused GraspGenX
tests plus relevant DimOS regression tests.

### Checkpoint and assets

Set `DIMOS_GRASPGENX_CHECKPOINT` to a hydrated checkpoint root containing:

```text
/path/to/checkpoint/
├── gen/
└── dis/
```

The checkpoint must use a sweep-volume-compatible backbone. The adapter never
downloads checkpoints, gripper assets, or repository data. A Git-LFS pointer is not a
checkpoint: if a file begins with `version https://git-lfs.github.com/spec`, hydrate
it with `git lfs install` and `git lfs pull`, then verify the files are populated.

## Run the YCB demo

This one-shot command loads the recorded fixture, crops the banana, initializes the
model, writes YAML, and exits:

```bash
export DIMOS_GRASPGENX_CHECKPOINT=/path/to/checkpoint
export DIMOS_GRASPGENX_OUTPUT=./graspgenx-ycb-demo.yaml  # optional
uv run --extra graspgenx dimos --viewer none run graspgenx-ycb-demo
```

The global `--rerun-open` contract is honored by worker completion: `none` writes a
complete RRD without launching, `native` launches the desktop viewer, `web` launches
Rerun 0.32 with `--web-viewer`, and `both` launches both. `--viewer none` disables
Rerun recording entirely:

```bash
uv run --extra graspgenx dimos --viewer rerun run graspgenx-ycb-demo
```

`--viewer none` is headless and creates no Rerun objects or recording files. Viewer launch
failures are nonfatal and never replace a finalized RRD or YAML output. Recording and YAML
failures are fatal and preserve any prior valid final files. Diagnostics before inference
include checkpoint, CUDA, and device; the final diagnostics include scene/object counts,
candidate count, best score, result frame, TCP calibration, output path, recording path,
and visualization completion.

With `rerun`, the demo writes a native Rerun 0.32 `.rrd` file and then launches
the native viewer automatically from the CLI parent, after worker/watchdog and
registry cleanup. This demo defaults that native launch to X11/XWayland: it removes
`WAYLAND_DISPLAY` from the child environment and uses the existing `DISPLAY`. This is
scoped to this demo; global DimOS viewer defaults are unchanged. The viewer remains
open after the one-shot exits and receives the complete recording: the raw scene in grey, the cropped object in yellow, and
the top 20 TCP candidate axes color-mapped by score. It also shows the best candidate's
fixed deployment sweep-volume boxes (open and half-open), using the same gripper
dimensions propagated to inference and visualization. The coordinator-owned one-shot
lifecycle runs this finite job once and exits 0 after cleanup when inference and YAML
output succeed.

The launcher uses `posix_spawn` with a detached process group, closes inherited
DimOS descriptors, and removes `DIMOS_RUN_ID` from the viewer environment. On
Linux/Python builds where `POSIX_SPAWN_SETSID` is exposed but unavailable at
runtime, it falls back to a private process group; a failed launch is non-fatal
and prints the exact manual command.

The user-verified failure mode for this demo is specific to the default Rerun 0.32
Wayland/NVIDIA path: the recording renders, then the native window becomes
unresponsive. The same recording was responsive when launched with
`env -u WAYLAND_DISPLAY rerun file.rrd` under X11/XWayland. This is observed evidence
for this environment, not a claim about all Linux systems. To override the demo's
backend, set the runner deployment option in a blueprint/configuration to
`native_window_backend: auto` or `native_window_backend: wayland`. `wayland` instead
requires `WAYLAND_DISPLAY` and removes `DISPLAY` from the child; `auto` preserves both.
For a finalized recording, the equivalent manual commands are:

```bash
env -u WAYLAND_DISPLAY rerun /absolute/path/to/foo.rrd  # x11 (demo default)
rerun /absolute/path/to/foo.rrd                          # auto
env -u DISPLAY rerun /absolute/path/to/foo.rrd           # wayland
```

The default recording path follows the YAML path: `foo.yaml` produces `foo.rrd`. Set
the deployment `recording_path` to override it. Capture uses a sibling `.partial` file,
flushes and disconnects before atomic publication, and removes stale or failed partials.
If the native executable is missing or cannot launch, inference remains successful and
the terminal prints the exact manual fallback command, for example:

```bash
rerun /absolute/path/to/foo.rrd
```

## Input and result semantics

`GraspGenSpec.propose_grasps` accepts one segmented `PointCloud2` containing only the
target object, with finite XYZ values in metres. Returned TCP poses are in the input
cloud frame. DimOS `PointCloud2` carries `frame_id` and `ts`; the adapter preserves
those values exactly, rather than claiming to preserve a richer input `Header`.

Scores are generator-local ranking values for one returned set. They are not
probabilities or cross-call measurements. Candidates are ordered descending by score
and limited to 100 by default. Successful no-result inference returns an empty array;
invalid input, missing files, incompatible configuration, CUDA/model initialization,
and inference failures raise exceptions.

Candidates are unvalidated proposals. Collision checking, IK, approach and motion
planning, actuation, and execution remain outside this API.

## Fixed deployment profile

The demo uses this fixed, asset-free sweep-volume profile:

```yaml
gripper:
  family: revolute_3f
  extents_open: [0.08, 0.045, 0.04]
  offset_open: [0.0, 0.0, 0.135]
  extents_half_open: [0.04, 0.045, 0.035]
  offset_half_open: [0.0, 0.0, 0.118]
  fingertip_depth: 0.15
max_candidates: 100
```

`grasp_frame_to_tcp` defaults to identity, which is correct only when model grasp
frame and installed TCP coincide. The adapter applies
`T_input_tcp = T_input_graspgenx @ T_graspgenx_tcp`.

The canonical inclusive banana ROI is `[0.18, 0.08, 0.10]` to
`[0.42, 0.32, 0.24]`. It is a deployment default, not proposal request data.
Override it through runner configuration (`roi_minimum` and `roi_maximum` in a
blueprint/config file), then restart the stack.

To deploy another gripper, intentionally create a deployment with its tested
sweep-volume dimensions and calibrated transform, validate it, and restart the
stack. There is no runtime switching or named asset selection.

## Fixture and YAML

The checked-in `ycb_banana_scene.npz` and adjacent JSON record the source banana OBJ,
source commit/hash, area-weighted barycentric sampling seed, transform, counts, frame,
timestamp, ROI, and final NPZ SHA-256. It contains 3,500 banana, 256 table, and 48
distractor points (3,804 total) in `world`. The inclusive ROI from
`[0.18, 0.08, 0.10]` to `[0.42, 0.32, 0.24]` isolates the banana. Runtime loads bytes
only; it does not download, resample, clone an upstream repository, or fetch a
checkpoint or asset. The fixture NPZ and provenance JSON are included in both wheel
and sdist packaging for offline use.

YAML uses format `dimos.graspgenx.ycb.v1` and contains `frame`, `timestamp`,
`tcp_calibration`, and ordered `grasps`, each with `score` and `tcp_pose` (`position`
and quaternion `orientation`).

## Troubleshooting

| Symptom | Action |
|---|---|
| Missing `gen/` or `dis/` | Point `DIMOS_GRASPGENX_CHECKPOINT` at the hydrated root. |
| Git-LFS pointer files | Install Git LFS, run `git lfs pull`, and verify content/size. |
| Sweep-incompatible backbone | Use sweep-volume-compatible diffusion and discriminator backbones. |
| CUDA/model initialization failure | Check driver/CUDA, `torch.cuda.is_available()`, resolved torch versions, and checkpoint contents; retry with `--viewer none`. |
| No candidates | Empty is valid API no-result, but demo smoke acceptance fails; inspect ROI, crop counts, checkpoint, and diagnostics. |
| Visualization failure | Use `--viewer none`; visualization is non-blocking while inference and YAML remain primary. |

## Contributor verification

```bash
uv run pytest dimos/manipulation/grasping/test_grasp_gen_x.py \
  dimos/manipulation/graspgenx_demo/test_demo.py \
  dimos/manipulation/graspgenx_demo/test_visualization.py
uv run pytest dimos/robot/test_all_blueprints_generation.py
uv run ruff check dimos/manipulation/grasping dimos/manipulation/graspgenx_demo \
  dimos/robot/manipulators/graspgenx_ycb_demo.py
uv run ruff format --check dimos/manipulation/grasping dimos/manipulation/graspgenx_demo \
  dimos/robot/manipulators/graspgenx_ycb_demo.py
```

An optional real-GPU smoke uses the production command with a real checkpoint.
Record CUDA initialization, inference, diagnostics, and YAML output separately; these
focused checks are not evidence for the broad full suite. Packaging checks must also
confirm the fixture is present in both wheel and sdist and retain source/hash
provenance. No runtime clone or network asset download is permitted.
