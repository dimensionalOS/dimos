---
title: "GraspGenX grasp proposals"
---

GraspGenX runs directly in-process as a deterministic contributor acceptance tool.
There is no ZMQ transport, sidecar, or runtime clone. The integration proposes robot
TCP targets from one
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

This direct-process contributor command loads one recorded scene, selects its labeled
banana point cloud, runs the real adapter once, saves an annotated PNG, and exits:

```bash
export DIMOS_GRASPGENX_CHECKPOINT=/path/to/checkpoint
export DIMOS_GRASPGENX_OUTPUT=./graspgenx-ycb-demo.png  # optional
uv run --extra graspgenx python -m dimos.manipulation.demo_graspgenx
```

The output is a static image, so the demo has no viewer process, Rerun recording, YAML
format, or DimOS Blueprint lifecycle. Use `--output` to choose another PNG path:

```bash
uv run --extra graspgenx python -m dimos.manipulation.demo_graspgenx \
  --output /tmp/graspgenx.png
```

The path is deliberately direct and deterministic:

```text
fixture + semantic label → banana point cloud → one GraspGenX call
       → ranked TCP candidates → annotated PNG
```

The image shows the complete scene in muted blue-gray, the selected Object Point Cloud
in yellow, and the top five candidates as Viridis-colored Grasp Envelope Glyphs with
rank and score labels. The candidate array still retains up to 100 ordered proposals.

### How to read the glyph

Fork mouth = candidate local +Z direction. Wider mouth = open span; narrow rear
bridge = half-open span. This is a 2D projection/profile of the 3D Gripper Model
and full 3D candidate pose, not physical finger geometry or execution validity.

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

To deploy another gripper, intentionally create a deployment with its tested
sweep-volume dimensions and calibrated transform, validate it, and restart the
stack. There is no runtime switching or named asset selection.

## Fixture

The recorded data is stored as `data/.lfs/graspgenx_ycb_banana_scene.tar.gz` and loaded
through `get_data("graspgenx_ycb_banana_scene")`. The standard data loader hydrates and
extracts the archive when needed. Its NPZ and JSON provenance record describe the source
banana OBJ, source commit/hash, sampling seed, transforms, semantic labels, counts,
frame, timestamp, and final NPZ SHA-256. The scene contains 3,500 banana, 256 table,
and 48 distractor points in `world`; label `0` selects the banana without a custom ROI
extractor.

## Troubleshooting

| Symptom | Action |
|---|---|
| Missing `gen/` or `dis/` | Point `DIMOS_GRASPGENX_CHECKPOINT` at the hydrated root. |
| Git-LFS pointer files | Install Git LFS, run `git lfs pull`, and verify content/size. |
| Sweep-incompatible backbone | Use sweep-volume-compatible diffusion and discriminator backbones. |
| CUDA/model initialization failure | Check driver/CUDA, `torch.cuda.is_available()`, resolved torch versions, and checkpoint contents. |
| No candidates | Empty is valid API no-result, but the image demo requires at least one proposal. |
| Image write failure | Check that the output directory exists or can be created and is writable. |

## Contributor verification

```bash
uv run pytest dimos/manipulation/grasping/test_grasp_gen_x.py \
  dimos/manipulation/demo_graspgenx/test_demo.py \
  dimos/manipulation/demo_graspgenx/test_render.py
uv run ruff check dimos/manipulation/grasping dimos/manipulation/demo_graspgenx
uv run ruff format --check dimos/manipulation/grasping dimos/manipulation/demo_graspgenx
```

An optional real-GPU acceptance run uses the direct contributor command with a real checkpoint.
Record CUDA initialization, inference diagnostics, and the output image separately;
these focused checks are not evidence for the broad full suite. No runtime clone or
checkpoint download is permitted.
