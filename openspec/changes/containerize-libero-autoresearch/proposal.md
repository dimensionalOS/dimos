## Why

The manipulation autoresearch runner currently depends on the mutable host checkout, Python environment, Node build, temporary filesystem, and Podman state. That makes a valid score difficult to reproduce and allowed infrastructure differences to dominate the first optimization attempts.

## What Changes

- Add a locally built, content-addressed outer evaluation image containing the locked DimOS Python environment and Pi code-policy extension.
- Add a host launcher that builds the existing LIBERO simulator image, builds the outer image, and runs one benchmark or environment check with the host rootless Podman socket and NVIDIA CDI device.
- Launch existing LIBERO simulator containers as siblings on the host network; do not nest a container engine or replace the native evaluator.
- Mount benchmark outputs and the verified asset cache at identical absolute paths and use a short dedicated temporary mount for IPC.
- Forward only the credentials and Evo result variables required by the evaluation.

## Capabilities

### New Capabilities

- `containerized-libero-autoresearch`: Reproducible local construction, checking, and execution of the LIBERO autoresearch environment.

### Modified Capabilities

None.

## Impact

This adds a container recipe, a small Python host launcher, focused tests, and evaluation documentation. It uses existing Podman, NVIDIA CDI, the checked-in dependency locks, and the existing LIBERO simulator container without changing task instructions, perception, grasping, planning, robot control, benchmark cases, or native scoring.
