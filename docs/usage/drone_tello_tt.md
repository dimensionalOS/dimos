# RoboMaster TT / Tello Integration

This page documents the DimOS Tello integration and the compatibility constraints used to keep existing drone functionality intact.

## Overview

DimOS now includes a RoboMaster TT / Tello implementation that runs the main runtime on an edge computer and communicates with the drone over the DJI Tello UDP SDK.

High-level stack:

```text
DimOS (laptop / edge)
  -> TelloConnectionModule
  -> Tello UDP SDK (cmd/state/video)
  -> Optional EXT commands (TT expansion board)

Video/state
  -> DroneTrackingModule + CameraModule
  -> agent / teleop / visualization modules
```

## New Blueprints

- `drone-tello-tt-basic`
- `drone-tello-tt-agentic`

List and run:

```bash
dimos list
dimos run --robot-ip 192.168.10.1 drone-tello-tt-agentic  --disable web-input
```

## Hardware + Network Connection Steps

For reliable Tello control plus cloud API access (OpenAI, optional Qwen, etc.), use two network paths:

1. Power on the RoboMaster TT / Tello and wait for SSID `TELLO-XXXXXX`.
2. Connect your primary Wi-Fi interface to `TELLO-XXXXXX` (this provides the `192.168.10.x` control link).
3. Keep internet on a second interface:
   - preferred: wired Ethernet (`eth0`), or
   - alternate: a second USB Wi-Fi adapter connected to home/office internet.
4. Verify routes:
   - Tello subnet exists on the Tello interface: `192.168.10.0/24`
   - default route points to your internet interface (not the Tello AP).
5. Verify connectivity:
   - `ping -c 1 192.168.10.1` (drone reachable)
   - `curl -I https://api.openai.com` (internet reachable)
6. Launch DimOS:
   - `dimos --robot-ip 192.168.10.1 run drone-tello-tt-agentic --disable web-input`

If you only have a single Wi-Fi radio, connecting to Tello usually removes internet access and cloud tool-calls will fail.

## New Modules

- `dimos.robot.drone.tello_sdk.TelloSdkClient`
  - Raw UDP command channel (`8889`)
  - State stream (`8890`)
  - Video stream (`11111`) decode + reconnect handling
- `dimos.robot.drone.tello_connection_module.TelloConnectionModule`
  - Skills: `takeoff`, `land`, `move`, `move_relative`, `yaw`, `rc`, `send_ext`, `follow_object`, `center_person_by_yaw`, `orbit_object`, `observe`
  - Publishes telemetry/status/odom/video and follow-command stream

## Tracking and Follow Behavior

`DroneTrackingModule` now supports both legacy and Tello-optimized behavior.

Compatibility defaults:

- `enable_passive_overlay=False`
- `use_local_person_detector=False`
- `force_detection_servoing_for_person=False`
- `person_follow_policy="legacy_pid"`

These defaults preserve existing `drone-agentic` behavior (Qwen + tracker path) unless a blueprint explicitly opts in.

Tello blueprint opt-in:

- `enable_passive_overlay=True`
- `use_local_person_detector=True`
- `force_detection_servoing_for_person=True`
- `person_follow_policy="yaw_forward_constant"`

This keeps Tello-specific behavior scoped to Tello blueprints.

## Optional Local Detector/GPU Settings

Local YOLO person detection is optional and only used when enabled by module options.

Supported env overrides:

```bash
export DIMOS_DRONE_YOLO_DEVICE=cuda:0   # or cpu
export DIMOS_DRONE_YOLO_MODEL=yolo11s-pose.pt
export DIMOS_DRONE_YOLO_IMGSZ=416
export DIMOS_DRONE_YOLO_MAX_DET=5
```

Notes:

- CUDA is not required.
- If CUDA init fails, tracking falls back to CPU safely.
- No laptop-specific paths or machine-specific constants are required.

## Files Added

- `dimos/robot/drone/tello_sdk.py`
- `dimos/robot/drone/tello_connection_module.py`
- `dimos/robot/drone/blueprints/basic/drone_tello_tt_basic.py`
- `dimos/robot/drone/blueprints/agentic/drone_tello_tt_agentic.py`

## Files Updated

- `dimos/robot/drone/drone_tracking_module.py`
  - Added configurable tracking modes for compatibility and Tello-specific behavior
  - Added optional local detector path and passive overlay
  - Added yaw-only centering and follow policies
- `dimos/robot/drone/__init__.py`
- `dimos/robot/drone/blueprints/__init__.py`
- `dimos/robot/drone/blueprints/basic/__init__.py`
- `dimos/robot/drone/blueprints/agentic/__init__.py`
- `dimos/robot/all_blueprints.py`
- `dimos/robot/drone/README.md`

## Non-Regression Intent

The integration is designed so legacy drone stacks are unaffected unless they opt into Tello-oriented tracking options.

Validation commands:

```bash
uv run ruff check dimos/robot/drone/
uv run mypy dimos/robot/drone/
python3 -m py_compile dimos/robot/drone/drone_tracking_module.py
python3 -m py_compile dimos/robot/drone/tello_connection_module.py
```
