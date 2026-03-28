# Simulation

DimSim is a browser-based 3D simulator that provides the same sensor and control interface as physical robots. When you run any `sim-*` blueprint, dimos automatically downloads and launches DimSim — no manual setup or external dependencies required.

## Running

```bash
# Basic sim — camera feed + visualization
dimos --simulation run sim-basic

# With navigation stack
dimos --simulation run sim-nav

# Full agentic stack (navigation + VLM agent + skills)
dimos --simulation run sim-agentic
```

This opens a browser tab with the 3D environment. The robot is controlled via the same `cmd_vel` topic as physical hardware.

## Blueprints

| Blueprint | What it includes |
|-----------|-----------------|
| `sim-basic` | Sensor bridge + TF + visualization |
| `sim-nav` | + voxel mapper, costmap, A* planner, frontier explorer |
| `sim-spatial` | + spatial memory |
| `sim-agentic` | + VLM agent, navigation/speak/follow skills, web input |
| `sim-eval` | Agentic + temporal memory, for running eval workflows |
| `sim-parallel-eval` | Headless parallel eval (3 concurrent instances) |
| `sim-temporal-memory` | Full agentic stack with temporal memory |

Each blueprint builds on the previous one. Pick the one that matches the modules you need.

## Sensor Topics

DimSim publishes the same topics as physical robots:

| Topic | Message Type | Notes |
|-------|-------------|-------|
| `/odom` | `PoseStamped` | Robot pose in world frame |
| `/color_image` | `Image` | JPEG, 960x432 |
| `/depth_image` | `Image` | Depth buffer |
| `/lidar` | `PointCloud2` | Simulated 2D LiDAR |
| `/pointcloud` | `PointCloud2` | 3D point cloud |
| `/camera_info` | `CameraInfo` | Camera intrinsics (1 Hz) |
| `/cmd_vel` | `Twist` | Velocity commands (input) |

Transform tree: `world -> base_link -> camera_link -> camera_optical`, plus `base_link -> lidar_link`.

## Headless Mode

To run without a browser window (for CI or automated testing):

```bash
DIMSIM_HEADLESS=1 dimos --simulation run sim-basic
```

Add `DIMSIM_RENDER=gpu` on macOS for faster rendering (uses Metal). Default is `cpu` (SwiftShader, works without GPU).

## Running Evals

Evals test navigation accuracy by sending the agent to target locations and scoring distance.

### Sequential

```bash
pytest dimos/e2e_tests/test_dimsim_eval.py -v -s -m slow
```

### Parallel (3 concurrent workflows)

```bash
pytest dimos/e2e_tests/test_dimsim_eval_parallel.py -v -s -m slow
```

### Single eval

```bash
pytest dimos/e2e_tests/test_dimsim_eval.py::TestSimEvalSequential::test_go_to_tv -v -s -m slow
```

### With log capture

```bash
DIMSIM_EVAL_LOG_DIR=./logs pytest dimos/e2e_tests/test_dimsim_eval.py -v -s -m slow
```

## Environment Variables

| Variable | Purpose | Default |
|----------|---------|---------|
| `DIMSIM_HEADLESS` | Run headless (no browser window) | unset (browser opens) |
| `DIMSIM_RENDER` | Headless rendering: `gpu` or `cpu` | `cpu` |
| `DIMSIM_CHANNELS` | Number of parallel browser pages | unset |
| `DIMSIM_CONNECT_ONLY` | Skip DimSim launch, connect to existing server | unset |
| `EVAL_INSTANCE_ID` | Isolate memory paths per parallel instance | `0` |
| `DIMSIM_EVAL_LOG_DIR` | Directory for subprocess logs (eval tests) | unset |
