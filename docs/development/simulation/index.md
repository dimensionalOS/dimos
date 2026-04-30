# Simulation

DimSim is a browser-based 3D simulator that provides the same sensor and control interface as physical robots. When you run any `sim-*` blueprint, dimos automatically downloads and launches DimSim — no manual setup or external dependencies required.

**Sub-pages:**
- [Scene Editing SDK](/docs/development/simulation/scene_editing.md) — Load custom scenes, swap embodiments, add objects at runtime
- [Viewing & Navigation](/docs/development/simulation/viewing.md) — Browser controls, god view, keyboard shortcuts
- [Evals](/docs/development/simulation/evals.md) — Running, creating, and rubric types *(in progress)*

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

| Topic | Message Type | Rate | Notes |
|-------|-------------|------|-------|
| `/odom` | `PoseStamped` | 50 Hz | Robot pose in world frame (server-side) |
| `/color_image` | `Image` | 5 Hz | JPEG, 960x432 |
| `/depth_image` | `Image` | 5 Hz | 16-bit depth (toggleable via `DIMSIM_DISABLE_DEPTH=1`) |
| `/lidar` | `PointCloud2` | 10 Hz | 15K-point simulated LiDAR (server-side raycast) |
| `/camera_info` | `CameraInfo` | 1 Hz | Camera intrinsics |
| `/cmd_vel` | `Twist` | — | Velocity commands (input) |

Transform tree: `world -> base_link -> {camera_link -> camera_optical, lidar_link}`.

Default camera FOV: 46 vertical (87 horizontal) — matches Go2 D435i depth camera.

## Sensor Configuration

Configure via `DimSimBridgeConfig` or environment variables (launch-time only):

```python
DimSimBridgeConfig(
    image_rate_ms=100,       # 10 Hz images (default: 200 = 5 Hz)
    enable_depth=False,      # skip depth publishing (saves bandwidth)
    camera_fov=46,           # vertical FOV in degrees (default: 46)
)
```

```bash
DIMSIM_IMAGE_RATE=100 DIMSIM_DISABLE_DEPTH=1 DIMSIM_CAMERA_FOV=46 dimos --simulation run sim-nav
```

## Headless Mode

To run without a browser window (for CI or automated testing):

```bash
DIMSIM_HEADLESS=1 dimos --simulation run sim-basic
```

Headless mode defaults to cpu usage. If gpu is available run with `DIMSIM_RENDER=gpu`.

## Evals

See [Evals](/docs/development/simulation/evals.md) — running, creating, and rubric types. *(in progress)*

## Environment Variables

| Variable | Purpose | Default |
|----------|---------|---------|
| `DIMSIM_SCENE` | Pre-made Scene to load (`apt`, `empty`, etc.) | `apt` |
| `DIMSIM_HEADLESS` | Run headless (no browser window) | unset |
| `DIMSIM_RENDER` | Headless rendering: `gpu` or `cpu` | `cpu` |
| `DIMSIM_LOCAL` | Use local DimSim repo instead of binary | unset |
| `DIMSIM_DISABLE_DEPTH` | Disable depth image publishing | unset |
| `DIMSIM_IMAGE_RATE` | Image publish interval in ms | `200` (5 Hz) |
| `DIMSIM_CAMERA_FOV` | Vertical camera FOV in degrees | `46` (Go2) |
| `DIMSIM_CHANNELS` | Number of parallel browser pages | unset |
| `DIMSIM_CONNECT_ONLY` | Skip launch, connect to existing server | unset |
| `DIMSIM_EVAL_LOG_DIR` | Directory for eval subprocess logs | unset |
