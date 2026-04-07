# Plan: Create dimos-module-pct-planner (C++ Native Module)

## Context

PCT (Point Cloud Tomography) Planner is a 3D global route planner for multi-story environments. The reference is a Python+C++ hybrid (`~/repos/ros-navigation-autonomy-stack/src/route_planner/PCT_planner/`). We need to create `~/repos/dimos-module-pct-planner` — a fully C++ native module with a nix build, matching the pattern of `dimos-module-far-planner`.

## What PCT Does (pipeline)

1. **Tomography** — Slices a 3D point cloud into horizontal layers, computes traversability cost per cell (slope, step height, clearance)
2. **A* Planning** — Searches across layers for optimal path (handles floor transitions)
3. **GPMP Trajectory Optimization** — Smooths the A* path with heading/velocity constraints using GTSAM factor graphs
4. **Waypoint Extraction** — Picks the next waypoint along the path via lookahead distance

## Architecture

Single C++ binary (`pct_planner`) that:
- Subscribes to: `odometry`, `explored_areas` (PointCloud2), `goal` (PointStamped)
- Publishes: `way_point` (PointStamped), `goal_path` (Path), `tomogram` (PointCloud2 viz)
- Embeds tomography + A* + GPMP + waypoint extraction in one process

## Repo Structure

```
~/repos/dimos-module-pct-planner/
├── flake.nix                    # nix build (like far_planner)
├── CMakeLists.txt               # top-level cmake
├── main.cpp                     # LCM main loop (like FAR main.cpp)
├── common/                      # dimos_native_module.hpp, point_cloud_utils.hpp (FetchContent)
├── src/
│   ├── tomography/              # C++ port of tomogram.py + kernels.py (~600 lines)
│   │   ├── tomogram.h/cpp       # Tomogram class: point2map(), initMappingEnv()
│   │   └── kernels.h/cpp        # CPU kernels: tomography, traversability, inflation
│   ├── a_star/                   # EXISTING C++ (copy from reference)
│   │   ├── a_star_search.h/cc
│   │   └── CMakeLists.txt
│   ├── ele_planner/              # EXISTING C++ (copy from reference)
│   │   ├── offline_ele_planner.h/cc
│   │   └── CMakeLists.txt
│   ├── trajectory_optimization/  # EXISTING C++ (copy from reference)
│   │   ├── gpmp_optimizer/       # GTSAM factor graphs
│   │   ├── height_smoother/
│   │   └── CMakeLists.txt
│   ├── map_manager/              # EXISTING C++ (copy from reference)
│   │   ├── dense_elevation_map.h/cc
│   │   └── CMakeLists.txt
│   ├── common/smoothing/         # EXISTING C++ (copy from reference)
│   └── planner/                  # C++ port of planner_wrapper.py + path_utils.py (~300 lines)
│       ├── pct_planner.h/cpp     # Orchestrator: loadTomogram(), plan(), getWaypoint()
│       └── path_utils.h/cpp      # Lookahead waypoint extraction
```

## Dependencies (Nix)

| Dependency | Source | Notes |
|------------|--------|-------|
| **GTSAM** | `~/repos/gtsam-extended` → `gtsam-cpp` | Try 4.3a1, fix API breaks if needed |
| **OSQP** | `nixpkgs#osqp` | For trajectory smoothing |
| **Eigen3** | `nixpkgs#eigen` | Linear algebra |
| **LCM** | `lcm-extended` | Message transport (same as FAR) |
| **dimos-lcm** | `github:dimensionalOS/dimos-lcm` | LCM message definitions |

No pybind11 needed — this is a pure C++ binary, not Python extensions.

## flake.nix

```nix
{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    lcm-extended.url = "github:jeff-hykin/lcm_extended";
    dimos-lcm = { url = "github:dimensionalOS/dimos-lcm/main"; flake = false; };
    gtsam-extended = { url = "github:jeff-hykin/gtsam-extended"; };
  };

  outputs = { self, nixpkgs, flake-utils, lcm-extended, dimos-lcm, gtsam-extended, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        lcm = lcm-extended.packages.${system}.lcm;
        gtsam = gtsam-extended.packages.${system}.gtsam-cpp;
      in {
        packages.default = pkgs.stdenv.mkDerivation {
          pname = "dimos-module-pct-planner";
          version = "0.1.0";
          src = ./.;
          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [ lcm gtsam pkgs.eigen pkgs.osqp ];
          cmakeFlags = [ "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}" ];
        };
      });
}
```

## What Needs to Be Written vs Copied

### Copy from reference (C++ already exists — ~77 files)
- `src/a_star/` — A* search (2 files)
- `src/ele_planner/` — Elevation planner (2 files)
- `src/trajectory_optimization/` — GPMP optimizer (~20 files, uses GTSAM)
- `src/map_manager/` — Dense elevation map (2 files)
- `src/common/smoothing/` — Spline utilities (uses OSQP)

**Changes needed:** Remove pybind11 `python_interface.cc` files, adapt CMakeLists to link as static/shared libs instead of pybind11 modules.

### Port from Python to C++ (~2000 lines Python → ~1500 lines C++)
- `tomogram.py` + `kernels.py` → `src/tomography/` — Grid-based point cloud slicing, traversability kernels (NumPy array ops → Eigen matrices)
- `planner_wrapper.py` → `src/planner/pct_planner.cpp` — Tomogram loading, plan() orchestration, coordinate conversion
- `path_utils.py` → `src/planner/path_utils.cpp` — Lookahead waypoint extraction
- `pct_planner_node.py` → `main.cpp` — LCM subscribers/publishers, main loop

### Write new (LCM integration)
- `main.cpp` — LCM message handling, CLI args, main loop (pattern from FAR planner's `main.cpp`)

## Implementation Steps

### Step 1: Scaffold repo + build existing C++ libs
1. Create `~/repos/dimos-module-pct-planner/`
2. Copy C++ sources from reference (a_star, ele_planner, trajectory_optimization, map_manager, common)
3. Remove all pybind11 interfaces
4. Create CMakeLists.txt + flake.nix
5. **Build test**: `nix build --no-write-lock-file` — verify GTSAM linking works

### Step 2: Port tomography to C++
1. Port `Tomogram` class (grid allocation, point2map, layer simplification)
2. Port kernels (tomography, traversability, inflation) as CPU-only Eigen operations
3. Unit test: feed synthetic point cloud, verify traversability output

### Step 3: Port planner orchestration + waypoint to C++
1. Port `TomogramPlanner` (loadTomogram, initPlanner, plan, pos2idx)
2. Port `path_utils` (get_waypoint_from_traj with lookahead)
3. Wire together: tomogram → A* → GPMP → waypoint

### Step 4: Write main.cpp (LCM integration)
1. LCM subscribers: odometry, explored_areas, goal/clicked_point
2. LCM publishers: way_point, goal_path, tomogram (viz)
3. CLI args for all config params (resolution, slice_dh, slope_max, etc.)
4. Main loop: on goal → plan → publish waypoints at 5Hz
5. On explored_areas → rebuild tomogram

### Step 5: DimOS integration
1. Create `modules/pct_planner/pct_planner.py` (NativeModule config, like far_planner.py)
2. Add to blueprint as alternative to FarPlanner
3. Test in Unity sim

## DimOS Module Config (Python side)

```python
class PCTPlannerConfig(NativeModuleConfig):
    cwd: str | None = _MODULE_DIR
    executable: str = "result/bin/pct_planner"
    # LOCAL nix build during development — switch to github URL for release
    build_command: str | None = "nix build path:$HOME/repos/dimos-module-pct-planner --no-write-lock-file"

    resolution: float = 0.075
    slice_dh: float = 0.4
    slope_max: float = 0.45
    step_max: float = 0.5
    lookahead_distance: float = 1.25
    cost_barrier: float = 100.0
    kernel_size: int = 11

class PCTPlanner(NativeModule):
    explored_areas: In[PointCloud2]  # from PGO/visualizationTools
    odometry: In[Odometry]
    goal: In[PointStamped]
    way_point: Out[PointStamped]
    goal_path: Out[Path]
```

## GTSAM Version Risk

PCT C++ uses: `Matrix, Vector, NonlinearFactorGraph, LevenbergMarquardtOptimizer, Values, Symbol, NoiseModel, PriorFactor, OptionalJacobian, GaussNewtonOptimizer`. All core stable API — should work with 4.3a1. If not, fixes will be in `trajectory_optimization/gpmp_optimizer/factors*/` constructors.

---

## Testing (CRITICAL — nothing works until integration test passes)

### Unit Tests: `modules/pct_planner/test_pct_planner.py`

```python
class TestPCTPlannerConfig:
    def test_default_config(self): ...          # all params have correct defaults
    def test_cli_args_generation(self): ...     # config → CLI args roundtrip

class TestPCTPlannerModule:
    def test_ports_declared(self): ...          # In/Out stream types correct
    def test_cwd_resolves(self): ...            # executable path exists after build

class TestPathResolution:
    def test_executable_exists(self): ...       # result/bin/pct_planner exists
```

### Build Test: verify nix builds and binary runs

```bash
cd ~/repos/dimos-module-pct-planner
nix build --no-write-lock-file
./result/bin/pct_planner --help   # should print usage, not crash
```

### Integration Test: `tests/test_pct_planning.py` (THE MOST IMPORTANT TEST)

**Pattern:** Identical to `test_cross_wall_planning.py` — builds full blueprint, starts Unity sim, sends goals, tracks odometry, asserts robot reaches each waypoint within timeout.

**This test is the ONLY way to know if PCT works.** Unit tests verify plumbing; this test verifies the planner actually navigates a robot.

```python
# dimos/navigation/smart_nav/tests/test_pct_planning.py

"""E2E integration test: PCT planner navigation in Unity sim.

Verifies that PCT planner can:
1. Build a tomogram from explored_areas
2. Plan a path to a goal
3. Publish waypoints that the local planner follows
4. Actually move the robot to the goal

Run: DISPLAY=:1 pytest dimos/navigation/smart_nav/tests/test_pct_planning.py -v -s -m slow
"""

WAYPOINTS = [
    # (name, x, y, z, timeout_sec, reach_threshold_m)
    ("p0", -0.3, 2.5, 0.0, 30, 1.5),     # open corridor speed test
    ("p1", 3.3, -4.9, 0.0, 120, 2.0),     # navigate toward doorway area
    ("p2", 11.3, -5.6, 0.0, 120, 2.0),    # into right room
    ("p2→p0", -0.3, 2.5, 0.0, 180, 2.0),  # CRITICAL: back through doorway
]

WARMUP_SEC = 20.0  # PCT needs time to build initial tomogram

@pytest.mark.slow
class TestPCTPlanning:
    def test_pct_navigation_sequence(self) -> None:
        # Build blueprint with PCT planner instead of FAR
        blueprint = autoconnect(
            UnityBridgeModule.blueprint(...),
            SensorScanGeneration.blueprint(),
            _smart_nav_pct_sim,  # PCT variant of _smart_nav_sim
        ).remappings([...]).global_config(n_workers=8, simulation=True)

        coordinator = blueprint.build()
        # ... same odom tracking + goal sending + assertion pattern
        # as test_cross_wall_planning.py
```

### What the integration test validates

| Check | How |
|-------|-----|
| Binary builds and starts | Coordinator doesn't crash on `start()` |
| LCM topics wired | Odometry flows (odom_count > 0) |
| Tomogram builds | Robot can plan after warmup (doesn't get stuck at start) |
| Path planning works | Robot moves toward goal (position changes) |
| Waypoint following works | Robot reaches p0 within 30s |
| Multi-goal navigation | Robot reaches p1, p2 sequentially |
| **Cross-area routing** | Robot navigates from p2 back to p0 (the key test) |

### Test execution

```bash
# Run ONLY the PCT integration test
DISPLAY=:1 pytest dimos/navigation/smart_nav/tests/test_pct_planning.py -v -s -m slow

# Run BOTH FAR and PCT integration tests
DISPLAY=:1 pytest dimos/navigation/smart_nav/tests/ -v -s -m slow

# Run unit tests (fast, no sim needed)
uv run pytest dimos/navigation/smart_nav/modules/pct_planner/ -v
```

### IF THE INTEGRATION TEST FAILS, NOTHING WORKS

Do not consider any step complete until this test passes. The binary building is necessary but NOT sufficient. The unit tests are necessary but NOT sufficient. Only the integration test proves the PCT planner can actually navigate a robot through a simulated environment.

---

## Key Reference Files

| Target | Reference |
|--------|-----------|
| `main.cpp` | `~/repos/ros-navigation-autonomy-stack/src/route_planner/PCT_planner/pct_planner/scripts/pct_planner_node.py` (337 lines) |
| `src/tomography/` | `~/repos/ros-navigation-autonomy-stack/src/route_planner/PCT_planner/pct_planner/tomography/scripts/tomogram.py` + `kernels.py` (576 lines) |
| `src/planner/` | `~/repos/ros-navigation-autonomy-stack/src/route_planner/PCT_planner/pct_planner/planner/scripts/planner_wrapper.py` + `utils/path_utils.py` (300 lines) |
| C++ libs (copy) | `~/repos/ros-navigation-autonomy-stack/src/route_planner/PCT_planner/pct_planner/planner/lib/src/` (77 files) |
| Config params | `~/repos/ros-navigation-autonomy-stack/src/route_planner/PCT_planner/config/pct_planner_params.yaml` |
| LCM integration pattern | `dimos/navigation/smart_nav/modules/far_planner/repo/main.cpp` |
| Integration test pattern | `dimos/navigation/smart_nav/tests/test_cross_wall_planning.py` |
| Nix build pattern | FAR planner's `repo/flake.nix` |
