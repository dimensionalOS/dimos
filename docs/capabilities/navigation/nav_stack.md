# Smart Nav

Smart Nav is a modular navigation stack for autonomous robot navigation and exploration. It handles terrain classification, obstacle avoidance, global path planning, local trajectory selection, and loop-closure-corrected mapping -- all wired together as composable Blueprint modules.

It's a good fit when you have a lidar-equipped robot and need end-to-end autonomous navigation: give it a registered point cloud and odometry, and it produces velocity commands. The stack runs without ROS -- modules communicate over DimOS streams (LCM/SHM) and each component can be swapped or tuned independently.

```python
from dimos.navigation.nav_stack.main import nav_stack

blueprint = nav_stack()
```

Smart Nav consumes three external streams (typically provided by a SLAM module like FastLio2):

| Stream | Type | Description |
|--------|------|-------------|
| `registered_scan` | `PointCloud2` | World-frame lidar scan |
| `odometry` | `Odometry` | SLAM odometry |
| `clicked_point` | `PointStamped` | Navigation goal from a viewer or agent |

And produces:

| Stream | Type | Description |
|--------|------|-------------|
| `cmd_vel` | `Twist` | Velocity command for the robot |
| `corrected_odometry` | `Odometry` | PGO loop-closure-corrected pose |
| `global_map` | `PointCloud2` | Accumulated keyframe map |

## Customizing the Navigation

All configuration is done through `nav_stack()` keyword arguments. Each module has its own config dict, and there are a few top-level switches for structural choices.

```python
nav_stack(
    use_simple_planner=False,      # Use A* instead of FAR planner
    use_tare=False,                # Add TARE frontier exploration
    use_terrain_map_ext=True,      # Persistent terrain accumulator
    vehicle_height=None,           # Propagated to terrain + planner modules

    # Per-module config overrides (dicts merged onto defaults):
    terrain_analysis={...},
    local_planner={...},
    path_follower={...},
    far_planner={...},
    simple_planner={...},
    pgo={...},
    movement_manager={...},
    tare_planner={...},
    terrain_map_ext={...},
)
```

### Global Planner Selection

- **FarPlanner** (default) -- visibility-graph planner with larger sensor range. Better for outdoor or long-range navigation.
- **SimplePlanner** (`use_simple_planner=True`) -- grid-based A* planner. Lightweight, readable, good for smaller environments or debugging.

### Exploration

Set `use_tare=True` to add the TARE frontier exploration module. When enabled, TARE takes over waypoint generation and drives the robot to unexplored frontiers autonomously.

### Obstacle Sensitivity

TerrainAnalysis and LocalPlanner both have `obstacle_height_threshold`. Keep them aligned -- if TerrainAnalysis flags something as an obstacle but LocalPlanner's threshold is higher, the planner may drive through it.

```python
nav_stack(
    terrain_analysis={"obstacle_height_threshold": 0.1},
    local_planner={"obstacle_height_threshold": 0.1},
)
```

### Speed

Speed is controlled at two levels. LocalPlanner caps how fast it will plan, PathFollower caps how fast it will execute.

```python
nav_stack(
    local_planner={"max_speed": 1.5, "autonomy_speed": 1.0},
    path_follower={"max_speed": 1.5, "autonomy_speed": 1.0},
)
```

### Vehicle Height

`vehicle_height` propagated from the top level sets it on TerrainAnalysis (ignore-above filter) and SimplePlanner (ground offset). For FarPlanner, pass it explicitly:

```python
nav_stack(
    vehicle_height=1.2,
    far_planner={"vehicle_height": 1.2},
)
```

### Visualization

Smart Nav includes Rerun visualization configuration out of the box:

```python
from dimos.navigation.nav_stack.main import nav_stack_rerun_config

vis_config = nav_stack_rerun_config(
    user_config=None,          # optional overrides
    agentic_debug=False,       # elevate nav elements for top-down view
)
```

Key visual elements:
- **terrain_map** -- green=ground, red=obstacle (height-based coloring)
- **path** -- green line showing the local planner's chosen trajectory
- **goal_path** -- orange/yellow global plan
- **way_point** -- red sphere at the current intermediate target
- **goal** -- purple sphere at the navigation destination

Set `agentic_debug=True` to raise goals, paths, and waypoints 3m above the scene for a clear top-down view when terrain occludes planning elements.

### Module Parameter Reference

<details>
<summary><b>TerrainAnalysis</b> -- classifies lidar points into ground vs. obstacle, publishes a terrain cost map</summary>

| Parameter | Default | Description |
|-----------|---------|-------------|
| `obstacle_height_threshold` | 0.1 m | Height above ground to classify as obstacle |
| `ground_height_threshold` | 0.1 m | Ground classification threshold |
| `vehicle_height` | 1.5 m | Ignore points above this height |
| `terrain_voxel_size` | 0.2 m | Terrain grid cell size |
| `terrain_voxel_half_width` | 10 | Grid radius in cells (full grid = 2N+1) |
| `decay_time` | 1.0 s | Point expiry time |
| `clearing_distance` | 8.0 m | Dynamic obstacle clearing distance |
| `scan_voxel_size` | 0.05 m | Input scan downsampling |
| `min_relative_z` | -1.5 m | Height-band filter min |
| `max_relative_z` | 0.3 m | Height-band filter max |

</details>

<details>
<summary><b>LocalPlanner</b> -- evaluates candidate paths against terrain/obstacles to select collision-free trajectories</summary>

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_speed` | 2.0 m/s | Maximum velocity |
| `autonomy_speed` | 1.0 m/s | Velocity cap in autonomous mode |
| `obstacle_height_threshold` | 0.15 m | Height to classify as obstacle |
| `goal_clearance` | 0.5 m | Minimum clearance around the goal |
| `two_way_drive` | false | Allow reverse driving |
| `use_terrain_analysis` | true | Use terrain cost map for avoidance |
| `min_relative_z` | -0.4 m | Height-band filter min |
| `max_relative_z` | 0.3 m | Height-band filter max |
| `vehicle_length`, `vehicle_width` | -- | Robot footprint dimensions |

</details>

<details>
<summary><b>PathFollower</b> -- pure-pursuit controller with PID yaw control</summary>

| Parameter | Default | Description |
|-----------|---------|-------------|
| `look_ahead_distance` | 0.5 m | Pure pursuit lookahead |
| `max_speed` | 2.0 m/s | Maximum velocity |
| `max_yaw_rate` | 80.0 deg/s | Maximum turning rate |
| `goal_tolerance` | 0.3 m | Path-end distance threshold |
| `autonomy_speed` | -- | Autonomous velocity cap (overrides max_speed) |
| `max_acceleration` | -- | Linear acceleration limit |
| `vehicle_config` | `"omniDir"` | Kinematics model (`"omniDir"` or `"standard"`) |

</details>

<details>
<summary><b>FarPlanner</b> -- visibility-graph global planner</summary>

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sensor_range` | 30.0 m | Sensor range for graph building |
| `terrain_range` | 7.5 m | Terrain processing range |
| `local_planner_range` | 2.5 m | Overlap with local planner |
| `robot_dimension` | 0.5 m | Robot footprint size |
| `vehicle_height` | 0.75 m | Robot height |
| `converge_dist` | 1.5 m | Goal convergence distance |
| `goal_adjust_radius` | 10.0 m | Goal adjustment search radius |
| `update_rate` | 5.0 Hz | Planning rate |

</details>

<details>
<summary><b>SimplePlanner</b> -- grid-based A* with stuck detection</summary>

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cell_size` | 0.3 m | Costmap grid resolution |
| `obstacle_height_threshold` | 0.15 m | Height to classify as obstacle |
| `inflation_radius` | 0.2 m | Safety margin around obstacles |
| `lookahead_distance` | 2.0 m | Waypoint lookahead on the path |
| `replan_rate` | 5.0 Hz | Planning loop frequency |
| `replan_cooldown` | 2.0 s | Minimum time between A* searches |
| `stuck_seconds` | 5.0 s | Time stationary before declaring stuck |
| `progress_epsilon` | 0.25 m | Minimum progress to not be stuck |
| `stuck_shrink_factor` | 0.5 | Inflation shrink per stuck escalation |
| `stuck_min_inflation` | 0.2 m | Floor for inflation shrink |

</details>

<details>
<summary><b>PGO</b> -- keyframe-based loop closure with ICP + GTSAM iSAM2</summary>

| Parameter | Default | Description |
|-----------|---------|-------------|
| `key_pose_delta_trans` | 0.5 m | Translation threshold for new keyframe |
| `key_pose_delta_deg` | 10 deg | Rotation threshold for new keyframe |
| `loop_search_radius` | 15.0 m | Radius to search for loop closures |
| `loop_time_thresh` | 60.0 s | Minimum time gap for loop candidate |
| `loop_score_thresh` | 0.3 | ICP fitness score threshold |
| `global_map_publish_rate` | 0.5 Hz | Map publication frequency |
| `global_map_voxel_size` | 0.15 m | Map voxel downsampling |

</details>

<details>
<summary><b>TerrainMapExt</b> -- persistent rolling voxel grid for wider terrain context</summary>

| Parameter | Default | Description |
|-----------|---------|-------------|
| `voxel_size` | 0.1 m | Voxel cell size |
| `decay_time` | 30.0 s | Point expiry time |
| `publish_rate` | 2.0 Hz | Publication frequency |
| `max_range` | 40.0 m | Maximum distance from robot |

</details>

<details>
<summary><b>MovementManager</b> -- multiplexes teleop and autonomous velocity, relays goals</summary>

| Parameter | Default | Description |
|-----------|---------|-------------|
| `tele_cooldown_sec` | 1.0 s | Cooldown before nav re-enables after teleop |

</details>

## Architecture

```mermaid
flowchart TB
    subgraph external [External Inputs]
        lidar[/"registered_scan\n(PointCloud2)"/]
        odom[/"odometry\n(Odometry)"/]
        click[/"clicked_point\n(PointStamped)"/]
        teleop[/"tele_cmd_vel\n(Twist)"/]
    end

    subgraph pgo_block [Pose Graph Optimization]
        PGO
    end

    subgraph terrain [Terrain Processing]
        TA[TerrainAnalysis]
        TME[TerrainMapExt]
    end

    subgraph planning [Planning]
        MM_goal[MovementManager\n--- goal relay ---]
        FAR["FarPlanner\n(or SimplePlanner)"]
    end

    subgraph local [Local Control]
        LP[LocalPlanner]
        PF[PathFollower]
        MM_vel[MovementManager\n--- velocity mux ---]
    end

    subgraph output [Output]
        cmd[/"cmd_vel\n(Twist)"/]
        corr_odom[/"corrected_odometry\n(Odometry)"/]
        gmap[/"global_map\n(PointCloud2)"/]
    end

    %% Odometry paths
    odom -->|raw odometry| PGO
    odom -.->|raw odometry\n"local frame"| LP
    odom -.->|raw odometry\n"local frame"| PF
    PGO -->|corrected_odometry\n"global frame"| TA
    PGO -->|corrected_odometry| FAR
    PGO -->|corrected_odometry| MM_goal
    PGO --> corr_odom
    PGO --> gmap

    %% Lidar path
    lidar --> PGO
    lidar --> TA
    lidar --> LP

    %% Terrain path
    TA -->|terrain_map| TME
    TA -->|terrain_map| LP
    TME -->|terrain_map_ext| FAR

    %% Goal path
    click --> MM_goal
    teleop --> MM_goal
    MM_goal -->|goal| FAR
    MM_goal -->|stop_movement| FAR

    %% Planning path
    FAR -->|way_point| LP
    FAR -->|goal_path| output

    %% Local control path
    LP -->|path| PF
    PF -->|nav_cmd_vel| MM_vel
    teleop --> MM_vel
    MM_vel --> cmd

    %% Styling
    classDef ext fill:#e8e8e8,stroke:#999,color:#333
    classDef mod fill:#4a90d9,stroke:#2c5f8a,color:#fff
    classDef out fill:#5cb85c,stroke:#3d8b3d,color:#fff
    class lidar,odom,click,teleop ext
    class PGO,TA,TME,MM_goal,FAR,LP,PF,MM_vel mod
    class cmd,corr_odom,gmap out
```

Odometry is split into two paths following the CMU autonomy convention:

- **Local modules** (LocalPlanner, PathFollower) use raw SLAM odometry directly -- they work in the sensor/body frame.
- **Global modules** (FarPlanner/SimplePlanner, TerrainAnalysis, MovementManager) use PGO-corrected odometry for globally consistent coordinates.

## Using with a New Robot

If you have a robot with a Livox Mid-360 lidar and a module that accepts `cmd_vel: In[Twist]`, you can get autonomous navigation running with three blueprints composed via `autoconnect`.

### Minimal Blueprint

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.navigation.nav_stack.main import nav_stack

from my_robot.control import MyRobotControl  # your module

my_robot_nav = (
    autoconnect(
        # 1. Lidar SLAM — produces registered_scan + odometry
        FastLio2.blueprint(
            host_ip="192.168.1.5",       # your machine's IP on the lidar network
            lidar_ip="192.168.1.155",    # the Mid-360's IP
            mount=Pose(z=0.5),           # sensor height above ground
        ),

        # 2. Navigation stack — consumes registered_scan + odometry,
        #    produces cmd_vel
        nav_stack(
            use_simple_planner=True,
            vehicle_height=0.8,          # your robot's height
        ),

        # 3. Your robot — consumes cmd_vel
        MyRobotControl.blueprint(),
    )
    .remappings([
        # FastLio2 publishes "lidar", but nav_stack expects "registered_scan"
        (FastLio2, "lidar", "registered_scan"),
    ])
)
```

### What Your Robot Module Needs

The only requirement is a module with a `cmd_vel: In[Twist]` stream that subscribes to velocity commands and drives the hardware. A minimal example:

```python
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Twist import Twist


class MyRobotConfig(ModuleConfig):
    pass


class MyRobotControl(Module):
    config: MyRobotConfig
    cmd_vel: In[Twist]

    @rpc
    def start(self) -> None:
        super().start()
        self.cmd_vel.subscribe(self._on_cmd_vel)

    def _on_cmd_vel(self, twist: Twist) -> None:
        vx = twist.linear.x      # forward/back (m/s)
        vy = twist.linear.y      # strafe left/right (m/s)
        vyaw = twist.angular.z   # rotation (rad/s)
        # ... send to your hardware SDK ...
```

### Key Wiring Details

- **Stream name remap**: FastLio2 outputs `lidar`, but nav_stack expects `registered_scan`. The `.remappings()` call handles this. The `odometry` stream name matches on both sides, so it connects automatically.
- **`mount` pose**: Set this to your sensor's position relative to the ground. The z component shifts the SLAM origin so ground sits at z=0, which is critical for terrain analysis to classify obstacles correctly.
- **`vehicle_height`**: Tells TerrainAnalysis to ignore lidar points above the robot (e.g. ceilings). Set it to your robot's actual height.
- **`cmd_vel` convention**: `linear.x` = forward, `linear.y` = strafe, `angular.z` = yaw rate. If your robot is differential-drive (no strafe), set `local_planner={"two_way_drive": False}` and `path_follower={"vehicle_config": "standard"}`.

### Adding Visualization

To see what the navigation stack is doing, add a Rerun bridge:

```python
from dimos.navigation.nav_stack.main import nav_stack, nav_stack_rerun_config
from dimos.visualization.rerun.bridge import RerunBridgeModule

my_robot_nav = (
    autoconnect(
        FastLio2.blueprint(...),
        nav_stack(...),
        MyRobotControl.blueprint(),
        RerunBridgeModule.blueprint(**nav_stack_rerun_config()),
    )
    .remappings([
        (FastLio2, "lidar", "registered_scan"),
    ])
)
```

### Adding Teleop

Smart Nav's MovementManager accepts `tele_cmd_vel` for manual override. When teleop commands arrive, MovementManager cancels the active navigation goal and forwards teleop velocities directly. After `tele_cooldown_sec` (default 1s) of silence, autonomous navigation resumes.

Wire any module that publishes `tele_cmd_vel: Out[Twist]` (keyboard teleop, joystick, etc.) into the `autoconnect` and it connects automatically.

### Sending Goals

Navigation goals come in via the `clicked_point` stream (`PointStamped` with x/y/z in the map frame). You can:

- Click in the Rerun viewer (if RerunBridgeModule is active)
- Use `dimos agent-send "go to the door"` (if an MCP agent is wired up)
- Publish programmatically from another module with `clicked_point: Out[PointStamped]`
- Use the CLI: `bin/send_clicked_point <x> <y> <z>`
