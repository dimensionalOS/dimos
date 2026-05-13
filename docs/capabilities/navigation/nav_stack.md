# Nav Stack

A modular navigation stack for autonomous robot navigation: terrain classification, obstacle avoidance, global path planning, local trajectory selection, and loop-closure-corrected mapping — composed as Blueprint modules.

Good fit when you have a lidar-equipped robot and need end-to-end autonomy: feed it a registered point cloud and odometry, and it produces velocity commands. No ROS — modules communicate over DimOS streams (LCM/SHM).

```python session=nav_stack
from dimos.navigation.nav_stack.main import create_nav_stack

blueprint = create_nav_stack()
```

## Streams

The stack consumes (typically from a SLAM module like `FastLio2`):

| Stream | Type | Description |
|--------|------|-------------|
| `registered_scan` | `PointCloud2` | World-frame lidar scan |
| `odometry` | `Odometry` | SLAM odometry |

It needs a goal source — `way_point` (`PointStamped`) drives the planners. A separate `MovementManager` module (`dimos/navigation/movement_manager/movement_manager.py`) is the usual goal source: it accepts `clicked_point` from a viewer/agent and produces `way_point`, plus it muxes `nav_cmd_vel` with `tele_cmd_vel` into the final `cmd_vel`.

The stack produces:

| Stream | Type | Description |
|--------|------|-------------|
| `nav_cmd_vel` | `Twist` | Velocity command (becomes `cmd_vel` after MovementManager) |
| `corrected_odometry` | `Odometry` | PGO loop-closure-corrected pose |
| `global_map_pgo` | `PointCloud2` | Accumulated keyframe map |

## Customizing

All configuration goes through `create_nav_stack()` keyword arguments. Top-level switches plus per-module config dicts:

```python skip
create_nav_stack(
    planner="simple",              # "far" (default) or "simple" (A*)
    use_tare=False,                # Add TARE frontier exploration
    use_terrain_map_ext=True,      # Persistent terrain accumulator
    vehicle_height=None,           # Propagated to terrain + planners
    max_speed=None,                # Propagated to local planner + path follower
    waypoint_threshold=None,       # "Close enough" distance (m)
    terrain_voxel_size=0.2,
    replan_rate=0.5,               # Global planner replan rate (Hz)
    record=False,                  # Enable NavRecord module

    # Per-module config overrides (merged onto defaults):
    terrain_analysis={...},
    local_planner={...},
    path_follower={...},
    far_planner={...},
    simple_planner={...},
    pgo={...},
    tare_planner={...},
    terrain_map_ext={...},
    nav_record={...},
)
```

### Global planner

- **FarPlanner** (default) — visibility-graph planner, larger sensor range. Better for outdoor/long-range.
- **SimplePlanner** (`planner="simple"`) — grid-based A*. Lighter, easier to debug.

### Exploration

`use_tare=True` adds the TARE frontier exploration module, which takes over waypoint generation and drives toward unexplored frontiers.

### Obstacle sensitivity

Keep `obstacle_height_threshold` aligned between TerrainAnalysis and LocalPlanner — if TerrainAnalysis flags something but LocalPlanner's threshold is higher, the planner will drive through it.

```python session=nav_stack
create_nav_stack(
    terrain_analysis={"obstacle_height_threshold": 0.1},
    local_planner={"obstacle_height_threshold": 0.1},
)
```

### Speed

Capped at two levels: LocalPlanner caps how fast it will *plan*, PathFollower caps how fast it will *execute*.

```python session=nav_stack
create_nav_stack(
    local_planner={"max_speed": 1.5, "autonomy_speed": 1.0},
    path_follower={"max_speed": 1.5, "autonomy_speed": 1.0},
)
```

## Visualization

```python
from dimos.navigation.nav_stack.main import nav_stack_rerun_config

vis_config = nav_stack_rerun_config(
    user_config=None,
    agentic_debug=False,    # lift nav elements above terrain for top-down clarity
)
```

Key visual elements:
- **terrain_map** — green=ground, red=obstacle (height-based)
- **path** — green line, local planner's chosen trajectory
- **goal_path** — orange/yellow global plan
- **way_point** — red sphere at the current intermediate target
- **goal** — purple sphere at the navigation destination

## Architecture

```mermaid
flowchart TB
    subgraph external [External Inputs]
        lidar[/"registered_scan"/]
        odom[/"odometry"/]
        wp[/"way_point"/]
    end

    subgraph stack [Nav Stack]
        PGO
        TA[TerrainAnalysis]
        TME[TerrainMapExt]
        FAR["FarPlanner / SimplePlanner"]
        LP[LocalPlanner]
        PF[PathFollower]
    end

    subgraph output [Outputs]
        cmd[/"nav_cmd_vel"/]
        corr_odom[/"corrected_odometry"/]
        gmap[/"global_map_pgo"/]
    end

    odom --> PGO
    odom -.->|local frame| LP
    odom -.->|local frame| PF
    PGO -->|corrected_odometry| TA
    PGO -->|corrected_odometry| FAR
    PGO --> corr_odom
    PGO --> gmap

    lidar --> PGO
    lidar --> TA
    lidar --> LP

    TA -->|terrain_map| TME
    TA -->|terrain_map| LP
    TME -->|terrain_map_ext| FAR

    wp --> FAR
    FAR -->|way_point| LP
    LP -->|path| PF
    PF --> cmd

    classDef ext fill:#e8e8e8,stroke:#999,color:#333
    classDef mod fill:#4a90d9,stroke:#2c5f8a,color:#fff
    classDef out fill:#5cb85c,stroke:#3d8b3d,color:#fff
    class lidar,odom,wp ext
    class PGO,TA,TME,FAR,LP,PF mod
    class cmd,corr_odom,gmap out
```

Following the CMU autonomy convention, odometry splits into two paths: **local** modules (LocalPlanner, PathFollower) use raw SLAM odometry in body frame; **global** modules (FarPlanner/SimplePlanner, TerrainAnalysis) use PGO-corrected odometry.

## Using with a new robot

If you have a Livox Mid-360 lidar and a module that consumes `cmd_vel: In[Twist]`, compose three blueprints with `autoconnect`:

```python skip
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack.main import create_nav_stack

from my_robot.control import MyRobotControl  # your module

my_robot_nav = (
    autoconnect(
        FastLio2.blueprint(
            host_ip="192.168.1.5",       # your machine's IP on the lidar network
            lidar_ip="192.168.1.155",
            mount=Pose(z=0.5),           # sensor height above ground
        ),
        create_nav_stack(
            planner="simple",
            vehicle_height=0.8,
        ),
        MovementManager.blueprint(),     # click→goal relay + teleop/nav velocity mux
        MyRobotControl.blueprint(),
    )
    .remappings([
        # FastLio2 publishes "lidar"; nav_stack expects "registered_scan"
        (FastLio2, "lidar", "registered_scan"),
    ])
)
```

### Your robot module

Just needs `cmd_vel: In[Twist]` and a subscription that drives the hardware:

```python skip
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Twist import Twist


class MyRobotControl(Module):
    config: ModuleConfig
    cmd_vel: In[Twist]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))

    def _on_cmd_vel(self, twist: Twist) -> None:
        v_x = twist.linear.x      # forward (m/s)
        v_y = twist.linear.y      # strafe (m/s)
        v_yaw = twist.angular.z   # yaw rate (rad/s)
        # ...send to hardware SDK...
```

### Wiring notes

- **Stream remap** — FastLio2 outputs `lidar`, nav_stack expects `registered_scan`. `odometry` matches on both sides automatically.
- **`mount` pose** — sensor position relative to the ground. `z` shifts the SLAM origin so ground sits at z=0, which TerrainAnalysis depends on.
- **`vehicle_height`** — tells TerrainAnalysis to ignore points above the robot (e.g. ceilings). Propagates to FarPlanner/SimplePlanner automatically.
- **Differential drive** — for robots without strafe, set `path_follower={"vehicle_config": "standard"}`.

### Visualization

Add a Rerun bridge:

```python skip
from dimos.navigation.nav_stack.main import nav_stack_rerun_config
from dimos.visualization.rerun.bridge import RerunBridgeModule

my_robot_nav = autoconnect(
    FastLio2.blueprint(...),
    create_nav_stack(...),
    MovementManager.blueprint(),
    MyRobotControl.blueprint(),
    RerunBridgeModule.blueprint(**nav_stack_rerun_config()),
).remappings([(FastLio2, "lidar", "registered_scan")])
```

### Teleop

`MovementManager` accepts `tele_cmd_vel` for manual override. Teleop commands cancel the active goal and forward velocities directly; after `tele_cooldown_sec` (default 1s) of silence, autonomous navigation resumes. Wire any module that publishes `tele_cmd_vel: Out[Twist]` (keyboard, joystick) into the `autoconnect` and it connects automatically.

### Sending goals

Goals come in via `clicked_point` (`PointStamped`, map frame), which `MovementManager` relays to the planners. You can:

- Click in the Rerun viewer (with `RerunBridgeModule` active)
- `dimos agent-send "go to the door"` (with an MCP agent wired up)
- Publish from another module with `clicked_point: Out[PointStamped]`
- CLI: `bin/send_clicked_point <x> <y> <z>`
