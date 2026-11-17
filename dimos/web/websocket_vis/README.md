# WebSocket Visualization Module

The `WebsocketVisModule` provides real-time data for visualization and control of the robot.

## Overview

Visualization:

- Robot position and orientation
- Navigation paths
- Costmaps

Control:

- Set navigation goal
- Set GPS location goal
- Keyboard teleop (WASD)
- Trigger exploration

## What it Provides

### Inputs (Subscribed Topics)
- `odom` (PoseStamped): Current robot position and orientation
- `gps_location` (LatLon): GPS coordinates of the robot
- `path` (Path): Planned navigation path
- `global_costmap` (OccupancyGrid): Global costmap for visualization

### Outputs (Published Topics)
- `goal_request` (PoseStamped): Goal positions set by user clicks in the web interface
- `gps_goal` (LatLon): GPS goal coordinates set through the interface
- `explore_cmd` (Bool): Command to start autonomous exploration
- `stop_explore_cmd` (Bool): Command to stop exploration
- `tele_cmd_vel` (Twist): Direct movement commands from the interface
- `movecmd_stamped` (TwistStamped): Timestamped movement commands

## How to Use

### Basic Usage

This example adds the interface to the Go2 navigation stack using replay data.
`autoconnect` wires matching stream names and types on the active transport.

```python skip
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

blueprint = autoconnect(
    unitree_go2,
    WebsocketVisModule.blueprint(show_rerun_dashboard=True),
).solidify().global_config(replay=True)

if __name__ == "__main__":
    ModuleCoordinator.build(blueprint).loop()
```

For another robot stack, compose `WebsocketVisModule.blueprint()` with its
blueprint. It must supply the input streams and consume the relevant control
outputs above; GPS controls require a stack with GPS support. Replay displays
recorded sensor data and does not move a physical robot.

### Accessing the Interface

Open the web visualization at the configured port (default: 7779).
