# WebSocket Visualization Module

The `WebsocketVisModule` provides a real-time data for visualization and control of the robot in Foxglove (see `dimos/web/command-center-extension/README.md`).

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
- `robot_pose` (PoseStamped): Current robot position and orientation
- `gps_location` (LatLon): GPS coordinates of the robot
- `path` (Path): Planned navigation path
- `global_costmap` (OccupancyGrid): Global costmap for visualization

### Outputs (Published Topics)
- `click_goal` (PoseStamped): Goal positions set by user clicks in the web interface
- `gps_goal` (LatLon): GPS goal coordinates set through the interface
- `explore_cmd` (Bool): Command to start autonomous exploration
- `stop_explore_cmd` (Bool): Command to stop exploration
- `movecmd` (Twist): Direct movement commands from the interface
- `movecmd_stamped` (TwistStamped): Timestamped movement commands

## How to Use

### Basic Usage

```python
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule
from dimos import core

# Deploy the WebSocket visualization module
websocket_vis = dimos.deploy(WebsocketVisModule, port=7779)

# Receive control from the Foxglove plugin.
websocket_vis.click_goal.transport = core.LCMTransport("/goal_request", PoseStamped)
websocket_vis.explore_cmd.transport = core.LCMTransport("/explore_cmd", Bool)
websocket_vis.stop_explore_cmd.transport = core.LCMTransport("/stop_explore_cmd", Bool)
websocket_vis.movecmd.transport = core.LCMTransport("/cmd_vel", Twist)
websocket_vis.gps_goal.transport = core.pLCMTransport("/gps_goal")

# Send visualization data to the Foxglove plugin.
websocket_vis.robot_pose.connect(connection.odom)
websocket_vis.path.connect(global_planner.path)
websocket_vis.global_costmap.connect(mapper.global_costmap)
websocket_vis.gps_location.connect(connection.gps_location)

# Start the module
websocket_vis.start()
```

### Accessing the Interface

See `dimos/web/command-center-extension/README.md` for how to add the command-center plugin in Foxglove.
