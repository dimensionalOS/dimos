# Multi-Waypoint Route

Plan a route of up to 20 waypoints by clicking the map in `dimos-viewer`, then drive
it from the companion Route window. The robot visits the waypoints in order, passes
through intermediate ones without stopping, and comes to a full stop at the last.

## Run it

```bash
# Simulation (no robot needed)
dimos --simulation run unitree-go2-route

# Replay recorded data
dimos --replay run unitree-go2-route

# Go2 hardware
dimos run unitree-go2-route --robot-ip 192.168.123.161
```

The native viewer and the Route window both open. Use the viewer map for clicks and
the Route window for controls.

## Requirements

The Route window needs pygame, which the `unitree` extra provides:

```bash
pip install 'dimos[unitree]'
```

The window forces the X11 SDL video driver, so a Wayland-only desktop needs XWayland.

## The Route window

| Control | What it does |
|---------|--------------|
| Add-waypoints switch | Off by default. On: a map click adds a waypoint. Off: a map click sends the robot to that single point exactly as it does today. |
| Start | Begins the route. Disabled with no waypoints, with no robot position, and while a route runs. Also resumes a held route from the first waypoint not yet reached. |
| Stop | Stops the robot and ends the route as stopped. Enabled while a route runs or is held. |
| Clear | Removes every waypoint. Disabled while a route runs. |
| X on a row | Removes that waypoint and renumbers the list 1 to N. Disabled while a route runs. |
| Status line | `planning`, `running`, `completed`, `stopped`, `interrupted`, `failed` or `held`. |
| Target line | The current target, `waypoint N of M`. |
| Message line | The latest refusal or end-state text, for example "stop the route first". |

Closing the window never stops the module. While a route runs the close is ignored
and the window says to stop the route first; otherwise the window hides and switches
add-waypoints off. Reopen it from `dimos shell`:

```bash
dimos shell
```

```bash
# typed into the shell session
route_control.show_window()
```

If the Route window sends no signal for 3 seconds while a route runs, the route
becomes held: the robot finishes its current leg and stops at that waypoint without
starting the next one.

## Map clicks

| Add-waypoints | Route | What a map click does |
|---------------|-------|-----------------------|
| off | not running | Forwarded unchanged. The robot goes to that single point exactly as before. |
| on | not running | Becomes the next waypoint. The robot never moves while editing. |
| on or off | running | Dropped. The message line says to stop the route first. |

Editing the route never moves the robot. Motion begins only on Start. If another
source takes control of the robot (teleop, an agent, any module setting a goal), the
route ends as interrupted.

## On the map

Waypoints are numbered points coloured by state: grey pending, yellow current, green
reached, red failed. A line joins them in order, drawn in a colour distinct from the
planner path.

## Code

- [RouteFollower](/dimos/navigation/route_follower/module.py) owns the route and feeds the planner one goal at a time
- [RouteControl](/dimos/navigation/route_follower/control_window.py) is the pygame window
- [unitree-go2-route](/dimos/robot/unitree/go2/blueprints/smart/unitree_go2_route.py) composes both into the Go2 stack

See also [viewer backends](/docs/usage/visualization.md) and the
[navigation overview](/docs/capabilities/navigation/index.md).
