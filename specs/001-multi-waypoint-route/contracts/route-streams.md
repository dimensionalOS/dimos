# Contract: Route streams

Interfaces between the viewer websocket server, `RouteFollower`, `RouteControl` (the Route window), the planner and `MovementManager`. Everything is additive: blueprints that do not include the two new modules behave exactly as before (constitution VIII).

## Modules and ports

### `RouteFollower` (`dimos/navigation/route_follower/module.py`)

| Port | Direction | Type | Remapped to (in `unitree-go2-route`) | Purpose |
|------|-----------|------|--------------------------------------|---------|
| `odom` | in | `PoseStamped` | `odom` | Pass-radius check, robot-position-received check. |
| `viewer_click` | in | `PointStamped` | `viewer_click` (the websocket server's `clicked_point` is remapped here) | Map clicks from the viewer. |
| `goal_reached` | in | `Bool` | `goal_reached` | Final arrival or failure from the planner. |
| `stop_seen` | in | `Bool` | `stop_movement` | Detect outside stops (teleop, agent). |
| `goal_seen` | in | `PoseStamped` | `goal_request` | Detect outside goals (agent, patrol, exploration). |
| `tele_cmd_seen` | in | `Twist` | `tele_cmd_vel` | Filter idle zero-twist `stop_movement` bursts from the viewer (T028). |
| `route_cmd` | in | `String` | `route_cmd` | Commands from the Route window. |
| `clicked_point` | out | `PointStamped` | `clicked_point` | Click forwarded to the planner and `MovementManager` (only when it should act). |
| `route_goal` | out | `PoseStamped` | `goal_request` | Next goal for the planner. |
| `route_stop` | out | `Bool` | `stop_movement` | Operator stop. |
| `route_state` | out | `String` | `route_state` | State document for the Route window and the map overlay. |
| `route_path` | out | `Path` | `route_path` | Line through the waypoints, drawn by the Rerun bridge. |

### `RouteControl` (`dimos/navigation/route_follower/control_window.py`)

| Port | Direction | Type | Purpose |
|------|-----------|------|---------|
| `route_state` | in | `String` | Draws the window from the state document. |
| `route_cmd` | out | `String` | Operator commands and the 1 Hz `keepalive`. |

Remapping names are part of the contract because a module cannot hold an input and an output of the same name; the blueprint, not the modules, ties them to the shared streams.

## Command document (`route_cmd`)

One JSON object per message. Unknown `op` values are ignored and logged.

```json
{ "op": "set_mode", "enabled": true }
{ "op": "remove", "index": 2 }
{ "op": "clear" }
{ "op": "start" }
{ "op": "stop" }
{ "op": "keepalive" }
```

Validation: `op` is one of the six values; `enabled` is a boolean; `index` is an integer; extra fields are ignored.

## State document (`route_state`)

```json
{
  "revision": 12,
  "status": "running",
  "add_mode": false,
  "current_index": 2,
  "message": null,
  "waypoints": [
    { "index": 1, "x": 1.5, "y": -0.5, "z": 0.0, "state": "reached" },
    { "index": 2, "x": 3.0, "y": 0.0, "z": 0.0, "state": "current" },
    { "index": 3, "x": 4.5, "y": 1.0, "z": 0.0, "state": "pending" }
  ],
  "robot_position_known": true
}
```

| Field | Type | Notes |
|-------|------|-------|
| `revision` | integer | Monotonic. A consumer ignores a document older than one it already shows. |
| `status` | string | `planning`, `running`, `completed`, `stopped`, `interrupted`, `failed`, `held`. |
| `add_mode` | boolean | Current add-waypoints setting. |
| `current_index` | integer or null | The `current` waypoint while `running` or `held`. |
| `message` | string or null | Refusal or end-state text. |
| `waypoints` | array | Ordered, at most 20. Each entry has `index`, `x`, `y`, `z`, `state` (`pending`, `current`, `reached`, `failed`). |
| `robot_position_known` | boolean | False until an `odom` message has arrived; drives the Start enable rule (FR-004). |

The follower publishes on every change and again at `state_publish_hz`.

## Map overlay (viewer)

The Go2 viewer shows one 3D view rooted at `world`. The overlay is logged at `world/route_state` (markers) and `world/route_path` (line), in the same space as the viewer's clicks. The route overrides are merged into the Go2's existing `rerun_config`, not added as a second bridge.

- `route_path` is drawn by `Path.to_rerun()` as a line through the waypoints in order.
- A `visual_override` entry in the new blueprint maps the `route_state` document to a Rerun point set, one point per waypoint, labelled with its index, coloured by state: pending grey, current yellow, reached green, failed red. When the route is empty it logs an empty set so old markers disappear.

## Behavioral guarantees

- No `route_cmd` message and no click with add-waypoints on ever moves the robot. Only `start` on a non-empty, non-running route publishes a goal.
- With add-waypoints off and no route running, a viewer click reaches `clicked_point` unchanged (SC-008).
- `stop` is honoured in every status and publishes `Bool(True)` on `stop_movement`.
- While `running`, every viewer click is dropped and `message` says to stop the route first; `set_mode` is still accepted but has no effect on clicks until the route ends.
- A duplicate `start` while `running` changes nothing.
- No command for `heartbeat_timeout_s` while `running` moves the route to `held` after the current leg, and sets `add_mode` to false whether running or not.
- Any `goal_request` message that does not match the follower's last published goal, or any `stop_movement` the follower did not publish, ends a running route as `interrupted`.
