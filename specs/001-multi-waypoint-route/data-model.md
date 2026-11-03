# Data Model: Multi-Waypoint Route on the Map

Entities from the spec, with the fields and rules needed to build and test them. Held in memory by `RouteFollower`; nothing is persisted.

## Waypoint

One planned stop on the map.

| Field | Type | Rules |
|-------|------|-------|
| `index` | integer, 1-based | Always `1..N` with no gaps. Recomputed after a removal (FR-008). |
| `x`, `y`, `z` | number (metres, frame of the click) | Finite. Within the range limits `MovementManager` applies to clicks. |
| `state` | enum | `pending`, `current`, `reached`, `failed`. |

Identity is the `index` within the route. Two waypoints at the same position are allowed.

### Waypoint state transitions

```text
pending --(becomes the active goal)--> current
current --(robot within pass radius; planner arrival for the last one)--> reached
current --(planner reports failure)--> failed
pending/current --(route stopped, interrupted or held)--> unchanged; route status explains why
```

A waypoint the robot is already within reach of when its turn comes goes `pending -> current -> reached` in one step without the robot moving.

## Route

The ordered list of waypoints plus run status and the add-waypoints setting.

| Field | Type | Rules |
|-------|------|-------|
| `waypoints` | list of Waypoint | At most 20. Order is the order placed. |
| `status` | enum | `planning`, `running`, `completed`, `stopped`, `interrupted`, `failed`, `held`. |
| `add_mode` | boolean | add-waypoints switch. Default `false`. Forced `false` when the Route window goes silent. |
| `current_index` | integer or null | The `current` waypoint while `running` or `held`; null otherwise. |
| `revision` | integer | Increases on every change so consumers can discard stale state. |
| `message` | string or null | Last refusal or end-state text (for example "stop the route first"). |

One route exists at a time, for the one connected robot.

### Route status transitions

```text
planning    --start (>=1 waypoint, not running)-->                      running
running     --last waypoint arrival-->                                  completed
running     --operator stop-->                                          stopped
running     --outside stop_movement or outside goal_request-->          interrupted
running     --planner failure at a waypoint-->                          failed
running     --no Route window signal for heartbeat_timeout_s-->         held
held        --start-->                                                  running (from the first waypoint not yet reached)
held/stopped/interrupted/failed/completed
            --add / remove / clear-->                                   planning
```

Rules:

- `add`, `remove`, `clear` and map clicks are ignored while `running`; each sets `message` (FR-014).
- `start` is ignored while `running`, and refused with no waypoints.
- Entering `completed`, `stopped`, `interrupted`, `failed` or `held` leaves waypoint states as they were so the operator can see which were reached and which were not.
- The first edit after an ended route resets all waypoint states to `pending` and status to `planning`.

## Click handling (decision table)

| `add_mode` | `status` | Map click |
|------------|----------|-----------|
| off | not running | Forwarded unchanged on `clicked_point` (single-point goal, as today) |
| on | not running | Becomes a waypoint (refused with a message at 20); nothing forwarded |
| off or on | running | Ignored; `message` = "stop the route first" |

## Commands (Route window to follower)

| `op` | Fields | Effect |
|------|--------|--------|
| `set_mode` | `enabled` (bool) | Set add-waypoints (the spec and the window call it "add-waypoints"; the field is `add_mode`). |
| `remove` | `index` | Remove that waypoint and renumber. Refused when running. |
| `clear` | none | Remove all waypoints. Refused when running. |
| `start` | none | Begin the route (or resume a held one). |
| `stop` | none | Stop the robot and end the run as `stopped`. |
| `keepalive` | none | Refresh the watchdog. Sent about once per second while the window is open. |

Waypoints are added only by map clicks, not by a command.

## Configuration (module config, not hardcoded)

| Field | Default | Purpose |
|-------|---------|---------|
| `max_waypoints` | 20 | Route length limit (FR-015). |
| `pass_radius_m` | 0.6 | Distance at which the next goal is issued. Must exceed the planner's arrival tolerance. |
| `heartbeat_timeout_s` | 3.0 | Route window silence after which a running route becomes `held`. |
| `state_publish_hz` | 2.0 | Periodic republish so a late consumer is correct. |
