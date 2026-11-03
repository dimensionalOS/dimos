# Quickstart: Validating the multi-waypoint route

Run these in order. Do not skip to hardware. Stream details are in [contracts/route-streams.md](contracts/route-streams.md) and state rules in [data-model.md](data-model.md).

## Prerequisites

```bash
bash scripts/install.sh --mode dev --project-dir .
source .venv/bin/activate
```

A desktop session is needed for the viewer and the Route window.

## 1. Automated checks

```bash
uv run pytest dimos/navigation/route_follower dimos/robot/unitree/go2/blueprints/smart
uv run pytest dimos/robot/test_all_blueprints_generation.py
uv run mypy dimos/
```

Expected: all pass, `all_blueprints.py` is current, no type errors.

## 2. Simulation, end to end

```bash
uv run dimos --simulation run unitree-go2-route
```

The native `dimos-viewer` and the Route window both open. Use the viewer's map for clicks and the Route window for controls.

| Step | Action | Expected |
|------|--------|----------|
| S1 | Turn add-waypoints on, click 3 points on the map | Three numbered markers and a joining line appear on the map and in the Route window list; the robot does not move (FR-002, SC-003). |
| S2 | Press Start | Robot goes to 1, passes through 2 without stopping, stops at 3. Markers turn green in order and the current one is yellow (FR-005, FR-006, FR-009). With no key pressed the route runs to completion and is never marked interrupted. Record the minimum robot speed between entering one waypoint's pass radius and reaching the next from the recorded `odom`; it stays above 0.1 m/s (SC-002). |
| S3 | Place 4 waypoints, remove #2, then Clear | Remaining waypoints renumber 1 to 3 on the map, then the map is empty and Start is disabled (FR-007, FR-008, FR-004). |
| S4 | Start a route, press Stop mid-way | Robot is stationary within 2 s, no later waypoint is pursued, reached and unreached markers stay visible (FR-010, SC-004). |
| S5 | Put waypoint 2 inside an obstacle, press Start | Robot stops after 1; waypoint 2 shows failed (red) with a message; 3 is not attempted (FR-011, SC-005). |
| S6 | During a route, press a teleop key | Route ends as interrupted (FR-012). |
| S7 | During a route, click the map with add-waypoints on, then off; press Start again | Both clicks ignored with "stop the route first"; the robot is not redirected; second Start ignored (FR-014). |
| S8 | During a route, click the window's close button, then in `dimos shell` call `route_control.pause_keepalive(True)` | The close is ignored with "stop the route first". Within about 3 s of pausing the keepalive the robot finishes its current leg and stops at that waypoint; it does not start the next leg. Call `route_control.pause_keepalive(False)` (and `route_control.show_window()` if the window is hidden): route shows held; Start resumes from the first unreached waypoint (FR-017). |
| S9 | Open two viewers (or keep the viewer reconnecting) | Both show the same overlay (FR-013). |
| S10 | Add-waypoints off, click the map | Robot goes to that point exactly as before (FR-016, SC-008). |
| S11 | During a route, trigger another goal (agent send, or patrol) | Route ends as interrupted (FR-012). |
| S12 | Add-waypoints on with no route running, call `route_control.pause_keepalive(True)` in `dimos shell`, wait 3 s, click the map | The click behaves as a normal single-point goal because the switch reset to off (FR-017). Call `route_control.pause_keepalive(False)` afterwards. |

## 3. Replay

```bash
uv run dimos --replay run unitree-go2-route
```

Repeat S1, S3 and S10. Replay has no live robot, so confirm no commands change robot state.

## 4. Hardware, staged (Go2 PRO)

Only after sections 1 to 3 pass. Have a person at the manual emergency stop and a clear floor with at least 2 m of free space around the path.

1. Stage A, no motion: connect, turn add-waypoints on, place three waypoints, confirm the robot does not move. Clear.
2. Stage B, one short leg: one waypoint 1 m ahead, Start. Stop it once mid-leg (confirm halt), then run it again to the end.
3. Stage C, two waypoints about 1.5 m apart in open floor: confirm pass-through (no stop at waypoint 1) and a clean stop at waypoint 2.
4. Stage D, interruption: during Stage C press a teleop key; confirm the route ends as interrupted.
5. Stage E, three waypoints with a corner: confirm order and stop at the last.
6. Stage F, window loss: during Stage C close the Route window; confirm the robot finishes the leg and holds.
7. Record results in the PR description: stage, date, outcome.

Stop the whole test and return to simulation if the robot ever moves while only editing the route, does not halt on Stop, or skips a waypoint.
