# Research: Multi-Waypoint Route on the Map

Facts below were read from the repository on 2026-11-02. This revision replaces the earlier browser-cockpit research: the target is the native `dimos-viewer` application, whose compiled code is outside this repository. Items marked **verify** are assumptions that the first foundational task must confirm before building on them.

## R1. What the "standalone UI" is and what can change

- **Decision**: The target is `dimos-viewer`, the native desktop app that `dimos run` opens by default. It is a prebuilt wheel (`dimos-viewer==0.32.0a2`, a 119 MB compiled executable at `.venv/bin/dimos-viewer`) launched by `spawn_viewer()` in `dimos/visualization/rerun/init.py`. Its UI cannot be modified here, so the route controls go in a companion desktop window and the route is drawn on the viewer's map from this repo.
- **Rationale**: The viewer talks to this repo through two channels: Rerun data from `RerunBridgeModule` (anything logged is drawn) and a websocket to `RerunWebSocketServer` carrying `click`, `twist`, `stop` and `heartbeat` messages. Both can be used without changing the viewer.
- **Alternatives considered**: Changing `dimos-viewer` (separate repository and release, not available here). The browser cockpit (not the application the user runs). A terminal or agent-only trigger (no button).

## R2. Where the route lives

- **Decision**: A new `RouteFollower` module owns the route, the add-waypoints setting and the run logic. A separate `RouteControl` module owns the Route window. They talk over two string streams carrying JSON, so the window can be absent (headless) and the follower can be tested without a display.
- **Rationale**: The follower needs odometry, planner results and stop signals, which all live on the robot side. Keeping the window a thin client matches how `KeyboardTeleopModule` wraps a pygame UI as a Module (`dimos/teleop/keyboard/keyboard_teleop_module.py`).
- **Alternatives considered**: One module that does both (hard to test without a display, mixes UI and control). Putting the route in the planner (couples a general planner to one feature).

## R3. Capturing map clicks without moving the robot

- **Decision**: Interpose the follower between the websocket server and its consumers. In the new blueprint the server's `clicked_point` output is remapped to `viewer_click`; the follower reads it and re-publishes on `clicked_point` only when it should reach the planner and `MovementManager`.
  - add-waypoints off and no route running: forward the click unchanged (today's behaviour, SC-008).
  - add-waypoints on and no route running: turn the click into a waypoint, forward nothing.
  - route running: ignore the click and set a "stop the route first" message.
- **Rationale**: Today the server publishes `clicked_point` (`websocket_server.py`, `_dispatch`), and both the planner (`module.py`, `clicked_point.subscribe`) and `MovementManager._on_click` act on it immediately. Only an interposed module can stop a click from moving the robot. This also satisfies the clarified rule that all clicks are ignored during a run, for any viewer panel.
- **Alternatives considered**: Listening in addition to the planner (the robot would still move on every click). Changing the server (couples it to routes and affects every blueprint). Editing `MovementManager` and the planner (wider change, more regression risk).
- **Verified (V1, by code reading)**: `ModuleCoordinator._connect_streams` (`dimos/core/coordination/module_coordinator.py:379`) groups every stream of every module by its remapped `(name, type)` and gives each group one shared transport, so remapping the websocket server's `clicked_point` to `viewer_click` and giving the follower a matching input and a `clicked_point` output works. `RerunWebSocketServer` appears once in the composed blueprint (added by `vis_module`), so the class-keyed remapping in `Blueprint.remappings()` is unambiguous. The Go2 blueprints pin no transports for these streams. Still to confirm at run time: the follower's forwarded `clicked_point` reaches `ReplanningAStarPlanner` and `MovementManager` unchanged (quickstart S10).

## R4. How goals reach the planner

- **Decision**: The follower publishes `PoseStamped` goals on the planner's existing `goal_request` stream, through an output named `route_goal` remapped to `goal_request`. It also holds an input `goal_seen` remapped to the same `goal_request`, so it sees every goal on that stream, including its own.
- **Rationale**: `ReplanningAStarPlanner` subscribes to `goal_request` and `handle_goal_request` replaces the current goal and replans (`global_planner.py:143`). Several modules publish to `goal_request` already (`PatrollingModule`, `WavefrontFrontierExplorer` in the same blueprint), which is the "another source" the spec requires the route to detect. A module cannot declare an input and an output with the same name, hence the two names and two remappings.
- **Alternatives considered**: Calling `set_goal` over RPC (no way to see other sources). Using `target` (also published by `PersonTracker`, and the planner treats it the same).
- **Verified (V1, by code reading)**: Nothing in `_connect_streams` limits publishers or subscribers per stream: an `In` and an `Out` remapped onto one name, and several `Out` ports on one name, all attach to the same transport. The one rule to respect is that a stream name must keep a single message type, because a name used with two types gets a random topic (`_get_transport_for` / `_is_name_unique`). `goal_request` stays `PoseStamped`, `stop_movement` `Bool`, `clicked_point` `PointStamped`. The Spec/RPC fallback is therefore not needed unless the T008 test shows otherwise.

## R5. Pass-through at intermediate waypoints

- **Decision**: The follower watches `odom`. When the robot is within `pass_radius_m` (default 0.6 m) of the current intermediate waypoint, it marks it reached and immediately publishes the next goal. The final waypoint is not pass-through: the planner's own arrival and `goal_reached(True)` end the route.
- **Rationale**: The planner stops at every goal. On arrival it calls `cancel_goal(arrived=True)`, clears the path and emits `goal_reached(True)`. Waiting for that would stop the robot at every waypoint, which clarification Q1 rejected. A new goal request replans without emitting `goal_reached` (`_plan_path` calls `cancel_goal(but_will_try_again=True)`, which skips the event). The planner's arrival tolerance is 0.2 m, so the pass radius must be larger.
- **Alternatives considered**: Sending the whole route as one path (not supported by the planner interface). A time-based hand-off (not tied to position).

## R6. Completion, failure and interruption

- **Decision**: Use `goal_reached`, the `stop_movement` stream and the goals seen on `goal_request`.
  - `goal_reached(True)` while the final waypoint is current: completed.
  - `goal_reached(False)` with no stop requested by the follower and no outside stop or goal: the current waypoint failed (`_plan_path` and `_replan_path` call `cancel_goal()` on no safe goal, no path or retries exhausted).
  - `stop_movement(True)` that the follower did not publish, for example `MovementManager._cancel_goal` when teleop input arrives: interrupted.
  - A goal on `goal_request` that does not match the one the follower last published (position and timestamp): interrupted.
- **Rationale**: These are the only signals the planner exposes. The follower knows what it published, so it can tell its own stop and goals from outside ones. `MovementManager` already publishes `stop_movement` on any teleop input, which gives FR-012 for teleop. The `goal_request` check covers agents, patrol and exploration.
- **Alternatives considered**: Polling `navigation_state` (declared, marked `TODO: set it`, so unreliable). Adding a failure reason to the planner (a wider change than needed).
- **Risk**: A `goal_reached(False)` can arrive just after a stop. The follower sets a short "stop pending" guard before it publishes its own stop and ignores the matching event; covered by tests with a fake planner.
- **Verified (V1, by code reading)**: `stop_seen` remapped to `stop_movement` shares the transport that `MovementManager.stop_movement` publishes on. `stream_name_types(blueprint)` in `module_coordinator.py:790` lists every wired `(name, type)` without starting workers, which is what T008 should assert against.

## R7. Operator Stop

- **Decision**: Stop publishes `Bool(True)` on `stop_movement` through an output `route_stop` remapped to that name, then ends the route as `stopped`.
- **Rationale**: Every client already trusts this stream. `ReplanningAStarPlanner._on_stop_movement` cancels the goal and clears the path. No new stop mechanism means no new way for stop to fail (constitution III). `MovementManager` publishes on the same stream already, so sharing it is consistent.
- **Alternatives considered**: A route-specific stop honoured only by the follower (could leave a planner goal running).

## R8. Drawing the route on the viewer map

- **Decision**: The follower publishes `route_path: Out[Path]`, which `Path.to_rerun()` already draws as a line, and publishes the route state document, which the bridge turns into numbered, coloured points through a `visual_override` entry in the new blueprint (the "sideloading" mechanism in `bridge.py`, line 94 onward).
- **Rationale**: Anything on a stream the bridge subscribes to is drawn in the viewer with no viewer change. A `visual_override` function can return an `rr.Points3D` with per-point labels (order numbers) and colours by waypoint state.
- **Alternatives considered**: Logging directly with `rr.log` from the module after `rerun_init()` (documented under "Direct Visualization from a Module", but starting a second Rerun init in a worker is riskier than using the bridge). One entity per waypoint (more messages, same result).
- **Verified (V2, mostly by code reading; one simulation check left)**: The Go2 viewer layout is a single 3D view rooted at `world` (`_go2_rerun_blueprint` in `unitree_go2_basic.py`), and clicks arrive as x, y, z in that scene's space, which is the frame the planner already treats clicks in (single-click navigation works today). The bridge logs a message under `world/<topic>` and attaches it to a TF frame only if the message has a `frame_id` (`bridge.py` `_on_message`); the `String` route state has none, so markers are placed directly in `world` space, the same space as the clicks. Remaining check: confirm in simulation that a marker lands under the cursor position (quickstart S1).
- **Verified (V3, by code reading) with a constraint**: Overrides are keyed by entity path (`world/<stream>`) in `rerun_config["visual_override"]`, but that dict is baked into the `vis_module(...)` call inside `unitree_go2_basic`. Autoconnecting a second `RerunBridgeModule` atom would replace the first (`_eliminate_duplicates`: newer wins) and silently drop the Go2 overrides and viewer layout. The safe way is to import `rerun_config` from `unitree_go2_basic`, merge the route overrides into a copy, and add `vis_module(viewer_backend=global_config.viewer, rerun_config=merged)` after `unitree_go2`, which also respects `--viewer none`. Converters must be module-level functions or `functools.partial` (the config is pickled to the viewer's worker).

## R9. The Route window

- **Decision**: `RouteControl` is a pygame window run in its own thread, following `KeyboardTeleopModule`: `SDL_VIDEODRIVER=x11`, `ImportError` with an install hint if pygame is missing, a `threading.Event` stop flag, thread joined in `stop()`. The drawing and hit-testing logic is separate, pure code so it can be tested without opening a window.
- **Closing and reopening**: closing the window never ends the module. A close request is ignored while a route runs (with a "stop the route first" message); otherwise it hides the window and sends `set_mode` with `enabled` false, so hidden-window clicks are never swallowed. `show_window()` (RPC, callable from `dimos shell`) reopens it. Reopening is what makes resuming a held route possible in the same session.
- **Contents**: add-waypoints switch, Start, Stop, Clear, a waypoint list with a remove control per row, the status, the current target ("waypoint 2 of 5") and the latest message.
- **Rationale**: pygame is already a project dependency for this purpose, so no new dependency and a familiar pattern. The window is a desktop app window, satisfying the request for a button.
- **Alternatives considered**: tkinter (not used in the repo). A key-bound command in the viewer (the viewer's key handling is compiled and sends only `twist`/`stop`). Embedding in the viewer (not possible here).

## R10. Window presence and the held state

- **Decision**: While the `RouteControl` module is alive (window shown or hidden) it sends a `keepalive` command about once per second; a test hook `pause_keepalive(paused)` stops it for simulation. If the follower sees no command for `heartbeat_timeout_s` (3 s) while a route runs, the route becomes `held`: it stops advancing, the robot finishes its current leg and stops at that waypoint, and Start later resumes from the first waypoint not yet reached. Add-waypoints also switches off when the window goes silent, whether or not a route is running.
- **Rationale**: Mirrors the teleop watchdog pattern: the robot-side module is authoritative and needs no knowledge of the window process. Resetting add-waypoints prevents clicks from being absorbed with nobody to see why.
- **Status**: Confirmed by the operator on 2026-11-02: 3 s of silence, finish the leg, stop at that waypoint (recorded in the spec Clarifications).
- **Alternatives considered**: Stopping immediately where the robot is (abrupt, possibly unsafe on a moving legged robot). Continuing regardless (violates the safety principle).

## R11. Multiple viewers and who may command

- **Decision**: Any number of viewers may be open; they all show the same overlay because it comes from the single state stream. Route commands come only from the Route window. Stop must work from every client, so `stop_movement` stays the shared stop path.
- **Rationale**: Spec FR-013. No lease is needed, because only one Route window issues commands in practice; a second window would send valid commands that are applied in arrival order under a lock.
- **Alternatives considered**: A lease for editing (more machinery than the spec asks for).

## R12. Frame and waypoint pose

- **Decision**: A waypoint is the click's x, y (and z) in the frame the click arrives in. The goal pose uses that position with identity orientation, exactly as `PointStamped.to_pose_stamped()` does for plain clicks today.
- **Rationale**: Spec excludes per-waypoint heading. This matches how single clicks reach the planner.

## R13. Validation of inputs

- **Decision**: Validate in the follower: finite coordinates, the same range limits `MovementManager._on_click` applies, an integer in-range index for `remove`, at most 20 waypoints, and edits refused while running. Errors become a `message` field in the state document.
- **Rationale**: The follower must not trust the window or any other publisher on the command stream.

## R14. Validation path

- **Decision**: Unit tests for the state machine, messages and window logic; module tests with a fake planner and fake streams; a blueprint-level test for the remappings; then `dimos --simulation run unitree-go2-route`; then replay; then the staged hardware checklist in `quickstart.md`.
- **Rationale**: Constitution III requires simulation or replay before hardware and a written staged hardware checklist.
