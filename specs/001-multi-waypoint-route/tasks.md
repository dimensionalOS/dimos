---

description: "Task list for the multi-waypoint route feature (native viewer + Route window)"
---

# Tasks: Multi-Waypoint Route on the Map

**Input**: Design documents from `/specs/001-multi-waypoint-route/`

**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/route-streams.md, quickstart.md

**Tests**: Included. Constitution principle V requires a test that can fail for every new behavior. Tests live next to the code, use no fixed `time.sleep` (use events or polling with a timeout), contain no conditional logic, and fail (not skip) when pygame is missing.

**Organization**: Tasks are grouped by user story so each can be implemented and tested on its own.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependency on an incomplete task)
- **[Story]**: US1 to US4, matching the spec's user stories
- Every task names exact file paths

## Path Conventions

Python under `dimos/`, docs under `docs/`. Every new `.py` file needs the license header (copy it from a neighbour such as `dimos/navigation/base.py`). Run `source .venv/bin/activate` before any commit. `dimos-viewer`, `RerunWebSocketServer`, the planner and `MovementManager` are not modified.

---

## Phase 1: Setup

- [x] T001 Create the working branch `david/feat/multi-waypoint-route` from `main` (constitution VII). Do not commit the untracked `.specify/` or `graphify-out/` folders unless asked.
- [x] T002 Create the package `dimos/navigation/route_follower/` with an empty `__init__.py` (license header only, no re-exports). Confirm which extra a Go2 user installs and whether it includes pygame (declared today only in `control`, `sim` and `tests` in `pyproject.toml`); if not, add pygame to that extra.

---

## Phase 2: Foundational (blocks all user stories)

**Purpose**: State machine, message documents, the stream wiring (the riskiest assumption), the overlay and the empty window. No user-visible behavior yet.

- [x] T003 [P] Write failing unit tests for the `Route` state machine in `dimos/navigation/route_follower/test_route.py`: add up to 20 waypoints and refuse the 21st, remove with renumbering 1..N, clear, status transitions from `data-model.md`, `revision` increments on every change, edits refused while `running`, first edit after an ended route resets waypoint states to `pending`.
- [x] T004 Implement `Route` and `Waypoint` (pure Python, fully typed, thread-safe with a lock) in `dimos/navigation/route_follower/route.py` so T003 passes.
- [x] T005 [P] Write failing tests in `dimos/navigation/route_follower/test_messages.py`: each of the six command ops parses; unknown `op`, non-boolean `enabled`, non-integer `index` are rejected; non-finite or out-of-range click coordinates are rejected; the state document built from a `Route` matches the example in `contracts/route-streams.md` exactly.
- [x] T006 Implement command parsing and state-document building in `dimos/navigation/route_follower/messages.py` so T005 passes. Import the click range limits from `dimos/navigation/movement_manager/movement_manager.py` instead of copying the numbers.
- [x] T007 Create the `RouteFollower` skeleton in `dimos/navigation/route_follower/module.py` with `RouteFollowerConfig` (`max_waypoints=20`, `pass_radius_m=0.6`, `heartbeat_timeout_s=3.0`, `state_publish_hz=2.0`) and exactly the ports in `contracts/route-streams.md` (`odom`, `viewer_click`, `goal_reached`, `stop_seen`, `goal_seen`, `route_cmd` in; `clicked_point`, `route_goal`, `route_stop`, `route_state`, `route_path` out). Register every subscription with `register_disposable`, pair `start()`/`stop()` with `super()`, use `setup_logger()`, do no work in `__init__`. Expose `route_follower = RouteFollower.blueprint`.
- [x] T008 **Verify V1 first (riskiest assumption).** Create `dimos/robot/unitree/go2/blueprints/smart/unitree_go2_route.py` composing `unitree_go2` with `RouteFollower.blueprint()` and the remappings from `contracts/route-streams.md` (websocket server `clicked_point` to `viewer_click`; follower `route_goal` and `goal_seen` to `goal_request`; `route_stop` and `stop_seen` to `stop_movement`), then write `dimos/robot/unitree/go2/blueprints/smart/test_unitree_go2_route.py` that builds it and asserts: the build succeeds; the follower's `viewer_click` is fed by the websocket server; the planner's `clicked_point` and `MovementManager`'s `clicked_point` are fed by the follower; the follower sees a `stop_movement` published by `MovementManager`; the follower sees a `goal_request` published by another module. Follow the remapping syntax in `docs/usage/blueprints.md`. The test inspects the stream connections of the composed blueprint using the graph pattern in `dimos/core/coordination/test_blueprints.py`; it must not deploy workers or build the full robot stack. Assert against `stream_name_types(blueprint)` from `dimos/core/coordination/module_coordinator.py` (it lists every wired `(name, type)` with remappings applied, no workers needed), and assert that each stream name keeps a single message type. Code reading already supports V1 (see research.md); this test makes it permanent. **If this cannot work, stop and switch to the fallback in research.md R4 (Spec Protocol and RPC for goals and stops) before building further.**
- [x] T009 [P] Write failing tests in `dimos/navigation/route_follower/test_overlay.py`: an empty route yields an empty point set; N waypoints yield N points labelled with their index; colours are grey pending, yellow current, green reached, red failed.
- [x] T010 Implement the state-document to Rerun conversion in `dimos/navigation/route_follower/overlay.py` so T009 passes. The converters must be module-level functions or `functools.partial` (the bridge config is pickled to the viewer's worker). Provide `route_state_overlay` for `world/route_state` and a `route_path_overlay` for `world/route_path` that draws the line in a colour distinct from the planner's green path.
- [x] T011 [P] Write failing tests in `dimos/navigation/route_follower/test_control_layout.py`: button rectangles do not overlap and fit the window; hit-testing returns the right control for a point; 20 list rows fit; a row's remove control maps to that row's index; the Start button state is disabled with no waypoints, with `robot_position_known` false, and while `running`.
- [x] T012 Implement the window layout and hit-testing in `dimos/navigation/route_follower/control_layout.py` so T011 passes. Pure logic, no pygame import.
- [x] T013 Create the `RouteControl` window module in `dimos/navigation/route_follower/control_window.py`, following `dimos/teleop/keyboard/keyboard_teleop_module.py`: pygame imported in a `try/except` with an install hint, `SDL_VIDEODRIVER=x11`, a UI thread, a `threading.Event` stop flag, thread joined in `stop()`; `route_state: In[String]` and `route_cmd: Out[String]`; draws from the latest state document using `control_layout.py`. Add `dimos/navigation/route_follower/test_control_window.py` using a headless SDL driver that checks the window starts, redraws on a new state, and closes cleanly on `stop()`. Closing the window never ends the module: a close request hides the window (and sends `set_mode` with `enabled` false) when no route runs and is ignored (with the message "stop the route first") while one runs; add an `@rpc show_window()` that reopens it, with tests for both cases. Also add an `@rpc pause_keepalive(paused: bool)` test hook that stops or resumes the keepalive without touching the window, used by quickstart S8 and S12, with a test. If pygame is missing, raise an `ImportError` that names the extra to install, with a test of that message. Expose `route_control = RouteControl.blueprint`.

**Checkpoint**: `uv run pytest dimos/navigation/route_follower dimos/robot/unitree/go2/blueprints/smart` passes, and V1 holds (or the fallback is chosen).

---

## Phase 3: User Story 1 - Plan a route and start it (Priority: P1) MVP

**Goal**: Turn add-waypoints on, click several map points, press Start, and the robot visits them in order, passing through intermediate waypoints and stopping at the last.

**Independent Test**: Quickstart S1, S2, S10 and S7 in simulation.

### Tests for User Story 1

- [x] T014 [P] [US1] Add failing tests to `dimos/navigation/route_follower/test_route.py` for run logic: `start` with no waypoints is refused, `start` while running is ignored, `start` moves status to `running` and waypoint 1 to `current`, advancing marks the current waypoint `reached` and the next `current`, reaching the last sets `completed`.
- [x] T015 [P] [US1] Add failing tests to `dimos/navigation/route_follower/test_module.py` using fake streams and a fake planner: with add-waypoints off and no route running, a `viewer_click` is re-published unchanged on `clicked_point` (SC-008); with it on, the click becomes a waypoint, nothing is published on `clicked_point`, `route_goal` or `route_stop` (FR-002, SC-003); `set_mode` toggles the setting; `start` publishes exactly one `PoseStamped` for waypoint 1 on `route_goal`; an `odom` within `pass_radius_m` of waypoint 1 publishes waypoint 2 immediately and no stop; the final waypoint publishes its goal and `goal_reached(True)` completes the route; a waypoint already within reach at its turn is reached without waiting; a second waypoint at the same spot works; a duplicate `start` publishes nothing new; while running, a click with add-waypoints on or off is dropped and `message` says to stop the route first (FR-014); `robot_position_known` is false until the first `odom`.
- [x] T016 [P] [US1] Add failing tests to `dimos/navigation/route_follower/test_control_window.py` and `test_control_layout.py`: toggling the switch emits `set_mode`; Start emits `start` only when enabled; the window shows the status, current target ("waypoint N of M") and the message; keepalive is not yet required here.

### Implementation for User Story 1

- [x] T017 [US1] Extend `dimos/navigation/route_follower/route.py` with `start()`, `mark_current_reached()` and completion handling per `data-model.md` so T014 passes.
- [x] T018 [US1] Implement in `dimos/navigation/route_follower/module.py`: the click decision table from `data-model.md`; `set_mode`; `start` publishing waypoint 1 as a `PoseStamped` (identity orientation, frame of the click) on `route_goal`; the pass-radius check on each `odom`; completion on `goal_reached(True)` while the last waypoint is current; publishing `route_state` on every change and at `state_publish_hz`, and `route_path`. Guard all route state with the module lock because stream callbacks run on different threads. Makes T015 pass.
- [x] T019 [US1] Implement the switch, Start button, status line, waypoint list (read-only), current-target label and message line in `dimos/navigation/route_follower/control_window.py` and `control_layout.py` so T016 passes.
- [x] T020 [US1] Add `RouteControl.blueprint()` and the overlay `visual_override` (using `overlay.py`) to `dimos/robot/unitree/go2/blueprints/smart/unitree_go2_route.py`, extend `test_unitree_go2_route.py` to assert the overlay and window ports are connected, then run `uv run dimos --simulation run unitree-go2-route` and do quickstart S1 and S2. Do not add a second `RerunBridgeModule` atom: it would replace the Go2's existing bridge and drop its overrides and viewer layout. Instead import `rerun_config` from `dimos/robot/unitree/go2/blueprints/basic/unitree_go2_basic.py`, merge `{"world/route_state": route_state_overlay, "world/route_path": route_path_overlay}` into a copy of its `visual_override`, and autoconnect `vis_module(viewer_backend=global_config.viewer, rerun_config=merged)` after `unitree_go2` (this also respects `--viewer none`). Add a test that the merged config still contains every original override key. Remaining V2 check in simulation: a marker lands at the clicked position; if it is offset, convert the click in the follower and update `contracts/route-streams.md` and `research.md` R12.

**Checkpoint**: User Story 1 works alone in simulation. This is the MVP, for simulation only (see Implementation Strategy).

---

## Phase 4: User Story 2 - Edit the route before starting (Priority: P2)

**Goal**: Remove a single waypoint or clear the whole route before starting; numbering stays 1..N.

**Independent Test**: Quickstart S3.

### Tests for User Story 2

- [x] T021 [P] [US2] Add failing tests to `dimos/navigation/route_follower/test_module.py`: `remove` with an out-of-range index leaves the route unchanged and sets `message`; the 21st waypoint click is refused with a message (FR-015); `remove`, `clear` and `set_mode` edits while running leave the waypoint list unchanged and `remove`/`clear` set the "stop the route first" message; after `remove` the published state is renumbered 1..N.
- [x] T022 [P] [US2] Add failing tests to `dimos/navigation/route_follower/test_control_layout.py` and `test_control_window.py`: clicking a row's remove control emits `remove` with that index; Clear emits `clear`; both are disabled while `running`; Start is disabled after Clear.

### Implementation for User Story 2

- [x] T023 [US2] Implement the refusal rules and message texts in `dimos/navigation/route_follower/route.py` and the `remove`/`clear` handlers in `dimos/navigation/route_follower/module.py` so T021 passes.
- [x] T024 [US2] Add per-row remove controls and the Clear button to `dimos/navigation/route_follower/control_window.py` and `control_layout.py` so T022 passes.

**Checkpoint**: Stories 1 and 2 work together (quickstart S3).

---

## Phase 5: User Story 3 - Stop a running route and see progress (Priority: P2)

**Goal**: Stop halts the robot at any time; progress is visible; a route ends cleanly on outside interruption or when the Route window disappears.

**Independent Test**: Quickstart S4, S6, S7, S8, S9, S11, S12.

### Tests for User Story 3

- [x] T025 [P] [US3] Add failing tests to `dimos/navigation/route_follower/test_module.py`: `stop` publishes `Bool(True)` on `route_stop` and ends the route `stopped` with unreached waypoints still `pending`; the planner's resulting `goal_reached(False)` does not turn into `failed`; a `stop_seen(True)` the follower did not publish ends a running route `interrupted` (teleop, FR-012); a `goal_seen` that does not match the last goal the follower published ends it `interrupted` (agent, patrol, exploration); the follower's own goals on `goal_seen` do not interrupt; `stop` works in every status.
- [x] T026 [P] [US3] Add failing tests to `dimos/navigation/route_follower/test_module.py` for the watchdog: with a running route and no command for `heartbeat_timeout_s` (use a short configured value and an event or bounded poll, never a fixed sleep) the follower stops advancing, the current leg completes, and status becomes `held`; arrival at an intermediate held waypoint is not `completed`; `keepalive` resets the timer; `start` on a `held` route resumes from the first waypoint not yet reached; silence also sets `add_mode` to false whether or not a route runs.
- [x] T027 [P] [US3] Add failing tests to `dimos/navigation/route_follower/test_control_window.py` and `test_control_layout.py`: Stop emits `stop` and is enabled while `running` or `held`; a `keepalive` is emitted about once per second only while the window is open (use an injectable clock, no real sleeps); status text shows `stopped`, `interrupted`, `held` and `completed` distinctly; a close request while `running` is ignored and shows "stop the route first"; after `show_window()` a hidden window is visible again with the current state.

### Implementation for User Story 3

- [x] T028 [US3] Implement stop handling in `dimos/navigation/route_follower/module.py`: a "stop pending" guard set before publishing `route_stop` so the planner's matching `goal_reached(False)` is ignored; `interrupted` detection from `stop_seen` and `goal_seen` (compare against the follower's last published goal by position within 5 cm and a timestamp window, not exact equality, because message encoding rounds the time). If the viewer sends idle zero-twist messages that make `MovementManager` publish `stop_movement` (it cancels the goal on any `tele_cmd_vel` message), ignore a `stop_seen` that arrives within 100 ms of a zero `tele_cmd_vel` with no nonzero command. Add the `stopped` and `interrupted` transitions to `dimos/navigation/route_follower/route.py`. Makes T025 pass.
- [x] T029 [US3] Implement the heartbeat watchdog in `dimos/navigation/route_follower/module.py` (track last command time with `time.monotonic()`, a tracked thread joined in `stop()`, a `threading.Event` stop flag) and the `held` transition, resume, and add-mode reset in `dimos/navigation/route_follower/route.py`. Makes T026 pass.
- [x] T030 [US3] Add the Stop button, the 1 Hz keepalive and the distinct status text to `dimos/navigation/route_follower/control_window.py` and `control_layout.py`. Makes T027 pass.

**Checkpoint**: Quickstart S4, S6, S7, S8, S9, S11, S12 pass in simulation.

---

## Phase 6: User Story 4 - Handle a waypoint the robot cannot reach (Priority: P3)

**Goal**: An unreachable waypoint stops the robot, is marked failed with a message, and later waypoints are not attempted.

**Independent Test**: Quickstart S5.

### Tests for User Story 4

- [x] T031 [P] [US4] Add failing tests to `dimos/navigation/route_follower/test_module.py`: `goal_reached(False)` with no follower stop pending, no outside stop and no outside goal marks the current waypoint `failed`, sets status `failed`, sets `message` naming the waypoint number, and publishes no further `route_goal`; a failure on waypoint 1 and on a middle waypoint behave the same; the failed state is published within one state publication.
- [x] T032 [P] [US4] Add a failing test to `dimos/navigation/route_follower/test_control_window.py` that a `failed` state shows the failed waypoint and the message, and that Start is available again after the next edit.

### Implementation for User Story 4

- [x] T033 [US4] Implement the failure transition in `dimos/navigation/route_follower/module.py` and `dimos/navigation/route_follower/route.py` per research.md R6. Makes T031 pass.
- [x] T034 [US4] Show the failed waypoint and message in `dimos/navigation/route_follower/control_window.py` and `control_layout.py`. Makes T032 pass.

**Checkpoint**: All four stories work; quickstart S1 to S12 pass in simulation.

---

## Phase 7: Polish and cross-cutting

- [x] T035 [P] Document the route feature in a new page under `docs/capabilities/navigation/` (and a link from `docs/usage/visualization.md`): how to run `unitree-go2-route`, what each Route window control does, how clicks behave with add-waypoints on and off. Runnable example blocks, no em dashes, links that pass `doclinks`. Name the extra that provides pygame, how to reopen the Route window with `show_window()` from `dimos shell`, and that the window forces the X11 video driver (needs XWayland on a Wayland-only desktop).
- [x] T036 Regenerate the registry with `uv run pytest dimos/robot/test_all_blueprints_generation.py` so `dimos/robot/all_blueprints.py` includes `route_follower`, `route_control` and `unitree-go2-route` (never edit it by hand).
- [x] T037 [P] Run `uv run mypy dimos/`, `ruff format` and `ruff check`, and fix findings in the files this feature touched.
- [ ] T038 Run the automated checks and simulation steps S1 to S12 and the replay steps from `specs/001-multi-waypoint-route/quickstart.md`; fix any failures. Record the measured minimum speed between waypoints (SC-002) and the measured Stop-to-stationary time (SC-004) in the PR description.
- [ ] T039 Run hardware stages A to F from `specs/001-multi-waypoint-route/quickstart.md` on the Go2 PRO with a person at the manual emergency stop, and record stage, date and outcome in the PR description (constitution III). Tune `pass_radius_m` and `heartbeat_timeout_s` here if needed, and check on the robot that the held rule (3 s of window silence, finish the leg, stop at that waypoint) feels right; the rule itself is confirmed.
- [x] T040 Confirm the Constitution Check in `specs/001-multi-waypoint-route/plan.md` still passes, then open the PR to `main` after one batched push.

---

## Dependencies and Execution Order

### Phase dependencies

- **Phase 1 (Setup)**: none.
- **Phase 2 (Foundational)**: after Phase 1; blocks every user story. T008 (V1) comes before any later wiring work.
- **Phase 3 (US1)**: after Phase 2. MVP.
- **Phase 4 (US2)** and **Phase 5 (US3)**: after Phase 2; both extend files US1 touches (`route.py`, `module.py`, `control_window.py`, `control_layout.py`), so run them after US1 or coordinate edits.
- **Phase 6 (US4)**: after Phase 2; uses the same stop and goal signals as US3, so do it after US3.
- **Phase 7 (Polish)**: after the stories you ship.

### Within a story

Tests first and failing, then implementation, then the checkpoint.

### Parallel opportunities

- Phase 2: T003, T005, T009 and T011 touch different files and can start together. T004 follows T003, T006 follows T005, T010 follows T009, T012 follows T011. T007 and T008 are sequential and independent of the others.
- US1: T014, T015, T016 together. US2: T021 and T022. US3: T025, T026, T027. US4: T031 and T032.
- Polish: T035 and T037.
- Tasks that edit `module.py`, `route.py` or the window files are not marked [P] against each other.

---

## Implementation Strategy

### MVP first (User Story 1 only)

1. Phase 1 and Phase 2, with V1 resolved first.
2. Phase 3 (US1).
3. Stop and validate in simulation: quickstart S1, S2, S7, S10. Do not take this to hardware yet, because Stop and the interruption handling (US3) are required before any hardware test under the safety principle.

### Incremental delivery

1. Foundation, then US1 (demo in simulation).
2. US3 next, since it makes the feature safe for hardware, then US2 for editing comfort, then US4.
3. Polish, then the staged hardware checklist (T039) only after US1, US3 and US4 pass in simulation.

### Notes

- Commit in logical batches and push once, since each push starts about an hour of CI.
- Do not modify `dimos/navigation/go2/replanning_a_star/`, `dimos/navigation/movement_manager/`, `dimos/visualization/rerun/websocket_server.py` or the installed `dimos-viewer`: the design uses them only through their existing streams.
