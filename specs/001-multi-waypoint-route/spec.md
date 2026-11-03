# Feature Specification: Multi-Waypoint Route on the Map

**Feature Branch**: `001-multi-waypoint-route`

**Created**: 2026-11-02

**Status**: Draft (revised 2026-11-02: target changed from the browser cockpit to the standalone viewer application)

**Input**: User description: "In the standalone UI, in the map where one can provide a waypoint to the connected robot (a Go2 PRO), I want to be able to define multiple waypoints that the robot shall follow, one after one, until it reaches the final waypoint. The start of the journey, once the several waypoints are defined, shall be triggered by the click of a button (a button that needs to be added)."

## Clarifications

### Session 2026-11-02

- Q: When the robot reaches an intermediate waypoint, does it stop there or drive straight through? → A: Drive straight through; only the final waypoint is a full stop.
- Q: If the operator marks a point while a route is running, is it added to the route or refused? → A: Refused. While a route runs, every map click is ignored and the operator is told to stop the route first.
- Q: How does the operator switch between single-point travel and building a route? → A: An explicit "add waypoints" switch, off by default. Off: a map click sends the robot to that point as today. On: a map click only adds a waypoint.
- Q: Which application is the "standalone UI"? → A: The native desktop viewer that `dimos run` opens (`dimos-viewer`), not the browser cockpit. Its compiled code cannot change from this repository, so the route controls (the add-waypoints switch and the Start, Stop and Clear buttons) live in a small companion "Route window" that opens next to it. Waypoints are placed by clicking the viewer's map and are drawn on that map.
- Q: May the controls live in a separate window instead of inside the viewer's map window? → A: Yes.
- Q: When the Route window stops responding during a route, what does the robot do, and after how long? → A: After 3 seconds of silence the route becomes held: the robot finishes the leg it is on, stops at that waypoint, and does not start the next leg. Start resumes from the first waypoint not yet reached.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Plan a route of several waypoints and start it (Priority: P1)

An operator looks at the map in the standalone viewer. They switch "add waypoints" on in the Route window and click several points on the map, one after another. Numbered markers appear on the map in order. Nothing moves yet. When the operator is happy with the route, they press Start in the Route window and the robot drives to the first waypoint, then the second, and so on until the final waypoint.

**Why this priority**: This is the whole feature. A route that can be planned and then run is the smallest useful slice.

**Independent Test**: Switch add-waypoints on, click three points on the map, confirm the robot has not moved, press Start, and confirm the robot visits the waypoints in the order placed and stops at the last.

**Acceptance Scenarios**:

1. **Given** the viewer shows the map with a connected robot and no route, **When** the operator turns add-waypoints on and clicks three points, **Then** three numbered waypoints appear on the map in the order clicked and the robot does not move.
2. **Given** a route of three waypoints, **When** the operator presses Start, **Then** the robot goes to waypoint 1, then waypoint 2, then waypoint 3, passing through waypoints 1 and 2 without stopping, and comes to a full stop at waypoint 3.
3. **Given** a route is running, **When** the robot reaches a waypoint, **Then** that waypoint is shown as reached and the next waypoint is shown as the current target, both on the map and in the Route window.
4. **Given** a route of one waypoint, **When** the operator presses Start, **Then** the robot goes to that waypoint and stops.
5. **Given** add-waypoints is off, **When** the operator clicks the map, **Then** the robot goes to that single point exactly as it does today and no waypoint is added.

---

### User Story 2 - Edit the route before starting (Priority: P2)

Before pressing Start, the operator can fix mistakes: remove a single waypoint, clear the whole route, or add more points to the end.

**Why this priority**: Without editing, one misplaced click forces a restart. It makes the P1 flow practical but is not needed to prove it.

**Independent Test**: Place four waypoints, remove the second, confirm the remaining three are renumbered 1 to 3 on the map and in the Route window, clear the route, and confirm no waypoints remain.

**Acceptance Scenarios**:

1. **Given** a route of four waypoints that is not started, **When** the operator removes waypoint 2 in the Route window, **Then** the other three remain in order and are renumbered 1 to 3 on the map.
2. **Given** a route that is not started, **When** the operator presses Clear, **Then** all waypoints disappear from the map and Start is unavailable.
3. **Given** no waypoints are placed, **When** the operator looks at the Start button, **Then** it is disabled.

---

### User Story 3 - Stop a running route and see progress (Priority: P2)

While the robot drives the route, the operator sees which waypoint is the current target and how many remain, and can stop at any time. Stopping halts the robot and ends the route.

**Why this priority**: A physical robot must always be stoppable, and operators need to know where the robot is in the route. It is required before use on real hardware.

**Independent Test**: Start a three-waypoint route, press Stop while the robot is heading to waypoint 2, and confirm the robot stops, no further waypoint is pursued, and the Route window shows the route as stopped.

**Acceptance Scenarios**:

1. **Given** a route is running, **When** the operator presses Stop, **Then** the robot stops moving and the remaining waypoints are not pursued.
2. **Given** a route is running, **When** the operator looks at the map, **Then** the current target is highlighted, reached waypoints are marked as reached, and waypoints still to visit are marked as pending.
3. **Given** a route was stopped partway, **When** the operator looks at the map and the Route window, **Then** they can see which waypoints were reached and which were not, and can edit or start a new route.
4. **Given** a route is running, **When** the Route window stops responding for 3 seconds (a close request while a route runs is ignored), **Then** the route shows as held: the robot finishes the leg it is on, stops at that waypoint, and does not start the next leg. Reopening the window (`show_window()` from `dimos shell`) and pressing Start resumes from the first waypoint not yet reached.

---

### User Story 4 - Handle a waypoint the robot cannot reach (Priority: P3)

If the robot cannot reach a waypoint (no path, blocked, or gives up), the route does not silently skip ahead. The robot stops, and the operator is told which waypoint failed.

**Why this priority**: Predictable failure behavior matters on hardware, but the happy path comes first.

**Independent Test**: Start a route whose second waypoint is inside an obstacle, and confirm the robot stops after waypoint 1, the Route window says waypoint 2 could not be reached, and waypoint 3 is not attempted.

**Acceptance Scenarios**:

1. **Given** a running route whose next waypoint cannot be reached, **When** the planner reports failure, **Then** the robot stops, the failed waypoint is marked as failed on the map and in the Route window, and later waypoints are not attempted.

---

### Edge Cases

- Operator clicks the map while a route is running, with add-waypoints on or off: the click is ignored, nothing is added and the robot is not redirected, and the Route window tells the operator to stop the route first. (Appending to a running route is out of scope.)
- Operator presses Start while a route is already running: ignored, no restart.
- Another source commands the robot during a route (the operator presses a teleop key, an agent or another module sets a new goal): the route ends as interrupted and the Route window shows it.
- The Route window stops responding mid-route (for example it hangs or its process dies): the route becomes held as described in User Story 3. Add-waypoints also switches back off, so later map clicks cannot be silently swallowed.
- The operator tries to close the Route window while a route runs: the close is ignored and the window says to stop the route first. Closing it when no route runs hides it and switches add-waypoints off; `show_window()` brings it back.
- A very large number of waypoints (more than the supported maximum) is placed: the extra click is refused and the Route window says why.
- Two waypoints placed at nearly the same spot: both are kept; the second is reached immediately if the robot is already within the arrival distance.
- The robot is already within arrival distance of a waypoint when its turn comes: it counts as reached without moving.
- The viewer is not open or not connected: no clicks arrive, so no waypoints can be added. Route state, Start and Stop still work from the Route window.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: With add-waypoints on, the operator MUST be able to add a waypoint to the route by clicking the map in the standalone viewer.
- **FR-002**: Adding, removing or reordering waypoints MUST NOT cause the robot to move. Motion MUST begin only when the operator presses the Start button.
- **FR-003**: The viewer's map MUST show every planned waypoint with a visible order number and a route line joining them in order.
- **FR-004**: The Route window MUST provide a Start button that is disabled when the route has no waypoints, when no robot position is being received, and while a route is already running.
- **FR-005**: When Start is pressed, the robot MUST visit the waypoints strictly in the order they were placed. It MUST NOT stop at intermediate waypoints: on reaching one it continues directly toward the next, and that waypoint is marked reached.
- **FR-006**: The robot MUST come to a full stop at the final waypoint, and the Route window MUST show the route as completed.
- **FR-007**: The operator MUST be able to remove an individual waypoint and clear the whole route while no route is running.
- **FR-008**: Waypoints MUST be renumbered after a removal so the numbering is always 1 to N with no gaps.
- **FR-009**: While a route runs, each waypoint's state (pending, current target, reached, failed) MUST be visible on the map, through marker colour, and in the Route window.
- **FR-010**: The operator MUST be able to stop a running route at any time, and stopping MUST halt the robot and leave the unvisited waypoints unvisited.
- **FR-011**: If a waypoint cannot be reached, the system MUST stop the robot, mark that waypoint as failed, tell the operator which waypoint failed, and not attempt later waypoints.
- **FR-012**: If another source takes control of the robot during a route (teleop input, an agent, or any other module setting a goal), the route MUST end as interrupted and the Route window MUST show it.
- **FR-013**: The route shown on every open viewer map and in the Route window MUST be the same route, with the same progress.
- **FR-014**: Pressing Start twice MUST be ignored. While a route is running, every click on the map MUST be ignored whether add-waypoints is on or off, with a message in the Route window telling the operator to stop the route first. Neither MUST alter the running route.
- **FR-015**: The system MUST refuse waypoints beyond a maximum route length and say why in the Route window.
- **FR-016**: The Route window MUST provide an add-waypoints switch that is off by default and shows its state clearly. With it off, a map click sends the robot to that single point exactly as it does today. With it on, a map click only adds a waypoint and never moves the robot.
- **FR-017**: If the Route window sends no signal for 3 seconds while a route is running, the route MUST become held: the robot finishes its current leg and stops at that waypoint without starting the next. Start MUST resume a held route from the first waypoint not yet reached. Add-waypoints MUST switch off when the Route window goes silent.
- **FR-018**: The Route window MUST stay in sync with the route: it MUST show the add-waypoints state, the status (planning, running, completed, stopped, interrupted, failed, held), the waypoint list with states, and the latest message.

### Key Entities

- **Waypoint**: One planned stop on the map. Has a position on the map, an order number within the route, and a state (pending, current target, reached, failed).
- **Route**: The ordered list of waypoints plus a status (planning, running, completed, stopped, interrupted, failed, held) and an add-waypoints setting. A route belongs to the current session and one robot.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: An operator can plan a five-waypoint route and start it in under 30 seconds.
- **SC-002**: In test runs on the simulated Go2, the robot visits 100% of reachable waypoints in the order placed, with no skipped or reordered waypoint and no stop at any intermediate waypoint, meaning the robot's speed stays above 0.1 m/s between entering the pass radius of one waypoint and reaching the next.
- **SC-003**: Placing, editing and clearing waypoints produces zero robot movement across all test runs.
- **SC-004**: After the operator presses Stop, the robot is stationary within 2 seconds in 100% of test runs and no further waypoint is pursued.
- **SC-005**: When a waypoint is unreachable, the operator sees which waypoint failed within 5 seconds of the robot giving up, and the robot does not continue to later waypoints.
- **SC-006**: A first-time operator can run a three-waypoint route without instructions in under 2 minutes.
- **SC-007**: Before any hardware use, the full flow (plan, start, stop, unreachable waypoint, interruption, held) passes in simulation or replay.
- **SC-008**: With add-waypoints off, single-click navigation behaves identically to today in 100% of test runs.

## Assumptions

- The "standalone UI" is the native desktop viewer that `dimos run` opens by default (`dimos-viewer`). The browser cockpit and the web dashboard are not targets of this feature.
- The viewer's own code is not changed. It keeps sending map clicks to the robot software as it does today. Everything else, the add-waypoints switch, Start, Stop, Clear, the waypoint list and status, lives in a separate Route window that opens when the robot software starts and can be reopened with `show_window()` after it is hidden. A later release of the viewer could embed these controls, but that is out of scope.
- The Route window is a desktop window on the same machine that runs the robot software.
- The target robot is a Go2 PRO driven by the existing Go2 navigation stack; other robots are out of scope.
- The existing planner is reused to travel to each waypoint; this feature chooses the order and when to advance, not how the robot gets there.
- A waypoint counts as reached when the robot is within a pass radius of 0.6 m of it. The final waypoint ends with the planner's normal arrival and full stop.
- The route is held in the session, not saved across restarts. Saving and loading named routes is out of scope. Looping or repeating routes, pausing and resuming mid-leg, inserting waypoints into a running route, and setting a heading or speed per waypoint are out of scope.
- The maximum route length is 20 waypoints.
- Only one source drives the robot at a time, consistent with the project's robot-safety rules. Hardware testing follows a staged checklist with a manual emergency stop within reach, after simulation or replay passes.
- Operators are trained robot operators on a trusted machine; no new sign-in or permission levels are introduced.
