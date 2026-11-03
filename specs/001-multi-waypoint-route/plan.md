# Implementation Plan: Multi-Waypoint Route on the Map

**Branch**: `001-multi-waypoint-route` (spec id; the git branch must follow `<who>/<type>/<topic>`, for example `david/feat/multi-waypoint-route`) | **Date**: 2026-11-02 | **Spec**: [spec.md](spec.md)

**Input**: Feature specification from `/specs/001-multi-waypoint-route/spec.md`

**Revision note**: The first plan targeted the browser cockpit. The target is the native `dimos-viewer` application, so this plan replaces it.

## Summary

Operators place several waypoints by clicking the map in the native `dimos-viewer` and start the journey from a button in a small companion Route window. The viewer's code is a prebuilt binary outside this repo, so the work splits into two new modules plus a blueprint:

- `RouteFollower` sits between the viewer's websocket click output and the rest of the navigation stack. It decides per click whether it is a single-point goal (forwarded as today), a new waypoint (kept, nothing forwarded) or ignored (a route is running). It owns the route, feeds the existing planner one goal at a time, makes intermediate waypoints pass-through, detects completion, failure, stop and outside interruption, and publishes the route as state, a line and map markers that the Rerun bridge draws in the viewer.
- `RouteControl` is a pygame window (add-waypoints switch, Start, Stop, Clear, waypoint list, status) that talks to the follower over two JSON string streams and sends a 1 Hz keepalive.
- A new blueprint `unitree-go2-route` composes the existing Go2 smart stack with both modules and the stream remappings.

## Technical Context

**Language/Version**: Python 3.12.

**Primary Dependencies**: Existing DimOS `Module` and stream system, `ReplanningAStarPlanner`, `MovementManager`, `RerunBridgeModule` and `RerunWebSocketServer` (`dimos/visualization/rerun`), `pygame` (already used by `KeyboardTeleopModule` and declared in the `control`, `sim` and `tests` extras; optional import with an error that names the extra to install). No new third-party packages.

**Storage**: In-memory only, owned by `RouteFollower`. Not persisted across restarts.

**Testing**: `uv run pytest` with tests next to the code. Pure-logic tests for the route state machine and the window layout; module tests with fake planner streams; a blueprint-level test for the remappings; simulation then replay then a staged hardware checklist. `uv run mypy dimos/` and ruff.

**Target Platform**: Linux desktop running the DimOS runtime with the native `dimos-viewer` and the Go2 PRO connection (or `--simulation`/`--replay`).

**Project Type**: Robot runtime modules plus a desktop control window. No web code and no change to `dimos-viewer` or the cockpit.

**Performance Goals**: Route state reaches the window and the map within 1 s of a change. The next goal is issued within one odometry update of crossing the pass radius, so there is no pause at intermediate waypoints. Stop yields a stationary robot within 2 s (SC-004) by reusing `stop_movement`.

**Constraints**: Edits never move the robot; only Start does. All changes additive: blueprints without the new modules are unchanged. At most 20 waypoints. Behaviour stays correct if the window disappears (heartbeat watchdog in the follower). The viewer binary stays untouched.

**Scale/Scope**: One robot, one active route, up to 20 waypoints, one Route window.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | How the plan satisfies it |
|-----------|--------|---------------------------|
| I. Module Architecture | Pass, one item to verify | Both modules communicate only through typed `In`/`Out` streams. No module reaches into another. Ports are renamed in the blueprint with `.remappings()`, not by sharing instances. The remapping approach (V1 in research.md) is verified first; the fallback uses a `Spec` Protocol and RPC. Blueprint uses `autoconnect()`, named functions, no import-time work. |
| II. Agent Skills | Not touched | No new `@skill`. |
| III. Robot Safety (NON-NEGOTIABLE) | Pass, with hardware gate | Adding waypoints never moves the robot, only Start does. Every map click is intercepted, so a click cannot redirect a running route. Stop reuses the shared `stop_movement` stream. Teleop, agent, patrol or exploration commands end the route (`interrupted`). Window loss holds the robot after the current leg. Single driver: the follower detects any other goal or stop and yields. Simulation and replay first; the quickstart holds the staged hardware checklist. |
| IV. Code Quality | Pass | Full type annotations, mypy strict, license headers, project logger with key/values, no hardcoded ports (the viewer websocket keeps its existing owner). Constructors do no work; `start()`/`stop()` paired and call `super()`; all subscriptions registered with `register_disposable`; threads tracked and joined; stop flags are `threading.Event`s; shared state guarded by a lock on reads and writes. |
| V. Testing | Pass | Tests next to code, real assertions, no fixed sleeps, fixtures for cleanup. The window tests use a headless SDL driver and fail (not skip) if pygame is missing. |
| VI. Generated Artifacts | Action required | Run `pytest dimos/robot/test_all_blueprints_generation.py` after adding the modules and the `unitree-go2-route` blueprint. |
| VII. Git Workflow | Action required | Rename the working branch to `<who>/feat/multi-waypoint-route` before committing; PR targets `main`. |
| VIII. Compatibility | Pass | New modules, new streams and a new blueprint only. No existing stream, message type, skill or blueprint changes. |
| IX. Documentation | Action required | Document the route feature under `docs/` with a runnable example; pass `doclinks` and `emdash-check`. |

No violations, so Complexity Tracking stays empty. Re-check after Phase 1 design: unchanged.

## Project Structure

### Documentation (this feature)

```text
specs/001-multi-waypoint-route/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/
│   └── route-streams.md   # Phase 1 output
├── checklists/
│   └── requirements.md
└── tasks.md             # Phase 2 output (/speckit-tasks, not created here)
```

### Source Code (repository root)

```text
dimos/
├── navigation/
│   └── route_follower/
│       ├── __init__.py
│       ├── route.py                 # Route and Waypoint state machine, pure logic (new)
│       ├── messages.py              # command and state documents: parse, validate, build (new)
│       ├── module.py                # RouteFollower module (new)
│       ├── control_layout.py        # window layout and hit-testing, pure logic (new)
│       ├── control_window.py        # RouteControl module, pygame window (new)
│       ├── overlay.py               # route_state -> Rerun points for the bridge override (new)
│       ├── test_route.py
│       ├── test_messages.py
│       ├── test_module.py
│       ├── test_control_layout.py
│       ├── test_control_window.py
│       └── test_overlay.py
├── robot/unitree/go2/blueprints/smart/
│   ├── unitree_go2_route.py         # unitree-go2-route blueprint with remappings and overlay (new)
│   └── test_unitree_go2_route.py    # blueprint wiring tests (new)
└── robot/all_blueprints.py          # regenerated, never edited by hand

docs/capabilities/navigation/        # route feature page with a runnable example (new)
```

**Structure Decision**: One new Python package next to the other navigation modules, with pure logic (state machine, messages, layout, overlay) split from the two modules so most behaviour is testable without streams or a display. The only edits to existing files are generated or documentation. `dimos-viewer`, `RerunWebSocketServer`, the planner and `MovementManager` are not modified.

## Complexity Tracking

No constitution violations to justify.
