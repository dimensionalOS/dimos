# DimOS Project Context

## Current direction

DimOS Studio is a local, beginner-oriented workbench for Go2 development. It is
an interface over the existing DimOS CLI, Blueprint, MCP, Agent, visualization,
and model configuration layers. It must not become a second robot runtime.

For manual operation, Studio launches DimOS's existing
`teleop-hosted-go2-transport` Blueprint and hands the operator to
<https://teleop.dimensionalos.com/>. The official page remains the cockpit for
video, map, joystick, emergency stop, and click-to-navigate.

The current development target is a separate local-LAN Mission Control mode.
Its first vertical slice now combines the existing camera/map visualization,
natural-language mission submission, a deterministic mission safety state
machine, evidence-based door policy skills, direct MCP navigation cancellation,
and remote supervision in one local page. The accepted design is
`docs/plans/2026-07-24-go2-local-mission-control-design.md`; the implementation
plan is `docs/plans/2026-07-24-go2-mission-control-implementation.md`.

Safety rule: real hardware movement is locked by default. Simulation is the
preferred place to test new skills. Hardware startup requires both disabling the
lock and entering the explicit confirmation phrase `START GO2`.
The separate `hardware_readonly` mode connects sensor streams while
`go2connection.movement_enabled=false` rejects movement at the robot module.

## Current implementation

- Local FastAPI workbench: `dimos/web/studio/`
- User-editable external Blueprint package: `extensions/go2-studio-agent/`
- Clickable macOS bundle: `apps/DimOS Studio.app`
- macOS launcher: `scripts/open_dimos_studio.command`
- Figma baseline: <https://www.figma.com/design/fpQYydt4G0EVFubTDAI1Uy>
- Default Studio URL: <http://127.0.0.1:8765>
- Existing DimOS map/command center URL: <http://127.0.0.1:7779/command-center>

The Studio currently supports Go2 signal-port diagnostics, DimOS
start/stop/status, MCP tool listing, Agent task submission, safe source editing
for custom `@skill` methods, and persisted model/runtime parameters.
It also provides a real-Go2 read-only connection that does not auto-stand and
blocks movement, posture, sport, joystick, and raw motion-topic commands.

Studio now also provides:

- LAN Go2 discovery through `dimos go2tool discover`
- secure Hosted Teleop key storage in macOS Keychain
- official Hosted Teleop launch with `go2connection.auto_stand=false`
- duplicate-start protection while DimOS is still registering
- cleanup of a launcher process that fails before registration
- unified Mission Control with the existing DimOS viewer embedded at port 7779
- persisted mission lifecycle and explicit create/start/pause/resume/stop/E-STOP
- a direct `end_exploration` plus `stop_navigation` MCP cancellation path
- autonomous navigation capped at 0.1 m/s with reverse navigation disabled in
  the external Studio Blueprint
- three-frame, confidence- and position-consistent door verification helpers
- one-metre stand-off goal calculation that remains advisory until navigation
  is explicitly allowed

## Known constraints

- The desktop wrapper currently opens a Chrome app window (or the default
  browser); native Tauri packaging is deferred because Rust is not installed.
- A reachable Go2 port is not proof of a complete WebRTC control link.
- Hosted Teleop is not connected until the broker accepts the API key and the
  official page visibly shows the robot, video, and current status.
- The Hosted Teleop key is intentionally absent from repository files and
  settings. A valid `dtk_...` key must exist in macOS Keychain before launch.
- `GO2Connection.start()` can make the real robot stand, so hardware start must
  remain an explicit action.
- Map and teleoperation views only become available when the selected DimOS
  Blueprint successfully starts its visualization stack.
- The Mission Control person-yield state machine is implemented, but live
  person distance is not yet fed into it from a detector. Do not claim live
  crowd yielding until that stream bridge is built and tested.
- Door evidence policy and Agent skills are implemented, but automatic
  camera-frame-to-door-observation ingestion is not yet built. The external VLM
  path still needs simulation/replay evaluation.
- A local E-STOP state is not physical proof. Mission Control directly requests
  both exploration and navigation cancellation through MCP and reports whether
  those calls succeeded; the operator must still verify the robot stopped.
- No real autonomous motion has been validated. Keep dense-venue movement
  locked until simulation, hardware-read-only, empty-area, and controlled
  pedestrian gates all pass.

## Recent task log

### 2026-07-24 — Go2 Mission Control first implementation slice

- Changed: added a deterministic mission/safety state machine, mission REST API,
  unified camera/map/task page, direct MCP stop path, evidence-based door Agent
  skills, and a navigation speed/reverse safety filter.
- Why: provide one beginner-usable place to see DimOS telemetry, describe a
  task, supervise its state, and stop it without replacing the existing DimOS
  mapping, exploration, planning, spatial memory, or MCP layers.
- Files: `dimos/web/studio/`,
  `dimos/navigation/movement_manager/movement_manager.py`,
  `extensions/go2-studio-agent/`,
  `docs/plans/2026-07-24-go2-mission-control-implementation.md`, and this
  document.
- Commands/tests: Studio mission/API pytest suite, MovementManager pytest suite,
  external door-policy and Blueprint smoke tests, Ruff, Node JavaScript syntax
  check, and a live local browser inspection on port 8766.
- Validation: mission transitions, lock/runtime start gates, person-distance
  pause policy, three-second resume policy, navigation failure cutoff, direct
  stop failure reporting, door evidence thresholds, one-metre stand-off math,
  0.1 m/s navigation clipping, reverse blocking, and required Blueprint modules
  are covered by tests. The rendered page had no JavaScript console errors and
  showed an explicit offline state while the port-7779 viewer was unavailable.
- Safety boundary: no real robot movement or autonomous hardware command was
  issued during development. The direct speed limiter applies to autonomous
  navigation output; risky Unitree sport skills and person-follow are disabled
  in the Studio Blueprint.
- Unresolved: live person detection is not connected to the state machine,
  camera frames are not automatically recorded as door evidence, Agent progress
  is not yet streamed back into mission phases, and simulation/replay plus all
  real-world gates remain unvalidated.
- Next: connect perception telemetry to mission safety/evidence, run a recorded
  or simulated door-search mission, then validate real hardware read-only before
  considering empty-area movement.

### 2026-07-24 — Local autonomous Mission Control design

- Changed: accepted and documented the architecture for local autonomous
  exploration, semantic door finding, safe approach, crowd yielding, unified
  visualization, and remote E-STOP.
- Why: the user wants the Go2 to understand a natural-language task, explore
  without manually marked points, find a door, navigate to about one metre in
  front of it, and report from a cluttered indoor venue.
- Files: `docs/plans/2026-07-24-go2-local-mission-control-design.md` and this
  document.
- Validation: design reviewed against current mapping, frontier exploration,
  spatial memory, navigation, VLM, Agent/MCP, and stop-control source modules.
- Safety boundary: current hardware only; no person follows the robot, but a
  remote supervisor and E-STOP are mandatory. Dense-crowd movement stays locked
  until staged simulation and controlled real-world gates pass.
- Resolved later: the implementation plan, local telemetry/UI slice, mission
  state machine, and door policy skills now exist.
- Remaining: external VLM integration and final crowd-mode acceptance metrics
  still require staged validation before enabling real movement.

### 2026-07-24 — Restore official Hosted Teleop

- Changed: added Go2 auto-discovery, macOS Keychain-backed Teleop credentials,
  the official Hosted Teleop launcher, explicit `START TELEOP` confirmation,
  duplicate-start protection, and direct access to the official cockpit.
- Why: restore the proven control path used on 2026-07-23 instead of presenting
  the custom Agent command center as a replacement for official teleoperation.
- Files: `dimos/web/studio/`,
  `docs/plans/2026-07-24-hosted-teleop-recovery-design.md`, and this document.
- Commands/tests: Studio and Go2 connection unit tests, Ruff, JavaScript syntax
  check, macOS bundle plist check, live Studio health/API checks, live LAN
  discovery, and a non-motion TCP signal-port check.
- Validation: 14 tests passed; Ruff and JavaScript checks passed; the live Go2
  was discovered at `30.201.217.128` with serial `B42D1000PC4C1M86`; port 9991
  responded in 32.7 ms; Studio shows the new official Teleop controls.
- Unresolved: no Hosted Teleop `dtk_...` key is currently stored, so broker,
  video, and operator-control validation cannot start yet. The official site
  also timed out under automated inspection and must be checked interactively.
- Next: save a valid Teleop key in Studio, enter `START TELEOP`, start the
  transport, then verify broker heartbeat, online robot, video, and telemetry
  before any movement command.

### 2026-07-24 — Functional DimOS Studio baseline

- Changed: added the local Studio API/UI, a macOS launcher, and an editable
  external Go2 Agent Blueprint with starter skills.
- Why: provide an app-like, beginner-safe way to connect, configure, extend,
  and operate DimOS without replacing its runtime.
- Files: `dimos/web/studio/`, `extensions/go2-studio-agent/`,
  `apps/DimOS Studio.app`, `scripts/open_dimos_studio.command`, and this document.
- Validation: 5 Studio unit/API tests passed; Ruff passed; the macOS bundle
  plist passed; the editable package installed; `dimos-go2-studio.go2` loaded
  successfully with 20 modules; the local health page responded successfully.
- Resolved later: Go2 is reachable at `30.201.217.128:9991`.
- Remaining: native Tauri packaging, full custom Agent simulation, and parameter
  schema discovery remain follow-up work.
