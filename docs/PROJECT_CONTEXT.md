# DimOS Project Context

## Current direction

DimOS Studio is a local, beginner-oriented workbench for Go2 development. It is
an interface over the existing DimOS CLI, Blueprint, MCP, Agent, visualization,
and model configuration layers. It must not become a second robot runtime.

For manual operation, Studio launches DimOS's existing
`teleop-hosted-go2-transport` Blueprint and hands the operator to
<https://teleop.dimensionalos.com/>. The official page remains the cockpit for
video, map, joystick, emergency stop, and click-to-navigate.

The next development target is a separate local-LAN Mission Control mode. It
will combine camera, LiDAR mapping, frontier exploration, semantic door
recognition, Agent/MCP task execution, conservative person-yield behavior, and
remote supervision in one local page. The accepted design is
`docs/plans/2026-07-24-go2-local-mission-control-design.md`.

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

## Recent task log

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
- Unresolved: implementation has not started; external VLM provider and final
  crowd-mode acceptance metrics still require validation.
- Next: create the implementation plan, build the local telemetry/UI slice, and
  prove perception and safety behavior before enabling real movement.

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
