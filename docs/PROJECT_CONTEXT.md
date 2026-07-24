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
Candidate-only external visual verification is now also accepted in
`docs/plans/2026-07-25-go2-candidate-vision-verification-design.md`. It keeps
mapping, LiDAR geometry, navigation, safety, and motion local while sending only
up to three explicitly selected candidate screenshots to an OpenAI vision
provider. Its first offline implementation is available through
`python -m dimos.perception.offline_target_verification`.

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

Live state on 2026-07-25: the Mac reached the Go2 at `192.168.12.1` while
internet remained available through the phone. Mission
`8c1a12389dab462db2d5d2651b992742` stopped after battery reached 4%, below its
5% floor, and the operator then explicitly stopped it. The global E-STOP
validation subsequently closed real-hardware run
`20260725-012510-dimos-go2-studio-go2`; no robot runtime is currently active.
Exactly one Studio service remains available at `127.0.0.1:8765`, and it was
started with `--no-open` so it does not create duplicate browser tabs.

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
- a sticky global control bar on every Studio page: the green button routes a
  beginner to robot connection or mission startup, while the red
  `关闭控制 / 立即停止` button remains available even without an active mission
- latched global E-STOP behavior: Agent cancellation, exploration, patrol,
  navigation, and lookout stops are issued in parallel while the DimOS robot
  runtime is shut down on a separate parallel path
- direct `end_exploration`, `stop_patrol`, `stop_navigation`, and
  `stop_looking_out` cleanup on every task exit
- autonomous navigation capped at 0.2 m/s with reverse navigation disabled in
  the external Studio Blueprint
- independent 30-minute watchdog and five-second battery monitoring
- fresh-frame, three-view, position-consistent semantic verification plus
  contrastive rejection of glass walls, whiteboards, chairs, and people
- an offline candidate-only vision verifier that loads one to three named
  persisted frames, defaults to a no-upload dry run, and requires an explicit
  `--send` flag before constructing the OpenAI client
- automatic transition from exhausted frontier exploration to coverage patrol
- no automatic Rerun browser-tab launch; the existing Studio page embeds the
  DimOS viewer, avoiding duplicate pages and excess browser memory
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
- DimOS CLIP spatial-memory ingestion is now wired into the deterministic
  mission runner and has passed one bounded real-Go2 candidate-search test.
  CLIP provides a semantic camera-viewpoint candidate, not a door bounding box
  or metric door pose. A second-stage geometric/visual verifier is still
  required before any one-metre approach.
- A software E-STOP is not the Go2's physical power switch. The global red
  control now sends all five MCP cancellation calls and closes the robot
  runtime, reporting the two confirmations separately. If the computer,
  network, browser, or robot firmware is unresponsive, the operator must still
  use the Go2's physical stop/power control and physically verify it stopped.
- Supervised venue motion has been validated at the configured 0.2 m/s cap with
  the lidar cost map and replanning stack active. Camera-only person distance is
  still not bridged into Mission Control, so the claim is obstacle replanning,
  not independently verified person classification or social navigation.
- The candidate-only verifier has passed offline validation against the saved
  578-frame real-Go2 memory. This shell does not currently have
  `OPENAI_API_KEY`, so no cloud model verdict has been obtained. The verifier is
  not yet connected to the live mission runner, camera/LiDAR projection, or
  movement authorization.

## Recent task log

### 2026-07-25 — Offline candidate-only vision verifier

- Changed: added immutable candidate evidence/result contracts, selective
  persisted-frame loading and JPEG resizing, an OpenAI Responses structured
  output adapter, fail-closed timeout/retry behavior, a two-confirmed-view
  acceptance policy, and a dry-run-first CLI.
- Why: validate the semantic `AI check` path using saved real-Go2 frames before
  reconnecting the robot or permitting any new motion.
- Files: `dimos/perception/target_verification.py`,
  `dimos/perception/offline_target_verification.py`, their two test modules,
  `docs/plans/2026-07-25-go2-offline-vision-verification-implementation.md`,
  and this document.
- Commands/tests: ran both focused pytest modules (15 passed), Ruff on all four
  new Python modules (passed), and the offline CLI against
  `assets/output/memory/spatial_memory/visual_memory.pkl`.
- Validation: the real memory loaded 578 frames; only
  `frame_20260725_012917_f56dd2cc` and
  `frame_20260725_013012_fa0912d3` were selected and resized to 1024 by 576.
  The result reported `mode=dry_run` and `upload_authorized=false`; image bytes
  were not printed or persisted in the report. An explicit `--send` attempt
  with the same two frames failed closed as `uncertain` with
  `OPENAI_API_KEY is not configured`; no external request was made.
- Safety boundary: no robot service, browser page, external API request, or
  movement command was started. A model `yes` is downgraded unless two distinct
  selected views both contain positive bounding-box evidence.
- Unresolved: `OPENAI_API_KEY` is absent from this shell, so the actual OpenAI
  response remains unvalidated. Keychain-backed credentials, live mission
  wiring, camera-to-LiDAR/map projection, and controlled motion stay outside
  this offline phase.
- Next: configure the API key without committing it, run one explicit
  candidate-only `--send` check, inspect the structured evidence, then build
  the local geometry gate before any approach behavior.

### 2026-07-25 — Candidate-only OpenAI visual verification design

- Changed: accepted and documented a provider-neutral `VisionVerifier` that
  sends at most three candidate screenshots to the OpenAI API, receives a
  structured `yes/no/uncertain` verdict plus normalized image boxes, and keeps
  target projection, LiDAR passage checks, navigation, and E-STOP local.
- Why: the real run proved that CLIP can rank map-linked camera viewpoints but
  cannot reliably identify a door or produce the door's metric map position.
  The user selected candidate-only cloud upload rather than a fully local model.
- Files: `docs/plans/2026-07-25-go2-candidate-vision-verification-design.md`
  and this document.
- Validation: reviewed against the current `VisualMemory`, CLIP search skill,
  OpenAI VLM adapter, mission runner, and bbox navigation code. The last run's
  persisted visual memory contains 578 real frames and both known candidate
  frame IDs, so the first verifier acceptance can be offline.
- Safety boundary: no robot process, API request, credential change, or motion
  command was issued. A cloud verdict is semantic evidence only and cannot
  authorize movement without local multi-view and LiDAR geometry checks.
- Unresolved: implementation has not started; OpenAI Responses API support,
  Keychain-backed credential storage, candidate-frame retrieval, structured
  result validation, and camera/LiDAR map projection still need to be built.
- Next: create the implementation plan and complete the offline 578-frame
  verification slice before reconnecting the Go2.

### 2026-07-25 — Global close/start controls and latched software E-STOP

- Changed: added a sticky safety bar visible on every Studio page. Its green
  button routes to the robot connection controls when DimOS is off and to the
  Mission Control startup confirmation when DimOS is running. Its red
  `关闭控制 / 立即停止` button is never disabled merely because no mission exists.
  The E-STOP backend now cancels Agent work, exploration, patrol, navigation,
  and lookout concurrently and closes the DimOS robot runtime concurrently.
- Why: the old start/stop controls were hidden inside the Mission Control tab,
  the E-STOP button was disabled in the idle state, and sequential MCP calls
  could take roughly 25 seconds in the worst case. The user needs an obvious,
  global, fail-closed way to regain control.
- Files: `dimos/web/studio/service.py`,
  `dimos/web/studio/static/index.html`,
  `dimos/web/studio/static/styles.css`,
  `dimos/web/studio/static/app.js`,
  `dimos/web/studio/test_studio.py`, and this document.
- Commands/tests: Studio mission/API/runner suite passed 34 tests; Ruff passed;
  Node JavaScript syntax checking passed. A barrier-based test verifies that
  all five MCP cancellations are actually dispatched in parallel.
- Live validation: the active MCP server failed to acknowledge any stop tool
  within its timeout, so the E-STOP correctly took the runtime-shutdown path.
  Run `20260725-012510-dimos-go2-studio-go2` was stopped and its PID disappeared;
  Studio now reports `DimOS 当前未运行`. The run required SIGKILL after the normal
  SIGTERM timeout, so this proves runtime closure, not a physical power-off.
- Unresolved: automated visual refresh of the existing in-app browser tab was
  blocked by the browser's localhost policy. The served HTML contains both
  global controls and the static/API checks pass; the operator must press
  `Command + R` once in the existing Studio tab to load the new UI.
- Next: recharge the Go2, refresh Studio, use the green control to reconnect,
  and keep the red control in view during the next supervised test. Do not
  restart the door-search mission until target localization replaces blind
  coverage patrol.

### 2026-07-25 — Continuous real-Go2 exploration and coverage patrol

- Changed: extended the deterministic mission to 1,800 seconds and 20 m,
  increased the navigation cap to 0.2 m/s, added fresh-frame and contrastive
  semantic filtering, added five-second battery checks, and introduced a
  20-second frontier-to-coverage-patrol handoff. Cleanup now stops patrol as
  well as exploration, navigation, and lookout. Lowered the operator-requested
  mission battery floor from 15% to 5%, while retaining a nonzero collapse
  margin. Removed `--rerun-open web` so future starts reuse the embedded viewer
  instead of opening another browser page.
- Why: the earlier bounded explorer could remain logically running after all
  frontiers were exhausted while the physical Go2 was stationary. The user
  requires continuous venue search, visible mapping, and no duplicate local
  pages that consume browser memory.
- Files: `dimos/web/studio/mission.py`, `mission_runner.py`, `service.py`, their
  focused tests, the external Go2 Blueprint/policy files, and this document.
- Commands/tests: focused Studio mission, runner, and API suite passed 31 tests
  after all battery and browser-tab adjustments; Ruff and `git diff --check`
  passed while the physical mission remained active.
- Live validation: during this historical run, one Studio listener on
  `127.0.0.1:8765` and one Go2 runtime were confirmed. The real Go2 moved from
  about `(-2.54, 1.84)` to
  `(-1.70, 1.09)` during frontier exploration, published a target at
  `(0.94, 4.39)`, then cleanly called `end_exploration` and `start_patrol`.
  During coverage patrol it reached about `(-0.86, 0.23)`, selected a new goal
  near `(-1.53, 2.58)`, continued moving, and rejected a glass-wall/whiteboard
  false semantic target instead of stopping. Battery later reached 4%, the
  mission failed closed below its 5% floor, and the operator stopped it.
- Unresolved: no real door has yet been verified, CLIP still yields camera
  viewpoints rather than metric door geometry, and the final one-metre approach
  is not implemented. The local Moondream weights are not cached. Continuous
  operation is limited by the currently low battery and will stop at 5%.
- Next: do not resume blind coverage patrol. Expose the selected camera frame,
  verify a door bounding region, project it through depth/LiDAR, and only then
  authorize goal-directed motion. Recharge before another acceptance run.

### 2026-07-24 — Real-Go2 DimOS semantic candidate search

- Changed: replaced the first-stage mission dependency on the stalled local
  Moondream lookout with a deterministic runner over DimOS's existing CLIP
  spatial memory; added the `search_semantic_memory` MCP skill, three-frame
  spatial consistency, a 5 cm independent-view requirement, stop-before-report
  ordering, and fail-closed CLIP inference. On macOS the fused CLIP ONNX session
  now uses `CPUExecutionProvider` because CoreML failed after alternating image
  and text inference and the old error path returned random embeddings.
- Why: keep perception on the existing DimOS camera/map stack, remove the local
  `ollama:qwen3:4b` Agent from physical mission timing, and test fixes against
  the real Go2 instead of treating network/service health as acceptance.
- Files: `dimos/perception/image_embedding.py`,
  `dimos/perception/test_image_embedding.py`,
  `dimos/web/studio/mission_runner.py`,
  `dimos/web/studio/test_mission_runner.py`,
  `extensions/go2-studio-agent/src/dimos_go2_studio/skills.py`,
  `extensions/go2-studio-agent/tests/test_skills.py`, and this document.
- Commands/tests: ran focused mission-runner, extension-skill, Blueprint,
  mission-policy, CLIP fail-closed and macOS-provider tests; ran Ruff and
  `git diff --check` and JavaScript syntax checking. The final affected suite
  passed 62 tests with one warning; started run
  `20260725-000903-dimos-go2-studio-go2` in real hardware mode after the
  operator's `START GO2`/clear-area authorization; queried battery, executed a
  15-second maximum semantic mission, called exploration/navigation stop again,
  and inspected the live 2D map, camera and Rerun 3D point cloud.
- Validation: battery was 46% before motion and 45% afterward. Repeated queries
  for the same frame became deterministic at `0.2319`. The Go2 published a
  frontier goal and physically explored while 13 map-linked camera frames were
  stored. Three viewpoints separated by at least 5 cm produced a doorway
  candidate (`best_similarity=0.2533`); the runner stopped before reporting it.
  `end_exploration` returned after about 2.0 seconds, `stop_navigation` returned
  `Stopped`, the final visible pose was about `(-0.05, 0.18)`, and no later
  frontier goal was published. The live camera visibly contained framed
  glass/metal doorway openings consistent with the semantic candidate.
- Safety boundary: this proves real autonomous movement, semantic
  find-and-stop, live map/camera updates and direct stop. It does not prove a
  metric door position or one-metre stand-off, and no approach was attempted.
  Studio was restored to `movement_locked=true` and read-only run
  `20260725-002303-dimos-go2-studio-go2` with
  `go2connection.movement_enabled=false`.
- Unresolved: CLIP is an uncalibrated ranking model; the current threshold must
  not be treated as object-detection confidence. Candidate-frame retrieval,
  door geometry/depth fusion, live person distance, and one-metre approach
  approval remain to be implemented. The runtime still needs SIGKILL after the
  normal stop timeout.
- Next: expose the selected candidate frame, validate it with a stronger
  configured detector or local replay, project the verified doorway into the
  map using LiDAR/depth and calibration, then perform a separately authorized
  one-metre stand-off test.

### 2026-07-24 — First controlled real-Go2 movement acceptance

- Changed: temporarily unlocked real hardware after the user confirmed
  `START GO2`, reduced the runtime navigation parameter to `0.1`, ran one
  Agent-driven mission attempt, then ran one deterministic twenty-second
  `look_out_for` plus frontier-exploration attempt. The runtime was stopped and
  restored to `hardware_readonly` afterward.
- Why: verify the first complete path from the local Studio to a real Go2,
  including current camera/map data, autonomous frontier motion, visual target
  search, and remote stop behavior.
- Commands/checks: verified the direct `192.168.12.129 -> 192.168.12.1` route,
  9991 signal port, Studio/runtime APIs, battery and 20 MCP tools; inspected the
  live 2D map, camera, and Rerun 3D point cloud; used mission create/start/stop
  and E-STOP APIs; directly called `look_out_for`, `begin_exploration`,
  `end_exploration`, and `stop_navigation`; then verified the read-only process
  command and settings.
- Validation: Go2 network RTT averaged about 7.4 ms; fresh camera frames, map,
  and point cloud were visible; battery was 61% before the first attempt and
  59% before movement. Direct frontier exploration started successfully,
  published a first goal at `(-0.68, 0.09)`, reached it, and published a second
  goal before cancellation. The visible robot pose changed from about
  `(-0.00, 0.01)` to `(-0.23, 0.03)`, proving controlled real movement and map
  updates. `end_exploration` reported a physical stop and `stop_navigation`
  returned `Stopped`.
- Failed acceptance: local `ollama:qwen3:4b` produced no Agent tool call within
  90 seconds, so the Agent mission never moved. The deterministic vision call
  did not identify a door; `look_out_for` and `stop_looking_out` each ultimately
  hit the 120-second RPC timeout. The integrated Rerun pane displayed about
  44.6 seconds of latency near the end of the run.
- Safety boundary: no door was approached or crossed. The twenty-second motion
  window was stopped through direct exploration cancellation, navigation stop,
  and the Studio E-STOP path. Studio is back at `movement_locked=true`, with a
  live `hardware_readonly` runtime using
  `go2connection.movement_enabled=false` and
  `go2connection.auto_stand=false`.
- Unresolved: `mission_timeout_s` is not enforced by a runtime watchdog; a
  stalled Agent does not automatically pause the mission; pending Agent work
  has no independent cancellation path; and the perception-loop tools are not
  reliably cancellable through MCP. Live person-distance safety is also still
  absent.
- Next: implement an independent mission watchdog and pending-Agent
  cancellation, repair background perception-tool cancellation and evidence
  events, choose a responsive task-planning model, then repeat the same
  controlled test before any longer autonomous search.

### 2026-07-24 — Ring-to-Go2 generic Agent platform design

- Changed: proposed a typed local task gateway, reusable task profiles,
  observation-only execution mode, generic spatial evidence contract,
  phase-specific MCP tool permissions, and a reliable ring-to-Studio delivery
  boundary.
- Why: door finding is only the first test; the product needs to turn later
  home objectives from the ring into inspectable Agent tasks without generating
  new robot code or training a model for every request.
- Files: `docs/plans/2026-07-24-ring-to-go2-agent-platform-design.md` and this
  document.
- Current finding: the existing mission API accepts arbitrary text, but its
  Agent prompt is still door-specific; live safety telemetry, model detections,
  Agent tool events, and ring task delivery are not yet wired into one loop.
- Validation: design checked against the current Studio mission/API code,
  external Go2 skill package, DimOS Agent/MCP/perception/navigation docs, and
  the existing Ring Voice Input delivery contract. The user accepted the typed
  task architecture and selected `让机器狗执行` as the explicit ring activation
  phrase. No runtime or robot command was issued.
- Safety boundary: the first end-to-end acceptance is explicitly
  `observe_only`; ring input creates a task draft and cannot unlock or start
  physical movement.
- Unresolved: implementation has not started; live perception, safety telemetry,
  Agent tool events, and ring delivery still need to be wired.
- Next: implement the generic task core and software-only acceptance tests,
  then connect live perception/safety telemetry before the read-only Go2 test.

### 2026-07-24 — Live Go2 read-only Agent and visualization

- Changed: made CLIP preprocessing fully local, added an OpenCV JPEG fallback
  when native TurboJPEG is absent, removed redundant Studio daemonization,
  bypassed environment proxies for local MCP calls, disabled optional
  speech/microphone modules, switched the Agent to local `qwen3:4b`, and fixed
  the embedded Rerun viewer from stale `9090/9876` ports to `9878/9877`.
- Why: bring the real Go2 sensor/Agent stack up on the robot LAN while retaining
  phone-hotspot internet and without depending on paid cloud inference.
- Files: `dimos/perception/image_embedding.py`,
  `dimos/msgs/sensor_msgs/Image.py`, `dimos/agents/mcp/mcp_adapter.py`,
  `dimos/web/templates/rerun_dashboard.html`, `dimos/web/studio/`,
  `extensions/go2-studio-agent/`, their tests, and this document.
- Commands/tests: installed `python-multipart`, pulled Ollama `qwen3:4b`, ran
  the Studio, Blueprint, image, and MCP adapter pytest suites, ran Ruff on all
  changed Python files, checked Studio/runtime/MCP HTTP endpoints, and inspected
  the integrated page in the local browser.
- Validation: Studio and the runtime are live; the map shows real robot pose and
  obstacle data; Rerun shows the live 3D point cloud; new camera frames reach
  spatial perception; 20 MCP tools are registered; and the local Agent made a
  verified `studio_ready` tool call. Motion, exploration, patrol, navigation,
  following, and speech tools were not called.
- Safety boundary: `movement_locked=true`,
  `go2connection.movement_enabled=false`, and
  `go2connection.auto_stand=false`. No movement command was sent.
- Unresolved: the Rerun Camera pane still shows a loading state even though
  camera frames are reaching spatial perception. Live battery display,
  simulation/replay missions, physical E-STOP proof, and all autonomous-motion
  gates remain unvalidated.
- Next: fix the Rerun image-pane rendering, validate battery/telemetry without
  motion, then test the full mission in simulation or replay before any
  supervised empty-area movement.

### 2026-07-24 — Restore Go2 dependency and network connection path

- Changed: installed Git LFS 3.7.1, downloaded and SHA-256-verified the
  `models_clip` LFS object, extracted its ONNX model, and made every Studio
  hardware runtime add the robot IP plus localhost to both `NO_PROXY` and
  `no_proxy`.
- Why: the full Agent Blueprint first crashed because `git-lfs` was missing;
  after that was fixed, local WebRTC requests could still be misrouted through
  the macOS system proxy.
- Files: `dimos/web/studio/service.py`,
  `dimos/web/studio/test_studio.py`, and this document. The Git LFS object is in
  the local Git cache and the extracted model is under `data/models_clip/`.
- Commands/tests: verified Git LFS and its DimOS endpoint, checked out and
  extracted `models_clip`, ran the hardware-read-only Blueprint twice, ran
  `pytest -q dimos/web/studio/test_studio.py` (13 passed), and ran Ruff on the
  changed Studio files (passed).
- Validation: the missing-tool crash is resolved and ONNX initialization
  succeeds. BLE provisioning of `Go2_49060` succeeded and confirmed serial
  `B42D1000PC4C1M86`. No movement command was sent.
- Unresolved: the Mac is now on the venue `ADVX-Players` network, whose client
  isolation prevents LAN discovery/WebRTC. The prior `HUAWEI` personal hotspot
  is currently unavailable. Runtime therefore remains stopped.
- Next: enable a non-isolated personal router/hotspot, connect the Mac to it,
  provision `Go2_49060` to the same SSID with `dimos go2tool connect-wifi`,
  discover its new IP, update Studio settings, and retry hardware-read-only
  startup before any Agent movement.

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
