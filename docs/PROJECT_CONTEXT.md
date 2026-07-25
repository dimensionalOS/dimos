# DimOS Project Context

## Current direction

DimOS Studio is a local, beginner-oriented workbench for Go2 development. It is
an interface over the existing DimOS CLI, Blueprint, MCP, Agent, visualization,
and model configuration layers. It must not become a second robot runtime.

The canonical executable roadmap is now
`docs/plans/2026-07-25-go2-three-stage-robot-validation-plan.md`. It reduces
the current Goal to three robot-verifiable loops: basic Agent-to-Go2
control/return/stop, known semantic-place navigation, and bounded visual search
triggered from the computer or ring. The larger
`docs/plans/2026-07-25-go2-agent-fusion-master-plan.md` remains the long-term
architecture and backlog, but is not the current execution order.

The three-stage plan fuses the DimOS mission/runtime work with the friend repository at
`/Users/johnsonmac/ai_completion/agent`: DimOS remains the only robot runtime
and physical task executor, while the friend Pi Agent, Webhook Gateway, and MCP
Wrapper own instruction ingress, language-to-`TaskSpec` compilation, persistent
instruction/task binding, and user replies. The friend `components/dimos-mcp`
will become the single product composition/launch entry and load the P1 modules;
it must not start a second Go2 Blueprint or duplicate their implementation.

Each stage ends with a real-Go2 review and blocks the next stage when it fails.
Glasses hardware and the reusable household mission-profile platform remain
outside the current three-stage Goal. On 2026-07-25 the user explicitly added a
narrow official-person-follow MVP: reuse `PersonFollowSkillContainer` with a
SiliconFlow Qwen provider adapter and Agent/MCP wiring, without creating a
second tracker or treating it as a canonical navigation task. Future
implementation work must select one numbered `Sx-Tx` task, create a
task-specific child plan, pass its focused tests, and complete the stage's
`Sx-Rx` robot review before advancing.

Stage 1 passed its real-Go2 review on 2026-07-25. Stage 2 known semantic-place
navigation and `S2-R1` real-robot review are complete: `S2-T1` persistent
semantic resolution, `S2-T2` single-Runtime friend-MCP composition/replay,
`S2-T3` Agent `TaskSpec` compilation/persistent task binding/terminal
monitoring, and `S2-T4` Studio supervision passed their software acceptance.
The real Agent route `Agent测试起点 -> Agent测试点B -> Agent测试起点` completed
through one Runtime, including pause/resume/cancel evidence. Studio can confirm
the operator-named current place from fresh odometry, select a confirmed place,
submit/cancel a canonical task, and read the minimum canonical summary. Named
points, the current target, the official planned path and the accepted actual
path are rendered by the official Viewer instead of a second Studio renderer.
The software-only `S3-T2` saved-frame candidate
verifier is also complete, but live Stage 3 integration remains blocked by
`S3-T1` and its own robot review.

For manual operation, Studio launches DimOS's existing
`teleop-hosted-go2-transport` Blueprint and hands the operator to
<https://teleop.dimensionalos.com/>. The official page remains the cockpit for
video, map, joystick, emergency stop, and click-to-navigate.

The active local-LAN Studio is now a thin Stage 2 control surface, not a separate
Mission Control state machine. It names places, selects routes, submits/cancels
tasks through Gateway/Wrapper, and embeds the official Viewer. Legacy local
Mission Control source remains available for rollback, but `/api/mission/*` is
not mounted, its controller is not instantiated at app startup, and the browser
does not poll it. The older Mission Control design/implementation plans are
backlog, not the current execution source.
Candidate-only external visual verification is now also accepted in
`docs/plans/2026-07-25-go2-candidate-vision-verification-design.md`. It keeps
mapping, LiDAR geometry, navigation, safety, and motion local while sending only
up to three explicitly selected candidate screenshots to an OpenAI vision
provider. Its first offline implementation is available through
`python -m dimos.perception.offline_target_verification`.
The active Git baseline is `0a0e3d1f5`, with the later continuous live-frame
capture work still excluded. The default Go2 Studio Blueprint no longer includes
`SpatialMemory`, `PerceiveLoopSkill`, or an embedded `McpClient`; its camera
`observe` MCP tool captures a frame only when an external Agent explicitly
calls it.

The friend product Runtime now restores exactly one official
`PersonFollowSkillContainer`. `QwenVlModel` accepts deployment overrides for
SiliconFlow `Qwen/Qwen3-VL-8B-Instruct`; the official bbox helper requests a
0-1000 coordinate space and converts it to real image pixels before EdgeTAM
initialization. The Agent's `follow_person` intent fixes the query to the person
closest to frame center, and `stop_all` calls official `stop_following`.
This skill uses direct `VisualServoing2D`, explicitly has no obstacle avoidance,
does not automatically re-identify a lost person, and is not a canonical
`MissionExecutor` task.

Safety rule: real hardware movement is locked by default. Simulation is the
preferred place to test new skills. Hardware startup requires both disabling the
lock and selecting the explicitly labelled movement-enabled mode.
The separate `hardware_readonly` mode connects sensor streams while
`go2connection.movement_enabled=false` rejects movement at the robot module.
For the current controlled test session, the operator has confirmed that all
tests are taking place in a cleared environment, so repeated confirmation
prompts are not required. Built-in obstacle avoidance, stop controls, and
single-runtime enforcement remain active.

## Current implementation

- Local FastAPI workbench: `dimos/web/studio/`
- User-editable external Blueprint package: `extensions/go2-studio-agent/`
- Friend Agent repository: `/Users/johnsonmac/ai_completion/agent`
- Friend robot MCP entry: `agent/components/dimos-mcp/`
- Friend product MCP wrapper:
  `agent/components/agent-framework/dimos-mcp-wrapper/`
- Friend instruction/Agent gateway:
  `agent/components/agent-framework/agent-webhook-gateway/`
- Standalone Agent Console:
  `/Users/johnsonmac/ai_completion/agent/启动 Go2 Agent 控制台.command`
- Clickable macOS bundle: `apps/DimOS Studio.app`
- Native AppKit control app: `apps/DimOS Native.app`
- Double-click real-Go2 launcher: `启动 DimOS 机械狗.command`
- Canonical launcher source: `scripts/start_dimos_go2.command`
- macOS launcher: `scripts/open_dimos_studio.command`
- Figma baseline: <https://www.figma.com/design/fpQYydt4G0EVFubTDAI1Uy>
- Default Studio URL: <http://127.0.0.1:8765>
- Default Agent Console URL: <http://127.0.0.1:8080/>
- Existing DimOS map/command center URL: <http://127.0.0.1:7779/command-center>

Live state on 2026-07-26 after the moving-scan run: the Go2 remains reachable
at `192.168.12.1`, but no DimOS robot Runtime, MCP `:9990` listener, or
navigation child process is active. Runtime PID `27385` completed the bounded
five-minute scan and stopped successfully; replacement PID `29445` was later
terminated because the frontier explorer resumed goals before MCP tools were
registered. The command-center map tab is retained with the final received map
and trajectory, but it is not live while the Runtime is offline. The earlier
six `房间点位-*` records and the new session-world candidates must not be mixed.
Only `环境标记点-04` was actually created in this run, and no current point has
restart-stable `map` coordinates.

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
- thin Stage 2 naming/route-selection/task-submission control with the existing
  DimOS viewer embedded at port 7779
- no Studio-local mission lifecycle, route renderer, `/api/mission/*` routes or
  legacy Mission polling in the active App
- one E-STOP button that calls `POST /api/stage2/stop-all`, which forwards the
  canonical Wrapper `stop_all` once and does not mutate local task state
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
- the standalone Agent Console product profile uses
  `RERUN_OPEN=none/RERUN_WEB=true` and embeds the same Runtime's official
  Rerun web viewer at `:9878`; it does not start a native Viewer or another
  Rerun bridge. The separate legacy AppKit path can still use native Rerun.
- first-party `McpServer` without an embedded cloud model or API key
- first-party `UnitreeSkillContainer`, exposing `relative_move` to external MCP
  clients without adding an embedded model
- one background `MissionExecutor` exposing typed start/pause/resume/cancel and
  status skills while retaining `CAP_MOVEMENT` until a task becomes terminal
- one persistent `SemanticWorld` that stores only manually confirmed places
  with stable IDs, aliases, map ID/version and map-frame poses; it resolves
  current-map names through a typed DimOS `DestinationResolverSpec` and rejects
  unknown or wrong-map destinations before navigation
- one product-runtime `SemanticVisualizationAdapter` with no skills, hardware
  connection or navigation ownership. It renders confirmed semantic places,
  the canonical current target and RobotSummary-accepted actual odometry
  samples in the existing official Rerun Viewer; the official planner `path`
  remains the only planned-route stream.
- one operator-only Studio endpoint/button that turns a fresh finite odometry
  pose into a confirmed semantic place; the Agent cannot self-confirm a place
- the friend `components/dimos-mcp` now composes that `SemanticWorld` and the
  canonical `MissionExecutor` exactly once in both replay and Go2 modes; its
  default product profile exposes high-level mission/status/stop tools but not
  legacy raw movement, exploration, patrol or sport actions
- one cross-repository MCP replay in which a confirmed-place alias produced a
  single task ID, entered navigating, was cancelled, and ended cancelled with
  the replay navigator idle; `stop_all` also cancels the active mission first
- the friend product Gateway now runs a no-tool Pi parameter compiler, accepts
  only strict `go_to_place + destination`, creates the canonical task ID and
  TaskSpec itself, persists `instruction_id -> task_id`, submits `start_task`
  once, and produces a user reply only after the same task is terminal and
  inactive
- an interrupted Gateway with a persisted binding resumes only
  `get_task_status`; it never resubmits `start_task`, while an unbound legacy
  processing row fails closed
- conservative navigation recovery tuning: heading errors above 70 degrees
  rotate in place, translational stalls replan after 6 seconds, and a goal gets
  up to 8 same-area replans before cancellation
- a bounded navigation recovery supervisor that classifies the failure cause,
  alternates fresh-path requests with rotate/rescan headings, reports
  `NavigationState.RECOVERY`, publishes typed JSON events, and rejects an A*
  result if stop or a newer goal arrived while it was computing
- foreground process supervision on macOS, avoiding the broken RPC state seen
  after DimOS's `--daemon` double fork
- one-click real-Go2 startup with network gating, native Viewer, MCP, lidar
  mapping, and hardware movement locked by default
- one-metre stand-off goal calculation that remains advisory until navigation
  is explicitly allowed

## Known constraints

- `DimOS Native.app` is a native arm64 AppKit bundle and uses the native Rerun
  viewer for camera/map rendering. The legacy Studio and original command
  center remain browser pages for configuration and 2D controls.
- A reachable Go2 port is not proof of a complete WebRTC control link.
- Hosted Teleop is not connected until the broker accepts the API key and the
  official page visibly shows the robot, video, and current status.
- The Hosted Teleop key is intentionally absent from repository files and
  settings. A valid `dtk_...` key must exist in macOS Keychain before launch.
- `GO2Connection.start()` can make the real robot stand, so hardware start must
  remain an explicit action.
- Map and teleoperation views only become available when the selected DimOS
  Blueprint successfully starts its visualization stack.
- The friend `dimos-mcp` source includes the persistent `SemanticWorld`,
  canonical `MissionExecutor` and read-only Viewer Adapter. Real named-place
  navigation has passed separately; this adapter change itself has only
  software/replay validation.
- Studio exposes canonical task ID, semantic places, fresh-odometry/recovery
  summaries and task-ID cancellation by reading Wrapper/Gateway state. The
  official Viewer, not Studio, renders planned/actual trajectories. Studio owns
  only UI submission/reply idempotency evidence, not robot task state. Never run
  the standalone friend robot MCP and the Studio Go2 Blueprint together.
- Importing DimOS starts Zenoh threads that do not naturally let some macOS
  `unittest` processes exit after the suite prints `OK`. S2-T2 therefore runs
  the full suite in an isolated child and reclaims it only after the explicit
  test terminal result; leaving those test processes alive can compete with a
  later robot Runtime.
- The official navigation tool can return `Navigation goal reached` after
  `stop_all` cancelled a short action. Treat this as an upstream result string,
  not physical completion; completion requires fresh odometry and the canonical
  mission state.
- P1-T3 executes replan and rotate/rescan in production. Clearing stale local
  observations, reversing, and generating an alternate approach are still
  capability-gated off because the current grid has no obstacle-age/provenance
  layer, no verified rear-clearance contract, and no alternate-pose provider.
  The 86-test result does not prove live blocked-path recovery.
- The frozen legacy Mission Control person-yield state machine has no active
  Studio route and never received live person distance from a detector. Do not
  claim live crowd yielding from that code.
- DimOS CLIP spatial-memory ingestion is now wired into the deterministic
  mission runner and has passed one bounded real-Go2 candidate-search test.
  CLIP provides a semantic camera-viewpoint candidate, not a door bounding box
  or metric door pose. A second-stage geometric/visual verifier is still
  required before any one-metre approach.
- A software E-STOP is not the Go2's physical power switch. The active Studio
  button forwards the single canonical `stop_all` call and reports component
  failures; it does not maintain a second local stop state. If the computer,
  network, browser, or robot firmware is unresponsive, the operator must still
  use the Go2's physical stop/power control and physically verify it stopped.
- Supervised venue motion has been validated at the configured 0.2 m/s cap with
  the lidar cost map and replanning stack active. Camera-only person distance is
  not part of the active Stage 2 control plane, so the claim is obstacle
  replanning, not independently verified person classification or social
  navigation.
- The candidate-only verifier has passed offline validation against the saved
  578-frame real-Go2 memory. This shell does not currently have
  `OPENAI_API_KEY`, so no cloud model verdict has been obtained. The verifier is
  not yet connected to the live mission runner, camera/LiDAR projection, or
  movement authorization.

## Recent task log

### 2026-07-26 — Agent natural-language action routing

- Changed: the friend Gateway now recognizes bounded full-sentence colloquial
  commands such as `往前走`, `请向左走一下`, `马上停`, and `查询状态` before the
  product task compiler. Motion still uses the existing deterministic
  `stop_all -> relative_move` path with fixed `0.2 m / 15°` parameters. Named
  places, finite routes, marking and follow-person remain canonical high-level
  Agent tasks.
- Failure handling: the MCP HTTP client now classifies unavailable, timeout,
  protocol and Runtime-rejected failures. The Console displays failure as
  `未执行` instead of treating every persisted reply as task success. Timeout
  remains ambiguous and explicitly forbids automatic resubmission.
- Boundaries: no second Agent, MCP, Runtime, Planner or state owner was added;
  no model can choose velocity or raw motion parameters. Negated, compound,
  destination-bearing and distance-bearing phrases do not enter the direct
  physical-command path.
- Files: friend Gateway service/MCP client/Console/tests/docs,
  `docs/plans/2026-07-26-go2-agent-natural-language-action-design.md`, and this
  document.
- Validation: Gateway `81/81`, TypeScript, build, JavaScript syntax and both
  repository diff checks passed. The refreshed Gateway is online at `:8080`;
  Runtime `:9990`, Wrapper `:9991` and Viewer `:9878` remain offline. A read-only
  `查询状态` request returned the new unavailable-chain diagnostic. No physical
  movement command was sent.

### 2026-07-26 — Standalone map + text Agent Console

- Changed: added a same-origin front end to the existing friend
  `agent-webhook-gateway :8080`. It embeds the official Rerun web viewer
  `:9878`, submits text to the existing `/v1/instructions`, polls persisted
  instruction/task/outbox state, and sends the exact text `停` through the
  existing priority path. It does not call MCP or own robot state.
- Why: daily robot task control must work without using Codex while preserving
  one Gateway, Wrapper, MCP Runtime, navigation stack and task owner.
- Files: friend Gateway HTTP/config/store/service, `web/`, tests, docs/ADR,
  `agent/启动 Go2 Agent 控制台.command`,
  `docs/plans/2026-07-26-go2-agent-console-design.md`, this document, and
  private `~/.dimos/go2-stage2/product.env`.
- Commands/tests: Gateway focused tests `34/34`, TypeScript check, build,
  browser fake-Agent submit/reply visual check, `zsh -n`, and diff checks.
- Validation: the rendered Console showed the offline official-map placeholder,
  accepted one fake validation instruction, polled the same persisted ID, and
  displayed the final reply. No real Runtime, Wrapper, product Agent or Go2
  connection was started and no motion command was sent.
- Current state: product ports `8080/9877/9878/9990/9991` are not listening.
  The feature is implemented but live map/control remains unproven until the
  operator explicitly starts the single Product Runtime and then opens the new
  launcher. Do not use the old Native App “connect” action concurrently because
  it starts a different Blueprint.

### 2026-07-26 — Five-minute moving scan and live map

- Changed: ran the Go2 against `192.168.12.1` in frontier-exploration mode for
  300.7 seconds, displayed the official command-center cost map and actual
  trajectory, stopped at the time boundary, and saved four widely separated
  candidates selected only from physically traversed poses.
- Why: the user explicitly required five minutes of real movement and current
  environment collection before selecting four points; the earlier collection
  spent most of its window stationary and did not meet that intent.
- Files: `runtime/exports/scene-captures/20260726-043813-go2-five-minute-scan.json`
  and this document. Primary logs are
  `logs/dimos_20260726_043700_27385.jsonl`,
  `logs/dimos_20260726_043711_27431.jsonl`, and
  `/tmp/go2-live-scan-runtime.log`.
- Commands/tests: confirmed one Runtime owner, called canonical `stop_all`,
  `BalanceStand`, `SwitchJoystick`, and `begin_exploration`; sampled 103 fresh
  summaries; selected 51 deduplicated candidate poses; called `stop_all` at the
  five-minute boundary; checked pairwise distances; displayed the map; and
  validated the evidence JSON.
- Validation: the robot travelled 51.097 m, reached an observed 0.62 m/s, and
  the visible map contained obstacle/cost data, robot pose, and red actual
  trajectory. Candidate separations range from 5.580 m to 12.845 m.
  `stop_all` reported no failed component and the robot was stationary at
  0.0019 m/s. One stopped point was really tagged as `环境标记点-04` at
  `world=(-0.145914,-10.740105)`; the other three tags were not claimed.
- Unresolved: `world -> map` remained unavailable. A post-scan revisit returned
  `Navigation was cancelled or failed`, followed by stale odometry/WebRTC.
  After reconnection, a replacement Runtime registered zero MCP tools while the
  frontier explorer resumed issuing goals; the command-center Stop button only
  cancelled the current goal. Runtime PID `29445` and its process group were
  terminated, leaving no port-9990 listener or navigation child process.
- Next: before another motion run, make exploration restart-safe and route the
  UI Stop button to canonical `stop_all`/`end_exploration`. Then re-establish
  fresh odometry and complete the remaining three stopped-point tags. Do not
  treat the four world-frame candidates as restart-stable semantic places.

### 2026-07-25 — Preliminary five-minute scene capture (superseded)

- Changed: ran one bounded five-minute Go2 scene collection against
  `192.168.12.1`, stopped all motion at the time limit, and saved four new
  traceable scene candidates selected from the run's actual camera/odometry
  records. No legacy room point was reused and no persistent semantic place was
  created.
- Why: model the current environment as a fresh session and obtain four
  reasonably separated points without treating old waypoint state or odometry
  drift as new evidence.
- Files: `runtime/exports/scene-captures/20260725-235324-go2-scene-model.json`
  and this document. Runtime evidence remains in
  `assets/output/memory/spatial_memory/chromadb_data/chroma.sqlite3`,
  `logs/dimos_20260725_235159_87537.jsonl`, and
  `logs/dimos_20260725_235159_87543.jsonl`.
- Commands/tests: monitored fresh RobotSummary odometry for 300.2 seconds,
  queried the persisted SpatialMemory records, checked pairwise point
  distances, called canonical `stop_all`, confirmed stationary state, validated
  the evidence JSON, and ran `git diff --check`.
- Validation: 39 timestamped SpatialMemory frames were recorded; the selected
  points have a minimum pairwise separation of 0.996 m and a 3.069 m maximum
  span. The final state was stationary at 0.0007 m/s and `stop_all` reported no
  failed component.
- Unresolved: `world -> map` was still unavailable; relocalization scored
  0.439 against the 0.45 threshold with 35,387 points. The four points are
  therefore current-session `world` candidates, not restart-stable named
  navigation locations. The Runtime exited after evidence flush; the wrapper
  remains available on port 9991, but port 9990 is not currently listening.
- Superseded by: the 2026-07-26 moving scan above. Keep this artifact only as
  historical evidence; do not use its four candidates for the current route or
  mark them as completion of the user's later five-minute moving-scan request.

### 2026-07-26 — Official native map Viewer restored

- Changed: switched the Stage 2 product environment from
  `VIEWER=none/RERUN_OPEN=none` to `VIEWER=rerun/RERUN_OPEN=native`, and made
  Hugging Face/Transformers startup explicitly offline so cached
  `SpatialMemory` initialization cannot stall on network retries. No second
  Viewer bridge or second robot Runtime was retained.
- Why: the semantic visualization adapter was publishing typed state, but the
  only visible page was the port-7779 fallback command center. The official
  in-Runtime `RerunBridgeModule` is required to receive internal
  `global_map/global_costmap` streams.
- Live validation: one product Runtime main PID `90335`, one native
  `dimos-viewer` PID `90411`, one MCP listener on `127.0.0.1:9990`, one Rerun
  gRPC listener on `:9877`, and one Viewer interaction server on
  `127.0.0.1:3030`. The Viewer visibly renders live camera, `global_map`,
  `global_costmap`, pose, planned path and semantic adapter streams. Its 3D
  view was maximized and side panels were collapsed for a map-only operator
  view. No movement command was sent.
- Files: `/Users/johnsonmac/.dimos/go2-stage2/product.env` and this context.
- Unresolved: relocalization still skips at roughly 20k local points against
  `min_local_points=35000`, so `world -> map` is unavailable and map-frame
  semantic markers remain intentionally empty. The live world-frame map is
  unaffected.

### 2026-07-25 — Official Viewer semantic-route adapter

- Changed: added low-frequency semantic-place and mission-status `Out[String]`
  snapshots to the existing Stage 2 owners. `RobotSummary` publishes only the
  actual odometry points it already accepted. The friend product Runtime adds
  one read-only `SemanticVisualizationAdapter` that emits official
  `EntityMarkers` plus a blue actual-path entity; official planner `path`
  remains the sole planned-route source. Studio removed its local route SVG,
  legacy Mission UI/polling and active `/api/mission/*` routes; its E-STOP now
  forwards canonical `stop_all`.
- Why: MCP request/response state was visible to Studio but named points,
  current target and actual travel were not present in the native Viewer.
- Ownership: no new Agent, MCP layer, Navigator, GO2Connection or task state
  owner. Studio remains a naming/route-selection/submission surface.
- Files: DimOS extension semantic world/mission executor and tests; friend MCP
  adapter/Blueprint/summary/config/tests/docs; this context.
- Checks: DimOS extension + Stage 2 Studio/legacy-freeze 81/81; friend MCP non-integration
  65/65 and isolated integration 14/14; focused Ruff, launcher shell syntax
  and both repositories' `git diff --check` passed. This task's commands did
  not start a robot process or send a movement command.
- Read-only live check: a product Runtime with main PID `79657` appeared at
  23:08 and serves `9990/7779`; it was not launched by this task command.
  `get_robot_summary` reports ready/fresh odometry and the canonical task is
  idle. This task sent no motion command.
- Live boundary: relocalization currently reports
  `ready=false/reason=transform_unavailable`. The Adapter therefore
  intentionally suppresses map-frame named markers instead of drawing them in
  the wrong world pose. Actual-path publication is available; named-point and
  current-target visibility still require relocalization recovery and visual
  confirmation in the Viewer.
- Packaging: friend `dimos-mcp` now declares Python `>=3.12,<3.13`, matching
  `dimos-go2-studio` and the tested DimOS Python 3.12 runtime.
- Unresolved: the root `npm run check` pinned-dependency scan descends into the
  existing `components/dimos-mcp/.venv` and rejects third-party Dash/Plotly
  Jupyter package metadata. This does not affect the Python/Viewer tests.

### 2026-07-25 — Room frontier sampling and automatic semantic places

- Changed: no control code changed. Reused the friend MCP's existing
  `StrollSkill(WavefrontFrontierExplorer)` and the canonical
  `confirm_semantic_place` boundary to persist `房间点位-01` through
  `房间点位-06`. After detecting changing relocalization transforms, rebuilt
  all six poses from their timestamp-matched raw `actual_path` samples and
  reprojected them through one current transform. Adjacent points are
  `0.900–1.111 m` apart.
- Live validation: one Runtime owner PID `67504`, Go2 `192.168.12.1`, fresh
  odometry and relocalization ready. The pass travelled `5.339 m` in about
  `14 s`; `stop_all` succeeded and the next sample was stationary at
  `0.001 m/s`. After reprojection, six stationary samples over `10 s` had only
  `0.0008 m` maximum map-plane spread.
- Boundary: the lower Runtime used its maintenance profile only to reach the
  existing `start_stroll` skill. The user-facing Wrapper remained on its
  product allowlist, so maintenance sport commands were not exposed through
  the Agent. No second Go2 Runtime or new browser page was started.
- Unresolved: relocalization `ready=true` means a transform exists, not that
  repeated estimates are stable. The six points are coherent for this live
  Runtime; persistence across another Runtime restart still requires a
  separate route check.
- Next: wait for the user to name an ordered subset of the six points, then
  execute it through the existing Agent `visit_route` path and report
  per-leg terminal plus odometry evidence.

### 2026-07-25 — Friend Agent named-route live validation

- Changed: no DimOS control code changed. Started one product stack against
  Go2 `192.168.12.1`, with Runtime owner PID `65058`, MCP `:9990`, product
  Wrapper `:9991`, Gateway `:8080`, and a local reply sink `:9080`. Forced
  Hugging Face/Transformers offline mode so the existing local CLIP assets did
  not block WebRTC startup with metadata retries.
- Live validation: WebRTC, camera, LiDAR, fresh odometry and official
  `RelocalizationModule` were live. The Agent confirmed `Agent测试起点` and
  `Agent测试点B` on map `venue-s2@61d67bd5e844`, separated by `0.625 m`.
  A setup `relative_move(0.8 m)` produced `0.7671 m` net odometry displacement.
  The finite route `起点 -> B -> 起点` completed with per-leg arrival evidence
  and finished `0.0166 m` from the start. A second route was observed moving,
  paused to stationary, resumed to `0.5736 m/s`, then cancelled to a terminal
  inactive state. Final `stop_all` left the robot stationary and the local
  motion executor idle.
- Boundary: this run validates Agent input through the friend Wrapper into the
  single official DimOS navigation Runtime. It does not validate ASR, ring or
  glasses input, person following, restart recovery, or temporary-obstacle
  behavior.
- Unresolved: the Gateway's fixed manual step calls official
  `relative_move(0.2 m)`, but the current planner arrival tolerance accepted
  that goal after only about `0.0261 m` net displacement. The short-direction
  input therefore remains failed even though longer relative movement and
  semantic navigation work. Also keep odometry drift/relocalization changes
  separate from per-mission distance claims.

### 2026-07-25 — Costmap and odometry sidecar export

- Changed: added a read-only `scripts/dump_costmap_odom.py` sidecar exporter,
  its focused serialization test, and one 40-costmap/40-odometry JSON+NPZ
  sample under `runtime/exports/costmap-odom/replay-20260725-1916/`.
- Why: provide lossless, self-describing numeric map data for independent
  plotting without adding a second Go2 connection or publishing commands.
- Files: `scripts/dump_costmap_odom.py`,
  `scripts/test_dump_costmap_odom.py`, the replay export, and this context.
- Commands/tests: focused unittest passed, Python compile passed,
  `git diff --check` passed, and a foreground headless DimOS Go2 replay supplied
  the `global_costmap` and `odom` streams over Zenoh.
- Validation: JSON and NPZ each contain 40 costmaps and 40 poses; costmaps are
  lossless `int8`, use 0.05 m/cell, and retain each variable frame shape,
  origin, frame ID and timestamp. The temporary Runtime was stopped afterward.
- Unresolved: the Mac was not on the Go2 LAN, so this artifact is explicitly
  `source=replay`, not current-room live data. Rerun the same exporter against
  the single active product Runtime after live `192.168.12.1` reachability is
  restored.

### 2026-07-25 — Official person-follow adapter in friend Agent Runtime

- Changed: configured official `QwenVlModel` for provider/model/base URL
  overrides; standardized official bbox output as normalized 0-1000 JSON and
  converted it to image pixels; restored one official
  `PersonFollowSkillContainer` in the friend Go2 Blueprint; exposed
  `follow_person` through product MCP/Wrapper/Gateway; added
  `stop_following` to `stop_all`.
- Why: DimOS already owns Qwen initial detection, EdgeTAM tracking and visual
  servoing. A second custom tracker would duplicate official behavior and make
  the single-Runtime architecture worse.
- Files: DimOS Qwen/query source and tests, corrected implementation plan,
  friend Gateway/MCP/Wrapper/configuration/tests/docs, and this context.
- Checks: DimOS provider/query 6/6; friend Gateway 20/20; Blueprint 8/8; MCP
  integration 14/14; stop 3/3; Wrapper unit/integration 5/5; launcher 3/3.
  A real SiliconFlow smoke
  test on a non-robot sample image returned a valid pixel bbox in about four
  seconds through the official Qwen/query path.
- Validation boundary: software/provider only. No Go2 Runtime was started and
  no motion command was sent. Official follow has no obstacle avoidance,
  automatic re-identification, canonical task binding or terminal evidence.
- Next: perform a separate cleared-path robot review for center-person lock,
  short follow, loss stop and explicit `stop_all`; do not test crowds or clutter
  until a navigation-aware follow layer is designed.

### 2026-07-25 — Stage 2 stable-frame relocalization preparation

- Changed: `SemanticWorld` now stores confirmed places in stable `map` frame
  and resolves them into the current `world` frame. The friend Go2 Runtime now
  requires a premap, composes one official `RelocalizationModule`, publishes
  relocalization readiness, and bypasses system proxies for the robot IP.
- Why: raw odometry coordinates are not reusable after a Runtime restart, and
  macOS system proxying caused false `/con_notify` WebRTC failures.
- Files: shared semantic-world source/tests, friend MCP
  config/Blueprint/robot-summary/launcher/Stage 2 audit/tests/docs, this
  context and the three-stage plan.
- Audit: the new `dimos-stage2-audit` has read-only preflight, one-trip and
  cancel-check modes. It proves the one product owner, fresh odometry,
  relocalization, current-map places and idle task before any start; it never
  retries movement and records exact task ID, terminal state, stable-frame
  arrival error, and cancellation idle evidence.
- Checks: shared extension 50/50 and friend MCP 74/74 passed; focused Ruff
  passed. Friend root Biome passed; the aggregate check then stopped on
  third-party package manifests inside its generated Python venv. Its import,
  shrinkwrap, install-lock and browser-smoke checks passed separately; the
  existing `packages/ai` model-catalog type errors still block root TypeScript.
  A live read-only connection received fresh camera, LiDAR and odometry after
  ICE/peer/data-channel verification. No stand or movement command ran.
- Runtime asset: `~/.dimos/go2-stage2/venue-s2-recording.pc2.lcm` contains
  35,357 points and hashes to
  `61d67bd5e8447b4732a8cba19ffd331d097d409ce175f2f5b02fa066f0d5966f`.
- Validation boundary: official offline relocalization correctly rejected the
  static local scan because about 35.7k points is below its 50k minimum.
  S2-R1 is not passed and no stable `world -> map` transform is yet proven.
- Next: after a fresh `START GO2，场地已清空`, run one friend Runtime, perform
  a short bounded mapping movement, require relocalization ready, then confirm
  the two places and execute S2-R1 A/B/C.

### 2026-07-25 — S3-T2 candidate-only vision gateway (saved-frame replay)

- Changed: added provider-neutral `CandidateVisionGateway` with immutable
  job/provider metadata, bounded daemon workers, immediate submit, typed
  deadline/exception fallback, late-result suppression, candidate idempotency
  and payload-conflict rejection.
- Why: an external visual provider must inspect only a few explicit candidate
  frames and must never block the local robot-control/stop thread or authorize
  movement.
- Files: `dimos/perception/vision_gateway.py`,
  `dimos/perception/test_vision_gateway.py`, S3-T2 child plan, three-stage plan
  and this context.
- Checks: combined target verifier/offline CLI/gateway suite 20/20; Ruff and
  `git diff --check` pass.
- Validation boundary: only synthetic saved-frame replay with fake providers;
  no API key, network provider, real image upload, Go2 Runtime or motion.
- Unresolved: S3-T1 synchronized live observations, S2-R1 robot review and
  S3-R1 candidate-stop robot review remain incomplete.
- Next: wait for explicit direction to implement S3-T1/live integration or
  resume S2-R1; do not claim Stage 3 passed.

### 2026-07-25 — S2-T4 Studio supervision complete; S2-R1 startup interrupted

- Changed: added the Stage 2 supervision adapter/API/UI for current map,
  confirmed places, canonical task state, planned path, actual odometry path,
  latest recovery event and task-ID cancellation. Added an operator-only
  “confirm current place” flow requiring fresh finite odometry and normalized
  aliases.
- Runtime correction: friend `dimos-mcp` now composes the lean official
  `unitree_go2` Blueprint instead of `unitree_go2_spatial`; removed
  `SpatialMemory`, `NavigationSkillContainer`, `PerceiveLoopSkill`,
  `ReturnToUserAndGreetSkill` and `StandaloneAgentBridge` from the Stage 2 Go2
  graph. `stop_all` cancels the single `NavigationInterfaceSpec` directly.
- Checks: Studio 29/29; friend MCP 64/64; Wrapper 16/16; focused Ruff, frontend
  JS syntax and diff checks pass. Friend root `npm run check` passed format,
  pinned-dependency, import and lock checks, then failed on unrelated existing
  `packages/ai` model-catalog TypeScript errors.
- Hardware evidence: one WebRTC attempt reached ICE completed, peer connected
  and data channel OK. It failed before readiness because of the now-fixed
  module dependency. The retry coincided with a temporary Mac-to-Go2 LAN drop;
  ping later recovered to 0% loss, but Runtime was not restarted after the user
  reprioritized S3-T2. No navigation or motion command was sent.
- Unresolved: restart the one Go2 product stack, obtain fresh odometry, confirm `测试起点` and
  `门口测试点`, then execute the S2-R1 semantic round trips, restart/
  relocalization and bounded-obstacle review.
- Next: resume only S2-R1; Stage 3 remains blocked.

### 2026-07-25 — S2-T3 Agent task binding and terminal monitor

- Changed: product Pi became a no-tool `go_to_place` parameter compiler;
  Gateway now owns strict TaskSpec construction, deterministic task IDs,
  persistent instruction/task bindings, single-submit semantics, terminal
  monitoring, restart recovery, and deterministic user replies. Product Wrapper
  now forwards the six mission/semantic tools while hiding lower-level movement.
- Why: model text and `start_task accepted` are not physical completion, and
  retries/restarts must not create duplicate robot missions.
- Files: friend Gateway and Wrapper sources/tests/docs, root `USAGE.md` and
  `CONTEXT.md`, S2-T3 child plan, and the three-stage roadmap.
- Checks: 39 focused Gateway tests and package TypeScript check passed; 16
  Wrapper tests and Ruff passed. Root formatting/dependency checks passed after
  excluding the local generated Python venv, but full-root TypeScript remains
  blocked by unrelated existing `packages/ai` model-catalog type errors.
- Validation: software only; no Go2 Runtime was started and no motion command
  was sent.
- Next: complete S2-T4 Studio supervision, then start one friend product stack
  for S2-R1.

### 2026-07-25 — S2-T2 single-Runtime friend MCP mission composition

- Changed: made friend `components/dimos-mcp` explicitly depend on the shared
  `dimos-go2-studio` package; composed exactly one persistent
  `SemanticWorld`, canonical `MissionExecutor`, navigator and MCP Server in
  replay and Go2 Blueprints; added product/maintenance tool profiles; made
  `stop_all` cancel the active mission first; added an MCP task replay.
- Why: Stage 2 needs the friend MCP to be the only product Runtime while
  reusing, rather than copying, the canonical mission and semantic-place
  implementation.
- Files: friend MCP `pyproject.toml`, configuration, Blueprint, navigation,
  tool contract, server, stop chain, environment example, README and tests;
  DimOS mission executor/tests; and
  `docs/plans/2026-07-25-S2-T2-friend-mcp-mission-composition.md`.
- Checks: friend product tests 7/7; local MCP integration 14/14; complete friend
  MCP suite 62/62; DimOS semantic/executor/Blueprint suite 22/22; Ruff and
  `git diff --check` pass in both repositories. The isolated full-suite harness
  reclaimed its child after `OK` because DimOS Zenoh threads otherwise keep
  Python alive on macOS.
- Replay proof: task `task-replay-semantic-001` resolved alias `演示点`, entered
  `navigating`, cancelled with the same ID, and ended `cancelled` with
  `navigation_idle=true`; no hardware I/O occurred.
- Runtime boundary: the stale prior robot Runtime, Wrapper and Gateway were
  deliberately stopped before integration tests. No target ports or test
  Runtime remain; the Go2 LAN is reachable, but current fresh telemetry is not
  claimed.
- Next: execute `S2-T3 — Agent TaskSpec 和任务绑定`; do not restart or move Go2
  until the Agent chain and S2-T4 UI are ready for one controlled S2-R1 run.

### 2026-07-25 — S2-T1 persistent confirmed-place SemanticWorld

- Changed: added a strict, atomic JSON `SemanticWorld` for manually confirmed
  places; stable place IDs, aliases, map ID/version and pose persistence;
  fail-closed map/unknown/corrupt-store behavior; typed
  `DestinationResolverSpec`; one `SemanticWorld` wired into the product
  Blueprint and the existing `MissionExecutor`.
- Why: Stage 2 needs deterministic name-to-map-pose resolution before the
  friend MCP, Agent compiler or real robot can honestly execute “去会场正门”.
- Files:
  `extensions/go2-studio-agent/src/dimos_go2_studio/semantic_world.py`,
  `mission_executor.py`, `blueprint.py`, their focused tests, and
  `docs/plans/2026-07-25-S2-T1-minimal-persistent-semantic-world.md`.
- Checks: initial RED failed with the expected missing-module import; final
  focused suite passed 21 tests; Ruff passed; `git diff --check` passed;
  structural and annotation compliance both passed for
  `SemanticWorld -> DestinationResolverSpec`.
- Runtime check: the single friend Runtime, Wrapper and Gateway processes and
  ports remain present. The Mac can ping `192.168.12.1` and reach its TCP
  `9991`, but `get_robot_summary` reports stale odometry (last sample
  `2026-07-25T04:49:08Z`, motion state unknown). Therefore network reachability
  is confirmed, but a live robot telemetry/control link is not.
- Unresolved: `S2-T2` must compose exactly one `MissionExecutor` and
  `SemanticWorld` into the friend MCP, provide current map identity, replay the
  integration, then refresh/restart that single Runtime before hardware work.
  `S2-R1` has not started and the robot did not move during this task.
- Next: execute only `S2-T2 — friend MCP product composition and replay`.

### 2026-07-25 — Team collaboration handoff and Stage 2 entry

- Changed: added
  `docs/plans/2026-07-25-go2-team-collaboration-handoff.md` with the two-repo
  ownership model, current usable capabilities, explicit non-capabilities,
  Stage 1/2/3 tasks, teammate ownership, Git-sharing risk, test commands, and
  Mac-only hardware acceptance template. Corrected the stale footer in the
  canonical three-stage plan so the current Goal is Stage 2 and the next task
  is `S2-T1`.
- Why: teammates need a source-backed handoff that does not rely on chat
  history or confuse locally implemented code with remotely shared code or
  real-hardware completion.
- Files: the collaboration handoff, the canonical three-stage plan, and this
  document.
- Runtime check: the additional known-map patrol smoke test increased total
  odometry from about 4.38 m to 83.99 m. It was then stopped through the friend
  MCP `stop_all`; no stop component failed, and three consecutive fresh
  odometry reads were stationary at approximately 0.0004–0.0012 m/s. The
  Runtime itself remains available.
- Validation: source/plan/status review across both dirty worktrees; live
  single-Runtime/port and fresh-odometry inspection; Markdown and Git
  whitespace checks only. No integration test was run while the real Runtime
  was active.
- Unresolved: both repositories still contain substantial uncommitted local
  changes, so teammates cannot obtain the implemented work from the current
  remotes. The user must approve a fork/branch and commit/push split before
  code collaboration begins.
- Next: publish reviewable Stage 1 branches, then execute only
  `S2-T1 — Minimal Persistent SemanticWorld`.

### 2026-07-25 — Stage 1 Agent-to-Go2 real-robot gate passed

- Changed: implemented exclusive Runtime ownership and port preflight in friend
  MCP; five-tool validation profiles in Wrapper/Gateway; odometry-derived
  `get_robot_summary`; model-free deterministic validation Agent; correct
  Wrapper worker-port serialization; localhost proxy bypass; Native actual-path
  display.
- Why: prove the actual friend Agent → Wrapper MCP → one DimOS Runtime → Go2
  path without depending on model balance or treating HTTP/MCP acceptance as
  physical success.
- Files: friend repository `components/dimos-mcp/`,
  `components/agent-framework/dimos-mcp-wrapper/`,
  `components/agent-framework/agent-webhook-gateway/`; DimOS
  `dimos/agents/mcp/mcp_adapter.py`, `apps/DimOS Native/`; canonical three-stage
  plan and both project context documents.
- Checks: friend MCP 54 tests + Ruff; Wrapper 16 tests + Ruff; Gateway 27 tests,
  TypeScript check/build; Native 4 tests and ad-hoc codesign; MCP adapter 2
  tests; both repositories `git diff --check`.
- Live validation: Test A observed `0.1770 m` actual movement for a 0.3 m
  request; Test B reduced Runtime-start distance `0.3262 → 0.1836 m`; Test C
  issued stop while odometry was moving and first observed stationary in
  `0.600 s`, stable by `2.318 s`; final stop reported fresh stationary odometry
  and local command state idle.
- Unresolved: Pi `zai/glm-5.1` returned 429 insufficient balance, so Stage 1
  uses the validation Agent. The installed local Codex CLI hung even on
  `--version` while the desktop session was active, so it is not a supported
  Gateway backend yet. Navigation result strings remain weaker evidence than
  odometry.
- Next: start Stage 2 with two manually confirmed semantic places and
  `MissionExecutor` integration; do not start Stage 3.

### 2026-07-25 — Three-stage robot-verifiable Goal

- Changed: added
  `docs/plans/2026-07-25-go2-three-stage-robot-validation-plan.md` as the
  canonical executable plan and demoted the larger fusion master plan to
  long-term architecture/backlog.
- Why: the previous Goal combined too many capabilities and delayed physical
  feedback. The user requires each stage to end with a real robot review before
  more features are added.
- Stage 1: one Runtime, friend Agent validation profile, short relative motion,
  return-to-start, actual odometry path, and mid-motion stop.
- Stage 2: two manually confirmed semantic places, `MissionExecutor`,
  instruction/task binding, restart recovery, three short navigation trials,
  and one temporary-obstruction review.
- Stage 3: bounded exploration, candidate-stop, event-driven vision,
  camera/LiDAR/map grounding, one short stand-off approach, and ring-triggered
  duplicate-command verification.
- Deferred: person following, glasses hardware, general Mission Profiles,
  custom model training, and manipulation.
- Validation: documentation-only structure/path review and `git diff --check`;
  no source code, service, network, browser, or Go2 command changed.
- Next: execute `S1-T1 — 单一 Runtime 启动和诊断`; do not begin semantic or
  visual work before the Stage 1 robot gate passes.

### 2026-07-25 — Unified DimOS and friend-Agent master roadmap

- Changed: added
  `docs/plans/2026-07-25-go2-agent-fusion-master-plan.md` as the new canonical
  roadmap and marked the former semantic-autonomy master plan as superseded.
- Why: the previous roadmap correctly placed deterministic physical execution
  in DimOS but did not include the actual Pi Agent, Webhook Gateway, fixed MCP
  wrapper, instruction/task binding, or the friend MCP deployment entry.
- Architecture: the friend `components/dimos-mcp` becomes the only product
  composition/launch entry and loads the P1 modules; DimOS remains the only
  robot runtime; the Pi Agent compiles user language into task parameters and
  cannot declare physical completion.
- Plan content: four phases, cross-repository ownership, single-runtime
  topology, product versus maintenance MCP profiles, stable instruction/task
  contracts, exact implementation files, focused verification commands,
  per-task acceptance boundaries, phase gates, failure matrix, and evidence
  package requirements.
- Validation: documentation-only review against both repositories, the existing
  P1-T1 through P1-T3 implementations, friend MCP Blueprint, wrapper allowlist,
  Agent tool surface, Gateway persistence path, and current Git state. No source
  code, runtime process, network service, browser, or robot command changed.
- Unresolved: P1-T1 through P1-T3 are still local uncommitted work; the friend
  runtime does not yet load P1 modules; P1 semantic resolution remains
  fail-closed.
- Next: execute `P1-T4 — Persistent semantic place store`; do not begin the
  friend-Agent runtime fusion before its dependency gate.

### 2026-07-25 — P1-T3 bounded navigation recovery supervisor

- Changed: added a pure recovery policy and integrated distinct progress
  timeout, obstacle, path-deviation, and planner-error causes into the existing
  A* planner. Same-area recovery now alternates replans and rotate/rescan
  headings, exposes typed JSON events and `NavigationState.RECOVERY`, fails
  with `attempts_exhausted`, and discards an A* result computed across stop or
  goal replacement.
- Why: repeated replanning alone could retry the same geometry without
  refreshing perception, while a concurrent stop could previously race with a
  completed planning result.
- Files:
  `dimos/navigation/replanning_a_star/recovery_supervisor.py`,
  `test_recovery_supervisor.py`, `global_planner.py`, `module.py`,
  `docs/plans/2026-07-25-P1-T3-navigation-recovery-supervisor.md`, the master
  roadmap, and this document. Existing recovery tuning in `controllers.py`,
  `global_planner.py`, `replan_limiter.py`, and `test_recovery_tuning.py` was
  preserved.
- Commands/tests: recorded the initial missing-module failure; 21 focused
  navigation tests and an 86-test affected regression set passed; Ruff and
  `git diff --check` passed.
- Validation: the decision sequence is bounded; 75-degree errors still rotate
  without forward motion; rotate/rescan changes only the fresh path's first
  heading; cancellation clears recovery and stops the local planner; cancelled
  in-flight A* results cannot restart motion.
- Boundary: no runtime, replay, simulation, MCP network call, or live Go2
  movement was performed. Stale-local clearing, reverse, and alternate
  approach remain disabled until their exact prerequisite interfaces exist.
- Next: execute `P1-T4 — Persistent semantic place store`.

### 2026-07-25 — Native Viewer teleoperation startup repair

- Changed: made the App stop an existing registered DimOS run and the exact
  legacy `dimos_dog_mcp.blueprint` process group before every mode switch,
  wait for `7779/9990/3030/9877` to become free, explicitly enable
  `go2connection.movement_enabled=true` in control mode, and keep
  `auto_stand=false`. The double-click launcher now opens the App only instead
  of secretly starting a second read-only runtime. The UI now labels read-only
  and controllable modes unambiguously.
- Why: the legacy MCP held `7779/9990` while the Native App launched a different
  runtime with `movement_enabled=false`; Viewer keyboard commands reached
  `GO2Connection` but were rejected, and data/control surfaces came from two
  different process trees.
- Files: `apps/DimOS Native/main.swift`, rebuilt
  `apps/DimOS Native.app`, `apps/DimOS Native/test_native_launcher.py`,
  `scripts/start_dimos_go2.command`,
  `scripts/stop_dimos_native_conflicts.sh`, and this document.
- Commands/tests: Swift typecheck, Zsh syntax checks, three focused Pytest
  assertions, app build, strict codesign verification, plist validation, and
  `git diff --check` passed.
- Validation: the repaired App launched one read-only runtime connected to
  Go2 at `192.168.12.1`; native Viewer, command center (HTTP 200), control
  WebSocket, lidar/camera connection, 19 MCP tools, and battery telemetry (91%)
  were live. No browser page was opened and no movement was sent.
- Unresolved: the movement-enabled mode is implemented and compiled but was not
  physically exercised during this repair. A harmless Viewer-version warning remains:
  Viewer `0.32.0-alpha.1` versus SDK `0.32.0`.
- Next: click `连接并启用遥控` in the Native App, focus the Viewer, and perform one short
  operator-controlled movement; then stop and confirm the final pose.

### 2026-07-25 — Remove Native App confirmation phrase

- Changed: removed the `START GO2` field and validation from DimOS Native,
  renamed the control button to `连接并启用遥控`, and updated the double-click
  launcher instructions. Native startup still does not open Hosted Teleop or
  any browser control page.
- Why: the operator requested direct local control without a redundant
  confirmation phrase or official web controller.
- Files: `apps/DimOS Native/main.swift`, rebuilt
  `apps/DimOS Native.app`, `apps/DimOS Native/test_native_launcher.py`,
  `scripts/start_dimos_go2.command`, and this document.
- Validation: Swift typecheck, focused launcher tests, Zsh syntax, app build,
  codesign, plist validation, and live App inspection.
- Boundary: selecting the control button enables the local Viewer control
  channel but does not itself publish a movement command.

### 2026-07-25 — Small blocked-navigation recovery tuning

- Changed: reduced the existing rotate-before-drive threshold from 90 to 70
  degrees, reduced the position-stall replan window from 8 to 6 seconds, and
  increased the existing same-area replan limit from 6 to 8 attempts.
- Why: a blocked Go2 could continue pushing into a sharp bend or appear idle
  for too long before using DimOS's existing rotation and replanning behavior.
- Files: `dimos/navigation/replanning_a_star/controllers.py`,
  `global_planner.py`, `replan_limiter.py`,
  `test_recovery_tuning.py`, and this document.
- Commands/tests: focused recovery and global-planner pytest passed 5 tests;
  Ruff and `git diff --check` passed.
- Validation: a 75-degree heading error now produces zero linear velocity and
  positive angular velocity; a 45-degree correction still combines forward
  and angular motion; eight local replans remain available.
- Unresolved: the already-running Go2 process still has the previous constants
  loaded. No runtime restart or physical blocked-path test was performed.
- Next: restart once the operator is ready, use one supervised obstacle detour,
  and verify `initial_rotation`, `Robot is stuck. Replanning.`, and eventual
  goal completion in the live log before changing any costmap weights.

### 2026-07-25 — Three-phase semantic autonomy master roadmap

- Changed: added the canonical three-phase roadmap for semantic-place
  navigation, event-driven visual inspection/open-world search, and
  obstacle-aware person following with smart-ring command ingress.
- Why: replace several disconnected point plans with one dependency-ordered
  product plan that future Codex sessions can execute task by task.
- Files:
  `docs/plans/2026-07-25-go2-semantic-autonomy-master-plan.md` and this
  document.
- Plan content: architecture decisions, task IDs, exact expected source/test
  locations, Codex child-plan workflow, failure handling, upgrade decision
  gates, phase acceptance metrics, and the first allowed task (`P1-T1`).
- Commands/tests: documentation-only change; checked paths, repository status,
  existing plans, source boundaries, and planned test locations. No source
  tests or hardware commands were run.
- Validation: all three phases have explicit prerequisites and measurable
  gates; cloud vision remains outside local motion/stop/obstacle control; the
  existing DimOS runtime remains the sole robot runtime.
- Unresolved: task statuses remain `NOT_STARTED`; no implementation or physical
  behavior is claimed.
- Next: create the P1-T1 child plan and implement the canonical mission
  contracts with tests first.

### 2026-07-25 — P1-T1 canonical mission contracts

- Changed: implemented the canonical immutable Pydantic contracts for five
  mission kinds, priority, task IDs, UTC timestamps, bounded lifecycle
  transitions, pause/resume, terminal reasons, results, and evidence linkage.
  Studio mission HTTP DTOs now trim strings and reject unknown fields without
  importing the optional extension package.
- Why: invalid or incomplete physical tasks must be rejected before P1-T2 adds
  any executor or robot I/O.
- Files:
  `extensions/go2-studio-agent/src/dimos_go2_studio/mission_contracts.py`,
  `extensions/go2-studio-agent/tests/test_mission_contracts.py`,
  `dimos/web/studio/models.py`, `dimos/web/studio/test_mission.py`,
  `docs/plans/2026-07-25-P1-T1-canonical-mission-contracts.md`, the master
  roadmap, and this document.
- Commands/tests: 43 focused contract, mission, and Studio tests passed; Ruff
  and `git diff --check` passed.
- Validation: invalid mission field combinations, naive timestamps, illegal or
  post-terminal transitions, incomplete completion evidence, missing
  failure/cancellation reasons, and invalid pause state are rejected.
- Boundary: no runtime, MCP, navigation, vision, or real-Go2 behavior changed.
- Next: execute `P1-T2 — Background MissionExecutor lifecycle`; later tasks
  remain blocked by dependency order.

### 2026-07-25 — P1-T2 background mission executor

- Changed: added one background `MissionExecutor` with dependency-injected
  resolution/navigation, single-task mutual exclusion, typed lifecycle events,
  timeout, reached-plus-idle completion, pause/resume, idempotent cancellation,
  and MCP-facing start/pause/resume/cancel/status skills. The Blueprint now
  contains exactly one executor wired to `NavigationInterfaceSpec`.
- Why: mission goal acceptance, physical arrival, cancellation, and failure
  must be distinct before semantic resolution or autonomous recovery is added.
- Files:
  `extensions/go2-studio-agent/src/dimos_go2_studio/mission_executor.py`,
  its new tests, the extension Blueprint/skills, mission contract pause-cancel
  transition, `docs/plans/2026-07-25-P1-T2-background-mission-executor.md`,
  the master roadmap, and this document.
- Commands/tests: 57 contract, executor, Blueprint, mission, and Studio tests
  passed; Ruff and `git diff --check` passed.
- Validation: invalid JSON closes the background tool stream; default semantic
  resolution fails before a goal; concurrent movement tasks are rejected;
  `set_goal=True` is not completion; completion requires reached plus idle;
  timeout/cancel leave navigation idle; pause/resume reissues the retained goal.
- Boundary: no live runtime, MCP network call, replay, simulation, or real-Go2
  movement was performed. P1-T4 is still required before semantic destinations
  can resolve in production.
- Next: execute `P1-T3 — Navigation recovery supervisor`; P1-T4 and later work
  remain blocked by the current sequential roadmap.

### 2026-07-25 — Live MCP relative movement and duplicate-runtime recovery

- Changed: added the official `UnitreeSkillContainer` to the lightweight Go2
  Studio Blueprint so external MCP clients can call `relative_move`; added a
  Blueprint regression assertion and updated this context.
- Why: the connected runtime exposed camera, battery, exploration, and policy
  skills but no direct relative-distance movement tool.
- Files: `extensions/go2-studio-agent/src/dimos_go2_studio/blueprint.py`,
  `extensions/go2-studio-agent/tests/test_blueprint.py`, and this document.
- Runtime recovery: found two read-only DimOS parents plus orphaned forkserver
  workers and a duplicate Viewer. Stopped only those stale project processes,
  retained one native Viewer, and launched one movement-enabled foreground
  runtime without opening another browser page. A `--daemon` attempt was
  rejected after startup by a Zenoh `Bad file descriptor` panic in coordinator
  RPC setup, confirming the existing foreground-only macOS constraint.
- Commands/tests: focused Blueprint pytest passed (2 tests); Ruff and
  `git diff --check` passed. Live checks verified the Go2 signal port, WebRTC
  connection, battery, 19 registered MCP tools, one runtime, and one Viewer.
- Validation: MCP `relative_move(forward=1.0)` was accepted and caused physical
  motion. Odometry showed about 0.59 m before the planner declared arrival due
  to its tolerance. A second compensating `relative_move(forward=0.85)` brought
  total straight-line displacement to 1.131 m. Both calls ended with navigation
  idle and the robot stationary.
- Unresolved: the official relative-navigation skill is not a precise
  displacement primitive; MCP's default 30-second HTTP timeout can expire while
  a longer physical call continues server-side. Add a non-blocking,
  odometry-closed-loop distance skill or configurable arrival tolerance before
  promising exact metre-level motion.
- Next: expose a cancellable motion job with live odometry progress and stop at
  the requested distance, then validate repeatability over several 0.5 m and
  1.0 m trials.

### 2026-07-25 — Rollback, native macOS launcher, and first-party MCP

- Changed: preserved the newer work on backup branch
  `backup/pre-openai-rollback-20260725-0350` at `cbbeacfb4`, then moved the
  active branch to OpenAI offline-verifier commit `d52d8c851`. Removed the
  later continuous-frame Agent path from the active Go2 Blueprint.
- Why: restore the prior interface/behavior, stop continuous screenshot
  recognition, avoid browser WebGL load, and make real-Go2 startup usable from
  one macOS file.
- Files: `apps/DimOS Native/`, `apps/DimOS Native.app`,
  `scripts/build_dimos_native_app.sh`, `scripts/start_dimos_go2.command`,
  `启动 DimOS 机械狗.command`, `scripts/open_dimos_studio.command`,
  `dimos/web/studio/service.py`, the external Blueprint/tests, and this file.
- Runtime behavior: the native App and launcher start native Rerun, the
  original `7779` command-center service, lidar mapping, obstacle avoidance,
  and DimOS's built-in `McpServer`. Continuous `SpatialMemory`,
  `PerceiveLoopSkill`, and embedded `McpClient` are absent. Real-hardware
  one-click startup sets `movement_enabled=false` and `auto_stand=false`.
- Commands/tests: installed Homebrew `jpeg-turbo 3.2.0`; built and ad-hoc signed
  the arm64 AppKit bundle; `plutil` and `codesign --verify` passed; 15 focused
  pytest tests passed; Ruff, shell syntax, and `git diff --check` passed.
- Validation: replay registered 16 MCP tools, including `observe`,
  `begin_exploration`, `end_exploration`, patrol controls, battery, server
  status, and Studio policy skills. Ports `9877`, `9990`, `7779`, and `3030`
  listened and the native `dimos-viewer` launched. `observe` remained
  on-demand; no continuous recognition call was made.
- Current blocker: the Mac is on venue network `30.201.216.83`; the Go2 at
  `192.168.12.1:9991` is unreachable and LAN discovery is empty. The real
  launcher failed closed, so no fake connection or movement was reported.
- Next: connect the Mac to the Go2 network (or expose the Go2 on the same LAN),
  then double-click `启动 DimOS 机械狗.command`. The script will auto-discover,
  connect, open the native app/viewer, start MCP and mapping, while keeping
  physical movement locked.

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
