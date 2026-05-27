# SeatGuide Dog module split

This is the first demo-oriented module plan for the conference room seat-finding
hackathon idea. The goal is to keep each boundary testable without a Go2 while
the default Go2 path uses real browser/Whisper voice input, camera-backed VLM
seat/person recognition, robot navigation, and speech feedback.

## Demo-critical flow

User asks: "Find me an empty seat."

The system scans a conference room with one long table, detects chairs and
people, selects the nearest reachable empty chair, navigates beside it, and
speaks a short instruction.

## Modules

| Module | Owner boundary | Input | Output | Can build in parallel | Current status |
| --- | --- | --- | --- | --- | --- |
| Voice Command Intake | Converts speech or typed text into the `find_empty_seat` intent. | Browser microphone, web input, or agent text | SeatGuide request intent | Yes | `WebInput` routes SeatGuide voice/text directly to `handle_seat_request()` |
| Seat Perception | Detects chair poses and person positions in the conference room. | Go2 RGB camera frames/replay frames plus odometry | `SeatSceneObservation` in map frame | Yes | `CameraSeatObservationProvider` implemented with VLM detection, odom-backed map projection, and explicit calibrated fallback |
| Seat Occupancy Planner | Decides which chairs are empty and picks the nearest empty chair. | Chair poses, person positions, robot pose | Selected seat and guide pose | Yes | Implemented in `SeatGuidePlanner` |
| Guide Navigation | Sends the robot to a pose beside the selected chair and reports completion. | Guide pose in `map` frame | Navigation goal plus `goal_reached` status | Yes | Uses `NavigationInterfaceSpec` |
| Speech Feedback | Tells the user what was found and where to follow. | Planner result and navigation status | Spoken phrase | Yes | Optional `SpeakSkillSpec` integration implemented |
| Acceptance Harness | Runs the same flow without Go2 hardware. | Fixed synthetic or recorded image layout | Test result and expected goal pose | Yes | Covered by unit tests |

## Current software boundary

`dimos.agents.skills.seat_guide.SeatGuideSkillContainer` is the integration
module exposed to the agent. It supports direct debug input and provider-backed
real perception input.

Direct skill arguments:

- `seats`: flat `[x, y, yaw, x, y, yaw, ...]` chair poses in the `map` frame
- `people`: flat `[x, y, x, y, ...]` person positions in the `map` frame
- `robot_x`, `robot_y`: robot position for nearest-seat selection

Provider-backed scene:

- `SeatObservationProviderSpec.get_seat_scene()`
- `CameraSeatObservationProvider` subscribes to `color_image` and `odom`, asks the VLM for `chair` and `person` detections separately, and converts image-space detections to an approximate map-frame scene using the latest robot pose
- `camera_seat_provider_status()` reports camera frame, odometry, input freshness, VLM credential, runtime override, and fallback configuration readiness without running VLM detection
- `CameraSeatObservationProvider.set_seat_scene()` remains available as explicit runtime calibration/fallback when camera/VLM detection is unavailable
- `SyntheticSeatObservationProvider` remains for repeatable Go2-free tests and demos
- `unitree-go2-seat-guide` and `unitree-go2-seat-guide-agentic` include `CameraSeatObservationProvider` so the default SeatGuide bring-up path uses real camera recognition
- the default `qwen` VLM path requires `ALIBABA_API_KEY`; without it, SeatGuide reports `camera_detection_error` instead of silently treating missing credentials as a real no-seat observation

Voice/text intake:

- `parse_seat_guide_intent(text)` recognizes simple English and Chinese seat-finding requests
- `handle_seat_request(text)` rejects unrelated text or delegates to `find_empty_seat_from_scene()`; by default it only navigates from live `camera` perception
- `preview_seat_request(text)` validates a spoken/typed SeatGuide request and runs no-motion preflight instead of navigation
- `seat_guide_readiness_report()` runs scene status, live-perception preflight, and goal preview in one no-motion report
- `seat_guide_preflight()` checks navigation state, scene source, empty/occupied seat counts, selected seat/goal, and speaker wiring without moving; hardware acceptance requires `navigation=IDLE` and `speaker=connected`, it only passes live `camera` perception by default, and fallback calibration must be allowed explicitly with `require_live_perception=false`
- `seat_guide_status()` reports the current scene source, visible/configured seats, people, and robot pose without navigating
- `preview_empty_seat_goal()` runs the same planner and reports empty/occupied seat counts, the selected chair, and map-frame navigation goal without moving
- `seat_guide_navigation_status()` reports navigation state and `goal_reached` after a live SeatGuide request, so acceptance can prove the robot completed the task rather than only accepted a goal; if navigation already reported reached before the SeatGuide goal was sent, status waits to see `goal_reached=false` once before accepting a later `true`
- `WebInput` uses browser audio -> `WhisperNode` speech-to-text with language auto-detection; matching English or Chinese SeatGuide requests are routed directly to `handle_seat_request()` instead of waiting for the LLM to choose a tool, and the returned SeatGuide status is published to the web `agent_responses` text stream
- `web_input_status()` reports whether the browser voice/text entry point, browser audio upload endpoint, SeatGuide direct route, web response stream, STT pipeline, and `/human_input` fallback transport are connected
- unrelated text still goes through the normal `/human_input` agent path

Scene source values used by `seat_guide_status()`:

- `camera`: VLM detected at least one chair from the latest camera frame.
- `runtime_override`: operator-provided calibration from `set_seat_scene()`.
- `configured_fallback`: blueprint/static fallback scene.
- `no_camera_image`: no camera frame has arrived yet; check camera stream wiring.
- `camera_no_odom`: camera frames arrived but localization/odometry is missing; live navigation is no-go because SeatGuide cannot produce a trustworthy map-frame goal.
- `stale_camera_image`: camera frames are too old to prove live perception; restore the camera stream before live navigation.
- `stale_camera_odom`: odometry is too old to produce a trustworthy map-frame goal; restore localization before live navigation.
- `camera_no_seats_detected`: camera frames arrived, but VLM found no chairs; turn the robot toward the table or calibrate fallback.
- `camera_detection_error`: VLM detection raised an error; inspect logs/model setup or calibrate fallback.

When live navigation is requested from a non-`camera` source, SeatGuide refuses
to move and reports the source, seat/person counts, robot pose, and a specific
next step. This is intentional: fallback coordinates can be useful for
calibration, but they should not be mistaken for real chair/person recognition.

Speech feedback:

- if `SpeakSkill` is connected, SeatGuide speaks result/failure messages with `blocking=False`
- if no speaker is connected, SeatGuide still returns text and the core tests keep running
- `speech_status()` reports whether OpenAI TTS and local audio output are ready without speaking

This keeps perception independent from navigation. A later detector can return
the same scene contract without changing the planner tests.

## Unit-test acceptance

The Go2-free acceptance path is:

1. Build a synthetic long-table room layout.
2. Mark chairs occupied when a person is within 0.75 meters.
3. Select the nearest empty chair to the robot.
4. Compute a navigation pose beside the chair.
5. Verify a fake navigator receives the expected `PoseStamped`.
6. Verify `SyntheticSeatObservationProvider` can feed the same flow without
   direct skill arguments.
7. Verify text requests such as "Please find me an empty seat" and "帮我找一个空位"
   route into the same provider-backed flow.
8. Verify no-motion text requests such as "预检帮我找一个空位" route to `preview_seat_request()` and do not call navigation.
9. Verify `CameraSeatObservationProvider` converts camera/VLM chair/person detections into a map-frame `SeatSceneObservation` using latest odometry when available.
10. Verify `WebInput` creates Whisper without forcing English, so Chinese phrases such as "帮我找一个空位" can be transcribed by language auto-detection.
11. Verify `seat_guide_status()` can diagnose whether the scene came from camera detection, runtime calibration, or configured fallback before navigation is attempted.
12. Verify `seat_guide_preflight()` reports navigation/perception/speaker readiness without calling navigation, and that fallback scenes are no-go unless explicitly allowed.
13. Verify `seat_guide_readiness_report()` combines status, preflight, and preview without calling navigation.
14. Verify `handle_seat_request()` refuses fallback scenes by default and only navigates with fallback when `require_live_perception=false` is explicitly passed.
15. Verify `preview_empty_seat_goal()` reports the selected chair and map-frame goal without calling navigation.
16. Verify `seat_guide_navigation_status()` reports `goal_reached=true/false` and missing navigation without sending or canceling a goal.
17. Verify the SeatGuide MCP JSON-RPC path can list `seat_guide_status`, run preflight/readiness, preview the goal, call `handle_seat_request` with Chinese text, and report `goal_reached=true` without Go2 hardware.

Run:

```bash
uv run pytest dimos/agents/skills/test_seat_guide.py
```

## Simulation acceptance target

If a simulator is available, use the same synthetic or camera-derived layout as the unit test and
run the Go2 agentic stack in replay or sim mode. The minimum sim/replay acceptance is
that browser/Whisper text reaches SeatGuide, `CameraSeatObservationProvider`
returns a scene from camera frames or an explicitly calibrated fallback, and navigation receives
a goal.

Current replay evidence:

- `dimos --replay run unitree-go2-seat-guide-agentic` starts with MCP tools exposed.
- `dimos mcp list-tools` includes `set_seat_scene`, `find_empty_seat_from_scene`, `preview_seat_request`, `seat_guide_readiness_report`, `seat_guide_preflight`, `seat_guide_navigation_status`, `preview_empty_seat_goal`, `handle_seat_request`, `seat_guide_status`, `server_status`, and `list_modules`.
- `web_input_status` reports `seat_route=seat_guide_direct`, `voice_upload=connected`, and `stt=connected` when browser/Whisper input is wired directly to SeatGuide.
- `camera_seat_provider_status` reports whether Go2 camera frames, odometry, input freshness, and VLM credentials are ready before detection is attempted.
- Calling `set_seat_scene` followed by `handle_seat_request` returned: `I found an empty seat seat_3. Please follow me to the chair beside the table. Navigating to (3.65, -1.00).`
- Posting typed text to `http://localhost:5555/submit_query` with `query=帮我找一个空位` returned success and triggered the same SeatGuide navigation path.
- In replay, the configured fallback goal produced a planner warning `No path found to the goal`; this confirms a navigation goal was sent, but not that the fallback coordinates are reachable in the replay map.

## Tomorrow G2 bring-up path

The intended real-perception bring-up path is:

1. Place chairs in the Go2 camera view.
2. Start `unitree-go2-seat-guide-agentic` with the normal MCP server.
3. Run `seat_guide_status`, `seat_guide_preflight`, and `preview_empty_seat_goal` before any live motion.
4. If preflight reports `navigation=FOLLOWING_PATH` or `navigation=RECOVERY`, wait for the current action to finish or cancel it before asking SeatGuide to send a new goal.
5. Optionally call `set_seat_scene()` only if camera/VLM detection is unavailable or needs explicit fallback calibration.
6. Send or speak: "Please find me an empty seat" / "帮我找一个空位".
7. `WebInput` transcribes browser microphone audio with Whisper language auto-detection and routes matching text to `handle_seat_request(text)`.
8. `handle_seat_request()` parses the intent, reads the camera-backed scene, picks
   the nearest empty chair, and calls `NavigationInterfaceSpec.set_goal()`.

`set_seat_scene()` is now an explicit fallback/calibration tool, not the primary path.

### Parallel hardware-day checklist

Use this split when the Go2 is connected so the team can debug independently
before any live motion:

| Track | Owner checks | Passing evidence | No-go action |
| --- | --- | --- | --- |
| Voice intake | Browser page opens; microphone permission granted; Chinese preview phrase reaches WebInput. | `web_input_status` shows `web=started`, `thread=running`, `seat_route=seat_guide_direct`, `responses=connected`, `voice_upload=connected`, `stt=connected`, `human_transport=connected`; acceptance log shows `WebInput received text` for `预检帮我找一个空位`. | Fix browser/microphone/WebInput before touching navigation. |
| Perception | Go2 camera frame, odometry, and Qwen credential are live; no fallback scene is active. | `camera_seat_provider_status` shows `image=<width>x<height>`, `image_fresh=true`, `odom=(...)`, `odom_fresh=true`, `credential=present`, `override=inactive`, `configured_fallback_seats=0`, `configured_fallback_people=0`; `seat_guide_status` starts with `SeatGuide scene source=camera:`. | Turn robot toward the table, fix `ALIBABA_API_KEY`, restore stale camera/odom streams, or explicitly mark fallback calibration as non-acceptance. |
| Planner | Empty/occupied counts and selected goal make sense before motion. | `seat_guide_preflight`, `seat_guide_readiness_report`, and `preview_empty_seat_goal` report `empty=N occupied=N`, `selected=...`, and `goal=(...)` without sending a goal. | Adjust camera view or chair/person layout before live voice. |
| Navigation | Robot is idle before SeatGuide sends the live goal and reports completion after it. | Preflight has `navigation=IDLE`; after live voice, `seat_guide_navigation_status` reports a new `goal_sequence` and `goal_reached=true`. | Wait/cancel existing navigation or inspect navigation logs; do not rerun live voice until idle. |
| Speech feedback | TTS and local audio output are ready, or the team agrees to use web text as the user-facing fallback. | `speech_status` shows `tts=ready` and `audio_output=connected`; hardware acceptance also calls `speak` with `SeatGuide audio check. I can guide you to an empty seat.`, requires `Spoke: SeatGuide audio check`, and requires operator confirmation `HEARD`. | Set `OPENAI_API_KEY` and connect audio before official acceptance. |
| Acceptance evidence | The run is hardware, not replay/sim, and uses the SeatGuide blueprint. | `bin/demo_seat_guide_hardware_acceptance` records the run registry, no-motion gates, browser microphone gates, camera source, speech output check plus operator heard confirmation, ordered WebInput route logs, and `goal_reached=true`; `bin/demo_seat_guide_verify_acceptance_log <log>` passes. | Treat failures as real no-go evidence; do not replace them with direct MCP live calls. |

### Bring-up commands

One-command real Go2 bring-up:

```bash
export ALIBABA_API_KEY=...
export OPENAI_API_KEY=...
bin/demo_seat_guide_hardware_bringup --robot-ip 192.168.123.161
```

This starts `unitree-go2-seat-guide-agentic`, runs the no-motion smoke checks,
then launches the hardware acceptance flow. Use `--skip-start` if the stack is
already running, or `--skip-smoke` only when repeating acceptance after a known
passing smoke run.

Start the SeatGuide-focused Go2 stack. This keeps the real Go2 base,
navigation, camera, browser/Whisper voice input, MCP server, and SeatGuide
modules, while avoiding unrelated CUDA-only security demo modules:

```bash
dimos run unitree-go2-seat-guide-agentic --robot-ip 192.168.123.161 --daemon
```

For local replay without a real robot:

```bash
dimos --replay run unitree-go2-seat-guide-agentic --daemon
```

On macOS, DimOS replay and MCP require multicast on loopback for cross-process
LCM/RPC. If startup asks for this route and cannot run sudo non-interactively,
run it once in a terminal before replay or hardware bring-up:

```bash
sudo route delete -net 224.0.0.0/4 || true
sudo route add -net 224.0.0.0/4 -interface lo0
```

Do not use `PYTEST_VERSION=1` as a runtime workaround. It skips the system
configurator, but `McpServer/on_system_modules` can time out because the
cross-process LCM route is still missing.

The general Go2 agentic stack is still available, but it includes unrelated
demo modules and is not the SeatGuide acceptance target. The hardware
acceptance script intentionally rejects that general stack; start
`unitree-go2-seat-guide` or `unitree-go2-seat-guide-agentic` for SeatGuide
acceptance.

If `OPENAI_API_KEY` is not set, `McpClient` disables the LLM agent but the
direct voice route and MCP tools still start. `SpeakSkill` also degrades to a
no-op instead of failing startup.

The default camera detector uses Qwen VLM. Set `ALIBABA_API_KEY` before hardware
bring-up, or configure a different supported `detection_model`. If the key is
missing, `seat_guide_status` reports `source=camera_detection_error`; use logs
or `set_seat_scene` only as an explicit fallback/calibration path.

Confirm the SeatGuide tools are exposed:

```bash
dimos mcp list-tools
dimos mcp modules
```

Run the no-motion smoke script against the already-running stack. It checks the
SeatGuide MCP tools and fails early unless `web_input_status` reports
`web=started`, `thread=running`, `seat_route=seat_guide_direct`,
`responses=connected`, `voice_upload=connected`, `stt=connected`, and `human_transport=connected`, then
runs the no-motion scene/status/preflight/preview checks:

```bash
bin/demo_seat_guide_smoke
```

Run the hardware acceptance script against an already-running real Go2 stack.
It performs the no-motion WebInput, camera/VLM, scene, preflight, request
preview, and goal preview checks first. It only offers the `LIVE` prompt after
automated gates pass: the DimOS run registry must show a hardware run, not
`--replay` or `--simulation`, and the blueprint must be
`unitree-go2-seat-guide` or `unitree-go2-seat-guide-agentic`; WebInput must report `web=started`,
`thread=running`, `seat_route=seat_guide_direct`, `responses=connected`,
`voice_upload=connected`, `stt=connected`, and
`human_transport=connected`; camera frames, odometry, and VLM credentials must
be present, the camera provider runtime override must be inactive, and
configured fallback seats/people must both be zero; TTS/audio output must be ready; preflight must be ready with
`navigation=IDLE` and `speaker=connected`; the goal preview must select a seat;
and the script must resolve the active WebInput URL from `web_input_status`. Posting
`预检帮我找一个空位` to that WebInput HTTP text
endpoint must publish a `SeatGuide preflight ready` response on the web
`agent_responses` stream before `SEAT_GUIDE_WEBINPUT_TEXT_WAIT_S` seconds
(default `20`). It then opens a manual no-motion voice gate: the
operator presses Enter when ready, then uses the browser microphone to say
`预检帮我找一个空位`; the script only accepts a `SeatGuide preflight ready` response
that appears in the web response stream after that readiness point, before
`SEAT_GUIDE_WEBINPUT_VOICE_PREVIEW_WAIT_S` seconds (default `120`). After the
operator types `LIVE`, the script still does not call
`handle_seat_request` through MCP; it opens a live voice gate and requires the
operator to press Enter when ready, then say `帮我找一个空位` through the browser
microphone. The live gate passes only if the web response stream reports
`Navigating to` after that readiness point and before
`SEAT_GUIDE_WEBINPUT_VOICE_LIVE_WAIT_S` seconds (default `150`); after that, the
script polls `seat_guide_navigation_status()` until it reports both a new
`goal_sequence` and `goal_reached=true`; stale completion from a previous
navigation goal is suppressed until SeatGuide observes a reset. The transcript is saved under
`logs/seat_guide_acceptance/` by default; override with
`SEAT_GUIDE_ACCEPTANCE_LOG_DIR=/path/to/logs` when needed. The transcript
includes the MCP command outputs plus `dimos log -n 200` snapshots after the
no-motion checks, after the live navigation request, and on WebInput/navigation
failure paths. The saved logs must show `WebInput routing text to SeatGuide
preview` for the no-motion voice gate and `WebInput routing text to SeatGuide
live request` for the live gate, proving the request went through WebInput
rather than a direct MCP call:

```bash
bin/demo_seat_guide_hardware_acceptance
```

The hardware script automatically audits the saved transcript after the live
request completes. It requires the DimOS run registry path, hardware run mode,
SeatGuide Go2 blueprint name, running WebInput server/thread/transport, direct
SeatGuide routing, resolved WebInput URL, camera/odometry/VLM readiness,
`image_fresh=true` and `odom_fresh=true`, TTS audio check phrase, `Spoke:`
completion, operator audio confirmation `HEARD`, typed and spoken no-motion
responses, explicit browser microphone no-motion/live gates with the required
spoken phrases, WebInput preview/live route logs, empty/occupied seat counts,
DimOS log snapshots after no-motion checks and after the live request, the
no-motion completion marker, `LIVE` confirmation, live voice navigation start,
goal-sequence polling, and
`goal_reached=true` completion. It also requires at least three
`WebInput received text` log events,
covering typed no-motion input, no-motion browser microphone input, and live
browser microphone input; those log events must include the recognized
SeatGuide phrases `预检帮我找一个空位` and `帮我找一个空位`, not just arbitrary
WebInput text. The verifier rejects transcripts that contain direct MCP live
calls to `handle_seat_request`, fallback scene calibration with
`set_seat_scene`, clearing fallback overrides, or
`require_live_perception=false`. The no-motion flow must appear in order: typed
WebInput preview, browser microphone no-motion gate, readiness prompt before
the spoken phrase, WebInput preview route, no-motion log snapshot, no-motion
completion marker, and `LIVE` confirmation. The live flow must then appear in
order: `LIVE` confirmation, browser microphone live gate, readiness prompt
before the spoken phrase, WebInput live SeatGuide route, navigation start,
`goal_reached=true`, and completion marker.

To re-audit an existing transcript manually, run:

```bash
bin/demo_seat_guide_verify_acceptance_log logs/seat_guide_acceptance/<timestamp>.log
```

Run the replay smoke wrapper when no Go2 is connected. It checks the macOS
multicast route before starting replay, starts `unitree-go2-seat-guide-agentic`
with `--replay`, runs the no-motion smoke, and stops the stack:

```bash
bin/demo_seat_guide_replay_smoke
```

Run the no-motion readiness path without relying on microphone or LLM behavior:

```bash
dimos mcp call seat_guide_status
dimos mcp call web_input_status
dimos mcp call camera_seat_provider_status
dimos mcp call speech_status
dimos mcp call seat_guide_readiness_report
dimos mcp call seat_guide_preflight
dimos mcp call seat_guide_navigation_status
dimos mcp call preview_seat_request --json-args '{"text": "预检帮我找一个空位"}'
dimos mcp call preview_empty_seat_goal
```

Do not use a direct MCP `handle_seat_request` call as the live hardware demo
evidence. The hardware acceptance verifier rejects that path because it bypasses
the required browser microphone -> Whisper -> WebInput route.

Run the real voice path:

1. Open the web interface printed by `WebInput` (`http://localhost:5555` by default).
2. Allow browser microphone access.
3. First speak "预检帮我找一个空位" or "preview find me an empty seat" to validate the real microphone path without motion.
4. Then speak "帮我找一个空位" or "Please find me an empty seat" when live navigation is intended.
5. The browser audio is transcribed by `WhisperNode` with language auto-detection; no-motion preview text is routed to `preview_seat_request()`, and live SeatGuide text is routed directly to `handle_seat_request()`.
6. Watch the web `agent_responses` stream for the exact SeatGuide result, especially if `SpeakSkill` is unavailable or audio output is hard to hear during bring-up.

If MCP is healthy, this should route through:

`seat_guide_readiness_report` for combined no-motion checks ->
`WebInput` -> `WhisperNode` -> `handle_seat_request` ->
`CameraSeatObservationProvider.get_seat_scene` using camera frames and odom ->
`SeatGuidePlanner.find_empty_seat` -> `NavigationInterfaceSpec.set_goal` ->
optional `SpeakSkillSpec.speak`.

Calibrate the fallback scene at runtime if camera/VLM detection is not reliable:

```bash
dimos mcp call set_seat_scene --json-args '{"seats": [0.0, -1.0, 0.0, 1.5, -1.0, 0.0, 3.0, -1.0, 0.0], "people": [0.1, -1.0, 1.6, -1.0], "robot_x": -1.0, "robot_y": -1.0}'
```

After fallback calibration, use explicit fallback preflight:

```bash
dimos mcp call seat_guide_preflight --json-args '{"require_live_perception": false}'
```

To intentionally test fallback navigation without live camera recognition, also
pass the override on the request itself:

```bash
dimos mcp call handle_seat_request --json-args '{"text": "帮我找一个空位", "require_live_perception": false}'
```

For the real G2 demo, use map-frame chair and aisle coordinates that are
reachable by the active map. A successful `handle_seat_request` response only
proves the goal was submitted; confirm `dimos log -f` does not show `No path
found to the goal` before relying on that calibration.

Clear runtime calibration and return to blueprint defaults:

```bash
dimos mcp call clear_seat_scene_override
```

Use the direct skill only for debugging the configured coordinates:

```bash
dimos mcp call find_empty_seat --json-args '{"seats": [0.0, -1.0, 0.0, 1.5, -1.0, 0.0, 3.0, -1.0, 0.0], "people": [0.1, -1.0, 1.6, -1.0], "robot_x": -1.0, "robot_y": -1.0}'
```

Stop after testing:

```bash
dimos stop
```
