# SeatGuide Robot Dog Empty-Seat Guidance Step-by-Step Plan

Goal: let a user tell the Go2, through browser microphone or typed text, "find me an empty seat." The system should use the real camera to recognize chairs and people, decide which seats are empty, send a navigation goal, and respond through the web or phone relay. When the Go2 is not connected, every locally verifiable module must have unit or smoke coverage. After the Go2 is connected, we should only need to run the hardware acceptance flow instead of assembling functionality on the spot.

## Overall Module Split

| Module | Responsibility | Input | Output | Can run in parallel | Current verification |
| --- | --- | --- | --- | --- | --- |
| 1. Basic voice/text control entry | Accept browser microphone, browser text, or normal agent text; first recognize basic movement/posture commands, then recognize seat-finding intent | WebInput `/submit_query`, `/upload_audio`, Whisper text, agent text | Normal agent tool call, or SeatGuide preview/live request | Yes | MCP tool acceptance, WebInput unit tests, HTTP TestClient, hardware acceptance script |
| 2. Scene perception | Use Go2 RGB image + odom + YOLO fast detection for chairs and people, with optional VLM fallback, then project them into map coordinates | `color_image`, `odom`, YOLO `yolo11n.pt` | `SeatSceneObservation` | Yes | Camera provider unit tests, `camera_seat_provider_status` |
| 3. Empty-seat planning | Decide which chairs are occupied, select the nearest empty seat, and generate the guide pose for the robot | Chair poses, person positions, robot position | Selected chair and navigation goal pose | Yes | Planner unit tests, `preview_empty_seat_goal` |
| 4. Navigation execution | Send the target pose to the existing navigation module and read completion status | SeatGuide goal pose | `set_goal()`, `goal_reached` | Partially | Fake navigator unit tests, `seat_guide_navigation_status` |
| 5. Phone/web feedback | Tell the user which seat was found, whether to follow, or why the request failed | SeatGuide result text | Web response text, phone speaker relay | Yes | `web_input_status`, optional `phone_speaker_test` |
| 6. Acceptance scripts | Chain no-motion checks, real voice input, and real navigation, then save a transcript | Current DimOS stack | Pass/fail reason and acceptance log | Yes | `bin/demo_seat_guide_*` |

## Stage 1: Basic Voice-Control Acceptance

Purpose: first prove the smallest working loop: "a person says or types a command -> the system recognizes the intent -> the command reaches the Go2 -> the Go2 executes it." This stage does not find seats, does not depend on VLM, and does not depend on chair detection.

Work items:

1. Call posture and movement tools directly through MCP to prove the Go2 control tools work.
2. Type normal movement commands in the browser to prove text enters the agent and triggers a Go2 tool.
3. Speak normal movement commands through the browser microphone to prove microphone -> Whisper -> agent -> Go2 tool works.
4. Verify stop/safety commands so every small movement can be stopped.
5. Accept only low-risk actions: stand, recovery stand, short forward/backward movement, and small turns. Do not test jumps, flips, or other dynamic motions in this first stage.

### Stage 1 Acceptance Path Breakdown

| Path | Entry point | Will it move the Go2? | Verification command/action | Pass criteria |
| --- | --- | --- | --- | --- |
| 1A. MCP posture command | MCP tool | May change posture, no walking | `dimos mcp call execute_sport_command --json-args '{"command_name":"BalanceStand"}'` | The tool returns success and the Go2 enters a stable standing/balancing state |
| 1B. MCP short movement | MCP tool | Yes, short distance | Run `relative_move` forward 0.3m, backward 0.3m, and turn left 30 degrees | The Go2 makes the small movement, or navigation status reports the goal completed |
| 1C. Browser text -> agent -> Go2 tool | Web page text box or `/submit_query` | Yes, short distance | Type `walk forward 30 centimeters` and `walk backward 30 centimeters` | Logs show WebInput received the text, non-seat text went to the normal agent path, and the agent called `relative_move` |
| 1D. Browser microphone -> Whisper -> agent -> Go2 tool | Computer browser microphone | Yes, short distance | Say `walk forward 30 centimeters`, or the equivalent Chinese command, into the browser | Logs show Whisper recognized the text, the agent called the matching Go2 tool, and the Go2 executed it |
| 1E. Stop/safety | MCP tool or agent tool | Stops current navigation/action | `dimos mcp call stop_navigation` | Navigation returns to stopped/idle and the robot does not continue moving |

Recommended acceptance commands:

```bash
dimos mcp call execute_sport_command --json-args '{"command_name":"BalanceStand"}'
dimos mcp call relative_move --json-args '{"forward":0.3,"left":0,"degrees":0}'
dimos mcp call relative_move --json-args '{"forward":-0.3,"left":0,"degrees":0}'
dimos mcp call relative_move --json-args '{"forward":0,"left":0,"degrees":30}'
dimos mcp call stop_navigation
```

Pass criteria:

- Direct MCP calls can make the Go2 execute posture and short movement commands.
- Browser text commands trigger normal agent tools instead of incorrectly entering SeatGuide.
- Browser microphone commands are transcribed and trigger the same Go2 tool.
- If any action fails, the failure can be attributed to one layer: control tool, agent tool selection, Whisper, WebInput, or Go2 connection.
- The current default input device is the **computer browser microphone**, not the Go2 body microphone. Using the Go2 onboard microphone would require a separate input module later.

## Stage 2: SeatGuide Core Module Development And Local Unit Tests

Purpose: prove the core logic without connecting the robot dog.

Work items:

1. Implement SeatGuide data models: chairs, people, scene, planner result, and voice intent.
2. Implement occupancy detection: a chair is occupied if a person is within 0.75 meters.
3. Implement nearest-empty-seat selection from the robot's current position.
4. Implement guide-pose generation beside the chair in the aisle direction, not at the chair center.
5. Implement preview and live paths: preview never moves; live sends the navigation goal.

Verification:

```bash
uv run pytest dimos/agents/skills/test_seat_guide.py -q -k 'planner or find_empty_seat or preview_empty_seat_goal'
```

Pass criteria:

- The correct empty seat is selected.
- Occupied chairs are not selected.
- Preview does not call navigation.
- Live only calls navigation when navigation is available and the scene source is trusted.

## Stage 3: SeatGuide Voice And WebInput Path

Purpose: allow the user to trigger SeatGuide through the browser text box or microphone instead of requiring a manual MCP call.

Work items:

1. Route WebInput text input from `/submit_query` directly to SeatGuide requests.
2. Push browser audio uploads from `/upload_audio` into `audio_subject`.
3. Let Whisper auto-detect language instead of forcing English.
4. Chinese preview phrase: `预检帮我找一个空位` only runs no-motion checks.
5. Chinese live phrase: `帮我找一个空位` triggers navigation.
6. Publish the SeatGuide response to `agent_responses` so it is visible in the browser.

### Stage 3 Acceptance Path Breakdown

| Path | Entry point | Will it move the Go2? | Verification command/action | Pass criteria |
| --- | --- | --- | --- | --- |
| 3A. Module status check | MCP tool | No | `dimos mcp call web_input_status` | Output includes `web=started`, `voice_upload=connected`, `stt=connected`, and `seat_route=seat_guide_direct` |
| 3B. Browser text preview | Web page text box or `/submit_query` | No | Type `预检帮我找一个空位` in the WebInput page, or let the hardware script POST it automatically | `agent_responses` shows `SeatGuide preflight ready` or a clear no-go reason; no navigation goal is sent |
| 3C. Browser microphone preview | Computer browser microphone | No | Open the WebInput URL, allow microphone access, click the microphone button, and say `预检帮我找一个空位` | After Whisper recognition, DimOS logs include `WebInput received text` and `WebInput routing text to SeatGuide preview` |
| 3D. Normal agent text fallback | `/human_input` agent path | No, unless the agent later explicitly calls a tool | Enter non-seat text such as `what time is the meeting` | WebInput does not call SeatGuide; the text goes to the normal agent path |
| 3E. Browser microphone live | Computer browser microphone | Yes | Only after no-motion checks pass and the physical area is safe, say `帮我找一个空位` | Logs include `WebInput routing text to SeatGuide live request`, and SeatGuide returns `Navigating to ...` |

Recommended acceptance order:

1. Run 3A first to confirm WebInput, STT, and browser audio upload are online.
2. Run 3B next to confirm browser text input routes directly to SeatGuide preview.
3. Run 3C next to confirm computer browser microphone -> Whisper -> SeatGuide preview.
4. Run 3E last because it sends a real navigation goal.

Note: the current default path uses the **computer browser microphone**, not the Go2 body microphone. The user speaks to the computer browser, the computer uploads audio to DimOS, Whisper transcribes it, and then SeatGuide sends navigation to the Go2.

Verification:

```bash
uv run pytest dimos/agents/skills/test_seat_guide.py -q -k 'web_input or upload_audio or submit_query'
```

Pass criteria:

- `/submit_query` can trigger SeatGuide preview.
- `/upload_audio` can produce an `AudioEvent`.
- Missing voice input configuration or audio decode failure cannot be reported as success.
- `web_input_status` must include:
  - `web=started`
  - `thread=running`
  - `seat_route=seat_guide_direct`
  - `responses=connected`
  - `voice_upload=connected`
  - `stt=connected`
  - `human_transport=connected`

## Stage 4: Real Camera/VLM/Odom Perception

Purpose: the final result must not rely on fake mocks. Hardware acceptance must prove the scene came from the real camera source.

Work items:

1. `CameraSeatObservationProvider` subscribes to `color_image` and `odom`.
2. Use Qwen/VLM to detect `chair` and `person` separately.
3. Estimate map-frame chair/person positions from the bounding-box center and odometry.
4. Diagnose no-go states such as stale image, stale odom, missing key, and missing camera.
5. Keep `set_seat_scene` as fallback/calibration, but reject fallback for official hardware acceptance.

Verification:

```bash
uv run pytest dimos/agents/skills/test_seat_guide.py -q -k 'camera_observation_provider or camera_seat_provider_status'
```

Pre-hardware checks:

```bash
dimos mcp call camera_seat_provider_status
dimos mcp call seat_guide_status
```

Pass criteria:

- `camera_seat_provider_status` shows:
  - `image=<width>x<height>`
  - `image_fresh=true`
  - `odom=(...)`
  - `odom_fresh=true`
  - `credential=present`
  - `override=inactive`
  - `configured_fallback_seats=0`
  - `configured_fallback_people=0`
- `seat_guide_status` must start with `SeatGuide scene source=camera:`.

## Stage 5: Navigation And Phone/Web Feedback

Purpose: after finding an empty seat, the Go2 must actually receive a navigation goal and provide visible feedback through the web, or audible feedback through a phone mounted on the robot.

Work items:

1. Inject `NavigationInterfaceSpec` into SeatGuide.
2. On a live request, call `set_goal(PoseStamped)`.
3. If navigation is busy, refuse to overwrite the current task.
4. Read `navigation_state` and `goal_reached`.
5. Return clear SeatGuide result text and publish it to the WebInput `agent_responses` stream.
6. If audible feedback is required, open the speaker relay on the mounted phone and verify it with `phone_speaker_test`.

Verification:

```bash
uv run pytest dimos/agents/skills/test_seat_guide.py -q
```

Pre-hardware checks:

```bash
dimos mcp call seat_guide_preflight
dimos mcp call preview_empty_seat_goal
dimos mcp call seat_guide_navigation_status
```

Pass criteria:

- Preflight shows `navigation=IDLE`.
- Preview includes `selected=...` and `goal=(...)`.
- The WebInput response stream is visible; if sound is needed, the phone relay can play a test message.
- After live navigation, `seat_guide_navigation_status` eventually shows a new `goal_sequence` and `goal_reached=true`.

## Stage 6: Mac Replay / SHM Video Stream Fix

Purpose: on macOS replay, high-bandwidth video/pointcloud/map streams should not disappear because of UDP/LCM large-packet issues.

Work items:

1. Integrate `dimensionalOS/dimos#2245` / `danvi/experimental/route-replay-through-SHM`.
2. Route high-bandwidth Go2 replay streams through `pSHMTransport`:
   - `color_image`
   - `lidar`
   - `pointcloud`
   - `global_map`
   - `merged_map`
   - `global_costmap`
   - `navigation_costmap`
3. Let the Rerun bridge receive SHM visual transports.

Verification:

```bash
uv run pytest dimos/protocol/pubsub/test_registry.py dimos/visualization/rerun/test_viewer_integration.py -q
bin/demo_seat_guide_replay_smoke
```

Pass criteria:

- The replay stack starts.
- Logs show high-bandwidth streams with `transport=pSHMTransport`.
- `bin/demo_seat_guide_replay_smoke` completes and stops the stack.

## Stage 7: Real Go2 Bring-Up

Purpose: after connecting the robot dog, you should run one entry point instead of manually composing commands.

Manual preparation:

1. Power on the robot dog and connect it to the same network as the Mac.
2. Confirm the Go2 IP. The default example is `192.168.123.161`.
3. Optionally prepare an API key for the normal agent. The SeatGuide direct voice/MCP path does not need an LLM key; seat/person VLM still requires Alibaba/Qwen when Qwen is selected:

```bash
export OPENROUTER_API_KEY="your OpenRouter key"
export OPENROUTER_MODEL="openai/gpt-4o-mini"
```

Start one-command bring-up:

```bash
bin/demo_seat_guide_hardware_bringup --robot-ip 192.168.123.161
```

The script automatically:

1. Checks the YOLO fast detection path; if no agent key is set, normal agent chat is disabled but the direct SeatGuide route still works.
2. Starts `unitree-go2-seat-guide-agentic`.
3. Runs `bin/demo_seat_guide_smoke` for no-motion checks.
4. Runs `bin/demo_seat_guide_hardware_acceptance` for real browser voice input and navigation acceptance.

Manual actions during the script:

1. Open the WebInput URL printed by the script.
2. Allow browser microphone access.
3. During the no-motion stage, say into the browser: `预检帮我找一个空位`.
4. After confirming the area around the Go2 is physically safe, type `LIVE` in the terminal.
5. During the live stage, say into the browser: `帮我找一个空位`.

Pass criteria:

- All no-motion gates pass.
- The live stage WebInput logs include the recognized Chinese text.
- SeatGuide returns `Navigating to ...`.
- `seat_guide_navigation_status` eventually shows a new `goal_sequence` and `goal_reached=true`.
- `bin/demo_seat_guide_verify_acceptance_log <log>` passes.

## Stage 8: Module-Level Debugging When Something Fails

| Failure point | Command to inspect | Common cause | Fix |
| --- | --- | --- | --- |
| WebInput is not started | `dimos mcp call web_input_status` | Port conflict or WebInput module did not start | Check `dimos status`, restart the stack |
| Microphone input does not arrive | `web_input_status`, browser permissions | `voice_upload=missing`, browser denied microphone access | Allow microphone permission, refresh the WebInput page |
| STT is not working | `web_input_status` | Whisper/faster-whisper initialization failed | Inspect DimOS logs and confirm dependencies |
| No image | `camera_seat_provider_status` | Go2 camera/replay stream did not arrive | Turn toward the table, confirm replay/SHM stream |
| Odom missing or stale | `camera_seat_provider_status` | Localization did not start or stale odom | Wait for odom, inspect Go2/replay stack |
| YOLO/VLM failed | `seat_guide_status` | YOLO model load failure, or missing remote VLM key after enabling VLM fallback | Confirm `yolo11n.pt` can load; if using Qwen fallback, re-export the matching key, then restart the stack |
| No chairs found | `seat_guide_status` | Camera not facing the table, lighting or recognition issue | Adjust robot view; use fallback only for debugging |
| Navigation busy | `seat_guide_preflight` | `navigation=FOLLOWING_PATH` or `RECOVERY` | Wait for the task to finish or stop navigation before retrying |
| Phone feedback unavailable | `web_input_status` / `phone_speaker_test` | Phone has not opened the relay page or the relay is unreachable | First confirm the web response stream; if sound is needed, open a reachable relay page on the phone |

## Current Completion Status

Completed:

- SeatGuide planner / scene / intent / navigation integration.
- WebInput Chinese voice and text directly routed to SeatGuide.
- Camera/VLM/odom provider.
- WebInput response stream and optional phone speaker relay.
- Go2 SeatGuide blueprints.
- macOS replay SHM route integration.
- One-command hardware bring-up script.
- No-motion smoke, hardware acceptance, and acceptance log verifier.

Verified:

- SeatGuide/MCP tests pass.
- pubsub/Rerun SHM tests pass.
- `bin/demo_seat_guide_replay_smoke` completes on Mac.

Not complete yet:

- Real Go2 hardware transcript. Final completion requires proof of real browser microphone input, real camera/VLM/odom, real navigation, and `goal_reached=true`.
