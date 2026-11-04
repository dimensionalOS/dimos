# 7. Agents, perception and evals

Agent checks need the configured LLM credentials. Most run on replay data or in simulation. See the [agent docs](../../docs/capabilities/agents/index.md).

## 7.1 Local LLM backends and request tracing

Needs Ollama running (`ollama serve`) and `OPENAI_API_KEY`, since `SpeakSkill` speaks through OpenAI TTS. Run each stage below separately: `agent-send` returns when the message is sent, before the model has finished responding.

```bash
ollama pull qwen3:8b
uv run dimos --simulation run unitree-g1-agentic-sim --daemon --mcpclient.model ollama:qwen3:8b
```

In another terminal, start `uv run humancli` and leave it open for both runs. Send the first greeting from the original terminal:

```bash
uv run dimos agent-send "say hello"
```

Wait for the spoken greeting to finish and the final assistant reply to appear in `humancli`. Then stop the local-model run and start a provider run with a fresh trace directory:

```bash
uv run dimos stop
RELEASE_TRACE_DIR="$(mktemp -d)"
uv run dimos --simulation run unitree-g1-agentic-sim --daemon --mcpclient.trace-dir "$RELEASE_TRACE_DIR"
uv run dimos agent-send "say hello"
```

Wait again for the spoken greeting and final assistant reply before inspecting the trace and stopping:

```bash
ls "$RELEASE_TRACE_DIR"
uv run dimos stop
```

Expected result: the Ollama run greets through the speakers with the local model, and the provider run produces complete request/response pairs in the new trace directory.

## 7.2 Tool streams and capability conflicts

```bash
uv run dimos run demo-capabilities
uv run humancli
```

Type these prompts in `humancli`, one at a time:

* "Start patrolling and start an environment scan."
* "Without stopping patrol first, try to turn in place 90 degrees. If there is a conflict, report the exact error and do not recover."
* "Stop patrol, then turn in place 90 degrees."

Expected result: the first prompt shows two live tool boxes that stream at the same time. The second is refused with `Cannot start 'turn_in_place': capability 'movement' is held by 'start_patrol'`. After the third the patrol box closes and the turn completes.

## 7.3 Point navigation skill

Needs the Habitat setup from the [Habitat check](08_navigation.md#85-habitat-simulation).

```bash
uv run dimos run habitat-nav mcp-server point-nav-skill-container
# In another terminal:
uv run dimos mcp call go_to --arg x=<x> --arg y=<y>
uv run dimos mcp call go_to --arg x=<x2> --arg y=<y2>
uv run dimos mcp call stop_navigation
```

* For each `<x>` and `<y>`, hover over a free spot on the planner surface a few metres away in Rerun and read its coordinates. Send the second goal after the robot arrives, and stop it on the way.

Expected result: each `go_to` prints a line starting with "Navigating." and draws a path. The robot stops at the first point, and `stop_navigation` prints "Stopped." and halts it short of the second.

## 7.4 TypeSafe navigation agent

Needs a TypeSafe API key from https://console.typesafe.ai/settings/keys and the Habitat setup from the [Habitat check](08_navigation.md#85-habitat-simulation).

```bash
export TYPESAFENAVIGATIONAGENT__API_KEY=<typesafe-api-key>
uv run dimos run habitat-typesafe
# In another terminal:
uv run humancli --transport zenoh
```

* In Rerun drive with WASD until a chair is in the camera view, then type `go to the chair` in `humancli`.

Expected result: the robot turns toward the chair and drives up to it, and the agent ends with `goal reached or unreachable; stopped`.

## 7.5 Perception localize

Needs a CUDA GPU. The first run downloads the SigLIP, OWLv2 and EdgeTAM weights. See [the localize demo](../../dimos/perception/localize/demo_blueprints/README.md).

```bash
uv run dimos run xarm-localize-live
# In another terminal, the recorded xArm feed:
uv run python -m dimos.perception.localize.demo_blueprints.xarm_feed --seek 427 --duration 76
# In a third terminal. Repeat until the state reads ready.
uv run dimos mcp call state
uv run dimos mcp call localize -j '{"objects": "book", "start": 0, "duration": 76}' --timeout 300
```

Expected result: `state` goes from `not ready` to `ready`, and `localize` prints a `'book' at (x, y, z) in world` line with a score for each instance it found.

## 7.6 Camera calibration and AprilTag markers

Needs a webcam, a printed chessboard with 8 by 6 inner corners, and the tag from the first command printed at 100% scale. See [Camera calibration](../../docs/usage/camera_calibration.md).

```bash
uv run dimos apriltag --ids 0 --size-mm 100 --out apriltags.pdf
uv run dimos cameracalibrate --source webcam --cols 8 --rows 6 --square-size-m <square-m> --out ./camera_info.yaml ./camera_info.preview.png
uv run dimos run desk-marker-tf rerun-bridge-module --marker-length-m 0.1
```

* Calibration: `<square-m>` is the measured edge of one printed square in meters. Press Space on about 20 sharp views of the board.
* Markers: hold the tag in front of the webcam.

`desk-marker-tf` uses its bundled `fixtures/camera_info.yaml`, not the file the calibration wrote.

Expected result: calibration prints an RMS line and writes both files, and Rerun shows the webcam feed with a box labelled with the tag id.

## 7.7 Agent evals

```bash
uv run dimos evals list
uv run dimos evals run dimos.evals.suites.examples --agent dimos.evals.agents.question_answer
```

Expected result: `list` prints the suites. The examples run prints `PASS` for both cases and a summary line that ends in the run directory, which holds `results.jsonl` and `summary.json`.

## 7.8 VQA dataset generation

Needs a CUDA GPU and a fresh recording, since the shipped replay datasets have no `camera_info` or `tf` streams.

```bash
# Let it run for about a minute, then Ctrl+C.
uv run dimos --record --simulation run unitree-go2
uv run dimos evals vqa generate recordings/<run-id>/memory.db --start 20 --stop 300 --stride 100
uv run dimos evals vqa run ~/.local/state/dimos/datasets/vqa/memory-frames --model gpt-4o-mini
```

The first recorded images can precede the first `camera_info` or `tf` message. If generation reports missing calibration or pose at the selected image timestamp, increase `--start` until the first selected image is after both streams begin. `20` skipped the startup gap in this check.

Expected result: `generate` prints "Generated N VQA case(s)" with the dataset directory, and `run` prints a result line per case and a summary with no errors.

## 7.9 xArm6 WorldBelief

The `xarm6-worldbelief` hardware blueprint has a broken TF tree, so this check runs its `scan` and `recall` skills on a recorded xArm6 session. The first run downloads the detector and embedding models.

```bash
uv run python -c "from dimos.utils.data import get_data; print(get_data('xarm6_worldbelief_realsense_d435i_stationery_calibrated/xarm6_worldbelief_20260729_203624_161992.db'))"
uv run dimos --replay run world-belief-module mcp-server --daemon --worldbeliefmodule.db-path <printed-path>
# Wait for "WorldBelief scan models warmed up" in uv run dimos log -f.
uv run dimos mcp call scan -j '{"prompt": ["book", "pen"]}' --timeout 300
uv run dimos mcp call recall -a text="pen" --timeout 300
uv run dimos stop
```

Expected result: `scan` prints `Scan complete: <n> object(s) present` with a confirmed `pen` among the objects, and `recall` returns the pen with `when_ts`, `where_camera` and a `where_object` position.
