# 7. Agents, perception and evals

Agent checks need the configured LLM credentials. Most run on replay data or in simulation. See the [agent docs](../../docs/capabilities/agents/index.md).

## 7.1 Local LLM backends and request tracing

Needs Ollama running (`ollama serve`). Run each stage below separately: `agent-send` returns when the message is sent, before the model has finished responding.

```bash
ollama pull qwen3:8b
uv run dimos --replay run unitree-go2-agentic-ollama --daemon
```

In another terminal, start `uv run humancli` and leave it open for both runs. Send the first greeting from the original terminal:

```bash
uv run dimos agent-send "say hello"
```

Wait for the spoken greeting to finish and the final assistant reply to appear in `humancli`. Then stop the local-model run and start a provider run with a fresh trace directory:

```bash
uv run dimos stop
RELEASE_TRACE_DIR="$(mktemp -d)"
uv run dimos --replay run unitree-go2-agentic --daemon --mcpclient.trace-dir "$RELEASE_TRACE_DIR"
uv run dimos agent-send "say hello"
```

Wait again for the spoken greeting and final assistant reply before inspecting the trace and stopping:

```bash
ls "$RELEASE_TRACE_DIR"
uv run dimos stop
```

Expected result: the Ollama run greets through the speakers with the local model, and the provider run produces complete request/response pairs in the new trace directory. `unitree-go2-agentic-huggingface` is the same check with a local Hugging Face model, which downloads on first start.

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

Needs the Habitat setup from the [Habitat check](08_navigation.md#87-habitat-simulation).

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

Needs a TypeSafe API key from https://console.typesafe.ai/settings/keys and the Habitat setup from the [Habitat check](08_navigation.md#87-habitat-simulation).

```bash
export TYPESAFENAVIGATIONAGENT__API_KEY=<typesafe-api-key>
uv run dimos run habitat-typesafe
# In another terminal:
uv run humancli --transport zenoh
```

* In Rerun drive with WASD until a chair is in the camera view, then type `go to the chair` in `humancli`.

Expected result: the robot turns toward the chair and drives up to it, and the agent ends with `goal reached or unreachable; stopped`.

## 7.5 Object detection in replay

The first run pulls the YOLO weights.

```bash
uv run dimos --replay run unitree-go2-detection
# In another terminal, as soon as Rerun opens. The recording plays once and lasts 60 s.
uv run dimos --transport lcm topic echo /detector3d/image/0
```

The detection blueprint explicitly publishes this output on LCM, so the echo needs `--transport lcm` even when the rest of the stack uses Zenoh. The host needs the [LCM multicast setup](05_cli-and-api.md#56-transports-and-topics) for a separate echo process to receive it.

Expected result: the echo prints one `Image(shape=...)` line for each camera frame with a detection. In Rerun, `world/detected_pointcloud_0` highlights the detected object's points on the map.

## 7.6 Perception localize

Needs a CUDA GPU. The first run downloads the SigLIP, OWLv2 and EdgeTAM weights.

```bash
uv run dimos --replay run go2-localize
# In another terminal. Repeat until the state reads ready.
uv run dimos mcp call state
uv run dimos mcp call localize -j '{"objects": "table", "start": 0, "duration": 60}' --timeout 300
```

Expected result: `state` goes from `not ready` to `ready`, and `localize` prints a `'table' at (x, y, z) in world` line with a score for each instance it found.

## 7.7 Camera calibration and AprilTag markers

Needs a webcam, a printed chessboard with 8 by 6 inner corners, and the tag from the first command printed at 100% scale. See [Camera calibration](../../docs/usage/camera_calibration.md).

```bash
uv run dimos apriltag --ids 0 --size-mm 100 --out apriltags.pdf
uv run dimos cameracalibrate --source webcam --cols 8 --rows 6 --square-size-m <square-m> --out ./camera_info.yaml ./camera_info.preview.png
uv run dimos run desk-marker-tf rerun-bridge-module --marker-length-m 0.1
```

* Calibration: `<square-m>` is the measured edge of one printed square in meters. Press Space on about 20 sharp views of the board.
* Markers: hold the tag in front of the webcam.

The stock `desk-marker-tf` blueprint loads its bundled `fixtures/camera_info.yaml` for both webcam setup and marker pose estimation; it does not read the `camera_info.yaml` produced by the preceding command. To check metric marker poses on a different webcam, configure both `create_desk_webcam` and `MarkerDetectionStreamModule` with that camera's calibrated `CameraInfo` in a custom blueprint. The stock command still checks marker detection and display.

Expected result: calibration prints an RMS line and writes both files, and Rerun shows the webcam feed with a box labelled with the tag id.

## 7.8 Agent evals

The live case needs the DimSim assets from the [DimSim check](02_replay-and-simulation.md#23-dimsim-browser-simulation), the [LCM multicast host setup](05_cli-and-api.md#56-transports-and-topics), and an `OPENAI_API_KEY` for the agentic stack's `SpeakSkill` startup. It takes several minutes.

```bash
uv run dimos evals list
uv run dimos evals run dimos.evals.suites.examples --agent dimos.evals.agents.question_answer
DIMOS_TRANSPORT=lcm uv run dimos evals run dimos.evals.suites.dimsim_house --agent dimos.evals.agents.mcp_client_adapter --set 'modules=["unitree-go2-agentic"]'
```

Expected result: `list` prints the suites. The examples run prints `PASS` for both cases and a summary line that ends in the run directory, which holds `results.jsonl` and `summary.json`. In the live run the robot tours the apartment and drives to the bed, and `dimsim_go_to_bed` passes.

## 7.9 VQA dataset generation

Needs a CUDA GPU and a fresh recording, since the shipped replay datasets have no `camera_info` or `tf` streams.

```bash
# Let it run for about a minute, then Ctrl+C.
uv run dimos --record --simulation run unitree-go2
uv run dimos evals vqa generate recordings/<run-id>/memory.db --start 20 --stop 300 --stride 100
uv run dimos evals vqa run ~/.local/state/dimos/datasets/vqa/memory-frames --model gpt-4o-mini
```

The first recorded images can precede the first `camera_info` or `tf` message. If generation reports missing calibration or pose at the selected image timestamp, increase `--start` until the first selected image is after both streams begin. `20` skipped the startup gap in this check.

Expected result: `generate` prints "Generated N VQA case(s)" with the dataset directory, and `run` prints a result line per case and a summary with no errors.

## 7.10 xArm6 WorldBelief

The `xarm6-worldbelief` hardware blueprint has a broken TF tree, so this check runs its `scan` and `recall` skills on a recorded xArm6 session. The first run downloads the detector and embedding models.

```bash
uv run python -c "from dimos.utils.data import get_data; print(get_data('xarm6_worldbelief_realsense_d435i_stationery_calibrated/xarm6_worldbelief_20260729_203624_161992.db'))"
uv run dimos --replay run world-belief-module mcp-server --daemon --worldbeliefmodule.db-path <printed-path>
# Wait for "WorldBelief scan models warmed up" in uv run dimos log -f.
uv run dimos mcp call scan -j '{"prompt": ["book", "pen"]}' --timeout 300
uv run dimos mcp call recall -a text="book" --timeout 300
# If scan found pen but no book, also check the confirmed object:
uv run dimos mcp call recall -a text="pen" --timeout 300
uv run dimos stop
```

Expected result: `scan` prints `Scan complete: <n> object(s) present` with n of at least 1, and `recall` returns the best match with `when_ts`, `where_camera` and `where_object` fields. If there is no detector-confirmed book, `recall book` may have `where_object: None`; recalling a confirmed scanned object checks the grounded object position.

## 7.11 Depth2DepthCloud

Needs a Go2 whose Jetson has a dimos checkout and nix. The first start builds the native modules with nix. On the Jetson it also builds the TensorRT engine, which takes minutes.

```bash
# On the Jetson:
uv run dimos run go2-dds depth2-depth-cloud --daemon
uv run dimos topic echo /depth_cloud   # Ctrl+C after a few messages
uv run dimos log -n 50
uv run dimos stop
```

Expected result: the echo prints `PointCloud2(frame_id=..., num_points=...)` messages with a nonzero point count, and the log shows `Depth model loaded.` and `depth2depth_cloud timing` lines.
