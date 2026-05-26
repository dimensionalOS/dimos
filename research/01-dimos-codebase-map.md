# DimOS Codebase Map — Ground Truth for the DIMENSIONAL Hackathon

**Researcher's note.** All citations below are from a fresh `gh repo clone dimensionalOS/dimos` (default branch `main`, ~3.3k stars, `pushed_at 2026-05-26T00:14:58Z` — i.e. the repo was last pushed the morning the hackathon opens, so it is a moving target). File paths are relative to repo root unless noted. Where I write **VERIFIED** I read the actual source; **CLAIM** means README/AGENTS.md asserts it and I could not fully confirm; **DEBUNKED/CORRECTED** flags where docs diverge from code.

The single best onboarding doc in the repo is **`AGENTS.md`** (root, 15 KB) — point your coding agents at it. It is mostly accurate but has a few stale file references (see §1).

---

## 1. Real architecture: Modules / Streams / Blueprints / RPC / Skills / Transports

DimOS is a **Python-native actor/dataflow framework**. No ROS required (there's an optional ROS *bridge transport*). The mental model: **Modules** are actors running in forkserver worker processes; they talk over typed **streams** (`In[T]`/`Out[T]`) carried by pluggable **transports** (LCM by default); **Blueprints** wire modules together by `(name, type)` matching; **RPC** lets one module call another's methods with compile-time-typed `Spec` protocols; **Skills** are RPC methods additionally exposed to an LLM.

**Module base class** — `dimos/core/module.py` (`class Module`). Declare streams as class annotations:

```python
from dimos.core.module import Module
from dimos.core.stream import In, Out          # dimos/core/stream.py
from dimos.core.core import rpc                 # dimos/core/core.py — the @rpc decorator
from dimos.msgs.sensor_msgs.Image import Image

class MyModule(Module):
    color_image: In[Image]      # subscribes
    processed:   Out[Image]     # publishes

    @rpc
    def start(self) -> None:
        super().start()
        self.color_image.subscribe(self._process)

    def _process(self, img: Image) -> None:
        self.processed.publish(do_something(img))
```

**Blueprints** — `dimos/core/coordination/blueprints.py`. `autoconnect(*modules)` matches each `Out[T]` to a same-named `In[T]` and returns a `Blueprint`. `.build()` deploys modules into workers (`dimos/core/coordination/python_worker.py`, `module_coordinator.py`); `.loop()` blocks. `.transports({...})` overrides a stream's transport; `.remappings(...)` resolves ambiguous matches; `.requirements(...)` gates on preconditions (e.g. `ollama_installed`).

```python
from dimos.core.coordination.blueprints import autoconnect
autoconnect(module_a(), module_b()).build().loop()
```

**RPC + Spec injection** — `dimos/spec/utils.py`. To call another module's methods, declare a `Spec`/`Protocol` and annotate an attribute with it; the blueprint injects the real module at build time (fails at build, not runtime, if unmatched):

```python
class NavigationInterfaceSpec(Spec, Protocol):     # dimos/navigation/navigation_spec.py
    def set_goal(self, goal: PoseStamped) -> bool: ...
    def get_state(self) -> NavigationState: ...

class UnitreeSkillContainer(Module):
    _navigation: NavigationInterfaceSpec           # injected by blueprint
    _connection:  GO2ConnectionSpec
```

(Real example: `dimos/robot/unitree/unitree_skill_container.py` lines 30-32, 197-198.)

**Transports** — `dimos/core/transport.py`:
- `LCMTransport` — default, multicast UDP, the backbone.
- `SHMTransport`/`pSHMTransport` — shared memory; used for images & point clouds (e.g. Go2 connection imports `pSHMTransport`).
- `pLCMTransport` — pickled LCM for arbitrary Python objects (used by `agent_send` over topic `/human_input`).
- `ROSTransport` — ROS 2 topic bridge (interop only).
- `DDSTransport` — `dimos/protocol/pubsub/impl/ddspubsub.py`, behind `uv sync --extra dds`.

**GlobalConfig** — `dimos/core/global_config.py` (`class GlobalConfig(BaseSettings)`). Singleton; cascade defaults → `.env` → `DIMOS_*` env vars → blueprint → CLI flags. Key fields I verified: `robot_ip`, `simulation`, `replay`, `replay_db="go2_short"`, `mcp_port=9990`, `listen_host="127.0.0.1"`, `n_workers=2`, `obstacle_avoidance=True`, `detection_model="moondream"`, `dimsim_scene="apt"`, `dimsim_port=8090`. **Every field is auto-exposed as a CLI flag.**

**CLI** — `dimos/robot/cli/dimos.py` (typer). Verbs: `run/status/stop/restart/list/show-config/log/mcp/agent-send/lcmspy/agentspy/humancli/topic/rerun-bridge`. Logs at `~/.local/state/dimos/logs/<run-id>/main.jsonl`; runs tracked in `~/.local/state/dimos/runs/`.

> **Stale doc flag:** AGENTS.md references `dimos/agents/agent.py` ("Agent module (LangGraph-based)"). **That file does not exist.** The live agent runtime is `dimos/agents/mcp/mcp_client.py` (`McpClient`). There is also a `dimos/agents_deprecated/` tree (old `Agent`, ChromaDB spatial DB) — **do not build on `agents_deprecated`.**

---

## 2. The Skill API — VERIFIED, with a real example

The **live** skill system is the `@skill` decorator, *not* the `SkillLibrary`/`AbstractSkill(BaseModel)` classes in `dimos/skills/skills.py` (those are the legacy/deprecated path — they still exist but the agentic blueprints don't use them).

**`@skill`** — `dimos/agents/annotation.py`. It wraps the function, sets `__skill__ = True`, and applies `@rpc` (line 109-110). So `@skill` ⇒ callable over RPC **and** exposed to the LLM. **Never stack `@rpc` and `@skill`.** The wrapper also injects per-call MCP context (`_mcp_context`, `progress_token`), times the call, and logs `SKILL <name> result=… duration_ms=…`.

Schema generation rules (enforced at startup — break them and the module fails to register, taking *all* its skills down):
- **Docstring mandatory** → `ValueError` at startup if missing.
- **Type-annotate every param** (supported: `str/int/float/bool/list[str]/list[float]`; avoid nested types).
- **Return `str`** (or a `SkillResult`, or an `Image`). A `None` return makes the agent hear "It has started. You will be updated later."
- The full docstring becomes the tool `description` the LLM sees — keep the `Args:` block tight.

**Minimal real skill** (pattern from `unitree_skill_container.py`):

```python
from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module

class MySkillContainer(Module):
    @rpc
    def start(self) -> None: super().start()
    @rpc
    def stop(self) -> None: super().stop()

    @skill
    def move(self, x: float, duration: float = 2.0) -> str:
        """Move the robot forward or backward.

        Args:
            x: Forward velocity in m/s. Positive = forward, negative = backward.
            duration: How long to move in seconds.
        """
        return f"Moving at {x} m/s for {duration}s"

my_skill_container = MySkillContainer.blueprint   # expose for autoconnect()
```

**How skills are registered/exposed.** Skill containers are plain `Module`s added to the agentic blueprint via `autoconnect`. At deploy time the **`McpServer.on_system_modules()`** RPC (`mcp_server.py` line 278) walks every deployed module, calls `module.get_skills()`, and builds `app.state.skills` (a list of `SkillInfo`: `func_name`, `class_name`, `args_schema`) plus an `app.state.rpc_calls` map. The LLM agent (`McpClient`) then fetches them over HTTP.

**Is the MCP server real, and what does it expose?** **VERIFIED REAL.** `dimos/agents/mcp/mcp_server.py` is a **FastAPI + uvicorn** app:
- `POST /mcp` — JSON-RPC 2.0. Methods: `initialize`, `tools/list`, `tools/call`. Protocol version `"2025-11-25"`, serverInfo `{"name":"dimensional","version":"1.0.0"}`.
- `GET /mcp` — **SSE** stream (Streamable-HTTP out-of-band channel) that fans out tool-progress notifications.
- CORS `allow_origins=["*"]`.
- Binds to `GlobalConfig.listen_host` (**default `127.0.0.1`**) and `GlobalConfig.mcp_port` (**default `9990`**). So `http://localhost:9990/mcp` — the claim — is **VERIFIED**, *but* note the bind is loopback-only by default.
- Built-in MCP-exposed skills on the server itself: `server_status`, `list_modules`, `agent_send`.

**Can you drive the robot from an external MCP client (Claude Desktop/Slack)?** **Mostly VERIFIED, with a caveat.** The server speaks standard MCP Streamable-HTTP and the code comment explicitly says *"External clients like Claude Code still use GET /mcp"* (`mcp_client.py` line 207). Any Streamable-HTTP MCP client can `tools/list` then `tools/call move`/`speak`/etc. **Caveats that will bite you:** (1) default bind is `127.0.0.1` — for an off-box client (Claude Desktop on your laptop, a Slack bot) you must set `--mcp-host 0.0.0.0` (or `listen_host`) and reach it over LAN/tunnel; (2) Claude Desktop's stock config expects stdio servers — you need an HTTP/SSE bridge or a client that supports remote HTTP MCP; (3) there is **no auth** on the endpoint. The cleanest in-arena demo is `dimos mcp call <skill> --arg k=v` (uses the same JSON-RPC path) or pointing **Claude Code** (HTTP MCP capable) at it.

---

## 3. Go2 integration — what's REALLY implemented

**Connection / WebRTC** — `dimos/robot/unitree/go2/connection.py` (`GO2Connection`), driven by `dimos/robot/unitree/connection.py` (`UnitreeWebRTCConnection`), built on the org fork `dimensionalOS/go2_webrtc_connect` (imported as `unitree_webrtc_connect`). The connection exposes `lidar_stream`, `odom_stream`, `video_stream`, `move(twist, duration)`, `standup/liedown/balance_stand`, `set_obstacle_avoidance`, `enable_rage_mode`, and a raw `publish_request(topic, data)` escape hatch. A `ReplayConnection` subclass replays recorded sessions from a SQLite store (`dimos/memory2/`) — this is how `--replay` works with **no hardware**.

**Action primitives** — `dimos/robot/unitree/unitree_skill_container.py` (VERIFIED, read in full):
- `relative_move(forward, left, degrees)` — converts a body-frame offset to a world goal pose and hands it to the **navigation** stack (`_navigation.set_goal`), then polls `NavigationState` until arrival/timeout. So "move forward 2m" is a *planned* move, not open-loop.
- `execute_sport_command(name)` — sends a WebRTC SPORT_MOD request by API ID. **The full Unitree trick table is hardcoded** (`UNITREE_WEBRTC_CONTROLS`, lines 38-184): `Hello`(1016), `Stretch`(1017), `Dance1`(1022)/`Dance2`(1023), `WiggleHips`(1033), `FingerHeart`(1036), `FrontFlip`(1030), `FrontJump`(1031), `FrontPounce`(1032), `Handstand`(1301), `MoonWalk`(1305), `Backflip`(1044), `Sit`/`StandUp`/`RecoveryStand`, etc. These are **demo gold** — instant crowd-pleasers wired to natural language already.
- `wait(seconds)`, `current_time()`.

**Navigation stack** — `dimos/navigation/` (VERIFIED dirs):
- `replanning_a_star/` — a real A* global+local planner with a **C++ extension** (`min_cost_astar_cpp.cpp` / `min_cost_astar.py`), costmap (`navigation_map.py`), goal validator, path clearance, replan limiter.
- `frontier_exploration/wavefront_frontier_goal_selector.py` — wavefront frontier exploration (autonomous "explore the room").
- `visual_servoing/` — `visual_servoing_2d.py` (`VisualServoing2D.compute_twist`) + `detection_navigation.py` (servo to a detected bbox).
- `nav_stack/`, `patrolling/`, `bbox_navigation.py`.
- Plus heavier **native (no-ROS) planner modules** in *separate org repos*: `dimos-module-far-planner`, `dimos-module-tare-planner`, `dimos-module-local-planner`, `dimos-module-path-follower`, `dimos-module-terrain-analysis`, `dimos-module-pct-planner`, and `dimos-module-fastlio2` (FAST-LIO2 SLAM). These are pulled in as native modules; assume they need the native build (`--build-native`) and possibly CUDA.

**Perception** — `dimos/perception/` (VERIFIED):
- Detectors: `detection/detectors/yolo.py`, `yoloe.py`, person detector; default `detection_model="moondream"` (a VLM, `dimos/models/vl/moondream.py`). Other VLMs present: `qwen.py` (Alibaba Qwen-VL — note `ALIBABA_API_KEY` in `default.env`), `openai.py`, `florence.py`, `moondream_hosted.py`.
- 2D/3D object trackers (`object_tracker_2d.py`, `object_tracker_3d.py`), **EdgeTAM** continuous tracker (used by person-follow), ReID (`detection/reid/`), fiducials (`perception/fiducial/`).
- `perceive_loop_skill.py` — `PerceiveLoopSkill` with `look_out_for(...)` / `stop_looking_out()`: a **trigger skill** that watches the detection stream and, on a match, calls `McpClient.dispatch_continuation()` to fire a follow-up tool *without* a round-trip to the LLM (template args like `"$bbox"` resolved from detection context). Powerful for reactive demos.

**Spatial memory** — **ChromaDB is REAL.** `dimos/perception/spatial_perception.py` (`class SpatialMemory(Module)`): `chromadb.PersistentClient` at `data/.../chromadb_data`, collection `"spatial_memory"`, **CLIP** image embeddings (512-d) via `ImageEmbeddingProvider`. `query_by_location`, `query_tagged_location`, `save`. The `NavigationSkillContainer.navigate_with_text(query)` cascade (lines 113-143) is: tagged location → object in current view (VLM bbox) → semantic-map ChromaDB query. **Note:** the ChromaDB path imports from `agents_deprecated.memory.image_embedding` — it works but sits on the older memory code. There's a newer `dimos/memory2/` (SQLite-backed observation store, voxel map, replay) used by the connection/replay layer.

---

## 4. Agent runtime — VERIFIED

The agent is `McpClient` (`dimos/agents/mcp/mcp_client.py`). It is **LangChain/LangGraph**, not a bespoke loop:
- `from langchain.agents import create_agent` builds a `CompiledStateGraph` (LangGraph).
- Config `McpClientConfig`: `model="gpt-4o"` (default), `system_prompt=SYSTEM_PROMPT`, `mcp_server_url="http://localhost:9990/mcp"`.
- On `on_system_modules`, it `initialize`s + `tools/list`s the MCP server, converts each MCP tool to a LangChain `StructuredTool` (`_mcp_tool_to_langchain`), and creates the agent with `(model, tools, system_prompt)`.
- Inputs arrive on the `human_input: In[str]` stream (LCM topic `/human_input`, fed by `dimos agent-send`, `humancli`, or `WebInput` at `localhost:7779` with optional Whisper STT). A worker thread streams the LangGraph graph and re-publishes messages on `agent: Out[BaseMessage]`.
- **Model selection is a string** passed to `create_agent`, so it resolves any LangChain-supported provider. Variants ship: `unitree-go2-agentic` (gpt-4o + MCP), `unitree-go2-agentic-ollama` (`model="ollama:qwen3:8b"`, gated on `ollama_installed`), `unitree-go2-agentic-huggingface`. **Anthropic is a dependency** (`anthropic>=0.19.0` in `pyproject.toml`) and `ANTHROPIC_API_KEY` is in `default.env`, so a Claude model string should work via `langchain` — *I did not see a shipped Claude blueprint*, so treat "Claude as the brain" as a small, plausible extension, not a turnkey path.

**Where API keys go:** root **`default.env`** (copied to `.env`) and/or shell env. Keys present: `OPENAI_API_KEY` (required for default agent + TTS), `ANTHROPIC_API_KEY`, `ALIBABA_API_KEY` (Qwen-VL), `HUGGINGFACE_ACCESS_TOKEN`/`HF_TOKEN`. `ROBOT_IP`, `CONN_TYPE=webrtc`, `WEBRTC_SERVER_PORT=9991`, `DISPLAY=:0`.

**System prompts:** Go2 default in `dimos/agents/system_prompt.py` (`SYSTEM_PROMPT`, persona "Daneel"). G1 prompt in `dimos/robot/unitree/g1/system_prompt.py`. Pass per-robot via `McpClient.blueprint(system_prompt=...)` — the default prompt names Go2-specific skills and will cause hallucinated skills on G1.

---

## 5. Install / run reality

```bash
# Recommended: uv
uv venv --python 3.12 && source .venv/bin/activate
uv pip install 'dimos[base,unitree]'        # base = runtime/CLI; unitree = Go2/G1 WebRTC+skills
# (dev from clone:)  uv sync --extra all

# No hardware — replay a recorded Go2 office session (downloads ~75 MB via LFS on first run)
dimos --replay run unitree-go2
dimos --replay run unitree-go2-agentic --daemon   # + LLM agent + skills + MCP server (needs OPENAI_API_KEY)

# Simulation (MuJoCo)
uv pip install 'dimos[base,unitree,sim]'          # adds mujoco>=3.3.4
dimos --simulation run unitree-go2
dimos --simulation run unitree-g1-agentic-sim     # full agentic humanoid, no robot

# Real Go2
export ROBOT_IP=<ip>     # e.g. 192.168.123.161
dimos run unitree-go2-agentic --robot-ip <ip>
```

Interactive installer: `curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash` (detects OS/RAM/disk/NVIDIA, offers Nix or apt/brew). Nix flake (`flake.nix`) and `.devcontainer/` exist. **Stated requirements:** Ubuntu 22.04+/macOS 12.6+, Python 3.12, 16 GB RAM (32 recommended), GPU optional (required only for perception/VLMs/AI). `requires-python>=3.10` in pyproject but docs target 3.12.

**Sim mode availability:** **YES** — MuJoCo sim (`--simulation`) and a `dimsim` simulator (`dimsim_scene`, `dimsim_port=8090`). There is also full **replay** mode. **You can build and demo the agentic Go2 stack with zero hardware.**

**Docker/ROS2:** Docker is *ready* (`docker/`, `.dockerignore`, hello-world example) but **not required**. **ROS 2 is NOT required** — it's an optional transport bridge only.

**Common setup failures to expect:**
- **LFS**: first `--replay` blocks ~75 MB download; Rerun window black until it lands. If `git lfs` missing, replay data won't fetch.
- **LangGraph/LangChain version pin hell**: `pyproject.toml` has an explicit `[tool.uv] override-dependencies` pinning `langgraph-prebuilt<=1.0.8` because `langchain==1.2.3` caps `langgraph<1.1`. **Do not bump LangChain/LangGraph** — it breaks `create_agent`.
- **No `OPENAI_API_KEY`** → agentic blueprints fail at agent init (and `speak` uses OpenAI TTS).
- **MuJoCo / CUDA** wheels are heavy; `--extra sim`/`--extra cuda` installs add minutes and disk.
- **Native modules** (FAR/TARE/FAST-LIO2 planners) need `--build-native` and a C++ toolchain (and the A* planner has a C++ ext that must compile).
- **MCP from off-box** fails silently if you forget `--mcp-host 0.0.0.0` (default loopback).
- **Audio**: `speak` needs a working `sounddevice` output (`SounddeviceAudioOutput`, 24 kHz) — headless boxes need PulseAudio/ALSA config.

---

## 6. Highest-leverage 48h extension points (ranked)

Ranked by (impact for a 90-sec demo) × (ease given existing code):

1. **Add custom `@skill`s to a new container and drop it into `_common_agentic`.** Touch `dimos/robot/unitree/go2/blueprints/agentic/_common_agentic.py` + a new `MySkillContainer(Module)`. This is the lowest-friction high-impact move: every skill instantly becomes an LLM tool *and* an MCP tool. Update `SYSTEM_PROMPT` to advertise it. **Start here.**
2. **Chain sport-command "tricks" into a choreographed routine** (`execute_sport_command`). The trick table (FingerHeart, Dance1/2, Handstand, MoonWalk, Backflip) is already wired to natural language — a "do a show" skill that sequences them with `speak` narration is a guaranteed crowd-pleaser, sim or replay-safe to author.
3. **Reactive "guard dog" using `look_out_for` + `dispatch_continuation`** (`dimos/perception/perceive_loop_skill.py`, `mcp_client.py:248`). Trigger a follow/speak/bark on detecting a person/object with **no LLM round-trip** — feels instant on video. Template-arg substitution (`"$bbox"`) is already implemented.
4. **Voice-driven everything via `WebInput` (localhost:7779) + Whisper STT + `speak` TTS.** Already shipped (`_common_agentic` includes `WebInput`, `SpeakSkill`). A 90-sec "talk to the dog, it talks back and acts" demo needs basically zero new code — just a good prompt and a USB mic.
5. **External MCP control from Claude Code / a Slack bot.** Run `--mcp-host 0.0.0.0`, point an HTTP MCP client at `:9990/mcp`. "Drive the robot from Slack" is a strong Agents-track story and the transport already exists. Budget time for the bind-host + (optional) tunnel.
6. **`navigate_with_text` + `tag_location` spatial-memory tour.** Tag locations during a walkthrough, then "go to the kitchen" hits ChromaDB semantic recall. High narrative value; `unitree-go2-spatial` / `-temporal-memory` blueprints already assemble the pieces.
7. **Swap the agent brain to Claude.** `McpClient.blueprint(model="claude-...")` via LangChain. Low effort, on-brand for judges, but verify the model string resolves under the pinned LangChain (test early).
8. **New blueprint composition** (Open/Creative track): `autoconnect(...)` your own module mix, then run `pytest dimos/robot/test_all_blueprints_generation.py` to register it in `all_blueprints.py` (auto-generated — don't hand-edit). `dimos list` shows ~80 existing blueprints to crib from.

---

## 7. Gotchas that will eat hours

- **`all_blueprints.py` is auto-generated.** Add a blueprint → run `pytest dimos/robot/test_all_blueprints_generation.py` or `dimos run` won't find it (and CI asserts it's current).
- **`@skill` startup is fragile:** missing docstring or param annotation = `ValueError` that **drops the whole module's skills**. One bad skill silently removes its neighbors.
- **Don't stack `@rpc` + `@skill`** (skill already implies rpc). Don't build on `dimos/skills/skills.py` (`SkillLibrary`) or `dimos/agents_deprecated/` — both are dead-ends.
- **AGENTS.md cites `dimos/agents/agent.py` which doesn't exist** — the real agent is `agents/mcp/mcp_client.py`. Trust the code over the doc.
- **LangChain/LangGraph versions are pinned via uv overrides** — upgrading breaks the agent.
- **MCP server binds loopback by default** (`listen_host=127.0.0.1`) and has **no auth** — fine on-box, needs `--mcp-host 0.0.0.0` + network thought for external clients.
- **`relative_move` is planner-mediated**, not open-loop — it can time out (100 s) or report "cancelled/failed" if the costmap blocks the path. For pure showmanship prefer `execute_sport_command`.
- **`speak` needs real audio out** + `OPENAI_API_KEY` (OpenAI TTS). Headless = no sound.
- **The repo was last pushed the morning of the event** — expect churn; pin to a known-good commit once your build works, and `git pull` deliberately.
- **GPU**: perception/VLM/AnyGrasp/EdgeTAM paths want CUDA; CPU-only works for control/replay but VLM detection will be slow.

---

## Appendix: Org repos that matter

`dimensionalOS/dimos` (core), `dimos-unitree` (Go2/G1 integration — see also DeepWiki `deepwiki.com/dimensionalOS/dimos-unitree`), `go2_webrtc_connect` (the WebRTC driver `unitree_webrtc_connect`), `unitree_sdk2_python`, `go2_ros2_sdk`. Native planner modules: `dimos-module-{far,tare,local,path-follower,terrain-analysis,pct}-planner`, `dimos-module-fastlio2`, `dimos-orb-slam3`, `dimos-module-arise-slam`. Manipulation: `anygrasp_sdk`, `contact_graspnet_pytorch`, `graspnetAPI`. Sim: `Genesis`, `MineDojo`. Viz/data: `dimos-viewer` (Rerun-style), `dimos-lcm`, `python_lcm_msgs`.
