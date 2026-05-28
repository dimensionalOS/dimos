# DimOS — hands-on learnings (verified on the VPS)

> Everything here was **actually run** on the dev VPS (Ubuntu 24.04, 6-core, 11 GB RAM, **no GPU**,
> **headless/no DISPLAY**) on 2026-05-26 — not copied from docs. The clone lives at `~/code/dimos`
> (sibling of this repo, not committed here); experiments + template live in `learn/`.

## 0. Bottom line
The full Go2 stack **runs on our headless, GPU-less VPS** — great for learning the framework, the CLI, and
the `@skill` seam without a robot. What we *can't* do here: GUI viewers, MuJoCo rendering, fast local VLMs,
or talk to a real dog. Those wait for the venue laptop + hardware.

## 1. Install (worked)
```bash
sudo apt-get install -y curl g++ portaudio19-dev git-lfs libturbojpeg python3-dev
cd ~/code/dimos
uv venv --python 3.12 && source .venv/bin/activate
uv pip install 'dimos[base,unitree]'   # CLI = `dimos`
```
- Repo's own `.python-version` = **3.12** (matches the VPS system Python). 
- The **`unitree` extra is heavy** — it pulls `ultralytics`, `transformers`, `triton` (perception/YOLO/VLM).
  Fine, but on a no-GPU box those run on CPU (slow) → **prefer hosted VLMs** (Qwen/Claude/OpenAI) for reasoning.
- GPU is **only** needed for perception/VLMs; basic control + the whole framework run CPU-only.

## 2. ⚠️ Version skew + cwd shadowing (real traps)
- **Installed (PyPI) = `dimos 0.0.12.post2`. Clone `main` = commit `b45e5d5` (2026-05-25).** They differ.
  Example: `dimos.agents.skill_result.SkillResult` exists on **main** but **NOT** in 0.0.12.post2.
- `dimos.agents` is a **namespace package**, and **running from the clone root shadows the installed package**
  (cwd's `dimos/` wins for some imports, site-packages for others → inconsistent mixing).
  - **Rule:** either run from a *neutral* directory to use the installed pkg cleanly, **or** do an editable
    install (`uv pip install -e '.[base,unitree]'`) and always work inside the clone. **Don't mix.**
  - At the venue: check which version the provided image uses and match it. Write skills with **`str` returns**
    (work everywhere); only use `SkillResult` if you're on main.

## 3. Headless is solved: `--viewer`
The CLI has `--viewer [rerun | rerun-web | rerun-connect | foxglove | none]`. Use **`--viewer none`** on the
VPS; use `rerun-web` if you want a browser viewer over a port.

## 4. The no-hardware Go2 replay (ran clean, headless)
```bash
dimos --replay --viewer none run unitree-go2
```
Brought up the **entire navigation pipeline with zero errors**: a 9-worker pool deploying
`GO2Connection`, `VoxelGridMapper`, `CostMapper`, `ReplanningAStarPlanner`, `WavefrontFrontierExplorer`,
`PatrollingModule`, `WebsocketVisModule` (web UI on **:7779**). You can watch `autoconnect` wire modules by
**named LCM topics**: `/odom`, `/cmd_vel`, `/lidar`, `/pointcloud`, `/color_image`, `/global_costmap`,
`/path`, `/goal_request`, `/goal_reached`, `/navigation_state`, `/explore_cmd`, …
- First run downloads **~76 MB** of replay data via **git-lfs** (then cached). Budget ~1 min the first time.
- LCM prints recommended sysctl/multicast tweaks (optional, smoother LCM):
  `sudo ip link set lo multicast on`, `sudo sysctl -w net.core.rmem_max=67108864`.

## 5. CLI map (from `dimos --help`)
`run · status · stop · restart · log · agent-send · list · show-config · mcp · topic · lcmspy · agentspy ·
humancli · top · rerun-bridge · go2tool`
- `dimos list` → the blueprint catalog. Highlights for us: **`unitree-go2-security`** (literally Patrol Dog),
  `unitree-go2-memory` / `-temporal-memory` / `-spatial`, `unitree-go2-agentic` (+ `-ollama` for a local LLM),
  `unitree-go2-detection`, plus `teleop-phone-go2`, `unitree-go2-fleet`.
- Skill calls without an LLM: `dimos mcp list-tools`, `dimos mcp call <skill> --arg k=v`.
  Via the LLM: `dimos agent-send "explore the room"`.

## 6. The `@skill` seam (the 80% — verified working)
A `@skill` is just a `Module` method with a **mandatory docstring** (the LLM reads it as the tool
description), **typed params with defaults**, and a **`str`** (or `SkillResult`) return. The decorator adds
timing/context and logs `SKILL <name> result=<OK|…> duration_ms=…` per call. `Module.get_skills()` collects them.

Real examples in `dimos/robot/unitree/unitree_skill_container.py`:
`relative_move(forward, left, degrees)`, `wait(seconds)`, `current_time()`, `execute_sport_command(command_name)`
— the last maps names→IDs from a big WebRTC table (`BalanceStand` 1002, `Sit` 1009, `Hello` 1016, `Dance1` 1022,
`Wallow` 1021, …).

Our artifacts (both **run/import clean** against 0.0.12.post2):
- [`try_skill.py`](./try_skill.py) — minimal tested decorator demo (`greet`, `add`).
- [`patrol_dog_skills.py`](./patrol_dog_skills.py) — a Patrol Dog skill **container template** (Module subclass:
  `patrol_route`, `go_to_waypoint`, `describe_surroundings`, `check_for_anomaly`, `log_incident`,
  `incident_report`) with mocked bodies + TODOs to wire to real specs on Day 1.

## 7. Minor gotcha: the `simplerobot` example
`examples/simplerobot/simplerobot.py --headless --selftest` produced **no stdout** — not a hang. Its prints
happen inside forked **worker** processes, so they don't reach a piped parent stdout. Don't panic if a
Module's internal prints "disappear"; use `dimos log` / the logger instead.

## 8. `default.env` keys that matter
`OPENAI_API_KEY`, `ANTHROPIC_API_KEY`, **`ALIBABA_API_KEY`** (Qwen — on-brand for the Ali venue),
`HUGGINGFACE_ACCESS_TOKEN`/`HF_TOKEN`, `ROBOT_IP`, `CONN_TYPE=webrtc`.

## 9b. ⚠️ LLM provider reality (Gemini + a langchain pin trap)
- **`uv pip install 'dimos[base,unitree]'` resolved newer langchain (1.3.1) than the repo lock (1.2.3)**, which
  **breaks the agent**: `from langchain.agents import create_agent` → `ImportError: ToolCallTransformer`.
  **Fix (pin to the lock):**
  ```bash
  uv pip install "langchain==1.2.3" "langchain-core==1.3.3" \
    "langgraph==1.0.8" "langgraph-prebuilt==1.0.7" "langgraph-checkpoint==4.0.0"
  ```
  Better: use `uv sync` (honors the lockfile) instead of `uv pip install` for the agentic stack.
- **Gemini is NOT natively supported** in this version. The agent defaults to `gpt-4o`; installed providers are
  `langchain-openai`, `-anthropic`, `-huggingface`, `-ollama`. There is **no `langchain-google-genai`**, and
  **installing it bumps `langgraph` and breaks `create_agent`.** So for the DimOS agent, use **OpenAI or
  Anthropic** (supported, won't break the stack), or run Gemini experiments in a **separate venv**.
- The Gemini **key itself is valid and works**: verified via the REST `models` list, and end-to-end
  tool-calling via `ChatGoogleGenerativeAI(model="gemini-2.5-flash").bind_tools([...])` — it correctly emitted
  `go_to_waypoint({'name':'back door'})`. Set `GOOGLE_API_KEY` (or `GEMINI_API_KEY`). (Rotate the key post-event.)

## 9c. TypeScript interop = LCM bus client, NOT a framework language
`examples/language-interop/ts` (Deno + JSR `@dimos/lcm`, `@dimos/msgs`) lets TS **publish/subscribe to LCM
topics** (e.g. subscribe `/odom`, publish `/cmd_vel`) and even drive a browser UI via a WebSocket↔UDP bridge.
But you **cannot** write Modules, Blueprints, `@skill`s, the agent runtime, nav, perception, or memory in TS —
all of that is Python. **TS is a frontend / bus-client / payments language here, not a way to avoid Python.**
See the "TypeScript role" split in IDEAS/RUNBOOK.

## 9. Next steps (when you have an LLM key / the laptop / the dog)
1. Put a key in `default.env`, then `dimos --simulation run unitree-go2-agentic --daemon` →
   `dimos mcp list-tools` to see skills exposed as MCP tools; `dimos agent-send "..."` to drive via LLM.
2. Read **`unitree-go2-security`** + `dimos/robot/unitree/go2/blueprints/agentic/_common_agentic.py` — that's
   where skill containers get registered into the agent.
3. Wire `PatrolDogSkills` into that blueprint and call it via `dimos mcp call`.
4. Decide PyPI vs. editable-from-main early and match the venue image.
