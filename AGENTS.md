# Dimensional AGENTS.md

## What is DimOS

The agentic operating system for generalist robotics. `Modules` communicate via typed streams over LCM, ROS2, DDS, or other transports. `Blueprints` compose modules into runnable robot stacks. `Skills` give agents the ability to execute physical on-hardware functions like `grab()`, `follow_object()`, or `jump()`.

---

## Quick Start

```bash
# Install
uv sync --all-extras --no-extra dds

# List all runnable blueprints
dimos list

# --- Go2 quadruped ---
dimos --replay run unitree-go2                  # perception + mapping, replay data
dimos --replay run unitree-go2 --daemon         # same, backgrounded
dimos --replay run unitree-go2-agentic          # + LLM agent (GPT-4o) + skills + MCP server
dimos run unitree-go2-agentic --robot-ip 192.168.123.161  # real Go2 hardware

# --- G1 humanoid ---
dimos --simulation run unitree-g1-agentic-sim   # G1 in MuJoCo sim + agent + skills
dimos run unitree-g1-agentic --robot-ip 192.168.123.161   # real G1 hardware

# --- Inspect & control ---
dimos status
dimos log              # last 50 lines, human-readable
dimos log -f           # follow/tail in real time
dimos agent-send "say hello"
dimos stop             # graceful SIGTERM → SIGKILL
dimos restart          # stop + re-run with same original args
```

### Blueprint quick-reference

| Blueprint | Robot | Hardware | Agent | MCP server | Notes |
|-----------|-------|----------|-------|------------|-------|
| `unitree-go2-agentic` | Go2 | real | via McpClient | ✓ | McpServer live |
| `unitree-g1-agentic-sim` | G1 | sim | GPT-4o (G1 prompt) | — | Full agentic sim, no real robot needed |
| `xarm-perception-agent` | xArm | real | GPT-4o | — | Manipulation + perception + agent |
| `xarm-perception-sim-agent` | xArm | sim | GPT-4o | — | Manipulation + perception + agent, sim |
| `xarm7-planner-coordinator` | xArm7 | real | — | — | Trajectory planner coordinator |
| `teleop-quest-xarm7` | xArm7 | real | — | — | Quest VR teleop |
| `dual-xarm6-planner` | xArm6×2 | real | — | — | Dual-arm motion planner |

Run `dimos list` for the full list.

---

## Tools available to you (MCP)

**MCP only works if the blueprint includes `McpServer`.** All shipped agentic blueprints use `McpServer` + `McpClient`. E.g.: `unitree-go2-agentic`.

```bash
# Start the MCP-enabled blueprint first:
dimos --replay run unitree-go2-agentic --daemon

# Then use MCP tools:
dimos mcp list-tools                                              # all available skills as JSON
dimos mcp call move --arg x=0.5 --arg duration=2.0               # call by key=value args
dimos mcp call move --json-args '{"x": 0.5, "duration": 2.0}'    # call by JSON
dimos mcp status      # PID, module list, skill list
dimos mcp modules     # module → skills mapping

# Send a message to the running agent (works without McpServer too):
dimos agent-send "walk forward 2 meters then wave"
```

The MCP server runs at `http://localhost:9990/mcp` (`GlobalConfig.mcp_port`).

### Adding McpServer to a blueprint

Use **both** `McpServer.blueprint()` and `McpClient.blueprint()`.

```python
from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer

unitree_go2_agentic = autoconnect(
    unitree_go2_spatial,   # robot stack
    McpServer.blueprint(), # HTTP MCP server — exposes all @skill methods on port 9990
    McpClient.blueprint(), # LLM agent — fetches tools from McpServer
    _common_agentic,       # skill containers
)
```

Reference: `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic.py`

---

## Repo Structure

```
dimos/
├── core/                    # Module system, blueprints, workers, transports
│   ├── module.py            # Module base class, In/Out streams
│   ├── core.py              # @rpc decorator
│   ├── stream.py            # In[T], Out[T], Transport[T]
│   ├── transport.py         # LCM/SHM/ROS/DDS/Jpeg transports
│   ├── coordination/
│   │   ├── blueprints.py           # Blueprint, autoconnect()
│   │   ├── module_coordinator.py   # Deploy + lifecycle orchestration
│   │   ├── python_worker.py        # Forkserver workers + Actor IPC
│   │   └── worker_manager_*.py     # Python / docker worker pools
│   ├── global_config.py     # GlobalConfig (env vars, CLI flags, .env)
│   └── run_registry.py      # Per-run tracking + log paths
├── robot/
│   ├── cli/dimos.py         # CLI entry point (typer)
│   ├── all_blueprints.py    # Auto-generated blueprint registry (DO NOT EDIT MANUALLY)
│   ├── unitree/             # Unitree robot implementations (Go2, G1, B1)
│   │   ├── unitree_skill_container.py  # Go2 @skill methods
│   │   ├── go2/             # Go2 blueprints and connection
│   │   └── g1/              # G1 blueprints, connection, sim, skills
│   └── drone/               # Drone implementations (MAVLink + DJI)
│       ├── connection_module.py        # MAVLink connection
│       ├── camera_module.py            # DJI video stream
│       ├── drone_tracking_module.py    # Visual object tracking
│       └── drone_visual_servoing_controller.py  # Visual servoing
├── agents/
│   ├── agent.py             # Agent module (LangGraph-based)
│   ├── system_prompt.py     # Default Go2 system prompt
│   ├── annotation.py        # @skill decorator
│   ├── mcp/                 # McpServer, McpClient, McpAdapter
│   └── skills/              # NavigationSkillContainer, SpeakSkill, etc.
├── navigation/              # Path planning, frontier exploration
├── perception/              # Object detection, tracking, memory
├── visualization/rerun/     # Rerun bridge
├── msgs/                    # Message types (geometry_msgs, sensor_msgs, nav_msgs)
└── utils/                   # Logging, data loading, CLI tools
docs/
├── usage/modules.md         # ← Module system deep dive
├── usage/blueprints.md      # Blueprint composition guide
├── usage/configuration.md   # GlobalConfig + Configurable pattern
├── development/testing.md   # Fast/slow tests, pytest usage
├── development/dimos_run.md # CLI usage, adding blueprints
└── agents/                  # Agent system documentation
```

---

## Architecture

### Modules

Autonomous subsystems. Communicate via `In[T]`/`Out[T]` typed streams. Run in forkserver worker processes.

```python
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.core import rpc
from dimos.msgs.sensor_msgs import Image

class MyModule(Module):
    color_image: In[Image]
    processed: Out[Image]

    @rpc
    def start(self) -> None:
        super().start()
        self.color_image.subscribe(self._process)

    def _process(self, img: Image) -> None:
        self.processed.publish(do_something(img))
```

### Blueprints

Compose modules with `autoconnect()`. Streams auto-connect by `(name, type)` matching.

```python
from dimos.core.coordination.blueprints import autoconnect

my_blueprint = autoconnect(module_a(), module_b(), module_c())
```

To run a blueprint directly from Python:

```python
# build() deploys all modules into forkserver workers and wires streams
# loop() blocks the main thread until stopped (Ctrl-C or SIGTERM)
autoconnect(module_a(), module_b(), module_c()).build().loop()
```

Expose as a module-level variable for `dimos run` to find it. Add to the registry by running `pytest dimos/robot/test_all_blueprints_generation.py`.

### GlobalConfig

Singleton config. Values cascade: defaults → `.env` → env vars → blueprint → CLI flags. Env vars prefixed `DIMOS_`. Key fields: `robot_ip`, `simulation`, `replay`, `viewer`, `n_workers`, `mcp_port`.

### Transports

- **LCMTransport**: Default. Multicast UDP.
- **SHMTransport/pSHMTransport**: Shared memory — use for images and point clouds.
- **pLCMTransport**: Pickled LCM — use for complex Python objects.
- **ROSTransport**: ROS topic bridge — interop with ROS nodes (`dimos/core/transport.py`).
- **DDSTransport**: DDS pub/sub — available when `DDS_AVAILABLE`; install with `uv sync --extra dds` (`dimos/protocol/pubsub/impl/ddspubsub.py`).

---

## CLI Reference

### Global flags

Every `GlobalConfig` field is a CLI flag: `--robot-ip`, `--simulation/--no-simulation`, `--replay/--no-replay`, `--viewer {rerun|rerun-web|foxglove|none}`, `--mcp-port`, `--n-workers`, etc. Flags override `.env` and env vars.

### Core commands

| Command | Description |
|---------|-------------|
| `dimos run <blueprint> [--daemon]` | Start a blueprint |
| `dimos status` | Show running instance (run ID, PID, blueprint, uptime, log path) |
| `dimos stop [--force]` | SIGTERM → SIGKILL after 5s; `--force` = immediate SIGKILL |
| `dimos restart [--force]` | Stop + re-exec with original args |
| `dimos list` | List all non-demo blueprints |
| `dimos show-config` | Print resolved GlobalConfig values |
| `dimos log [-f] [-n N] [--json] [-r <run-id>]` | View per-run logs |
| `dimos mcp list-tools / call / status / modules` | MCP tools (requires McpServer in blueprint) |
| `dimos agent-send "<text>"` | Send text to the running agent via LCM |
| `dimos lcmspy / agentspy / humancli / top` | Debug/diagnostic tools |
| `dimos topic echo <topic> / send <topic> <expr>` | LCM topic pub/sub |
| `dimos rerun-bridge` | Launch Rerun visualization standalone |

Log files: `~/.local/state/dimos/logs/<run-id>/main.jsonl`
Run registry: `~/.local/state/dimos/runs/<run-id>.json`

---

## Agent System

### The `@skill` Decorator

`dimos/agents/annotation.py`. Sets `__rpc__ = True` and `__skill__ = True`.

- `@rpc` alone: callable via RPC, not exposed to LLM
- `@skill`: implies `@rpc` AND exposes method to the LLM as a tool. **Do not stack both.**

#### Schema generation rules

| Rule | What happens if you break it |
|------|------------------------------|
| **Docstring is mandatory** | `ValueError` at startup — module fails to register, all skills disappear |
| **Type-annotate every param** | Missing annotation → no `"type"` in schema — LLM has no type info |
| **Return `str`** | `None` return → agent hears "It has started. You will be updated later." |
| **Full docstring verbatim in `description`** | Keep `Args:` block concise — it appears in every tool-call prompt |

Supported param types: `str`, `int`, `float`, `bool`, `list[str]`, `list[float]`. Avoid complex nested types.

#### Minimal correct skill

```python
from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module

class MySkillContainer(Module):
    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill
    def move(self, x: float, duration: float = 2.0) -> str:
        """Move the robot forward or backward.

        Args:
            x: Forward velocity in m/s. Positive = forward, negative = backward.
            duration: How long to move in seconds.
        """
        return f"Moving at {x} m/s for {duration}s"

my_skill_container = MySkillContainer.blueprint
```

### System Prompts

| Robot | File | Variable |
|-------|------|----------|
| Go2 (default) | `dimos/agents/system_prompt.py` | `SYSTEM_PROMPT` |
| G1 humanoid | `dimos/robot/unitree/g1/system_prompt.py` | `G1_SYSTEM_PROMPT` |

Pass the robot-specific prompt: `McpClient.blueprint(system_prompt=G1_SYSTEM_PROMPT)`. The default prompt is Go2-specific; using it on G1 causes hallucinated skills.

### RPC Wiring

To call methods on another module, declare a `Spec` Protocol and annotate an attribute with it. The blueprint injects the matching module at build time — fully typed, no strings, fails at build time (not runtime) if no match is found.

```python
# my_module_spec.py
from typing import Protocol
from dimos.spec.utils import Spec

class NavigatorSpec(Spec, Protocol):
    def set_goal(self, goal: PoseStamped) -> bool: ...
    def cancel_goal(self) -> bool: ...

# my_skill_container.py
class MySkillContainer(Module):
    _navigator: NavigatorSpec   # injected by blueprint at build time

    @skill
    def go_to(self, x: float, y: float) -> str:
        """Navigate to a position."""
        self._navigator.set_goal(make_pose(x, y))
        return "Navigating"
```

If multiple modules match the spec, use `.remappings()` to resolve. Source: `dimos/spec/utils.py`, `dimos/core/coordination/blueprints.py`.

### Adding a New Skill

1. Pick the right container (robot-specific or `dimos/agents/skills/`).
2. `@skill` + mandatory docstring + type annotations on all params.
3. If it needs another module's RPC, use the Spec pattern.
4. Return a descriptive `str`.
5. Update the system prompt — add to the `# AVAILABLE SKILLS` section.
6. Expose as `my_container = MySkillContainer.blueprint` and include in the agentic blueprint.

---

## Testing

```bash
# Fast tests (default)
uv run pytest

# Include slow tests (CI)
./bin/pytest-slow

# Single file
uv run pytest dimos/core/test_blueprints.py -v

# Mypy
uv run mypy dimos/
```

`uv run pytest` excludes `slow`, `tool`, and `mujoco` markers. CI (`./bin/pytest-slow`) includes slow, excludes tool and mujoco. See `docs/development/testing.md`.

---

## Pre-commit & Code Style

Pre-commit runs on `git commit`. Includes ruff format/check, license headers, LFS checks.

**Always activate the venv before committing:** `source .venv/bin/activate`

Code style rules:
- Imports at top of file. No inline imports unless circular dependency.
- Use `requests` for HTTP (not `urllib`). Use `Any` (not `object`) for JSON values.
- Prefix manual test scripts with `demo_` to exclude from pytest collection.
- Don't hardcode ports/URLs — use `GlobalConfig` constants.
- Type annotations required. Mypy strict mode.

---

## `all_blueprints.py` is auto-generated

`dimos/robot/all_blueprints.py` is generated by `test_all_blueprints_generation.py`. After adding or renaming blueprints:

```bash
pytest dimos/robot/test_all_blueprints_generation.py
```

CI asserts the file is current — if it's stale, CI fails.

---

## Git Workflow

- Branch prefixes: `feat/`, `fix/`, `refactor/`, `docs/`, `test/`, `chore/`, `perf/`
- **PRs target `dev`** — never push to `main` or `dev` directly
- **Don't force-push** unless after a rebase with conflicts
- **Minimize pushes** — every push triggers CI (~1 hour on self-hosted runners). Batch commits locally, push once.

---

## Further Reading

- Module system: `docs/usage/modules.md`
- Blueprints: `docs/usage/blueprints.md`
- Visualization: `docs/usage/visualization.md`
- Configuration: `docs/usage/configuration.md`
- Testing: `docs/development/testing.md`
- CLI / dimos run: `docs/development/dimos_run.md`
- LFS data: `docs/development/large_file_management.md`
- Agent system: `docs/agents/`

---

## Agent Pipeline (Cursor + Codex)

This section describes the automated "request → code → verify → PR → review →
auto-merge" pipeline. It is **additive** to the rules above; nothing here
overrides anything stated earlier.

### Single source of truth

```bash
bash scripts/verify.sh
```

Both local agents and CI invoke this exact script. It runs:

1. `uv sync --all-extras --no-extra dds` (and `--no-extra cuda` on macOS).
2. `uv run pytest -q` — fast suite (excludes `slow`, `tool`, `mujoco` markers).
3. `uv run mypy dimos/` — strict type check.

The slow tier (`./bin/pytest-slow`) is owned by full CI and is intentionally
*not* part of `verify.sh`; the agent loop optimizes for fast feedback. PRs
that land must still pass the full slow CI before auto-merge.

### Roles

| Actor | Responsibility | How it is triggered |
|-------|----------------|---------------------|
| **Cursor local agent** | Plan, edit code, run `verify.sh`, push, open PR, enable auto-merge, monitor CI | User types `/ship <one-liner>` in Cursor chat |
| **Codex GitHub App** | Comment-only review on every PR (P0/P1 focus) | Auto on `pull_request: opened` |
| **Human (you)** | Review the PR, approve, do GitHub web one-time setup, run hardware regression | Manual |

The agent must NOT:
- push directly to `main` or `dev`,
- force-push any reachable remote branch,
- click GitHub web UI settings on the user's behalf,
- silence `verify.sh` or comment out tests to make CI green,
- commit anything that looks like a token / SSH key / password.

### `/ship` command flow

`.cursor/commands/ship.md` defines the canonical 11-step flow. Summary:

1. Branch off `origin/dev` with a `feat/`, `fix/`, `chore/`, etc. prefix.
2. Implement the change.
3. Run `bash scripts/verify.sh` locally — must pass.
4. Commit with conventional title.
5. Push to `origin`.
6. Open PR targeting `dev` via `gh api repos/dimensionalOS/dimos/pulls -X POST` (REST is more reliable than `gh pr create` immediately after push).
7. Enable auto-merge with squash strategy.
8. Wait for CI green and Codex review.
9. Auto-merge fires.
10. Delete local + remote branch.
11. Report back to the user.

---

## Codex Review Guidelines (P0 / P1 / P2 / P3)

> This section is read by the Codex GitHub App on every PR. It defines what
> Codex should treat as blocking, recommended-fix, optional, or noise for the
> dimos repository. Generic Codex defaults are not specific enough for a
> robotics codebase that drives real hardware.

### Severity levels

| Level | Meaning | What Codex does |
|-------|---------|-----------------|
| **P0** | Could damage hardware, break a deployment, or compromise the repo | Write **BLOCK** in the review; require fix before merge |
| **P1** | Likely to regress functionality, break tests, or violate strict mypy / pre-commit | Flag with "recommend fix" |
| **P2** | Code quality (readability, duplication, naming) | Optional comment, do not block |
| **P3** | Typos, comment spelling, personal style preference | **Do not flag** |

### P0 checklist (any hit → BLOCK)

1. **Secrets in the diff** — anything resembling an API key, OAuth token, SSH
   private key, password, or hardcoded private IP that addresses a real
   robot dock.
2. **Direct push / force push to protected branches** — commits on the PR
   branch that bypass `dev` (e.g. base = `main` instead of `dev`), or any
   `--force` push history.
3. **Robot motion without bounds** — calls into `SportClient`, `LowCmd`,
   `Twist`, drone MAVLink velocity setpoints, or xArm joint commands that
   pass user/LLM-derived velocity, joint, or torque values **without
   `clamp`/`saturate`** to a documented physical limit.
4. **Memory / concurrency hazards** in C/C++ extensions or pybind code —
   use-after-free, dangling pointers, data race on a non-atomic shared
   variable, missing mutex on a callback-driven module stream.
5. **`verify.sh` neutered** — diff comments out a step in `scripts/verify.sh`,
   replaces a real check with `exit 0`, or removes `--strict` / `-q` flags
   in a way that hides failures.
6. **Mypy strict mode disabled** — adding `# type: ignore` on a public API,
   `--no-strict`, broad `Any` returns from a typed function, or a new
   `[mypy.overrides] ignore_errors = true` block without justification in
   the PR description.
7. **`@skill` invariants violated** — a method tagged with both `@rpc` and
   `@skill` (the decorator stacks are mutually exclusive per
   `dimos/agents/annotation.py`), or a `@skill` whose docstring is removed,
   whose params lack type annotations, or whose return type stops being
   `str`. These cause silent agent regressions, not loud crashes.

### P1 checklist (flag, do not auto-block)

1. **Critical `GlobalConfig` defaults changed** — `mcp_port`, `n_workers`,
   default `viewer`, `simulation`, `replay` flags. Flag and ask the PR to
   document why.
2. **Transport / topic renames** — LCM channel names, ROS2 topic strings,
   DDS topics referenced by string literals across modules; rename in one
   place, miss the other → silent disconnect.
3. **System prompt changes** — `dimos/agents/system_prompt.py`,
   `dimos/robot/unitree/g1/system_prompt.py`. Affects every LLM tool call;
   ask for an explicit before/after diff in the PR description.
4. **`all_blueprints.py` not regenerated** — any change adding/renaming a
   blueprint must be accompanied by `pytest dimos/robot/test_all_blueprints_generation.py`
   re-run. CI asserts this; if the diff touches blueprints but the
   `all_blueprints.py` diff is empty, flag.
5. **Tests skipped** — a new `@pytest.mark.skip`, `xfail`, or marker added
   to an existing test without a linked issue in the PR description.
6. **Pre-commit hook disabled** — `--no-verify` in any committed script,
   `.pre-commit-config.yaml` removing `ruff` / license header / LFS check.
7. **Inline imports added inside hot paths** — workspace rule says imports
   at top of file unless circular dependency; flag and ask for the
   circular-dep evidence.
8. **Hardcoded ports / URLs** — code style rule says use `GlobalConfig`
   constants. New `localhost:9990` or `192.168.123.161` literals → flag.
9. **`requests` vs `urllib`** — workspace rule says use `requests`. New
   `import urllib` for HTTP → flag.

### P2 / P3 — do not flag

**P2 (optional, low priority):**
- Repeated code that would benefit from a helper but works correctly.
- Naming that is consistent within the touched file but not the whole repo.
- Suggestions to convert a `for`-loop to a comprehension when the loop is
  more readable in context.

**P3 (do not flag at all):**
- English / Chinese typos in comments and markdown.
- Trailing-whitespace, tab-vs-space (ruff handles it).
- Personal style preferences not encoded in `pyproject.toml` ruff config.
- Existing code outside the diff (do not expand the review surface).

### Review output format

When Codex posts a review, prefer this structure so a human can scan it
quickly:

```
## P0 (BLOCKING)
- <file:line> one sentence: why this can hurt hardware / repo

## P1 (recommend fix)
- <file:line> one sentence + suggested fix direction

## P2 (optional)
- <file:line> one sentence

## Risk assessment
- Hardware regression needed?  (yes / no / unsure)
- Touches `@skill` / system_prompt / GlobalConfig?  (yes / no)
- Touches transport / topic strings?  (yes / no)
```

### Triggering Codex actions beyond review

Comment on the PR:
- `@codex review` — re-run review explicitly (default behavior is automatic).
- `@codex review for security regressions` — re-review with a security focus.
- `@codex fix the CI failures` — Codex opens a follow-up branch with a fix.
- `@codex add unit test for <function>` — Codex writes tests on a new branch.

Do **not** use bare `@codex` (without `review`); it triggers a full cloud
task and consumes more quota than necessary for plain code review.
