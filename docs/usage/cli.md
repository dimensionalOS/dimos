# CLI Reference

The `dimos` CLI manages the full lifecycle of a dimOS robot stack — start, stop, inspect, and interact.

## Global Options

Every [`GlobalConfig`](/docs/usage/configuration.md) field is available as a CLI flag. Flags override environment variables, `.env`, and blueprint defaults.

```bash
dimos [GLOBAL OPTIONS] COMMAND [ARGS]
```

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--robot-ip` | TEXT | `None` | Robot IP address |
| `--robot-ips` | TEXT | `None` | Multiple robot IPs |
| `--simulation` / `--no-simulation` | bool | `False` | Enable MuJoCo simulation |
| `--replay` / `--no-replay` | bool | `False` | Use recorded replay data |
| `--replay-db` | TEXT | `go2_bigoffice` | Replay memory SQLite database name |
| `--new-memory` / `--no-new-memory` | bool | `False` | Clear persistent memory on start |
| `--viewer` | `rerun\|none` | `rerun` | Visualization backend |
| `--rerun-open` | `native\|web\|both\|none` | `native` | How to open the Rerun viewer |
| `--rerun-web` / `--no-rerun-web` | bool | `False` | Serve the Rerun web viewer |
| `--n-workers` | INT | `2` | Number of forkserver workers |
| `--memory-limit` | TEXT | `auto` | Rerun viewer memory limit |
| `--mcp-port` | INT | `9990` | MCP server port |
| `--mcp-host` | TEXT | `127.0.0.1` | MCP server bind address |
| `--transport` | `lcm\|zenoh` | platform-dependent | Transport backend for streams, RPC, and TF. Defaults to `zenoh` on macOS, otherwise `lcm`. Set `DIMOS_TRANSPORT` (env var or `.env`) to switch every process at once. Standalone CLIs like `humancli`, `agentspy`, and `dtop`, which also accept `--transport`. |
| `--dtop` / `--no-dtop` | bool | `False` | Enable live resource monitor overlay |
| `--obstacle-avoidance` / `--no-obstacle-avoidance` | bool | `True` | Enable obstacle avoidance |
| `--detection-model` | `qwen\|moondream` | `moondream` | Vision model for object detection |
| `--robot-model` | TEXT | `None` | Robot model identifier |
| `--robot-width` | FLOAT | `0.3` | Robot width in meters |
| `--robot-rotation-diameter` | FLOAT | `0.6` | Robot rotation diameter in meters |
| `--planner-strategy` | `simple\|mixed` | `simple` | Navigation planner strategy |
| `--planner-robot-speed` | FLOAT | `None` | Planner robot speed override |
| `--mujoco-camera-position` | TEXT | `None` | MuJoCo camera position |
| `--mujoco-room` | TEXT | `None` | MuJoCo room model |
| `--mujoco-room-from-occupancy` | TEXT | `None` | Generate room from occupancy map |
| `--mujoco-global-costmap-from-occupancy` | TEXT | `None` | Generate costmap from occupancy |
| `--mujoco-global-map-from-pointcloud` | TEXT | `None` | Generate map from point cloud |
| `--mujoco-start-pos` | TEXT | `-1.0, 1.0` | MuJoCo robot start position |
| `--mujoco-steps-per-frame` | INT | `7` | MuJoCo simulation steps per frame |

### Configuration Precedence

Values cascade (later overrides earlier):

1. `GlobalConfig` default → `simulation = ""`
2. `.env` file → `SIMULATION=mujoco`
3. Environment variable → `export SIMULATION=mujoco`
4. Blueprint definition → `.global_config(simulation="mujoco")`
5. CLI flag → `dimos --simulation run ...`

Environment variables and `.env` values use the field name in uppercase, for example `ROBOT_IPS`.

---

## Commands

### `dimos run`

Start one or more robot blueprints. Built-in dimOS blueprints use bare names such as
`unitree-go2`; external blueprints installed from Python packages use namespaced names
such as `my-robot-stack.go2`.

```bash
dimos run <blueprint> [<blueprint> ...] [--daemon] [--disable <module> ...] [--<config-field> <value> ...]
```

| Option | Description |
|--------|-------------|
| `--config`, `-c` | Path to a JSON configuration file; dynamic flags override its values |
| `--daemon`, `-d` | Run in background (double-fork, health check, writes run registry) |
| `--disable` | Module class names to exclude from the blueprint |
| `--<config-field>` | Set a blueprint configuration field using its kebab-case name, for example `--voxel-size=1`; qualify ambiguous fields as `--voxelgridmapper.voxel-size=1` |
| `--help` | Display the run options and available blueprint configuration flags |

Dynamic values accept both `--field=value` and `--field value`. A shorthand is
available only when it identifies one active module. If two modules expose the
same field, the CLI reports the ambiguity and lists stable qualified forms such
as `--relocalizationmodule.map-file`. Global flags work on either side of
`run`; older wrapper-style overrides are no longer accepted.

```bash
# Foreground (Ctrl-C to stop)
dimos run unitree-go2

# Background (returns immediately)
dimos run unitree-go2-agentic --daemon

# Replay with Rerun viewer
dimos --replay --viewer rerun run unitree-go2

# Replay Big Office (on Linux use --transport=zenoh; on macOS Zenoh is default when installed)
dimos --transport=zenoh --dtop --replay --replay-db=go2_bigoffice run unitree-go2

# Real robot
dimos run unitree-go2-agentic --robot-ip 192.168.123.161

# Blueprint configuration (both value forms are accepted)
dimos run unitree-go2-relocalization --map-file recording_go2

# Compose modules dynamically
dimos run unitree-go2 keyboard-teleop

# Run an externally packaged blueprint
dimos run my-robot-stack.go2

# Compose built-in and external blueprints
dimos run unitree-go2 my-robot-stack.keyboard-teleop

# Disable specific modules
dimos run unitree-go2-agentic --disable OsmSkill WebInput
```

External blueprint names are always fully qualified as
`<canonical-distribution-namespace>.<external-local-blueprint-name>`. The namespace is
derived from the installed Python distribution name by lowercasing it and collapsing
runs of `-`, `_`, and `.` into `-`. The local blueprint name is the entry point name
and must be lowercase kebab-case, for example `keyboard-teleop`.

On macOS, heavy replay workloads can be unreliable over LCM UDP, so the default transport resolves to `zenoh`; you can still force either path explicitly with `--transport=lcm` or `--transport=zenoh`.

When `--daemon` is used, the process:
1. Builds and starts all modules (foreground — you see errors)
2. Runs a health check (polls worker PIDs)
3. Forks to background, writes a run registry entry
4. Prints run ID, PID, log path, and MCP endpoint

#### Adding a New Blueprint

For an in-repository dimOS blueprint, define a module-level `Blueprint` variable and
regenerate the built-in registry:

```bash
pytest dimos/robot/test_all_blueprints_generation.py
```

This auto-generates `dimos/robot/all_blueprints.py` for built-in blueprints. External
packages do not edit that file; they expose blueprints through Python package entry
points. See [blueprints](/docs/usage/blueprints.md) for composition and external
publishing details.

### `dimos shell`

Open an IPython session attached to the coordinator on the configured transport bus:

```bash skip
dimos shell
```

The command requires an interactive terminal. For scripts and automation, use
`Dimos.connect()` through the [Python API](/docs/usage/python-api.md) instead.
While attaching, the shell displays a waiting indicator and retries coordinator
discovery within the default five-second connection budget.

The shell starts with five names:

| Name | Purpose |
|------|---------|
| `app` | Connected `Dimos` instance; access modules and invoke RPCs directly |
| `guide()` | Reprint the shell's quick-start guide |
| `modules()` | Print module instances, classes, and RPC counts |
| `rpcs()` | Print every RPC's signature and docstring summary |
| `describe(value)` | Pretty-print a module or RPC's signature and documentation |

For example:

```python skip
modules()
guide()
rpcs("StressTestModule")
describe("StressTestModule.ping")
app.StressTestModule.ping()
```

For example, `describe("StressTestModule.echo")` prints:

```text
RPC: StressTestModule.echo
Signature: echo(message: str) -> str

Documentation:
Echo a message back to the caller.
```

Use `app.describe(...)` when you want the structured `ModuleInfo` or `RpcInfo`
record instead of formatted output.

Discovery is live, so modules loaded after attachment appear on the next
`modules()` or `rpcs()` call. Use `app.list_modules()` and `app.list_rpcs()` when
you want structured records. Exact instance names select one deployment when
multiple instances share a class.

RPC calls execute immediately against the running system. The shell does not filter
methods or ask for confirmation, so an RPC may move hardware or change lifecycle
state. Start with a non-hardware, simulation, or replay stack.

Exiting IPython closes only the shell's client connection. The coordinator and its
modules keep running. If the coordinator stops or restarts, calls fail visibly; start
a new `dimos shell` session to reconnect.

The shell normally displays the CLI run ID and blueprint. Coordinators launched
directly from Python have no run-registry metadata and are shown as
`unregistered coordinator`.

### `dimos status`

Show the running dimOS instance.

```bash
dimos status
```

Reads the run registry, verifies the PID is alive, and displays: run ID, PID, blueprint name, uptime, log path, and MCP port.

### `dimos stop`

Stop the running dimOS instance.

```bash
dimos stop [--force]
```

| Option | Description |
|--------|-------------|
| `--force`, `-f` | Immediate SIGKILL (skip graceful SIGTERM) |

Default behavior: SIGTERM → wait 5s → SIGKILL. Cleans up the run registry entry.

### `dimos restart`

Restart the running instance with the same original arguments.

```bash
dimos restart [--force]
```

| Option | Description |
|--------|-------------|
| `--force`, `-f` | Force kill before restarting |

Reads saved CLI args from the run registry, stops the current instance, then re-runs with the same arguments.

### `dimos log`

View logs from a dimOS run.

```bash
dimos log [OPTIONS]
```

| Option | Description |
|--------|-------------|
| `--follow`, `-f` | Follow log output (like `tail -f`) |
| `--lines`, `-n` | Number of lines to show (default: 50) |
| `--all`, `-a` | Show full log |
| `--json` | Raw JSONL output (for piping to `jq`) |
| `--run`, `-r` | Specific run ID (defaults to most recent) |

```bash
dimos log                    # last 50 lines, human-readable
dimos log -f                 # follow in real time
dimos log -n 100             # last 100 lines
dimos log --json | jq .event # raw JSONL, extract events
dimos log -r 20260306-143022-unitree-go2  # specific run
```

All processes (main + workers) write to the same `main.jsonl`. Filter by module:

```bash
dimos log --json | jq 'select(.logger | contains("RerunBridge"))'
```

### `dimos list`

List all available blueprints. Built-in and external blueprints are grouped separately;
external names are read from installed package metadata without importing their target
modules.

```bash
dimos list
```

Example output:

```text
Built-in blueprints:
  unitree-go2
  unitree-go2-agentic

External blueprints:
  my-robot-stack.go2
  my-robot-stack.keyboard-teleop
```

### `dimos show-config`

Print resolved GlobalConfig values and their sources.

```bash
dimos show-config
```

### `dimos cache clean`

Remove caches generated by dimOS, including downloaded robot assets, prepared
URDFs, cooked scene meshes, the ament index, and the auto-downloaded Deno
runtime. All of these live under the platform-specific dimOS cache directory.

```bash
dimos cache clean
```

The command does not remove logs, recordings, datasets, configuration, or
third-party model caches. It refuses to run while a dimOS blueprint is active.

Before deleting anything, the command displays the cache root and every
top-level entry currently present. It then asks for confirmation with a default
of `No`. Pass `--yes` to skip the prompt in automation.

### `dimos spy`

Universal transport spy: a live table of every topic on every pubsub transport (LCM, Zenoh, or both), with per-topic message rate, bandwidth, size, and liveness.

```bash
dimos spy                     # everything, all transports
dimos spy --transport zenoh   # filter to one transport (repeatable flag)
dimos lcmspy                  # deprecated alias for: dimos spy --transport lcm
```

## Agent & MCP Commands

### `dimos agent-send`

Send a text message to the running agent via LCM.

```bash
dimos agent-send "walk forward 2 meters"
```

Works with any agentic blueprint — does not require MCP. Publishes directly to the `/human_input` LCM topic.

### `dimos mcp`

Interact with the running MCP server. **Requires a blueprint that includes `McpServer`** — for example `unitree-go2-agentic`. The MCP server runs at `http://localhost:9990/mcp` by default (`--mcp-port` / `--mcp-host` to override).

To add MCP to a blueprint, include both `McpServer` (exposes skills as HTTP tools) and `McpClient.blueprint()` (LLM agent that fetches tools from the server):

```python
from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect

# Example wiring (replace # -ed with your stack and skill):
my_mcp_blueprint = autoconnect(
    # my_robot_stack,
    McpServer.blueprint(),
    McpClient.blueprint(),
    # my_skill_containers,
)
```

#### `dimos mcp list-tools`

List all available skills exposed by the MCP server.

```bash
dimos mcp list-tools
```

Returns JSON with tool names, descriptions, and parameter schemas.

#### `dimos mcp call`

Call a skill by name.

```bash
dimos mcp call <tool_name> [--arg key=value ...] [--json-args '{}']
```

| Option | Description |
|--------|-------------|
| `--arg`, `-a` | Arguments as `key=value` pairs (repeatable) |
| `--json-args`, `-j` | Arguments as a JSON string |

```bash
dimos mcp call relative_move --arg forward=0.5
dimos mcp call relative_move --json-args '{"forward": 2.0, "left": 0, "degrees": 0}'
dimos mcp call observe
dimos mcp call land
```

#### `dimos mcp status`

Show MCP server status — PID, uptime, deployed modules, skill count.

```bash
dimos mcp status
```

#### `dimos mcp modules`

List deployed modules and their skills.

```bash
dimos mcp modules
```

## Standalone Tools

These are installed as separate entry points and can be run directly without the `dimos` prefix.

### `humancli`

Interactive terminal for sending messages to the running agent.

```bash
humancli
```

### `lcmspy`

Deprecated alias for `dimos spy --transport lcm` (the LCM-only view of the spy). Prefer [`dimos spy`](#dimos-spy).

```bash
lcmspy
```

### `agentspy`

Monitor agent messages and tool calls.

```bash
agentspy
```

### `dtop`

Live resource monitor TUI — CPU, memory, and process stats. Can also be activated during a run with `--dtop`:

```bash
dimos --dtop run unitree-go2
```

Or run standalone:

```bash
dtop
```

### `rerun-bridge`

Launch the Rerun visualization bridge as a standalone process (outside of a blueprint).

```bash
rerun-bridge
```

Also available as `dimos rerun-bridge`.

## File Locations

| Path | Contents |
|------|----------|
| `~/.local/state/dimos/runs/<run-id>.json` | Run registry (PID, blueprint, args, ports). Used by `status`/`stop`/`restart`. Cleaned up when processes exit. |
| `~/.local/state/dimos/logs/<run-id>/main.jsonl` | Structured logs (main process + all workers) |
| `.env` | Local config overrides (`ROBOT_IP=192.168.123.161`) |
