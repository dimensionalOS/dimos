# CLI Reference

The `dimos` CLI manages the full lifecycle of a DimOS robot stack — start, stop, inspect, and interact.

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
| `--replay-dir` | TEXT | `go2_sf_office` | Replay dataset directory name |
| `--replay-file` | TEXT | `None` | Path to a `.db` recording file for replay |
| `--record-path` | TEXT | `None` | Record module outputs to a `.db` file |
| `--new-memory` / `--no-new-memory` | bool | `False` | Clear persistent memory on start |
| `--viewer` | `rerun\|rerun-web\|rerun-connect\|foxglove\|none` | `rerun` | Visualization backend |
| `--n-workers` | INT | `2` | Number of forkserver workers |
| `--memory-limit` | TEXT | `auto` | Rerun viewer memory limit |
| `--mcp-port` | INT | `9990` | MCP server port |
| `--mcp-host` | TEXT | `127.0.0.1` | MCP server bind address |
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

1. `GlobalConfig` default → `simulation = False`
2. `.env` file → `DIMOS_SIMULATION=true`
3. Environment variable → `export DIMOS_SIMULATION=true`
4. Blueprint definition → `.global_config(simulation=True)`
5. CLI flag → `dimos --simulation run ...`

Environment variables and `.env` values must be prefixed with `DIMOS_`.

---

## Commands

### `dimos run`

Start a robot blueprint.

```bash
dimos run <blueprint> [<blueprint> ...] [--daemon] [--disable <module> ...]
```

| Option | Description |
|--------|-------------|
| `--config` `-c` | Path to read JSON config file from (options can be overriden with `-o` |
| `--daemon`, `-d` | Run in background (double-fork, health check, writes run registry) |
| `--disable` | Module class names to exclude from the blueprint |
| `--option`, `-o` | Provide an configuration option to the blueprint (e.g. `-o voxelgridmapper.voxel_size=1` |
| `--help` | Display the available configuration options that can be changed with `-o` or the config file |

```bash
# Foreground (Ctrl-C to stop)
dimos run unitree-go2

# Background (returns immediately)
dimos run unitree-go2-agentic --daemon

# Replay with Rerun viewer (legacy pickle datasets)
dimos --replay --viewer rerun run unitree-go2

# Record all sensor outputs to a database
dimos --record-path recording.db run unitree-go2

# Replay a recording (disables recorded modules, replays via LCM)
dimos --replay-file recording.db --viewer rerun-web run unitree-go2

# Real robot
dimos run unitree-go2-agentic --robot-ip 192.168.123.161

# Compose modules dynamically
dimos run unitree-go2 keyboard-teleop

# Disable specific modules
dimos run unitree-go2-agentic --disable OsmSkill WebInput
```

#### Record/replay end-to-end test

```bash
# 1) Start a stack and record all configured output streams
dimos --record-path /tmp/recording.db run unitree-go2

# 2) Stop the run, then replay the same file
dimos --replay-file /tmp/recording.db --viewer rerun-web run unitree-go2
```

For interactive topic selection, use `dimos recorder` instead of `--record-path`:

```bash
dimos recorder /tmp/recording.db
```

Then select streams with `space` and press `r` to start/stop recording.

When `--daemon` is used, the process:
1. Builds and starts all modules (foreground — you see errors)
2. Runs a health check (polls worker PIDs)
3. Forks to background, writes a run registry entry
4. Prints run ID, PID, log path, and MCP endpoint

#### Adding a New Blueprint

Define a module-level `Blueprint` variable and register it in `all_blueprints.py`:

```bash
pytest dimos/robot/test_all_blueprints_generation.py
```

This auto-generates the registry. See [blueprints](/docs/usage/blueprints.md) for composition details.

### `dimos status`

Show the running DimOS instance.

```bash
dimos status
```

Reads the run registry, verifies the PID is alive, and displays: run ID, PID, blueprint name, uptime, log path, and MCP port.

### `dimos stop`

Stop the running DimOS instance.

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

View logs from a DimOS run.

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

List all available blueprints.

```bash
dimos list
```

### `dimos show-config`

Print resolved GlobalConfig values and their sources.

```bash
dimos show-config
```

---

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

my_mcp_blueprint = autoconnect(
    my_robot_stack,
    McpServer.blueprint(),
    McpClient.blueprint(),
    my_skill_containers,
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

---

## Standalone Tools

These are installed as separate entry points and can be run directly without the `dimos` prefix.

### `humancli`

Interactive terminal for sending messages to the running agent.

```bash
humancli
```

### `lcmspy`

Monitor LCM messages in real time.

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

### `recorder`

Record and replay tool — a terminal UI for recording LCM streams, playing back recordings, and trimming.
It allows interactive selection of topics to record.

```bash
dimos recorder [filename]
```

Common controls:
- `space`: select/deselect current stream
- `r`: start/stop recording selected streams
- `p`: play/pause recording
- `s`: stop playback/recording
- `[` / `]`: mark trim in/out
- `t`: trim to marked range
- `d`: delete marked range

### `rerun-bridge`

Launch the Rerun visualization bridge as a standalone process (outside of a blueprint).

```bash
rerun-bridge
```

Also available as `dimos rerun-bridge`.

---

## File Locations

| Path | Contents |
|------|----------|
| `~/.local/state/dimos/runs/<run-id>.json` | Run registry (PID, blueprint, args, ports). Used by `status`/`stop`/`restart`. Cleaned up when processes exit. |
| `~/.local/state/dimos/logs/<run-id>/main.jsonl` | Structured logs (main process + all workers) |
| `.env` | Local config overrides (`DIMOS_ROBOT_IP=192.168.123.161`) |
