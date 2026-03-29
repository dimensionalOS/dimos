# Agents

LLM agents run as native DimOS modules. They subscribe to camera, LiDAR, odometry, and spatial memory streams and they control the robot through skills.

## Architecture

```
Human Input ──→ Agent ──→ Skill Calls ──→ Robot
  (text/voice)     │         (RPC)
                   │
          subscribes to streams:
          color_image, odom, spatial_memory
```

**McpClient** (`dimos/agents/mcp/mcp_client.py`) is a `Module` with:
- `human_input: In[str]`: receives text from `humancli`, `WebInput`, or `agent-send`
- `agent: Out[BaseMessage]`: publishes agent responses (text, tool calls, images)
- `agent_idle: Out[bool]`: signals when the agent is waiting for input

The agent uses LangGraph with a configurable LLM. The default is `gpt-4o` and you need to provide an `OPENAI_API_KEY` environment variable. On startup, it discovers all `@skill`-annotated methods across deployed modules via RPC and exposes them as LangChain tools.

## Skills

Skills are methods decorated with `@skill` on any `Module`. The agent discovers them automatically at startup.

```python
from dimos.agents.annotation import skill
from dimos.core.module import Module

class MySkillContainer(Module):
    @skill
    def wave_hello(self) -> str:
        """Wave at the nearest person."""
        # ... robot control logic ...
        return "Waving!"
```

**Rules:**
- Parameters must be JSON-serializable primitives (`str`, `int`, `float`, `bool`, `list`, `dict`).
- Docstrings become the tool description the LLM sees. Write them clearly so the agent has sufficent context.
- The function must return a string or image which with be used by the agent to decide what to do next.

### Built-in Skills

| Skill | Module | Description |
|-------|--------|-------------|
| `relative_move(forward, left, degrees)` | `UnitreeSkillContainer` | Move robot relative to current position |
| `execute_sport_command(command_name)` | `UnitreeSkillContainer` | Unitree sport commands (sit, stand, flip, etc.) |
| `wait(seconds)` | `UnitreeSkillContainer` | Pause execution |
| `observe()` | `GO2Connection` | Capture and return current camera frame |
| `navigate_with_text(query)` | `NavigationSkillContainer` | Navigate to a location by description |
| `tag_location(name)` | `NavigationSkillContainer` | Tag current position for later recall |
| `stop_navigation()` | `NavigationSkillContainer` | Cancel current navigation goal |
| `follow_person(query)` | `PersonFollowSkill` | Visual servoing to follow a described person |
| `stop_following()` | `PersonFollowSkill` | Stop person following |
| `speak(text)` | `SpeakSkill` | Text-to-speech through robot speakers |
| `where_am_i()` | `GoogleMapsSkillContainer` | Current street/area from GPS |
| `get_gps_position_for_queries(queries)` | `GoogleMapsSkillContainer` | Look up GPS coordinates |
| `set_gps_travel_points(points)` | `GPSNavSkill` | Navigate via GPS waypoints |
| `map_query(query)` | `OsmSkill` | Search OpenStreetMap with VLM |

## MCP

All agentic blueprints use two modules: `McpServer` and `McpClient`.

* `McpServer` exposes the methods annotated with `@skill` as MCP tools. Any external client can connect to the server to use the MCP tools.
* `McpClient` has a LangGraph LLM which calls MCP tools from `McpServer`.

CLI access:

```bash
dimos mcp list-tools                                # List available skills
dimos mcp call relative_move --arg forward=0.5      # Call a skill
dimos mcp status                                    # Server status
```

## Input Methods

| Method | How it works |
|--------|-------------|
| `humancli` | Standalone terminal — type messages, see responses |
| `dimos agent-send "text"` | One-shot CLI command via LCM |
| `WebInput` | Web interface at localhost:7779 with optional Whisper STT |

## Models

| Config | Model | Notes |
|--------|-------|-------|
| Default | `gpt-4o` | Best quality, requires `OPENAI_API_KEY` |
| `ollama:llama3.1` | Local Ollama | Requires `ollama serve` running |
| Custom | Any LangChain-compatible | Set via `McpClient.blueprint(model="...")` |
