# DimOS cheat-sheet (on-site reference)

> Grounded in the public `dimos` / `dimos-unitree` repos. **Verify exact names against the repo on Day 1** —
> APIs move fast. This is a mental model + quick reference, not gospel.

## The 5-layer architecture

```
Agent      → LangGraph agents (Claude / GPT / Gemini) that reason + pick tools
Skills     → RPC functions exposed to the agent as tools (via MCP)   ← YOU WORK HERE
Planning   → global A*  +  local VFH / pure-pursuit  +  costmaps
Perception → video, YOLO detect, depth projection, point clouds, VLM, tracking streams
Robot      → UnitreeGo2 interface (WebRTC + ROS2 middleware)
```

## Core vocabulary

| Term | Meaning |
|---|---|
| **Module** | A subsystem running in parallel, with typed `In[]` / `Out[]` streams |
| **Stream** | RxPY pub/sub channel (`color_image`, `cmd_vel`, `PersonTrackingStream`, …) |
| **Blueprint** | Declarative wiring of modules; `autoconnect()` matches streams by name + type |
| **RPC** | Method call between modules (`@rpc`) |
| **Skill** | An `@rpc` function surfaced to the LLM as a callable tool |
| **Transport** | Wire layer: LCM (default), SHM, DDS, ROS2 — overridable per blueprint |

## Minimal code shapes

```python
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import Twist
from dimos.msgs.sensor_msgs import Image

class RobotConnection(Module):
    cmd_vel: In[Twist]
    color_image: Out[Image]

    @rpc
    def start(self): ...

# Wire an agentic Go2: connection + skill server + LLM client
blueprint = autoconnect(
    go2_connection(),
    McpServer.blueprint(),   # exposes Skills as tools
    McpClient.blueprint(),   # the LLM agent
)
blueprint.build().loop()
```

A **custom Skill** = an `@rpc` method with a clear name + docstring (the LLM reads the docstring to decide
when to call it). Keep args simple (strings/floats), keep the docstring crisp, return a short result string.

## Go2 primitives (what the agent can already do)

- **Motion**: `move`, `reverse`, `spinLeft`, `spinRight`, `pose`
- **WebRTC tricks** (great for video): `FrontFlip`, `FrontPounce`, `FrontJump`, …
- **Higher skills**: `Navigate` (text→nav), `NavigateToObject` (vision), `FollowHuman`, `GetPose`
- **Nav stack**: A* global planning · VFH/pure-pursuit local · costmaps · frontier exploration · visual servoing
- **Spatial memory**: ChromaDB-backed spatio-temporal RAG · object localization · permanence

## CLI

```bash
dimos run unitree-go2-agentic --daemon   # launch the agentic stack
dimos agent-send "explore the room"      # natural-language command to the agent
dimos mcp call relative_move --arg forward=0.5   # call a skill directly
dimos status                             # health
dimos log -f                             # follow logs
```
MCP server defaults to **http://localhost:9990/mcp** (skills exposed as HTTP tools).

## Install

```bash
curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash
uv pip install 'dimos[base,unitree,sim]'
# repos:
git clone https://github.com/dimensionalOS/dimos
git clone https://github.com/dimensionalOS/dimos-unitree
```

## Rerun (your debugger + video source)

- 3D scene replay, **time-travel** (scrub backwards), **click-to-teleop**
- Use it to debug perception/nav live AND to record the 90s demo video
- Learn: logging points/images/transforms, the timeline scrubber

## Gotchas (the stuff that eats hackathon hours)

- **Transforms/TF** are the #1 bug source — wrong frame = robot drives the wrong way. Sanity-check in Rerun.
- **ROS2 + Docker setup** is non-trivial — do it before the event, not on Day 1.
- **Relocalization drift** — short routes, distinctive landmarks, re-map the demo area before shooting.
- **VLM/LLM latency** — describe only at stops; tiny prompts; cache. Don't call the VLM every frame.
- **Local-network only** — robot link is WebRTC/ROS2 on the venue LAN; bring your own laptop on that network.
- **LLM key + credits** — the agent is dead without an authenticated, funded key.

## "I'm stuck" checklist

1. `dimos status` + `dimos log -f` — is the stack even up?
2. Rerun open — are streams flowing? Is TF sane?
3. Can the skill be called directly via `dimos mcp call …`? (isolates agent vs. skill bug)
4. Is the agent *seeing* the skill? (docstring/registration)
5. Teleop still works? (isolates software vs. hardware)
