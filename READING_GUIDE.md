# DimOS Code Reading Guide

A guided path through the DimOS codebase that follows the journey of a command:

**your text → agent → LLM → skill → topic → motors**

Start with the small foundations, then follow the flow. Each file is marked
**read fully** vs **skim/targeted** so you don't wade through 900-line files.

---

## Phase 1 — Foundations (the ROS2-like primitives)

Start here; everything else builds on these.

| # | File | Read | What to get from it |
|---|------|------|---------------------|
| 1 | `dimos/core/core.py` (70) | **full** | `@rpc` — makes a method callable across processes. Tiny; sets the pattern. |
| 2 | `dimos/core/stream.py` (279) | skim | `In[T]` / `Out[T]` and `.publish()` / `.subscribe()`. **These are your topics.** |
| 3 | `dimos/core/module.py` (852) | **skim — top ~150 lines + the `Module` class** | How a module declares `In`/`Out` attributes and `@rpc` methods. **This is your node.** Don't read all 852. |
| 4 | `dimos/core/coordination/blueprints.py` (300) | skim | `autoconnect()` — wires modules by matching `(name, type)`. **This is your launch file.** |

## Phase 2 — What the robot can do (skills)

| # | File | Read | What to get from it |
|---|------|------|---------------------|
| 5 | `dimos/agents/annotation.py` (152) | **full** | The `@skill` decorator. `skill()` at **:120–144**, `_make_skill` at **:72** — sets `__skill__=True` and builds `args_schema` from type hints. **Where a Python method becomes an LLM tool.** |
| 6 | `dimos/robot/unitree/unitree_skill_container.py` (317) | targeted | `relative_move` at **:210**, `execute_sport_command` at **:287**. Read the docstrings (what the LLM reads); note the publish/RPC to the connection at **:297**. Concrete atoms. |

## Phase 3 — The agent loop (the LLM part — the heart)

| # | File | Read | What to get from it |
|---|------|------|---------------------|
| 7 | `dimos/agents/system_prompt.py` (59) | **full** | The "Daneel" persona + rules prepended to every request. |
| 8 | `dimos/agents/mcp/mcp_server.py` (445) | targeted | The **menu + waiter**. `on_system_modules` **:381** (discovers all `@skill`s), `_handle_tools_list` **:94** (builds the menu JSON from docstrings+types), `_handle_tools_call` **:114** (routes a call to the skill). |
| 9 | `dimos/agents/mcp/mcp_client.py` (350) | targeted | The **agent itself**. `_fetch_tools` **:133** + `_mcp_tool_to_langchain` **:166** (menu → LangChain tools), `create_agent` **:222** (binds model + tools + system prompt), `_thread_loop`/`_process_message` **:306/:318** with `state_graph.stream(...)` **:326**. **This is "text → tool call."** |

## Phase 4 — How it's assembled & launched

| # | File | Read | What to get from it |
|---|------|------|---------------------|
| 10 | `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic.py` (27) + `_common_agentic.py` (30) | **full — both tiny** | The exact `autoconnect(...)` that snaps together robot + `McpServer` + `McpClient` + skills. Ties Phases 1–3 into one runnable stack. |
| 11 | `dimos/robot/cli/dimos.py` (976) | targeted | Entry points: `run` **:299** (launch), `agent_send_cmd` **:627** (publishes your text to `/human_input`), `mcp_list_tools` **:519** / `mcp_call_tool` **:552** (direct-call path that skips the LLM). |

## Phase 5 — Down to the motors (optional)

| # | File | Read | What to get from it |
|---|------|------|---------------------|
| 12 | `dimos/robot/unitree/go2/connection.py` (429) | targeted | `make_connection()` picks the backend — sim vs real — from config. |
| 13 | `dimos/robot/unitree/mujoco_connection.py` (371) | targeted | `move(twist)` at **:345** — turns a `Twist` into actual sim motion. Bottom of the stack. |

---

## If you only read five

`annotation.py` (5) → `unitree_skill_container.py` (6) → `mcp_server.py` (8) →
`mcp_client.py` (9) → `unitree_go2_agentic.py` (10).

That's the whole agentic loop, end to end.

## Tips while reading

- Keep `dimos mcp list-tools` output open next to `mcp_server.py:94` — you'll see
  the JSON being built from the exact docstrings/types in `unitree_skill_container.py`.
- Keep your running sim's launch-terminal logs open — lines like
  `SKILL relative_move ...` and `Discovered tools ...` are emitted from these
  exact functions, so you can match code to live output.

## Mental model (the one diagram)

```
YOU ──(dimos agent-send "text")──▶ /human_input topic ──▶ [Agent node + LLM] ──┐
                                                                                ├──▶ Skill (service) ──▶ /cmd_vel topic ──▶ motors
YOU ──(dimos mcp call relative_move --arg forward=1)────────────────────────────┘
```

- **Module** ≈ ROS2 node · **Stream/topic** ≈ ROS2 topic · **Skill** ≈ ROS2 service/action
- **Blueprint** ≈ launch file · `dimos run` ≈ `ros2 launch` · `dimos topic echo` ≈ `ros2 topic echo`
- The **agent** is just one client that calls skills based on English; `dimos mcp call`
  is you being that client directly (no LLM).
