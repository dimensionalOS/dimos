# CLI Tools

TUI utilities for debugging and interacting with dimos agents and skills.

---

## Overview

| Command | Purpose |
|---------|---------|
| `agentspy` | Monitor agent messages (Human/Agent/Tool/System) in real-time |
| `skillspy` | Track skill execution states and durations |
| `lcmspy` | LCM traffic statistics (bandwidth, frequency per topic) |
| `humancli` | Chat with agents from the terminal |

---

<!-- TODO: Add screenshots / terminal recordings etc for the following -->

## agentspy

Real-time monitor for agent message flow. Shows LangChain messages (HumanMessage, AIMessage, ToolMessage, SystemMessage) as they flow through the agent, color-coded by type:

- *Human* (green): User inputs
- *Agent* (yellow): LLM responses
- *Tool* (red): Skill execution results
- *System* (red): System prompts

Useful for debugging agent reasoning, inspecting prompts, and understanding the conversation flow.

```bash
agentspy
```

---

## skillspy

Real-time dashboard for skill execution monitoring. Shows skills as they execute, with state tracking (pending → running → completed/error), durations, and message counts.

Each row shows:
- *Call ID*: Unique identifier for the skill invocation
- *Skill Name*: Which skill is executing
- *State*: Current execution state (color-coded)
- *Duration*: How long the skill has been running
- *Messages*: Count of messages in the skill's state
- *Details*: Error messages or return values

```bash
skillspy
```

---

## lcmspy

Real-time LCM traffic statistics dashboard. Shows bandwidth and message frequency per topic, useful for profiling communication overhead and detecting message storms.

Each row shows:
- *Topic*: LCM channel name
- *Freq (Hz)*: Message frequency over the last 5 seconds
- *Bandwidth*: Data rate (auto-scaled to B/s, kB/s, MB/s)
- *Total Traffic*: Cumulative data since startup (auto-scaled to B, kB, MB, GB)

```bash
lcmspy
```

---

## humancli

IRC-style chat interface for interacting with dimos agents. Send messages and see agent responses, tool calls, and system messages in a familiar chat format.

```bash
humancli
```

---

## See also

Tutorials

- [Equip an agent with skills](../tutorials/skill_with_agent/tutorial.py)
- [Build a multi-agent RoboButler](../tutorials/multi_agent/tutorial.py): Uses notebook equivalent of `agentspy` to monitor multi-agent message flow

Concepts & API

- [Agent concept guide](../concepts/agent.md)
- [Agents API](./agents.md): LLM agents that these tools monitor

- [Skills concept guide](../concepts/skills.md)
- [Skills API](./skills.md)

- [Transport concept guide](../concepts/transport.md): discusses the LCM pub/sub that `lcmspy` monitors in more detail
