# Skills API

Skills let agents control and monitor robot capabilities. They are methods on `Module` classes decorated with `@skill()` that become LLM-callable tools. Each skill executes in a background thread and communicates state through a message protocol (pending → running → completed/error), with optional streaming modes (`Stream.call_agent` for progress updates, `Stream.passive` for background data accumulation).

---

## Core Decorator

The entry point for defining skills.

### skill

::: dimos.protocol.skill.skill.skill

---

## Skill Configuration

Configuration attached to decorated methods, used by SkillCoordinator to control execution.

### SkillConfig

::: dimos.protocol.skill.type.SkillConfig

---

## Configuration Enums

Values passed to the `@skill()` decorator to control behavior.

### Return

Controls how skill return values are delivered and whether they wake the agent.

::: dimos.protocol.skill.type.Return

### Stream

Controls how streaming skill outputs (generators/iterators) are handled.

::: dimos.protocol.skill.type.Stream

### Output

Presentation hint for how the agent should interpret skill output.

::: dimos.protocol.skill.type.Output

---

## Stream Processing

Reducers aggregate streaming values when `stream=Stream.passive` or `stream=Stream.call_agent`.

### Reducer

::: dimos.protocol.skill.type.Reducer

### make_reducer

Factory for creating custom reducer functions from simple aggregation logic.

::: dimos.protocol.skill.type.make_reducer

---

## Infrastructure

Base classes inherited by Modules. Most users don't interact with these directly.

### SkillContainer

::: dimos.protocol.skill.skill.SkillContainer

---

## Related

**Tutorials:**

- [Build your first skill](../tutorials/skill_basics/tutorial.md) — Defining and testing skills
- [Equip an agent with skills](../tutorials/skill_with_agent/tutorial.md) — Wiring skills to agents

**Concepts & API:**

- [Skills concept](../concepts/skills.md) — High-level overview including execution model and best practices
- [Modules concept](../concepts/modules.md) — Module architecture that provides skills
- [Agents API](./agents.md) — LLM agents that discover and invoke skills
