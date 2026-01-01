# Agents API

LLM-based reasoning systems that orchestrate robot behavior by invoking skills in response to natural-language commands. Agents manage long-running operations asynchronously and maintain conversation history across operations.

---

## Quick Start

The blueprint factory for composing agents with other modules.

### llm_agent

::: dimos.agents2.agent.llm_agent

---

## Core Classes

### Agent

::: dimos.agents2.agent.Agent

### LlmAgent

::: dimos.agents2.agent.LlmAgent

---

## Configuration

### AgentSpec

::: dimos.agents2.spec.AgentSpec

### AgentConfig

::: dimos.agents2.spec.AgentConfig

### Model

::: dimos.agents2.spec.Model

### Provider

::: dimos.agents2.spec.Provider

---

## Message Types

### AnyMessage

::: dimos.agents2.spec.AnyMessage

---

## Standalone Deployment

For quick prototyping without blueprint composition.

### deploy

::: dimos.agents2.agent.deploy

---

## Related

**Tutorials:**

- [Equip an agent with skills](../tutorials/skill_with_agent/tutorial.md) — Hands-on introduction to agents and skills
- [Build a multi-agent system](../tutorials/multi_agent/tutorial.md) — Coordinating multiple agents

**Concepts & API:**

- [Agent concept](../concepts/agent.md) — High-level overview and neurosymbolic orchestration patterns
- [Skills API](./skills.md) — Methods that agents discover and invoke
- [Modules concept](../concepts/modules.md) — Module architecture agents build upon
