# Agents API

The LLM-based Agents system lets you command any robot with natural language.

## Module: dimos.agents2

::: dimos.agents2

## Core Agent Function

The primary entry point for creating agent blueprints.

### llm_agent()

::: dimos.agents2.agent.llm_agent

## Agent Implementation

### Agent

The main agent class that handles LLM-based reasoning and skill orchestration.

::: dimos.agents2.agent.Agent

### LlmAgent

Specialized Agent subclass that automatically starts its processing loop on startup, designed for blueprint composition pattern.

::: dimos.agents2.agent.LlmAgent

## Base Classes

### AgentSpec

Abstract base class for implementing custom agents.

::: dimos.agents2.spec.AgentSpec

## Quick Deployment

### deploy()

::: dimos.agents2.agent.deploy

## Configuration

### AgentConfig

Configuration dataclass for agent initialization.

::: dimos.agents2.spec.AgentConfig

### Model

Enum of supported LLM models across providers.

::: dimos.agents2.spec.Model

### Provider

Enum of supported LLM providers.

::: dimos.agents2.spec.Provider

## Related

- [Agent concept](../concepts/agent.md) - High-level overview of the agent system and neurosymbolic orchestration
- [Skills API](./skills.md) - Methods that agents can discover and invoke
- [Modules concept](../concepts/modules.md) - Module architecture that agents build upon
