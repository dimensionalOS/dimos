# Agent

## What is an Agent?

An Agent is your robot's **brain** - an LLM-based reasoning system that turns natural language commands into robot actions. Give it "go to the kitchen" and it figures out which skills to call and when. It bridges what you want with how to execute it.

<!-- Evidence: dimos/agents2/agent.py:163-213 - Agent class with LLM integration, SkillCoordinator, message history -->
<!-- Evidence: dimos/agents2/spec.py:147 - AgentSpec inherits from Module -->

This is **neurosymbolic orchestration**: the LLM handles high-level reasoning (what to do) while skills handle execution (how). The LLM never controls motors or sensors - only decides which skills to call.

```python
from dimos.agents2.agent import llm_agent
from dimos.agents2.cli.human import human_input
from dimos.agents2.skills.navigation import navigation_skill
from dimos.core.blueprints import autoconnect
from dimos.robot.unitree_webrtc.unitree_go2_blueprints import basic

# Create an agentic robot system
blueprint = autoconnect(
    basic,                   # Hardware, navigation, mapping
    navigation_skill(),      # Exposes navigation as agent-callable skills
    llm_agent(               # The reasoning agent
        system_prompt="You are a helpful robot assistant."
    ),
    human_input()           # CLI for sending commands to the agent
)
```

<!-- Evidence: dimos/robot/unitree_webrtc/unitree_go2_blueprints.py:110-116 - Real "agentic" blueprint showing this pattern -->

## Purpose

Traditional robot programming requires manually coding every behavior. With agents, you describe **what** you want ("find the red ball") not **how**.

**Natural language interface** - Command robots in English, not code.

**Dynamic task decomposition** - Breaks "clean the kitchen" into steps: explore, identify objects, navigate to each, manipulate, verify.

**Context-aware reasoning** - Queries spatial memory and camera feeds to ground decisions in what the robot saw and where. Enables tasks like "go back to where you saw the red chair."

<!-- Evidence: dimos/agents2/agent.py:224 - history() combines system, history, and state messages for context -->

## The Agent Loop

The agent runs a continuous reasoning loop with asynchronous skill execution:

1. Receive query
2. Build context from conversation history + spatial memory
3. Call LLM with available tools
4. LLM decides which skills to call
5. Execute skills asynchronously
6. Wait for results (may include images)
7. Feed results back to LLM
8. Generate response
9. Loop until task completes

<!-- Evidence: dimos/agents2/agent.py:247-325 - Complete agent_loop() implementation showing this exact flow -->

The loop handles **long-running operations** without blocking. Navigation takes 30 seconds? The agent waits, then resumes reasoning with results.

<!-- Evidence: dimos/agents2/agent.py:304 - await coordinator.wait_for_updates() enables async waiting -->

Skills can **stream updates** back. A skill exploring an environment might yield periodic updates ("Found 3 objects so far...") keeping the agent informed.

<!-- Evidence: dimos/protocol/skill/skill.py:37-42 - Return enum documentation for passive vs call_agent -->
<!-- Evidence: dimos/protocol/skill/skill.py:44-49 - Stream enum for streaming results -->

## Key Concepts

### The llm_agent() Function

Add an agent using `llm_agent()`, which returns a blueprint that integrates with the composition system:

```python
from dimos.agents2.agent import llm_agent

# Create an agent blueprint
agent_bp = llm_agent(
    system_prompt="You are a warehouse robot. Focus on navigation and inventory tasks.",
    model="gpt-4o-mini",    # Model to use (default)
    provider="openai"       # LLM provider (default)
)
```

<!-- Evidence: dimos/agents2/agent.py:375 - llm_agent = LlmAgent.blueprint factory function -->
<!-- Evidence: dimos/agents2/spec.py:130-143 - AgentConfig dataclass with system_prompt, model, provider, model_instance fields -->

Agents are [Modules](./modules.md), so they fit into the distributed architecture and can run on any Dask worker.

<!-- Evidence: dimos/agents2/spec.py:147 - AgentSpec inherits from Module -->

### Skill Discovery

Compose an agent with skill modules using `autoconnect()` - the agent discovers available skills through the Module system:

```python
from dimos.core.module import Module
from dimos.protocol.skill.skill import skill, Return

class NavigationSkills(Module):
    """Module providing navigation capabilities."""

    rpc_calls = ["NavigationInterface.set_goal", "SpatialMemory.query_by_text"]

    @skill(ret=Return.call_agent)
    def navigate_with_text(self, query: str) -> str:
        """Navigate to a location by text description."""
        query_memory = self.get_rpc_calls("SpatialMemory.query_by_text")
        results = query_memory(query, n=1)

        if not results:
            return f"Could not find '{query}' in memory"

        set_goal = self.get_rpc_calls("NavigationInterface.set_goal")
        set_goal(results[0].pose)
        return f"Navigating to {query}"
```

<!-- Evidence: dimos/protocol/skill/skill.py:65-113 - @skill decorator implementation with Return, Stream, Reducer, Output parameters -->
<!-- Evidence: dimos/agents2/skills/navigation.py - NavigationSkillContainer example (imported in blueprints) -->

When you build the blueprint, the agent:

1. Finds all `@skill` decorated methods
2. Converts them to LLM tool definitions
3. Exposes them to the LLM

<!-- Evidence: dimos/agents2/agent.py:350-351 - get_tools() retrieves tools from coordinator -->
<!-- Evidence: dimos/protocol/skill/skill.py:96-105 - Skill decorator creates SkillConfig with function schema -->

**Any Module can provide skills** - no registration needed beyond the `@skill` decorator.

<!-- Evidence: dimos/core/module.py:77 - ModuleBase inherits from SkillContainer, so all modules can have skills -->

### State Management

Agents follow a one-way lifecycle - once stopped, they stay stopped:

```ascii
INITIALIZED → STARTED → RUNNING → STOPPED (terminal)
```

Stopped agents **cannot restart**. This prevents mixing old and new conversation contexts. To resume operations, create a fresh agent instance.

<!-- Evidence: dimos/agents2/agent.py:209-212 - stop() sets _agent_stopped = True -->
<!-- Evidence: dimos/agents2/agent.py:204-206 - start() does not reset _agent_stopped -->
<!-- Evidence: dimos/agents2/agent.py:250-256 - agent_loop() checks flag and returns early -->

This one-way pattern supports explicit state management - each agent instance represents a single conversation session with its own history and context.

## Blueprint Integration

Agents work with declarative blueprint composition:

```python
from dimos.core.blueprints import autoconnect

# Compose: hardware + skills + agent + interface
blueprint = autoconnect(
    basic,                      # Hardware, navigation, mapping
    navigation_skill(),         # Navigation skills
    llm_agent(
        system_prompt="You are an exploration robot."
    ),
    human_input()              # Command interface
)
```

<!-- Evidence: dimos/robot/unitree_webrtc/unitree_go2_blueprints.py:110-116 - Actual "agentic" blueprint -->

`autoconnect()` wires up streams, registers skills, binds RPC dependencies, and assigns transports.

<!-- Evidence: dimos/agents2/agent.py:342-348 - register_skills() called during blueprint composition -->

## Design Principles

### Separation of Reasoning and Execution

Agents handle high-level reasoning ("I need to go to the kitchen") while skills handle implementation ("navigate to coordinates x, y"). This separation simplifies the LLM's job, enables independent skill testing, and provides clear boundaries between symbolic and subsymbolic processing.

### Asynchronous Execution Model

The agent loop never blocks on skill execution. This enables concurrent execution when appropriate, streaming updates from long-running skills, graceful error handling, and real-time responsiveness.

<!-- Evidence: dimos/agents2/agent.py:247 - agent_loop is async def -->
<!-- Evidence: dimos/agents2/agent.py:304 - await coordinator.wait_for_updates() for non-blocking waiting -->

Skills run concurrently in a thread pool (max 50 workers), preventing blocking.

<!-- Evidence: dimos/protocol/skill/skill.py:126-129 - ThreadPoolExecutor with max 50 workers executes skills -->

### Composable Architecture

Agents compose with other modules through blueprints. You can swap LLM providers, add new skill modules, deploy across hardware platforms, and test with mock skills - all without code changes.

<!-- Evidence: dimos/agents2/spec.py:130-138 - AgentConfig allows provider/model swapping -->
<!-- Evidence: dimos/agents2/agent.py:342-348 - register_skills() allows dynamic skill addition -->

DimOS supports multiple LLM providers through Langchain - switching requires only configuration changes.

<!-- Evidence: dimos/agents2/agent.py:195-197 - init_chat_model usage with provider and model params -->
<!-- Evidence: dimos/agents2/spec.py:45-47 - Provider enum dynamically created from langchain's 20 supported providers -->

## Common Use Cases

**Exploration and Mapping** - Agent plans exploration pattern, navigates to waypoints, tags rooms in memory, reports findings.

**Object Search and Navigation** - Agent searches memory for target object, explores to locate it if not found, navigates to object's location, confirms arrival.

**Guided Tours and Explanations** - Agent navigates to key locations, describes what's at each, answers questions about equipment and procedures.

## Best Practices

**Use Return.call_agent for most skills** - Provides immediate feedback. Use `Return.passive` only when the agent doesn't need completion notification.

**Test skills independently** - Test skills in isolation before integrating with an agent. Simplifies debugging.

## How Agents Relate to Other Concepts

**Agent + Module** - Agents are [Modules](./modules.md), inheriting all module capabilities: streams, RPC, lifecycle management, and distributed deployment.

<!-- Evidence: dimos/agents2/spec.py:147 - AgentSpec inherits from Module -->

**Agent + Skills** - Skills from any module can be used. The `@skill` decorator exposes methods, and `SkillCoordinator` manages execution state.

<!-- Evidence: dimos/agents2/agent.py:175 - self.coordinator = SkillCoordinator() initialization -->

**Agent + Blueprint** - `llm_agent()` returns a blueprint composing with others via `autoconnect()`. This declarative pattern keeps systems readable and maintainable.

<!-- Evidence: dimos/agents2/agent.py:375 - llm_agent = LlmAgent.blueprint -->

**Agent + Memory** - Query spatial and semantic memory via RPC to ground reasoning in past observations. Enables episodic and location-aware reasoning.

<!-- Evidence: dimos/agents2/agent.py:224 - history() combines system, history, and state messages -->

<!-- Evidence: dimos/agents2/agent.py:22 - from langchain.chat_models import init_chat_model -->

## Related Concepts

- [Skills](./skills.md) - Methods that agents can discover and invoke
- [Modules](./modules.md) - The foundational abstraction that agents build upon

## API Reference

- [Agents API](../api/agents.md) - API reference for agent classes, functions, and configuration

## Summary

Agents orchestrate robot behavior through neurosymbolic orchestration - LLMs reason what to do while skills handle how. This separation makes robots controllable through natural language while maintaining safety boundaries between AI reasoning and low-level control.

As modules, agents fit naturally into the distributed architecture. Through dynamic skill discovery, the same agent works across robot platforms - just swap skill modules for each embodiment.
