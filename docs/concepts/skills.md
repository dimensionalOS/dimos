# Skills

## What is a Skill?

A Skill is the **bridge between AI reasoning and robot capabilities** - a method on a Module that agents can discover and invoke. Skills turn high-level intent ("navigate to the kitchen") into low-level execution (coordinate transformations, motor commands, sensor processing).

Think of skills as the robot's vocabulary: a catalog of actions that agents can discover, reason about, and invoke.
<!-- Citation: dimos/core/module.py:77 - ModuleBase inherits from SkillContainer -->

```python
from dimos.core import Module
from dimos.protocol.skill.skill import skill

class NavigationModule(Module):
    @skill()
    def navigate_to(self, location: str) -> str:
        """Navigate to a named location like 'kitchen'."""
        x, y, theta = self._lookup_location(location)
        self._set_navigation_goal(x, y, theta)
        return f"Navigating to {location}"
```

Instead of `PoseStamped(position=Vector3(3.5, -1.2, 0.0), orientation=Quaternion.from_euler(...))`, you write `navigate_to("kitchen")`.

## Purpose

**High-level semantic control** - Work with natural actions (`wave_hello()`) not motor commands (`set_joint_velocity(3, 0.5)`).

**Safety through constrained actions** - Agents invoke only tested behaviors, never arbitrary commands.

**Cross-platform portability** - Same skill interface (`pick_up()`) works across robots; implementations hide hardware differences.

**Composability** - Skills chain naturally: `navigate_to("kitchen")` → `detect_objects()` → `grasp("coffee_pot")`.

**Observable execution** - Every skill creates an audit trail for debugging autonomous systems.
<!-- Citation: dimos/protocol/skill/type.py:90-96 - MsgType enum defines message protocol for state tracking -->

## How Skills Work

**Every Module Can Have Skills** - Every Module can expose skills via `@skill`, because Modules inherit from SkillContainer. 
Skills automatically auto-register with agents when they inherit from `SkillModule`; they can then be invoked by agents as tool calls.
<!-- Citation: dimos/core/README_BLUEPRINTS.md:208-240 - Registration patterns -->
<!-- TODO: Add link to skills tutorial that explains this with examples -->
<!-- Citation: dimos/core/module.py:77,278,351 - Module inheritance chain -->

**The @skill Decorator** - Wraps methods with message protocol, state tracking (pending → started → completed), and automatic LLM schema generation.
<!-- Citation: dimos/protocol/skill/skill.py:65-113 - @skill decorator implementation -->
<!-- Citation: dimos/protocol/skill/type.py:90-96 - MsgType enum -->

**Distributed Execution** - Skills execute in background threads with full Module capabilities (RPC, streams). System handles threading, messaging, and result aggregation.
<!-- Citation: dimos/protocol/skill/skill.py:126-128,149-153 - ThreadPoolExecutor and SkillContainer -->
<!-- Citation: dimos/protocol/skill/skill.py:177-227 - call_skill implementation showing message flow -->

## Message Protocol and Execution Flow

When an agent calls a skill, here's what happens:

```
1. Agent decides to call skill based on LLM reasoning
2. SkillCoordinator creates SkillState tracking object
3. Skill executes in background thread pool
4. Skill publishes messages: start → [stream]* → ret/error
5. Coordinator aggregates results (using Reducer if streaming)
6. Agent receives final result and continues reasoning
```

<!-- Citation: dimos/protocol/skill/type.py:90-96 - MsgType enum: pending=0, start=1, stream=2, reduced_stream=3, ret=4, error=5 -->
<!-- Citation: dimos/protocol/skill/skill.py:191-227 - State machine implementation: start → [stream]* → ret/error -->

The message protocol enables:

- **Non-blocking execution** - Agents don't wait, skills run concurrently
- **Progress monitoring** - Long operations stream updates
- **Distributed deployment** - Skills run on different machines/processes
- **Audit trails** - Every execution is traceable

<!-- Citation: dimos/protocol/skill/skill.py:126-128 - Thread pool for non-blocking execution -->

## Best Practices

**Focus each skill** - One skill, one purpose; compose them for complexity.

**Return meaningful strings** - "Navigated to kitchen in 12 seconds" beats "ok" for LLM reasoning.

**Stream long operations** - Use generators with `stream=Stream.call_agent` for operations over a few seconds.
<!-- Citation: dimos/protocol/skill/type.py:34-41 - Stream enum for streaming support -->

**Write clear docstrings** - They become function descriptions LLMs see when choosing skills.
<!-- Citation: dimos/protocol/skill/schema.py - function_to_schema() extracts docstrings -->

**Handle errors gracefully** - Return contextual error messages for agent recovery, not exceptions.

**Test independently** - Skills are Module methods testable in isolation.

## Summary

Skills bridge the gap between what users want ("go to the kitchen") and what robots understand (velocity commands, joint angles). They enable semantic control, safety, portability, composability, and observability.

Every Module can expose skills to Agents. Skills execute in background threads with message-based coordination for distributed execution.
<!-- Citation: dimos/protocol/skill/type.py:90-96 - Message protocol with MsgType enum -->

## Related Concepts

- **[Agent](agent.md)** - LLM-based reasoning systems that invoke skills
- **[Modules](modules.md)** - The distributed actors that provide skills
<!-- Citation: docs/concepts/agent.md, docs/concepts/modules.md - Documentation exists -->
