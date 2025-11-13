# Skills

<!-- TODO: Add docstrings to dimos/protocol/skill/ (currently none exist):
     - skill() function: Add function docstring with Args section describing each parameter
     - Enum classes: Convert inline comments to proper docstrings on class and members
     - Examples to add: "ret: When to notify agent. Use Return.call_agent for interactive skills."
     - Could include: usage patterns, cross-references, performance notes (ThreadPoolExecutor, etc.)
-->

API for defining robot skills that agents can invoke. Skills are methods on Modules decorated with `@skill` that become discoverable and callable by AI agents.

## Decorator

::: dimos.protocol.skill.skill.skill
    options:
      show_source: true

## Configuration Types

### Return

::: dimos.protocol.skill.type.Return
    options:
      show_source: true

### Stream

::: dimos.protocol.skill.type.Stream
    options:
      show_source: true

### Output

::: dimos.protocol.skill.type.Output
    options:
      show_source: true

### Reducer

::: dimos.protocol.skill.type.Reducer
    options:
      show_source: true

## Related

- [Skills Concept](../concepts/skills.md) - High-level overview of the skill system including execution model and best practices
- [Modules Concept](../concepts/modules.md) - Module architecture that provides skills
