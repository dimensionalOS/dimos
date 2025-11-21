# Skills

## Motivation

Suppose your robot has certain capabilities -- e.g., it can move in certain ways, or play sounds through a speaker. How do you let an LLM agent control these capabilities?

Skills are how you do that: skills get turned into *tools* that agents can call.

Skills are often also defined at a *higher level of abstraction* than the robotic capabilities; e.g., a 'follow human' skill that uses computer vision data to control a robot. In this way, skills can be

* easier for agents to work with and reason about
* and hide or abstract over differences in the underlying hardware.

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
<!-- Citation: dimos/core/module.py:77 - ModuleBase inherits from SkillContainer -->

Finally, if there's information you want to get to an agent, you need to do that with skills -- more on this shortly.

## What is a skill?

At a high level, skills are wrappers over lower-level robot capabilities. But at a more prosaic level, a skill is just a method on a Module decorated with `@skill` that:

1. **Becomes an agent-callable tool** - The decorator generates an OpenAI-compatible function schema from the method signature and docstring
2. **Executes in background threads** - Skills run concurrently without blocking the agent
3. **Reports state via messages** - Each execution tracks state (pending → running → completed/error)

<!-- Citation: dimos/protocol/skill/skill.py:65-113 - @skill decorator implementation -->

> [!TIP]
> The docstring becomes the tool description LLMs see when choosing skills. Write it for an LLM audience: clear, concise, action-oriented.

## Basic usage

### Defining a simple skill

For a method on a `Module` to be discoverable by agents, it has to be decorated with `@skill()` and registered on the agent -- [see the 'equip an agent with skills' tutorial for more details](../tutorials/skill_with_agent/tutorial.md).

```python
from dimos.core import Module
from dimos.protocol.skill.skill import skill

class RobotSkills(Module):
    @skill()
    def speak(self, text: str) -> str:
        """Make the robot speak the given text aloud."""
        self.audio.play_tts(text)
        return f"Said: {text}"

    @rpc
    def set_LlmAgent_register_skills(self, register_skills: RpcCall) -> None:
        """Called by framework when composing with llm_agent().

        This method is discovered by convention during blueprint.build().
        """
        register_skills.set_rpc(self.rpc)
        register_skills(RPCClient(self, self.__class__))
```

### How skills reach agents

When you register a Module with an agent, the agent discovers its `@skill` methods and converts them into **tool schemas** that the LLM understands. Your method signature becomes the tool's parameters; your docstring becomes its description.

See these tutorials for examples:

* [Equip an agent with skills](../tutorials/skill_with_agent/tutorial.md).
* [Build a RoboButler multi-agent system](../tutorials/multi_agent/tutorial.md)

## Getting information to agents, in more detail

We've seen how any robot action you want an LLM to invoke needs to be a skill (this is also covered in detail in the first two skills tutorials).

But it's also worth stressing that agents -- or more precisely, `Module`s that subclass `Agent` -- can only "see" information that comes through skills. If you want the agent to know the current time, position, or sensor readings, expose that via a skill:

```python
@skill(stream=Stream.passive, reducer=Reducer.latest, hide_skill=True)
def current_time(self):
    """Provides current timestamp to the agent."""
    while True:
        yield str(datetime.datetime.now())
        time.sleep(1)
```
<!-- Citation: dimos/robot/unitree_webrtc/unitree_skill_container.py:100-106 -->

The `hide_skill=True` flag prevents the LLM from calling this directly, but the agent still receives timestamps whenever it processes responses. This pattern works for any background information: video streams, sensor telemetry, position tracking.

## Streaming skills

The `stream` parameter of the `@skill` decorator is worth being aware of, for long-running operations

If you have a long-running operation, you can use `Stream.call_agent` for the `stream` parameter of the `@skill` decorator to stream updates to the agent. This notifies the agent every time there is an update from the skill.

```python
@skill(stream=Stream.call_agent)
def navigate_to(self, location: str):
    """Navigate to location with progress updates."""
    yield f"Planning path to {location}..."
    path = self._plan_path(location)

    for i, waypoint in enumerate(path):
        self._move_to(waypoint)
        yield f"Reached waypoint {i+1}/{len(path)}"

    return f"Arrived at {location}"
```

The agent is apprized of each `yield`, and can take action if something goes wrong.

### Stream modes

* **`Stream.none`** (default): No streaming. Skill returns a single value.
* **`Stream.call_agent`**: Wakes the agent on every `yield`. Use for progress updates.
* **`Stream.passive`**: Accumulates values silently. See [Background data](#background-data-with-streamstreampassive).

<!-- Citation: dimos/protocol/skill/type.py:36-85 -->

## Background data with `stream=Stream.passive`

Sometimes you want to continuously collect data *without interrupting the agent's reasoning*. A camera feed shouldn't wake the agent on every frame. Position telemetry shouldn't flood updates.

In such cases, you probably want to use skills that have been initialized with `stream=Stream.passive` -- i.e., what the docs sometimes call *passive skills*. Such skills run in the background, and the agent only sees their data when it wakes for some other reason:

```python
@skill(stream=Stream.passive, output=Output.image, reducer=Reducer.latest)
def video_stream(self) -> Image:
    """Continuous camera feed, doesn't wake agent on every frame."""
    while True:
        frame = self.camera.capture()
        yield frame
```
<!-- Citation: dimos/hardware/camera/module.py:86-92 -->

When the agent wakes (for an active skill, human input, etc.), its snapshot includes the latest video frame—without having been interrupted for every frame in between.

> [!CAUTION]
> **Passive skills alone cannot keep the agent loop alive.** If only passive skills are running, the loop exits immediately.
> So, e.g., you might want to pair passive skills with other active skill(s):
>
> * Video stream (passive) + navigation task (active)
> * Sensor telemetry (passive) + human_input (active)
> * Position tracking (passive) + movement command (active)

<!-- Citation: dimos/protocol/skill/type.py:64-69 -->

## Best practices

**Return meaningful strings** - `"Navigated to kitchen in 12 seconds"` beats `"ok"` for LLMs.

**Write clear docstrings** - They become tool descriptions. Be specific about what the skill does and what parameters mean.
<!-- Citation: dimos/protocol/skill/schema.py - function_to_schema() extracts docstrings -->

**Handle errors gracefully** - Return contextual error messages for agent recovery, not raw exceptions.

**Monitor long-running skills** - Use `dimos skillspy` to watch skill execution in real-time. Skills are tracked in an execution database showing what's currently running and what has completed—invaluable for debugging navigation or other long operations. To see an example of this, check out the [skill basics tutorial](../tutorials/skill_basics/tutorial.md).

> [!WARNING]
> **Don't use both `@skill` and `@rpc` decorators on a single method** - The `@skill` wrapper can't be pickled for LCM transport. Use `@skill()` for agent tools, `@rpc` for module-to-module calls.

## Related concepts

* **[Agents](agent.md)** - LLM-based reasoning that invokes skills
* **[Modules](modules.md)** - The distributed actors that provide skills
* **[Blueprints](blueprints.md)** - Composing modules and skills into systems
<!-- TODO: Add links to API reference pages for skill decorator etc -->
