# Quickstart

DimOS is a modular framework for building agentive robots. In this quickstart, you'll learn the basics of DimOS by building an LLM agent that can make greetings.

## Installation

### Requirements

- Python 3.10+
- OpenAI API key in environment

### Install DimOS

```python
# TODO: Ideally, when this is released, this should be as simple as
# pip install dimos
```
<!-- TODO: Check what the installation directions should be, post-release; hopefully it will be this easy after the library gets published -->

## Define a skill

Suppose you have a robot with a speaker.

```python
from dimos.core.skill_module import SkillModule
from dimos.core.core import rpc
from dimos.protocol.skill.skill import skill

# See the Skills concept guide for more on SkillModule
class Robot(SkillModule):
    rpc_calls = []

    # In a real setting, there would also be things like a ConnectionModule
    # for the robot platform you are using
    @rpc
    def speak(self, text: str) -> str:
        print(f"[Robot] {text}")
        return f"SPEAK: {text}"
```

How can we wire up this `speak` capability to an LLM agent -- how can we go from this to an agentic robot that can make greetings by using that on-board speaker?

Answer: make a *skill* -- a method on a `Module` that's decorated with `@skill`, so that it gets turned into *tools* that an agent can call.

```python
class Greeter(SkillModule):
    rpc_calls = ["Robot.speak"]  # Declare dependency

    @skill()
    def greet(self, name: str = "friend") -> str:
        '''Greet someone by name.'''
        self.get_rpc_calls("Robot.speak")(f"Hello, {name}!")
        return f"Greeted {name}"
```

Notice that `Greeter` doesn't import `Robot` directly. Instead, it declares a dependency in `rpc_calls`, and the framework wires them together at runtime.

## Wire up and build the modules

Now we can call `llm_agent` to get an LLM agent module, combine it with our `Robot` and `Greeter` modules to get a [*blueprint*](concepts/blueprints.md) for the whole system, and then build it.

```python
from dotenv import load_dotenv

load_dotenv()

from dimos.agents2.agent import LlmAgent, llm_agent
from dimos.core.blueprints import autoconnect

dimos = (
    autoconnect(
        Robot.blueprint(),
        Greeter.blueprint(),
        llm_agent(system_prompt="You're a friendly robot. Use greet when asked to say hello."),
    )
    .global_config(n_dask_workers=1)
    .build()
)

print("System running!")
```

``` {title="Output"}
deployed: Robot-f970968e-... @ worker 0
deployed: Greeter-fe23b94c-... @ worker 0
deployed: LlmAgent-dc45564b-... @ worker 0
System running!
```

As part of this process, the blueprint system matches dependencies (e.g., `Greeter`'s need for `Robot.speak`) and converts `Greeter`'s `greet` skill to a tool for the LLM agent.

The system is now running. For long-running applications, you'd call `dimos.loop()` to keep it alive until Ctrl+C.

## Say hi to our agent

Time to say hi to our agent (in a real system, the robot's greetings would then be piped through the speakers):

```python
agent = dimos.get_instance(LlmAgent)
print(agent.query("Hi there!"))
```

``` {title="Output"}
Hello! How are you doing today?
```

> [!NOTE]
> Exactly what greeting the LLM will make will, of course, differ across runs.

```python
print(agent.query("Can you greet Alice as well?"))
```

``` {title="Output"}
[Robot] Hello, Alice!
Hello to Alice!
```

You now have a robot that you can ask -- in ordinary English -- for greetings!

## What you learned

You've seen the core DimOS pattern: define skills in modules, wire them together with the blueprint system, and let an LLM agent handle natural language requests.

## Next steps

### Tutorials

- [Build your first skill](tutorials/skill_basics/tutorial.md): A tutorial that explains how to build a skill -- and how the blueprint system works -- in more detail
- [Equip an agent with skills](tutorials/skill_with_agent/tutorial.md)
- [Build a multi-agent RobotButler](tutorials/multi_agent/tutorial.md): Build a multi-agent RoboButler system, where a planner agent coordinates specialist subagents.

### Concept guides

- [Blueprints](concepts/blueprints.md)
- [Agents](concepts/agent.md)
- [Modules](concepts/modules.md)
- [Transport](concepts/transport.md)
