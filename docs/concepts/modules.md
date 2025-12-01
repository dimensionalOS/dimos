# Modules

## What is a `Module`?

A `Module` is a *distributed, communicating unit of functionality* -- the fundamental building block for robot applications in DimOS. Modules are self-contained actors that encapsulate specific behaviors (camera processing, navigation, AI reasoning) and communicate through well-defined interfaces.

<!-- Citation: module.py:77-352 - ModuleBase and DaskModule implementation -->

```python
from dimos.core import Module, In, Out, rpc
from dimos.msgs.sensor_msgs import Image

class SpatialMemory(Module):
    """Builds semantic memory from camera streams."""
    color_image: In[Image] = None  # Typed input stream

    @rpc
    def query_by_text(self, text: str, limit: int = 5) -> list[dict]:
        """Expose RPC method for other modules to call."""
        return self._search_memory(text, limit)
```

<!-- Citation: perception/spatial_perception.py:53-64 - Real SpatialMemory module with color_image: In[Image] declaration -->

Every major component is a Module: hardware drivers, perception algorithms, navigation planners, AI agents. This unified abstraction solves three critical challenges:

**Composability** - Modules connect in flexible topologies without enforced hierarchies. A camera module can feed multiple perception modules; an agent can coordinate several navigation modules.

<!-- Citation: blueprints.py:41-45 - ModuleBlueprint has no hierarchical constraints, just module class + connections -->

**Safety through isolation** - Because modules are mapped onto separate processes, every module has its own address space. Even if one module fails catastrophically (e.g. a segfault), it won't bring down the others.

<!-- Citation: module.py:113-126 - _close_module() shows independent resource management (loop, rpc, tf, disposables) -->

**Distributed execution** - Modules run as Dask actors across a cluster. Deploy computationally intensive perception on GPU workers while running navigation on CPU workers. The system handles network communication, serialization, and RPC automatically.

<!-- Citation: module.py:313-317 - set_ref() stores Dask actor reference and worker name -->
<!-- Citation: module.py:133-153 - __getstate__/__setstate__ enable serialization for Dask distribution -->

## Modules and other DimOS concepts

### Streams

When you define a Module, you can declare what sorts of data it consumes and produces:

```python
class ModuleA(Module):
    image: Out[Image] = None
    start_explore: Out[Bool] = None
```

In particular, these declarations are done with *streams*: `In[T]` for input and `Out[T]` for output, where `T` is the type variable for the type of data the stream carries.

Streams provide reactive, push-based data flow between modules, built on ReactiveX. The [blueprint system](./blueprints.md) validates that connected streams have compatible types at build time.

<!-- Citation: module.py:295-310 - DaskModule.__init__ uses get_type_hints() to introspect In/Out type annotations and create stream instances -->
<!-- Citation: stream.py:26 - import reactivex as rx -->
<!-- Citation: stream.py:56-66 - pure_observable() and observable() implement ReactiveX interface -->

### RPC system

We've seen how modules might be wired to other modules on the basis of the sorts of data it consumes and produces. But there's yet another way in which a Module can in some sense depend on other Modules: a Module can declare that it needs to be able to (synchronously) invoke certain methods of certain other Modules via RPC.

```python
class Greeter(Module):
    """High-level Greeter skill built on lower-level RobotCapabilities, from the first skill tutorial."""

    # Declares what this module needs from other modules -- in this case, from
    # another RobotCapabilities module that provides lower-level capabilities.
    rpc_calls = [
        "RobotCapabilities.speak",
    ]

    @skill()
    def greet(self, name: str = "friend") -> str:
        """Greet someone by name."""
        # ...
        # A skill that invokes RobotCapabilities.speak
        # See the first skill tutorial for more details.

    # ...

class RobotCapabilities(Module):
    """Low-level capabilities that our (mock) robot possesses."""

    @rpc
    def speak(self, text: str) -> str:
        """Speak text out loud through the robot's speakers."""
        # ...

    # ...
```

<!-- Citation: module.py:85 - rpc_calls: list[str] = [] class variable declaration -->
<!-- Citation: module.py:268-275 - get_rpc_calls() retrieves from _bound_rpc_calls dictionary -->
<!-- Citation: module.py:104-105 - @rpc decorator on start() method shows usage pattern -->

### Modules are containers for skills

Suppose your robot has certain capabilities; e.g. it can move in certain ways. *Skills* are how you'd let AI agents control and monitor such capabilities. As is explained in more detail [in the concept guide](./skills.md), skills are methods on a `Module` that are decorated with `@skill` that get turned into *tools* that AI agents can call. (See also the [Skill tutorials](../tutorials/index.md) for end-to-end examples.)

And crucially, any `Module` can expose skills that AI agents discover and invoke, in virtue of inheriting from `SkillContainer`.

<!-- Citation: module.py:77 - class ModuleBase(Configurable[ModuleConfig], SkillContainer, Resource) -->

## How Modules run (Advanced)

> [!TIP]
> Feel free to skip this section on first read.

### Distributed actors

Modules deploy as Dask actors, each with its own event loop for async operations, automatic serialization for cross-worker communication, and transparent RPC handling. Modules communicate exclusively through Dask Actor references rather than direct Python object references, which enables transparent distributed deployment—you work with module references as local objects while Dask routes calls to appropriate workers.

<!-- Citation: module.py:91 - get_loop() creates dedicated event loop for each module -->
<!-- Citation: module.py:133-153 - Serialization implementation excludes unpicklable runtime attributes -->
<!-- Citation: module.py:283 - DaskModule stores self.ref = None (Actor reference) -->

### Lifecycle

Modules follow a defined lifecycle: initialize with configuration, deploy to Dask workers, start processing, handle streams and RPC calls while running, then stop with graceful resource cleanup. The system automatically handles event loop creation, stream initialization, RPC server setup, and resource disposal.

<!-- Citation: module.py:89-102 - __init__ handles loop, RPC, and disposables setup -->
<!-- Citation: module.py:104-106 - @rpc decorated start() method -->
<!-- Citation: module.py:108-111 - stop() calls _close_module() then super().stop() -->
<!-- Citation: module.py:113-126 - _close_module() handles cleanup of all resources -->

## Common module types

**Hardware modules** - Interface with sensors and actuators (`ConnectionModule`, `CameraModule`)

**Perception modules** - Process sensor data (`SpatialMemory`, `ObjectDetector`, `ObjectTracker`)

<!-- Citation: perception/spatial_perception.py:53-64 - SpatialMemory module implementation -->

**Navigation modules** - Path planning and control (`NavigationInterface`, `BehaviorTreeNavigator`, `AstarPlanner`)

**Agent modules** - AI reasoning and coordination (`BaseAgentModule`, `SkillCoordinator`)

## See also

- [Blueprints](./blueprints.md)
- [Skills](./skills.md)
- [Agents](./agent.md)
