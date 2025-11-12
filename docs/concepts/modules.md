# Modules

## What is a Module?

A Module is a **distributed, communicating unit of functionality** - the fundamental building block for robot applications in DimOS. Modules are self-contained actors that encapsulate specific behaviors (camera processing, navigation, AI reasoning) and communicate through well-defined interfaces.

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

Every major component is a Module: hardware drivers, perception algorithms, navigation planners, AI agents. This unified abstraction enables composing complex systems from independent pieces that communicate through typed streams and RPC calls.

## Purpose

The Module abstraction solves three critical challenges:

**Composability** - Modules connect in flexible topologies without enforced hierarchies. A camera module can feed multiple perception modules; an agent can coordinate several navigation modules. The peer-to-peer architecture provides complete flexibility.

<!-- Citation: blueprints.py:41-45 - ModuleBlueprint has no hierarchical constraints, just module class + connections -->

**Safety Through Boundaries** - Module boundaries enforce clear separation. LLM reasoning runs in agent modules, real-time control in hardware modules. Streams validate types at runtime, preventing data corruption. Each module manages its own resources and lifecycle independently.

<!-- Citation: module.py:113-126 - _close_module() shows independent resource management (loop, rpc, tf, disposables) -->
<!-- Citation: stream.py:87-99 - Stream class validates type parameter at initialization -->

**Distributed Execution** - Modules run as Dask actors across a cluster. Deploy computationally intensive perception on GPU workers while running navigation on CPU workers. The system handles network communication, serialization, and RPC automatically.

<!-- Citation: module.py:313-317 - set_ref() stores Dask actor reference and worker name -->
<!-- Citation: module.py:133-153 - __getstate__/__setstate__ enable serialization for Dask distribution -->

## Core Concepts

### Distributed Actors

Modules deploy as Dask actors, each with its own event loop for async operations, automatic serialization for cross-worker communication, and transparent RPC handling. You work with module references as local objects while Dask routes calls to appropriate workers.

<!-- Citation: module.py:91 - get_loop() creates dedicated event loop for each module -->
<!-- Citation: module.py:133-153 - Serialization implementation excludes unpicklable runtime attributes -->
<!-- Citation: module.py:283 - DaskModule stores self.ref = None (Actor reference) -->

### Typed Streams

Modules declare `In[T]` and `Out[T]` streams as class attributes. During initialization, type hints are introspected and stream instances created. Streams provide reactive, push-based data flow between modules, built on ReactiveX. Type validation ensures connected streams have compatible types.

<!-- Citation: module.py:295-310 - DaskModule.__init__ uses get_type_hints() to introspect In/Out type annotations and create stream instances -->
<!-- Citation: stream.py:26 - import reactivex as rx -->
<!-- Citation: stream.py:56-66 - pure_observable() and observable() implement ReactiveX interface -->

### RPC for Request-Response

While streams handle continuous data flow, `@rpc` decorated methods provide synchronous request-response communication. Modules declare dependencies in the `rpc_calls` list, which are bound during initialization. This makes dependencies explicit and enables validation before deployment.

<!-- Citation: module.py:85 - rpc_calls: list[str] = [] class variable declaration -->
<!-- Citation: module.py:268-275 - get_rpc_calls() retrieves from _bound_rpc_calls dictionary -->
<!-- Citation: module.py:104-105 - @rpc decorator on start() method shows usage pattern -->

### Skills for LLM Interaction

Every Module inherits from `SkillContainer` - any module can expose skills that AI agents discover and invoke. Skills bridge natural language commands and robot actions, maintaining clear boundaries between AI reasoning and robot control.

<!-- Citation: module.py:77 - class ModuleBase(Configurable[ModuleConfig], SkillContainer, Resource) -->

### Lifecycle

Modules follow a defined lifecycle: initialize with configuration, deploy to Dask workers, start processing, handle streams and RPC calls while running, then stop with graceful resource cleanup. The system automatically handles event loop creation, stream initialization, RPC server setup, and resource disposal.

<!-- Citation: module.py:89-102 - __init__ handles loop, RPC, and disposables setup -->
<!-- Citation: module.py:104-106 - @rpc decorated start() method -->
<!-- Citation: module.py:108-111 - stop() calls _close_module() then super().stop() -->
<!-- Citation: module.py:113-126 - _close_module() handles cleanup of all resources -->

## Design Principles

**Peer-to-Peer Architecture** - Modules are peers that connect in any topology, not layers or hierarchies. The architecture imposes no structural constraints.

<!-- Citation: blueprints.py:41-45 - ModuleBlueprint has no hierarchical structure, only connections -->

**Communication Boundaries** - Modules communicate only through defined interfaces: streams for reactive data flow, RPC for request-response, skills for agent-invocable actions. Modules never hold direct references to other instances, enforcing loose coupling and enabling distributed deployment.

<!-- Citation: module.py:283 - DaskModule only stores Actor reference (self.ref), not module instances -->

**Independence** - Each module is self-contained (own event loop, state, lifecycle), independently testable (runs in isolation with mock streams), and reusable (same module class works across robot platforms).

<!-- Citation: module.py:91 - Each module gets own event loop via get_loop() -->
<!-- Citation: module.py:89-102 - Isolated initialization with independent resources -->

**Validation** - Stream connections validate at runtime. Connecting mismatched types fails with clear errors, catching integration issues early.

<!-- Citation: stream.py:87-99 - Stream class stores and validates type parameter -->

## Relationship to Other Concepts

**Module + Stream** - Modules declare `In[T]` and `Out[T]` as class attributes. Type hints are introspected during initialization to create stream instances for reactive data flow.

<!-- Citation: module.py:295-310 - Type hint introspection creates In/Out stream instances -->
<!-- Citation: module.py:172-185 - inputs/outputs properties filter __dict__ for In/Out instances -->

**Module + Blueprint** - Blueprints provide declarative composition. `ModuleBlueprint` specifies module class, initialization args, and connections. `autoconnect()` matches streams by (name, type) to wire up graphs automatically.

<!-- Citation: blueprints.py:41-45 - ModuleBlueprint dataclass with module, connections, args, kwargs -->

**Module + Dask** - `DaskModule` manages actor references and worker assignments. Serialization enables sending instances across the network.

<!-- Citation: module.py:278-352 - DaskModule class implementation -->
<!-- Citation: module.py:313-317 - set_ref() stores actor reference and worker name -->

**Module + RPC** - The `rpc_calls` list declares dependencies, bound to callable RPC clients during initialization. RPC builds on Dask's actor communication.

<!-- Citation: module.py:85 - rpc_calls list declaration -->
<!-- Citation: module.py:268-275 - get_rpc_calls() binds to RPC clients -->

## Common Module Types

**Hardware Modules** - Interface with sensors and actuators (`ConnectionModule`, `CameraModule`)

**Perception Modules** - Process sensor data (`SpatialMemory`, `ObjectDetector`, `ObjectTracker`)

<!-- Citation: perception/spatial_perception.py:53-64 - SpatialMemory module implementation -->

**Navigation Modules** - Path planning and control (`NavigationInterface`, `BehaviorTreeNavigator`, `AstarPlanner`)

**Agent Modules** - AI reasoning and coordination (`BaseAgentModule`, `SkillCoordinator`)

---

Understanding how modules communicate through streams and RPC, deploy with Dask, and expose skills enables building sophisticated robot applications from composable pieces.
