# Modules

## What is a Module?

A Module in DimOS is a **distributed, communicating unit of functionality** - the fundamental building block for constructing robot applications. Think of modules as self-contained actors that encapsulate specific behaviors (like camera processing, navigation, or AI reasoning) and communicate through well-defined interfaces.

<!-- Citation: module.py:77-352 - ModuleBase and DaskModule implementation -->

Every major component in DimOS is a Module: hardware drivers, perception algorithms, navigation planners, and AI agents. This unified abstraction means you can compose complex robotic systems from independent pieces that work together through typed streams and RPC calls.

```python
from dimos.core import Module, In, Out, rpc
from dimos.msgs.sensor_msgs import Image

class SpatialMemory(Module):
    """A module that builds semantic memory from camera streams."""

    # Declare typed input stream
    color_image: In[Image] = None

    @rpc
    def query_by_text(self, text: str, limit: int = 5) -> list[dict]:
        """Expose RPC method for other modules to call."""
        return self._search_memory(text, limit)
```

<!-- Citation: perception/spatial_perception.py:53-64 - Real SpatialMemory module with color_image: In[Image] declaration -->

## Why Modules Matter

The Module abstraction solves three critical challenges in building robot applications:

### Composability
Build systems like LEGO blocks. Modules can be mixed and matched in any topology - there's no enforced hierarchy. Connect a camera module to multiple perception modules, or have an agent module coordinate several navigation modules. The peer-to-peer architecture gives you complete flexibility.

<!-- Citation: blueprints.py:41-45 - ModuleBlueprint has no hierarchical constraints, just module class + connections -->

```python
# Modules connect in flexible topologies
camera = dimos.deploy(CameraModule)
spatial_memory = dimos.deploy(SpatialMemory)
object_tracker = dimos.deploy(ObjectTracking)

# Both perception modules can consume the same camera stream
spatial_memory.color_image.connect(camera.image)
object_tracker.color_image.connect(camera.image)
```

### Safety Through Boundaries
Module boundaries enforce clear separation between components. LLM-based reasoning runs in agent modules, while real-time control runs in hardware modules. Streams validate types at runtime, preventing data corruption. Each module manages its own resources and lifecycle independently.

<!-- Citation: module.py:113-126 - _close_module() shows independent resource management (loop, rpc, tf, disposables) -->
<!-- Citation: stream.py:87-99 - Stream class validates type parameter at initialization -->

### Distributed Execution
Modules run as Dask actors across a cluster, enabling horizontal scaling. Deploy a computationally intensive perception module on a GPU worker while running navigation on a CPU worker. The system handles network communication, serialization, and remote procedure calls automatically.

<!-- Citation: module.py:313-317 - set_ref() stores Dask actor reference and worker name -->
<!-- Citation: module.py:133-153 - __getstate__/__setstate__ enable serialization for Dask distribution -->

## Core Concepts

### Distributed Actors with Dask

Modules are deployed as distributed actors using Dask. Each module gets:
- Its own event loop for asynchronous operations
- Automatic serialization for cross-worker communication
- Transparent remote procedure call handling

<!-- Citation: module.py:91 - get_loop() creates dedicated event loop for each module -->
<!-- Citation: module.py:133-153 - Serialization implementation excludes unpicklable runtime attributes -->
<!-- Citation: module.py:98-100 - RPC server initialization with serve_module_rpc() -->

```python
from dimos import core

# Start a Dask cluster with 4 workers
dimos = core.start(4)

# Deploy modules across the cluster
camera = dimos.deploy(CameraModule, device_id=0)
perception = dimos.deploy(SpatialMemory, embedding_model="clip")

# Modules can run on different machines
camera.start()
perception.start()
```

The deployment abstracts away the complexity of distributed systems. You work with module references as if they were local objects, but Dask handles routing calls to the appropriate workers.

<!-- Citation: module.py:283 - DaskModule stores self.ref = None (Actor reference) -->

### Typed Streams for Data Flow

Modules declare input and output streams as typed class attributes. Streams provide reactive, push-based data flow between modules.

<!-- Citation: module.py:295-310 - DaskModule.__init__ uses get_type_hints() to introspect In/Out type annotations and create stream instances -->

```python
class PersonTrackingStream(Module):
    # Input streams
    video: In[Image] = None

    # Output streams
    tracking_data: Out[dict] = None

    @rpc
    def start(self):
        super().start()
        # Subscribe to input stream in start() method
        unsub = self.video.subscribe(self._process_frame)
        self._disposables.add(Disposable(unsub))

    def _process_frame(self, frame):
        # Process frame and publish to output
        result = self._process_tracking(frame)
        if result:
            self.tracking_data.publish(result)
```

Streams are built on ReactiveX, providing powerful operators for filtering, mapping, and combining data flows. Type hints are introspected at runtime to validate that connected streams have compatible types.

<!-- Citation: stream.py:26 - import reactivex as rx -->
<!-- Citation: stream.py:56-66 - pure_observable() and observable() implement ReactiveX interface -->

The stream topology is configured declaratively:
```python
# Explicit connection
tracker.video.connect(camera.image)

# Or use autoconnect to match by (name, type)
from dimos.core.blueprints import autoconnect
blueprint = autoconnect(camera, tracker, navigation)
```

### RPC for Request-Response

While streams handle continuous data flow, RPC methods provide synchronous request-response communication between modules.

```python
# Note: NavigationModule is illustrative - actual navigation uses NavigationInterface
class NavigationModule(Module):
    # Declare RPC dependencies
    rpc_calls = ["SpatialMemory.query_by_text"]

    @rpc
    def navigate_to(self, location_name: str) -> bool:
        """Navigate to a named location."""
        # Call another module's RPC method
        query = self.get_rpc_calls("SpatialMemory.query_by_text")
        results = query(location_name, limit=1)

        if results:
            self._set_goal(results[0]['pose'])
            return True
        return False
```

<!-- Citation: module.py:85 - rpc_calls: list[str] = [] class variable declaration -->
<!-- Citation: module.py:268-275 - get_rpc_calls() retrieves from _bound_rpc_calls dictionary -->

RPC methods are decorated with `@rpc` and can be called from any other module in the system. The `rpc_calls` list declares dependencies, which are bound during system initialization. This makes dependencies explicit and enables validation before deployment.

<!-- Citation: module.py:104-105 - @rpc decorator on start() method shows usage pattern -->

### Skills for LLM Interaction

Every Module inherits from `SkillContainer`, meaning any module can expose skills - methods that AI agents can discover and invoke.

<!-- Citation: module.py:77 - class ModuleBase(Configurable[ModuleConfig], SkillContainer, Resource) -->

```python
from dimos.protocol.skill import skill
from dimos.protocol.skill.type import Return

class NavigationModule(Module):
    @skill(ret=Return.call_agent)
    def navigate_to_location(self, location: str) -> bool:
        """Navigate to a named location in the environment.

        Agents can call this method through the skill system.
        """
        return self.navigate_to(location)
```

Skills bridge the gap between natural language commands and robot actions. An agent module can query available skills, reason about which to invoke, and execute them - all while maintaining clear boundaries between AI reasoning and robot control.

### Lifecycle Management

Modules have a well-defined lifecycle:

**Initialization** - Create the module with configuration parameters:
```python
memory = SpatialMemory(
    collection_name="kitchen_memory",
    embedding_model="clip",
    min_distance_threshold=0.5
)
```

<!-- Citation: perception/spatial_perception.py:66-82 - SpatialMemory.__init__ shows configuration parameters -->

**Deployment** - Send the module to a Dask worker:
```python
deployed_memory = dimos.deploy(memory)
```

**Starting** - Begin processing:
```python
deployed_memory.start()  # Calls the module's start() method
```

<!-- Citation: module.py:104-106 - @rpc decorated start() method -->

**Running** - The module processes streams and handles RPC calls until stopped.

**Stopping** - Clean shutdown:
```python
deployed_memory.stop()  # Graceful resource cleanup
```

<!-- Citation: module.py:108-111 - stop() calls _close_module() then super().stop() -->

The module system handles event loop creation, stream initialization, RPC server setup, and resource disposal automatically.

<!-- Citation: module.py:89-102 - __init__ handles loop, RPC, and disposables setup -->
<!-- Citation: module.py:113-126 - _close_module() handles cleanup of all resources -->

## How Modules Relate to Other Concepts

### Module + Stream
Modules declare `In[T]` and `Out[T]` streams as class attributes. During initialization, these type hints are introspected and stream instances are created. Modules can then subscribe to inputs and publish to outputs using ReactiveX observables.

<!-- Citation: module.py:295-310 - Type hint introspection creates In/Out stream instances -->
<!-- Citation: module.py:172-185 - inputs/outputs properties filter __dict__ for In/Out instances -->

### Module + Blueprint
Blueprints provide declarative system composition. A `ModuleBlueprint` specifies a module class, initialization arguments, and stream connections. The `autoconnect()` function matches streams by `(name, type)` pairs to wire up module graphs automatically.

<!-- Citation: blueprints.py:41-45 - ModuleBlueprint dataclass with module, connections, args, kwargs -->

### Module + Dask
Modules are deployed as Dask actors. The `DaskModule` class manages actor references and worker assignments. Serialization (`__getstate__`/`__setstate__`) enables sending module instances across the network.

<!-- Citation: module.py:278-352 - DaskModule class implementation -->
<!-- Citation: module.py:313-317 - set_ref() stores actor reference and worker name -->
<!-- Citation: module.py:133-153 - Serialization methods for network distribution -->

### Module + RPC
Modules expose `@rpc` decorated methods for synchronous communication. The `rpc_calls` list declares dependencies, which are bound to callable RPC clients during system initialization. The RPC system builds on Dask's actor communication.

<!-- Citation: module.py:85 - rpc_calls list declaration -->
<!-- Citation: module.py:268-275 - get_rpc_calls() binds to RPC clients -->

### Module + GlobalConfig
Modules receive system-wide configuration through the `global_config` parameter. This provides access to robot model information, transport settings, and other environment-specific configuration.

<!-- Citation: module.py:71-74 - ModuleConfig with rpc_transport and tf_transport -->

## Design Principles

### Peer-to-Peer Architecture
Modules are not organized in layers or hierarchies. They are peers that can connect in any topology. An agent module can communicate directly with a hardware module, or through intermediate perception and navigation modules. The architecture imposes no structural constraints.

<!-- Citation: blueprints.py:41-45 - ModuleBlueprint has no hierarchical structure, only connections -->

### Communication Boundaries
Modules communicate only through defined interfaces:
- **Streams** for reactive data flow
- **RPC** for request-response interactions
- **Skills** for agent-invocable actions

Modules never hold direct references to other module instances. This enforces loose coupling and enables distributed deployment.

<!-- Citation: module.py:283 - DaskModule only stores Actor reference (self.ref), not module instances -->

### Independence and Reusability
Each module is:
- **Self-contained** - Has its own event loop, state, and lifecycle
- **Independently testable** - Can run in isolation with mock streams
- **Reusable** - The same module class works across different robot platforms when combined with appropriate hardware modules

<!-- Citation: module.py:91 - Each module gets own event loop via get_loop() -->
<!-- Citation: module.py:89-102 - Isolated initialization with independent resources -->

### Type Safety
Stream connections are validated at runtime. Connecting an `Out[Image]` to an `In[PointCloud]` will fail with a clear error message. This catches integration issues early and makes the data flow explicit.

<!-- Citation: stream.py:87-99 - Stream class stores and validates type parameter -->

## Practical Usage Patterns

### Hardware Abstraction
Hardware modules wrap robot-specific interfaces and expose generic streams:

```python
from dimos.msgs.sensor_msgs import PointCloud2

# Robot-specific implementation
connection = dimos.deploy(ConnectionModule, ip=robot_ip)
connection.video.transport = core.LCMTransport("/video", Image)
connection.point_cloud.transport = core.LCMTransport("/points", PointCloud2)

# Generic perception modules work with any hardware
perception = dimos.deploy(SpatialMemory)
perception.color_image.connect(connection.image)
```

This pattern enables cross-embodiment: the same perception and navigation code works with different robots by swapping the hardware module.

### Perception Pipelines
Chain perception modules to build processing pipelines:

```python
camera = dimos.deploy(CameraModule)
detector = dimos.deploy(ObjectDetector)
tracker = dimos.deploy(ObjectTracker)
memory = dimos.deploy(SpatialMemory)

# Build pipeline with stream connections
detector.image.connect(camera.image)
tracker.detections.connect(detector.detections)
tracker.image.connect(camera.image)
memory.color_image.connect(camera.image)
```

Each module in the pipeline is independently developed and tested. Modules can be swapped (e.g., different detection algorithms) without changing the rest of the system.

### Agent-Robot Integration
Agent modules coordinate robot behavior through skills and RPC:

```python
from dimos.protocol.skill import skill
from dimos.protocol.skill.type import Return

class RobotAgent(Module):
    rpc_calls = [
        "NavigationModule.navigate_to",
        "SpatialMemory.query_by_text",
        "CameraModule.get_latest_frame"
    ]

    @skill(ret=Return.call_agent)
    def go_to_location(self, description: str) -> bool:
        """Move to a location described in natural language."""
        # Query memory for matching locations
        query_memory = self.get_rpc_calls("SpatialMemory.query_by_text")
        locations = query_memory(description, limit=3)

        if locations:
            # Command navigation
            nav = self.get_rpc_calls("NavigationModule.navigate_to")
            return nav(locations[0]['pose'])
        return False
```

The agent reasons about high-level goals while navigation and control modules handle real-time execution. The module boundaries create a safety layer between AI and hardware.

### Debugging and Introspection
Modules provide introspection capabilities:

```python
# Inspect module interface
print(module.io().result())
# Output:
# Inputs:
#   - color_image: In[Image]
# Outputs:
#   - none
# RPC Methods:
#   - query_by_text(text: str, limit: int) -> list[dict]
#   - add_location(name: str, pose: Pose) -> None

# Access streams programmatically
inputs = module.inputs   # Dict of In[T] streams
outputs = module.outputs # Dict of Out[T] streams
rpcs = module.rpcs       # List of RPC method names
```

<!-- Citation: module.py:172-185 - inputs/outputs properties implementation -->
<!-- Citation: module.py:200-244 - io() method provides module interface introspection -->

This makes it easy to understand module interfaces and trace data flow through the system.

## Common Module Types in DimOS

**Hardware Modules** - Interface with robot sensors and actuators
- `ConnectionModule` - Robot platform interface (Unitree, Stretch, etc.)
- `CameraModule` - Video stream provider

**Perception Modules** - Process sensor data
- `SpatialMemory` - Semantic memory with vector search
- `ObjectDetector` - Visual object detection
- `ObjectTracker` - Multi-object tracking across frames

<!-- Citation: perception/spatial_perception.py:53-64 - SpatialMemory module implementation -->

**Navigation Modules** - Path planning and control
- `NavigationInterface` - Abstract navigation interface
- `BehaviorTreeNavigator` - High-level navigation coordination
- `AstarPlanner` - Global path planning
- `HolonomicLocalPlanner` - Local trajectory control

**Agent Modules** - AI reasoning and coordination
- `BaseAgentModule` - LLM-based reasoning
- `SkillCoordinator` - Skill execution management

Each module type follows the same architectural patterns while implementing domain-specific logic.

## Best Practices

**Keep modules focused** - Each module should encapsulate one coherent piece of functionality. If a module is doing too much, split it into multiple modules connected by streams.

**Use streams for continuous data** - Camera frames, sensor readings, and state updates should flow through streams. This enables reactive processing and decouples producers from consumers.

**Use RPC for discrete requests** - Configuration changes, queries, and commands work well as RPC methods. The synchronous nature makes sequencing explicit.

**Declare dependencies explicitly** - List RPC dependencies in `rpc_calls` and declare stream types in class attributes. This makes the module interface clear and enables validation.

<!-- Citation: module.py:85 - rpc_calls list for declaring dependencies -->

**Test modules in isolation** - Write tests that instantiate a module without deploying to Dask. Mock input streams and verify output streams. This is faster than integration tests and catches logic errors early.

**Expose skills thoughtfully** - Not every RPC method should be a skill. Skills are the agent-facing API - expose high-level actions that make sense for natural language commands.

## Other resources

- [API Reference](../api/index.md) - Detailed API documentation

<!-- TODO: Add something like a 'To see modules in action, ...' -->

The Module concept is the foundation of DimOS. Understanding how modules communicate through streams and RPC, how they're deployed with Dask, and how they expose skills to agents will allow you to build sophisticated robot applications from composable pieces.
