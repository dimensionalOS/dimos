# Blueprints

## Motivation

Modules in a robotic system need to be able to exchange data with each other; e.g.,
the navigation module might need information from the relevant sensor modules.
But when developing such a system, we don't want to wire modules up to each other
in a manual, fragile way.

That's where blueprints come in. In DimOS, instead of manually connecting modules,
you can just write a *blueprint*,
a *declarative specification* for each module that describes, e.g., its informational needs;
declare that your system consists of this and that module;
and the blueprint system will handle the plumbing for you.

```python
# Define your modules

# Combine their blueprints
blueprint = autoconnect(
    ModuleA.blueprint(),
    ModuleB.blueprint(),
)
# Then build and run
coordinator = blueprint.build()
```

### Background on `Module`s

Before diving into blueprints, we need to first review Modules. Recall that there are two ways in which a Module can also in some broad sense communicate with or depend on other modules. First, when you define a [Module](./modules.md) -- when you write a *blueprint* for it -- you can declare what sorts of data it consumes and what sorts of data it produces:

```python
class ModuleA(Module):
    image: Out[Image] = None
    start_explore: Out[Bool] = None
```

In particular, these declarations are done with *streams*: `In[T]` for input and `Out[T]` for output, where `T` is the type variable for the type of data the stream carries.

Nothing about this required specifying exactly what other Modules this Module will be wired up to. But a Module can also depend on other Modules via the RPC system -- it can declare that it needs to be able to invoke certain methods of certain other Modules. And that does require specifying what those other Modules are:

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
```

### The blueprint system takes your declarative blueprints and does the wiring up for you

We've just seen how there are various ways in which Modules need to be wired up to each other, based on their blueprints. This sort of wiring is the job of the blueprint system.

That is, given the blueprints, the blueprint system automatically

- wires up streams between modules, selecting appropriate transports for the streams
<!-- Citation: blueprints.py:294-295 - Connections grouped by (remapped_name, conn.type) tuple -->
<!-- Citation: blueprints.py:228-234 - _get_transport_for checks transport_map, then auto-selects based on lcm_encode -->
- and provides Modules like `Greeter` with any dependencies it needs for RPC calls.

## Key Benefits

**Modularity** - A module can just declare what data it consumes and produces, without needing to know the specific module(s) they are communicating with via the streams.
<!-- Citation: blueprints.py:62-66 - ModuleBlueprint.connections stores typed stream specs extracted from annotations -->

**Composability** - You can make a complex system by composing modules.
<!-- Citation: blueprints.py:487-504 - autoconnect() merges multiple ModuleBlueprintSet instances -->

```python
basic = autoconnect(
    connection(),
    mapper(),
    astar_planner(),
    holonomic_local_planner(),
    behavior_tree_navigator(),
).global_config(n_dask_workers=4)

standard = autoconnect(
    basic,
    spatial_memory(),
    object_tracking(),
).global_config(n_dask_workers=8)

agentic = autoconnect(
    standard,
    llm_agent(),
    navigation_skill(),
    human_input(),
)
```
<!-- Citation: unitree_go2_blueprints.py:46-116 - Real pattern from codebase: basic->standard->agentic composition -->

**Reusability** - The same blueprint deploys to different environments by changing configuration, not module code.
<!-- Citation: blueprints.py:88-113 - transports() method allows override without changing blueprint structure -->

**Type Safety** - Connections are validated at build time. An `Out[Image]` can only connect to `In[Image]`.
<!-- Citation: blueprints.py:294-295 - Connection matching requires both name AND type to match -->

## How to build and run a blueprint

Suppose you have

- defined your modules and got the blueprints with `.blueprint`,
<!-- Citation: module.py:246-251 - Module.blueprint classproperty returns partial(create_module_blueprint) -->
- combined the blueprints with `autoconnect()`
<!-- Citation: blueprints.py:487 - autoconnect(*blueprints) merges ModuleBlueprintSet instances -->
- and optionally added your own `.transports()` and `.global_config()` configuration (more on this later):
<!-- Citation: blueprints.py:88-138 - transports() and global_config() builder methods -->

```python
# From the first skill tutorial
combined_blueprint = autoconnect(
    RobotCapabilities.blueprint,  # Provides speak
    Greeter.blueprint,            # Requires RobotCapabilities.speak
)
```

To build the composed blueprint -- to get the modules wired up, deploy them to workers, and start them -- you just need to call the `build()` method:

```python
module_coordinator = combined_blueprint.build(global_config=config)
```

This returns a `ModuleCoordinator` instance that manages all deployed modules.
<!-- Citation: blueprints.py:392-458 - build() transforms blueprint into ModuleCoordinator -->

<!-- Citation: blueprints.py:445-447 - Creates/merges GlobalConfig with overrides -->
<!-- Citation: blueprints.py:449-450 - ModuleCoordinator(global_config).start() initializes cluster -->
<!-- Citation: blueprints.py:452 - _deploy_all_modules() calls coordinator.deploy() -->
<!-- Citation: blueprints.py:453 - _connect_transports() sets up pub/sub connections -->
<!-- Citation: blueprints.py:454 - _connect_rpc_methods() wires inter-module calls -->
<!-- Citation: blueprints.py:456 - module_coordinator.start_all_modules() -->

### Running and shutting down

After `build()`, the system is already running. For long-running applications (e.g. an honest-to-goodness robot),
use `loop()` to keep the process alive:

```python
module_coordinator.loop()
```

This sleeps indefinitely until interrupted (Ctrl+C / SIGINT), whereupon it calls `stop()` to shut down gracefully.

Alternatively, when e.g. writing batch scripts, you can build the blueprint, do whatever you need to do, and just call `stop()` when you're done:

```python
coordinator = blueprint.build()
# ...do whatever you need to do
module_coordinator.stop()  # Clean up when finished
```

## How the blueprint system works, in more detail (Advanced)

Now that we've seen what blueprints are, at a high level, and how to build and run them,
we are in a position to dive into details that are helpful for building more complicated systems, such as

- how the blueprint system matches compatible streams, and what to do when stream names don't match
- how to override the default configuration

> [!TIP]
> Feel free to skip this section on first read -- you can always come back when you need the details.

### How the blueprint system matches compatible streams

In/Out streams are matched on the basis of *both* the stream name *and* the type of data associated with the stream.

In particular, when modules declare streams with matching names and types, the blueprint system assigns them the same transport instance. This shared transport is what enables data to flow between publishers and subscribers.

<!-- Citation: blueprints.py:330 - Groups connections by (remapped_name, conn.type) -->
<!-- Citation: blueprints.py:334-337 - Gets one transport for each group and assigns same transport to all connections in
that group -->

```python
class ProducerModule(Module):
    image: Out[Image] = None

class ConsumerModule(Module):
    image: In[Image] = None

blueprint = autoconnect(
    ProducerModule.blueprint(),
    ConsumerModule.blueprint(),
)
# These streams share a transport instance
# Data published by ProducerModule.image flows to ConsumerModule.image
```

Matching on not just the name but also the *type* prevents mistakes: an `Out[Temperature]` won't connect to `In[Pressure]` even if both are named `data`.

### Order and override behavior

For stream wiring, the order of the Module arguments to `autoconnect()` doesn't matter -- connections match by name and type regardless.

However, order *does* matter for blueprint composition. This lets you extend a base blueprint while overriding specific modules or configuration.

In particular, when the same module class appears multiple times, **later occurrences override earlier ones**:
<!-- Citation: blueprints.py:507-515 - _eliminate_duplicates processes in reverse so newer blueprints override older -->

```python
# Base configuration
base = autoconnect(
    mapper(voxel_size=0.5),
    planner(algorithm="basic"),
).global_config(n_dask_workers=4)

# Override planner params and worker count
advanced = autoconnect(
    base,
    planner(algorithm="advanced", lookahead=10),  # Replaces earlier planner
).global_config(n_dask_workers=8)

# Result: mapper(0.5) + planner(advanced, lookahead=10), 8 workers
```

This makes it easy to build configurations in layers, or create environment-specific variants (e.g. sim vs real), without repeating yourself.

> [!NOTE]
> The same last-wins rule applies to `.transports()`, `.global_config()`, and `.remappings()`: later specifications override earlier ones.

### Topic

A *topic* (or *topic name*) is an identifier that the transport layer uses to route messages between publishers and subscribers. Streams with matching (name, type) will share the same topic. For instance, if `ProducerModule` publishes to an `image` stream and `ConsumerModule` subscribes to an `image` stream (both of type `Image`), both will use the same topic -- `/image`, say -- and the transport ensures messages flow between them.

By default, the topic is a forward slash followed by the *name* of the stream. That is, the topic for the following `image` stream

```python
class ProducerModule(Module):
    image: Out[Image] = None
```

will be `/image`.

Streams with the same name must have the same type -- this is how the blueprint system knows to wire them together. If two streams share a name but have different types, `build()` raises a `ValueError`.

### What to do when stream names don't match (remapping)

Sometimes you need to rename a connection to match what other modules expect.
You can use the `remappings` method to do this:

<!-- The following example and discussion is copied from dimos/core/README_BLUEPRINTS.md -->

```python
class ConnectionModule(Module):
    color_image: Out[Image] = None  # Outputs on 'color_image'

class ProcessingModule(Module):
    rgb_image: In[Image] = None     # Expects input on 'rgb_image'

# Without remapping, these wouldn't connect automatically
# With remapping, color_image is renamed to rgb_image
blueprint = (
    autoconnect(
        ConnectionModule.blueprint(),
        ProcessingModule.blueprint(),
    )
    .remappings([
        (ConnectionModule, 'color_image', 'rgb_image'),
    ])
)
```

After remapping:

- The `color_image` output from `ConnectionModule` is treated as `rgb_image`
- It automatically connects to any module with an `rgb_image` input of type `Image`
- The topic name becomes `/rgb_image` instead of `/color_image`

If you want to override the topic, you still have to do it manually:

```python
blueprint
.remappings([
    (ConnectionModule, 'color_image', 'rgb_image'),
])
.transports({
    ("rgb_image", Image): LCMTransport("/custom/rgb/image", Image),
})
```

<!-- Citation: blueprints.py:290-295 - remapped_name used for grouping, original_name preserved for module use -->

## Configuration Management

### Transport

<!-- TODO: Add links to API references for  LCMTransport, pLCMTransport, lcm_encode, transports method -->

Recall that when modules declare streams with matching names and types, the blueprint system assigns them the same transport instance.

By default, `LCMTransport` is selected if the data type supports `lcm_encode` (LCM stands for 'Lightweight Communications and Marshalling'). Otherwise `pLCMTransport` is used; this serializes Python objects by pickling them.

But, as noted earlier, you aren't confined to the defaults -- you can choose whatever transport you like for a given (name, type) stream group with the `transports` method:

```python
blueprint = autoconnect(...)
expanded_blueprint = autoconnect(blueprint, ...)
final_blueprint = expanded_blueprint.transports({
    ("image", Image): pSHMTransport(
        "/go2/color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    ),
    ("start_explore", Bool): pLCMTransport(),
})
```

### Overriding global configuration

The choice of transport isn't the only config you can override.

Before we can see why, we need to remind ourselves of another fact about modules.
Each module can optionally take a `global_config` option in `__init__`. E.g.:

```python
class ModuleA(Module):

    def __init__(self, global_config: GlobalConfig | None = None):
        ...
```

If a global config is not explicitly supplied when the Module is defined, it will get loaded from an `.env` or environment variables when the blueprint is built.

But if you want to override the global config for a specific blueprint, you can do that with the `.global_config()` method.

<!-- TODO: Link to GlobalConfig API reference -->

<!-- Citation: blueprints.py:115-138 - global_config() method merges kwargs into global_config_overrides -->
<!-- Citation: blueprints.py:447 - global_config.model_copy(update=self.global_config_overrides) merges at build time -->

For instance, you might want to change the number of workers for a particular blueprint:

```python
blueprint = blueprint.global_config(n_dask_workers=8)
```

Or perhaps you want to graft different configs onto the same core blueprint for different environments:

```python
dev = blueprint.global_config(replay=True, n_dask_workers=1)
prod = blueprint.global_config(robot_ip="192.168.1.1", n_dask_workers=8)
```

## See also

<!-- TODO:
- link to API reference on ModuleBlueprint, ModuleBlueprintSet
- link to the first tutorial
-->
- [Modules](./modules.md)
- [Transport](./transport.md) - How data is transferred between modules
