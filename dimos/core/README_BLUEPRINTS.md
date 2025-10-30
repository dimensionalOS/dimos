# Blueprints

Blueprints (`ModuleBlueprint`) are instructions for how to initialize a `Module`.

You don't typically want to run a single module, so multiple blueprints are handled together in `ModuleBlueprintSet`.

You create a `ModuleBlueprintSet` from a single module (say `ConnectionModule`) with:

```python
blueprint = create_module_blueprint(ConnectionModule, 'arg1', 'arg2', kwarg='value')
```

But the same thing can be acomplished more succinctly as:

```python
connection = ConnectionModule.blueprint
```

Now you can create the blueprint with:

```python
blueprint = connection('arg1', 'arg2', kwarg='value')
```

## Linking blueprints

You can link multiple blueprints together with `autoconnect`:

```python
blueprint = autoconnect(
    module1(),
    module2(),
    module3(),
)
```

`blueprint` itself is a `ModuleBlueprintSet` so you can link it with other modules:

```python
expanded_blueprint = autoconnect(
    blueprint,
    module4(),
    module5(),
)
```

Blueprints are frozen data classes, and `autoconnect()` always constructs an expanded blueprint so you never have to worry about changes in one affecting the other.

### Duplicate module handling

If the same module appears multiple times in `autoconnect`, the **later blueprint wins** and overrides earlier ones:

```python
blueprint = autoconnect(
    module_a(arg1=1),
    module_b(),
    module_a(arg1=2),  # This one is used, the first is discarded
)
```

This is so you can "inherit" from one blueprint but override something you need to change.

## How transports are linked

Imagine you have this code:

```python
class ModuleA(Module):
    image: Out[Image] = None
    start_explore: Out[Bool] = None

class ModuleB(Module):
    image: In[Image] = None
    begin_explore: In[Bool] = None

module_a = partial(create_module_blueprint, ModuleA)
module_b = partial(create_module_blueprint, ModuleB)

autoconnect(module_a(), module_b())
```

Connections are linked based on `(property_name, object_type)`. In this case `('image', Image)` will be connected between the two modules, but `begin_explore` will not be linked to `start_explore`.

## Topic names

By default, the name of the property is used to generate the topic name. So for `image`, the topic will be `/image`.

The property name is used only if it's unique. If two modules have the same property name with different types, then both get a random topic such as `/SGVsbG8sIFdvcmxkI`.

If you don't like the name you can always override it like in the next section.

## Which transport is used?

By default `LCMTransport` is used if the object supports `lcm_encode`. If it doesn't `pLCMTransport` is used (meaning "pickled LCM").

You can override transports with the `transports` method. It returns a new blueprint in which the override is set.

```python
blueprint = autoconnect(...)
expanded_blueprint = autoconnect(blueprint, ...)
blueprint = blueprint.transports({
    ("image", Image): pSHMTransport(
        "/go2/color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    ),
    ("start_explore", Bool): pLCMTransport(),
})
```

Note: `expanded_blueprint` does not get the transport overrides because it's created from the initial value of `blueprint`, not the second.

## Remapping connections

Sometimes you need to rename a connection to match what other modules expect. You can use `remappings` to rename module connections:

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

## Overriding global configuration.

Each module can optionally take a `global_config` option in `__init__`. E.g.:

```python
class ModuleA(Module):

    def __init__(self, global_config: GlobalConfig | None = None):
        ...
```

The config is normally taken from .env or from environment variables. But you can specifically override the values for a specific blueprint:

```python
blueprint = blueprint.global_config(n_dask_workers=8)
```

## Calling the methods of other modules

Imagine you have this code:

```python
class ModuleA(Module):

    @rpc
    def get_time(self) -> str:
        ...

class ModuleB(Module):
    def request_the_time(self) -> None:
        ...
```

And you want to call `ModuleA.get_time` in `ModuleB.request_the_time`.

You can do so by defining a method like `set_<class_name>_<method_name>`. It will be called with an `RpcCall` that will call the original `ModuleA.get_time`. So you can write this:

```python
class ModuleA(Module):

    @rpc
    def get_time(self) -> str:
        ...

class ModuleB(Module):
    @rpc # Note that it has to be an rpc method.
    def set_ModuleA_get_time(self, rpc_call: RpcCall) -> None:
        self._get_time = rpc_call
        self._get_time.set_rpc(self.rpc)

    def request_the_time(self) -> None:
        print(self._get_time())
```

Note that `RpcCall.rpc` does not serialize, so you have to set it to the one from the module with `rpc_call.set_rpc(self.rpc)`

## Defining skills

Skills have to be registered with `LlmAgent.register_skills(self)`.

```python
class SomeSkill(Module):

    @skill
    def some_skill(self) -> None:
        ...

    @rpc
    def set_LlmAgent_register_skills(self, register_skills: RpcCall) -> None:
        register_skills.set_rpc(self.rpc)
        register_skills(RPCClient(self, self.__class__))

    # The agent is just interested in the `@skill` methods, so you'll need this if your class
    # has things that cannot be pickled.
    def __getstate__(self):
        pass
    def __setstate__(self, _state):
        pass
```

Or, you can avoid all of this by inheriting from `SkillModule` which does the above automatically:

```python
class SomeSkill(SkillModule):

    @skill
    def some_skill(self) -> None:
        ...
```

## Building

All you have to do to build a blueprint is call:

```python
module_coordinator = blueprint.build(global_config=config)
```

This returns a `ModuleCoordinator` instance that manages all deployed modules.

### Running and shutting down

You can block the thread until it exits with:

```python
module_coordinator.loop()
```

This will wait for Ctrl+C and then automatically stop all modules and clean up resources.
