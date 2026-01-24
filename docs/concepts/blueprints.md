# Blueprints

Blueprints (`ModuleBlueprint`) are instructions for how to initialize a `Module`.

You don't typically want to run a single module, so multiple blueprints are handled together in `ModuleBlueprintSet`.

You create a `ModuleBlueprintSet` from a single module (say `ConnectionModule`) with:

```python session=blueprint-ex1
from dimos.core.blueprints import create_module_blueprint
from dimos.core import Module, rpc

class ConnectionModule(Module):
    def __init__(self, arg1, arg2, kwarg='value') -> None:
        super().__init__()

blueprint = create_module_blueprint(ConnectionModule, 'arg1', 'arg2', kwarg='value')
```

But the same thing can be accomplished more succinctly as:

```python session=blueprint-ex1
connection = ConnectionModule.blueprint
```

Now you can create the blueprint with:

```python session=blueprint-ex1
blueprint = connection('arg1', 'arg2', kwarg='value')
```

## Linking blueprints

You can link multiple blueprints together with `autoconnect`:

```python session=blueprint-ex1
from dimos.core.blueprints import autoconnect

class Module1(Module):
    def __init__(self, arg1) -> None:
        super().__init__()

class Module2(Module):
    ...

class Module3(Module):
    ...

module1 = Module1.blueprint
module2 = Module2.blueprint
module3 = Module3.blueprint

blueprint = autoconnect(
    module1(),
    module2(),
    module3(),
)
```

`blueprint` itself is a `ModuleBlueprintSet` so you can link it with other modules:

```python session=blueprint-ex1
class Module4(Module):
    ...

class Module5(Module):
    ...

module4 = Module4.blueprint
module5 = Module5.blueprint

expanded_blueprint = autoconnect(
    blueprint,
    module4(),
    module5(),
)
```

Blueprints are frozen data classes, and `autoconnect()` always constructs an expanded blueprint so you never have to worry about changes in one affecting the other.

### Duplicate module handling

If the same module appears multiple times in `autoconnect`, the **later blueprint wins** and overrides earlier ones:

```python session=blueprint-ex1
blueprint = autoconnect(
    module1(arg1=1),
    module2(),
    module1(arg1=2),  # This one is used, the first is discarded
)
```

This is so you can "inherit" from one blueprint but override something you need to change.

## How transports are linked

Imagine you have this code:

```python session=blueprint-ex1
from functools import partial

from dimos.core.blueprints import create_module_blueprint, autoconnect
from dimos.core import Module, rpc, Out, In
from dimos.msgs.sensor_msgs import Image

class ModuleA(Module):
    image: Out[Image]
    start_explore: Out[bool]

class ModuleB(Module):
    image: In[Image]
    begin_explore: In[bool]

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

```python session=blueprint-ex1
from dimos.core.transport import pSHMTransport, pLCMTransport

base_blueprint = autoconnect(
    module1(arg1=1),
    module2(),
)
expanded_blueprint = autoconnect(
    base_blueprint,
    module4(),
    module5(),
)
base_blueprint = base_blueprint.transports({
    ("image", Image): pSHMTransport(
        "/go2/color_image", default_capacity=1920 * 1080 * 3,  # 1920x1080 frame x 3 (RGB) x uint8
    ),
    ("start_explore", bool): pLCMTransport("/start_explore"),
})
```

Note: `expanded_blueprint` does not get the transport overrides because it's created from the initial value of `base_blueprint`, not the second.

## Remapping connections

Sometimes you need to rename a connection to match what other modules expect. You can use `remappings` to rename module connections:

```python session=blueprint-ex2
from dimos.core.blueprints import autoconnect
from dimos.core import Module, rpc, Out, In
from dimos.msgs.sensor_msgs import Image

class ConnectionModule(Module):
    color_image: Out[Image]  # Outputs on 'color_image'

class ProcessingModule(Module):
    rgb_image: In[Image]  # Expects input on 'rgb_image'

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

```python session=blueprint-ex2
from dimos.core.transport import LCMTransport
blueprint.remappings([
    (ConnectionModule, 'color_image', 'rgb_image'),
]).transports({
    ("rgb_image", Image): LCMTransport("/custom/rgb/image", Image),
})
```

## Overriding global configuration.

Each module can optionally take a `global_config` option in `__init__`. E.g.:

```python session=blueprint-ex3
from dimos.core import Module, rpc
from dimos.core.global_config import GlobalConfig

class ModuleA(Module):

    def __init__(self, global_config: GlobalConfig | None = None):
        ...
```

The config is normally taken from .env or from environment variables. But you can specifically override the values for a specific blueprint:

```python session=blueprint-ex3
blueprint = ModuleA.blueprint().global_config(n_dask_workers=8)
```

## Calling the methods of other modules

Imagine you have this code:

```python session=blueprint-ex3
from dimos.core import Module, rpc

class ModuleA(Module):

    @rpc
    def get_time(self) -> str:
        ...

class ModuleB(Module):
    def request_the_time(self) -> None:
        ...
```

And you want to call `ModuleA.get_time` in `ModuleB.request_the_time`.

To do this, you can request a link to the method you want to call in `rpc_calls`. Calling `get_time_rcp` will call the original `ModuleA.get_time`.

```python session=blueprint-ex3
from dimos.core import Module, rpc

class ModuleB(Module):
    rpc_calls: list[str] = [
        "ModuleA.get_time",
    ]

    def request_the_time(self) -> None:
        get_time_rpc = self.get_rpc_calls("ModuleA.get_time")
        print(get_time_rpc())
```

You can also request multiple methods at a time:

```python session=blueprint-ex3
class ModuleB(Module):
    def request_the_time(self) -> None:
        method1_rpc, method2_rpc = self.get_rpc_calls("ModuleX.m1", "ModuleX.m2")
```

## Alternative RPC calls

There is an alternative way of receiving RPC methods. It is useful when you want to perform an action at the time you receive the RPC methods.

You can use it by defining a method like `set_<class_name>_<method_name>`:

```python session=blueprint-ex3
from dimos.core import Module, rpc
from dimos.core.rpc_client import RpcCall

class ModuleB(Module):
    @rpc # Note that it has to be an rpc method.
    def set_ModuleA_get_time(self, rpc_call: RpcCall) -> None:
        self._get_time = rpc_call
        self._get_time.set_rpc(self.rpc)

    def request_the_time(self) -> None:
        print(self._get_time())
```

Note that `RpcCall.rpc` does not serialize, so you have to set it to the one from the module with `rpc_call.set_rpc(self.rpc)`

## Calling an interface

In the previous examples, you can only call methods in a module called `ModuleA`. But what if you want to deploy an alternative module in your blueprint?

You can do so by extracting the common interface as an `ABC` (abstract base class) and linking to the `ABC` instead one particular class.

```python session=blueprint-ex3
from abc import ABC, abstractmethod
from dimos.core.blueprints import autoconnect
from dimos.core import Module, rpc

class TimeInterface(ABC):
    @abstractmethod
    def get_time(self): ...

class ProperTime(Module, TimeInterface):
    def get_time(self):
        return "13:00"

class BadTime(TimeInterface):
    def get_time(self):
        return "01:00 PM"


class ModuleB(Module):
    rpc_calls: list[str] = [
        "TimeInterface.get_time", # TimeInterface instead of ProperTime or BadTime
    ]

    def request_the_time(self) -> None:
        get_time_rpc = self.get_rpc_calls("TimeInterface.get_time")
        print(get_time_rpc())
```

The actual method that you get in `get_time_rpc` depends on which module is deployed. If you deploy `ProperTime`, you get `ProperTime.get_time`:

```python session=blueprint-ex3
blueprint = autoconnect(
    ProperTime.blueprint(),
    # get_rpc_calls("TimeInterface.get_time") returns ProperTime.get_time
    ModuleB.blueprint(),
)
```

If both are deployed, the blueprint will throw an error because it's ambiguous.

## Defining skills

Skills have to be registered with `AgentSpec.register_skills(self)`.

```python session=blueprint-ex4
from dimos.core import Module, rpc
from dimos.core.skill_module import SkillModule
from dimos.protocol.skill.skill import skill
from dimos.core.rpc_client import RpcCall
from dimos.core.global_config import GlobalConfig

class SomeSkill(Module):

    @skill
    def some_skill(self) -> None:
        ...

    @rpc
    def set_AgentSpec_register_skills(self, register_skills: RpcCall) -> None:
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

```python session=blueprint-ex4
from dimos.core.skill_module import SkillModule
from dimos.protocol.skill.skill import skill

class SomeSkill(SkillModule):

    @skill
    def some_skill(self) -> None:
        ...
```

## Building

All you have to do to build a blueprint is call:

```python session=blueprint-ex4
module_coordinator = SomeSkill.blueprint().build(global_config=GlobalConfig())
```

This returns a `ModuleCoordinator` instance that manages all deployed modules.

### Running and shutting down

You can block the thread until it exits with:

```python session=blueprint-ex4
module_coordinator.loop()
```

This will wait for Ctrl+C and then automatically stop all modules and clean up resources.
