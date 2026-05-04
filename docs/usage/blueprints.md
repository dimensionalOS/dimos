# Blueprints

Blueprints (`BlueprintAtom`) are instructions for how to initialize a `Module`.

You don't typically want to run a single module, so multiple blueprints are handled together in `Blueprint`.

You create a `Blueprint` from a single module (say `ConnectionModule`) with:

```python session=blueprint-ex1
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig

class ConnectionConfig(ModuleConfig):
    arg1: int
    arg2: str = "value"

class ConnectionModule(Module):
    config: ConnectionConfig

blueprint = Blueprint.create(ConnectionModule, arg1=5, arg2="foo")
```

But the same thing can be accomplished more succinctly as:

```python session=blueprint-ex1
connection = ConnectionModule.blueprint
```

Now you can create the blueprint with:

```python skip session=blueprint-ex1
blueprint = connection('arg1', 'arg2', kwarg='value')
```

## Linking blueprints

You can link multiple blueprints together with `autoconnect`:

```python session=blueprint-ex1
from dimos.core.coordination.blueprints import autoconnect

class Config(ModuleConfig):
    arg1: int = 42

class Module1(Module):
    config: Config

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

`blueprint` itself is a `Blueprint` so you can link it with other modules:

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

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out, In
from dimos.msgs.sensor_msgs import Image

class ModuleA(Module):
    image: Out[Image]
    start_explore: Out[bool]

class ModuleB(Module):
    image: In[Image]
    begin_explore: In[bool]

module_a = partial(Blueprint.create, ModuleA)
module_b = partial(Blueprint.create, ModuleB)

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
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out, In
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

Each module includes the global config available as `self.config.g`. E.g.:

```python session=blueprint-ex3
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.global_config import GlobalConfig

class ModuleA(Module):
    def some_method(self):
        print(self.config.g.viewer)
        ...
```

The config is normally taken from .env or from environment variables. But you can specifically override the values for a specific blueprint:

```python session=blueprint-ex3
blueprint = ModuleA.blueprint().global_config(n_workers=8)
```

## Providing blueprint configuration to users

`Blueprint.config()` can be used to get a `pydantic.BaseModel` that can be used to
inspect or test configuration settings that can be passed to `Blueprint.build()`:

```python session=blueprint-ex1
# Validate config input
blueprint_args = {
    "module1": {"arg1": 5}
}
config = base_blueprint.config()
config(**blueprint_args)  # raises pydantic.ValidationError if args are incorrect
```

`dimos.robot.cli.dimos.arg_help()` is a helper function that will return a string
containing all details of these arguments (this is how the output is produced when
running `dimos run unitree-go2 --help`, for example):

```python session=blueprint-ex1
from dimos.robot.cli.dimos import arg_help

print(arg_help(base_blueprint.config(), base_blueprint))
```

<!--Result:-->
```
    module1:
      * module1.default_rpc_timeout: float (default: 120.0)
      * module1.frame_id_prefix: str | None (default: None)
      * module1.frame_id: str | None (default: None)
      * module1.arg1: int (default: 1)
    module2:
      * module2.default_rpc_timeout: float (default: 120.0)
      * module2.frame_id_prefix: str | None (default: None)
      * module2.frame_id: str | None (default: None)
```

Another function is `dimos.robot.cli.dimos.load_config_args()` which can create the
argument dict for users from a config file, environment variables and CLI arguments:


```python session=blueprint-ex1
from pathlib import Path

from dimos.robot.cli.dimos import load_config_args

config_path = Path.home() / "base-blueprint-config.json"
cli_args = ["module1.arg1=5"]
blueprint_args = load_config_args(base_blueprint.config(), cli_args, config_path)
# Test user input is valid
config(**blueprint_args)
# Then pass blueprint_args to ModuleCoordinator.build(...) (see coordinator docs)
```

## Calling the methods of other modules

Imagine you have this code:

```python session=blueprint-ex3
from dimos.core.core import rpc
from dimos.core.module import Module

class Drone(Module):

    @rpc
    def get_time(self) -> str:
        ...

class HelperModule(Module):
    def set_alarm_clock(self) -> None:
        ...
```

And you want to call `ModuleA.get_time` in `ModuleB.request_the_time`.

To do this, you can request a module reference.

```python session=blueprint-ex3
from dimos.core.core import rpc
from dimos.core.module import Module

class HelperModule(Module):
    drone_module: Drone

    def set_alarm_clock(self) -> None:
        print(self.drone_module.get_time_rpc())
```

But what if we want `HelperModule` to work for more than just `Drone`? For that we can use a spec.

```python session=blueprint-ex3
from dimos.spec.utils import Spec
from typing import Protocol

class Drone(Module):
    def get_time(self) -> str:
        return "1:00 PM"

class Car(Module):
    def get_time(self) -> str:
        return "2:00 PM"

# Your Spec
class AnyModuleWithGetTime(Spec, Protocol):
    def get_time(self) -> str: ...

class ModuleB(Module):
    device: AnyModuleWithGetTime

    def request_the_time(self) -> None:
        # autoconnect() will automatically find whatever module has a get_time() method
        print(self.device.get_time())
```

### Optional module references

If a dependency might not be present in every blueprint, annotate it as `SomeSpec | None = None`. The blueprint will try to resolve it but won't raise if no matching module is found:

```python session=blueprint-ex3
class ModuleC(Module):
    device: AnyModuleWithGetTime | None = None

    def maybe_get_time(self) -> str:
        if self.device is None:
            return "No clock available"
        return self.device.get_time()
```

## Defining skills

Skills are methods on a `Module` decorated with `@skill`. The agent automatically discovers all skills from launched modules at startup.

```python session=blueprint-ex4
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.agents.annotation import skill

class SomeSkill(Module):

    @skill
    def some_skill(self) -> str:
        """Description of the skill for the LLM."""
        return "result"
```

## Building

All you have to do to build a blueprint is call:

```python skip session=blueprint-ex4
from dimos.core.coordination.module_coordinator import ModuleCoordinator

module_coordinator = ModuleCoordinator.build(SomeSkill.blueprint())
module_coordinator.stop()
```

<!--Result:-->
```
16:30:00.119 [inf][dination/module_coordinator.py] Building the blueprint
16:30:00.133 [inf][dination/module_coordinator.py] Starting the modules
16:30:01.320 [inf][ation/worker_manager_python.py] Worker pool started. n_workers=2
16:30:01.321 [inf][ation/worker_manager_python.py] Shutting down all workers...
16:30:01.480 [inf][ation/worker_manager_python.py] All workers shut down
```

This returns a `ModuleCoordinator` instance that manages all deployed modules.

### Running and shutting down

You can block the thread until it exits with:

```python skip session=blueprint-ex4
module_coordinator.loop()
```

This will wait for Ctrl+C and then automatically stop all modules and clean up resources.
