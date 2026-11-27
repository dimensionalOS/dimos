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

```python session=blueprint-ex1
blueprint = connection(arg1=5, arg2="foo")
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

`autoconnect()` returns a `MetaBlueprint`: an unresolved composition of concrete
blueprints, other metablueprints, and optional choices. You can compose it further:

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

Methods such as `.global_config()`, `.transports()`, `.remappings()`,
`.namespace()`, and `.hosted()` work on both types. A `MetaBlueprint` records
these operations and applies them, in order, after resolving choices. Each
operation applies only to the fragment where it was declared. Validation that
needs concrete modules also runs at that point.

Call `.solidify()` when you need to inspect the concrete modules or pass the
blueprint to `BlueprintConfigParser`:

```python session=blueprint-ex1
concrete_blueprint = expanded_blueprint.solidify()
```

Both types are frozen data classes. Solidification leaves the composition intact,
so it can be resolved repeatedly with different choices. A `MetaBlueprint` also
offers `.build(choices=...)`, which solidifies and calls `ModuleCoordinator.build()`.
`ModuleCoordinator.build()` and `.load_blueprint()` also accept a `MetaBlueprint`
and resolve its default choices at runtime. Keep exported compositions unresolved
so the CLI can pass `--choice` selections before building. Calling `.solidify()`
while defining an exported composition fixes its choices at import time.
As with other coordinator builds, call it under an `if __name__ == "__main__":`
guard when running a script.

## Choices

`ChoiceAtom` selects one fragment by name. Each alternative can be a concrete
`Blueprint` or a nested `MetaBlueprint`:

```python session=blueprint-choices
from dimos.core.coordination.blueprints import ChoiceAtom, autoconnect
from dimos.core.module import Module

class VisModule(Module):
    pass

class RaytracerModule(Module):
    pass

class VoxelMapper(Module):
    pass

mapping = autoconnect(
    VisModule.blueprint(),
    ChoiceAtom(
        name="PointClearing",
        default="raycasting",
        choices={
            "raycasting": RaytracerModule.blueprint(),
            "column_carving": VoxelMapper.blueprint(),
        },
    ),
).global_config(n_workers=4)

default_mapping = mapping.solidify()
voxel_mapping = mapping.solidify(choices={"PointClearing": "column_carving"})
```

Expose the unresolved `mapping` variable through the blueprint registry or a
package entry point to select alternatives at launch:

```bash
dimos run mapping
dimos run mapping --choice PointClearing=column_carving
```

Repeat `--choice NAME=OPTION` for multiple choices. Names and options are case
sensitive. A choice without a default requires an explicit selection. Invalid
options, unknown or inactive names, and repeated CLI selections are errors.
Nested choices are resolved only inside selected alternatives, so an unselected
branch's required choices do not need values. Distinct active choices must have
unique names; the same `ChoiceAtom` object can be reused across compositions.

The CLI solidifies before configuration parsing and dependency checks. Module
options and `--help` therefore describe the selected modules. Blueprint runtime
features, including hosted placement, operate on the resulting concrete blueprint.

## Publishing external blueprints

dimOS can discover runnable blueprints from installed Python packages. External
packages declare entry points in the `dimos.blueprints` group:

```toml
[project]
name = "my-robot-stack"

[project.entry-points."dimos.blueprints"]
go2 = "my_robot_stack.go2:go2_blueprint"
keyboard-teleop = "my_robot_stack.teleop:KeyboardTeleop"
```

After the package is installed in the same Python environment as dimOS, users can run
those blueprints by fully qualified name:

```bash
dimos run my-robot-stack.go2
dimos run unitree-go2 my-robot-stack.keyboard-teleop
```

External names are always `<canonical-distribution-namespace>.<external-local-blueprint-name>`:

- The namespace comes from the installed distribution name. dimOS lowercases it and
  collapses runs of `-`, `_`, and `.` into `-`, so `My_Robot.Stack` becomes
  `my-robot-stack`.
- The local blueprint name is the entry point name. It must be lowercase kebab-case
  matching `^[a-z0-9]+(-[a-z0-9]+)*$`, such as `go2` or `keyboard-teleop`.

Entry point targets may be either:

- a `Blueprint` or `MetaBlueprint` object, such as a module-level `go2_blueprint`; or
- a dimOS `Module` class, such as `KeyboardTeleop`, which dimOS converts with
  `.blueprint()`.

`dimos list` includes external names from package metadata without importing the target
modules. `dimos run my-robot-stack.go2` imports only the requested entry point target.

Remote coordinator resolution happens in the coordinator environment. If a client asks
a coordinator to load `my-robot-stack.go2`, the `my-robot-stack` package must be
installed where the coordinator performs name resolution; installing it only in the
client environment is not enough.

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
from dimos.msgs.sensor_msgs.Image import Image

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

Streams follow the configured backend, which defaults to Zenoh. Objects supporting `lcm_encode` use `ZenohTransport`; other objects use `pZenohTransport`. Selecting `--transport=lcm` uses the corresponding LCM transports.

You can override transports with the `transports` method. It returns a new blueprint in which the override is set.

```python session=blueprint-ex1
from dimos.core.transport_factory import make_transport

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
    ("image", Image): make_transport("go2/color_image", Image),
    ("start_explore", bool): make_transport("start_explore", bool),
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
from dimos.msgs.sensor_msgs.Image import Image

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

## Multi-robot blueprints (namespaces)

You can use namespaces to control several robot instances.

Use `.namespace(prefix, shared=...)` on a blueprint to isolate its modules
under a name prefix so several copies can coexist. Because blueprints are plain
Python values, a variable-size (and mixed-type) fleet is just a loop:

```python session=blueprint-ns
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out

class SensorConfig(ModuleConfig):
    ip: str = ""

class Sensor(Module):
    config: SensorConfig
    pointcloud: Out[str]

class AggregateMapper(Module):
    pointcloud: In[str]

robot_ips = ["10.0.0.1", "10.0.0.2"]

fleet = autoconnect(
    AggregateMapper.blueprint(),   # shared: one instance for the whole fleet
    *[
        Sensor.blueprint(ip=ip).namespace(f"robot{i}", shared={"pointcloud"})
        for i, ip in enumerate(robot_ips)
    ],
)
```

Inside a namespace everything is prefixed:

* instance names (`robot0/go2connection`),
* stream names and topics (`/robot0/lidar`),
* TF frames (`frame_id_prefix` comes from the instance name, ahead of any you set yourself),
* and the module's name within its run-scoped RPC address (`robot0/go2connection`).

Prefixed streams only connect within their namespace.

Stream names listed in `shared` are left unprefixed, so they connect globally.
That is how data crosses the boundary.

In the above example, both `Sensor` modules can send `pointcloud` data to the `AggregateMapper` module.

If you want to manually redirect to a namespaced name, you can use the namespaced prefix:  `.remappings([(FleetPlanner, "cmd_r0", "robot0/cmd_vel")])`

Module references (Specs or direct classes) resolve within the consumer's
namespace first, then enclosing namespaces, then globally, so per-robot
consumers bind to their own robot's providers.

Dynamic CLI configuration uses the canonical namespaced instance address:
`--robot0/go2connection.ip=10.0.0.5`. Environment configuration uses
`ROBOT0_GO2CONNECTION__IP=...`. `.remappings` accepts an instance name string
wherever it accepts a module class.

The downside of this is that the number of modules is fixed at blueprint creation. But it can be easily overcome by using a global variable. For example you can invoke with this:

```bash
ROBOT_IPS=10.0.0.1,10.0.0.2 dimos run blueprint-name
```

and construct `robot_ips` from the environment variable:

```python skip
robot_ips = (global_config.robot_ips or "").split(",")
```

For convenience `global_config.processed_robot_ips` is available which automatically splits the string and errors if no IPs are present.

This works in simulation too: adding `--simulation` starts one MuJoCo instance per robot and the IPs are ignored. For example, to run two simulated Go2 robots:

```bash
ROBOT_IPS=10.0.0.1,10.0.0.2 dimos --simulation run unitree-go2-multi
```

## Sharing a transport bus

Each coordinator gives its modules distinct RPC addresses. Two stacks can deploy the same
module class or instance name and call it independently. By default, `dimos run` claims the
shared `Coordinator` service before deploying modules. To run a companion stack on the same
bus, set `serve_coordinator_rpc=False` on that stack. `dimos shell` attaches to the stack
that serves `Coordinator`.

A stack started from Python with `ModuleCoordinator.build(...).loop()` claims the service in
`loop()`, after its modules have started and published on the shared stream topics. If the
service is taken, `loop()` stops the modules and raises `CoordinatorAlreadyRunningError`. Pass
`reserve_rpc_name=CoordinatorRPC.NAME` to `build()` to be rejected before any module starts,
as with `dimos run`.

Launches sharing a local runtime directory take turns claiming the service. Across hosts,
the check uses bus discovery, so simultaneous launches can both miss the other's claim.

An explicit `rpc_name` in a blueprint is an alias within its run. For a manual remote
reference, use the provider's resolved wire address from its `Coordinator/list_modules`
descriptor (`rpc_name`) or its local proxy (`remote_name`). Hosted fragments use the
run and host namespace agreed on by the fragment compiler.

Stream topics are shared by default so companion stacks can exchange camera images, commands,
and other data. Use blueprint namespaces or remappings to separate selected topics.

## Running across machines

`.hosted(...)` places part of a blueprint on another machine. See [Placing modules](/docs/usage/distributed/blueprints.md).

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

`BlueprintConfigParser` discovers the configuration exposed by a blueprint,
resolves configuration sources, and returns an immutable
`ParsedBlueprintConfig` for the coordinator:

```python session=blueprint-ex1
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser

concrete_blueprint = base_blueprint.solidify()
parser = BlueprintConfigParser(concrete_blueprint)
parsed = parser.parse(["--module1.arg1=5"])
```

Use the same parser to render blueprint-aware CLI help (this is how
`dimos run unitree-go2 --help` lists module configuration):

```python skip
print(parser.format_help())
```

The output includes each unambiguous shorthand alongside its stable qualified
address:

```text
Blueprint configuration options:
  ...
  --arg1, --module1.arg1 <int> (default: 1)
  --module1.default-rpc-timeout <float> (default: 120.0)
  --module2.default-rpc-timeout <float> (default: 120.0)
```

`parse()` combines a config file, recognized environment variables, programmatic
overrides, and dynamic CLI flags. To run the resulting configuration, save
`base-blueprint-config.json` next to your script (use `{}` for defaults) and run it
from that directory:

```python skip
from pathlib import Path

from dimos.core.coordination.module_coordinator import ModuleCoordinator

if __name__ == "__main__":
    config_path = Path("base-blueprint-config.json")
    cli_args = ["--module1.arg1=5"]
    parsed = BlueprintConfigParser(concrete_blueprint).parse(
        cli_args,
        config_path=config_path,
    )
    ModuleCoordinator.build(concrete_blueprint, parsed).loop()
```

Note that `build()` resets the process-global `global_config` to the full parsed
resolution (schema defaults plus all sources). Loading into an already-running
coordinator with `load_blueprint(blueprint, parsed)` applies only the fields a
configuration source explicitly set.

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

And you want to call `Drone.get_time` in `HelperModule.set_alarm_clock`.

To do this, you can request a module reference. Annotate an attribute with the module class and dimOS will inject a proxy for the running module at build time. Calling `get_time()` on it performs the RPC call.

```python session=blueprint-ex3
from dimos.core.module import Module

class HelperModule(Module):
    drone_module: Drone

    def set_alarm_clock(self) -> None:
        print(self.drone_module.get_time())
```

But what if we want `HelperModule` to work for more than just `Drone`? For that we can use a spec.

```python session=blueprint-ex3
from dimos.spec.utils import Spec
from typing import Protocol

class Drone(Module):
    @rpc
    def get_time(self) -> str:
        return "1:00 PM"

class Car(Module):
    @rpc
    def get_time(self) -> str:
        return "2:00 PM"

# Your Spec
class AnyModuleWithGetTime(Spec, Protocol):
    def get_time(self) -> str: ...

class HelperModule(Module):
    device: AnyModuleWithGetTime

    def set_alarm_clock(self) -> None:
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

`ModuleCoordinator.build()` deploys the modules, connects their streams and starts them.
It returns the coordinator that owns the running modules.

### Running and shutting down

Save the `SomeSkill` definition above in `my_skill.py` and append the following code.
Workers import the script during startup, so keep the module class at the top level and
launch the coordinator inside the main guard.

```python skip session=blueprint-ex4
from dimos.core.coordination.module_coordinator import ModuleCoordinator

if __name__ == "__main__":
    ModuleCoordinator.build(SomeSkill.blueprint()).loop()
```

Run `python my_skill.py`. The coordinator serves the running modules until you press
Ctrl+C, then `loop()` stops them and releases the workers.
