# Run your modules with a blueprint

A blueprint is a set of modules plus the connections between them. This page
adds a blueprint file to the `garden-monitor` package from [Add a module in your
own project](/docs/use-dimos-as-a-library/external-module.md),
registers it under a name that `dimos run` can resolve, and starts it.

You still work only in your own project. The blueprint is a value in your
package, and dimOS discovers it from installed package metadata.

## 1. Add the blueprint

Create `garden_monitor/blueprints.py`:

```python
from dimos.core.coordination.blueprints import autoconnect

from garden_monitor.modules import GardenLogger, GardenSensor

garden = autoconnect(
    GardenSensor.blueprint(interval=1.0, beds=2),
    GardenLogger.blueprint(),
)
```

`autoconnect` links the fragments by stream name and type. `GardenSensor` has
`readings: Out[String]`, `GardenLogger` has `readings: In[String]`, so the two
`readings` streams are connected. If the names differed you would rename one
side with `.remappings(...)`; see [Blueprints](/docs/usage/blueprints.md) for
the remaining composition rules and transport options.

`GardenSensor.blueprint(interval=1.0, beds=2)` is a blueprint fragment for one
configured module instance; the long form is
`Blueprint.create(GardenSensor, interval=1.0, beds=2)`. `autoconnect(...)`
returns a new expanded `Blueprint`, so expanding a stack never changes the
fragments it was built from and the same fragment can be reused elsewhere. A
blueprint can contain other blueprints as well as modules, which is how the
stack below adds a built-in dimOS module.

The package tree now looks like this:

```text
garden-monitor/
  pyproject.toml
  garden_monitor/
    __init__.py
    blueprints.py
    modules.py
```

## 2. Register the blueprint

`dimos run` resolves a blueprint by name, and a name reaches your package
through a Python entry point. Add this table to `pyproject.toml`:

```toml
[project.entry-points."dimos.blueprints"]
garden = "garden_monitor.blueprints:garden"
```

The name has two parts:

- `garden-monitor` is the namespace, taken from the distribution name in
  `[project] name`. dimOS lowercases it and joins runs of `-`, `_` and `.` into
  `-`.
- `garden` is the local blueprint name, here the entry point name. It must be
  lowercase kebab-case and match `^[a-z0-9]+(-[a-z0-9]+)*$`.

The installed blueprint is addressed as `garden-monitor.garden`. The entry point
target is the module-level `garden` blueprint. A dimOS `Module` class is also
accepted as a target, and dimOS converts it with `.blueprint()`.

Reinstall so the installed metadata includes the new entry point:

```sh
python -m pip install -e .
```

Editing a source file is not enough on its own. Entry points are read from
distribution metadata, so the install step is what makes the name resolvable.

## 3. Find it

```sh
dimos list
```

External blueprints are listed after the built-in ones:

```text
External blueprints:
  garden-monitor.garden
```

`dimos list` reads package metadata and does not import your modules. A name
appears here even before `dimos run` imports the blueprint.

## 4. Run it

```sh
dimos run garden-monitor.garden
```

dimOS prints its startup logs, then the lines from `GardenLogger`:

```text
garden: bed 1: moisture 40%
garden: bed 2: moisture 43%
garden: bed 1: moisture 40%
garden: bed 2: moisture 43%
```

`GardenSensor` publishes to its `readings` output every second, so one line per
bed appears per second. Ctrl-C stops the run: dimOS stops both modules, shuts
down their worker processes and removes the run entry.

## 5. Change the configuration

The defaults live in `GardenSensorConfig`. Override them for one run:

```sh
dimos run garden-monitor.garden --interval 0.2 --beds 3
```

Then the output cycles through three beds five times per second:

```text
garden: bed 1: moisture 40%
garden: bed 2: moisture 43%
garden: bed 3: moisture 46%
garden: bed 1: moisture 40%
```

`--interval` and `--beds` are unambiguous here. Use the qualified form
`--gardensensor.interval 0.2` when another module in the blueprint has a field
with the same name. `dimos run garden-monitor.garden --help` lists every
available option.

The same settings can be fixed in the blueprint where the module is added:

```python
garden = autoconnect(
    GardenSensor.blueprint(interval=0.2, beds=3),
    GardenLogger.blueprint(),
)
```

Blueprint values apply when the flag is absent, so a run-time flag still wins.

## 6. Mix in built-in modules

Modules that ship with dimOS compose the same way. This stack adds the built-in
webcam module to the garden modules:

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.module import WebcamModule

from garden_monitor.modules import GardenLogger, GardenSensor

garden = autoconnect(
    GardenSensor.blueprint(),
    GardenLogger.blueprint(),
    WebcamModule.blueprint(),
)
```

`WebcamModule` declares its own streams and configuration, and its settings
show up in `dimos run garden-monitor.garden --help` as `--webcammodule.*`.
Starting this stack needs a camera; composing it and listing its options does
not.

## Notes

- The package must be installed in the environment where the coordinator
  resolves the name. If a client asks a remote coordinator for
  `garden-monitor.garden`, install `garden-monitor` where that coordinator
  runs, not only on the client.
- Do not edit `dimos/robot/all_blueprints.py`. That generated registry is for
  in-repository blueprints; installed entry points are discovered separately.
- The `dimos.blueprints` entry point group is the supported registration
  mechanism at this revision.
