# Add a module in your own project

A garden monitor needs a module that publishes soil readings. The module lives
in your own Python package, imports dimOS and is installed next to it. dimOS
does not learn about the module until a blueprint asks for it, so nothing in
this tutorial edits the dimOS repository or its built-in registries.

The example package is called `garden-monitor`. This page creates the package
and its two modules. [Run your modules with a
blueprint](/docs/use-dimos-as-a-library/external-blueprint.md) adds the
blueprint and the entry point that make them runnable.

## Before you start

- Python 3.12 and `pip`.
- An environment with dimOS installed, as described in [Use dimOS as a
  library](/docs/use-dimos-as-a-library/README.md).

This package contains Python only. Unlike installing a dimOS checkout, it needs
no C++ compiler.

## 1. Create the project

```text
garden-monitor/
  pyproject.toml
  garden_monitor/
    __init__.py
    modules.py
```

`garden-monitor/` is a directory outside the dimOS checkout. `garden_monitor`
is the Python package inside it, and `modules.py` holds the module classes.

Create `pyproject.toml`:

```toml
[build-system]
requires = ["setuptools>=70"]
build-backend = "setuptools.build_meta"

[project]
name = "garden-monitor"
version = "0.1.0"
description = "A garden sensor module and blueprint for dimOS"
requires-python = ">=3.12"
dependencies = ["dimos>=0.0.15", "reactivex"]

[tool.setuptools]
packages = ["garden_monitor"]
```

`dimos>=0.0.15` is the oldest dimOS release this package is written against.
`reactivex` is the observable library the sensor uses to schedule its readings;
dimOS installs it too, but your package imports it directly, so it is listed.

## 2. Write the modules

Create `garden_monitor/__init__.py` as an empty file, then
`garden_monitor/modules.py`:

```python
from __future__ import annotations

import reactivex as rx

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.std_msgs.String import String


class GardenSensorConfig(ModuleConfig):
    interval: float = 1.0
    beds: int = 2


class GardenSensor(Module):
    """Publish one soil reading per bed on a fixed interval."""

    config: GardenSensorConfig
    readings: Out[String]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._bed = 0

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(rx.interval(self.config.interval).subscribe(self._emit))

    def _emit(self, _tick: int) -> None:
        bed = self._bed
        self._bed = (bed + 1) % self.config.beds
        self.readings.publish(String(data=f"bed {bed + 1}: moisture {40 + bed * 3}%"))


class GardenLogger(Module):
    """Print every soil reading that reaches its input."""

    readings: In[String]

    async def handle_readings(self, msg: String) -> None:
        print(f"garden: {msg.data}")
```

The parts worth knowing:

- `Module` is the dimOS base class. A subclass never calls another module
  directly; it declares typed streams instead.
- `readings: Out[String]` and `readings: In[String]` are streams. `String`
  comes from `dimos.msgs.std_msgs` in the installed dimOS, not from this
  package.
- `config: GardenSensorConfig` turns the class attributes of a `ModuleConfig`
  into settings. `interval` and `beds` get their defaults here and can be
  overridden per blueprint or per run.
- `start()` is an `@rpc` lifecycle method. dimOS calls it after the module is
  deployed and deployed streams are wired. `register_disposable` hands the
  subscription back to the module so `stop()` disposes it. Without that, the
  timer keeps running after shutdown.
- `GardenLogger` has no custom `start()`. For every `x: In[T]` stream, a method
  named `handle_x` is subscribed automatically at `start()` and runs on the
  module's asyncio loop, so the handler needs no thread-safe bookkeeping.

Only `GardenSensor` publishes and only `GardenLogger` subscribes. Neither
knows about the other yet; names and types are enough to wire them later.

## 3. Install and inspect it

From `garden-monitor/`, with the environment that has dimOS installed:

```sh
python -m pip install -e .
```

Then ask the module what it exposes:

```sh
python -c "from garden_monitor.modules import GardenSensor; print(GardenSensor.io())"
```

Output:

```text
┌┴─────────────┐
│ GardenSensor │
└┬─────────────┘
 ├─ readings: String
```

`io()` is built from the class annotations and needs no running coordinator, no
transport and no worker process. `GardenLogger.io()` prints the same diagram
with its input stream.

## 4. Configure it

Configuration has defaults on the class and can be changed where the module is
added to a blueprint:

```python
GardenSensor.blueprint(interval=0.5, beds=3)
```

`GardenSensor.blueprint(...)` builds the blueprint fragment for one module
instance with those values. After a blueprint is registered, the same fields
appear as `dimos run` flags, for example `--interval 0.5` or
`--gardensensor.interval 0.5`; `dimos run <blueprint> --help` lists them. See
[Configuration](/docs/usage/configuration.md) for the full resolution order.

The lifecycle and stream rules for modules in a package are the same as for
modules inside dimOS. [Modules](/docs/usage/modules.md) covers the rest in
depth, including async handlers and RPC.

## More module patterns

These two patterns apply to any module, including the modules in this package.

### Sync input handlers

If you don't need an asyncio loop, subscribe to your `In[T]` streams from `start()` and register the unsubscribe with `register_disposable` so cleanup happens automatically at `stop()`.

```python
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.std_msgs.Int32 import Int32


class Counter(Module):
    value: In[Int32]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._total = 0

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.value.subscribe(self._on_value)))

    def _on_value(self, msg: Int32) -> None:
        self._total += msg.data
```

`In.subscribe(cb)` returns an *unsubscribe function*, not a `DisposableBase`. Wrap it in `Disposable(...)` so `register_disposable` can dispose it on `stop()`. Without this, your handler keeps running after `stop()` and tests will fail thread-leak checks.

The callback runs on whatever thread emits the message, so guard mutable state with a lock if multiple inputs share it.

### Triggering side effects via Specs

A common pattern is "subscribe to a stream, react by calling another module". Declare the other module's protocol as a `Spec` field (single-underscore, private). The coordinator binds the proxy at deploy time, so handlers can call it directly with no extra wiring:

```python
from typing import Protocol

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.std_msgs.Int32 import Int32
from dimos.spec.utils import Spec


class NotifierSpec(Spec, Protocol):
    def notify(self, text: str) -> None: ...


class Watchdog(Module):
    value: In[Int32]

    _notifier: NotifierSpec

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.value.subscribe(self._on_value)))

    def _on_value(self, msg: Int32) -> None:
        if msg.data > 100:
            self._notifier.notify(f"value={msg.data}")
```

The Spec must match the target module's `@rpc` signatures. Sync and async are interchangeable (see [Async modules](/docs/usage/modules.md#async-modules-lock-free-state)).

To deploy `Watchdog`, add `Watchdog.blueprint()` to an existing blueprint's `autoconnect(...)` chain. The coordinator matches `Out[T]` to `In[T]` by name across the union of modules, and resolves `_notifier: NotifierSpec` to whichever module in the blueprint implements `notify`. No manual wiring required.

Next: [Run your modules with a
blueprint](/docs/use-dimos-as-a-library/external-blueprint.md).
