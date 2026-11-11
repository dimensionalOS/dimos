# Add a message in your own project

A garden sensor needs to publish soil readings from a package that is not part
of this repository. This tutorial creates the `plant-telemetry` package with a
`plant_msgs/msg/SoilReading` message that reuses the built-in `Header` from an
installed dimOS.

## Before you start

- Python 3.12.
- An environment with `dimos` and `dimos-cdrgen==0.1.2` installed. Install DimOS
  with its release instructions; this tutorial assumes the installed
  `dimos_messages` manifest reports version `0.0.15`.
- For a DimOS checkout that is not published yet, install that checkout into the
  environment first, so the external build uses it instead of a PyPI release:

```sh
python -m venv .venv
. .venv/bin/activate
pip install /path/to/dimos
```

The external package generates Python only. It does not need a C++ compiler,
ROS, a robot or a transport.

## 1. Create the project

```text
plant-telemetry/
  pyproject.toml
  setup.py
  interfaces/
    plant_msgs/
      msg/
        SoilReading.msg
  demo.py
```

Three names describe the package and have to line up:

- `plant-telemetry` is the pip distribution name, set in `[project] name`.
- `plant_telemetry` is the Python package that receives the generated code, set
  by `[tool.dimos.messages] python-module`.
- `plant_msgs` is the schema package, taken from the directory below
  `interfaces`.

The generated import follows those names:
`from plant_telemetry.plant_msgs.msg import SoilReading`. The generated schemas
install as `plant_telemetry_schemas`, and the package registers itself in the
`dimos.messages` entry point group so `dimos-cdrgen` can find it.

## 2. Add the message

Create `interfaces/plant_msgs/msg/SoilReading.msg`:

```text
# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

# One soil sample from a garden sensor.
std_msgs/Header header
float32 moisture_percent
float32 temperature_c
bool pump_on
```

`std_msgs/Header` is not defined in this project. It resolves to the class in the
installed `dimos_messages` package, whose exact message version is declared in
the next step.

## 3. Describe the package

Create `pyproject.toml`. It is an ordinary setuptools project file:

```toml
[build-system]
requires = ["setuptools>=70", "wheel", "dimos-cdrgen==0.1.2", "dimos>=0.0.15"]
build-backend = "setuptools.build_meta"

[project]
name = "plant-telemetry"
version = "0.1.0"
description = "Garden soil readings that reuse the installed DimOS Header"
requires-python = ">=3.12"
dependencies = ["dimos>=0.0.15", "dimos-cdrgen==0.1.2"]

[tool.dimos.messages]
python-module = "plant_telemetry"

[tool.dimos.messages.dependencies]
"dimos_messages" = "0.0.15"
```

Create `setup.py`. It hands the project directory to the generator:

```python
from pathlib import Path

from dimos_cdrgen.source_package import setup_arguments
from setuptools import setup

setup(**setup_arguments(Path(__file__).resolve().parent))
```

The two version declarations are different contracts:

- `dimos>=0.0.15` in `[build-system]` and `[project] dependencies` is a pip
  requirement. It sets the oldest DimOS release this package installs with.
- `[tool.dimos.messages.dependencies]` is a message identity, not a pip
  requirement. `dimos-cdrgen` matches each entry against the installed message
  manifest and requires the exact version string. `"dimos_messages" = "0.0.15"`
  matches the manifest that dimOS 0.0.15 ships. `dimos-cdrgen==0.1.2` currently
  requires that exact value: a specifier such as `">=0.0.15"` is rejected, and a
  newer installed manifest is not accepted automatically. Change the literal
  when you deliberately build against a newer message package and a generator
  release that supports it.

## 4. Build and install

With the DimOS environment active, from `plant-telemetry/`:

```sh
pip install --no-build-isolation --no-deps .
```

`--no-build-isolation` builds with the `dimos-cdrgen` and DimOS already in the
environment. An isolated build resolves its own dependencies, so it can fetch a
DimOS release that does not contain these message packages. `--no-deps` leaves
the installed DimOS alone; the generated schemas were already built against it.

To build a wheel without installing it:

```sh
pip wheel --no-build-isolation --no-deps --wheel-dir dist .
```

The result is `dist/plant_telemetry-0.1.0-py3-none-any.whl`. Install it with
`pip install --no-deps dist/plant_telemetry-0.1.0-py3-none-any.whl`.

## 5. Read a value back

Create `demo.py`:

```python
from dimos_cdrgen.registry import decode, encode
from dimos_messages.builtin_interfaces.msg import Time
from dimos_messages.std_msgs.msg import Header
from plant_telemetry.plant_msgs.msg import SoilReading

reading = SoilReading(
    header=Header(stamp=Time(sec=1, nanosec=0), frame_id="garden"),
    moisture_percent=41.5,
    temperature_c=21.5,
    pump_on=False,
)

payload = encode(reading)
restored = decode(payload, SoilReading)

print(f"encoded bytes: {len(payload)}")
print(f"frame: {restored.header.frame_id}")
print(f"moisture: {restored.moisture_percent}%")
print(f"pump on: {restored.pump_on}")
print(f"same value: {restored == reading}")
print(f"same Header class: {type(restored.header) is Header}")
```

Run it from `plant-telemetry/` with the same environment:

```sh
python demo.py
```

Output:

```text
encoded bytes: 33
frame: garden
moisture: 41.5%
pump on: False
same value: True
same Header class: True
```

`type(restored.header) is Header` is `True` because both names refer to the
class installed by `dimos_messages`. The external package does not generate a
second `Header`.

`float32` values are rounded to 32 bits on decode. This example uses values that
fit exactly, so `restored == reading` is `True`. When a value does not fit,
compare with a tolerance instead of equality.

## 6. Change the definition

Edit `interfaces/plant_msgs/msg/SoilReading.msg`, then rerun the install command
from step 4. The build regenerates the classes and installs the package again.
Restart any Python process that already imported `plant_telemetry`; the message
registry is initialized once per process.
