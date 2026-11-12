# Add a message in your own project

A garden sensor needs to publish soil readings from a package that is not part
of this repository. This tutorial adds a `plant_msgs/msg/SoilReading` message to
an existing Python project and reuses the built-in `Header` from an installed
dimOS.

## Before you start

- Python 3.12 and `pip` 25.3 or newer.
- dimOS installed in the environment you build with. Follow
  [installation](/docs/installation/index.md); against a source checkout, use
  the checkout install described there.
- The message build backend is not in the released `dimos` on PyPI yet, and pip
  builds in an isolated environment. Until it is released, pass the checkout's
  wheel as a build constraint: [Use the checkout in your own pip
  builds](/docs/installation/index.md#use-the-checkout-in-your-own-pip-builds).
  [the external message example](/examples/external-messages/README.md) is the
  same recipe as a runnable project in this repository.
- Your own project already has a `pyproject.toml` with a `[project]` table, a
  release version such as `0.1.0`, and its own Python modules.

Check the installed versions:

```sh
python -c "from importlib.metadata import version; print(version('dimos'), version('dimos-cdrgen'))"
```

Building the message package needs no C++ compiler, ROS, robot or transport.

## 1. Add message packaging

From the project directory:

```sh
dimos package add messages
```

The command creates `interfaces/` and makes the smallest edit to
`pyproject.toml` that turns every `pip install` into a message build. For a
project called `plant-telemetry` the added configuration is:

```toml
[build-system]
requires = ["setuptools>=70", "wheel", "dimos>=0.0.15"]
build-backend = "dimos.message_codegen.backend"

[project]
# unchanged: name, version, description, your own dependencies ...
dependencies = [
    # your existing dependencies ...
    "dimos>=0.0.15",
]

[tool.dimos.messages]
source = "interfaces"

[tool.setuptools.package-data]
plant_telemetry = ["py.typed", "*.pyi", "**/*.pyi"]
plant_telemetry_schemas = ["schemas.json", "schemas/**/*", "package/**/*"]

[project.entry-points."dimos.messages"]
plant_telemetry = "plant_telemetry_schemas.provider"
```

You do not choose a Python module: generated messages land in the project name
with `-` replaced by `_`, so `plant-telemetry` generates `plant_telemetry`.
Existing metadata, dependencies, package data and comments are preserved.

Running the command again changes nothing. It refuses, before touching any
file, projects that have a `setup.py`/`setup.cfg`, a build backend other than
setuptools, or explicit `[tool.setuptools] packages`/`package-dir`/`py-modules`
configuration.

## 2. Write the message

```text
plant-telemetry/
  pyproject.toml
  interfaces/
    plant_msgs/
      msg/
        SoilReading.msg
  demo.py
```

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

`std_msgs/Header` is not defined in this project. It resolves to the class in
the installed `dimos_messages` package, and that exact built-in schema is
packaged alongside the new one, so importing and decoding uses one `Header`
class.

## 3. Build and install

With the dimOS environment active, from `plant-telemetry/`:

```sh
pip install .
```

pip builds in an isolated environment, so it does not see the dimOS installed in
your active environment, and the released `dimos` on PyPI predates this backend.
Until the version you are reading this for is released, select the checkout's
wheel for the build (pip 25.3 or newer):

```sh
python -m pip install --build-constraint /tmp/dimos-build-constraint.txt .
```

[Use the checkout in your own pip
builds](/docs/installation/index.md#use-the-checkout-in-your-own-pip-builds)
builds that wheel, installs it in your environment and writes the constraint
file, and shows the editable variant. The `DIMOS_ALLOW_MISSING_COCKPIT=1` flag on
that wheel build is required for a checkout whose web bundles are not built;
[the external message example](/examples/external-messages/README.md) runs the
same recipe end to end with the expected output.

Every build regenerates the Python message packages under
`build/message-source` from `interfaces/`, packages them with the project's own
modules, and registers the schemas. There is no `setup.py`, no manual
`dimos-cdrgen` step and no version pin to maintain: the build reads the message
providers installed in its environment.

During development use an editable install instead, with the same constraint:

```sh
python -m pip install --build-constraint /tmp/dimos-build-constraint.txt -e .
```

`build/message-source` is build output. Do not commit it; `interfaces/` is the
source of truth.

To build a wheel without installing it:

```sh
python -m pip wheel --build-constraint /tmp/dimos-build-constraint.txt --no-deps --wheel-dir dist .
python -m pip install --no-deps dist/plant_telemetry-0.1.0-py3-none-any.whl
```

## 4. Read a value back

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

Run it with the environment that installed the package:

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
class installed by `dimos_messages`; the project does not generate a second
`Header`.

`float32` values are rounded to 32 bits on decode. This example uses values that
fit exactly, so `restored == reading` is `True`. When a value does not fit,
compare with a tolerance instead of equality.

## 5. Change or remove a definition

Edit `interfaces/plant_msgs/msg/SoilReading.msg`, then rerun `pip install .` (or
`pip install -e .`). The build regenerates the classes, so new or changed fields
appear and a type whose `.msg` file was deleted or renamed disappears on the
next install. Restart any Python process that already imported
`plant_telemetry`; the message registry is initialized once per process.

If one of your project files has the same path as a generated file, for example
`plant_telemetry/_types.py`, the build stops with an error naming both paths
instead of dropping your file. Rename the project file, or move the schemas to
another module with `[tool.dimos.messages] module = ...`.
