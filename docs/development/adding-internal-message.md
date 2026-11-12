# Add a message inside the DimOS repository

A new battery sensor has to report voltage, current and temperature to other
modules. This tutorial adds a `dimos_msgs/msg/BatteryReading` definition to the
built-in messages, rebuilds dimOS, and reads one value back from its CDR bytes.

## Before you start

- Python 3.12 and [uv](https://docs.astral.sh/uv/).
- A checkout of this repository on a working branch.
- A C++ compiler (`gcc` or `clang`).

The runnable examples for the built-in message package, including the Python,
C++ and Rust roundtrip, are in
[examples/message-package](/examples/message-package/README.md).

## 1. Create the definition

Message inputs live under `packages/dimos-messages/interfaces`. `dimos_msgs` is
the package DimOS authors in this repository. Every other package directory
under `interfaces` is vendored from an upstream repository: builds read those
files, but you do not edit them or add your own files there.
`packages/dimos-messages/schema-sources.json` records the upstream repository,
revision, archive hash and per-file hashes for the vendored directories. It has
no entry for `dimos_msgs`.

Create `packages/dimos-messages/interfaces/dimos_msgs/msg/BatteryReading.msg`:

```text
# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

# One battery sample for a named robot component.
std_msgs/Header header
float64 voltage
float64 current
float32 temperature_c
uint8 STATE_UNKNOWN=0
uint8 STATE_OK=1
uint8 STATE_LOW=2
uint8 state
```

The first field is a nested type: package and message name, with no `import`.
`Header` comes from the vendored `std_msgs` package and already contains
`builtin_interfaces/Time stamp` and `string frame_id`. Constants are declared
before the field that uses them.

## 2. Build and install

From the repository root:

```sh
uv venv .venv-msg --python 3.12
DIMOS_ALLOW_MISSING_COCKPIT=1 uv pip install --python .venv-msg/bin/python .
```

`DIMOS_ALLOW_MISSING_COCKPIT=1` is only for a checkout that has no built web
bundles. A complete distribution ships them.

No message configuration changes are needed. The root `pyproject.toml` already
declares the input directory and the generated package:

```toml
[tool.dimos.messages]
source = "packages/dimos-messages/interfaces"
python-module = "dimos_messages"
```

`setup.py` calls `dimos_cdrgen.source_package.prepare(...)` before packaging.
That call collects every `.msg` file below `source` and writes Python classes to
`build/message-source/python`. The install then puts the result into the
environment as `dimos_messages`. `build/message-source` is generated output:
change the `.msg` file and reinstall instead of editing it.

## 3. Read a value back

Create `battery_demo.py` at the repository root:

```python
from dimos_cdrgen.registry import decode, encode
from dimos_messages.builtin_interfaces.msg import Time
from dimos_messages.dimos_msgs.msg import BatteryReading
from dimos_messages.std_msgs.msg import Header

reading = BatteryReading(
    header=Header(stamp=Time(sec=17, nanosec=250_000_000), frame_id="battery"),
    voltage=24.2,
    current=1.5,
    temperature_c=21.5,
    state=BatteryReading.STATE_OK,
)

payload = encode(reading)
restored = decode(payload, BatteryReading)

print(f"encoded bytes: {len(payload)}")
print(f"frame: {restored.header.frame_id}")
print(f"stamp: {restored.header.stamp.sec}.{restored.header.stamp.nanosec:09d}")
print(f"voltage: {restored.voltage}")
print(f"state: {restored.state} ({BatteryReading.STATE_OK} is ok)")
print(f"same value: {restored == reading}")
print(f"same Header class: {type(restored.header) is Header}")
```

Run it with the environment that has the package installed:

```sh
.venv-msg/bin/python battery_demo.py
```

Output:

```text
encoded bytes: 49
frame: battery
stamp: 17.250000000
voltage: 24.2
state: 1 (1 is ok)
same value: True
same Header class: True
```

`encode` returns CDR bytes and `decode` rebuilds the message from them. Both
take generated classes from the installed package. The import path mirrors the
schema directory: `dimos_msgs/msg/BatteryReading.msg` becomes
`dimos_messages.dimos_msgs.msg.BatteryReading`. The nested `Header` is the same
class as `dimos_messages.std_msgs.msg.Header`, so a message never carries a
second copy of a built-in type.

`float32` values are rounded to 32 bits on decode. This example uses values that
fit exactly, so `restored == reading` is `True`. When a value does not fit,
compare with a tolerance instead of equality.

## 4. Change the definition

Edit the `.msg` file, then rerun the install command:

```sh
DIMOS_ALLOW_MISSING_COCKPIT=1 uv pip install --python .venv-msg/bin/python .
```

Each install regenerates the classes before it installs them. To regenerate
without installing, run `.venv-msg/bin/python -m dimos_cdrgen.source_package .`
from the repository root; the classes land in
`build/message-source/python/dimos_messages` for inspection.

A Python process that already imported `dimos_messages` keeps the registry it
initialized. Restart the process after reinstalling so it loads the changed
classes.
