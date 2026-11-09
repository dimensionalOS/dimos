# External message projects

An external project owns its `.msg` definitions and registers its generated
message provider through the `dimos.messages` entry-point group.

The [standalone example](/examples/external-messages/README.md) declares
dimOS as both a build and runtime dependency and `dimos.messages` as its message
dependency. Its small `setup.py` passes cdrgen's `setup_arguments` to setuptools.
Configuration, generation, provider registration and packaging belong to cdrgen;
dimOS does not implement another backend or generator.

## Install and run

Run from the repository root. First complete the
[built-in Python installation](/docs/development/message-source-packages.md#install-and-run-python)
to create `.venv-msg-review` with this checkout's dimOS package. No CMake or Cargo
is needed for this external registration example.

```sh skip
uv pip install --python .venv-msg-review/bin/python 'setuptools>=70' wheel
uv pip install --python .venv-msg-review/bin/python --no-build-isolation ./examples/external-messages
.venv-msg-review/bin/python -I examples/external-messages/demo_reading.py
```

`--no-build-isolation` uses the matching dimOS already installed in this environment.
Ordinary isolated source, wheel, sdist and editable installs are also supported
when pip has access to that matching review wheel. An older release with the same
version cannot supply these built-ins.

Expected output:

```text
Installed external message sources: <venv>/lib/python3.12/site-packages/acme_telemetry_schemas/package
PASS: external message registered; CDR roundtrip reuses built-in Header
```

The example verifies a single registered provider, a custom-message CDR roundtrip,
and exact identity of the Header class supplied by dimOS.

## Edit a definition

Edit `examples/external-messages/interfaces/telemetry_msgs/msg/DeviceReading.msg`,
then rerun the installation command and restart Python processes. Generated
artifacts are outputs; edit the `.msg` source instead. Editable installations
also need reinstalling after definitions change. Tests cover field changes,
renames and deletion of obsolete generated types.

The native built-in example is documented in
[Run C++ and Rust](/docs/development/message-source-packages.md#run-c-and-rust).
Transport and SDK integration are separate features.
