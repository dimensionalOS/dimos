# External message package example

`demo_msgs/msg/Reading` reuses the built-in `std_msgs/Header` from an installed
dimOS instead of redeclaring it, so one `Header` identity exists at runtime.
`pyproject.toml` is the configuration `dimos package add messages` writes for an
existing project: the `.msg` definitions live in `interfaces/`, and every build
generates, packages and registers the Python message code.

```sh
pip install .
python demo_external.py
```

`pip install .` runs the generation as part of the normal pip build. There is no
`setup.py`, no separate `dimos-cdrgen` step and no message-version pin to
maintain: the build reads the message providers installed in the environment.

The message build backend is not in the released `dimos` on PyPI yet, and pip
builds in an isolated environment that does not see your active environment.
Until it is released, build one wheel from the checkout and pass it as a build
constraint (pip 25.3 or newer):

```sh
python -m pip wheel --no-deps --wheel-dir /tmp/dimos-wheel /path/to/dimos
wheel=$(ls /tmp/dimos-wheel/dimos-*.whl | head -n 1)
echo "dimos @ file://$wheel" > /tmp/dimos-build-constraint.txt
python -m pip install --build-constraint /tmp/dimos-build-constraint.txt .
python demo_external.py
```

The same constraint applies to `pip install -e .` for an editable install. A
build constraint only affects the isolated build environment; it does not
change the packages already installed in your environment.

`demo_external.py` round-trips a `Reading` message and checks that its nested
`Header` is the same class as the installed `dimos_messages` type.

Expected final output:

```text
Reused built-in schemas from: <installed dimos_messages_schemas package>
PASS: external Reading message shares the installed dimos_messages Header identity
```

To build your own package with these files, copy this project's layout: put the
`.msg` definitions under `interfaces/`, point `[tool.dimos.messages].source` at
that directory, and use `dimos.message_codegen.backend` as the build backend.
[`docs/development/message-source-packages.md`](/docs/development/message-source-packages.md) describes the
generator integration behind it.
