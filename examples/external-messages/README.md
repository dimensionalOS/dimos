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
Until it is released, build one wheel from the checkout, install that wheel in
the environment that runs the example, and point a build constraint at it
(Python 3.10 or newer, `gcc` or `clang`, pip 25.3 or newer):

```sh
# Only for a checkout without the built web bundles; see below.
export DIMOS_ALLOW_MISSING_COCKPIT=1

python -m pip wheel --no-deps --wheel-dir /tmp/dimos-wheel /path/to/dimos

wheel=$(ls /tmp/dimos-wheel/dimos-*.whl | head -n 1)
python -m pip install "$wheel"
echo "dimos @ file://$wheel" > /tmp/dimos-build-constraint.txt

python -m pip install --build-constraint /tmp/dimos-build-constraint.txt .
python demo_external.py
```

`DIMOS_ALLOW_MISSING_COCKPIT=1` is needed for a plain git checkout: the web
bundles are binary assets that are not committed, and without them the wheel
build refuses to produce a UI-less wheel. The flag covers both bundles and skips
only the local relay UI; messages, native code and the Python API install as
usual. Build the bundles with `deno task build` in `web/sdk` and `web/cockpit`
when you need the relay UI, and leave the flag unset.

A build constraint only selects that exact wheel for the isolated build
environment; it installs nothing. That is why the checkout wheel is installed
into the environment that runs the example first. The same constraint applies to
`pip install -e .` for an editable install, with `python -m pip install -e
/path/to/dimos` in place of the wheel install if you prefer to work from the
checkout.

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
