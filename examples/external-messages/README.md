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
Until it is released, select the checkout's wheel for the build (pip 25.3 or
newer):

```sh
python -m pip install --build-constraint /tmp/dimos-build-constraint.txt .
python demo_external.py
```

[Use the checkout in your own pip builds](/docs/installation/index.md#use-the-checkout-in-your-own-pip-builds)
builds that wheel and constraint file. The same constraint applies to
`pip install -e .` for an editable install.

`demo_external.py` round-trips a `Reading` message and checks that its nested
`Header` is the same class as the installed `dimos_messages` type.

Expected final output:

```text
Reused built-in schemas from: <installed dimos_messages_schemas package>
PASS: external Reading message shares the installed dimos_messages Header identity
```

To build your own package with these files, follow
[Add a message in your own project](/docs/use%20dimos%20as%20a%20library/external-message.md).
