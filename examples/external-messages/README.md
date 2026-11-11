# External message package example

This package defines one message (`demo_msgs/msg/Reading`) that reuses the
built-in `std_msgs/Header` by version dependency instead of redeclaring it.
That keeps a single `Header` identity at runtime.

`pyproject.toml` is an ordinary setuptools project file. It depends on
`dimos>=0.0.15`, so `pip install .` resolves any compatible DimOS release and
the built wheel requires the same minimum (`Requires-Dist: dimos>=0.0.15`).

The `dimos_messages` entry under `[tool.dimos.messages.dependencies]` is not a
pip requirement: `dimos-cdrgen` matches it against the installed message
manifest and requires the manifest's exact version string, so it names the
built-in message package this example is generated against.

```sh
pip install .
python demo_external.py
```

`pip install .` builds the package and its generated message artifacts from
the committed `interfaces/` definitions.

Against a DimOS checkout that is not published yet, use the local install so
the build cannot download a different PyPI release:

```sh
pip install --no-build-isolation --no-deps .
python demo_external.py
```

`demo_external.py` round-trips a `Reading` message and checks that its nested
`Header` is the same class as the installed `dimos_messages` type.

Expected final output:

```text
Reused built-in schemas from: <installed dimos_messages_schemas package>
PASS: external Reading message shares the installed dimos_messages Header identity
```
