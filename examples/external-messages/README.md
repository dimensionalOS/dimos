# External message package example

`demo_msgs/msg/Reading` reuses the built-in `std_msgs/Header` from an installed
dimOS instead of redeclaring it, so one `Header` identity exists at runtime.
`pyproject.toml` and `setup.py` are the ordinary setuptools files for a package
that defines messages.

```sh
pip install .
python demo_external.py
```

`pip install .` builds the package and its generated message artifacts from the
committed `interfaces/` definitions.

Against a DimOS checkout that is not published yet, use the local install so the
build cannot download a different PyPI release:

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

To build your own package with these files, follow
[Add a message in your own project](/docs/use%20dimos%20as%20a%20library/external-message.md).
It also explains the exact message-version entry that `dimos-cdrgen` checks
against the installed manifest.
