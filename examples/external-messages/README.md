# External message package example

This package defines one message (`demo_msgs/msg/Reading`) that reuses the
built-in `std_msgs/Header` by version dependency instead of redeclaring it.
That keeps a single `Header` identity at runtime.

`pyproject.toml.in` contains no version literal: `build_example.py` reads the
installed DimOS version from the distribution metadata (or the generated
`message-package.json`), renders a temporary `pyproject.toml`, and builds with
`--no-build-isolation --no-deps` so the build cannot download a different PyPI
DimOS release.

```sh
pip install .
python examples/external-messages/build_example.py
```

The script builds a wheel, installs it into a temporary target directory and
runs `demo_external.py`, which round-trips a `Reading` message and checks that
its nested `Header` is the same class as the installed `dimos_messages` type.

Expected final output:

```text
PASS: external Reading message shares the installed dimos_messages Header identity
```
