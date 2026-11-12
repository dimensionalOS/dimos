# Built-in message examples

**WIP:** CDR transport and application support are incomplete.

- `demo_installed_wheel.py` checks installed Python messages and CDR roundtrips.
- `demo_native.py` sends a built-in Header through Python, C++, Rust, and back to Python.
- `native/` contains the CMake and Cargo source files for `demo_native.py`.

Install the checkout and run the Python example from the repository root:

```sh
uv venv .venv-msg --python 3.12
DIMOS_ALLOW_MISSING_COCKPIT=1 uv pip install --python .venv-msg/bin/python .
.venv-msg/bin/python -I examples/message-package/demo_installed_wheel.py
```

The native example additionally needs CMake, Rust/Cargo and
`dimos-cdrgen[native]==0.1.2` in the same environment; run
`.venv-msg/bin/python -I examples/message-package/demo_native.py`.

See [`docs/development/message-source-packages.md`](/docs/development/message-source-packages.md)
for the generator integration. To add a message to the built-in package, add a
`.msg` file under `packages/dimos-messages/interfaces/dimos_msgs/msg` and
reinstall the checkout; the next build regenerates the Python classes.
