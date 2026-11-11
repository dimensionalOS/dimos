# CDR generation in dimOS

Built-in message definitions are committed under
`packages/dimos-messages/interfaces` together with their upstream licenses and
`schema-sources.json` provenance (repository, revision, archive hash and
per-file hashes). `setup.py` generates Python messages from those files through
`dimos-cdrgen==0.1.2`; builds never download definitions or prepare schema
archives.

The wheel contains the generated `dimos_messages` package, the
`dimos_messages_schemas` resources (schemas, C++/Rust build inputs and
metadata), and the upstream licenses. The main `dimos` distribution owns the
built-in messages; there is no separate `dimos-generated` distribution.
The [cdrgen guides](https://tomcc7.github.io/dimos-cdrgen/) describe its APIs
and supported schemas. `dimos.message_codegen.native_build.write_cmake_toolchain`
adds paths for the dimOS SDK.

## Run the built-in examples

Run all commands below from the repository root. Install `uv` and a C++
compiler first; the compiler is needed by dimOS's existing A* extension even for
a source installation used only by Python. The examples use Python 3.12.

### Install and run Python

```sh skip
uv venv .venv-msg-review --python 3.12
DIMOS_ALLOW_MISSING_COCKPIT=1 uv pip install --python .venv-msg-review/bin/python .
.venv-msg-review/bin/python -I examples/message-package/demo_installed_wheel.py
```

`DIMOS_ALLOW_MISSING_COCKPIT=1` is the existing option to install without prepared
web UI bundles. Omit it when building a complete distribution with those assets.
The example imports the installed package, prints the generated Python and
schema/native-source directories, and checks image and custom-message CDR
roundtrips. Expected output includes:

```text
Installed package: <venv>/lib/python3.12/site-packages/dimos_messages
Generated Python messages: <venv>/lib/python3.12/site-packages
Packaged schemas and native sources: <venv>/lib/python3.12/site-packages/dimos_messages_schemas/package
PASS: sensor_msgs/msg/Image pixels, encoding and source stamp
PASS: dimos_msgs/msg/LineSegments3D geometry, weight and nested type identity
PASS: packaged definitions and C++/Rust build inputs; no compiled message libraries
```

The generated directories are installation outputs, not files to edit. Message
inputs live under `packages/dimos-messages/interfaces` in the checkout.

### Run C++ and Rust

Install CMake and Rust/Cargo, then reuse the same environment:

```sh skip
uv pip install --python .venv-msg-review/bin/python 'dimos-cdrgen[native]==0.1.2'
.venv-msg-review/bin/python -I examples/message-package/demo_native.py
```

This one command compiles the ordinary CMake and Cargo consumers in
`examples/message-package/native`. The example reads the generated Rust crate
version from the packaged manifest and pins the temporary Cargo dependency to
it, so the example does not hardcode a release number. Python writes an
installed built-in Header, C++ appends `/cpp` to its frame ID, Rust appends
`/rust`, and Python verifies `map/cpp/rust` with the timestamp unchanged.
Expected final output:

```text
PASS: built-in Header Python -> C++ -> Rust -> Python
```

The script also prints the installed native-source directory. Temporary example
files are cleaned up. Native support builds use `~/.cache/dimos/messages`;
`DIMOS_MESSAGE_NATIVE_CACHE` overrides that location. Set `CARGO_TARGET_DIR` to
retain Rust build artifacts. Cold builds download pinned dependencies;
`DIMOS_OFFLINE=1` requires populated native and Cargo caches.

### External message package

`examples/external-messages` shows a package that defines one message and reuses
the installed built-in `Header` by version dependency instead of redeclaring it:

```sh skip
cd examples/external-messages
pip install .
python demo_external.py
```

`pyproject.toml` is an ordinary setuptools project file: it declares
`dimos>=0.0.15`, so the build resolves any compatible DimOS release and the
built wheel carries the same minimum. The `dimos_messages` entry under
`[tool.dimos.messages.dependencies]` is a message identity, not a pip
requirement. `dimos-cdrgen` matches it against the installed message manifest
and requires that manifest's exact version string. Against an unpublished
checkout, build with `pip install --no-build-isolation --no-deps .` so the
local DimOS is used instead of a PyPI release.

### What installation does

Definitions → setuptools calls cdrgen → Python/schema/native-source
artifacts in the dimOS wheel → installed consumers. Source installation
generates artifacts from the committed definitions. Wheel installation only
installs those artifacts. The sdist includes definitions needed to rebuild the
wheel.

Installation does not compile or test the C++/Rust message consumers. Running
`demo_native.py` performs that verification explicitly. These examples use files,
with no robot connection, transport or SDK pub/sub.

## Verify the integration

Use a Python environment with the published generator and test tools:

```sh skip
uv pip install 'dimos-cdrgen==0.1.2' numpy build pytest pytest-asyncio pytest-env
python -m pytest --noconftest -o addopts='' -q dimos/message_codegen/test_integration.py
```

`dimos/message_codegen/test_bundled.py` is the artifact acceptance suite. It
skips during ordinary unit-test runs; build the wheel and sdist first, then run:

```sh skip
DIMOS_BUNDLE_DIST=dist DIMOS_BUNDLE_WHEELHOUSE=wheelhouse DIMOS_BUNDLE_REQUIRED=1 \
  python -m pytest --noconftest -o addopts='' -q dimos/message_codegen/test_bundled.py
```

With `DIMOS_BUNDLE_REQUIRED=1`, missing wheel, sdist or wheelhouse inputs fail
the run instead of skipping. The `message-artifacts` CI job runs exactly that
command against the artifacts it built. The suite checks required message
classes, schema closure, provider uniqueness, CDR roundtrips, and installed
wheel/sdist/editable/git layouts, including external `Header` identity.

Generic generator regression tests belong to the cdrgen repository.
