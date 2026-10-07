# Independent DimOS packages

Start with [native](native/README.md), the canonical minimal Rust probe,
and the [package authoring guide](/docs/usage/packages.md). Each directory is a
separate ordinary Python distribution with a single `tool.dimos` declaration.

`rust` and `cpp` extend the same source-only recipe with real SDK typed ports.
Their wheels carry source, not compiled executables. Building/installing the Python
package does not invoke Cargo or CMake. Selecting a blueprint prepares only its
native target in the writable DimOS cache.

## Build inputs and installation

The SDK examples are validated on Linux x86_64/Python 3.12. SDK input is pinned to
`dbb630f8e298218e9862e03dc24d6ba47080d20c` and message schemas to
`04d78e8622500244123ba9cefa4c51b4cb454549`. Rust's committed Cargo.lock fixes the
schema revision while retaining the SDK's branch source identity. C++ FetchContent
uses exact revisions. Local source inputs live under the Python package; remote
native dependencies still need a provisioned cache or network at preparation time.
The examples require a matching DimOS host artifact with source_package support;
the historical 0.0.14 metadata version alone does not establish that provenance.

Build the unpublished `packages/dimos-build-config` wheel first and expose it with
PIP_FIND_LINKS/UV_FIND_LINKS. Then run `python -m build` in each example and install
its source-only wheel. See the authoring guide for exact commands.

Rust preparation requires Cargo/rustc. C++ preparation requires CMake, a C++20
compiler, pkg-config, LCM development files and Zenoh-C/Zenoh-C++ 1.10.1 with unstable
API enabled. Set CMAKE_PREFIX_PATH, PKG_CONFIG_PATH and the loader path for a private
dependency prefix. These dynamic libraries remain runtime requirements; a universal
source wheel does not promise native compatibility on every platform.

For offline preparation use Cargo's normal caches and CARGO_NET_OFFLINE=true.
CMake's standard CMAKE_TOOLCHAIN_FILE environment variable can point to a file with
FETCHCONTENT_SOURCE_DIR_DIMOS_SDK, FETCHCONTENT_SOURCE_DIR_DIMOS_MESSAGES and SDK
dependency overrides. Match the pinned revisions. No new resolver is introduced.

## Runtime composition

```bash
dimos --transport zenoh --viewer none run dimos-package-rust.ping dimos-package-cpp.pong
```

The producer sends `geometry_msgs.Twist` on `data`. The consumer returns it on
`confirm`, setting `angular.z` to its `sample_config` (42 by default). The producer
logs the returned value. Both use the same schemas and the coordinator's session
settings. The modules exchange typed messages; there is no native RPC claim.

Editable source changes are rebuilt on the next module preparation; reinstall after
entry-point changes or adding files. Uninstall removes the Python distribution,
while native cache cleanup remains a separate explicit operation.

## Python and isolated Python

`python` exports `dimos-package-python.observer` and
`dimos-package-python.isolated`. Both consume the native producer's `Twist`.
The lightweight installed declaration owns a runtime project whose dependency
on `packaging==25.0` is independent of the host's packaging version. Build this
pure Python wheel with `python -m build examples/packages/python`.

The runtime must resolve the same DimOS/contract artifacts as the host from your
configured uv index or wheelhouse. Its development project is unlocked; a release
should generate and include `uv.lock` against its deployment artifacts. Detailed
runtime, editable-source and cache semantics are in the
[isolated Python documentation](../../../dimos/experimental/isolated_python/README.md#installed-package-projects).

## Manual composition

From the repository root, build a compatible host wheel with
`python -m build --wheel`. Set `DIMOS_ALLOW_MISSING_COCKPIT=1` to build a UI-less
host without browser assets. Use a Python interpreter matching the host wheel's
ABI and provision the native libraries described above.

After installing the Rust, C++ and Python example wheels into a compatible host,
run all four blueprints together:

```bash
dimos --transport zenoh --viewer none run dimos-package-rust.ping dimos-package-cpp.pong dimos-package-python.observer dimos-package-python.isolated
```

The ordinary Python observer uses the host environment; the isolated observer
uses its packaged runtime project with `packaging==25.0`. Provision that project's
dependencies through your configured uv index or wheelhouse before running it.
Stop the composition with Ctrl-C. Native compilation happens only when a selected
module prepares; package builds and installs remain source-only.
