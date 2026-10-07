# Independent DimOS packages

Each subdirectory is its own Python distribution. Install into the coordinator's
environment and use normal Python imports to compose its public module classes.
Only CLI-runnable exports need a `dimos.blueprints` entry point.

- [native](native/README.md): small real executable/resource/lifecycle probe.
- `rust`: Rust SDK producer, exports `dimos-package-rust.ping`.
- `cpp`: C++ SDK consumer/echo, exports `dimos-package-cpp.pong`.

## Build inputs and installation

The SDK examples target Linux x86_64 with Python 3.10-3.12. They pin the SDK to
`dbb630f8e298218e9862e03dc24d6ba47080d20c` and messages to
`04d78e8622500244123ba9cefa4c51b4cb454549`. Rust records transitive Git and registry
inputs in its committed Cargo.lock and builds with `--locked`. Its message source
retains the SDK's branch identity so Cargo uses one set of message types; the
lockfile, not the branch tip, selects the schema revision. Updating that lockfile
is an explicit compatibility change. C++ selects the same messages with a fixed
FetchContent revision. Neither recipe relies on paths inside a DimOS checkout.

These examples pin the Python distribution metadata to 0.0.14, the version on the
validated source revision. A version number alone does not identify that source:
use a DimOS wheel built from the matching revision (and lock its hash) until a
release containing the required interfaces is available. These example packages
are not published to a registry.

Rust needs Cargo/rustc and CMake; C++ needs a C++20 compiler, CMake, pkg-config,
LCM development files, Zenoh-C and Zenoh-C++ 1.10.1 (Zenoh-C with unstable API).
Use `CMAKE_PREFIX_PATH` and `PKG_CONFIG_PATH` for locally provisioned dependencies.
When unpacking a relocatable dependency archive, check its pkg-config prefix:
upstream archives may still contain `/usr/local`. No system installation is
required when these paths point to a private dependency prefix.

```bash
# From each project directory, using a prepared build environment:
python -m build
# Install the matching platform wheel into the coordinator environment:
python -m pip install dist/*.whl
```

scikit-build-core supplies the PEP 517 backend and platform tags. CMake installs
only the package executable via the `python` install component; it does not copy
the entire SDK's development tree into the wheel. Rust's CMake target invokes
Cargo directly. The executable is a process, not a CPython extension, so its
wheel uses `py3-none-<platform>`.

C++ wheels still require compatible liblcm and libzenohc at runtime. Provision
those libraries using the deployment's existing package/environment tools; for
a private prefix use its loader search path. The examples do not bundle or
promise manylinux compatibility for those libraries. Inspect dependencies with
`ldd` before deployment. Do not distribute a wheel built with CPU-specific flags
as a generic architecture wheel.

For offline builds, prepare Cargo's registry/Git cache and use `CARGO_NET_OFFLINE=true`.
CMake supports its standard `FETCHCONTENT_SOURCE_DIR_DIMOS_SDK` and
`FETCHCONTENT_SOURCE_DIR_DIMOS_MESSAGES` overrides, plus the SDK's dependency
overrides, for already provisioned source trees. Use the pinned revisions for
release builds; local overrides are also useful for development. No resolver or
source downloader is added to the runtime.

## Runtime composition

```bash
dimos --transport zenoh --viewer none run dimos-package-rust.ping dimos-package-cpp.pong
```

The producer sends `geometry_msgs.Twist` on `data`. The consumer returns it on
`confirm`, setting `angular.z` to its `sample_config` (42 by default). The producer
logs the returned value. Both use the same schemas and the coordinator's session
settings. The modules exchange typed messages; there is no native RPC claim.

`pip install -e .` follows the backend's editable behavior. Rebuild native changes
and restart processes; changing entry points also requires reinstalling metadata.
For an already built development artifact, explicitly pass its absolute path to
`RustPing.blueprint(executable=...)` or `CppPong.blueprint(executable=...)`.

A package that composes these modules declares Python distribution dependencies.
A native implementation that consumes their messages depends on the message
bindings, not their executable. Reusable native algorithms belong in a Cargo
library or CMake library target. Compiling several Rust modules into one process
is optional: the existing `dimos bake` registry is checkout-scoped and is not
required for package discovery or cross-process composition.
