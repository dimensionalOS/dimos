# Authoring dimOS packages

A dimOS package is an ordinary Python distribution. Use `tool.dimos` to package
Python declarations, native sources and resources, and to generate standard
`dimos.blueprints` entry points. Installation and `dimos list` do not compile native
code. A selected native module builds during preparation, in a writable cache.

```toml
[build-system]
requires = ["scikit-build-core>=1.0,<2", "dimos-build-config"]
build-backend = "scikit_build_core.build"

[project]
name = "acme-probe"
version = "0.1.0"
dynamic = ["entry-points"]
dependencies = ["dimos"] # Pin a compatible release/source artifact for deployment.

[tool.dimos]
package = "src/acme_probe"

[tool.dimos.blueprints]
probe = "acme_probe.module:Probe"

[[tool.dynamic-metadata]]
provider = "dimos"
```

`package` names one relative Python package directory. Blueprint names use lowercase
kebab-case and targets use `module:object`; values are not imported during builds.
Declare these entries only here, not again in `project.entry-points`. Resource-only
packages may omit blueprints and the dynamic metadata plumbing. Ordinary Python,
Rust/C++ declarations and isolated Python declarations use the same packaging API.
There is no `packaging` selector, native target DSL, custom registry or backend wrapper.

The independent build integration is in `packages/dimos-build-config`. It is **not
published yet**. Build its wheel locally and make it available with `--find-links`
(or `PIP_FIND_LINKS` / `UV_FIND_LINKS`) for isolated builds. Do not assume an index
contains it. It depends only on a TOML parser for Python 3.10, not the dimOS runtime.

```bash
python -m build --wheel packages/dimos-build-config --outdir /tmp/dimos-build-wheels
# From an external package directory, with the build tool installed:
PIP_FIND_LINKS=/tmp/dimos-build-wheels python -m build
python -m pip install dist/*.whl
# Development: source changes are read on the next preparation.
PIP_FIND_LINKS=/tmp/dimos-build-wheels python -m pip install -e .
dimos list
dimos --transport zenoh --viewer none run acme-probe.probe
```

## Optional message definitions

With the matching CDR proposal's `dimos-message-build` wheel available, the same
package can own messages. Add `dimos-message-build==0.1.0` to
`build-system.requires`, keep the scikit-build-core backend above, and add:

```toml
[tool.dimos.messages]
source = "interfaces"
module = "acme_messages"
```

For example, `interfaces/acme_msgs/msg/Reading.msg` contains:

```text
int32 sequence
float64 value
```

Declare `dimos-message-build==0.1.0`, `rosbags==0.11.0`, and `numpy>=1.26.4`
as project dependencies for these generated Python types. The generated module
must differ from the application package name. No `setup.py` is required.
Message-only distributions may omit `tool.dimos.package` and blueprints; keep
`dynamic = ["entry-points"]` and the `dimos` metadata provider.

```sh
# Optional inspection step; run in an environment containing the generator.
dimos-message-generate
# Builds from .msg inputs automatically; no prerequisite generation step.
python -m pip install .
# After editing .msg inputs, regenerate through editable installation:
python -m pip install -e .
```

```python
from acme_messages.acme_msgs.msg import Reading
reading = Reading(sequence=1, value=21.5)
```

The explicit command and packaging invoke the same generator and configuration.
Generated Python values, schemas and native source resources are included in the
wheel; installing a wheel does not generate or compile. Message generation does
not compile native modules and never runs at import. Restart Python after schema
changes. The generator is optional: ordinary source packages retain their thin
build dependencies and do not require CDR tooling. These proposal tools are not
published; provision matching local wheels as above. The CDR layer supplies the
generator; this layer supplies its packaging integration.

## Native source layout and runtime API

Place all local native inputs under the importable package, for example
`src/acme_probe/native/{Cargo.toml,Cargo.lock,src/main.rs}`. Keep resources under
`src/acme_probe/resources`. Cargo dependencies and CMake FetchContent dependencies
remain their respective build systems' responsibility; pin them and provision
network access or offline caches before first use. A source wheel is platform
independent as an archive, not a promise that its native code supports every OS.

```python skip
from dimos.core.native_module import NativeModule, NativeModuleConfig

class ProbeConfig(NativeModuleConfig):
    source_package: str | None = "acme_probe"
    source_dir: str | None = "native"
    executable: str = "target/release/package_probe"
    build_command: str | None = (
        "cargo build --release --locked --bin package_probe --target-dir target"
    )

class Probe(NativeModule):
    config: ProbeConfig
```

The module configuration owns the exact build target. The cache copies the entire
source directory, then runs both build and executable there. Installed sources are
not writable build directories. Pass explicit package resource paths when a native
process needs files outside that source tree. See the
[canonical Rust probe](/examples/packages/native/README.md). Its separate text
resource deliberately verifies resource inclusion and lookup; applications can use
ordinary config strings when no resource is needed. The second binary tests target
selection, rather than introducing another packaging mode.

`source_package` is not in released dimOS 0.0.14: use a host artifact containing this
change until a compatible release exists. `source_package=None` retains checkout/main
fallback. Native package authoring is source-only: Python package builds and installs
never compile native modules. Compilation belongs to selected-module preparation.

## Build settings and boundaries

The config provider uses `scikit-build-core.config.default`; the separate metadata
provider generates the standard entry-point table without importing application code.
The config provider also invokes the optional message generator when `messages` is
declared. The defaults
set `wheel.cmake=false`, `sdist.cmake=false`, `wheel.platlib=false`, `wheel.py-api="py3"`
and `editable.rebuild=false`. Source files and resources are included in wheel and
sdist; common build outputs, virtualenvs, `.env*`, private-key files and VCS metadata
are excluded. Inspect artifacts: filename exclusions are not a secret scanner.
Symlinks in the package tree are rejected; local build inputs must be self-contained.

Project `tool.scikit-build` settings override provider defaults; environment and
frontend `-C` settings have still higher precedence. List settings replace the whole
list. Use standard settings for extra sources/exclusions, without disabling the
source-only defaults. Obvious TOML/environment compilation conflicts are rejected.
The provider does not receive the final merged configuration, so arbitrary overrides,
other providers or `SKBUILD_NO_ENTRYPOINT_CONFIG=1` cannot be policed completely.
Those are outside the supported source-only contract, not a prebuilt option.

Editable installs keep source changes visible; new files or entry-point changes may
require reinstalling. Restart the module after edits. Native source changes invalidate
the cached executable; generated intermediate outputs remain available for incremental
builds. Source preparation holds the workspace lock while the existing native builder
runs. Unchanged inputs reuse the executable. Untracked toolchain
changes may require `--build-native`. Stop dependent processes before rebuilding or
cleaning caches. `pip uninstall acme-probe` removes installed package files, not
source checkouts or dimOS caches; use `dimos cache clean` separately when appropriate.
Build commands and installed Python packages are trusted executable code, not sandboxes.

References: [configuration providers](https://scikit-build-core.readthedocs.io/en/latest/configuration/entrypoint_config.html),
[dynamic metadata](https://scikit-build-core.readthedocs.io/en/latest/configuration/dynamic.html),
[editable installs](https://scikit-build-core.readthedocs.io/en/latest/configuration/editable.html).
