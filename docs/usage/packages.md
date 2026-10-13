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

## Consumer projects and Git repositories

`dimos package` delegates installation and resolution to uv. It edits the selected
consumer's ordinary `pyproject.toml` and `uv.lock`; it does not introduce a package
registry, solver or lock format. Install uv first; the local acceptance check uses uv 0.12.21. These commands need a compatible
dimOS host and a resolvable `dimos-build-config` build dependency. Both are release
gates: the provider is still unpublished and the historical dimOS 0.0.14 release
does not contain this stack's package support.

Use `--project` to choose a consumer directory; `add` creates a minimal Python
3.10–3.12 application manifest if needed. Without it, package commands use the
nearest project (or create one in the current directory). They refuse to implicitly
edit the dimOS source checkout. Local source paths are relative to the calling
directory, independently of the consumer location.

The following repository and package names are placeholders:

```bash skip
dimos --project ./robot package add "git+https://github.com/OWNER/REPO.git" \
  --tag v1.0.0 --subdirectory packages/acme-probe --extra camera
dimos --project ./robot package sync
dimos --project ./robot list
dimos --project ./robot --transport zenoh --viewer none run acme-probe.probe
dimos --project ./robot package update acme-probe
dimos --project ./robot package remove acme-probe
```

Keep a compatible dimOS host as a direct consumer dependency if it should remain
installed after removing plugin packages. Until releases exist, supply the exact
compatible host/provider wheels through normal uv wheelhouse/index settings.
`--extra` requests standard package extras; platform markers and Python constraints
remain uv's responsibility. Only the declared dimOS native targets avoid install-time
compilation; other source dependencies can still require compilers.

`add` also accepts `--rev` (a commit), `--branch`, or `--editable /local/package`.
Git URLs can carry `#subdirectory=...` instead of the separate option. A convenience
selector is available for statically declared project names:

```bash skip
dimos package add "github:OWNER/REPO#acme-probe" --tag v1.0.0 --project ./robot
dimos package add --editable ../author/packages/acme-probe --project ./robot
```

The shorthand examines the root project and declared `tool.uv.workspace.members`
at the selected Git revision, honors member exclusions and requires one matching
canonical `project.name`. It does not import author code. Without a selector it uses
the repository's default branch, not an assumed branch named `main`. The member is
translated to a normal Git source with a subdirectory; its source namespace is
independent of its distribution, import-package and blueprint names. Private repos
use Git's existing HTTPS credentials; explicit `git+ssh://` sources use SSH credentials.

`sync` and `list`/`run` require an existing current lock and never refresh a branch
tip. `update` explicitly refreshes resolutions within the declared selectors; it
does not switch a pinned revision or tag to a different selector. uv may retain a
Git workspace revision when unselected siblings still pin it; use `package update`
without package names to refresh the full closure. Offline updates use cached
revisions. Failed edits
restore the previous manifest and lock. If installation was interrupted, repeat
`sync` to reconcile the environment; metadata rollback does not roll back installed
files. Edits are serialized locally. Removing a distribution does not delete native
or isolated runtime caches; stop running modules before upgrading/removing their
packages, and use cache cleanup separately.

`list`/`run` dispatch through uv into that consumer before importing the robot CLI,
with lock validation and isolated Python import paths. A nearest non-dimOS project
is also selected automatically. Missing dimOS, a missing/stale lock or failed sync
fails the command; it never falls back to the global dimOS environment. The existing
in-repository workflow and ordinary Module behavior remain unchanged. Python runtime
isolation is optional and continues to use `IsolatedPythonModule`/`PackageProject`.

One repository can own multiple independently packaged subprojects, each with its
own `pyproject.toml` and `tool.dimos` declaration. An optional uv authoring workspace
shares a lock/environment and therefore requires compatible dependencies. Modern
uv resolves Git workspace sibling dependencies at the same repository revision;
the generic acceptance fixture exercises that path with the thin provider. This is
not a promise for ordinary pip or standalone wheels: `tool.uv.sources` is not
standard published dependency metadata. Published wheels must declare independently
resolvable sibling dependencies. Check release builds with `uv build --no-sources`
and install artifacts outside the author checkout. Package-owned native/runtime
snapshots cannot rely on parent-relative source trees outside their packaged directory.

## Release preparation

Before advertising index-based installation, release the independent build provider
and a compatible dimOS host. Test those exact artifacts in a fresh consumer outside
the author checkout, including discovery without native compilation and preparation
of a selected module. Record the supported uv version and Python range. For a
monorepo release, verify each distribution can resolve its siblings without
`tool.uv.sources`. These are release gates; this consumer change publishes no artifacts.
The Rust/C++ SDK examples remain manual checks described in their READMEs.

## Module instance names

Classes defined outside `dimos` and `dimos.*` default to their qualified Python
name, for example `acme_robot.module.Worker`. Classes defined inside dimOS keep
their existing short names. The defining class controls this identity: reexporting
a class or registering it through another package does not rename it, while a
subclass uses its own definition path.

Set `instance_name="front_camera"` on a blueprint to choose a stable explicit
identity. Different classes must not share an instance name in one composition;
repeating the same class and instance name keeps the last blueprint's configuration.
The identity is also used for external modules' RPC endpoints and isolated Python
facades. Stream names, topics and TF frames are unchanged; `.namespace()` remains
an explicit way to isolate them.

Module configuration accepts `--acme_robot.module.Worker.speed` or the escaped
`--acme_robot_module_Worker.speed`. Environment variables use the escaped root,
for example `ACME_ROBOT_MODULE_WORKER__SPEED=2`. Dots and slashes become underscores;
configuration rejects ambiguous escaped roots instead of silently overriding one.

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
provider generates the standard entry-point table. Both only read TOML. The defaults
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

## Messages, resources and isolated dependencies

Keep typed ports and Python contract declarations authoritative. Share one
contract/message distribution across dependent packages; generated native
bindings must use matching schemas and serialization. Python pickle is not a
native interoperability format.

Include small immutable resources as package data and use `importlib.resources`
or an installed package-relative path with an appropriate lifetime. Give native
processes absolute resource paths. Keep downloaded models, logs, recordings and
mutable state out of installation directories.

For isolated Python, use [PackageProject](/dimos/experimental/isolated_python/README.md#installed-package-projects).
The runtime project declares its own dependencies and optional lock; dimOS and
shared contract content must align with the host. An installed contract does not
need to pull heavy implementation dependencies into the host. Normal uv index,
wheelhouse and offline settings control dependency provisioning. Release projects
should ship locks and deploy verified artifacts. Isolation separates dependencies;
it does not isolate privileges or make untrusted Python/native code safe.
