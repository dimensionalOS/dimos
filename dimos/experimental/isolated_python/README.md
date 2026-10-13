# Experimental Isolated Python Modules

`IsolatedPythonModule` runs a Python module in a separate dependency environment
while preserving dimOS streams, RPCs, skills, module references, and lifecycle
management. Its API is experimental and may change without compatibility aliases.

## Project layout

Keep the host contract in `dimos/` and its isolated project outside the package:

```text
dimos/my_module/contract.py
native/python/my_module/
├── pyproject.toml
└── my_runtime/
    └── runtime.py
```

Define the host-visible contract:

```python skip
from dimos.core.core import rpc
from dimos.experimental.isolated_python.module import (
    IsolatedPythonModule,
    IsolatedPythonModuleConfig,
)


class MultiplierConfig(IsolatedPythonModuleConfig):
    initial_multiplier: int = 2


class Multiplier(IsolatedPythonModule):
    project_dir = "native/python/my_module"
    implementation = "my_runtime.runtime:MultiplierRuntime"
    config: MultiplierConfig

    @rpc
    def get_multiplier(self) -> int:
        raise NotImplementedError
```

The isolated runtime imports and implements that contract:

```python skip
from typing import Any

from dimos.core.core import rpc
from dimos.my_module.contract import Multiplier


class MultiplierRuntime(Multiplier):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._multiplier = self.config.initial_multiplier

    @rpc
    def get_multiplier(self) -> int:
        return self._multiplier
```

Every contract RPC and skill must be overridden with a compatible signature and
the same `@rpc` or `@skill` classification. Startup fails if the runtime inherits
a contract stub or changes its signature or classification.

## Runtime behavior

During `build()`, dimOS uses `uv run` to sync the declared project and prepare a
cached overlay containing dimOS from the shared checkout and its dependencies.
The first build can take minutes to download; later builds reuse the cache.
If `pixi.toml` exists, Pixi supplies `uv`. If `uv.lock` exists, dimOS uses
`--frozen` and treats the lockfile as the source of truth.

The runtime project and child dimOS come from `get_project_root()`, the shared
LFS checkout helper. Development uses the current checkout, including local edits.
Installed hosts reuse the cached repository or clone `main` on first use. The child
installs dimOS from that checkout with `--with-editable`; its revision may differ
from the host's. Existing clones are not updated automatically. The checkout must
contain the declared project. Restart running modules after editing sources.

The project's `.python-version` and `requires-python` select its Python
version. Environments are stored under the dimOS cache directory in
`isolated-python/<project-path-hash>/.venv`, so projects do not share environments.
Preparation also warms the DimOS overlay before starting the readiness deadline.

Runtime projects are not packaged in dimOS wheels or source distributions.
The examples use `[tool.uv] package = false` and import runtime code from the
project working directory. Load models and download checkpoints in `start()`,
keeping imports and construction lightweight.

The host contract retains the public module name and forwards contract RPCs to a
unique internal endpoint. Ordinary dimOS serialization and transport handle RPC
values, exceptions, timeouts, async methods, skills, streams, and module
references. Restarting the contract starts a fresh interpreter and reloads the
runtime package.

Runtime classes and tests live outside `dimos/`, so host blueprint discovery and
source checks do not scan them. Run runtime tests with their project's pytest
configuration and `--confcutdir=.` to avoid loading host fixtures.

## Example

The source tree includes a complete example with a locked external project:

```bash
uv run python -m dimos.experimental.isolated_python.example.run
```

The example demonstrates streams, RPCs, skills, an injected module reference,
restart behavior, and automatic shutdown.

## Runtime development

Root pytest and mypy check `dimos/`; isolated projects live outside that tree. Run their tests and type
checks inside their own environment. For GraspGenX, from the repository root:

```bash
cd native/python/graspgenx
export UV_PROJECT_ENVIRONMENT="${XDG_CACHE_HOME:-$HOME/.cache}/dimos/graspgenx-tests"
uv run --frozen --group tests --with-editable ../../.. python -m pytest
uv run --frozen --group lint --with-editable ../../.. python -m mypy
```

The tests mock the model backend and need no GPU or checkpoints. Runtime mypy
reads the annotated dimOS and GraspGenX source despite their missing `py.typed`
markers. Each runtime owns its lint configuration and dependencies.

## Installed package projects

A separately installed contract can select its own runtime project:

```python
from dimos.experimental.isolated_python.package import PackageProject

class Detector(IsolatedPythonModule):
    package_project = PackageProject("acme_detector", "runtime", "acme-detector")
    implementation = "runtime:DetectorRuntime"
```

The owning package must be an unpacked wheel or editable filesystem package.
Include the runtime's `pyproject.toml`, implementation sources and, for a locked
release, `uv.lock` as package data. The runtime declares its own DimOS, contract
and implementation dependencies. Configure normal uv indexes or wheelhouses to
make the matching artifacts available; no host checkout is implicitly installed.
Add `(distribution_name, import_package)` pairs to `shared_packages` when other
shared message or API packages must match the host too.

Preparation checks the installed distribution version, Python source fingerprint
and wheel payload records for DimOS and each shared contract. Different artifacts
with the same version fail before the child starts. Editable dependencies must
be editable on both sides with matching code. This alignment check is not a
signature verifier: deployment locks and artifact hashes remain the package
manager's responsibility, and native ABI compatibility is still separate.

Projects are copied into a content-addressed cache before uv writes an environment
or lockfile. The key covers project contents, host contracts, Python and platform.
A file lock serializes preparation; project publication is atomic and a failed uv
sync is retried on the next build. Existing locks use `--frozen`; an unlocked
development project resolves on its first preparation and reuses the cached lock.
Launch uses the prepared environment with `--no-sync`. Host `PYTHONPATH` and
`PYTHONHOME` are removed so they cannot defeat dependency isolation.

Editing packaged runtime sources, dependencies or a shared contract gives a new
cache on the next module construction. Restart after reinstalling/updating a
package. `dimos cache clean` can reclaim these regenerable caches after runs stop;
uninstalling a distribution does not delete development sources or runtime data.
Repository-relative `project_dir` projects retain their existing behavior.

The [independent Python example](../../../examples/packages/python/pyproject.toml)
exports both an ordinary Python consumer and an isolated consumer of native
`Twist` messages. Its runtime pins `packaging==25.0`, independently of the host's
version. The example is deliberately an unlocked development project; generate
and ship a lock against your deployment's actual DimOS/contract artifacts for a
release. No additional RPC protocol is introduced.

For editable development, configure uv sources for both DimOS and the contract
as editable dependencies in the runtime project. Use absolute development paths,
or relative paths contained inside the packaged runtime directory: parent-relative
paths outside that directory do not survive its cache snapshot. Native/toolchain
build outputs and arbitrary sibling repositories are not copied implicitly.
