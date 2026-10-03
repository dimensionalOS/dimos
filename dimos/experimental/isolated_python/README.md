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

During `build()`, dimOS syncs the project's lockfile with `uv sync --frozen`,
then installs dimOS from the shared checkout using `uv pip install --no-deps --editable`.
Runtime projects explicitly declare their dependencies, including the DimOS
transport and message imports they use. Launch uses `uv run --no-sync`.
If `pixi.toml` exists, Pixi supplies the toolchain and `uv`.

The runtime project and child dimOS come from `get_project_root()`, the shared
LFS checkout helper. Development uses the current checkout, including local edits.
Installed hosts reuse the cached repository or clone `main` on first use. The child
installs dimOS from that checkout editable; its revision may differ
from the host's. Existing clones are not updated automatically. The checkout must
contain the declared project. Restart running modules after editing sources.

The project's `.python-version` and `requires-python` select its Python
version. Environments are stored under the dimOS cache directory in
`isolated-python/<project-path-hash>/.venv`, so projects do not share environments.
Preparation completes before starting the readiness deadline.

The bootstrap calls `run_runtime(stopping)` on its main thread after establishing
RPC service. The default waits for shutdown; thread-affine engines override it
and queue operations from RPC handlers. `stop()` must signal the loop to exit.

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
uv sync --frozen --group tests --group lint
uv pip install --no-deps --editable ../../..
uv run --no-sync python -m pytest
uv run --no-sync python -m mypy
```

The tests mock the model backend and need no GPU or checkpoints. Runtime mypy
reads the annotated dimOS and GraspGenX source despite their missing `py.typed`
markers. Each runtime owns its lint configuration and dependencies.
