# Experimental Isolated Python Modules

`IsolatedPythonModule` runs a Python module in a separate dependency environment
while preserving dimOS streams, RPCs, skills, module references, and lifecycle
management. Its API is experimental and may change without compatibility aliases.

## Project layout

Place the host contract beside a `python/` project that contains the concrete
runtime:

```text
my_module/
├── contract.py
└── python/
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
    implementation = "my_runtime.runtime:MultiplierRuntime"
    config: MultiplierConfig

    @rpc
    def get_multiplier(self) -> int:
        raise NotImplementedError
```

The sibling runtime imports and implements that contract:

```python skip
from typing import Any

from dimos.core.core import rpc
from my_module.contract import Multiplier


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

During `build()`, dimOS runs `uv sync` in the sibling project. If `pixi.toml`
exists, Pixi supplies `uv`. If `uv.lock` exists, dimOS uses `--frozen` and treats
the lockfile as the source of truth.

Source checkouts make the current dimOS checkout available to the runtime.
Installed hosts follow their recorded `direct_url.json` source: Git installations
reuse the resolved commit, direct wheels and archives reuse the original URL and
recorded hash, and local directory installations reuse that directory's current
contents. These sources must remain accessible when uv prepares the environment;
unpublished Git commits need no matching PyPI release. Unsupported or invalid
metadata fails preparation, without substituting an index build. Index installations
have no direct-source metadata and intentionally use unpinned `dimos`; host/runtime
version compatibility is not guaranteed in that case.

The sibling project's `.python-version` and `requires-python` select its Python
version. Environments are stored under the dimOS cache directory in
`isolated-python/<project-path-hash>/.venv`, so projects do not share environments.
Preparation also warms the DimOS overlay before starting the readiness deadline.

For runtime sources shipped inside wheels, use a flat project with
`[tool.uv] package = false` and include its sources, manifest, and lockfile as package
data. Python imports the runtime from the project working directory, without an
editable build writing metadata into `site-packages`. Load models and download
checkpoints in the runtime's `start()`, keeping imports and construction lightweight.

The host contract retains the public module name and forwards contract RPCs to a
unique internal endpoint. Ordinary dimOS serialization and transport handle RPC
values, exceptions, timeouts, async methods, skills, streams, and module
references. Restarting the contract starts a fresh interpreter and reloads the
runtime package.

Nested Python projects own their runtime classes and tests. Host blueprint discovery
and pytest collection stop at directories containing another `pyproject.toml`.
Runtime classes can use public names without becoming host registry entries.
Run runtime tests explicitly with their project's pytest configuration and
`--confcutdir=.` to avoid loading the host's test fixtures. Keep test environments
outside `dimos/`, where repository source checks would otherwise scan dependencies.

## Example

The source tree includes a complete example with a locked external project:

```bash
uv run python -m dimos.experimental.isolated_python.example.run
```

The example demonstrates streams, RPCs, skills, an injected module reference,
restart behavior, and automatic shutdown.
