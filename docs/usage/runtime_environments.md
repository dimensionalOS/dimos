# Runtime Environments

Runtime environments let a blueprint choose how selected processes launch without making that choice part of the module class.

Use them when one module needs a different Python environment, or when a native module should get its executable settings from a named environment.

## Python venv workers

A Python module can run in a named Python environment while other modules stay in the default worker pool.

```python skip
from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.runtime_environment import PythonVenvRuntimeEnvironment

runtime = PythonVenvRuntimeEnvironment(
    name="sensors",
    python_executable=Path("/opt/dimos-sensors/.venv/bin/python"),
)

blueprint = (
    autoconnect(CameraModule.blueprint(), ConsumerModule.blueprint())
    .runtime_environments(runtime)
    .runtime_placements({CameraModule: "sensors"})
)
```

`CameraModule` runs in a worker launched by `/opt/dimos-sensors/.venv/bin/python`. `ConsumerModule` remains in the default Python worker pool. Modules in the same named environment share that environment's worker pool. Modules in different named environments never share a worker process.

The venv worker uses the same DimOS worker protocol as the default worker. Lifecycle calls, streams, module refs, and RPCs work the same way.

## Import-safe venv modules

The coordinator imports module classes before it launches workers. A venv-deployable module file must therefore be importable in the coordinator environment.

Follow these rules:

- Keep the module class at a top-level import path that exists in both the coordinator and worker environments.
- Do not import worker-only dependencies at module import time.
- Import worker-only dependencies inside `start()`, stream callbacks, RPC methods, or helper functions called by those methods.
- Avoid class-level annotations that require worker-only packages. The coordinator resolves annotations while it builds the blueprint.
- Install the module package and its worker-only dependencies into the named worker environment.

Minimal pattern:

```python skip
from dimos.core.core import rpc
from dimos.core.module import Module


class VenvOnlyModel(Module):
    @rpc
    def run_model(self, text: str) -> str:
        from worker_only_package import predict

        return predict(text)
```

The demo package at `packages/dimos-demo-worker-module/` follows this pattern. Its publisher imports a runtime helper only inside a worker-side RPC. The test `dimos/core/test_venv_module_demo.py` proves the coordinator can import and build the blueprint, while the placed worker runs the module through the named runtime environment.

## Packaging convention

Place venv-deployable modules in their own Python package when they have a dependency closure that should not be installed in the coordinator environment.

Recommended layout:

```text
packages/my-venv-module/
├── pyproject.toml
└── src/my_venv_module/
    ├── __init__.py
    └── blueprint.py
```

The package's `pyproject.toml` should declare the worker dependencies needed by that module. The coordinator does not need those dependencies if the module imports them lazily.

Phase 1 packages may depend on the root `dimos` package. A future split can replace that with a smaller worker runtime package.

## Native module runtime environments

Native modules can reference a named native runtime environment instead of repeating executable/build settings in every config.

```python skip
from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.native_module import NativeModuleConfig
from dimos.core.runtime_environment import NixNativeRuntimeEnvironment

native_env = NixNativeRuntimeEnvironment(
    name="mid360-native",
    executable="result/bin/mid360_native",
    build_command="nix build .#mid360_native",
    cwd=Path("cpp"),
    env={"RUST_BACKTRACE": "1"},
)

class Mid360Config(NativeModuleConfig):
    runtime_environment: str | None = "mid360-native"
    host_ip: str = "192.168.1.5"

blueprint = autoconnect(Mid360.blueprint()).runtime_environments(native_env)
```

Precedence is deterministic:

1. The runtime environment provides defaults for `executable`, `build_command`, `cwd`, and environment variables.
2. Non-`None` config values, including subclass defaults, override `executable`, `build_command`, and `cwd`.
3. `extra_env` overlays the runtime environment's environment variables.
4. Module-specific config fields still become CLI args as before.

Legacy native configs without `runtime_environment` continue to work.

## Phase 1 limits

- Venv workers run on the same machine as the coordinator.
- The venv must contain compatible DimOS worker runtime code. In phase 1, this usually means the venv can import the same source checkout or an equivalent installed `dimos` package.
- DimOS does not create, install, or synchronize venvs for you yet. Prepare the environment before running the blueprint.
- There is no remote deployment agent yet. Runtime environments only select local process launch material.
