# Runtime Environments

Runtime environments let a blueprint choose how selected processes launch without making that choice part of the module class.

Use them when one module needs a different Python environment, when a Python project owns its own worker environment, or when a native module should get its executable settings from a named environment.

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

## Python project workers

Use `PythonProjectRuntimeEnvironment(name, project)` when the worker module lives in a separate Python project that should own project-local state.

```python skip
from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.runtime_environment import PythonProjectRuntimeEnvironment

runtime = PythonProjectRuntimeEnvironment(
    name="grasping",
    project=Path("packages/my-grasping-worker"),
)

blueprint = (
    autoconnect(GraspingModule.blueprint(), ConsumerModule.blueprint())
    .runtime_environments(runtime)
    .runtime_placements({GraspingModule: "grasping"})
)
```

The project directory must contain `pyproject.toml`. DimOS uses these conventions only:

- `.venv/` contains the uv-managed worker Python environment.
- `.pixi/` is optional Pixi state. If `pixi.toml` exists, DimOS treats the project as Pixi-backed.

Prepare project runtimes explicitly before running a blueprint:

```bash
dimos runtime prepare <blueprint>
dimos runtime prepare <blueprint> --runtime grasping
```

Preparation is blueprint-scoped and acts only on runtimes used by active module placements. Direct `PythonVenvRuntimeEnvironment` entries are externally managed no-ops.

`dimos run` is non-mutating. It does not install, sync, or update environments. If `.venv/bin/python` is missing, the run fails fast with a diagnostic telling you which `dimos runtime prepare` command to run. This is an existence-only staleness check: after changing `pyproject.toml`, `uv.lock`, `pixi.toml`, or `pixi.lock`, rerun `dimos runtime prepare` yourself.

Prepare commands run from the project directory on every invocation:

```bash
# uv-only project
uv venv --seed
uv sync

# Pixi-backed project
pixi install
pixi run uv venv -p .pixi/envs/default/bin/python --seed
pixi run uv sync
```

Worker launch also stays non-mutating:

- uv-only: `uv run --no-sync python -m dimos.core.coordination.venv_worker_entrypoint ...`
- Pixi-backed: `pixi run uv run --no-sync python -m dimos.core.coordination.venv_worker_entrypoint ...`

### GPD worker demo

`packages/dimos-gpd-worker-demo/` is an import-safe project-runtime demo. It pins `gpd @ git+https://github.com/TomCC7/gpd.git@c088d8ae2f7965b067e9a12b3c0dacdbe9da924a`, includes optional Pixi native build dependencies, and exposes `GpdImportProbe.import_gpd_core()`. The RPC lazily imports `gpd.core` inside the worker runtime and returns `gpd import ok: ...` on success.

Prepare the demo project when you want the optional integration to run:

```bash
cd packages/dimos-gpd-worker-demo
pixi install
pixi run uv venv -p .pixi/envs/default/bin/python --seed
pixi run uv sync
cd ../..
uv run pytest dimos/core/test_gpd_worker_demo.py -q
```

The same blueprint pattern can be prepared through `dimos runtime prepare <blueprint> --runtime dimos-gpd-worker-demo` if you expose the demo blueprint in a registry entry. Without a prepared `.venv`, the Pixi/GPD integration tests skip; import-safety and placement tests still run.

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

## Current limits

- Venv workers run on the same machine as the coordinator.
- The venv must contain compatible DimOS worker runtime code. In phase 1, this usually means the venv can import the same source checkout or an equivalent installed `dimos` package.
- Direct `PythonVenvRuntimeEnvironment` entries are not created, installed, or synchronized by DimOS. Prepare those environments outside DimOS before running the blueprint.
- There is no remote deployment agent yet. Runtime environments only select local process launch material.
