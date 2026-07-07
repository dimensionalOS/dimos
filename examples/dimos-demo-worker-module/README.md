# Demo Worker Runtime Project

This is a dependency-light example of splitting a coordinator-imported Module
Contract from a worker-only Runtime Implementation.

- `dimos_demo_worker_module.contract.DemoWorkerModule` is safe for the
  coordinator to import. It declares the module's RPC surface and avoids
  runtime-only dependency imports.
- `dimos_demo_worker_module.runtime.DemoWorkerRuntimeModule` subclasses that
  contract and is imported only in the selected Python Runtime Project worker.
- `dimos_demo_worker_module.blueprint.demo_worker_runtime_blueprint` shows how
  to register a Runtime Project and place the contract into it.

The project intentionally keeps dependencies minimal, but includes `inflection`
as a runtime-only dependency to prove the contract can be imported without the
worker dependency stack. DimOS automatically adds its own import path when
launching the runtime worker; the demo only adds the example package source and
compatible dependency paths. The example lockfile only captures the runtime-only
dependency closure. Sync from the checked-in lockfile before running it:

```bash
cd examples/dimos-demo-worker-module
uv sync --locked
```

Run the blueprint from the repository's main environment at the repo root:

```bash
cd ../..
uv run python examples/dimos-demo-worker-module/demo_run_blueprint.py
```

The script prints the main Python executable, then calls RPC methods on the
runtime-placed module. The reported worker Python should be under this example's
`.venv/`, showing that the coordinator script ran in the main environment while
the module ran in the Runtime Project environment. It also prints a value derived
from the runtime-only dependency.
