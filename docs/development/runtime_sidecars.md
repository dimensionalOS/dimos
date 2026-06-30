# Simulator runtime modules

DimOS benchmark runtime integrations keep heavy simulator dependencies outside
the main DimOS environment while exposing each simulator as a first-class DimOS
module at the blueprint boundary. The target shape is a **Simulator Runtime
Module** placed into a named Python project runtime environment (uv or
Pixi-backed uv), not a long-running HTTP runtime API.

## Target boundary

- `packages/dimos-runtime-protocol` contains backend-neutral Pydantic models,
  compatibility helpers, and codecs. The models are reused by module RPCs; they
  do not imply HTTP.
- Runtime packages may import `dimos` for module and blueprint definitions, but
  coordinator-visible imports must remain simulator-import-safe. Heavy backend
  SDK imports stay on the placed worker/runtime path.
- Control-plane operations are DimOS RPCs on the runtime module: `describe`,
  `reset`, synchronous `step`, and `score`.
- Data-plane outputs are typed DimOS streams. Camera observations use
  `Image`/`CameraInfo`; motor state and runtime events use typed protocol
  models. Large NumPy arrays must not be returned through `step()` RPC payloads.
- Simulator mutation and camera capture run on a simulator owner thread. MuJoCo /
  Robosuite render contexts are thread-sensitive; RPC handlers marshal work to
  that owner thread rather than calling simulator APIs directly from pubsub/RPC
  worker threads.

The legacy HTTP runtime servers, `/payloads/{id}` image fetch endpoints,
`RuntimeSidecarClient`, and HTTP-first demos have been removed from the active
runtime path. Migration remains complete only while benchmark execution uses the
module RPC and stream surfaces described above.

## Package-local blueprint helpers

Each runtime package should expose a helper that registers its named Python
project runtime environment and places only the simulator runtime module there.
Callers use the helper instead of writing placement boilerplate.

```python skip
from dimos_robosuite_sidecar.blueprint import robosuite_runtime_blueprint

blueprint = robosuite_runtime_blueprint()
```

The helper follows the standard runtime environment API:

```python skip
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.runtime_environment import PythonProjectRuntimeEnvironment

environment = PythonProjectRuntimeEnvironment(
    name="dimos-robosuite-runtime",
    project="packages/dimos-robosuite-sidecar",
)

blueprint = (
    autoconnect(RobosuiteRuntimeModule.blueprint())
    .runtime_environments(environment)
    .runtime_placements({RobosuiteRuntimeModule: environment.name})
)
```

Prepare project runtimes explicitly before running placed modules. `dimos run`
is non-mutating and fails fast if the runtime project has not been prepared.

```bash
dimos runtime prepare <blueprint> --runtime <runtime-name>
```

See `docs/usage/runtime_environments.md` for uv and Pixi-backed preparation
commands.

## Fake runtime smoke demo

The fake demo requires no Robosuite or LIBERO installation and validates the
module-native runtime path: DimOS RPC control, lightweight `step()` responses,
typed stream publication, runtime-plan resolution, and artifact writing.

```bash
PYTHONPATH="packages/dimos-runtime-protocol/src:packages/dimos-fake-runtime-sidecar/src" \
  uv run python scripts/benchmarks/demo_fake_runtime_sidecar.py
```

Expected output includes `"ok": true` and artifacts under
`artifacts/benchmark/fake-runtime-smoke/`.

## Robosuite Panda Lift plumbing demo

The Robosuite demo should run against the placed `RobosuiteRuntimeModule` from
`packages/dimos-robosuite-sidecar`. The DimOS-facing path is module-native:

- runtime metadata comes from `describe()`;
- episode setup comes from `reset()`;
- deterministic benchmark advancement uses synchronous `step()` RPCs with the
  runtime-derived ordered `MotorActionFrame`;
- score/artifacts come from `score()` and the demo artifact writer;
- camera output is observed through `Image` and `CameraInfo` streams, not through
  fetched `.npy` payloads.

Run it from the host DimOS environment. The demo builds the package-local
Robosuite blueprint and deploys `RobosuiteRuntimeModule` into the prepared
`packages/dimos-robosuite-sidecar` Python project runtime; the host environment
does not need to install Robosuite.

```bash
uv run python scripts/benchmarks/demo_robosuite_panda_lift.py \
  --config dimos/benchmark/runtime/configs/robosuite_panda_lift.json
```

Use `--visual` only in a GUI-capable environment. Use `--rerun` to verify the
normal DimOS stream visualization path; Rerun must consume the module-published
streams rather than direct runtime-boundary SDK logging or HTTP payload fetching.

The demo writes artifacts under
`artifacts/benchmark/robosuite-panda-lift/`, including runtime description,
resolved plan, motor trace, score when available, image/camera stream summaries,
and cleanup status.

## LIBERO-PRO registered-task runtime demo

The LIBERO-PRO demo should run against the placed `LiberoProRuntimeModule` from
`packages/dimos-libero-pro-sidecar`. LIBERO-PRO assets remain explicit: startup
or reset validates the prepared asset layout, and asset download/bootstrap happens
only when requested by an explicit preparation command or flag.

The DimOS-facing path is the same module-native contract as Robosuite:

- `describe()` exposes task/runtime metadata and the ordered Panda motor surface;
- `reset()` validates prepared BDDL/init-state assets and establishes episode
  state synchronously;
- `step()` advances the backend on the simulator owner thread and returns
  lightweight reward/done/success/motor metadata;
- camera output is observed through `Image` and `CameraInfo` streams;
- `score()` writes backend-owned task score metadata.

Example prepared-asset run:

```bash
LIBERO_CONFIG_PATH=/path/to/libero-config \
PYTHONPATH=/path/to/LIBERO-PRO \
uv run python scripts/benchmarks/demo_libero_pro_runtime.py
```

Use `--visual` only when the underlying MuJoCo/Robosuite viewer stack can open a
local display. Use `--rerun` to verify the normal DimOS stream visualization path.

The default config is
`dimos/benchmark/runtime/configs/libero_pro_goal_task0.json`. It declares backend
`libero-pro` with common runtime fields plus registered-suite options for task
selection, init-state selection, controller, cameras, horizon, and asset roots.
Missing BDDL or init-state assets should fail before episode stepping with a clear
module reset/setup error.

## HTTP runtime removal audit

When reviewing future changes, classify references to the old HTTP runtime path:

- **migrated**: fake, Robosuite, or LIBERO behavior now covered by module-native
  RPCs and streams;
- **removed**: `RuntimeSidecarClient`, HTTP server entrypoints, `/payloads/{id}`
  endpoints, camera payload smoke scripts/tests, and HTTP-first demo plumbing;
- **non-runtime usage**: unrelated HTTP URLs such as Rerun proxy links or web UI
  documentation.

Do not introduce new simulator data-plane protocols while removing the old HTTP
path. Prefer existing DimOS transports and typed messages.
