# Runtime sidecars

DimOS benchmark runtime sidecars keep heavy simulator dependencies outside the
main DimOS environment while still exercising the real DimOS control path.

## Boundaries

- `packages/dimos-runtime-protocol` contains only Pydantic protocol schemas,
  compatibility checks, and codecs. It must not import `dimos`, Robosuite,
  LIBERO, or OmniGibson.
- Sidecar packages import `dimos_runtime_protocol` and their backend SDKs, but
  not the main `dimos` package.
- The remote runtime boundary is synchronous HTTP in this slice:
  `/health`, `/describe`, `/reset`, `/step`, `/score`, and referenced
  observation payloads under `/payloads/{id}`.
- The Robosuite sidecar intentionally serves HTTP requests on one thread.
  MuJoCo / Robosuite render contexts are thread-sensitive; keeping `reset`,
  `step`, offscreen camera payload capture, and the interactive viewer on the
  same server thread avoids corrupted camera frames in visual mode.
- The local motor data plane is OS shared memory between the DimOS runtime demo
  code and the `benchmark_runtime` `WholeBodyAdapter`. SHM is not the remote
  sidecar protocol.

## Fake runtime smoke demo

The fake demo requires no Robosuite installation and validates protocol,
sidecar startup, local SHM, `WholeBodyAdapter`, and real `ControlCoordinator`
wiring.

```bash
PYTHONPATH="packages/dimos-runtime-protocol/src" \
  uv run python scripts/benchmarks/demo_fake_runtime_sidecar.py
```

Expected output includes `"ok": true` and artifacts under
`artifacts/benchmark/fake-runtime-smoke/`.

## Robosuite Panda Lift plumbing demo

Run this from an environment that can import Robosuite 1.5.x and this monorepo.
The DimOS process still does not import Robosuite; the sidecar subprocess owns
Robosuite environment construction and stepping.

```bash
uv run --with robosuite python scripts/benchmarks/demo_robosuite_panda_lift.py
```

To open the Robosuite viewer and watch the Panda receive a longer moving command
sequence:

```bash
uv run --with robosuite python scripts/benchmarks/demo_robosuite_panda_lift.py --visual
```

The visual mode enables Robosuite's on-screen renderer in the sidecar process,
runs at least 600 ticks, and sends an oscillating joint-position command through
the same `ControlCoordinator` → SHM → runtime sidecar path. It requires a local
display/GUI-capable environment. Visual mode uses Robosuite/MuJoCo free-camera
viewer mode, so the viewport can be changed interactively with the viewer mouse
controls while the scripted motion runs. The named `agentview` camera is still
used for protocol observation metadata.

To verify the camera observation path through DimOS streams and Rerun, run:

```bash
uv run --with robosuite python scripts/benchmarks/demo_robosuite_panda_lift.py --rerun
```

To verify the Robosuite camera payload path directly, without Rerun, run:

```bash
uv run --with robosuite python scripts/benchmarks/demo_robosuite_camera_payload_smoke.py --ticks 2
```

This starts the Robosuite sidecar, receives real Robosuite camera observation
frames, fetches each referenced `.npy` payload twice, decodes it with NumPy, and
asserts that the decoded array matches the sidecar-computed source hashes,
shape, dtype, and pixel summaries. Results are written to
`artifacts/benchmark/robosuite-camera-payload-smoke/camera_payload_smoke_summary.json`.
The same fetched/decoded images are also written as JPEGs under
`artifacts/benchmark/robosuite-camera-payload-smoke/images/`: `_raw.jpg` files
are the exact received arrays, and `_display_flipud.jpg` files apply the current
OpenGL display flip hypothesis for side-by-side inspection.

If you need to verify the visualization transport independently of Robosuite,
run the deterministic color-bar smoke:

```bash
uv run python scripts/benchmarks/demo_rerun_color_smoke.py
```

The smoke script writes `artifacts/benchmark/rerun-color-smoke/color_smoke_summary.json`
and publishes a fixed RGB image through the same `.npy` decode, DimOS `Image`
LCM encode/decode, and Rerun bridge path. The expected display is top red,
middle green, bottom blue, with no color changes over time. If this smoke test
looks wrong, debug the DimOS/Rerun visualization path before debugging
Robosuite camera payloads.

`--rerun` keeps the Robosuite viewer optional. The sidecar returns `agentview`
observation frames with raw NumPy `.npy` payload references, the script fetches
those payloads from `/payloads/{id}`, decodes them, vertically flips Robosuite's
default OpenGL-convention images for normal image-display semantics, and
publishes a private demo `Image(format=RGB)` / `CameraInfo` stream pair through
DimOS transports. The Rerun bridge uses an isolated gRPC port and an isolated LCM
port by default, so repeated demo runs do not mix with older recordings or other
DimOS camera topics. The Rerun server/viewer memory cap defaults to `128MB`, and
image logging is throttled to `10Hz` to avoid unbounded raw-image memory use.
Override with `--rerun-memory-limit`, `--rerun-grpc-port`, `--rerun-lcm-port`, or
`--rerun-max-hz` when needed. When `--rerun` is enabled, the demo also writes
sampled fetched camera payloads as JPEGs under
`artifacts/benchmark/robosuite-panda-lift/images/`; `_raw.jpg` files are exact
decoded payload arrays and `_display.jpg` files apply the current display
transform. Control the sampling with `--camera-jpeg-dump-every N` (`1` dumps
every tick, `<=0` disables). Use `--visual --rerun` if you want both the simulator
viewer and the DimOS/Rerun stream view at the same time.

`agentview` is a scene/task camera, not a wrist camera. To inspect a wrist-mounted
Panda camera instead, run:

```bash
uv run --with robosuite python scripts/benchmarks/demo_robosuite_panda_lift.py --rerun --camera-name robot0_eye_in_hand
```

Useful Robosuite camera names for this demo include `agentview`, `frontview`,
`sideview`, `birdview`, `robot0_robotview`, and `robot0_eye_in_hand`.

When `--visual --ticks N` is used, the script automatically raises the Robosuite
episode horizon to at least `N + 1`; otherwise long visual runs would hit the
default demo horizon and Robosuite would reject later `/step` calls after the env
terminates. You can still override this explicitly with `--horizon`.

The demo uses `dimos/benchmark/runtime/configs/robosuite_panda_lift.json`, starts
`dimos_robosuite_sidecar.server`, resolves the Panda motor surface, builds a
Robosuite `Lift` + `Panda` env with a `JOINT_POSITION` arm controller plus
`GRIP`, runs a scripted joint-position target through `ControlCoordinator`, and
writes artifacts under `artifacts/benchmark/robosuite-panda-lift/`.

If Robosuite is not installed, the script exits with an explicit sidecar health
failure and writes `robosuite_sidecar.log` with the import error.

## LIBERO-PRO registered-task runtime demo

LIBERO-PRO support follows the same runtime sidecar boundary as Robosuite, but the
sidecar must run in an environment that can import the LIBERO-PRO stack and has
prepared registered-suite assets. The DimOS process still does not import
LIBERO-PRO, Robosuite, or Torch; it starts `dimos_libero_pro_sidecar.server` in a
subprocess, resolves the described Panda motor surface, drives
`ControlCoordinator` through the SHM motor bridge, fetches `.npy` camera payloads,
and records sidecar-owned score output.

Prepared assets are validated by default and are never downloaded or rearranged by
sidecar startup or `/health`. If asset preparation is desired, use the explicit
bootstrap path instead of relying on implicit startup mutation:

```bash
uv run --with huggingface-hub python scripts/benchmarks/demo_libero_pro_runtime.py --prepare-assets
```

For already-prepared assets, run with a LIBERO-PRO-capable sidecar Python. If
that environment is separate from the main DimOS environment, point the demo at
it with `DIMOS_LIBERO_PRO_SIDECAR_PYTHON`:

```bash
LIBERO_CONFIG_PATH=/path/to/libero-config \
PYTHONPATH=/path/to/LIBERO-PRO \
DIMOS_LIBERO_PRO_SIDECAR_PYTHON=/path/to/libero-env/bin/python \
uv run python scripts/benchmarks/demo_libero_pro_runtime.py
```

To verify camera wiring live, run the LIBERO-PRO demo with the same DimOS Rerun
bridge path as the Robosuite demo:

```bash
LIBERO_CONFIG_PATH=/path/to/libero-config \
PYTHONPATH=/path/to/LIBERO-PRO \
DIMOS_LIBERO_PRO_SIDECAR_PYTHON=/path/to/libero-env/bin/python \
uv run python scripts/benchmarks/demo_libero_pro_runtime.py --rerun --camera-name agentview
```

For a live MuJoCo/Robosuite viewer window plus Rerun camera streaming, use
`--visual --rerun`:

```bash
LIBERO_CONFIG_PATH=/path/to/libero-config \
PYTHONPATH=/path/to/LIBERO-PRO \
DIMOS_LIBERO_PRO_SIDECAR_PYTHON=/path/to/libero-env/bin/python \
uv run python scripts/benchmarks/demo_libero_pro_runtime.py --visual --rerun --camera-name agentview
```

`--visual` defaults to a longer run and larger joint target amplitude than the
smoke config so the Panda arm motion is visible. The sidecar defaults
`MUJOCO_GL=glfw` for visual mode; override it in the environment if your display
stack needs a different MuJoCo backend.

`--rerun` starts a private Rerun bridge and LCM bus, publishes fetched sidecar
camera payloads as DimOS `Image` / `CameraInfo` streams, and writes
`rerun_summary.json` plus raw/display JPEG samples under the demo artifact
directory. `--rerun-grpc-port`, `--rerun-lcm-port`, `--rerun-max-hz`, and
`--camera-jpeg-dump-every` mirror the Robosuite demo flags. Rerun is the
headless-friendly verification path when an interactive viewer cannot be opened.

`LIBERO_CONFIG_PATH` must contain a `config.yaml` whose `bddl_files` and
`init_states` entries point at the prepared LIBERO-PRO assets. The extra
`PYTHONPATH` is only needed when running against a source checkout whose package
is not installed into the sidecar environment.

The default config is
`dimos/benchmark/runtime/configs/libero_pro_goal_task0.json`. It declares backend
`libero-pro` with common runtime fields plus backend-specific options for the
registered suite, task index, init-state index, controller, camera names, horizon,
and BDDL/init-state roots. Missing BDDL or init-state assets should fail before
episode reset with a clear sidecar validation error.

The scripted demo is a plumbing acceptance check, not an agent task-success
benchmark. It can pass when protocol stepping, motor command/state flow, camera
payload retrieval, score collection, artifact writing, and cleanup succeed even if
the scripted servo target does not solve the selected LIBERO-PRO task.
