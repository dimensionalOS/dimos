# Dependencies

dimOS declares every Python requirement in [pyproject.toml](/pyproject.toml) and resolves it once into [uv.lock](/uv.lock). A robot stack needs more than the core install, so each built-in blueprint and module is assigned to one **runtime bundle**: a generous aggregate extra that covers all of its ordinary configurations (real hardware, replay, simulation, agents, viewer, relay). You name the blueprint; dimOS installs the bundle.

```sh skip
dimos deps unitree-go2            # which bundle, which backend, what is not covered
dimos prepare unitree-go2         # install it into the active virtualenv
dimos --replay run unitree-go2    # run as usual
```

To prepare and run in one command, use `dimos run --prepare unitree-go2`. It prepares
all named built-in blueprints and modules with automatic backend selection, then
starts them in a fresh Python process. Preparation finishes before daemon startup
when combined with `--daemon`; installation failures prevent startup. Use the
separate `dimos prepare` command to choose `--backend` or `--offline` explicitly.

`dimos prepare` installs Python packages only. It does not start modules, contact a robot, download model weights, build native modules, or install vendor SDKs.

## Bundles

| Bundle | Feature extras | Covers |
|---|---|---|
| `runtime-common` | base (agents, web, perception, visualization), mapping, misc, webrtc, apriltag | perception, navigation, mapping, memory, agents, web, lidar and camera modules, demos |
| `runtime-unitree` | runtime-common + unitree, sim, control | Go2 (WebRTC, replay, MuJoCo, DimSim), G1 high-level WebRTC and simulation, B1, Unitree teleop |
| `runtime-manipulation` | runtime-unitree + planning, learning | arms (xArm, Piper, A1Z, A750, OpenArm, OpenYAM), control coordinators, WebXR and hosted teleop rigs, imitation collection, R1 Pro |
| `runtime-unitree-dds` | runtime-manipulation + unitree-dds | G1 whole-body control over DDS, the Go2 DDS adapter (`unitree-go2-keyboard-teleop`) |
| `runtime-drone` | runtime-common + drone | MAVLink and DJI drones |
| `runtime-spot` | runtime-common + spot | Boston Dynamics Spot |

The bundles form a chain, so preparing several blueprints in any order yields one consistent environment: both installation paths keep the packages that are already present. A directory declares the bundle of the blueprints and modules below it in a `dependency_bundle.py` file holding `DEPENDENCY_BUNDLE = "runtime-unitree"` (for example `dimos/robot/unitree/`); a file that needs a different bundle declares the same literal itself, and the nearest declaration wins. The registry test collects the result into the generated [`dimos/deps/bundles.json`](/dimos/deps/bundles.json) that ships with the package. `dimos deps NAME` prints the assignment.

The inference backend is chosen separately. `--backend auto|cpu|cuda` (default `auto`) adds the `cpu` or `cuda` extra; `auto` picks `cuda` on Linux x86_64 with an NVIDIA driver for CUDA 12 or newer, otherwise `cpu`. On Linux x86_64 the locked PyTorch wheels come from the CUDA 12.8 index even for `cpu`, so `cpu` only selects the ONNX Runtime provider. Switching an environment from `cuda` to `cpu` is refused; use a fresh virtualenv.

## How preparation works

- In a source checkout (editable install), `dimos prepare` runs `uv sync --locked --inexact --no-default-groups --extra <bundle> --extra <backend>` against the checkout's `uv.lock`, targeting the interpreter that runs the command. A stale lockfile is an error, never a silent re-resolution. Working-tree changes stay visible through the editable install.
- From an installed release it runs `uv pip install -r dimos/deps/locks/pylock.<bundle>-<backend>.toml` for each bundle. These files are exported from the same `uv.lock` at build time (`python -m dimos.deps.export_locks`) and ship in the wheel and sdist with hashes, markers and resolved artifact URLs, so a release installs what the checkout tested. A missing file is a packaging error; nothing falls back to an unlocked install.
- With `--backend cuda`, the CPU `onnxruntime` that chromadb depends on and `onnxruntime-gpu` unpack into the same directory. After installing, `prepare` reinstalls `onnxruntime-gpu` (the same locked version) and then checks in a fresh interpreter that `CUDAExecutionProvider` is available and that `cv2.legacy` (opencv-contrib) survived.
- `--offline` passes uv's offline mode: installation succeeds from uv's cache or fails. A few packages are source distributions (`pyturbojpeg`, `iopath`, `antlr4-python3-runtime`), so the first preparation needs network.
- An ordinary virtualenv is required. With a system interpreter, `prepare` explains how to create one. A running `dimos run` in the same environment must be stopped first (`dimos stop`).
- Runtime flags (`--replay`, `--viewer none`, `--disable`) belong on `dimos run`; they never reduce a bundle.

## Support matrix

| Bundle | Linux x86_64 cpu | Linux x86_64 cuda | macOS arm64 cpu | Linux aarch64 cpu |
|---|---|---|---|---|
| `runtime-common` | yes | yes | yes | yes |
| `runtime-unitree` | yes | yes | yes | yes |
| `runtime-manipulation` | yes | yes | Python 3.12 only (Drake) | yes, without Drake |
| `runtime-unitree-dds` | needs the CycloneDDS library (below) | same | same | same |
| `runtime-drone` | yes | yes | yes | yes |
| `runtime-spot` | yes | yes | yes | yes |

Python 3.10, 3.11 and 3.12 are supported; the installer uses 3.12. `dimos prepare` refuses unsupported combinations before installing anything. The clean-install tests ([dimos/deps/test_prepare_integration.py](/dimos/deps/test_prepare_integration.py), marker `clean_install`) run in the install workflow on Linux x86_64 (cpu and cuda) and Linux aarch64 (cpu), once with the minimum uv version, and on macOS arm64 on demand.

Known limits, also printed by `dimos deps`:

- Drake ships no Linux aarch64 wheel and only a Python 3.12 wheel for Apple silicon (macOS 14). Drake planning backends are unavailable there; roboplan, the default, works.
- `a750-control` ships one wheel (Python 3.12, Linux x86_64): the A750 adapter needs that interpreter.
- `gtsam-extended` (PGO relocalization) has no Python 3.10 wheel except on macOS arm64.
- `cyclonedds` has wheels only for Python 3.10 on Linux x86_64 and macOS arm64. Elsewhere it builds against the CycloneDDS C library: set up [DDS](/docs/usage/transports/dds.md) first; `prepare` requires `CYCLONEDDS_HOME` to point at that installation and stops with the link otherwise.
- The CUDA backend is Linux x86_64 only. Jetson keeps its [documented setup](/docs/installation/index.md); CUDA on Jetson is not supported.
- `scene` (scene cooking) belongs to no bundle and has no Linux aarch64 wheel (`usd-core`): `uv sync --extra scene --inexact`.
- The `dds` transport extra is selected through the Python API, not by a blueprint: `uv sync --extra dds --inexact`.

## Prerequisites bundles do not install

- Native modules (Mid-360, Point-LIO, FAST-LIO2, RealSense, V4L2, voxel ray tracing, MLS and local planners, trajectory follower, go2_dds, dim_slam) are built with nix or cargo: see [native modules](/docs/usage/native_modules.md). The default trajectory follower controller needs the maturin extension `dimos_trajectory_follower`.
- System libraries: libturbojpeg, PortAudio, libsndfile, ffmpeg, GStreamer with PyGObject (`gstreamer-camera-module`, DJI video). [`scripts/install.sh`](/scripts/install.sh) installs them on Ubuntu and macOS.
- ROS 2 for R1 Pro, `unitree-go2-ros` and the `transport_ros` adapters; the ZED SDK for `zed-camera`; the Galaxea A1Z vendor packages; Habitat's own environment.
- Deno (downloaded on demand) for the cockpit relay and DimSim; uv at runtime for GraspGenX and LeRobot policies, which run in isolated environments.
- Model weights, replay data, credentials, API keys (OpenAI, Alibaba, Google Maps, TypeSafe) and an Ollama daemon.

## External blueprints

Blueprints from other packages (`dimos.blueprints` entry points, addressed as `<distribution>.<name>`) get their dependencies from their own distribution, which may depend on dimOS bundle extras such as `dimos[runtime-unitree]`. `dimos deps` names the owning distribution; `dimos prepare` rejects them.

## Contributor workflow

1. Add the package to the feature extra whose code imports it in `pyproject.toml`, with a comment when the need is not obvious, then run `uv lock`.
2. Make sure the runtime bundles covering the affected blueprints include that feature extra.
3. Regenerate the release artifacts with `python -m dimos.deps.export_locks`. CI verifies them with `--check` through [dimos/deps/test_locks_generation.py](/dimos/deps/test_locks_generation.py). The export always runs the uv version pinned in [dimos/deps/export_locks.py](/dimos/deps/export_locks.py) through `uv tool run`, because uv releases change the exported markers.
4. A new blueprint or module inherits the `DEPENDENCY_BUNDLE` of the nearest `dependency_bundle.py` above it; a file that needs a different bundle (or a new directory tree) declares `DEPENDENCY_BUNDLE = "runtime-..."` at module scope. One bundle per file: a file that mixes robots takes the larger one. `pytest dimos/robot/test_all_blueprints_generation.py` regenerates the registry and `dimos/deps/bundles.json` and fails on an entry without any declaration or with an unknown bundle.
5. Run the affected clean-install checks: `uv run pytest -m clean_install dimos/deps/test_prepare_integration.py` (`DIMOS_CLEAN_INSTALL_BUNDLES=runtime-unitree` narrows them).

Top-level imports do not define a dependency. Lazily imported packages, downloaded model code (`einops`, `sentencepiece`, `timm`), subprocess entry points and the adapters a coordinator selects by name all count.
