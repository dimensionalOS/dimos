# Geometry, simulation, eval and experimental consumers

The previous published HEAD `3a233c549807d0b7172551702aa9f356d38ed817` passed
main CI run 36750845515 and message-codegen run 36750850610. Python 3.10, 3.11,
3.12 and ARM all reached terminal success, as did documentation, Web, lint,
Rust and native C++/Rust builds. Self-hosted and large-data tests were skipped
by the dispatch workflow. The original 100 ms RPC assertion was unchanged;
its previous isolated timeout root cause is not established by this green run.

The subsequent geometry/simulation/eval/recall/detection regression passed
120 tests on Python 3.12.14; strict scoped mypy passed 19 production files.
This includes 67 original geometry numerical tests, explicit stamped frame
mismatch rejection, pursuit forward gating/angular clamp/arrival stop, track
pose publication, Habitat wire geometry, mock environment freshness checks,
image sharpness/brightness selection, thumbnails and source stamps, invalid
depth rejection and real reactive Reid alignment with an owned thread pool.
Only external tracker/inference/transport boundaries are mocked.

The Habitat server and connection use standalone generated CDR messages.
The installer now builds the same package for the separate native interpreter
instead of installing dimos-lcm; its Nix shell provides compiler and CMake.
The installer passed shell syntax checks but was not executed: no Habitat,
scene assets or native simulator was installed/started by this validation.

Python 3.9.25 was downloaded into ignored `build/python` (27.0 MiB), and an
isolated environment at `build/message-codegen/habitat-py39/venv` installed
numpy 1.26.4, Zenoh 1.10.1 and pinned pybind11 3.0.1 build dependencies.
The Habitat-specific generated package contains an 18-type closure, builds
against the existing Fast-CDR 2.4.0 local prefix, and installs without DimOS.
Its actual server functions encoded image, CameraInfo, PointCloud2, Odometry
and TFMessage under Python 3.9. Independent rosbags decoding under Python 3.12
confirmed all five payloads and exact source stamp (12 sec, 250000000 nanosec).
The offline check is `examples/message-codegen/demo_habitat_native.py`:

```bash
build/message-codegen/habitat-py39/venv/bin/python -I \
  examples/message-codegen/demo_habitat_native.py
```

The output is JSON plus five small CDR samples in the ignored evidence folder.
These checks do not prove the full Nix/conda installer, EGL rendering, a real
camera, robot control, large historical recordings, or human viewer acceptance.
Remaining consumers, fixture/dependency retirement and final cumulative demos
remain open; no whole-plan completion is claimed.
