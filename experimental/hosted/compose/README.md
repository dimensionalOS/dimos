# Hosted Docker Compose validation

This environment exercises real Host discovery, automatic placement, remote
fragment lifecycle, and cross-Host Zenoh streams. All services use the same
locally built DimOS image and connect through a dedicated Zenoh router.

## Scenarios

| Scenario | Placement | What it proves |
| --- | --- | --- |
| `basic` | source → `edge-a`; transform → `compute-a`; sink → controller | Exact-name placement, tag placement, local default, and two boundary streams |
| `replay` | SQLite replay → `replay-a`; transform → `compute-a`; sink → controller | Real DimOS replay scheduling and delivery across Hosts |
| `sim` | headless MuJoCo → `sim-a`; odom probe → `compute-a`; sink → controller | A real simulation module and typed odometry crossing Host boundaries |
| `visual` | moving MuJoCo → `sim-a`; odom probe → `compute-a`; Rerun → controller | The same real multi-Host sim, kept alive in an interactive browser 3D scene |

The source/simulator starts last, after the remote consumers and local result
sink report healthy. Every scenario checks the resolved module-to-Host mapping,
waits for an application-level result, and prints a JSON report. The finite
validation profiles then stop all accepted fragments; `visual` keeps them alive
until Ctrl-C.

## Run

From the repository root:

```bash
experimental/hosted/compose/run.sh basic
experimental/hosted/compose/run.sh replay
experimental/hosted/compose/run.sh sim
experimental/hosted/compose/run.sh visual
experimental/hosted/compose/run.sh all
```

For the directly visible scene, run `visual`, wait for `VISUAL:READY`, then open
the full URL printed by the runner. It includes the Rerun gRPC source; opening
only `http://localhost:9878` shows the generic welcome page. The blue body is
attached to the real `PoseStamped` odometry published by MuJoCo on `sim-a`; the
yellow arrow shows its forward direction. The visual simulator updates its real
MuJoCo root pose and velocity every simulation step, moving continuously around
a one-meter-radius circle in roughly eight seconds. Rerun runs on the controller
and receives that odometry over the same cross-Host Zenoh boundary used by the
validation. The profile stays alive so the browser can remain connected. Press
Ctrl-C in the Compose terminal to stop it and clean up the containers.

```text
http://localhost:9878/?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9877%2Fproxy
```

The first run builds a runtime-only image from the pinned dependencies in
`requirements.txt`; unrelated perception stacks and the native desktop viewer
are intentionally left out. The Rerun SDK is included for the browser scene.
Later runs reuse both the OS/package layers and uv's download cache. A
successful controller exits with code 0 and prints a report containing
`placement` and `result`; Compose uses that exit code for the scenario. The
runner removes scenario containers and the network while preserving the image
cache. `all` runs the three finite validation scenarios and intentionally omits
the long-running `visual` profile.

For direct Compose control:

```bash
docker compose -f experimental/hosted/compose/compose.yaml \
  --profile basic up --build --abort-on-container-exit \
  --exit-code-from verify-basic
```

The equivalent direct visual command is:

```bash
docker compose -f experimental/hosted/compose/compose.yaml \
  --profile visual up --build
```
