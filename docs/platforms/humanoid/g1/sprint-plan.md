# Unitree G1 simulation sprint plan

This plan is for running DimOS/DimensionalOS G1 without physical hardware, then making it easy to move the same blueprint shape onto a real G1 later.

## Sprint 0 — discovery / runnable baseline

Goal: prove the repo, CLI, and G1 simulation entrypoints are discoverable without hardware.

- Use Python 3.12 and install `dimos[unitree,sim]` for simulation.
- Confirm the CLI exposes `unitree-g1-sim`, `unitree-g1-basic-sim`, `unitree-g1-agentic-sim`, and `unitree-g1-nav-sim`.
- Prefer `--viewer none` for headless CI/smoke checks; use Rerun/native viewer for local interactive runs.
- Default smoke command:

```bash
uv venv --python 3.12
source .venv/bin/activate
uv pip install 'dimos[unitree,sim]'
dimos --simulation --viewer none run unitree-g1-sim
```

## Sprint 1 — local sim loop

Goal: make the basic humanoid simulation usable for local development.

- Run `unitree-g1-sim` first; it is the smallest perceptive G1 simulation blueprint.
- If visualizing, use `dimos --simulation --viewer rerun run unitree-g1-sim`.
- Keep `dimos status`, `dimos log`, and `dimos stop` as the standard lifecycle commands for daemon runs.
- If Rerun or the viewer crashes, reduce visual load before changing robot logic.

## Sprint 2 — navigation / command-center loop

Goal: validate the navigation stack against the simulated G1.

- Run the native navigation sim blueprint:

```bash
dimos --simulation --viewer rerun run unitree-g1-nav-sim
```

- `unitree-g1-nav-sim` connects `UnityBridgeModule`, native nav stack, `MovementManager`, and visualization.
- The blueprint rate-limits heavy visualization (`vis_throttle=0.1`) because G1 visualization can overwhelm Rerun.
- Use this sprint for path-planning, waypoints, local planner tuning, and command-center/Rerun checks.

## Sprint 3 — agentic / cloud-ready harness

Goal: package a repeatable G1 sim run for agent/MCP work and optional GCP execution.

- Agentic local command:

```bash
dimos --simulation --viewer rerun run unitree-g1-agentic-sim --daemon
dimos status
dimos mcp list-tools
dimos log -n 100
```

- For GCP, use the requested project alias/ID explicitly on every command, e.g. `gcloud ... --project YOUR_PROJECT_ID`. If a short name is only a local alias, resolve it before submitting builds.
- A practical cloud target is a Docker image that runs the same smoke command headlessly (`--viewer none`) for CI; interactive graphics/GPU should stay local or on a GPU VM with display forwarding.
- Do not require real robot networking (`ROBOT_IP`, DDS/WebRTC, sport mode) for simulation sprints.

## Real-G1 boundary

Simulation sprints do **not** perform real G1 actions. Real hardware requires the G1 doc flow: SSH/network setup, sport mode safety positioning, robot-side DimOS install, and explicit human supervision.
