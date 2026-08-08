# DimOS Evaluation Suite

This directory contains the hardware-benchmark utility for DimOS. It evaluates whether a given blueprint will successfully run on target hardware, and grades DimOS network transports across degraded link conditions.

## What's Included

The suite is split into two primary tools:

1. **Resource Eval (`dimos.eval`)**: "Will blueprint X fit on this hardware?" 
   - Measures per-worker CPU, memory (PSS), GPU utilization, and startup time.
   - Evaluates a PASS/FAIL verdict based on the host's actual memory and CPU core count.
2. **Network Eval (`dimos.eval.network`)**: "Does a transport hold up over a degraded link?"
   - Replays the DimOS pubsub throughput benchmark under `tc netem` impairment profiles (perfect, wifi, lte, congested, satellite).
   - Grades loss% and latency against strict budgets.

Both tools export their data into a JSON artifact that embeds a `HardwareProfile`, allowing seamless diff analysis across separate hardware boards.

---

## Migration Prerequisites

If you are porting this Evaluation Suite into a fresh clone of `dimos-main`, ensure you copy the following folders:
* `dimos/eval/*` (The core benchmark utilities and testing logic)
* `docker/eval/*` (The containerization recipe for OS-independent execution)

---

## Environment Setup & Execution

The Network Evaluation tool relies heavily on Linux's Traffic Control (`tc netem`) system to dynamically emulate real-world network packet loss and latency. Because of this, **running the network suite is completely restricted to Linux environments**. macOS and Windows developers *must* use Docker.

Follow the instructions below based on your operating system:

### 1. 🐧 Native Linux (Ubuntu / Debian)

Running natively on Linux is the fastest and most direct approach, as you do not need Docker.

**Prerequisites:**
1. Install `uv` for python dependency resolution: `curl -LsSf https://astral.sh/uv/install.sh | sh`
2. Install `iproute2` which provides `tc netem`: `sudo apt-get update && sudo apt-get install iproute2 -y`

**Running the Evals:**
```bash
# Dependencies
uv sync 

# 1. Benchmark hardware
uv run python -m dimos.eval unitree-go2 --duration 20

# 2. Benchmark hardware network
sudo -E uv run python -m dimos.eval.network --iface lo
```

### 2. 🪟 Windows (via Docker Desktop + WSL2)

Windows natively lacks both `tc netem` and the Linux Ext4 POSIX layer necessary for some DimOS components. You will proxy the evaluations through a lightweight Linux Docker container.

**Prerequisites:**
- Docker Desktop installed and running (with the WSL2 engine backend enabled).
- NVIDIA Container Toolkit (Optional, but required if you want GPU utilization benchmarks).

**Running the Evals:**
```powershell
# 1. Build the Docker container
docker build -f docker/eval/Dockerfile -t dimos-eval .

# 2. Execute the evaluation suite transparently via Docker
docker run --rm -it --gpus all --cap-add=NET_ADMIN -v "${PWD}:/workspaces/dimos" -w /workspaces/dimos dimos-eval bash docker/eval/run_evals.sh
```

### 3. 🍎 macOS (via Docker Desktop / Colima)

Similar to Windows, macOS uses the `Darwin` Unix kernel, which means it completely lacks the `tc netem` traffic manipulation suite. You must route the network evaluation through Docker.

**Prerequisites:**
- Docker Desktop (or an alternative like Colima `colima start --cpu 4 --memory 8`).
- Apple Silicon (M1/M2/M3) users should natively compile outside of Docker if solely testing Resource Evals, but Network sweep forces Docker.

**Running the Evals:**
```bash
# 1. Build the Docker container 
docker build -f docker/eval/Dockerfile -t dimos-eval .

# 2. Execute the validation sweep 
docker run --rm -it --cap-add=NET_ADMIN -v "${PWD}:/workspaces/dimos" -w /workspaces/dimos dimos-eval bash docker/eval/run_evals.sh
```
