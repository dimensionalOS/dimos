# Implementation Details: CI Level Performance Logging/Testing (Issue #1732)

This document provides a comprehensive breakdown of the implementation for Issue #1732, focusing on automated performance tracking for the `dimos` software stack.

## 1. Objective and Context

**Goal:** Prevent performance regressions by implementing automated CPU tracking in the CI pipeline for full system "blueprints."
**Requirements:**
1. Execute a full blueprint (specifically `unitree-go2` replay) in the CI environment.
2. Measure system and user CPU time during execution.
3. Store logs long-term to track performance trends over time.
4. Implement an alerting mechanism for significant performance degradation (>15%).

---

## 2. Exploration & Strategy Phase

1. **Identifying the Target:**
   - The issue specifically requested testing "full blueprints." The most stable and reproducible blueprint for CI is the `unitree-go2` replay, as it uses recorded data (`go2_sf_office`) instead of requiring live hardware or a heavy physics simulator like MuJoCo.
   - Found that `dimos --replay run unitree-go2` is the standard way to trigger this.

2. **Tool Selection:**
   - **Measurement:** Chose `psutil` (already a dependency) for high-precision CPU time tracking. It allows summing CPU times for a parent process and all its recursive children, which is necessary because `dimos` spawns multiple processes (for perception, control, and coordination).
   - **CI Tracking:** Selected `github-action-benchmark` (a popular GitHub Action) to handle the data persistence in a `gh-pages` branch and generate the trend graphs.

---

## 3. Tool Implementation

### A. Performance Measurement Script (`bin/measure-perf`)
This script acts as a "stopwatch" for CPU cycles.

*   **Process Group Management**: Uses `os.setsid()` to start the robot software in a new process group. This ensures that when the measurement is over, we can reliably kill the entire stack (including all sub-processes) using `os.killpg()`.
*   **The "Sampling Window" Logic**: 
    - Robot software often has a "startup spike" while loading models and initializing memory. 
    - The script waits **2 seconds** for the system to stabilize before taking the "Initial CPU" sample.
    - It then runs for the remainder of the requested duration before taking the "Final CPU" sample.
    - Resulting CPU Time = `(Final - Initial)`.
*   **Recursive CPU Summation**: Implemented `get_process_group_cpu()` to traverse the entire process tree using `psutil.Process.children(recursive=True)`, summing user and system time for every active component.

### B. Benchmark Formatter (`bin/perf-to-github-action.py`)
This utility bridges the gap between our raw measurement and the GitHub Action.

*   **Format Conversion**: Transforms our JSON output into the "Custom Generic" format required by the benchmark action: `[{"name": "...", "unit": "...", "value": ...}]`.
*   **Metrics Reported**: 
    - Total CPU Time (seconds)
    - User CPU Time (seconds)
    - System CPU Time (seconds)
    - CPU Usage Percentage (%)

---

## 4. CI Workflow Integration (`.github/workflows/ci.yml`)

I added a new job called `perf-audit` to the main CI pipeline.

*   **Environment**: Runs on `[self-hosted, Linux]` using the `ghcr.io/dimensionalos/ros-dev:dev` container, matching the standard test environment.
*   **Execution**:
    1. Checks out the code.
    2. Installs dependencies using `uv`.
    3. Runs `bin/measure-perf` for **60 seconds** on the `unitree-go2` replay blueprint.
    4. Passes `--viewer none` to `dimos` to ensure headless execution in CI.
*   **Benchmarking Action**:
    - **`auto-push: true`**: Automatically commits the new data point to the `gh-pages` branch.
    - **`alert-threshold: "115%"`**: This is the "regression detector." If the current run uses >15% more CPU than the average of previous runs, the CI fails.
    - **`gh-pages-branch: "gh-pages"`**: Stores the historical JSON data and the visualization dashboard.

---

## 5. Verification & Testing

### Manual Verification
I tested the measurement tool manually on the current environment:
```bash
./bin/measure-perf --duration 10 ./.venv/bin/dimos --replay --viewer none run unitree-go2
```

**Sample Output Captured:**
```json
{
  "command": "./.venv/bin/dimos --replay --viewer none run unitree-go2",
  "measurement_duration_s": 8.018,
  "user_cpu_s": 1.26,
  "system_cpu_s": 1.32,
  "total_cpu_s": 2.58,
  "cpu_percentage": 32.17
}
```
This confirmed that the script successfully identifies the process group and aggregates the load from all robot components.

### Structural Verification
Verified the `.github/workflows/ci.yml` file is syntactically correct and the `perf-audit` job is correctly positioned after `ci-complete` with appropriate permissions for writing to the repository.
