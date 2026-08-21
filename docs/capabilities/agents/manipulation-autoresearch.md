---
title: "Manipulation autoresearch with Evo"
---

# Manipulation autoresearch with Evo

DimOS exposes the task-independent manipulation Agent Harness as a research target
and the LIBERO-PRO panel as an inline Evo benchmark. Evo remains an unmodified,
external development tool: this repository does not fork, patch, vendor, subclass,
or import Evo. Evo owns experiment branches and search; DimOS owns native execution,
scoring, and artifacts.

## Install and initialize

Install the upstream CLI and its Codex plugin, recording the installed version with
the run artifacts:

```bash
uv tool install evo-hq-cli
evo install codex
evo --version
evo init
```

During `evo init` (the CLI equivalent of discovery), configure:

- target: `dimos/benchmark/evaluation/`
- benchmark command:

  ```bash
  uv run python -m dimos.benchmark.libero_pro.autoresearch \
    --panel dimos/benchmark/libero_pro/cases/autoresearch/dev-panel.json \
    --output "/tmp/dimos-evo-$EVO_EXPERIMENT_ID" --json
  ```

- metric: maximize `score`
- instrumentation: inline
- resource profile: one concurrent experiment when the simulator/GPU services are
  singleton

The command reads `EVO_RESULT_PATH`, `EVO_TRACES_DIR`, and `EVO_EXPERIMENT_ID`
directly. Its atomic result contains the macro score and one task dimension per panel
case. Each trace points back to policy, logs, video, Memory2, and native result files.
No Evo Python package is needed inside DimOS.

Select Evo's built-in `pareto_per_task` frontier in the dashboard, then start the
standard loop from Codex:

```text
$evo optimize subagents=1
```

The per-task frontier retains branches that improve different manipulation cases even
when their macro scores tie. Use one subagent initially because local LIBERO services
share GPU, ports, and caches; increase concurrency only after isolating those resources.

## Panel identity and interpretation

The development panel currently contains goal, spatial, object, and long-horizon
cases. The runner resolves it once and hashes the panel plus case specifications. The
hash is written to the normal DimOS result, Evo result, and every trace; changing a case
starts a different baseline.

These four binary outcomes are integration evidence for the autoresearch system. They
are not a statistically robust benchmark result and must not be reported as
publication-grade manipulation performance. Multi-seed measurement is a later layer.

Outside Evo, omit the Evo environment variables. The same command still writes
`panel-score.json` and prints normal JSON, which is useful for local contract checks.

## Reproducible evaluation container

The benchmark command automatically launches through the shared locked Evaluation
container. It builds the current DimOS candidate from digest-pinned Python and Node
bases plus `uv.lock` and `package-lock.json`, then executes the resulting local image
by immutable ID. Start the rootless service and verify the generic environment first:

```bash
systemctl --user enable --now podman.socket
nvidia-smi -L

uv run dimos eval check --workspace "$PWD/.container-eval/check"
```

Then use the same panel command configured in Evo:

```bash
uv run python -m dimos.benchmark.libero_pro.autoresearch \
  --panel dimos/benchmark/libero_pro/cases/autoresearch/dev-panel.json \
  --output "$(dirname "$EVO_RESULT_PATH")/dimos-panel" --json
```

The shared launcher uses host networking and the mounted rootless Podman socket to
launch simulator containers as siblings. It mounts only private output staging,
DimOS cache, and configured Evo result/trace directories at identical absolute paths.
Agent IPC uses a short `/runner-tmp` mount. Only named OpenAI/Hugging Face credentials
and Evo result variables are forwarded. Each invocation uses a fresh run-scoped NVIDIA
CDI spec and never reads or replaces system CDI configuration.
