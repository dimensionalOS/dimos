---
title: "Frozen recording evaluation"
---

`dimos eval run` asks Pi one integer question about a frozen Memory2 recording.
The evaluator prepares the runtime map, exposes read-only `memory` through one
`python_exec` MCP tool, and checks the final `ANSWER: <integer>` line against a
private oracle. It does not start a robot, simulation, replay blueprint, or live
DimOS module.

## Setup

From a source checkout, install the lightweight Python runtime and build the Pi
extension:

```bash
uv sync --extra agents
npm ci --prefix packages/pi-code-policy-extension
npm run build --prefix packages/pi-code-policy-extension
```

The package pins Pi `0.80.10` and requires Node 22.19.0 or newer. Set the API key
before running a case:

```bash
export OPENAI_API_KEY=...
```

## Run the direct demo case

```bash
uv run dimos eval run \
  dimos/benchmark/short_horizon_qa/cases/demo_go2_hongkong_office-room-count-smoke/case.json \
  --output=/tmp/dimos-eval-smoke
```

The demo fixture uses the synthetic sentinel `0`, not a reviewed Hong Kong office
room count. A semantic failure can therefore mean the agent and runtime worked but
the response did not match that plumbing sentinel.

The supported options are deliberately small:

| Option | Default | Purpose |
| --- | --- | --- |
| `--agent.backend` | `pi` | Use the pinned Pi backend. |
| `--agent.model` | `gpt-5.6-luna` | Use the pinned model. |
| `--agent.thinking-level` | `medium` | Use the pinned thinking level. |
| `--agent.api-key-env` | `OPENAI_API_KEY` | Select the environment variable containing the API key. |
| `--output` | required | Publish this run to the exact directory. |
| `--json` | off | Print the compact result as JSON. |
| `--quiet` | off | Suppress status messages on stderr. |

The API key is passed only to the Pi subprocess. It is not placed in arguments,
results, or the Jupyter kernel environment.

## Output and exit status

`--output` must name an absent or empty directory. The evaluator builds the run in
a temporary sibling and atomically publishes it on completion. It never merges
with or overwrites a nonempty directory.

The directory contains only:

- `result.json`;
- `pi-transcript.jsonl`, when Pi wrote a native transcript;
- `stderr.log`, only when nonempty diagnostics are available.

Exit code `0` means evaluation completed, whether the semantic score passed or
failed. Exit code `1` means a caught runtime or agent infrastructure failure; the
published `result.json` includes `infra_error`. Exit code `2` means preflight
failed before a run started.

## Trust boundary

CodePolicy executes agent-authored Python in a persistent Jupyter kernel. It is
trusted and **unsandboxed**. The `memory` object is cutoff-limited and its SQLite
connections are truly read-only, but Python can still access other host files and
processes. Run only trusted evaluation agents, or place the whole command in an OS
sandbox or container.
