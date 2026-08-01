# Local DimSim Pi agent evaluation

This smoke test runs one generated destination task through a real Pi SDK
session, the normal DimOS spatial/navigation stack, and an attached DimSim
instance. DimSim evaluates the generated private contract. Pi receives only the
public task text and one model-facing tool, `python_exec`.

This is a trusted local developer workflow. Agent-written Python is
unsandboxed, can use every deployed DimOS skill and RPC through `app`, and can
read the configured observation store through `memory`. Oracle separation is
logical, not a security boundary against a hostile local process. The workflow
does not provide process isolation, anti-cheating, fairness, reproducibility
across machines, or leaderboard-comparable scores.

## Prerequisites

Install the agent dependencies and build the pinned Pi adapter:

```bash
uv sync --extra agents
cd packages/pi-spatial-adapter
npm install
npm run build
cd ../..
```

Start the evaluation blueprint manually and leave it running. Disabling the
optional viewer reduces unrelated rendering load without removing the robot,
navigation, memory, MCP, code-policy, recorder, or DimSim modules:

```bash
dimos --viewer none run unitree-go2-dimsim-external-pi-eval --daemon
dimos status
dimos mcp list-tools
```

The runner attaches to this stack. It never launches or stops DimOS or DimSim.
Normal completion, failure, and interruption release only the runner's Pi,
evaluation, and RPC-client resources.

## Generate and select the task

Generate the release from the running, compatible apartment. Do not use
`--fixture` for a real attempt: that fixture has synthetic contract geometry
and intentionally fails the live post-reset provenance check.

```bash
uv run python -m dimos.benchmark.dimsim \
  --output /tmp/dimsim-live-smoke \
  --host localhost \
  --port 8090
```

Inspect `public/tasks.jsonl` and select the record whose `category` is
`destination`:

```bash
jq -c 'select(.category == "destination")' \
  /tmp/dimsim-live-smoke/public/tasks.jsonl
```

Copy its opaque `task_id` into a JSON configuration:

```json
{
  "release_root": "/tmp/dimsim-live-smoke",
  "task_id": "dimsim_task_REPLACE_WITH_THE_64_HEX_DIGEST",
  "output_root": "/tmp/dimsim-agent-attempts",
  "mcp_endpoint": "http://127.0.0.1:9990/mcp",
  "pi": {
    "model": "gpt-5.6-luna",
    "thinking_level": "medium",
    "auth_mode": "environment",
    "credential_env": "OPENAI_API_KEY"
  },
  "timeouts": {
    "readiness_s": 600.0,
    "mcp_call_s": 120.0,
    "reset_s": 300.0,
    "evaluation_start_s": 10.0,
    "cancellation_s": 5.0
  },
  "episode_timeout_s": 180.0,
  "dimsim": {
    "endpoint": "http://127.0.0.1:8090",
    "expected_scene_id": "dimsim-apartment"
  }
}
```

`readiness_s` also bounds creation of the fresh Pi session. A cold local
subscription/model cache can take more than five minutes to initialize; warm
starts are normally much faster.

For Pi subscription authentication, replace the three authentication fields
with:

```json
{
  "auth_mode": "subscription",
  "credential_path": "/absolute/path/to/the/pi/oauth.json"
}
```

The resolved manifest records only the authentication mode and a
domain-separated credential-binding digest. Credential contents and paths are
not sent in protocol frames, prompts, session records, call records, or
diagnostics.

## Run and inspect

For the current bathtub destination smoke:

```bash
uv run python -m dimos.benchmark.agent_eval run --config /tmp/dimsim-eval.json
```

Exit code `0` means the evaluation infrastructure and required evidence
completed, whether the task passed or failed. A nonzero exit means preflight,
infrastructure, interruption, cleanup, or artifact failure.

Each fresh attempt directory contains:

```text
attempt_<opaque-id>/
├── attempt-manifest.v1.json
├── task.v1.json
├── events.jsonl
├── mcp-inventory.v1.json
├── code-policy-calls.jsonl
├── dimsim-result.v1.json
├── outcome.v1.json
├── pi-adapter.stderr.log
├── pi-prompt/
│   ├── system.txt
│   └── initial.txt
└── pi-session/
    └── <native Pi session>.jsonl
```

`dimsim-result.v1.json` is the unchanged native evaluator result.
`outcome.v1.json` separately reports infrastructure completion and task
pass/fail. Pi's final text never determines success. The manifest references
the selected private contract and expected outcome by identity and digest; it
does not copy the complete scene oracle.

To inspect a run:

```bash
jq . /tmp/dimsim-agent-attempts/attempt_*/outcome.v1.json
jq . /tmp/dimsim-agent-attempts/attempt_*/attempt-manifest.v1.json
less /tmp/dimsim-agent-attempts/attempt_*/events.jsonl
less /tmp/dimsim-agent-attempts/attempt_*/code-policy-calls.jsonl
```

The attached DimOS/DimSim stack remains running after the attempt. Stop it
explicitly only when finished:

```bash
dimos stop
```

## Manual visual smoke test

Use this flow to watch the simulator and talk to Pi interactively. It verifies
that the simulator, viewer, MCP server, code-policy interface, and robot control
work together. It is not a scored benchmark attempt and does not create the
artifact set described above.

First stop any unrelated DimOS stack, then launch the evaluation blueprint with
the browser dashboard enabled:

```bash
dimos stop
DIMSIM_HEADLESS=false DIMSIM_RENDER=cpu dimos --rerun-open web \
  run unitree-go2-dimsim-external-pi-eval --daemon
dimos status
dimos mcp list-tools
```

Open these pages if the browser does not open automatically:

- DimSim scene and free camera: http://localhost:8090
- DimOS/Rerun map, sensor streams, and controls: http://localhost:7779

On a Linux desktop:

```bash
xdg-open http://localhost:8090
xdg-open http://localhost:7779
```

`DIMSIM_HEADLESS=false` prevents DimOS from creating its usual hidden
Playwright page. Open the scene URL once: that browser tab becomes the
authoritative simulator client. Do not open multiple copies of the DimSim page.
The Rerun page is a separate passive DimOS visualization.

In a second terminal, attach the read-only code-policy observer before asking
Pi to act:

```bash
dimos code-policy-watch
```

For a notebook-style browser view, run:

```bash
dimos code-policy-watch --web
```

The web view binds only to `127.0.0.1:8766` and opens after the readiness
probe succeeds. Use `--no-open` on a headless machine, or `--web-port 0` to
select an available loopback port. The command prints the exact URL.

Wait for `ready`. The command prepares the lazy policy kernel, attaches only
to its Jupyter IOPub stream, verifies that subscription with a fixed silent
probe, and then prints each subsequent policy cell and its output. It cannot
execute code, inspect the live namespace, answer stdin, interrupt, restart, or
shut down the kernel. Ctrl+C detaches only the observer.

The browser receives the same bounded, credential-free observation events as
the terminal. It replays the current recording when the page reloads and then
streams new cells. It renders text and raster images, but never executes
observed HTML, SVG, or JavaScript. The page has no input or control endpoint.

Each invocation reserves a new private recording beneath
`~/.local/state/dimos/code-policy-watch/` (or beneath the parent supplied with
`--output`). The recording contains:

```text
observation-<UTC timestamp>/
├── iopub-events.jsonl
├── code-policy-transcript.ipynb
└── blobs/
```

The JSONL file is append-only and the notebook is materialized atomically on
detach. If the observer is interrupted during finalization, rerunning notebook
materialization from the valid JSONL prefix produces an explicitly incomplete
notebook. Content is bounded but deliberately not heuristically redacted, so
it can contain agent code, environment values, perception, map, and robot
data.

In a third terminal, start Pi's native TUI:

```bash
cd packages/pi-spatial-adapter
PI_SPATIAL_MCP_ENDPOINT=http://127.0.0.1:9990/mcp npm run manual
```

The manual extension refuses to start unless the attached MCP inventory
contains exactly one `python_exec` skill. Pi's CLI also disables built-in
tools, discovered extensions, skills, prompt templates, and context files, then
allowlists only `python_exec`. Other deployed DimOS skills and RPCs remain
reachable indirectly through the preloaded `app` object inside that Python
session.

Paste a generated public task into the TUI, for example:

```text
Go to the bathtub and stop within 1 meter of its outer edge.
```

Watch DimSim for physical motion and Rerun for the robot pose, map, and sensor
updates. Watch the observer terminal for the exact code-policy cells. In the Pi
TUI, `/tools` should show only `python_exec`. Exit Pi with Ctrl+D and detach
the observer with Ctrl+C. When finished with the simulator:

```bash
dimos stop
```

For a native pass/fail result and retained evidence, run the scored command in
the preceding section instead of relying on visual judgment. The observer is a
standalone local debugging aid: there is intentionally no runner `--observe`
flag, interactive Jupyter frontend or notebook attachment, remote observer
mode, or effect on benchmark scoring and evaluator cleanup. The read-only web
view described above is an observer, not an attached interactive frontend.
