# DimSim and Pi usage runbook

This runbook covers the current Go2 apartment workflow: start DimSim and the
DimOS navigation stack, inspect or drive the robot manually, attach Pi and the
code-policy observer, and run one automatic scored destination episode.

The supported stack is `unitree-go2-dimsim-external-pi-eval`. It includes the
Go2 spatial/navigation modules, observation recording, the persistent
code-policy kernel, MCP, and the skills that Pi can reach through `app` ([stack
definition](/dimos/benchmark/agent_eval/blueprint.py#L15-L51)).

## 1. Prepare the checkout

From the repository root:

```bash
uv sync --extra agents
node --version            # packages/pi-spatial-adapter requires Node >=22.19
cd packages/pi-spatial-adapter
npm install
npm run build
cd ../..
```

The first DimSim launch may need network access. DimOS downloads pinned Deno,
clones the pinned external DimSim revision, and stores both under the DimOS
state directory. Hidden mode also installs pinned Playwright Chromium when it is
missing ([Deno bootstrap](/dimos/simulation/dimsim/deno_utils.py#L29-L73),
[DimSim checkout](/dimos/simulation/dimsim/dimsim_process.py#L142-L207)).
Later launches reject a modified or wrong-revision checkout rather than running
an unverifiable simulator.

Pi also needs authentication. For API-key authentication, export the variable
used by the evaluation config:

```bash
export OPENAI_API_KEY=...
```

Subscription authentication instead uses an absolute path to Pi's OAuth JSON;
see the [evaluation configuration example](/docs/benchmark/dimsim-agent-evaluation.md#generate-and-select-the-task).

## 2. Choose a launch mode

Use one of these modes. Do not start two DimOS stacks on the same local LCM bus.

### Visible manual mode

Use this mode to watch the scene, inspect perception and navigation, talk to Pi,
or invoke skills yourself. For the first launch, keep the stack in the
foreground so startup failures remain visible:

```bash
export DIMSIM_HEADLESS=false
export DIMSIM_RENDER=cpu
dimos stop
dimos --rerun-open web run unitree-go2-dimsim-external-pi-eval
```

Do not skip `dimos stop`: this blueprint uses a fixed temporary SQLite
recording path, so overlapping evaluation stacks can corrupt the recorder's
WAL state. If there is no running stack, the command is harmless.

Leave that terminal running and use a second terminal for the remaining
commands. After the foreground flow works, add `--daemon` when you want the
stack recorded in the run registry and the launch command to return:

```bash
dimos --rerun-open web run unitree-go2-dimsim-external-pi-eval --daemon
dimos status
dimos mcp list-tools
```

In foreground mode, Ctrl+C stops the whole stack. `--daemon` eventually returns
the terminal after startup and records the run in the run registry. It may take
time to build modules before it detaches
([CLI lifecycle](/docs/usage/cli.md#dimos-run)).

Open exactly one DimSim scene tab:

```bash
xdg-open http://localhost:8090
xdg-open http://localhost:7779
```

- `http://localhost:8090` is the DimSim scene and browser-controlled viewpoint.
- `http://localhost:7779` is the passive Rerun view of the map, robot pose, and
  sensor streams.

DimSim's browser controls move its user player/viewpoint; they do not command
the Go2 used by the DimOS evaluation stack. Treat DimSim's own spawn/task
controls as a separate upstream workflow. Drive the evaluated robot through
DimOS, Pi, or an explicit MCP call as described below.

### Inspect and interact with the scene

Use these controls in the single DimSim tab:

- Click the scene to lock the pointer, then move the mouse to look around.
- Use W/A/S/D to move the browser-controlled player. `F` toggles fly mode; in
  fly mode, Space and Shift move up and down. `G` toggles collision-free ghost
  mode.
- Click **Camera: Agent/User** to switch between following the connected Go2
  and the user-controlled viewpoint.
- Aim at a nearby asset and press `E` (or left-click while the pointer is
  locked) to show its available interactions. Press `R` to cycle nearby
  targets, a number key to choose an action, and Escape to close the popup.
- `B` and the page's spawn/task controls belong to DimSim's own agent workflow;
  do not use them for a DimOS/Pi benchmark attempt.

These controls are useful for checking the room layout and manually toggling an
asset such as the television. They alter the live scene, so regenerate tasks or
restart/reset the stack before relying on benchmark state after manual
interaction.

With `DIMSIM_HEADLESS=false`, DimOS does not create its usual hidden Playwright
client. The first page you open at port 8090 becomes the authoritative simulator
client. Opening several copies can create competing simulator clients. Rerun is
separate and safe to view independently ([manual visual workflow](/docs/benchmark/dimsim-agent-evaluation.md#manual-visual-smoke-test),
[launcher behavior](/dimos/simulation/dimsim/dimsim_process.py#L54-L83)).

### Headless automatic mode

Use this mode for the automatic smoke runner:

```bash
unset DIMSIM_HEADLESS
export DIMSIM_RENDER=cpu
dimos stop
dimos --viewer none run unitree-go2-dimsim-external-pi-eval --daemon
dimos status
dimos mcp list-tools
```

The default `DIMSIM_HEADLESS=true` starts one hidden Playwright simulator client.
`--viewer none` disables only Rerun; it does not remove DimSim, navigation,
memory, MCP, code policy, or recording. Avoid opening another DimSim tab during
an automatic run.

CPU rendering is the predictable fallback. If the host has a working GPU path,
omit `DIMSIM_RENDER=cpu`; DimSim defaults to GPU outside CI ([render selection](/dimos/simulation/dimsim/dimsim_process.py#L54-L79)).

## 3. Check and operate the running stack

Use the lifecycle commands from any terminal:

```bash
dimos status             # current run ID, PID, blueprint, uptime, and log
dimos log -n 100         # recent human-readable logs
dimos log -f             # follow logs; Ctrl+C stops following only
dimos mcp status         # MCP and module status
dimos mcp modules        # module-to-skill inventory
dimos mcp list-tools     # callable tool schemas
dimos restart            # restart with the saved CLI arguments
dimos stop               # graceful stop, then forced stop after its deadline
dimos stop --force       # immediate SIGKILL; use only when graceful stop fails
```

Keep `DIMSIM_HEADLESS` and `DIMSIM_RENDER` exported if you expect `dimos
restart` to preserve the same visible/headless behavior: restart saves the CLI
arguments, while these two settings are environment variables. The general CLI
behavior is documented in [CLI Reference](/docs/usage/cli.md#commands).

### Direct operator control

The DimSim page is primarily a scene/player view. Drive the robot through DimOS.
A small relative-navigation check is:

```bash
dimos mcp call relative_move \
  --json-args '{"forward": 0.5, "left": 0.0, "degrees": 0.0}'
```

Other examples are a 90-degree right turn (`degrees: -90`) or a left offset
(`left: 0.5`). `relative_move` sets a navigation goal and waits for arrival or
failure; it is not raw velocity teleoperation ([skill semantics](/dimos/robot/unitree/unitree_skill_container.py#L210-L255)).
Cancel a navigation goal with:

```bash
dimos mcp call stop_navigation
```

For a low-level code-policy sanity check, call the same single tool exposed to
Pi and inspect the preloaded remote interface:

```bash
dimos mcp call python_exec --json-args '{"code":"print(app.skills)"}'
```

The persistent Python namespace preloads `app` for deployed DimOS RPCs and
skills and `memory` for recorded observations. Code is trusted, unsandboxed, and
persists across calls ([code-policy contract](/dimos/agents/code_policy.py#L186-L237)).
Do not paste untrusted Python into this interface.

## 4. Run Pi manually and observe its code

Start the observer before prompting Pi. Choose either terminal output:

```bash
dimos code-policy-watch
```

or the read-only notebook-style page:

```bash
dimos code-policy-watch --web
```

The web command binds to `127.0.0.1:8766` and opens after it reports `ready`.
For a headless host, use `--no-open`; use `--web-port 0` to choose a free
loopback port. Ctrl+C detaches the observer without stopping the kernel or robot.
Each invocation writes an append-only event log and a notebook beneath
`~/.local/state/dimos/code-policy-watch/`, or beneath `--output DIR` ([observer
guide](/docs/benchmark/dimsim-agent-evaluation.md#manual-visual-smoke-test), [CLI options](/dimos/cli/dimos.py#L621-L707)).

In another terminal, start Pi's restricted TUI:

```bash
cd packages/pi-spatial-adapter
PI_SPATIAL_MCP_ENDPOINT=http://127.0.0.1:9990/mcp npm run manual
```

Confirm `/tools` shows only `python_exec`, then enter a public task such as:

```text
Go to the bathtub and stop within 1 meter of its outer edge.
```

Pi may inspect `app` and `memory`, then call any deployed skill or RPC from
inside Python. It cannot call those host tools directly: the Pi CLI disables its
built-ins and other extensions and exposes only `python_exec` ([manual command](/packages/pi-spatial-adapter/package.json#L12),
[adapter check](/packages/pi-spatial-adapter/src/manual-code-policy-extension.ts#L55-L113)).

Watch three views while it works:

1. DimSim for physical motion.
2. Rerun for pose, map, and sensor updates.
3. `code-policy-watch` for the exact Python cells and outputs.

Exit Pi with Ctrl+D. Stop the observer with Ctrl+C. This manual flow produces no
scored result; use the automatic runner for pass/fail and retained evidence.

## 5. Run one automatic scored episode

The runner attaches to the already-running stack. It never starts or stops
DimOS or DimSim ([runner ownership](/docs/benchmark/dimsim-agent-evaluation.md#prerequisites)).

### Generate a release from the live scene

```bash
uv run python -m dimos.benchmark.dimsim \
  --output /tmp/dimsim-live-smoke \
  --host localhost \
  --port 8090
```

Do not add `--fixture`: fixture geometry intentionally cannot pass live
post-reset provenance checks. Select the generated destination task:

```bash
jq -c 'select(.category == "destination")' \
  /tmp/dimsim-live-smoke/public/tasks.jsonl
```

Copy its `task_id` into `/tmp/dimsim-eval.json`:

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

The current runner accepts only a generated `destination` task and the pinned
`gpt-5.6-luna`/`medium` combination ([selection constraints](/dimos/benchmark/agent_eval/config.py#L72-L85),
[destination validation](/dimos/benchmark/agent_eval/config.py#L125-L159)).

Run it:

```bash
uv run python -m dimos.benchmark.agent_eval run \
  --config /tmp/dimsim-eval.json
```

Exit code 0 means the evaluation infrastructure completed and wrote required
evidence. It does **not** mean the task passed: inspect `task_result` in the
outcome. A nonzero code means preflight, infrastructure, interruption, cleanup,
or artifact failure ([outcome model](/dimos/benchmark/agent_eval/models.py#L179-L198),
[runner exit mapping](/dimos/benchmark/agent_eval/runner.py#L309-L326)).

## 6. Inspect automatic-run artifacts

The command prints the fresh `attempt_<opaque-id>` path. Inspect that exact path
rather than an older glob match:

```bash
export ATTEMPT=/tmp/dimsim-agent-attempts/attempt_REPLACE_ME
jq . "$ATTEMPT/outcome.v1.json"
jq . "$ATTEMPT/dimsim-result.v1.json"
jq . "$ATTEMPT/attempt-manifest.v1.json"
less "$ATTEMPT/events.jsonl"
less "$ATTEMPT/code-policy-calls.jsonl"
less "$ATTEMPT/pi-adapter.stderr.log"
```

Read the files as follows:

- `outcome.v1.json`: normalized infrastructure status and independent task
  pass/fail result.
- `dimsim-result.v1.json`: unchanged native DimSim evaluator result.
- `attempt-manifest.v1.json`: task, contract, expected outcome, source revisions,
  session identities, and artifact digests.
- `events.jsonl`: ordered attempt lifecycle.
- `code-policy-calls.jsonl`: brokered `python_exec` calls and results.
- `pi-prompt/`: exact system and initial prompts.
- `pi-session/`: Pi's native JSONL session.

The complete layout and evidence boundary are documented in [Run and inspect](/docs/benchmark/dimsim-agent-evaluation.md#run-and-inspect).
The runner resets the code-policy session and simulator pose before each attempt,
starts the private evaluator, and uses only the native DimSim terminal result for
pass/fail ([runner sequence](/dimos/benchmark/agent_eval/runner.py#L127-L223)).

## 7. Common failure checks

- **Port 8090 shows 404 or never advances:** in visible mode, confirm
  `DIMSIM_HEADLESS=false`, wait for `dimos status`, inspect `dimos log -f`, and
  open the scene once. In headless mode, do not replace the hidden authoritative
  page with several manual tabs.
- **Another service uses port 8090:** choose a different `--dimsim-port` only if
  every client/config uses the matching port. Starting DimSim attempts to stop
  an existing process on its configured port ([port cleanup](/dimos/simulation/dimsim/dimsim_process.py#L123-L139)).
- **MCP is unavailable:** confirm the evaluation blueprint is running and check
  `dimos mcp status`; its default endpoint is `127.0.0.1:9990`.
- **Startup reports `sqlite3.OperationalError: disk I/O error`:** stop every
  existing evaluation stack before retrying. The blueprint currently uses one
  fixed WAL-mode database in `/tmp`; overlapping launches can unlink its main
  file while an older process still owns its sidecars. Remove the `.db`,
  `.db-wal`, and `.db-shm` files together only after confirming no stack is
  running.
- **Observer never becomes ready:** first call can lazily start the Jupyter
  kernel. Increase `--startup-timeout`, and check stack logs. The observer needs
  the `agents` extra.
- **Pi TUI rejects the stack:** run `npm run build`, then verify `dimos mcp
  list-tools` contains exactly one skill named `python_exec` (other host-only
  tools may also exist).
- **Automatic generation fails:** the live apartment must match the pinned
  profile, scene revision, spawn, asset IDs, and collision geometry. The
  generator fails closed by design ([generation compatibility](/docs/benchmark/dimsim-task-generation.md#private-oracle-contract)).
- **Runner says another attempt is active:** only one attempt may hold a given
  `output_root` lock at a time ([attempt lock](/dimos/benchmark/agent_eval/store.py#L44-L80)).
- **A cold run times out:** Pi subscription or model initialization can take
  more than five minutes, so keep `readiness_s` at 600 seconds for the first
  attempt.

When finished, stop the attached stack explicitly:

```bash
dimos stop
```

The automatic runner deliberately leaves DimOS and DimSim running so several
fresh attempts can reuse the same attached stack.
