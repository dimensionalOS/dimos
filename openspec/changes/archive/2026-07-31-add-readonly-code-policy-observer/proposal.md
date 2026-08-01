## Why

Agent-authored `python_exec` programs currently run inside a persistent Jupyter
kernel without a live, human-readable view of the submitted code and its
outputs. Debugging local robot evaluations therefore depends on indirect logs
and post-run artifacts, which makes failures in perception, planning, RPC use,
and kernel recovery unnecessarily difficult to understand.

## What Changes

- Add a standalone `dimos code-policy-watch` command for manually observing one
  running `CodePolicyModule`.
- Add host-only observation RPCs that prepare the lazy code-policy kernel and
  return only the information required to subscribe to its Jupyter IOPub
  channel.
- Render agent code, streaming output, rich-output references, errors, and
  kernel/session transitions as append-only notebook-style terminal blocks or
  a loopback-only read-only browser page.
- Retain bounded native observation envelopes in a non-overwriting private
  recording and materialize a recoverable standard `.ipynb` transcript.
- Automatically reconnect across code-policy kernel generations without
  interrupting or controlling the kernel.
- Prove that the observer has no shell, control, stdin, execution, interrupt,
  restart, or shutdown path.
- Keep the first version manual and local. It does not add an evaluation-runner
  `--observe` option, an interactive Jupyter frontend, Rerun integration,
  remote access, or namespace inspection.

## Capabilities

### New Capabilities

- `readonly-code-policy-observation`: Manual, IOPub-only observation of
  code-policy executions with live terminal rendering and recoverable notebook
  evidence.

### Modified Capabilities

None.

## Impact

- Extends `CodePolicyModule` with non-skill, host-only observation metadata and
  kernel-preparation RPCs while preserving the single model-facing
  `python_exec` tool.
- Adds a DimOS CLI command and supporting Jupyter IOPub subscriber, terminal
  and loopback web renderers, append-only evidence writer, and notebook
  materializer.
- Uses the existing `jupyter_client` and `nbformat` libraries; any package used
  directly by production code must be declared explicitly rather than relied
  on as a transitive dependency.
- Adds focused lifecycle, security-boundary, artifact-recovery, and CLI tests
  plus manual debugging documentation.
- Does not change benchmark task outcomes, evaluator cleanup, robot control, or
  the completed `run-local-dimsim-pi-smoke-eval` change.
