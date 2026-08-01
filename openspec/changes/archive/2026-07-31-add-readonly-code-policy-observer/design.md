## Context

`CodePolicyModule` owns a persistent IPython kernel created lazily with
`jupyter_client.KernelManager`. The external Pi agent invokes the single
model-facing `python_exec` skill; the module executes the submitted program
synchronously and retains a bounded `CodePolicyExecutionRecord` after the call
finishes. Humans can currently inspect DimOS logs and completed call artifacts,
but they cannot watch the actual cells and their output as the kernel executes
them.

Jupyter kernels broadcast `execute_input`, stream, display, result, error, and
status messages on IOPub. A generic Jupyter frontend can display those messages,
but its full connection file also provides shell and control endpoints and
therefore permits execution. Notebook and JupyterLab also introduce a
server-owned document/kernel lifecycle that this local debugging feature does
not need.

The observer must remain a separate local diagnostic process, must be unable to
write to the kernel, and must survive normal code-policy resets well enough to
produce useful partial evidence. The completed evaluation change remains
unchanged; this proposal does not add runner coordination.

## Goals / Non-Goals

**Goals:**

- Show the agent's code and outputs live in a notebook-style terminal or local
  browser view.
- Make the observer read-only by construction, not merely by operator
  convention.
- Capture the first agent cell after an explicit readiness handshake.
- Follow code-policy session and kernel-generation changes without controlling
  the kernel.
- Retain crash-recoverable native observation evidence and a portable notebook.
- Keep the implementation small and based on existing Jupyter protocols and
  DimOS RPC/CLI patterns.

**Non-Goals:**

- Expose an interactive Python prompt or namespace inspection.
- Add Jupyter Console, QtConsole, Notebook, JupyterLab, or any interactive
  browser frontend with kernel or observer controls.
- Add an evaluation-runner `--observe` flag or make observation required for a
  benchmark result.
- Add Rerun integration, remote access, multi-user authorization, or a
  leaderboard/fairness mechanism.
- Replace the authoritative `CodePolicyExecutionRecord` or change
  `python_exec` behavior for normal runs.

## Decisions

### Use an IOPub-only subscriber instead of a generic Jupyter client

The observer will construct a ZeroMQ `SUB` connection for the descriptor's
IOPub endpoint and use Jupyter's session deserialization only to authenticate
and decode received messages. It will not instantiate a
`BlockingKernelClient`, `KernelManager`, or any shell, control, stdin, or
heartbeat channel.

This preserves native Jupyter message ordering and rich-output semantics while
removing every observer interface that could submit source or control the
kernel. Reusing `jupyter console --existing` was rejected because the full
connection file and console prompt permit writes. Relaying translated events
from inside `CodePolicyModule` was rejected because it would add custom logic to
the execution path despite IOPub already providing the required stream.

### Publish a minimized host-only observer descriptor

`CodePolicyModule` will expose typed `@rpc` methods, never `@skill` methods, for
preparation, descriptor retrieval, and a fixed readiness probe. The immutable
descriptor contains:

- IOPub transport, IP, and port
- signature scheme and encoded verification key
- code-policy session identity
- code-policy Jupyter client session identity
- monotonic kernel generation

It omits the connection-file path and all non-IOPub ports. The generation
advances after every successful initial start and restart. Descriptor retrieval
will report explicit unavailable/stopped states rather than stale connection
details.

The verification key is still sensitive local material because Jupyter uses one
key across its channels. Minimizing the descriptor removes direct writable
addresses but is not a defense against an already malicious same-user local
process. Files and diagnostics therefore remain private and local.

### Verify initial attachment with a fixed module-owned probe

A ZeroMQ subscription can connect before its subscription has propagated, so
TCP connection alone is not enough to promise capture of the first short cell.
After connecting, the observer requests a fixed module-owned readiness probe.
`CodePolicyModule` emits that probe through its existing trusted kernel client
with silent, history-free execution and returns its Jupyter message identity.
The observer waits until the matching IOPub lifecycle arrives, suppresses it
from evidence, and only then prints `ready`.

The observer cannot choose probe source or submit arbitrary input. This narrow
handshake is preferable to a timing sleep or claiming readiness without
evidence. If the handshake fails, manual debugging does not begin.

### Keep the observer as a separate manual CLI process

`dimos code-policy-watch` discovers compatible running modules through the
normal DimOS host API. It selects the sole candidate or requires an explicit
run/module identity when discovery is ambiguous. It reserves an output
directory, performs the handshake, renders until Ctrl-C or module shutdown, and
then finalizes artifacts. Ctrl-C affects only the observer.

The default directory lives under DimOS's local state hierarchy with a unique
timestamp/opaque identity. `--output` selects an alternate parent but retains
exclusive non-overwriting reservation. The command prints the absolute path,
readiness, session identity, and sensitivity warning.

Runner-managed startup was rejected for the first version because it couples
debug evidence lifecycle to evaluation lifecycle before the manual UX has
proved useful.

### Filter and correlate native messages

The observer accepts supported IOPub messages only when their parent session
matches the code-policy Jupyter client identity. It groups messages by parent
message ID and execution count. Fixed probe traffic and unrelated-client
traffic are discarded.

`execute_input` opens a terminal/notebook cell; streams and rich output append
to it; error and idle messages close its live portion. Because the shell-channel
`execute_reply` is intentionally unavailable, the observer obtains completed
`CodePolicyExecutionRecord` snapshots through the existing host RPC and joins
them by `jupyter_message_id`. Those records remain authoritative for duration,
timeout interruption, namespace preservation, restart, and remote-work
warnings.

### Treat reconnect as observational and non-blocking

After initial readiness, the CLI periodically polls the minimized descriptor.
A changed generation or policy session closes the obsolete subscriber, appends
a lifecycle boundary, and attaches to the new endpoint. A fixed readiness probe
verifies the replacement subscription.

Observer loss after initial readiness never gates `python_exec`. Therefore a
very fast cell immediately following an unexpected kernel replacement can be
missed live. The observer marks the interval incomplete and backfills completed
source/status from authoritative records where possible rather than delaying
robot control.

### Retain an append-only event log and derive the notebook

The observer writes a strict versioned envelope for accepted messages to
`iopub-events.jsonl` before rendering them. Envelopes contain sequence,
timestamps, kernel/session identity, parent message identity, execution count,
message type, bounded text, MIME metadata, and artifact references. Transport
addresses, verification keys, signatures, and unrelated Jupyter headers are
never retained.

Binary and oversized supported MIME payloads are placed in bounded,
digest-addressed files. Per-execution limits align with code policy's configured
output limit; separate payload and recording-wide limits prevent unbounded
debug artifacts. Every truncation is itself an event.

`code-policy-transcript.ipynb` is an atomic materialized view of the event log
plus reconciled execution records. Code becomes code cells, supported outputs
become nbformat outputs, and reset, timeout, restart, truncation, and
incompleteness become generated markdown cells. The JSONL log remains the
recovery source if notebook finalization is interrupted.

### Keep rendering simple and append-only

The terminal renderer prints complete bounded source when an execution begins,
then streams text beneath it and prints concise terminal state when reconciled.
It does not redraw previous blocks or provide an input prompt. Rich payloads are
represented by type, digest, and local artifact path when the terminal cannot
display them.

This deliberately avoids a terminal UI framework or notebook document server.
Syntax coloring can use an already-declared lightweight repository dependency
when available, with deterministic plain-text fallback.

### Offer a credential-free loopback web projection

`dimos code-policy-watch --web` starts a small HTTP server on `127.0.0.1` and
projects the current recording through a session-status endpoint and a
server-sent event stream. The terminal renderer remains the default. The web
view starts before attachment, opens the browser only after the verified probe,
and shows status rather than duplicating cell output in the terminal.

The server exposes only `GET` routes and never receives the IOPub descriptor,
verification key, or port. Page reload replays events retained by the current
observer process, then resumes live delivery by event sequence. The browser
uses text nodes for source and text output, accepts only PNG and JPEG data
images, and ignores executable rich output such as HTML, SVG, and JavaScript.
This is a presentation layer over the observer event seam, not a Jupyter client
or notebook server.

## Risks / Trade-offs

- **The minimized descriptor still carries Jupyter's shared signing key** →
  Keep it in host-only RPC responses, never persist it, omit writable
  endpoints, restrict the feature to same-user local debugging, and document
  that it is not hostile-local-process isolation.
- **A replacement kernel can execute before polling reconnects** → Never block
  robot control after initial readiness; mark gaps and reconcile completed
  records.
- **The fixed readiness probe technically executes trusted kernel code** → Make
  the source constant, silent, history-free, inaccessible to the operator and
  model, and test that it cannot mutate the user namespace.
- **IOPub can produce unbounded or binary output** → Validate message types,
  enforce per-cell/payload/session limits, externalize bounded blobs, and record
  truncation explicitly.
- **Observer and authoritative records can disagree after a crash** → Preserve
  both sources, join only by stable Jupyter message identity, and mark
  unmatched intervals rather than inventing completion.
- **Polling adds small local RPC overhead** → Use a modest configurable
  interval and immutable generation comparison; do not poll the kernel itself.
- **Private artifacts can still contain sensitive robot/environment data** →
  Create user-only files, print a warning, and avoid lossy heuristic redaction.
- **Observed rich output can contain active browser content** → Serve strict
  security headers, render source and text with `textContent`, allow only
  bounded raster data images, and expose no mutating HTTP route.

## Migration Plan

1. Add typed observer models and host-only RPCs behind the existing
   `CodePolicyModule`; normal `python_exec` callers remain unchanged.
2. Add the standalone subscriber, evidence store, notebook materializer, and
   CLI command with focused fake-kernel and real-local-kernel tests.
3. Document the three-terminal manual workflow and artifact recovery.
4. Validate against one local DimSim/Pi debugging session before considering
   any runner integration.

Rollback removes the CLI and observer RPCs. Existing evaluation blueprints and
the model-facing `python_exec` schema require no data migration or compatibility
shim.

## Open Questions

None. Runner integration will be reconsidered only after the standalone manual
observer has diagnosed a real evaluation attempt.

## Current API Probe

Verified locally on 2026-07-30 against the repository lock:

- `jupyter_client` 8.8.0, IPykernel 7.2.0, and pyzmq 27.1.0 expose the required
  classic connection and message APIs.
- `KernelManager.get_connection_info()` returns the transport, loopback IP,
  IOPub port, signature scheme, and byte key needed by the observer, alongside
  writable ports that the minimized descriptor must discard.
- `client.session.session` is the parent-session identity used to distinguish
  code-policy traffic.
- A raw ZeroMQ `SUB` socket plus `Session.feed_identities()` and
  `Session.deserialize()` received `status`, `execute_input`, `stream`, and
  terminal `status` messages for another client's execution without opening a
  writable Jupyter channel.
- DimOS remote modules are discovered and addressed by unique class name through
  the Coordinator RPC service; one coordinator owns a class at most once.
- The root Typer application accepts a direct `code-policy-watch` command, and
  `STATE_DIR` provides the existing XDG-aware private recording root.
