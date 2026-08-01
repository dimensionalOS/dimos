## 1. Current-API and Read-Only Gates

- [x] 1.1 Probe the pinned `jupyter_client`, IPykernel, ZeroMQ, DimOS RPC discovery, CLI registration, and local-state APIs needed for an IOPub-only observer, and record any version-specific constraints in the design notes
- [x] 1.2 Add failing contract tests for the typed observer descriptor, including kernel generation, policy/client identities, IOPub verification fields, unavailable/stopped states, and omission of the connection-file path and every non-IOPub endpoint
- [x] 1.3 Add a failing security-boundary test proving the observer constructs only a ZeroMQ `SUB` socket and exposes no shell, control, stdin, heartbeat, execution, interrupt, restart, shutdown, or arbitrary-probe path
- [x] 1.4 Add failing MCP inventory tests proving all observation operations are `@rpc` only and `python_exec` remains the sole code-policy model-facing skill
- [x] 1.5 Audit direct production imports and declare `nbformat`, ZeroMQ, or other required packages explicitly in the appropriate project extra without adding Jupyter Console, QtConsole, Notebook, JupyterLab, or Jupyter Server

## 2. Code-Policy Observation RPCs

- [x] 2.1 Add strict immutable models for observer availability, minimized IOPub descriptors, kernel generation, and fixed readiness-probe receipts
- [x] 2.2 Track and test monotonic kernel generations across initial creation, successful timeout restart, shutdown/recreation, explicit session reset, failed restart, and module stop
- [x] 2.3 Implement a host-only preparation/descriptor RPC that lazily creates and bootstraps the policy kernel without agent history and returns only the current minimized descriptor
- [x] 2.4 Implement the constant, silent, history-free readiness probe RPC and test that its source cannot be supplied by the caller, it does not mutate the persistent user namespace, and its message identity can be observed and suppressed
- [x] 2.5 Make descriptor polling return explicit unavailable, replaced, and stopped states without leaking stale credentials or changing normal `python_exec` execution

## 3. IOPub Subscriber and Lifecycle

- [x] 3.1 Implement the IOPub-only subscriber using a ZeroMQ `SUB` socket plus Jupyter message verification/deserialization, with strict supported-message validation and no generic kernel client
- [x] 3.2 Filter messages by the code-policy Jupyter client identity, suppress readiness-probe and unrelated-client traffic, and group accepted messages by parent message ID and execution count
- [x] 3.3 Implement initial connect/probe readiness and test that the first subsequent agent execution is captured without timing sleeps
- [x] 3.4 Implement descriptor polling, generation/session boundary detection, subscriber replacement, and gap marking while ensuring reconnect failure never gates policy execution after initial readiness
- [x] 3.5 Reconcile completed live cells with `CodePolicyExecutionRecord` snapshots by Jupyter message identity for duration, timeout, namespace, restart, and remote-work status
- [x] 3.6 Test successful cells, streaming stdout/stderr, results, display data, Python errors, unrelated traffic, malformed/signature-invalid messages, explicit reset, timeout restart, observer loss, missed-generation backfill, module shutdown, and Ctrl-C detach

## 4. Recoverable Observation Artifacts

- [x] 4.1 Add exclusive non-overwriting recording reservation under the DimOS local-state hierarchy and `--output` parent, with user-only directory and file permissions
- [x] 4.2 Define and implement a strict versioned `iopub-events.jsonl` envelope containing sequence, time, session/generation, parent identity, execution count, supported content, truncation, and artifact references while excluding transport credentials and unrelated headers
- [x] 4.3 Enforce configurable per-execution, MIME-payload, and recording-wide limits; retain supported binary/oversized MIME content as bounded digest-addressed blobs and record every truncation explicitly
- [x] 4.4 Materialize `code-policy-transcript.ipynb` atomically with ordered code/output cells and generated markdown cells for resets, restarts, timeouts, truncation, gaps, and incomplete finalization
- [x] 4.5 Implement idempotent notebook recovery from a partial event log and test completed, interrupted, malformed-tail, missing-blob, truncated, unmatched-record, and pre-existing-output cases

## 5. Manual CLI and Terminal UX

- [x] 5.1 Add `dimos code-policy-watch` with startup deadline, polling interval, output-parent, and explicit run/module selection options using existing DimOS CLI and host RPC discovery patterns
- [x] 5.2 Implement fail-closed single-module selection with actionable candidate reporting and test zero, one, ambiguous, explicitly selected, incompatible, and stopped module cases
- [x] 5.3 Implement append-only notebook-style terminal rendering for source, streams, errors, rich-output references, authoritative completion, session boundaries, reconnect gaps, and truncation without an input prompt or historical redraw
- [x] 5.4 Print the absolute recording path, sensitivity warning, selected identities, attachment progress, and `ready` only after the verified probe handshake
- [x] 5.5 Implement signal handling so Ctrl-C and observer failures finalize or preserve recoverable artifacts, close only observer resources, and leave the code-policy kernel alive and responsive

## 6. Verification and Manual Debugging

- [x] 6.1 Add a real local-kernel integration test proving source and incremental output from another client appear through IOPub while the observer has no writable Jupyter channel
- [x] 6.2 Add an end-to-end fake-host CLI test covering preparation, first-cell capture, generation reconnect, execution-record reconciliation, Ctrl-C, JSONL retention, notebook validity, and non-interference
- [x] 6.3 Run focused code-policy/observer tests, CLI tests, formatting, lint, type checking, dependency/lock checks, and the relevant fast repository test subset
- [x] 6.4 Document the three-terminal DimSim, `code-policy-watch`, and manual Pi workflow; artifact contents and recovery; local sensitivity boundary; and all excluded interactive Jupyter and runner behavior
- [x] 6.5 Run one real local DimSim/Pi debugging session, verify that the first and subsequent agent cells render live, inspect the recovered notebook and generation markers, and record actionable UX or lifecycle gaps without adding runner integration

## 7. Optional Read-Only Web View

- [x] 7.1 Refactor terminal rendering behind an observer-view seam without changing the IOPub-only kernel boundary
- [x] 7.2 Add a loopback-only FastAPI server with credential-free session state, replayable server-sent events, strict browser security headers, and no mutating routes
- [x] 7.3 Add a packaged notebook-style page that groups cells, follows live output, handles `clear_output`, and renders observed content without executing HTML, SVG, or JavaScript
- [x] 7.4 Add `--web`, `--web-port`, and `--open/--no-open` CLI options while keeping terminal rendering as the default
- [x] 7.5 Add focused hub, HTTP security, replay, browser-opening, CLI, reconnect, and notebook-clear tests; document the local browser workflow
