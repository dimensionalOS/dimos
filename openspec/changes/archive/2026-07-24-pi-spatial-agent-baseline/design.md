## Context

The corpus-only change defines immutable map-question cases and keeps authoritative geometry, topology, answers, and review overrides private. This change adds an external Pi-agent baseline without moving map generation into the evaluated system. The unit under evaluation is the Pi session, pinned model, prompt mode, public case, custom tools, and answer protocol.

The runner must permit useful spatial analysis, including arbitrary Python, package installation, and general outbound network access, while preserving a hard boundary around the host, private oracle, and benchmark control plane. Both prompt modes carry the identical instruction not to use online information or services to solve the task; package installation is allowed. The modes differ only in whether visualization is instructed off or explicitly encouraged, and must not differ in case data, tool schemas, or answer feedback.

## Goals / Non-Goals

**Goals:**

- Run the external Pi SDK from a host control plane using either Codex OAuth with provider `openai-codex` or an owner-only dotenv `OPENAI_API_KEY` with provider `openai`, model `gpt-5.6-luna`, and a separate medium thinking budget/level.
- Give each case and prompt mode a fresh, rootless, disposable execution environment.
- Stage exactly the selected public corpus case and no oracle material.
- Expose only custom case-bound tools, including a typed immutable `submit_answer`.
- Return visualization results as generated image blocks rather than host paths or unrestricted files.
- Allow arbitrary Python, general outbound network access, and `uv` dependencies while auditing observable network-oriented activity.
- Preserve Pi transcripts, tool traces, and sandbox command audits for post-run review, and flag observable network-oriented commands/configuration heuristically.
- Keep scoring, correctness, and retained evidence private to the host.

**Non-Goals:**

- Changing corpus generation, labels, map artifacts, schemas, or source-data handling.
- Providing host Pi tools, DimOS RPC access, LCM access, private geometry, or oracle answers to the session.
- Treating heuristic audit records as proof that the agent did not use online information or services.
- Returning incremental correctness, confidence, hints, or score information to Pi.

## Architecture

```text
host control plane
  ├─ explicit host authentication + Pi SDK session (fresh per case/mode)
  ├─ public-case verifier/stager
  ├─ rootless Podman container lifecycle and audit policy
  ├─ custom tool broker
  └─ private scorer + ledger + exported evidence
          │
          ├─ read-only /input: exact public case staging
          ├─ writable /work: agent scripts, environments, and outputs
          └─ general outbound network: permitted, audited heuristically
                 │
                 ▼
          disposable per-case container
```

The host control plane owns scheduling, credentials, case selection, tool dispatch, container lifecycle, export, and scoring. It never registers host-side Pi tools. Tool calls are handled by a narrow broker that binds every request to the current case and either reads staged public input, runs an allowed operation in the container, or submits the typed prediction to the host.

### 1. Host Pi control plane and session isolation

The runner requires an explicit authentication mode on the host. `codex-oauth` reads stored Codex OAuth credentials and resolves provider `openai-codex`; `openai-api-key` reads only `OPENAI_API_KEY` from an owner-only dotenv file and resolves provider `openai`. Both modes pin model `gpt-5.6-luna` with a separate medium thinking budget/level. Before execution, the runner resolves the model in the provider catalog and fails closed unless the provider/API pairing is correct and the catalog entry advertises image and reasoning capability. Credentials remain in the host-side Pi adapter process and are not mounted into the container or workspace or emitted in transcripts or tool responses. The host creates a new Pi session for every case/mode pair; no conversation history, tool state, working directory, dependency environment, or generated image is reused across sessions.

The host records the authentication mode, resolved model identity, prompt mode, SDK/tool implementation digests, budgets, and run configuration in the run manifest. A failed or interrupted session is terminal for that case/mode and cannot be resumed with hidden prior context. Normal Pi agent completion without an accepted submission is not terminal: the adapter sends a neutral continuation prompt to the same session, retaining its conversation and working context, and does not create a new session or authentication context. It repeats this until durable acceptance or exhaustion of the global configured turn, tool, or wall-clock budget, or until session/protocol failure. `accepted=false` does not end the run.

### 2. Exact public-only case staging

Before launching the container, the stager resolves one immutable corpus instance and verifies its manifest references, schema version, relative paths, and artifact hashes. It emits a canonical, versioned `case.v1.json` projection containing only the selected Scene, Trajectory, Question, Snapshot, and Instance records, plus release identity, map-artifact identity, and staging-manifest/schema references. The projection is accompanied by the referenced map artifact and staging manifest/schema; it does not copy or expose shared full JSONL records.

The staging manifest records the exact corpus release, `instance_id`, selected variant, projection version, relative paths, and hashes. The `oracle/` root, authoritative geometry, topology, answers, review overrides, source paths, shared full JSONL records, and unrelated cases are absent. The staged directory is mounted read-only as `/input`; the writable `/work` directory is separate. The agent receives the case question through the Pi prompt/tool contract and can inspect only the staged public input made available by the custom tools or documented input files.

This is consumption of the existing corpus release, not a second generator or a transformed benchmark-data pipeline. Any Structured3D-derived staging remains subject to the corpus access controls and licensing decision.

### 3. Rootless disposable container

Each case/mode runs in a fresh rootless Podman container with a pinned runner image and explicit resource limits. Rootless Podman has been verified available on the evaluation host, but availability, rootless mode, image resolution, mount modes, and required isolation are validated at run time; any failed validation stops the run before agent execution (fail closed). The container has no host Docker/Podman socket, host filesystem mount, LCM endpoint, DimOS module reference, private scorer endpoint, authentication credential, or ambient benchmark directory. Only read-only `/input`, writable `/work`, the tool transport, and general outbound network access are available.

The host attempts to export the required evidence while the container is alive and verifies export completion and integrity when possible. Failed-run evidence is retained even when export fails, using host-side lifecycle, audit, transcript, and error records plus any partial export. Container cleanup is unconditional in a `finally` path: destruction is attempted after success, timeout, tool failure, malformed submission, agent error, or export failure. The exported workspace is evidence, not a way to restore execution state.

### 4. Custom case-bound tools and image responses

The tool profile is explicit and identical in both modes. It contains only operations needed to inspect the staged case, execute analysis in the container, retrieve generated visualizations, and submit the answer. Every operation is checked against the current `instance_id` and run identity; there is no generic host tool, module registry, shell-on-host tool, MCP passthrough, or private-data lookup.

The agent generates images through its container analysis in `/work`, then invokes the bounded `read_generated_image` operation with a relative workspace path. The host validates that the resolved path remains contained by `/work`, accepts only PNG MIME, enforces byte, dimension, pixel, and image-count limits, and returns the validated bytes as native image blocks. It never returns a host path or URL. This is not a fixed host rendering operation. `read_generated_image` is available in the same tool profile for both modes. The prompts use these mandatory mode instructions verbatim: visualization-forbidden mode says **“Visualization is forbidden. Do not call `read_generated_image`.”**; visualization-encouraged mode says **“Visualization is required for acceptance: after at most two sandbox attempts, generate a useful PNG under `/work` and call `read_generated_image` using a relative path. The image must be an existing regular non-symlink PNG that can be decoded successfully and must satisfy the applicable configured byte, width, height, and pixel limits. After a successful read, stop exploration: do not run more analysis commands or generate more images. Use the available evidence and call `submit_answer` exactly once with your best answer, even if uncertain. Do not consume remaining turn or tool budget seeking confidence.”** In visualization-encouraged mode, an answer is accepted only after at least one successful bounded read of an agent-generated `/work` image. A run without that successful read is failed and unscored. In visualization-forbidden mode, image reads are rejected, and any attempted read makes the run policy-noncompliant.

Arbitrary Python is allowed only inside the disposable container through the case-bound execution surface. Its filesystem view is limited to `/input` and `/work`, and general outbound network access is permitted. The host broker does not interpret or execute agent-generated Python.

### 5. Dependency installation and network audit

The container provides `uv` and a predeclared Python runtime. Package installation is allowed, as is general outbound network access; this is not a network-enforced benchmark and there is no package-index-only destination policy. The host preserves package/dependency events and sandbox command audit records, and heuristically flags observable network-oriented commands or configuration for review.

The audit is heuristic: it can identify observable network-oriented commands and configuration but cannot prove that the agent did not use online information or services, including through code or dependencies that do not leave detectable indicators. This limitation is explicit in run evidence.

The run bundle records requested dependency declarations, resolved lock/export information where available, package hashes, Pi transcript, tool trace, sandbox command audit, and flagged network-oriented observations. Dependencies are installed into the writable case workspace and are not shared between runs.

### 6. Paired prompt modes

When both prompt conditions select the same evaluation case, they use byte-identical public staging, tool schemas, tool implementations, resource limits, model configuration, and network availability. Both prompts contain this identical behavioral instruction: **Do not use online information or services to solve the task; package installation is allowed.** Only the visualization instruction changes:

- **Visualization forbidden:** **“Visualization is forbidden. Do not call `read_generated_image`.”**
- **Visualization encouraged:** **“Visualization is required for acceptance: after at most two sandbox attempts, generate a useful PNG under `/work` and call `read_generated_image` using a relative path. The image must be an existing regular non-symlink PNG that can be decoded successfully and must satisfy the applicable configured byte, width, height, and pixel limits. After a successful read, stop exploration: do not run more analysis commands or generate more images. Use the available evidence and call `submit_answer` exactly once with your best answer, even if uncertain. Do not consume remaining turn or tool budget seeking confidence.”**

The mode instruction does not add or remove data or tools, and the online-use/package-installation instruction is identical. The host records the prompt digest and mode in each run manifest and compares staging/tool-profile/network-availability digests before accepting a paired result. Tool traces and run outcomes retain the required image-read success or forbidden-mode rejection/attempt so compliance is reviewable. These records do not claim that inspecting an image proves its relevance to the answer or that the agent used no online information or services.

### 7. Immutable submission and private scoring

The broker exposes one typed `submit_answer` operation whose schema is derived from the case question's public answer contract. The first valid call becomes the immutable prediction for that run; subsequent calls, malformed values, late calls, and missing calls do not alter it. The response confirms only receipt or protocol failure and never indicates correctness, the oracle answer, a score, or an answer-dependent hint. The adapter's terminal protocol requires both an accepted submission and a submitted terminal reason before it can return `ok=true`. Every nonterminal completion and continuation prompt is recorded in the Pi prompt and tool transcript, and the final terminal reason is recorded in the evidence bundle. Budget exhaustion and session/protocol failure are terminal outcomes without accepted submission; they are not converted into success.

After the session ends, the host private scorer joins an eligible prediction to the private physical-question answer and any approved review override. It emits a versioned private score record and appends a ledger entry keyed by run, case, mode, corpus release, and scorer revision. A visualization-encouraged run lacking a successful bounded image read is failed and unscored; a visualization-forbidden run with any attempted image read is policy-noncompliant and ineligible for scoring. The ledger is outside the container and is not copied into `/input`, returned through tools, exposed in the agent transcript, or shown in operational scheduler status.

### 8. Exported evidence

Before destroying the container, the host attempts to export and hash an evidence bundle containing the public staging, writable workspace, container logs, Pi transcript, tool trace, sandbox command audit, dependency manifest/resolution, flagged network-oriented observations, run configuration, canonical prediction, and private score. The public staging and workspace are retained as separate named artifacts; the private score is access-controlled separately. Export failure blocks acceptance and prevents the run from being treated as complete, but it does not suppress failed-run evidence or defer cleanup.

### 9. Repo-owned neutral experiment scheduler

The scheduler replaces the `pi-baseline run-paired` smoke scaffold. Its executor contract is deliberately small and agent-neutral: an executor receives an immutable expanded case, a named condition, and an attempt context, and emits normalized lifecycle events plus one terminal outcome. The contract does not define universal tools, sessions, prompts, or agent abstractions. The Pi SDK runner is one executor implementation; its hardened single-mode core retains public staging, Pi Node session, rootless Podman, case-bound tools, evidence export, scoring, and unconditional cleanup.

An experiment is the Cartesian product of an immutable expanded case set and named conditions. Pi visualization modes are ordinary independent named conditions. Plan expansion, deterministic sampling/sharding, and canonical hashing happen before execution.

The coordinator writes authoritative state only to one host-local POSIX filesystem. Experiment creation stages plan and manifest data in a sibling temporary directory, fsyncs durable files and directory metadata, and atomically renames the completed directory; the manifest is written last as the commit marker. Each attempt has an immutable directory containing an immutable scheduler-manifest snapshot and digest, append-only event log, exported evidence, and exactly one terminal `outcome.v1.json` record. Terminal outcomes, not mutable summaries or lifecycle events, are authoritative for reconstruction. Job summaries are replaceable operational cache written to temporary siblings and atomically renamed; no database, service, multi-host coordination, or shared mutable evidence is used. One coordinator owns state and the UI and submits work to a bounded in-process thread pool. The manifest records a fixed worker count, default 10; there is no autoscaling and resume preserves that count. Executor-specific external boundaries remain the Pi Node subprocess and rootless Podman container; the scheduler adds no child Python worker layer.

The immutable manifest includes the expanded plan digest, executor, model, prompt, tool, corpus, runner-image, scorer, resource-limit, and worker-count fingerprints. Plan parsing recomputes the canonical digest and rejects duplicate identifiers, duplicate jobs, and invalid references. Every attempt captures the manifest snapshot and digest at creation time; private runtime bindings remain in the executor-specific boundary and are not scheduler abstractions. Any execution-affecting change forks the experiment and produces a new manifest; it cannot mutate an existing plan. `resume` schedules only pending or interrupted jobs. `retry` requires an explicit reason and creates a new attempt directory without overwriting prior evidence. The scheduler owns created and terminal lifecycle records; executors emit only progress and artifact events. Preflight validates the manifest, host filesystem, image/runtime, credentials, disk/evidence budget, and executor prerequisites before work begins. Graceful cancellation records interruption out of band and lets active executors clean up.

The noninteractive Rich surface is intentionally small: live run progress exposes operational health only, static status summarizes lifecycle state, and `--json` emits machine-readable operational records. Correctness, scores, oracle-derived data, and private comparisons are never shown live. Retention policy, retry filters, failure triage, disk/evidence budgets, and deterministic sampling/sharding are first-class scheduler capabilities. Textual, multi-host execution, autoscaling, external orchestration frameworks, and interactive control are deferred.

## Decisions and Alternatives

### Host control plane with credentials, not host tools

The Pi SDK and selected credential source remain on the host because credential handling, session creation, scheduling, and private scoring are control-plane responsibilities. Host tools are deliberately not registered with Pi: exposing them would create an accidental path to host files, credentials, transports, process control, or private benchmark state.

An alternative was to run Pi and its credentials inside the container. That would reduce host mediation but would require credential injection into an untrusted arbitrary-code environment and make session/export control less reliable. Another alternative was to expose existing DimOS MCP tools. That was rejected because the baseline requires a stable, case-bound public interface rather than ambient module discovery or RPC reachability.

### Arbitrary scripts in an external sandbox, not a DimOS worker

Arbitrary Python is retained as a baseline capability because spatial analysis may require custom parsing, computation, and `uv` packages. It runs only in the rootless disposable container. A normal DimOS worker, restricted globals dictionary, import denylist, or AST filter is not considered a security boundary and is therefore not used.

The alternative of forbidding scripts and providing only fixed high-level spatial tools would improve containment but would measure tool design more than Pi's ability to compose analysis. The alternative of host-side `exec` is rejected entirely.

### General network access with heuristic audit, not network enforcement

Package installation and general outbound network access are explicitly permitted. The paired prompt instruction is the behavioral control: do not use online information or services to solve the task; package installation is allowed. The host preserves transcript, tool trace, and sandbox command audit and flags observable network-oriented activity, but this is heuristic and cannot prove no online access. A network-enforced proxy was rejected because this baseline is no longer intended to enforce a package destination boundary.

### Bounded experiment concurrency

Experiments use the coordinator-owned bounded thread pool with the immutable manifest worker count (default 10). Every job has a separate attempt directory, container, workspace, audit identity, and ledger identity; resume preserves the worker count. Concurrency changes scheduling only and cannot change staging, tool behavior, model configuration, or scoring.

### Image blocks rather than paths

Returning generated images as image blocks gives Pi the intended visual observation while preventing host-path discovery and ambiguous out-of-band file access. Returning paths or URLs is rejected because paths can reveal mounts and URLs can become an uncontrolled data or network channel. Text-only visualization would not test the visualization-encouraged mode faithfully.

## Risks / Trade-offs

- **Credential or host-control compromise:** A leaked OAuth token or API key, or an overbroad host tool, could escape the benchmark boundary. Keep credentials host-side, require owner-only credential files, register no host tools, redact credentials from logs, use short-lived/least-privilege credentials where supported, and make the infrastructure gate fail closed.
- **Arbitrary scripts discover or exfiltrate private data:** Rootless isolation, minimal mounts, no host sockets, bounded resources, audit records, and public-only staging reduce this risk. General network access remains permitted, so audit cannot prove absence of online use; the host must validate that staging contains no oracle material.
- **Online information is used despite the prompt:** Preserve transcript, tool trace, and sandbox command audit; flag observable network-oriented commands/configuration; explicitly treat the audit as heuristic rather than proof of compliance.
- **Concurrent runs contaminate one another:** Use unique case/mode/run IDs, separate containers and writable workspaces, isolated audit identities/caches, ledger uniqueness constraints, and a fixed bounded worker count.
- **Visualization mode creates an unfair comparison:** Keep case bytes, tool schemas, tool implementation digests, budgets, and dependency policy identical; vary only the explicit off/on instruction and verify paired manifest equality.
- **Image generation leaks private context:** Render only from staged public data, prohibit oracle overlays and arbitrary host paths, bound image responses, and inspect image-tool traces in the review bundle.
- **Submission feedback changes behavior:** `submit_answer` confirms only receipt and becomes immutable. Scoring occurs after teardown on the host and is never sent to the session.
- **Untracked pilot artifacts or licensing violations:** The pilot docs state that scene-derived artifacts must remain gated until Structured3D permission is cleared. Do not commit or redistribute generated maps, questions, answers, manifests, or other derived files; record the access-control/legal decision before any release.
- **Container destruction loses diagnostic evidence:** Attempt export and hash the complete review bundle, retain host-side lifecycle/audit/scoring/error records and partial evidence even when export fails, and put destruction in unconditional `finally` cleanup.
- **Model or dependency drift undermines reproducibility:** Pin the model identifier, prompts, SDK/tool digests, runner image, dependency resolution, corpus hashes, and run configuration; mark runs incomplete when any required digest is unavailable.

## Migration Plan

This is a new external evaluation capability with no runtime migration and no corpus regeneration. Validate public-only staging, rootless isolation, network-audit collection, `/work`-contained `read_generated_image` validation and native image-block transport, immutable submission, private scoring, export, and teardown before using the runner on development or held-out cases.

The existing corpus release remains immutable. Any staging or runner defect is corrected in the runner and produces a new run configuration/tool digest; it does not rewrite public cases, oracle records, or generated pilot artifacts.

## Open Questions

- What refresh and rotation policy should deployments use for the pinned Pi `0.80.10` Codex OAuth and OpenAI API-key credential modes?
- Which package/dependency audit fields and network-oriented command/configuration patterns should the heuristic flag?
- What exact rootless Podman image, resource limits, and image digest are used on the evaluation host, beyond the verified runtime availability?
- What exact PNG byte, dimension, pixel, and image-count limits should `read_generated_image` enforce for `/work`-relative paths?
- What retention and access-control policy applies to private score ledgers and exported workspace/transcript bundles?
- What written Structured3D permission or legal decision is required before any scene-derived pilot artifact can leave gated storage?
