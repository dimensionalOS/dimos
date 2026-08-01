## Scheduler work (new coordinator capability)

- [x] S1 Define the neutral executor contract: immutable expanded case + named condition + attempt context -> normalized lifecycle events and terminal outcome; keep Pi-specific tools and sessions inside the Pi executor.
- [x] S2 Define immutable experiment, expanded-plan, condition, job, attempt, event, summary, manifest, fork, and terminal-outcome schemas, including executor/model/prompt/tool/corpus/image/scorer/limits/worker-count fingerprints.
- [x] S3 Implement deterministic plan expansion, canonical hashing with parsed-plan self-integrity validation, sampling, sharding, fixed worker count (default 10), and fork-on-change validation.
- [x] S4 Implement one-coordinator, host-local POSIX filesystem state with transactional experiment creation, immutable manifest snapshots and terminal outcomes per attempt, append-only events, atomic operational summaries, retention policy, and no database/service/multi-host or child Python worker layer.
- [x] S5 Implement bounded in-process thread-pool scheduling, fail-closed preflight, graceful out-of-band cancellation, disk/evidence budgets, retry filters, stable failure triage, and resume of pending/interrupted jobs only.
- [x] S6 Implement explicit retries with required reasons, immutable attempt manifest snapshots, authoritative terminal outcome records, and new attempt identities that never overwrite prior evidence.
- [x] S7 Replace `pi-baseline run-paired` with `pi-baseline experiment create|run|resume|retry|status`; provide small noninteractive Rich live/static/`--json` operational views with no private correctness or scores.
- [x] S9 Defer Textual, multi-host coordination, autoscaling, external orchestration frameworks, and interactive live control; document these non-goals.

## Case executor work (retain/refactor existing hardened implementation; do not mark complete from current code)

## 1. Contracts and Runtime Configuration

- [x] 1.1 Define versioned run, case-staging, tool-profile, prompt-mode, dependency, prediction, score-ledger, and review-bundle record schemas.
- [x] 1.2 Define the pinned Pi SDK/model configuration, explicit Codex OAuth or owner-only OpenAI API-key dotenv source, redaction rules, resource limits, runner image reference, and required implementation digests.
- [x] 1.3 Define unique case/mode/run identifiers, fresh-session rules, terminal failure states, retention/access-control policy, and the public/private artifact boundary.
- [x] 1.4 Add package/runtime scaffolding that exposes an explicit host runner entry point and keeps the benchmark opt-in rather than part of the default test or run path.
- [x] 1.5 Verify configuration validation rejects missing credentials, unsupported prompt modes, missing digests, invalid case identifiers, and incomplete resource limits before execution.

## 2. Public Case Staging

- [x] 2.1 Implement a corpus-manifest resolver for one immutable case that validates release, schema, relative paths, stable IDs, and artifact hashes.
- [x] 2.2 Implement a canonical versioned `case.v1.json` projection containing only the selected Scene, Trajectory, Question, Snapshot, and Instance records, plus release identity, map artifact, and staging manifest/schema references; do not stage shared full JSONL records.
- [x] 2.3 Emit a staging manifest containing the corpus release, instance ID, selected variant, projection version, relative paths, and hashes without answer-bearing fields.
- [x] 2.4 Add staging rejection checks for oracle roots, authoritative geometry, topology, answers, review overrides, source paths, shared full JSONL records, unrelated cases, and private metadata.
- [x] 2.5 Verify a staged case byte/hash-matches its corpus manifest and is independently usable without the oracle root or corpus generator.

## 3. Rootless Container and Proxy Runtime

- [x] 3.1 Verify rootless Podman availability on the host and implement fresh per-case/mode rootless Podman container creation with a pinned image, explicit image-entrypoint override, explicit resource limits, read-only `/input`, writable isolated `/work`, and unique runtime identifiers; verify post-start running and exec readiness plus pre/post-exec liveness, and fail closed with terminal `container_runtime_failed` if any prerequisite, readiness, liveness, or isolation check fails.
- [x] 3.2 Ensure the container has no host filesystem or container-runtime socket, LCM endpoint, DimOS module reference, authentication credential, private scorer endpoint, or ambient benchmark directory.
- [x] 3.3 Permit general outbound network access and `uv`/Python dependency installation; do not enforce a package-index destination allowlist.
- [x] 3.4 Preserve Pi transcripts, tool traces, sandbox command audits, dependency declarations/resolutions, and observable network-oriented commands/configuration for post-run review; flag such observations heuristically and document that auditing cannot prove no online access.
- [x] 3.5 Verify the runtime permits local arbitrary Python, package installation, and general outbound network access while preserving staged-input read-only, host isolation, and cross-run workspace/cache isolation.

## 4. Pi Sessions, Custom Tools, and Prompt Modes

- [x] 4.1 Implement explicit host-side Codex OAuth or owner-only OpenAI API-key dotenv authentication and a fresh external Pi SDK session for every case/prompt-mode pair using model `gpt-5.6-luna`, the matching provider/API, and a separate medium thinking budget/level; resolve the provider catalog and fail closed unless the model advertises image and reasoning capability.
- [x] 4.2 Implement the explicit case-bound tool broker for public case inspection, container analysis, bounded `read_generated_image` access to agent-generated `/work` images, and typed answer submission.
- [x] 4.3 Bind every tool request to the current case and run identity, and reject generic host tools, MCP passthrough, module discovery, private-data lookup, host paths, URLs, and arbitrary host execution.
- [x] 4.4 Implement `read_generated_image` for relative workspace paths, validating `/work` containment, PNG MIME, byte/dimension/pixel/count limits, and returning native image blocks without host paths or URLs; record the tool trace.
- [x] 4.5 Implement visualization-forbidden and visualization-encouraged prompts with identical case bytes, tool schemas and implementations, model, thinking budget/level, image-capability catalog validation, container policy, network availability, and the identical instruction: "Do not use online information or services to solve the task; package installation is allowed."
- [x] 4.6 Record authentication mode, model identity, prompt digest, mode, tool-profile digest, SDK/tool digests, budgets, and paired-input digests in each run manifest.
- [x] 4.7 Verify the two modes differ only in the visualization instruction and expose identical public data, tool access, schemas, and limits.
- [x] 4.8 Enforce the mandatory visualization prompt wording, require a successful bounded `/work` image read before accepting an encouraged-mode answer, and fail/unscore runs that lack it.
- [x] 4.9 Reject image reads in forbidden mode, mark any attempted read policy-noncompliant, and retain mode-specific image-read outcomes in run evidence.
- [x] 4.10 Continue normally completed turns neutrally in the same session until durable acceptance or global turn/tool/wall-clock budget or session/protocol failure; do not create a new session or authentication context.
- [x] 4.11 Require accepted submission plus submitted terminal reason for `ok=true`, and record continuations and the final terminal reason in prompt/tool/evidence transcripts.

## 5. Typed Submission and Private Scoring

- [x] 5.1 Derive the `submit_answer` schema from the public question answer contract and implement validation for valid, malformed, late, missing, and repeated submissions.
- [x] 5.2 Make the first valid submission immutable and make all responses receipt/protocol-only without correctness, score, oracle, confidence, hint, or answer-dependent feedback.
- [x] 5.3 Implement host-side scoring after session termination by joining the prediction with the private authoritative answer and approved review override.
- [x] 5.4 Implement versioned private score records and append-only ledger entries keyed by run, case, mode, corpus release, and scorer revision.
- [x] 5.5 Keep private answers, scores, and ledger storage outside `/input`, the container, tool responses, public staging, and agent transcript.
- [x] 5.6 Verify repeated and malformed submissions cannot change the canonical prediction and that the runtime receives no correctness signal before teardown.

## 6. Exported Evidence Bundles

- [x] 6.1 Implement attempted export and hashing of separate public staging and writable workspace artifacts plus container logs, Pi transcript, tool trace, sandbox command audit, dependency manifest/resolution, flagged network-oriented observations, run configuration, prediction, and private score.
- [x] 6.2 Make export integrity a completion prerequisite, retain failed-run evidence including partial export and host-side error/lifecycle records when export fails, and do not suppress that evidence.
- [x] 6.3 Implement unconditional container destruction in `finally` after success, timeout, tool failure, malformed submission, agent error, or export failure paths.

## 7. Unit, Integration, and Security Tests

- [x] 7.1 Add unit tests for manifest resolution, exact public staging, hash/provenance records, private-material rejection, prompt configuration, tool binding, typed submission, and ledger keys.
- [x] 7.2 Add container integration tests proving rootless Podman execution with an explicit image-entrypoint override, post-start running/exec readiness, pre/post-exec liveness, terminal `container_runtime_failed` classification for container death/control-plane errors, read-only `/input`, writable `/work`, fresh per-run state, absent host mounts/sockets/credentials, and unconditional teardown.
- [x] 7.3 Add network-audit tests proving general outbound access and allowed package installation, preservation of command/network observations, heuristic flagging of observable network-oriented activity, and documentation that auditing cannot prove no online access.
- [x] 7.4 Add tool security tests proving no host tools or private lookups are exposed, `read_generated_image` rejects paths outside `/work` or invalid/oversized PNGs, returns bounded native image blocks without paths/URLs, and case/run mismatches are rejected.
- [x] 7.5 Add privacy tests proving oracle answers, private scores, credentials, and scorer endpoints do not appear in staging, container files, tool responses, transcripts, or exported public artifacts.
- [x] 7.6 Add end-to-end tests for a disposable case through Pi session setup, both prompt modes, typed submission, host scoring, bundle export, and container destruction.
- [x] 7.7 Verify paired manifests have identical case/tool/runtime/network-availability/dependency digests and identical online-use/package-installation instructions, with the only prompt difference being visualization forbidden versus encouraged.
- [x] 7.8 Add tests for encouraged-mode missing-read failure/unscored outcomes and forbidden-mode read rejection/policy-noncompliance.
- [x] 7.9 Add tests proving normal completion and `accepted=false` continue in the same session, budgets/failures terminate explicitly, and `ok=true` requires accepted submission plus terminal reason.
