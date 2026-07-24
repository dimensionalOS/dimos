## 1. Pinned Pi contract and private evidence model

- [x] 1.1 Pin `@earendil-works/pi-coding-agent` to `0.80.10`; inspect the installed package's file-backed `SessionManager` constructor/factory and session/CLI export APIs, and document the exact v3 header, tree, message, close, and `pi --export INPUT.jsonl OUTPUT.html` contracts used by the adapter.
- [x] 1.2 Add hermetic no-auth tests using the pinned package that create a file-backed session, emit representative v3 JSONL, close it, reopen/inspect it, and exercise the export API/command through a controlled mock; ensure these tests never require OAuth, network access, or the ambient `pi` executable.
- [x] 1.3 Define versioned private record types for the session receipt, verification result, exact-prompt context sidecar, benchmark execution/configuration sidecar, lifecycle/tool-policy audit sidecar, provisional prediction/score sidecars, and failure metadata; include only the minimum fields needed to bind, verify, retain, and classify an attempt.
- [x] 1.4 Define the private per-attempt directory layout, expected filenames, restrictive permissions/ownership, allowed temporary export location, and terminal states `complete`, `partial`, `unavailable`, and `failed`; make every stored path relative to the attempt directory and explicitly exclude public bundles, credentials, oracle material, and HTML from canonical evidence.

## 2. Adapter session retention

- [x] 2.1 Change each evaluated attempt in `packages/pi-spatial-adapter` to create a fresh file-backed native Pi session under its private attempt directory using only the pinned `0.80.10` API; remove the evaluated path's `SessionManager.inMemory` usage and do not introduce a parallel custom conversation journal or transcript.
- [x] 2.2 Implement controlled session-path handoff from adapter to host: pass only an admitted relative session path/attempt identity across the boundary, reject absolute paths, traversal, symlinks, unexpected basenames, and credential-bearing values, and keep host paths and secrets out of protocol messages, tool responses, logs, and public outputs.
- [x] 2.3 Close each attempt without rewriting, normalizing, reordering, copying through a text serializer, or otherwise changing native JSONL bytes; preserve the native file and its bytes on normal completion and every failure/interruption path, and produce `partial` or `unavailable` metadata when no inspectable file exists.
- [x] 2.4 Emit the exact system and initial prompts as private sidecars with SHA-256 digests bound to the session receipt, alongside separate execution/configuration, lifecycle/tool-audit, prediction, and score sidecars; verify sidecar writes cannot mutate native JSONL or enter public result artifacts.
- [x] 2.5 Add adapter/runtime material needed to build the file-backed session and expose the receipt/context handoff while keeping the existing paired-mode gate and `pi-spatial-agent-baseline` artifacts unchanged.

## 3. Host admission and native-session verification

- [x] 3.1 Implement safe private-file admission that resolves the attempt root without following symlinks, permits only the declared native JSONL, receipt, approved private sidecars, and disposable export path, and rejects path escapes, symlinks, undeclared files, credentials, oracle data, and public-bundle destinations before reading session evidence.
- [x] 3.2 Implement receipt generation and verification with verbatim byte preservation, exact byte size, SHA-256, JSONL entry count, relative basename, producer/package version `0.80.10`, session-format version `3`, and terminal state checks; compare receipt values against the file without rewriting it.
- [x] 3.3 Implement native v3 validation for JSONL syntax, header identity/version, parent/child tree structure, required message fields, ordering/integrity, and complete versus valid-prefix semantics; classify evidence as `complete`, `partial`, or `unavailable` without fabricating a transcript.
- [x] 3.4 Verify exact system/initial prompt sidecar bytes and recorded digests, receipt/session binding, attempt identity, and approved benchmark sidecar references; keep prompt contents private and expose only safe status, hashes, counts, and failure codes operationally.

## 4. Provisional results, export, and publication gate

- [x] 4.1 Stage prediction and score as provisional records only after accepted submission and normal scoring; require normal scoring to complete but do not require answer correctness for the narrow precursor gate.
- [x] 4.2 Implement the result gate in dependency order: accepted submission and scoring, safe admission, receipt/hash/size/count and version checks, native v3 tree/message validation, prompt/sidecar binding, then pinned export verification; promote prediction and score atomically to the canonical private ledger only after every check succeeds.
- [x] 4.3 Invoke the pinned Pi CLI executable resolved from the pinned `0.80.10` installation as `pi --export INPUT.jsonl OUTPUT.html`, never an ambient or unverified `pi`; use the admitted native file as input, record export success/failure, confirm the source bytes/hash are unchanged, and delete disposable HTML/workspace output after inspection.
- [x] 4.4 On any evidence or publication failure, discard only provisional prediction/score staging, publish no canonical prediction or score, retain any valid partial native session, receipt, prompts, approved sidecars, and safe failure metadata, and never rewrite retained native bytes or committed ledger records.
- [x] 4.5 Enforce terminal failure precedence as infrastructure/session execution, evidence admission/receipt, native-tree validation, export, then result publication; record later cleanup errors only as supplemental safe metadata and make cleanup idempotent.

## 5. Manifests and public-safety boundaries

- [x] 5.1 Update private attempt/evidence manifests to identify canonical native session, receipt, sidecars, verification state, export result, provisional state, terminal failure, hashes, sizes, counts, package/session versions, and retention paths without embedding prompt text, credentials, oracle data, host paths, or HTML.
- [x] 5.2 Update public result and operational projections to include only safe verification/gate status and non-sensitive identifiers; add assertions that native JSONL, receipt contents, exact prompt sidecars, private scores, failure evidence, credentials, and exported HTML cannot be emitted or copied into public artifacts.

## 6. Focused tests and security coverage

- [x] 6.1 Add focused TypeScript adapter tests for fresh per-attempt file-backed sessions, absence of `inMemory`, controlled path handoff, normal/failure byte preservation, partial/unavailable states, receipt fields, prompt digests, and sidecar separation.
- [x] 6.2 Add focused Python/host tests for admission, receipt size/hash/count matching, pinned package/session identity, v3 header/tree/message validation, complete/partial/unavailable classification, prompt binding, provisional publication, failure precedence, and cleanup/rollback behavior.
- [x] 6.3 Add export tests with a hermetic mock pinned executable/API covering the exact `--export` argument order, source immutability, successful disposable output, export failure, and rejection of an ambient/unverified `pi`; keep all tests no-auth and network-free.
- [x] 6.4 Add privacy and unsafe-path tests covering traversal, absolute paths, symlinks, wrong attempt roots, undeclared files, credential/oracle leakage, public-path leakage, malformed JSONL, wrong versions, receipt mismatches, prompt digest mismatches, and failed evidence with retained safe partial metadata.
- [x] 6.5 Add a gate integration test proving accepted submission plus normal scoring, complete matching receipt/hash, valid session, and successful pinned export promotes prediction/score, while missing/invalid evidence leaves no canonical result and does not alter the paired-mode gate.

## 7. Narrow visualization-forbidden precursor

- [x] 7.1 Add exactly one narrowly scoped visualization-forbidden precursor command/mode that reuses the existing executor, adapter, submission, and normal scoring paths; require accepted submission, normal scoring, complete verified native session/receipt/hash, and successful pinned export, and do not replace, merge with, or alter the later paired gate.
- [x] 7.2 Add precursor state/manifest checks and focused tests for pass/fail conditions; explicitly keep answer correctness out of this gate and reject incomplete submission, scoring, session, receipt/hash, or export evidence without publishing prediction/score.

## 8. Build, configured precursor run, and documentation validation

- [x] 8.1 Build the adapter and current runtime artifacts needed by the change, run the focused TypeScript and Python test suites plus relevant lint/type checks, and resolve failures without unrelated refactoring, custom viewers, event journals, 100-case runs, or paired full runs. Qualifying runtime evidence must include an explicit sandbox image-entrypoint override and post-start running and sandbox-exec readiness checks.
- [x] 8.2 Run one fresh immutable configured visualization-forbidden precursor requiring an accepted submission and normal scoring (correctness not required); explicitly override the sandbox image entrypoint, verify post-start running and exec readiness plus pre/post-exec container liveness, classify container death or control-plane errors as terminal `container_runtime_failed`, and verify the complete session receipt/hash and successful `pi --export` using the pinned executable. Record only safe inspection metadata; the prior diagnostic-only precursor, whose persistent sandbox image entrypoint was not overridden, whose container exited, and whose sandbox execs were therefore infrastructure-invalid, cannot satisfy this task.
- [x] 8.3 Update only the affected docs/domain glossary if terminology or operator procedures changed, including native JSONL as canonical evidence, sidecars, provisional results, pinned CLI usage, private paths, precursor-versus-paired gate scope, and the rule that host session/export evidence does not attest sandbox health; leave unrelated baseline documentation unchanged.
- [x] 8.4 Run `openspec validate retain-native-pi-sessions --strict`, verify this `tasks.md` exists with exact `- [ ]` checkbox syntax and dependency order, and report focused-test, fresh precursor-run, manual-inspection-command, and validation results; do not commit or stage files.
