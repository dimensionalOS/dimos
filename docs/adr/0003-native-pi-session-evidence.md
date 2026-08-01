# ADR 0003: Use native Pi sessions as canonical conversation evidence

## Decision

DimOS spatial benchmark attempts retain Pi's native, versioned session JSONL verbatim as the canonical conversation record. Benchmark instructions, host lifecycle, tool-policy audit, prediction, and score remain separate private evidence: Pi's session format does not represent all of them. HTML is a disposable derivative made by the pinned Pi CLI, never a second transcript.

This preserves Pi's session-tree, migration, and inspection tooling without a parallel conversation format. Publication requires an admitted complete native session. A failed attempt may retain an admitted partial prefix for diagnosis, but it cannot publish a result.

## 1. Scope, pin, and upgrade rule

This contract is pinned to `@earendil-works/pi-coding-agent@0.80.10` and native session format v3. The installed package declares version `0.80.10`, the `pi` bin `dist/cli.js`, and the package entry point in `packages/pi-spatial-adapter/node_modules/@earendil-works/pi-coding-agent/package.json`; the adapter pins the same version in `packages/pi-spatial-adapter/src/session.ts` (`PINNED_PI_VERSION`).

Before any Pi upgrade, revalidate the installed types, session-format documentation, CLI behavior, adapter validator, export path, and all evidence tests. Do not treat a package or format-version change as compatible merely because existing JSONL parses.

## 2. Exact API and lifecycle

The pinned SDK exposes:

```ts
SessionManager.create(cwd, sessionDir?, options?): SessionManager
createAgentSession({ sessionManager }): Promise<CreateAgentSessionResult>
```

These signatures and `NewSessionOptions` are in `packages/pi-spatial-adapter/node_modules/@earendil-works/pi-coding-agent/dist/core/session-manager.d.ts` (`SessionManager.create`, `NewSessionOptions`) and `dist/core/sdk.d.ts` (`createAgentSession`). `create(cwd, sessionDir)` stores the header `cwd` and chooses a new JSONL file in `sessionDir`; if `sessionDir` is omitted, Pi uses its default per-cwd directory. The adapter supplies an attempt-local directory through `createSessionManager()` in `packages/pi-spatial-adapter/src/session.ts`.

The manager is append-only: `appendXXX()` adds entries, branching changes the leaf, and entries cannot be modified or deleted (`SessionManager.getEntries` documentation in the pinned declaration). It has no manager `close()` API. `SessionManager.inMemory()` exists upstream but is explicitly not admitted here: native evidence must be persisted.

Agent shutdown is host-owned: if a turn is active, `await session.abort()`; call `session.dispose()` exactly once; after disposal returns, read and validate the settled session bytes, then issue the receipt. The adapter exposes `abort`, idempotent `dispose`, and `sessionEvidence` in `packages/pi-spatial-adapter/src/session.ts`; the ordering seam is in `packages/pi-spatial-adapter/src/main.ts`. The SDK manager has no separate close operation.

## 3. Canonical conversation and private lifecycle evidence

Canonical conversation evidence is native Pi v3 JSONL, retained verbatim. `adapter.transcript.ndjson` is prohibited: it is not a compatible substitute, a second canonical transcript, or a recovery format. Lifecycle evidence is private and content-free `adapter-lifecycle-audit.v1.jsonl`. Its records are versioned and typed and contain only bounded event, status, count, and digest metadata; they never contain prompts, tool arguments, tool results, model content, or other conversation content.

## 4. Native v3 JSONL contract

Pi writes one JSON object per line. The first line is a header, not a tree entry:

```json
{"type":"session","version":3,"id":"<session-id>","timestamp":"<ISO timestamp>","cwd":"<cwd>","parentSession":"<optional parent session path>"}
```

`parentSession` is optional. Header field names and types are defined by `SessionHeader` in the pinned `dist/core/session-manager.d.ts` and `docs/session-format.md`.

Every subsequent entry has the common fields `type`, `id`, `parentId` (a string or `null`), and `timestamp` (an ISO timestamp string). The upstream `SessionEntry` union is exactly: `SessionMessageEntry`, `ThinkingLevelChangeEntry`, `ModelChangeEntry`, `CompactionEntry`, `BranchSummaryEntry`, `CustomEntry`, `CustomMessageEntry`, `LabelEntry`, and `SessionInfoEntry`. Their discriminators are respectively `message`, `thinking_level_change`, `model_change`, `compaction`, `branch_summary`, `custom`, `custom_message`, `label`, and `session_info` (`SessionEntry` and the individual interfaces in the pinned declaration; examples in `docs/session-format.md`).

The local admitted union is the same nine entry types, with the fields checked by `validate_native_session()` in `dimos/benchmark/spatial/pi_baseline/native_session.py`: message payloads, the top-level fields for each non-message type, and only the references that the validator knows about. IDs are unique. A non-null `parentId` and `firstKeptEntryId`, `fromId`, or `targetId` reference must identify an earlier entry. A complete session has one complete header and one tree root among entries; the header is not counted and is never a root.

## 5. Message roles and required fields

Upstream `AgentMessage` includes user, assistant, tool-result, bash-execution, custom, branch-summary, and compaction-summary message shapes (`docs/session-format.md`, “AgentMessage Union”). In a native `message` entry, the pinned local validator admits only these roles: `user`, `assistant`, `toolResult`, `custom`, and `bashExecution`. The local role allow-list and requirements are in `_ROLES` and `validate_native_session()` in `native_session.py`.

Required local fields are:

* `user`: `content`, numeric `timestamp`.
* `assistant`: `content`, `api`, `provider`, `model`, object `usage`, `stopReason`, numeric `timestamp`.
* `toolResult`: `content`, `toolCallId`, `toolName`, boolean `isError`, numeric `timestamp`.
* `custom`: `content`, `customType`, boolean `display`, numeric `timestamp`.
* `bashExecution`: `command`, `output`, boolean `cancelled`, boolean `truncated`, numeric `timestamp`; an optional `exitCode` must be an integer.

These required fields and type checks are **host admission policy**, not a claim that the upstream TypeScript types reject every extra or provider-specific field. Upstream permits user/custom string or text-image-array content; upstream assistant content is an array of text/thinking/tool-call blocks, and tool-result content is an array of text/image blocks (`docs/session-format.md`). The local validator admits user/custom strings or arrays whose items have `type` `text` or `image`, and requires assistant/tool-result content to be arrays without interpreting provider payloads. Nested message timestamps are finite JSON numbers and reject booleans. Assistant `stopReason` must be exactly `stop`, `length`, `toolUse`, `error`, or `aborted`. Top-level entry timestamps are non-empty strings.

## 6. Newlines, counts, and state

The file is newline-delimited JSON: one object per line, with a complete terminating newline for a complete capture. `entry_count` excludes the header. `validate_native_session()` reports:

* `complete`: header, entries, references, one root, and terminating newline all pass; at least one entry is required.
* `partial`: a header and valid prefix are present; an unterminated final JSON line may be retained as an incomplete tail, and a valid final line may also be retained without a terminating newline; the count is the valid-entry count.
* `unavailable`: no safely usable session can be admitted (for example missing, unreadable, unsafe, oversized, or invalid before a usable header/prefix).

State is caller-supplied and is recorded explicitly; a partial state does not turn an invalid complete file into a complete session (`validate_native_session()` and `receipt_for_session()` in `native_session.py`).

## 7. Receipt and prompt evidence

The host creates a receipt only after the abort/dispose and byte-settle ordering above. `receipt_for_session()` itself only reads and validates the retained files. `NativeSessionReceipt` in `native_session.py` records producer `@earendil-works/pi-coding-agent`, package version `0.80.10`, format version `3`, state, session ID, attempt-relative path, byte count, SHA-256, entry count, prompt records, and an optional failure reason. Captured receipts require all identity, size, hash, count, and prompt fields; unavailable receipts carry only state and reason.

The exact system prompt and initial prompt are separate `0600` files at `pi-prompt/system.txt` and `pi-prompt/initial.txt`. Each `PromptContextRecord` records kind, relative path, byte count, SHA-256, and the native session ID. The sidecars bind prompt bytes to the session without injecting synthetic messages or changing native JSONL (`PromptContextRecord`, `write_prompt_context`, and `verify_prompt_context` in `native_session.py`; retention in `packages/pi-spatial-adapter/src/session.ts`).

## 8. Pinned export

Pi's documented export syntax is:

```sh
pi --export INPUT.jsonl OUTPUT.html
```

For this pin, a direct package-entrypoint invocation must use a verified Node executable:

```sh
NODE='<verified-node-executable>'
PI_PACKAGE_ROOT='<verified-package-root>'
INPUT='<attempt-private-session.jsonl>'
OUTPUT='<temporary-private-export.html>'
"$NODE" "$PI_PACKAGE_ROOT/dist/cli.js" --export "$INPUT" "$OUTPUT"
```

The package metadata binds `pi` to `dist/cli.js`; `resolvePinnedPiCli()` in `packages/pi-spatial-adapter/src/session.ts` resolves the nearest verified `node_modules/@earendil-works/pi-coding-agent/package.json` while walking upward from the adapter module, then checks exact version `0.80.10`, exact bin metadata `"pi": "dist/cli.js"`, and a present entrypoint. `resolvePinnedPiExportCommand()` uses `process.execPath` plus that entrypoint. The dependency declaration and lock resolution are in `packages/pi-spatial-adapter/package.json` and `packages/pi-spatial-adapter/package-lock.json`.

`export_session()` in `dimos/benchmark/spatial/pi_baseline/native_session.py` uses the admitted source descriptor to make a private snapshot, invokes without a shell, applies a timeout, requires exit zero and non-empty HTML, and rechecks the original source identity and bytes after export. The native input is never rewritten. The HTML is temporary/private, not canonical evidence, and must be deleted after inspection.

## 9. Provisional publication state machine

Publication stages `prediction.provisional.v1.json` and `score.provisional.v1.json` first. Each is promoted to its canonical result path only with fd-relative Linux `renameat2(RENAME_NOREPLACE)`. There is no hardlink or overwrite fallback. The exact-identity `evidence-manifest.v1.json` commit marker is written last, after both canonical promotions.

Markerless recovery removes provisional files and canonical result orphans. When a marker exists, the loader preserves an exact committed result. If marker admission fails because its identity, syntax, or artifact hashes conflict, recovery preserves the marker and all result files unchanged, then raises a deterministic diagnostic conflict; committed-looking evidence is never rolled back automatically.

## 10. Native export verification contract

The private typed `native-export-verification.v1.json` record contains status, pinned package/version/v3 identity, the relative session path, source counts and hashes before and after export, verified CLI identity, confirmation that disposable non-empty HTML was produced and disposed, and a bounded failure code/reason. It is retained on both success and failure; on success it is bound into the evidence manifest. Package-resolution failures also retain a failed record with the bounded `executable_unsafe` code before the attempt fails. The record contains no exception text, HTML, stdout, stderr, host paths, or private contents. The export gate succeeds only on a typed success record.

## 11. Compatibility evidence and upgrade checklist

Current evidence is deliberately named and executable:

* Adapter persistence, path safety, create/append/discoverability, prompt bytes, pinned export, and package metadata: `file-backed session directory accepts only a simple relative name`, `precreated symlink and non-directory session children are rejected`, `fresh sessions are persisted, distinct, and discoverable through public APIs`, `pinned SessionManager.open reopens an unchanged native v3 session`, `a persisted manager with a delayed nonexistent file is unavailable`, `native JSONL remains after a model-independent public-API append`, `pinned Pi CLI exports a synthetic native session without auth or rewriting JSONL`, and `pinned package command validates executable and bin metadata` in `packages/pi-spatial-adapter/test/session.test.ts`. The reopen test uses the pinned upstream `SessionManager.open(path, sessionDir?)` API documented in `packages/pi-spatial-adapter/node_modules/@earendil-works/pi-coding-agent/docs/session-format.md` and declared in `dist/core/session-manager.d.ts`; it verifies native v3 identity, entries, tree continuity, and unchanged bytes without inventing a second format.
* Protocol setup and terminal unavailable evidence: `runs a fresh injected session and completes a tool roundtrip` and `setup failure after run_start emits terminal unavailable session evidence` in `packages/pi-spatial-adapter/test/protocol.test.ts`.
* Strict v3 tree, messages, IDs, prior references, complete/partial states, prompt binding, safe paths, and Python export validation: `test_representative_v3_tree_and_references`, `test_wrong_message_roles_are_rejected`, `test_message_declared_fields_are_not_relaxed`, `test_message_content_containers_and_aborted_stop_reason`, `test_custom_message_content_and_numeric_timestamps_are_strict`, `test_numeric_entry_fields_reject_bool`, `test_custom_message_is_not_a_message_role`, `test_tree_and_references_must_be_prior`, `test_state_is_caller_supplied_and_receipts_are_explicit`, `test_prompt_exact_bytes_and_permissions`, `test_session_admission_rejects_unsafe_paths_and_links`, and `test_export_uses_validated_package_and_does_not_mutate_source` in `dimos/benchmark/spatial/pi_baseline/test_native_session.py`.

Before upgrading: verify the exact installed package and bin metadata; reread `SessionHeader`, `SessionEntry`, `NewSessionOptions`, `SessionManager.create`, `SessionManager.open`, `createAgentSession`, and export implementation; regenerate representative native v3 files; rerun adapter and Python tests; recheck complete, partial, unavailable, prompt, path, source-integrity, timeout, and non-empty-output behavior; then update this ADR and the glossary reference. No upgrade is accepted with a changed session version, union, shutdown contract, or export implementation until that evidence is reviewed.

## 12. Gate distinctions and operator inspection example

The exactly-one forbidden-precursor check remains distinct from the paired prediction-and-score publication gate; passing one does not imply passing the other. Answer correctness is measured, but it is not a gate for session verification or evidence publication.

Host adapter session retention and pinned export evidence attest only to the Pi session and its review derivative; they do not attest sandbox health. Acceptance of the correctness-independent forbidden precursor additionally requires valid sandbox infrastructure: an explicit image-entrypoint override, post-start running and exec readiness, and liveness before and after sandbox execs. The prior precursor is diagnostic-only because its persistent sandbox image entrypoint was not overridden, the container exited, and its sandbox execs were infrastructure-invalid; it cannot satisfy the precursor task or gate.

`container_runtime_failed` is a safe terminal infrastructure classification for container death, lost running/exec readiness, or container-runtime/control-plane errors. It does not describe the inner command's exit status, does not imply an answer or score, and cannot be recovered into precursor acceptance. A nonzero inner-command result is distinct and recoverable only when the sandbox remains live and ready through the exec.

Use placeholders only; never paste credentials, oracle material, private prompt contents, or live session content into an issue or report:

```sh
NODE='<verified-node-executable>'
PI_PACKAGE_ROOT='<verified-package-root>'
INPUT='<attempt-private-session.jsonl>'
OUTPUT='<temporary-private-export.html>'
"$NODE" "$PI_PACKAGE_ROOT/dist/cli.js" --export "$INPUT" "$OUTPUT"
```

Inspect the temporary private HTML only in the approved workspace, then delete it. The retained JSONL and its receipt remain the evidence; the HTML is disposable.
