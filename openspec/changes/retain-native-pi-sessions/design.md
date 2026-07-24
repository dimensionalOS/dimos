## Context

The spatial Pi adapter currently uses `SessionManager.inMemory`, leaving only an adapter transcript that cannot be reconstructed or inspected as the session Pi actually used. The canonical evidence must instead be the native session format emitted by the pinned `@earendil-works/pi-coding-agent` 0.80.10, whose session format is v3. Pi's native JSONL does not necessarily contain the exact system prompt, initial benchmark prompt, or benchmark lifecycle records, so those records need private sidecars rather than a replacement transcript.

## Goals / Non-Goals

### Goals

- Make a file-backed native Pi v3 session, scoped to one private attempt directory, the canonical conversation record.
- Preserve native JSONL byte-for-byte, with a receipt sufficient to identify and verify the retained file.
- Keep exact prompt context, benchmark lifecycle, tool audit, prediction, and score evidence separate and private.
- Verify a complete session with native v3 tree parsing and the pinned Pi CLI export before publishing results.
- Retain useful partial-session evidence on failed attempts while publishing no invalid result.
- Add the narrow one-run visualization-forbidden precursor gate without changing the later paired-mode gate.

### Non-Goals

- No custom conversation journal, transcript conversion, or viewer.
- No public session, prompt, oracle, credential, or HTML artifact.
- No change to the existing `pi-spatial-agent-baseline` artifacts or paired-mode gate.
- No new general event journal, distributed recovery protocol, or answer-correctness requirement for the precursor gate.

## Decisions

### Native, file-backed session is canonical

Use `SessionManager` backed by the private attempt directory instead of `SessionManager.inMemory`. Pin `@earendil-works/pi-coding-agent` at `0.80.10` and treat its native session format v3 as the compatibility contract. The native JSONL is copied or retained verbatim: no conversion, normalization, reordering, or parallel conversation representation is permitted.

The session receipt records the package and session-format versions plus the relative path, byte size, SHA-256, entry count, and state: `complete`, `partial`, or `unavailable`. The relative path is admitted only under the attempt directory; receipt checks reject traversal, unexpected files, and mismatched metadata.

**Alternative rejected:** continue retaining the adapter transcript or create a normalized journal. Either loses native Pi structure or creates a second conversation source that can drift from Pi's session tree and tooling.

### Context and benchmark evidence remain sidecars

Store exact system and initial prompts in private context sidecars because the system prompt is not guaranteed to appear in native JSONL. Keep benchmark execution/configuration, lifecycle, tool-policy audit, prediction, and score in their own sidecars. These records reference the session receipt but do not replace or rewrite native session entries.

**Alternative rejected:** inject all benchmark metadata into the native transcript. That would alter the canonical Pi record and blur Pi conversation evidence with host-controlled evaluation evidence.

### Export is verification output, not canonical evidence

Generate HTML only on demand by invoking the pinned CLI as `pi --export INPUT.jsonl OUTPUT.html`. The export input is the admitted native JSONL, and successful completion is recorded in the verification result. HTML is disposable and noncanonical; it may be used for inspection during verification but is not required as retained canonical evidence.

**Alternative rejected:** retain HTML as a second transcript. It duplicates the session and makes renderer output, rather than the native record, part of the evidence contract.

### Session-gated publication and narrow precursor gate

The prediction and score are staged as provisional sidecar results. They become canonical ledger results only after all of the following pass for the one visualization-forbidden run:

1. The submission was accepted and normal scoring completed; answer correctness is not required for this gate.
2. The receipt matches the private native file and the file is admitted under the attempt directory.
3. The session is a parseable native v3 tree.
4. The pinned Pi CLI export succeeds.

Any evidence failure fails the attempt, discards provisional result staging, and leaves no canonical prediction or score. A failed attempt may retain a valid partial native session and safe failure metadata. The paired visualization comparison remains a later, separate gate and is out of scope here.

**Alternative rejected:** publish prediction and score before evidence verification and repair them later. That would expose non-authoritative results and violate immutable-ledger semantics.

### Failure precedence and rollback

Use a simple terminal precedence order: infrastructure/session execution failure, evidence-admission or receipt failure, native-tree validation failure, export failure, then result-publication failure. The first applicable failure is the attempt's terminal failure reason; later cleanup errors are recorded as supplemental safe metadata and do not obscure it.

Publication is a final guarded step: write provisional sidecars first, verify the receipt/session/export, then atomically promote the prediction and score into the immutable private ledger. If verification or promotion fails, remove only provisional result staging; never rewrite the native JSONL, receipt, or already-retained failure evidence. No rollback of a committed canonical ledger record is attempted.

### Privacy and file admission

Attempt artifacts live in a controlled private directory with restrictive ownership/permissions and are never placed in public result bundles. Admission allows only the expected native JSONL, receipt, declared private sidecars, and temporary export path; it rejects oracle material, credentials, path escapes, and undeclared files. The export workspace is disposable and cleaned after verification.

**Alternative rejected:** rely on a custom viewer or broad directory snapshot for evidence. That expands the privacy boundary and introduces a non-Pi interpretation of the conversation.

### Implementation flow

```text
accepted submission
        |
        v
file-backed Pi v3 session in private attempt dir
        |
        +--> native JSONL + context/lifecycle/tool/result sidecars
        |
        v
close attempt -> receipt (path, size, sha256, entries, state)
        |
        v
admit + parse native v3 tree
        |
        v
pin-checked `pi --export INPUT.jsonl OUTPUT.html`
        |
   pass  +--------------------+  fail
        v                    v
promote provisional     fail attempt; retain valid
prediction/score        partial session + safe metadata
to immutable ledger
```

## Risks / Trade-offs

- Pinning Pi 0.80.10 limits upgrades until the v3 format and export behavior are revalidated, but avoids ambiguous session compatibility.
- A crash can leave truncated JSONL; retaining it is diagnostically useful, while the receipt state and parser prevent it from becoming canonical result evidence.
- Native JSONL may omit system context, requiring private sidecars and careful access controls; this is preferable to modifying the native record.
- CLI export adds a verification dependency and runtime cost, but provides an inspectable Pi-compatible check without making HTML canonical.
- Restrictive file admission can reject unexpected adapter output; explicit failure is safer than silently retaining unreviewed private data.

## Migration Plan

1. Pin and verify `@earendil-works/pi-coding-agent` 0.80.10 and identify the v3 session entry/tree contract used by the adapter.
2. Change new attempts to create a file-backed `SessionManager` under a private per-attempt directory and capture the prompt/context sidecars and receipt.
3. Add receipt admission, native-tree validation, pinned export verification, provisional-result staging, and guarded ledger promotion.
4. Add focused tests for byte preservation, partial/unavailable states, privacy/admission, receipt matching, export failure, failure precedence, and publication rollback.
5. Run the one-run visualization-forbidden precursor gate. Keep existing paired-mode artifacts and gate behavior untouched; prior in-memory attempts are not retrofitted and remain noncanonical.

## Open Questions

None. The remaining implementation details can follow the pinned Pi v3 API and the existing private run-artifact conventions without introducing another design decision.
