## Why

The current Pi adapter uses `SessionManager.inMemory`, so its retained adapter transcript is not a reconstructable Pi session. Evaluation therefore cannot independently verify the exact native agent context or reliably produce the same inspectable record that Pi itself used.

## What Changes

- Retain the native Pi v3 JSONL session as the canonical Pi agent session record, preserving it verbatim in private run artifacts.
- Keep benchmark execution/configuration, tool audit, prediction, and score records as sidecars. Exact system and initial prompts remain private session-context evidence rather than public result data.
- Generate HTML only on demand with the pinned `pi --export INPUT.jsonl OUTPUT.html` command; exported HTML is derived evidence, not canonical session data.
- Gate result publication on session validity and inspectability. Prediction and score remain provisional until the retained native session passes validation; any evidence failure fails the entire attempt and publishes no canonical prediction or score.
- Retain valid partial native sessions privately for failed attempts.
- Add a narrow, one-run visualization-forbidden precursor gate requiring an accepted submission, normal scoring (answer correctness is not required), a valid retained session, a matching receipt, and successful Pi CLI export. Keep the existing paired-mode gate later and separate.
- Do not modify the existing `pi-spatial-agent-baseline` artifacts in this change.

## Capabilities

### New Capabilities

- `native-pi-session-retention`: Create and retain a private, verbatim, reconstructable native Pi v3 JSONL session alongside benchmark sidecars, including valid partial sessions from failed attempts.
- `pi-session-verification-gate`: Verify retained sessions and matching receipts, perform on-demand pinned Pi export, and publish prediction/score results only after the narrow precursor gate passes.

### Modified Capabilities

None. No existing main capability specification matches this narrow change.

## Impact

- Changes session creation in `packages/pi-spatial-adapter` from in-memory adapter retention to native Pi session retention.
- Updates the spatial Pi runner/controller and evidence manifests to track canonical sessions, private context evidence, sidecars, receipts, provisional outcomes, and verification status.
- Adds private per-run native JSONL and on-demand export verification, including partial-session retention for failed attempts; exported HTML is not retained as canonical evidence.
- Adds focused tests and documentation for native session reconstruction, privacy, validation, receipt matching, export, and publication gating.
- Depends on the pinned `@earendil-works/pi-coding-agent` version `0.80.10` and its Pi v3 JSONL/session and CLI export behavior.
- Leaves the existing paired-mode gate and `pi-spatial-agent-baseline` artifacts unchanged.
