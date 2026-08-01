## Why

Pi's pinned HTML export verifies that a retained session remains inspectable, but its presentation is too crude for efficient human review. Operators need a readable browser view of one retained Pi session without turning the viewer into a benchmark dashboard or allowing it to alter canonical evidence.

## What Changes

- Add `pi-baseline session view <attempt-directory>` to open one retained Pi session in a local browser.
- Show only Pi-native session history, including messages, thinking, tool calls and results, timestamps, usage data when present, and session branches.
- Keep the viewer strictly read-only and exclude benchmark prompts, scores, predictions, policy audits, logs, and other sidecars.
- Verify the retained session, stage a disposable copy, bind the viewer to loopback, and remove the staged copy when the command exits.
- Build a small repo-owned static viewer with shadcn/Base UI primitives, selectively adapted AI Elements presentation components, and Streamdown Markdown rendering.
- Map native Pi v3 JSONL into an immutable viewer model without importing a live agent or chat runtime.
- Retain the exact Pi `0.80.10` pin and revalidate native-session and export contracts without adding Pi Web.
- Keep the pinned `pi --export` check as the compatibility gate; the new viewer replaces neither canonical JSONL nor export verification.

## Capabilities

### New Capabilities

- `pi-session-history-viewer`: Open one verified native Pi session in a private, disposable, read-only browser UI while preserving canonical session evidence.

### Modified Capabilities

None.

## Impact

- Adds a session-review command to the spatial benchmark CLI and supporting private staging and process-lifecycle code.
- Adds a private Vite/React frontend package, committed source-owned display components, prebuilt local assets, and exact runtime dependency pins.
- Updates the native-session compatibility documentation and ADR with the viewer boundary and frontend provenance.
- Adds tests for version compatibility, source integrity, read-only enforcement, loopback binding, cleanup, and safe failure.
- Does not change public benchmark artifacts, scoring, result publication, or the canonical native-session format.
