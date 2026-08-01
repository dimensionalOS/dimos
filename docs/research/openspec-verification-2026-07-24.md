# OpenSpec Verification Report — 2026-07-24

## Scope

This review checked all four active OpenSpec changes against the current implementation, tests, and design artifacts:

| Change | Task status | Strict validation | Assessment |
| --- | ---: | --- | --- |
| `agentic-skill-benchmark-harness` | 40/40 | Passed | Archived |
| `retain-native-pi-sessions` | 31/31 | Passed | Archived |
| `add-pi-session-history-viewer` | 23/23 | Passed | Archived |
| `pi-spatial-agent-baseline` | 52/52 | Passed | Archived after release-phase scope removal |

OpenSpec's artifact status reports all four changes as structurally complete because each has a proposal, design, specs, and tasks file. The task checkboxes remain the implementation-completion authority.

## Verification results

### Automated checks

- `uv run --no-sync pytest dimos/benchmark/spatial -q`: 589 passed, 1 skipped.
- `packages/pi-spatial-adapter`: 24 tests passed; typecheck and production build passed.
- `packages/pi-session-viewer`: 7 tests passed; typecheck and production build passed.
- `openspec validate <change> --strict`: passed for all four changes.
- `git diff --check`: passed.

The skipped Python test was the opt-in Podman integration test because `DIMOS_SPATIAL_RUNNER_IMAGE` did not contain a named immutable digest in this verification shell. The completed 1,170-case benchmark exercised the live API-key, scheduler, adapter, and Podman path, but this report does not count that prior run as a fresh execution of the skipped test.

The first sandboxed adapter run reported two file-level failures. Both tests launch a child Node process, and the minimized run showed `spawnSync /usr/bin/node EPERM`. Re-running the exact clean package test outside the process-restricted sandbox passed all 24 tests. No implementation change was required.

### Implementation and specification mapping

The corpus harness now has direct implementation and test evidence for every task. It uses the real `VoxelGridMapper`, emits `global_map.pc2.lcm`, validates deterministic generation and public/oracle separation, covers seven predicates and three variants, enforces the smoke gate, and records the Structured3D distribution restriction in `docs/development/static_spatial_benchmark_data_terms.md`.

Native Pi session retention creates file-backed sessions through the pinned public Pi API, preserves and verifies native JSONL, retains the initial system prompt independently, exports review HTML, and publishes committed evidence only after integrity checks. The adapter suite also verifies reopen and export behavior without model authentication.

The session viewer admits committed attempt evidence, stages a read-only local bundle, binds only to loopback, serves a restrictive CSP, and opens the retained Pi history without rewriting native session evidence. Its Python admission/server tests and frontend tests, typecheck, and build all pass.

The Pi baseline's archived scope matches 52 tasks:

- Dual authentication is explicit and fail-closed in `dimos/benchmark/spatial/pi_baseline/config.py:21` and `dimos/benchmark/spatial/pi_baseline/config.py:99-212`. Codex OAuth maps to `openai-codex`; an owner-only dotenv API key maps to `openai`.
- Prompt specifications now match the exact hardened encouraged-mode instruction in `dimos/benchmark/spatial/pi_baseline/prompts.py:15-18`.
- The scheduler uses immutable manifests and attempts, fixed bounded concurrency, explicit resume/retry semantics, atomic terminal outcomes, and operational-only status.
- The case executor stages public data only, runs analysis in isolated rootless Podman workspaces, exposes three case-bound tools, records native sessions and audit evidence, and scores privately.

The proposal, design, runner spec, prompt-mode spec, and tasks now describe both authentication modes, the exact encouraged-image prompt, and the split between a serial smoke precursor and bounded experiment concurrency.

## Scope decision and archive result

The release/report phase was removed from the required baseline scope before archival. The removed requirements covered private review/report commands, condition-comparison publication, a fixed paired-smoke release gate, release-specific documentation, and opt-in release CI. The working experiment scheduler, private per-run scoring, prompt-condition parity, immutable evidence, native-session retention, and local session viewer remain specified.

All four changes were synced to 13 main capability specs and moved under `openspec/changes/archive/2026-07-24-*`. No active OpenSpec changes remain.

## Warnings and suggestions

- OpenSpec printed PostHog DNS errors after each successful validation because telemetry could not reach `edge.openspec.dev`. Each validator returned exit code 0; the telemetry error did not affect validation.
- The proposed `submit_answer` additions—an `unsure` result and a separate reasoning field—remain a wishlist item in the progress notes. They are not implemented and were not marked complete or added to the current baseline requirements.

## Final assessment

The corpus harness, baseline execution path, native-session retention, and session viewer are coherent with the retained scope and archived. The archived baseline includes the verified API-key execution path and excludes the deferred release/report phase.
