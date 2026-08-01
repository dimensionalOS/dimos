# Pi 0.80.10 viewer compatibility check

Check date: 2026-07-23

Decision: **go**. The viewer was implemented without changing the Pi package
pin, native format, adapter lifecycle, receipt models, prompt binding, export
path, or publication gate.

## ADR 0003 checklist

| Contract | Evidence run | Result |
|---|---|---|
| Exact package, CLI bin, and native v3 identity | `packages/pi-spatial-adapter/package.json`, lockfile, and `pinned package command validates executable and bin metadata` | Pi coding agent and Pi AI remain exactly `0.80.10`; pass |
| File-backed create and discoverability | `fresh sessions are persisted, distinct, and discoverable through public APIs` | Pass |
| Reopen with tree and bytes unchanged | `pinned SessionManager.open reopens an unchanged native v3 session` | Pass |
| Abort/dispose terminal lifecycle | adapter protocol tests, including setup failure and tool roundtrip | Pass |
| Complete and valid partial validation | Python native-session and viewer tests | Pass |
| Receipt, digest, safe path, and prompt binding | Python native-session and viewer admission tests | Pass |
| Pinned CLI export and source immutability | `pinned Pi CLI exports a synthetic native session without auth or rewriting JSONL` plus Python export tests | Pass |
| Publication remains independently gated | Python native-publication contract tests | Pass |

Commands and outcomes:

```text
cd packages/pi-spatial-adapter && npm test
24 passed

uv run --no-sync pytest \
  dimos/benchmark/spatial/pi_baseline/test_session_viewer.py \
  dimos/benchmark/spatial/pi_baseline/test_native_session.py \
  dimos/benchmark/spatial/pi_baseline/test_native_publication_contract.py \
  dimos/benchmark/spatial/pi_baseline/test_cli.py -q
64 passed
```

The adapter suite needed normal process-spawn permission for its verified Node
CLI export subprocess. Its initial sandbox-denied `EPERM` was environmental;
the unchanged suite passed once that permission was supplied.
