# PR #1539 (unity sim) — Paul Review Fixes

## Commits (local, not pushed)

### 1. `4aaed7713` — Launch Unity in thread to avoid blocking start()
- `_launch_unity()` blocked up to 30s during blueprint build
- Now runs in a daemon thread
- **Revert:** `git revert 4aaed7713`

### 2. `c22be178b` — Pipe Unity stderr to logger
- stderr was DEVNULL → crashes undiagnosable
- Now piped to reader thread, logged at warning level
- **Revert:** `git revert c22be178b`

### 3. `9010b6177` — Clear _unity_ready on disconnect
- Was one-shot Event, never cleared → inconsistent state on reconnect
- Now cleared in connection finally block
- **Revert:** `git revert 9010b6177`

## Reviewer was wrong on
- Test `_wire()` not wiring `tf` — tf auto-creates via LCMTF lazy init, tests pass
- `__raw__` magic string — acceptable internal protocol, not worth a typed tuple

## Not addressed (need Jeff's input)
- `__getstate__`/`__setstate__` key duplication → extract to `_UNPICKLABLE` frozenset?
- `np.sqrt` → squared distance comparison — minor perf optimization
- Slow-path O(N) Python loops in ros1.py pointcloud deserialization
- Test `time.sleep` for sync — acceptable for integration tests
