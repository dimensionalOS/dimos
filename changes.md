# PR #1568 (rosnav) — Paul Review Fixes

## Commits (local, not pushed)

### 1. `9c1a963a8` — Send Move() before starting timeout timer
- Timer could fire before Move() was sent → stop-then-move race
- Now sends Move() first, then starts timer
- **Revert:** `git revert 9c1a963a8`

## Not addressed (need Jeff's input / bigger refactor)
- Container launch in `__init__` vs `start()` — lifecycle redesign
- Deterministic container naming collision across processes
- `_goal_reach` tristate without memory barrier — needs threading.Event refactor
- `_running` flag TOCTOU in ROSNav.start() / _spin_node
- `stop_navigation()` + new thread state ordering race
- Class-level mutable `rpc_timeouts: dict = {}`
- `docker pull` error missing stderr
- O(N) Python loops in slow-path pointcloud deserialization
