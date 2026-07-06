# TASK: `dimos spy` — universal transport spy

## Goal (the outcome)

`dimos spy` shows every topic flowing on every DimOS pubsub transport — LCM and Zenoh
simultaneously in v1 — with per-topic message rate, bandwidth, average size, total
traffic, and liveness (age since last message), in a Textual TUI matching `dimos lcmspy`'s
look. **The spy never decodes a message payload** (hard constraint from Ivan): its
per-message work is `(topic string, payload length, timestamp)` and nothing else.

Along the way, fix the pubsub spec so `subscribe_all` means the same thing on every
transport (every message, no conflation) and move conflation to an explicit opt-in.

## Context & where it fits

Design doc (read first): `docs/usage/transports/spy.md`. Decisions were ratified by
Ivan/brain in ticket f6c74d39; the doc records them. Summary: zenoh's `subscribe_all`
conflates (latest-wins, built for the rerun bridge, its only consumer) and only watches
`dimos/**`; LCM's delivers everything. The spy builds on `subscribe_all`, so the spec
asymmetry gets fixed as part of this task.

The branch already contains the full skeleton — this file, typed stubs (compile + pass
mypy, bodies raise `NotImplementedError`), and contract tests in
`dimos/protocol/pubsub/test_spy.py` that fail until implemented. **The tests are the
acceptance criteria.** Don't weaken them; if one seems wrong, flag it instead.

## What to implement

### 1. Spec fix: non-conflating `subscribe_all` + opt-in `subscribe_latest`

- `dimos/protocol/pubsub/spec.py` — implement
  `SubscribeLatestMixin.subscribe_latest()`: the latest-wins-per-topic buffer + single
  drain thread + wake event currently living in `ZenohPubSubBase.subscribe_all`
  (`impl/zenohpubsub.py:199`) — lift that machinery here essentially verbatim (it is
  transport-agnostic: key the latest-dict by `str(topic)`). Contract is in the stub
  docstring: callback on the drain thread, newest-per-topic eventually delivered,
  returned callable unsubscribes and stops+joins the drain.
- `impl/zenohpubsub.py` — `subscribe_all` becomes a plain non-conflating
  `self.subscribe(Topic("**"), callback)` (scope widened from `dimos/**` to `**` per
  ratified decision; see design doc). Keep the stop()-vs-subscribe locking behavior that
  is already there. Delete the now-lifted drain machinery, but keep zenoh's
  drain-stop bookkeeping working for `subscribe_latest` subscriptions —
  `stop()` must still terminate any live drain threads (move `_drain_stops`
  handling as needed; simplest: `subscribe_latest` registers its stopper the same
  way `subscribe_all` used to).
- `dimos/visualization/rerun/bridge.py:461` — the bridge needs conflation
  (rerun lags otherwise, see the old comment): switch `pubsub.subscribe_all(...)` to
  `pubsub.subscribe_latest(...)`.
- LCM's `subscribe_all` already satisfies the contract — leave it.

### 2. Spy core: `dimos/protocol/pubsub/spy.py`

All stubs documented in-file. Implementation notes:

- `TopicStats`: port the math from `dimos/utils/cli/lcmspy/lcmspy.py` `Topic` (deque of
  `(timestamp, nbytes)`, windowed freq/bandwidth/avg-size, lifetime totals) but with
  explicit timestamps (see stub docstrings) and typed. Lock around deque mutation vs
  reads (record() on transport threads, queries from the UI thread).
- `LCMSpySource` / `ZenohSpySource`: own a raw-bytes bus instance
  (`LCMPubSubBase(**kwargs)` / `ZenohPubSubBase(**kwargs)`), `start()/stop()` delegate,
  `tap(cb)` = `bus.subscribe_all(lambda msg, topic: cb(str(topic), len(msg)))`.
  That lambda is the entire hot path — no other work is permitted there.
- `TransportSpy`: on `start()`, start each source and tap it with a callback that stamps
  `time.time()` once and updates the per-`SpyKey` `TopicStats` + `totals`. `snapshot()`
  returns a shallow copy of the dict (TopicStats objects shared, dict copied).
- `subscribe_decoded`: leave raising `NotImplementedError` — it is the spec'd hook for
  the follow-up task (per-topic opt-in decode, separate subscription via the typed
  encoder classes; never part of the tap path).
- `default_sources()`: `[LCMSpySource(), ZenohSpySource()]` — both, always; the spy
  ignores `DIMOS_TRANSPORT` (it observes, it doesn't participate).

### 3. TUI + CLI: `dimos/utils/cli/spy/run_spy.py`

Model on `dimos/utils/cli/lcmspy/run_lcmspy.py` (DataTable, 0.5 s refresh interval,
theme colors/gradients, `q`/`ctrl+c` quit, `web` argv mode via textual-serve). Layout
and flags are in the stub docstring. Sort rows by total traffic; render Type via
`split_type_suffix`; grey/dim rows whose age exceeds a few seconds (liveness).
Call `lcmservice.autoconf(check_only=True)` before entering raw TUI mode when the LCM
source is active (run_lcmspy.py:84 shows why).

`dimos spy` is already registered in `dimos/robot/cli/dimos.py`. Make `dimos lcmspy`
a deprecated alias: print a one-line deprecation note to stderr and run the spy with
the lcm source only (keep the command working; delete nothing in
`dimos/utils/cli/lcmspy/` in this task).

### 4. Hot-path caveat to fix while you're there

`Topic.from_channel_str` (`impl/lcmpubsub.py:113`, called per message on LCM pattern
subscriptions) calls `resolve_msg_type`, which attempts imports and is **uncached**
(`dimos/msgs/helpers.py:26`) — that's real per-message overhead on the spy's LCM path.
Zenoh already caches the equivalent via `@lru_cache` on `_key_expr_to_topic`
(`impl/zenohpubsub.py:93`). Add `functools.lru_cache` to `resolve_msg_type` (or cache
channel→Topic in the LCM handler, whichever is smaller).

## Constraints / non-goals

- **No payload decoding on the spy hot path, ever.** The contract tests monkeypatch
  `Vector3.lcm_decode` to explode and publish undecodable garbage to enforce it.
- No conflation anywhere in the spy path — every message counts once.
- Non-goals for v1: SHM/ROS/DDS/Redis sources; implementing `subscribe_decoded`;
  restructuring `dimos/utils/cli/lcmspy/` (the alias just routes around it);
  zero-copy zenoh sizing (`len(sample.payload)` without `to_bytes()`) — noted as
  future work in the design doc.
- Follow repo conventions: `uv` for deps, mypy + pytest green before commit
  (`bin/pytest-fast`), no docstring bloat.

## Pointers (read in this order)

1. `docs/usage/transports/spy.md` — the design doc (why + decisions).
2. `dimos/protocol/pubsub/spec.py` — subscribe-all abstraction + new mixin stub.
3. `dimos/protocol/pubsub/impl/zenohpubsub.py:199` — the conflation machinery to lift.
4. `dimos/utils/cli/lcmspy/lcmspy.py` — stats math to port; `run_lcmspy.py` — TUI to model.
5. `dimos/protocol/pubsub/spy.py` — the stubs you're filling in.
6. `dimos/protocol/pubsub/test_spy.py` — the acceptance tests.
7. `dimos/visualization/rerun/bridge.py:461` — the one-line bridge migration.

## Acceptance

- `uv run pytest dimos/protocol/pubsub/test_spy.py` — all pass.
- Existing suites stay green, notably `test_spec.py`, `test_zenohpubsub.py`,
  `test_lcmpubsub.py`, and rerun bridge tests.
- `mypy` clean.
- Manual smoke: `dimos --transport=zenoh run unitree-go2 --replay ...` (or any
  publisher) in one terminal, `dimos spy` in another → topics from both zenoh and LCM
  visible with sane rates; `dimos lcmspy` still works and warns it's deprecated.
