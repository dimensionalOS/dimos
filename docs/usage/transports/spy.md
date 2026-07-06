---
title: "dimos spy"
---

`dimos spy` is the universal transport spy: one live view of every topic moving on every
DimOS pubsub transport — names, message rates, bandwidth, sizes, and liveness — regardless
of whether the system runs on LCM, Zenoh, or both.

> Status: design ratified, implementation in progress. This page is the design doc;
> it becomes the user doc as the tool lands.

## Motivation

`dimos lcmspy` only sees LCM. Since the Zenoh transport merged (#2362), a robot can run
with `--transport=zenoh` (default on macOS) and the spy goes blind. Debugging "is data
flowing?" should not require knowing which backend is active — the spy should observe
everything at once.

## The hard constraint: no decoding on the hot path

Decoding messages is expensive (LCM decode, image payloads, pointclouds). The spy's job
is *flow* visibility — topic names, Hz, bytes/s, last-seen — none of which needs message
contents. Therefore:

**The spy never decodes a payload.** Its per-message hot path is
`(topic string, payload length, timestamp)` — an O(1) stats update.

This falls out of the pubsub layering: encoding/decoding lives in `PubSubEncoderMixin`
subclasses (`LCM`, `Zenoh`), while the base classes (`LCMPubSubBase`, `ZenohPubSubBase`)
move raw bytes. The spy taps the base layer. Message *types* are still displayed for
free because DimOS embeds them in topic strings (`/cmd_vel#geometry_msgs.Twist` on LCM,
`dimos/cmd_vel/geometry_msgs.Twist` as a Zenoh key — both normalize to the `#` form via
`str(Topic)`).

Per-topic decoded inspection (echo/preview) is a separate, opt-in concern with its own
subscription (`SpySource.subscribe_decoded`), spec'd now and implemented in a follow-up.

## Architecture

```
LCMPubSubBase.subscribe_all(".*")      ZenohPubSubBase.subscribe_all("**")
        │ raw bytes                            │ raw bytes
   LCMSpySource ("lcm")                  ZenohSpySource ("zenoh")
        │ (topic_str, nbytes)                  │ (topic_str, nbytes)
        └──────────────┬───────────────────────┘
                 TransportSpy
        dict[SpyKey(transport, topic) → TopicStats]  +  totals
                       │ snapshot()
                  dimos spy TUI
     Transport | Topic | Type | Freq | Bandwidth | Total | Age
```

- **`SpySource`** (Protocol): one transport's raw firehose. `tap(cb)` delivers
  `(topic, nbytes)` for *every* observable message. v1 ships `lcm` and `zenoh` sources;
  SHM/ROS/DDS/Redis plug in later by implementing the protocol.
- **`TopicStats`**: sliding-window `(timestamp, nbytes)` history per topic — freq,
  bytes/s, avg size over a window, plus lifetime totals and `last_seen` for liveness.
  Explicit timestamps keep it deterministic under test.
- **`TransportSpy`**: owns sources, merges taps, serves thread-safe snapshots to the UI.

Code: `dimos/protocol/pubsub/spy.py` (core), `dimos/utils/cli/spy/run_spy.py` (TUI).

## The subscribe_all contract fix

The spy builds on the pubsub layer's subscribe-all abstraction
(`dimos/protocol/pubsub/spec.py`), which exposed a semantic asymmetry:

- LCM's `subscribe_all` delivered every message (regex `.*`).
- Zenoh's `subscribe_all` **conflated** — latest-wins per topic with a drain thread,
  scoped to `dimos/**` — because it was built for its only consumer, the rerun bridge.

A spy counting rates through a conflating tap silently under-reports. Ratified decision
(Ivan, ticket f6c74d39): **`subscribe_all` is defined non-conflating on every
transport.** Conflation becomes the explicit opt-in `subscribe_latest()` (same
latest-wins + drain-thread semantics, hosted in the spec as a default), and the rerun
bridge migrates to it. Zenoh's `subscribe_all` also widens from `dimos/**` to `**` —
"all topics" means everything the transport can see; the rerun bridge is unaffected
because unknown/untyped keys already fail type resolution and are skipped by its decoder.

## Decisions (ticket f6c74d39)

| # | Decision |
|---|----------|
| 1 | `subscribe_all` = every message, non-conflating, on all transports; conflation moves to opt-in `subscribe_latest()`; rerun bridge migrates. Confirmed by Ivan. |
| 2 | v1 sources: LCM + Zenoh, behind the pluggable `SpySource` protocol. SHM/ROS/DDS/Redis later (SHM needs a discovery mechanism first). |
| 3 | `dimos spy` shows **all transports simultaneously**, rows tagged by transport; `dimos lcmspy` becomes a deprecated alias. |
| 4 | Lazy per-topic decode is spec'd (`subscribe_decoded`) but not implemented in v1 — decode stays off the hot path. |
| 5 | This design doc lives in dimos `docs/`. |

## CLI

```bash
dimos spy                     # everything, all transports
dimos spy --transport zenoh   # filter to one transport (repeatable flag)
dimos spy web                 # serve the TUI in a browser (textual-serve)
dimos lcmspy                  # deprecated alias for: dimos spy --transport lcm
```

## Future work

- SHM source (needs channel discovery), ROS/DDS/Redis sources.
- Per-topic decoded echo/preview from the TUI row (`subscribe_decoded` follow-up).
- Zero-copy zenoh sizing (`len(sample.payload)` without `to_bytes()`), if profiling
  shows the copy matters for pointcloud-heavy systems.
