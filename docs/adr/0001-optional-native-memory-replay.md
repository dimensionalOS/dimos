# ADR 0001: Keep native memory replay opt-in

## Status

Accepted

## Context

Python Memory2 replay reads, decodes, schedules, and republishes each observation
through Python. Recorded robot streams can fall behind wall time. DimOS already
has a native module boundary and a Rust recorder that writes SQLite and MCAP.

## Decision

Add a Rust SQLite replay source that reads selected streams incrementally,
decodes storage codecs, uses one replay clock, and publishes transport bytes
directly. Expose it through declared `Out[T]` ports and a separate Go2
blueprint.

Keep Python replay as the default. Selecting Rust is explicit and startup fails
on unsupported artifacts; it never falls back to Python. Keep arbitrary Memory2
queries in Python.

## Consequences

- Operators can compare replay engines on the same graph and recording.
- Rust owns artifact timing and high-rate publication; Python still owns graph
  composition and Go2-specific derived TF and camera calibration.
- SQLite metadata and timestamp indexes become a checked boundary between the
  recorder and replayer. Native MCAP replay is unsupported.
- SQLite applies the launch window and cross-stream ordering. A blocking cursor
  decodes into a capacity-one channel, so memory does not grow with recording
  size.
- Structural errors fail during startup. Payload corruption discovered later
  stops replay after earlier observations may have been published.
- Promoting Rust to the default requires measured latency gains and identical
  observation counts and ordering.
