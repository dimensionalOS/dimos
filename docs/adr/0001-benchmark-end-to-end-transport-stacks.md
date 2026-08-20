# ADR 0001: Benchmark end-to-end transport stacks

- Status: Accepted
- Date: 2026-08-19

## Context

DimOS supports LCM and native Zenoh, while ROS 2 can use Zenoh through
`rmw_zenoh_cpp`. Their APIs, serialization, delivery semantics, discovery, and
router requirements differ. An in-process microbenchmark cannot represent the
deployed path and can hide process, middleware, and loss costs.

## Decision

Benchmark complete production stacks with separate publisher and subscriber
processes. The public comparison contains DimOS LCM, DimOS native Zenoh, and
ROS 2 Jazzy with `rmw_zenoh_cpp`. Pin the ROS stack in a Docker image and run
the canonical two-endpoint suite on one Linux host. Use the official public ROS
Jazzy image, a dedicated Docker bridge, and symmetric `tc netem` qdiscs on both
endpoint containers.

Separate best-effort/drop results from reliable/block results. Disable shared
memory optimization in headline network results. Save raw per-message samples,
configuration, versions, topology, logs, and process-level resource usage.

The benchmark measures stack performance. It must not describe the difference
between native Zenoh and ROS-over-Zenoh as pure ROS API overhead because the
stacks can contain different Zenoh versions and configuration layers.

## Consequences

Results are reproducible on one suitable machine and include real kernel and
container networking. The public suite costs more time than an in-process
microbenchmark. Cross-host clock synchronization is unnecessary for the
canonical suite because all containers share the host monotonic clock; a later
physical two-host suite must use RTT or measured PTP synchronization error.
