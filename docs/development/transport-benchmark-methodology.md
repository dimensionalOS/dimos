# Transport benchmark methodology

The public transport benchmark compares these complete stacks:

| Stack | Data path |
|---|---|
| `lcm` | DimOS typed message → LCM → UDP multicast |
| `zenoh` | DimOS typed message → native Zenoh |
| `ros2-zenoh` | ROS 2 Jazzy topic → `rmw_zenoh_cpp` → Zenoh |

It does not treat native Zenoh and ROS-over-Zenoh as identical middleware with
one extra wrapper. The manifest records each installed version and runtime
configuration.

## Comparison contract

Delivery semantics define separate cohorts:

| Cohort | LCM | Native Zenoh | ROS 2 over Zenoh |
|---|---|---|---|
| Best effort / drop | Included | best effort, drop | BEST_EFFORT, KEEP_LAST |
| Reliable / block | Not applicable | reliable, block | RELIABLE, KEEP_LAST |

Never make a single leaderboard across the two cohorts. LCM has no reliable
delivery mode, and loss can be the mechanism that keeps its latency low under
load. Shared-memory transport optimization is disabled in headline network
results and must be reported as a separate configuration if added later.

## Canonical single-machine setup

The local suite starts each publisher and subscriber in a separate host
process. The public emulated suite starts two endpoint containers on a
dedicated Docker bridge:

```text
publisher container ── Docker bridge ── subscriber container
        tc netem ↑                         ↑ tc netem
```

Both egress directions receive the selected named profile. All
endpoints run on one Linux kernel, so embedded `perf_counter_ns` timestamps
share a monotonic clock and one-way latency is valid. The bridge and containers
are created and removed for every trial. Endpoint MTU is fixed at 1500 bytes
and the LCM receive-buffer defaults and maximum are fixed at 64 MiB. Docker
network inspection, link state, and effective `tc qdisc` state are saved.
The campaign seed controls trial order and bootstrap resampling. Some packaged
`tc-netem` versions, including Ubuntu Noble's current build, do not expose an
RNG seed option; on those hosts, loss and jitter parameters are fixed but their
individual draws are stochastic across the five process restarts.

Requirements are Linux, Docker access, and the benchmark image pinned to the
official public ROS Jazzy base image:

```bash skip
docker build -f docker/transport-benchmark/Dockerfile \
  -t dimos-transport-benchmark:local .

dimos benchmark transport run \
  --suite public \
  --environment emulated \
  --image dimos-transport-benchmark:local \
  --output ./benchmark-results
```

Use the short end-to-end check while developing:

```bash
uv sync --extra benchmark
dimos benchmark transport run --suite smoke --stack lcm --output ./benchmark-results
```

Exercise the container topology with all three stacks before a campaign:

```bash
dimos benchmark transport run \
  --suite smoke \
  --environment emulated \
  --image dimos-transport-benchmark:local \
  --output ./benchmark-gate
```

Regenerate a report without rerunning trials:

```bash
dimos benchmark transport report ./benchmark-results
```

Campaign output is a durable checkpoint. If a run is interrupted, rerun the
same command with the same output directory. The runner validates the manifest,
keeps successful trials, and reruns failed or missing trial IDs. Transient
container setup failures are retried four times with backoff before becoming a
recorded trial error.

## Workloads

The synthetic matrix covers 64 B, 1 KiB, 16 KiB, the 60/64/65 KiB boundary,
256 KiB, 1 MiB, and 4 MiB. Paced rates cover control, telemetry, image, and
point-cloud regimes. Separate unpaced trials measure a bounded saturation burst:
at most 100,000 messages or 4 GiB offered, whichever comes first. Throughput for
these cells uses the recorded burst duration rather than the nominal paced
window. A mixed robot workload publishes control, odometry, image, and
point-cloud-shaped payloads at their own rates. Baseline topology is one
publisher and one subscriber, with focused 1-to-2/4/8 fanout trials after the
baseline.

The curated public campaign has 146 explicit cells, each with a 2 second
warmup, a 20 second measurement window, a 1 second drain period, and five
independent process restarts: 730 trials total. It does not form a Cartesian
product of every dimension. The campaign randomizes trial order with a recorded
seed. Discovery readiness is measured separately and is not included in
steady-state latency.

The headline single-machine network comparison selects `--environment
emulated`: 106 cells and 530 trials. The remaining 40 local cells are a separate
same-host process suite. Do not merge the two environments into one aggregate.

## Metrics and artifacts

Every payload contains a topic, publisher sequence, and monotonic publish
timestamp. The runner records offered and delivered messages and bytes,
sequence gaps, duplicates, out-of-order delivery, publish-call blocking time,
one-way latency, drain time, CPU time, context switches, and peak RSS for each
process. Reports show run-level medians and bootstrap 95% confidence intervals;
latency includes p50, p90, p99, p99.9, maximum, and p99-minus-p50 jitter.

The output directory is a portable evidence bundle:

```text
manifest.json        versions, host, command, seed, and configuration
trials.jsonl.gz      one summary record per trial
summary.json/csv     report input
comparisons.json     cells matched against the ROS 2 over Zenoh baseline
samples/*.jsonl.gz   raw per-message observations
topology/            Docker network, link, and qdisc state
logs/                endpoint logs
report.html          self-contained interactive report
```

The report shows native stacks relative to the matched ROS 2 over Zenoh cell.
Negative latency, CPU, and RSS deltas favor the native stack; positive goodput
deltas favor it; loss is reported as percentage-point difference. Use
within-stack changes on controlled hardware as regression gates. Cross-stack
results are descriptive; an absolute winner threshold is not a portable CI
contract.

## Primary references

- [LCM UDP multicast protocol](https://lcm-proj.github.io/lcm/content/udp-multicast-protocol.html)
  and [multicast setup](https://lcm-proj.github.io/lcm/content/multicast-setup.html)
- [`rmw_zenoh` design](https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md)
  and [configuration](https://github.com/ros2/rmw_zenoh)
- [Zenoh default transport configuration](https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5)
- [Linux `tc-netem`](https://man7.org/linux/man-pages/man8/tc-netem.8.html)
  and [`perf stat`](https://man7.org/linux/man-pages/man1/perf-stat.1.html)
