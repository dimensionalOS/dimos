# Network quality preflight

`dimos network check` measures a bounded, synthetic Zenoh connection before you
start a robot stack. Run it on the local computer. It uses existing SSH access
to start an owned responder with the **absolute remote dimOS executable path**.
Both endpoints must already contain this command and the same supported Zenoh
version (`>=1.10.1,<2`, validated with 1.10.1). It never installs dimOS, transfers
a helper, starts a blueprint, or sends robot actions.

For example, use `dimos network check robot@robot-host --remote-dimos
/opt/dimos/.venv/bin/dimos`. Replace both the SSH target and the executable path
with your existing installation. The executable's shebang selects its environment;
no remote shell activation is needed. Batch-mode SSH requires working keys and
host-key configuration. This command does not troubleshoot or configure SSH.

```bash
dimos network check --help
```

The SSH connection carries orchestration and results. The measured traffic is
**direct TCP Zenoh**, outside the SSH tunnel, in a random per-session namespace.
Multicast and gossip discovery are disabled. A free remote port is chosen unless
you supply `--port`. `--listen-host` restricts the remote listener to a particular
IP; the default is `0.0.0.0`. Use `--peer-host` when an SSH alias is not the direct
TCP hostname/IP. No firewall, route, interface or router settings are changed.
Use this on a trusted network: the synthetic TCP session is not authenticated or
encrypted by SSH. The random namespace prevents topic collisions, not hostile
network access. A port-selection/bind race fails visibly.

## What happens

1. Start the remote peer, check protocol and Zenoh versions, then verify an actual
   Zenoh round trip before measuring. A connected socket alone is insufficient.
2. Measure idle RTT.
3. Ramp remote-to-local synthetic traffic, with simultaneous local-origin ping
   requests and remote pong replies on the same session.
4. Repeat the ramp local-to-remote. These directions are sequential, not a
   bidirectional saturation test.
5. Close both sessions and stop the SSH-owned responder. Report confirmed cleanup
   separately from unconfirmed cleanup after an SSH failure.

Engineering defaults are deliberately bounded and configurable: 50 Mbps offered
rate cap, 5 s idle, four steps at 1/8, 1/4, 1/2 and all of the cap, 5 s measurement
per step, 0.5 s warm-up and 1 s drain, 64 KiB synthetic messages, 50 Hz RTT probes,
0.5 s probe timeout, 90 s overall duration and 256 MiB sent bytes **per endpoint**.
Warm-up consumes the byte budget. Message size, scheduling and endpoint CPU can
limit the actual offered rate; the report shows it separately from the target.
Rate and byte caps count synthetic published message bytes; probe/control traffic
and TCP/Zenoh framing overhead are additional. These are not wire-rate caps.
The caps also include connection setup. A complete observation window must fit
before another step starts. Hard configurable limits are 1000 Mbps, 600 s,
1 GiB per sender, 200 Hz probes and 1 MiB messages. These are resource safeguards,
not recommendations for a particular robot or WiFi link.

## Read the dashboard

The terminal shows received goodput bars, RTT p95/p99 versus offered load, reply
sequence sparklines, p50/p95/p99, timeouts, missing messages and arrival gaps.
`R`, `L` and `+` identify plot lines without relying on color. Narrow terminals
stack the plots; noninteractive output uses a static ASCII report and progress
lines. `NO_COLOR=1` suppresses colors.
Use options after `network check`; preflight does not load blueprint/global
configuration or `.env` defaults.

RTT is measured with the local monotonic clock, including both transport paths
and endpoint handling. It is never divided by two to claim one-way latency.
Percentiles describe completed probes, with timeouts counted separately; small
sample counts make p99 coarse. The sparkline uses bucket maxima from the reply
sequence, not a uniformly sampled timeline or a synchronized remote clock.

Receiver callbacks count unique phase/sequence identifiers without latest-wins
coalescing. Goodput counts synthetic payload bytes (including their diagnostic
header) received in a fixed window starting at the first sample arrival. The
drain permits delivery accounting but does not extend the goodput window. Missing
means absent by the observation/drain deadline, **not proven network packet loss**.
The JSON also contains duplicates, reordering, mean arrival gap, gap standard
deviation and sender process CPU time. CPU time includes other work in that endpoint
process; it cannot alone identify a network bottleneck. Reliable/drop QoS is
explicit, matching Zenoh's usual publisher defaults. RTT probes use express
publication at the same default priority to avoid publisher batching delay.

Reaching the offered-rate cap does not establish maximum capacity. Synthetic
Zenoh tests do not certify stock Go2 WebRTC, video codecs, inference latency,
application topic QoS, other active routers, multicast discovery, or robot readiness.

## Thresholds, output and cancellation

By default the command reports measurements and no pass/fail verdict. Optional
`--min-goodput-mbps`, `--max-rtt-p95-ms` and `--max-missing-pct` apply only to
the specified criteria in each direction. With a goodput target, each direction
stops early when all supplied thresholds are met. Otherwise it ramps to the cap.
A latency threshold also requires zero probe timeouts; a missing reply must not
silently improve the percentile. The final threshold verdict uses each direction's
last completed step. This is a scoped threshold verdict, never a robot-ready verdict.

`--json` writes only versioned JSON to stdout and progress to stderr. `--json-file`
also saves full step counters and raw RTT samples while retaining the terminal
report. Exit codes: 0 completed (including measurements only), 1 error/incomplete
duration cap, 2 thresholds unmet or invalid CLI arguments, 130 user cancellation.

Ctrl-C stops owned traffic and requests peer cleanup. Stdin EOF on SSH disconnect
also stops the responder; its independent duration lease remains in force if the
disconnect is not immediately detected, with a final process deadline five seconds
later. Cleanup can exceed the measurement cap by bounded teardown time. Existing
dimOS processes and robot services are never targeted for termination. Results
from an interrupted run are partial and any requested verdict is inconclusive.

Rerun and distributed multi-machine orchestration are deferred. The MVP owns one
local endpoint and one remote endpoint.

See the [runner](/dimos/cli/network/runner.py),
[Zenoh endpoint](/dimos/cli/network/session.py), and
[terminal rendering](/dimos/cli/network/display.py) for implementation.
