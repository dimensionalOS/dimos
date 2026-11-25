# Diagnostics consolidation

The deliverable is a reusable way to attribute a slow robot session to a measured
processing stage or network path, with missing evidence shown explicitly. The
runtime and gateway own collection; applications consume it. A Wi-Fi survey adds
location to those observations. It is not a replacement navigation stack.

## Source integration

The November 25 consolidation uses these existing sources:

| Repository | Upstream base | Carried work |
| --- | --- | --- |
| `dimensionalOS/dimos` | `cae7e397f3` | Diagnostics from `d5dccdd2d9`; survey work from `a6aa689494`, `22e0009073`, `84317dccd1`; reconciliation and shared collectors on top. |
| `dimensionalOS/zenoh-gateway` | `8d6757b` | Existing streaming recovery `afb90ac`, then tracing `87f5b96`, replayed as `26eb1fd` and `c177858`. |
| `Nabla7/dim-spatial-desktop` | `bbc2ce7` | Existing browser integration `d636305`, replayed as `8d206cd`, plus safe cleanup when a camera is unsubscribed. |

All three candidates use `pim/refactor/diagnostics-consolidation`. The original branches
remain available. DimOS uses current upstream control routing; no historical
sim2/teleop stack ancestry is needed. The gateway base is Dimensional's repository,
not the older `jeff-hykin/zenoh-gateway` remote. No sibling Cargo source override is
part of these candidates.

The key consolidation changes are:

- One radio parser and bounded ICMP implementation under hardware sensors, shared
  by CLI and blueprint collection. Linux and macOS ping timeout/exit semantics
  remain distinct.
- Existing survey observations enter the incident bundle without another polling
  task. Missing interfaces recover with a fresh baseline; collection failures
  do not become packet loss or zero throughput.
- Process samples reuse the `dtop` collector and survive in incident history.
- Tracing preserves the current coordinator's command, deadman and hold behavior.
  Optional transport negotiation does not break existing custom transports.
- Capture roots resolve on each worker host and native children inherit an
  absolute path. The packaged radio helper and report template ship with DimOS.

## Contribution and release boundaries

1. **DimOS:** the existing Wi-Fi survey PR owns measurement messages and the map.
   Consolidation adds the shared incident integration and removes duplicate
   collectors. Reconcile that PR before merging this candidate; do not merge two
   copies of the survey implementation.
2. **Gateway:** upstream the streaming recovery and tracing changes in the shared
   gateway/SDK. Desktop then updates its normal pinned gateway dependency. No
   permanent private Desktop build or patched local dependency is the release path.
3. **Spatial app:** release the app against that SDK. Ordinary viewing works
   without tracing; opting into `?diagnosticsRun=RUN` requires the instrumented
   SDK and explicitly reports an unsupported client. The app keeps its own
   presentation callbacks and declared upload route.

A source branch is not a released dependency. Until the gateway changes are
released and Desktop's dependency is updated, a stock Desktop does not provide
these new trace APIs. This consolidation does not retire every patch in the
previous R1 media deployment, and it has not been deployed to the robot.

## Navigation use

Keep Jeff's navigation implementation separate. Compose its blueprint with
`wifi_survey(...)`, supplying the deployment's interface, literal probe targets
and a new archive path. Explicitly connect the survey to the mapper's pose stream
(Point-LIO for the tested R1 mapping stack). Wheel odometry is not interchangeable
just because both streams use a frame named `odom`.

Enable runtime diagnostics with the same run ID on participating processes.
Enable `dtop` independently if its live resource UI is needed. Collect the robot,
gateway and browser evidence for that run before evaluating latency. The survey
archive and incident bundle contain metadata; enabling them does not enable the
robot-data recorder.

## What this does and does not establish

Local regression coverage includes coordinator routing/holds with fake robots,
Python/Rust message lineage, survey recovery, bounded storage, gateway delivery,
and app upload/render callbacks. Package validation builds the normal DimOS wheel
and the app's `dimosApp` derivation. These are software checks, not new RF or
physical-robot performance measurements.

The remaining measurement boundaries are explicit:

- Application RTT and endpoint/station counters do not isolate AP scheduling,
  over-air retransmissions or backhaul. Those need matching AP/controller evidence.
- Current upstream R1 head cameras use native V4L2 modules. SDK publication and
  gateway stages are instrumented; capture-to-native-publication timing is not
  established by the older ROS `camera.acquired` hook. That hook covers the
  remaining vendor depth path. A missing start boundary stays unmeasured.
- Vendor command publication is not motor acknowledgment or physical movement.
- Hosted-provider, headset and sustained R1 qualification must run on the released
  combination. Prior synthetic results do not qualify this rebased candidate.
- The previous overhead experiment did not conclusively meet the additional
  command-age budget; repeat matched workloads with usable clock bounds.

Do not turn missing measurements into zero latency, infer a physical hop from a
logical edge, or call a successful fixture a production teleoperation result.
