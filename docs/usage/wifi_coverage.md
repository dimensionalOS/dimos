# Wi-Fi coverage mapping

`WifiMonitor` observes a Linux Wi-Fi interface. `WifiCoverage` joins those
observations to timestamped robot odometry and publishes a coloured point cloud
at the **receiver's locations**. It does not estimate access-point positions.
The two modules are independent of any robot, SLAM implementation, or viewer.

With runtime diagnostics enabled, the existing radio and performance samples are
also retained in that run's bounded incident bundle. This reuses the measurements;
it does not start another sampler. The map and its survey archive work independently
when diagnostics is off. See [runtime diagnostics](/docs/usage/diagnostics.md).

For a **performance survey**, add `NetworkSurvey` through the composition helper.
This is the same coverage tool: one point-cloud output with selectable radio,
network, host and optional viewer metrics, plus a local evidence file.

```python
from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.wifi.blueprints import wifi_survey

survey = autoconnect(
    your_mapping_blueprint,
    wifi_survey(
        interface=wifi_interface,
        targets={"router": router_ip, "viewer": viewer_ip},
        record_path=STATE_DIR / "wifi-surveys" / "office-01.jsonl",
        desktop_diagnostics_url=desktop_diagnostics_url,  # optional
    ),
)
```

Set the deployment's interface, literal probe IPs and optional full diagnostics
URL (`/apps/dim-spatial-desktop/api/diagnostics`). Use a new evidence filename for
each survey; existing files are never overwritten. The sampler runs on the robot;
the optional Desktop endpoint and gateway must use that robot's clock domain.
It reads existing viewer telemetry and never subscribes to camera pixels.

`NetworkSurvey` measures interface TX/RX Mbps, host CPU percent and one ICMP
probe per configured target per polling cycle. These are small probes, not a
throughput benchmark. `probe_loss_pct` measures unanswered ICMP probes; execution
errors are recorded separately. A target that blocks ICMP is not evidence of a
broken Wi-Fi link. Linux `iputils` ping permissions are required for probing.

In `WifiCoverage`'s module RPC panel (or any RPC client):

1. `get_layers()` lists measured metric/source pairs. Each viewer, camera and probe
   target stays separate. No viewer measurements means no invented video layer.
2. `select_metric(metric="rtt_ms", source="ping/router", good=10, bad=200, statistic="p95")` changes
   `wifi_coverage` to a latency overlay. The endpoints are your display scale,
   not an asserted SLA. A source-FPS layer can use `good=30, bad=0`.
3. `get_cells()` gives grid coordinates, sample counts, mean, p50/p95, range,
   timestamps and observed BSSIDs. The cloud displays the selected statistic
   (`mean`, `p50`, `p95`, `min` or `max`); RSSI uses mean.
4. `get_samples(cell=[x, y, z])` returns the raw timeline for that grid cell;
   omit `cell` to also see unlocated samples and their rejection reasons.
5. `select_access_point(bssid)` filters the layer by observed association.
   `select_metric(metric="rssi_dbm", source="", good=-40, bad=-90)` restores RSSI.

`get_status()` identifies the active layer, scale, epoch and archive destination.
`NetworkSurvey.get_status()` separately reports sampler health and the age of its
last published batch; an open archive does not prove the probes are still running.
Probe timeouts remain explicit errors and do not stop subsequent polling cycles.

Wire the survey to the same pose estimator used by the map. A wheel-odometry stream
and lidar odometry can both be labelled `odom` while describing different coordinate
systems; matching frame names alone does not align them. Use an explicit remapping
when a blueprint exposes multiple odometry sources.

The JSONL evidence contains pose records, Wi-Fi observations and located performance
samples (including rejected/missing measurements), grouped by map epoch. It is
written on the robot and survives a viewer disconnect; it contains no camera
images. The live performance map retains the newest `max_samples` samples;
cell statistics describe those retained samples, with counts exposed. The file
preserves the full survey. Restart/reset or a frame-ID change begins a new epoch.

Viewer metrics distinguish `application_rtt_ms` from `rtc_rtt_ms` and ICMP
`rtt_ms`. Source/sent/received/decoded/displayed FPS are separate layers;
`last_displayed_ago_ms` is time since a frame was shown, **not** capture-to-display
latency. `report_arrival_age_ms` exposes missing viewer telemetry. Hidden tabs
remain labelled hidden in the evidence; do not compare their display FPS to an
active headset. Other applications can publish `SurveyMetrics` on `survey_metrics`
without the Desktop adapter, using robot-clock timestamps and explicit units.

Browser timestamps use the gateway's clock-offset estimate, with half its clock
exchange RTT retained as timing uncertainty. Missing clock estimates, stale data,
missing poses, or measurement windows spanning different grid cells stay in the
evidence but are excluded from the overlay. This conservative join avoids painting
a slow, old report at the robot's new location. Stop for 10–15 seconds per survey
location and inspect sample counts before interpreting percentiles. Keep stream
settings consistent and retain actual traffic measurements alongside latency.

Compose them with a blueprint that publishes `odometry: Out[Odometry]`:

```python
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.wifi.module import WifiMonitor
from dimos.mapping.wifi.module import WifiCoverage

# Set the interface for the deployment; do not assume every robot uses wlan0.
with_coverage = autoconnect(
    your_mapping_blueprint,
    WifiMonitor.blueprint(interface="wlan0"),
    WifiCoverage.blueprint(cell_size=0.5),
)
```

Use blueprint remappings if the robot's odometry stream has another name. Its
pose must describe the robot in the map frame you want to overlay. Synchronize
clocks when acquisition and odometry run on different machines.

## Observation and survey modes

Install Linux `iw`. By default the monitor polls once per second, reading the
connected radio and the existing scan cache. It **does not trigger scans** or
change association, channel, power settings, or network configuration. Disconnected
and unavailable measurements do not become zero-strength observations.

Neighbour observations older than five seconds, or without an age, are excluded.
Their measurement timestamps are preserved: a cached reading is mapped at the
robot's earlier position, not where it happens to be when the cache is read.
This matters because driver caches can retain readings for many minutes.

To survey all visible APs, explicitly set `scan_interval_s=30` on `WifiMonitor`.
Scan triggering requires `CAP_NET_ADMIN` or an appropriately privileged deployment.
Off-channel scans can interrupt Wi-Fi traffic; use this mode for a deliberate
survey, not as an unnoticed addition to latency-sensitive teleoperation.
Coverage includes only radios the interface actually observes, not every AP in
the building. Failures are logged; missing measurements remain missing.

## Streams and display

- `wifi_scan: WifiScan` contains timestamped BSSID, SSID, frequency, RSSI, connected
  status and negotiated transmit/receive PHY rates. It uses DimOS's CBOR transport.
- `wifi_coverage: PointCloud2` is an ordinary map layer, usable by Spatial Desktop
  and other point-cloud viewers. Red is −90 dBm, yellow −65 dBm, green −40 dBm;
  values outside this display range are clipped. This scale is not a latency SLA.
- `get_access_points()` lists BSSIDs. Radios sharing an SSID remain separate.
- `select_access_point(bssid)` selects one radio; an empty BSSID shows the strongest
  observed mean RSSI per cell. This combines historical observations, not a prediction
  of which AP a roaming client will choose now.
- `get_cells()` returns the selected cells with sample count, mean/min/max RSSI
  and last-observed timestamp, for inspection or export.

The map interpolates bracketing poses or uses a pose within `max_pose_age_s`
(default 0.5 seconds). It rejects observations without a sufficiently close pose,
deduplicates cached measurements, and retains at most `max_cells` AP/cell entries
(default 10,000, least-recently-updated eviction). Unvisited space stays unknown.
Frame-ID changes clear the map. Call `reset()` after relocalization or a new map
epoch that reuses the same frame ID. State is in memory; restart clears it.

RSSI and PHY rate do not measure available throughput, retries, queueing, or
application latency. Correlate this layer with network and processing diagnostics;
a green coverage cell alone cannot establish that teleoperation will work well.
