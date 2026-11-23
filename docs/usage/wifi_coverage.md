# Wi-Fi coverage mapping

`WifiMonitor` observes a Linux Wi-Fi interface. `WifiCoverage` joins those
observations to timestamped robot odometry and publishes a coloured point cloud
at the **receiver's locations**. It does not estimate access-point positions.
The two modules are independent of any robot, SLAM implementation, or viewer.

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
