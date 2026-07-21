# M20 drdds → Zenoh bridge

Onboard bridge that republishes the DeepRobotics M20's Fast-DDS fork ("drdds")
LIO/localization topics onto dimos's **Zenoh** transport, LCM-encoded, so the
rest of dimos consumes them as ordinary typed messages.

It's the reliable-transport twin of the sibling LCM bridge
(`../../dds/cpp/main.cpp`): identical per-sample drdds → `dimos_lcm` conversions,
but the carrier is **reliable Zenoh unicast** (auto-discovered, retransmits at
each hop) instead of LCM udpm multicast. The dense localization clouds
(`/ALIGNED_POINTS`, `/grid_map_3d`) lose ~87% over multicast on the M20's
NOS→GEN path; over Zenoh they arrive losslessly at line-rate gigabit.

## Wire format (matches dimos `zenohpubsub.Zenoh`)

- **Key:** `dimos/<name>/<pkg.Type>` (e.g. `dimos/aligned_points/sensor_msgs.PointCloud2`).
  The NativeModule passes an LCM channel `<topic>#<type>`; we swap `#`→`/`.
- **Payload:** LCM-encoded bytes from the header-only `cpp_lcm_msgs` types — same
  `.lcm` source as dimos's Python types, so fingerprints match and
  `lcm_decode()` round-trips. No `liblcm` is linked (that was only the udpm transport).

## Build (on a robot box — needs the drdds SDK + zenoh-c)

```sh
cd cpp && ./build.sh        # cmake -B build && cmake --build build -j
```

The boxes have no clean internet; stage a local dimos-lcm checkout at
`/tmp/dimos-lcm` and `build.sh` feeds it to FetchContent (else it clones GitHub).
zenoh-c (`libzenohc.so` + headers, v1.5.x to match the consumer) must be in
`/usr/local`.

## Run (on NOS, as root for SHM access)

```sh
sudo ./build/m20_drdds_zenoh_bridge \
    --aligned   'dimos/slam_aligned_points#sensor_msgs.PointCloud2' --aligned_topic /ALIGNED_POINTS \
    --grid      'dimos/grid_map_3d#sensor_msgs.PointCloud2'         --grid_topic    /grid_map_3d \
    --odometry  'dimos/slam_odom#nav_msgs.Odometry'                 --odom_topic    /ODOM \
    --iface eth1 --domain 0
```

Ports (all optional; wire only what you pass): `--lidar`, `--aligned`, `--grid`,
`--locbody` (PointCloud2), `--imu` (Imu), `--odometry` (Odometry). Each takes the
dimos channel string; the drdds source defaults to the ROS name and is overridable
via `--<port>_topic` (odometry uses `--odom_topic`). `--iface` pins the multicast
scout NIC (eth1 = NOS .31 segment); `--shm` (default true) uses the SHM transport
needed for SHM-only writers like `/ALIGNED_POINTS`.

`--connect <eps>` adds explicit, comma-separated router endpoint(s) (e.g.
`tcp/10.21.31.103:7447,tcp/10.21.33.103:7447`). Use this when multicast scouting
can't reach the router — e.g. the NOS↔AOS L2 segment drops multicast (no IGMP
querier), so the bridge never auto-discovers the router even though TCP works.
Multicast scouting still runs alongside, so the flag is purely additive.

## Verify (on GEN, in the dimos venv)

```sh
dimos spy --transport zenoh -n --duration 7 --interval 1
```
The bridged keys should appear with their freq/bandwidth.
