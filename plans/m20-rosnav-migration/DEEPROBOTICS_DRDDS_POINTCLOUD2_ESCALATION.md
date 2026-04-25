# Deep Robotics drdds PointCloud2 receive issue after M20 V1.1.8.5 OTA

Date: 2026-04-25

## Executive summary

After upgrading an M20 to Deep Robotics system software V1.1.8.5, our
PointCloud2 consumer on the NOS board could no longer receive lidar
clouds from `rsdriver` through the documented/visible `libdrdds`
subscriber APIs.

The failure is specific:

- `rsdriver` continues to publish valid PointCloud2 data.
- DDS endpoint discovery succeeds.
- `DrDDSSubscriber` and `DrDDSChannel` report matched endpoints.
- No PointCloud2 samples are delivered to the libdrdds callback.
- A raw FastDDS subscriber using the same generated DR IDL type receives
  the same stream correctly when built-in transports are disabled and an
  explicit UDPv4 transport is configured.

This strongly suggests that the post-OTA `rsdriver` / `libdrdds` /
FastDDS stack is selecting a built-in local DataSharing/shared-memory
transport path for PointCloud2 that discovers endpoints but does not
deliver large PointCloud2 samples to a normal `DrDDSSubscriber` or
`DrDDSChannel`.

Our production workaround is to bypass `libdrdds` for sensor receive and
subscribe directly with raw FastDDS on explicit UDPv4.

## What we need from Deep Robotics

1. Confirm whether V1.1.8.5 changed `rsdriver`, `libdrdds`,
   `drqos.xml`, FastDDS transport defaults, or PointCloud2 receive
   expectations.
2. Provide the supported way for third-party processes to receive
   `/LIDAR/POINTS` and `/LIDAR/POINTS2` PointCloud2 from `rsdriver`
   through `libdrdds`.
3. Confirm whether PointCloud2 consumers are expected to disable
   FastDDS built-in transports or disable local DataSharing/SHM.
4. Confirm whether the `use_shm` argument to `DrDDSSubscriber` /
   `DrDDSChannel` is sufficient for PointCloud2 after V1.1.8.5, or
   whether there is another required API/configuration step.
5. If this is a `libdrdds` bug, provide a patched `libdrdds` or a
   documented transport override for large PointCloud2 streams.

## System context

Robot: Deep Robotics M20

System software: V1.1.8.5 OTA

Board under test: NOS

Publisher: `rsdriver` / `rslidar`

Affected topics:

- `/LIDAR/POINTS` -> DDS topic `rt/LIDAR/POINTS`
- `/LIDAR/POINTS2` -> DDS topic `rt/LIDAR/POINTS2`

Message type:

- `sensor_msgs::msg::dds_::PointCloud2_`

Current installed artifacts:

```text
e476b1f6b39a372b5be9114341189fdc7f785ae16eada95a9299ae019ce72d8c  /usr/local/lib/libdrdds.so.1.1.7
0c0dec4e719aab80709984947bc250abfd352bef8aa1d5009f085906f89e547b  /usr/local/lib/libfastrtps.so.2.14.2
b36b17aa799b15f31dd60f2fb1862b0e01f360b6bee1d877b94bfe0e155f4f56  /usr/local/lib/libfastcdr.so.2.2.5
cb2dc759da63b41bb614be9d33eabca6d5665e52cd996b53f1b036dc3fb00b4e  /opt/drdds/dr_qos/drqos.xml
f036085a5ed60b1e89309fd1e54be1e985262a400f491f4a5a4040b4c78b589c  /opt/robot/share/node_driver/bin/rslidar
```

The current `rslidar` binary still contains the expected topic strings:

```text
/LIDAR/IMU201
/LIDAR/IMU202
/LIDAR/POINTS
/LIDAR/POINTS2
/LIDAR/STATUS
```

It also still contains the `eth0/eth1` network string used for
`DrDDSManager::Init`.

## Important pre/post comparison

We have a pre-OTA backup from immediately before the V1.1.8.5 upgrade.
That backup captured:

- `rsdriver_config.yaml`
- process/network state
- `/dev/shm/fastrtps_*` state
- systemd unit/drop-in state

It did not capture:

- `/usr/local/lib/libdrdds.so*`
- `/opt/drdds/dr_qos/drqos.xml`
- `/opt/robot/share/node_driver/bin/rslidar`

The restored current `rsdriver_config.yaml` is byte-for-byte identical
to the pre-OTA backup. So this is not explained by a lingering
`rsdriver` config difference.

The active config still includes the intended dual-lidar settings:

```yaml
send_separately: true
lidar_type: RSAIRY
use_lidar_clock: true
dense_points: true
ts_first_point: false
dds_send_point_cloud_topic: /lidar_points
```

Note: although the YAML says `/lidar_points`, the current `rslidar`
binary publishes the hardcoded DR topics `/LIDAR/POINTS` and
`/LIDAR/POINTS2`, with FastDDS DDS topic names `rt/LIDAR/POINTS` and
`rt/LIDAR/POINTS2`.

## Observed failure

Using `libdrdds` after the OTA:

- `DrDDSSubscriber<sensor_msgs::msg::PointCloud2PubSubType>` matches
  the publisher.
- `DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>` matches the
  publisher.
- Neither receives samples.
- The callback is never invoked.

Representative output from a minimal libdrdds probe:

```text
[drdds_probe] Init domain=0 network=eth0/eth1 mode=channel topic=/LIDAR/POINTS use_shm=false pub_use_shm=false prefix=rt seconds=20
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] done msgs=0
```

The same symptom occurs with plain `DrDDSSubscriber`:

```text
[drdds_probe] Init domain=0 network=eth0/eth1 mode=subscriber topic=/LIDAR/POINTS use_shm=false pub_use_shm=false prefix=rt seconds=15
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] done msgs=0
```

Enabling `use_shm=true` does not fix it:

```text
[drdds_probe] Init domain=0 network=eth0/eth1 mode=subscriber topic=/LIDAR/POINTS use_shm=true pub_use_shm=false prefix=rt seconds=10
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] status matched=2 pub_matched=-1 msgs=0
[drdds_probe] done msgs=0
```

Using an empty topic prefix does not match, confirming that the active
DDS topic is `rt/LIDAR/POINTS`:

```text
[drdds_probe] Init domain=0 network=eth0/eth1 mode=subscriber topic=/LIDAR/POINTS use_shm=false pub_use_shm=false prefix=<empty> seconds=10
[drdds_probe] status matched=0 pub_matched=-1 msgs=0
[drdds_probe] status matched=0 pub_matched=-1 msgs=0
[drdds_probe] done msgs=0
```

## Raw FastDDS behavior

Raw FastDDS with built-in transports has the same failure:

```text
[fastdds_pc2_probe] domain=0 topic=rt/LIDAR/POINTS type=sensor_msgs::msg::dds_::PointCloud2_ reliability=BEST_EFFORT custom_udp=false keep_builtin=false interface=<all> recv_buffer=8388608 custom_shm=false data_sharing=auto seconds=10
[fastdds_pc2_probe] matched=1
[fastdds_pc2_probe] matched=2
[fastdds_pc2_probe] status matched=2 msgs=0
[fastdds_pc2_probe] status matched=2 msgs=0
[fastdds_pc2_probe] done msgs=0
```

Raw FastDDS works when built-in transports are disabled and explicit
UDPv4 is configured:

```text
[fastdds_pc2_probe] domain=0 topic=rt/LIDAR/POINTS type=sensor_msgs::msg::dds_::PointCloud2_ reliability=BEST_EFFORT custom_udp=true keep_builtin=false interface=<all> recv_buffer=8388608 custom_shm=false data_sharing=auto seconds=10
[fastdds_pc2_probe] matched=1
[fastdds_pc2_probe] matched=2
[fastdds_pc2_probe] msg #1 stamp=1777093036.399991274 width=34641 height=1 point_step=26 bytes=900666
[fastdds_pc2_probe] msg #2 stamp=1777093036.499986172 width=34903 height=1 point_step=26 bytes=907478
[fastdds_pc2_probe] msg #3 stamp=1777093036.599982262 width=34755 height=1 point_step=26 bytes=903630
[fastdds_pc2_probe] msg #4 stamp=1777093036.699985266 width=34786 height=1 point_step=26 bytes=904436
[fastdds_pc2_probe] msg #5 stamp=1777093036.799975157 width=34631 height=1 point_step=26 bytes=900406
[fastdds_pc2_probe] msg #10 stamp=1777093037.299981117 width=34737 height=1 point_step=26 bytes=903162
[fastdds_pc2_probe] msg #20 stamp=1777093038.299987078 width=34902 height=1 point_step=26 bytes=907452
[fastdds_pc2_probe] status matched=2 msgs=20
[fastdds_pc2_probe] msg #30 stamp=1777093039.299987078 width=34728 height=1 point_step=26 bytes=902928
[fastdds_pc2_probe] msg #40 stamp=1777093040.299985170 width=34777 height=1 point_step=26 bytes=904202
[fastdds_pc2_probe] msg #50 stamp=1777093041.299983263 width=34612 height=1 point_step=26 bytes=899912
[fastdds_pc2_probe] msg #60 stamp=1777093042.299984217 width=34733 height=1 point_step=26 bytes=903058
[fastdds_pc2_probe] msg #70 stamp=1777093043.299988270 width=34973 height=1 point_step=26 bytes=909298
[fastdds_pc2_probe] status matched=2 msgs=70
[fastdds_pc2_probe] done msgs=70
```

Raw explicit UDP also works with a RELIABLE reader:

```text
[fastdds_pc2_probe] domain=0 topic=rt/LIDAR/POINTS type=sensor_msgs::msg::dds_::PointCloud2_ reliability=RELIABLE custom_udp=true keep_builtin=false interface=<all> recv_buffer=268435456 custom_shm=false data_sharing=auto seconds=15
[fastdds_pc2_probe] matched=1
[fastdds_pc2_probe] matched=2
[fastdds_pc2_probe] msg #1 stamp=1777092933.999992132 width=34788 height=1 point_step=26 bytes=904488
...
[fastdds_pc2_probe] status matched=2 msgs=148
[fastdds_pc2_probe] done msgs=148
```

Raw explicit UDP fails again if built-in transports are kept enabled:

```text
[fastdds_pc2_probe] domain=0 topic=rt/LIDAR/POINTS type=sensor_msgs::msg::dds_::PointCloud2_ reliability=BEST_EFFORT custom_udp=true keep_builtin=true interface=<all> recv_buffer=8388608 custom_shm=false data_sharing=auto seconds=10
[fastdds_pc2_probe] matched=1
[fastdds_pc2_probe] matched=2
[fastdds_pc2_probe] status matched=2 msgs=0
[fastdds_pc2_probe] status matched=2 msgs=0
[fastdds_pc2_probe] done msgs=0
```

That is the key observation: the same reader and same publisher work
only when built-in transports are disabled. Keeping built-ins enabled
causes endpoint matching without sample delivery.

## Probe matrix

All rows target live `/LIDAR/POINTS` from the same `rsdriver` instance.

| Probe | Transport/QoS | Result |
| --- | --- | --- |
| `drdds_probe --mode channel --use-shm 0` | libdrdds `DrDDSChannel`, prefix `rt` | `matched=2`, `msgs=0` for 20 s |
| `drdds_probe --mode subscriber --use-shm 0` | libdrdds `DrDDSSubscriber`, prefix `rt` | `matched=2`, `msgs=0` for 15 s |
| `drdds_probe --mode subscriber --use-shm 1` | libdrdds subscriber with SHM enabled | `matched=2`, `msgs=0` for 10 s |
| `drdds_probe --mode subscriber --prefix __empty__` | libdrdds, no `rt` prefix | `matched=0`, `msgs=0` |
| `fastdds_pc2_probe` | raw FastDDS, built-in transports | `matched=2`, `msgs=0` |
| `fastdds_pc2_probe --custom-udp 1 --recv-buffer 8388608` | raw FastDDS, built-ins disabled, explicit UDP | `matched=2`, `msgs=70` in 10 s |
| `fastdds_pc2_probe --custom-udp 1 --recv-buffer 268435456` | raw FastDDS, built-ins disabled, explicit UDP | `matched=2`, `msgs=130` in 15 s |
| `fastdds_pc2_probe --custom-udp 1 --keep-builtin 1` | raw FastDDS, explicit UDP plus built-ins kept | `matched=2`, `msgs=0` |
| `fastdds_pc2_probe --custom-udp 1 --reliable 1` | raw FastDDS, explicit UDP, RELIABLE reader | `matched=2`, `msgs=148` in 15 s |

## Minimal code shapes

### libdrdds subscriber shape that matches but receives zero

```cpp
DrDDSManager::Init(0, "eth0/eth1");

DrDDSSubscriber<sensor_msgs::msg::PointCloud2PubSubType> sub(
    callback,
    "/LIDAR/POINTS",
    0,
    "rt",
    false  // use_shm; true was also tested and still received zero
);
```

### raw FastDDS shape that receives correctly

```cpp
using namespace eprosima::fastdds::dds;
using eprosima::fastdds::rtps::UDPv4TransportDescriptor;

DomainParticipantQos pqos;
pqos.name("pc2_raw_probe");
pqos.transport().use_builtin_transports = false;

auto udp = std::make_shared<UDPv4TransportDescriptor>();
udp->maxMessageSize = 65500;
udp->receiveBufferSize = 8 * 1024 * 1024;
udp->sendBufferSize = 8 * 1024 * 1024;
udp->non_blocking_send = false;
pqos.transport().user_transports.push_back(udp);

auto* participant =
    DomainParticipantFactory::get_instance()->create_participant(0, pqos);

TypeSupport pc2_type(new sensor_msgs::msg::PointCloud2PubSubType());
pc2_type->setName("sensor_msgs::msg::dds_::PointCloud2_");
pc2_type.register_type(participant);

auto* topic = participant->create_topic(
    "rt/LIDAR/POINTS",
    pc2_type->getName(),
    TOPIC_QOS_DEFAULT);

DataReaderQos rqos = DATAREADER_QOS_DEFAULT;
rqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
rqos.durability().kind = VOLATILE_DURABILITY_QOS;
rqos.history().kind = KEEP_LAST_HISTORY_QOS;
rqos.history().depth = 8;

auto* reader = subscriber->create_datareader(topic, rqos, &listener);
```

The same explicit-UDP approach also worked with
`RELIABLE_RELIABILITY_QOS`.

## What we ruled out

### Topic name or prefix mismatch

Ruled out.

`rt/LIDAR/POINTS` matches. Empty prefix does not match. The current
`rslidar` binary contains `/LIDAR/POINTS` and `/LIDAR/POINTS2`, and
the active DDS topic name is the usual `rt/`-prefixed name.

### PointCloud2 type mismatch

Ruled out.

Raw FastDDS decodes full `sensor_msgs::msg::dds_::PointCloud2_` frames
using the same generated DR IDL type. The frames contain normal sizes:

- front lidar: roughly 34k to 35k points, roughly 0.9 MB per frame
- rear lidar: roughly 47k to 48k points, roughly 1.23 MB per frame

### RELIABLE vs BEST_EFFORT mismatch

Ruled out.

Raw explicit UDP receives with both BEST_EFFORT and RELIABLE readers.

### Socket receive buffer size

Mostly ruled out.

Explicit UDP works with an 8 MiB receive buffer. We still use a larger
buffer in production for margin, but the failure reproduces based on
built-in transport selection, not buffer size.

### rsdriver YAML config drift

Ruled out for the restored current system.

The current `rsdriver_config.yaml` is byte-for-byte identical to the
pre-OTA backup.

## Current production workaround

We replaced the libdrdds receive path in our bridge with a forked raw
FastDDS child process that subscribes to all sensor topics using explicit
UDPv4 and built-in transports disabled:

- `rt/LIDAR/POINTS`
- `rt/LIDAR/POINTS2`
- `rt/IMU`
- `rt/LIDAR/IMU201`
- `rt/LIDAR/IMU202`

The bridge then writes the data into our existing POSIX shared-memory
buffers for downstream consumers.

Verified result:

```text
[drdds_recv] status: lidar_front_matched=2 lidar_front_msgs=241 lidar_rear_matched=1 lidar_rear_msgs=240
[drdds_recv] imu_status: imu_yesense_matched=1 imu_yesense_msgs=4665 imu_airy_front_matched=3 imu_airy_front_msgs=9375 imu_airy_rear_matched=3 imu_airy_rear_msgs=9375
...
[drdds_recv] status: lidar_front_matched=2 lidar_front_msgs=941 lidar_rear_matched=1 lidar_rear_msgs=940
[drdds_recv] imu_status: imu_yesense_matched=1 imu_yesense_msgs=18666 imu_airy_front_matched=3 imu_airy_front_msgs=37377 imu_airy_rear_matched=3 imu_airy_rear_msgs=37377
```

This is steady front/rear PointCloud2 at approximately 10 Hz, Yesense
IMU at approximately 200 Hz, and Airy IMUs at approximately 400 Hz as
seen by the receive callbacks.

We also ran our native navigation stack in a dock-safe no-motion mode.
FAST-LIO consumed the PC2 path correctly:

```text
[fastlio2] stationary preroll complete after 2.00s - releasing lidar scans to FAST-LIO
[fastlio2] frame #0  path=pc2 ... npts=31646
...
[fastlio2] frame #29  path=pc2 ... npts=34088
[drdds_bridge] lidar #301 pts=34263 bytes=890838
[nav_cmd_pub] #351 x=0.000 y=0.000 yaw=0.000 matched=1
```

## Current root-cause hypothesis

After V1.1.8.5, Deep Robotics' `rsdriver` / `libdrdds` stack advertises
matching PointCloud2 endpoints, but the default FastDDS/libdrdds receive
path selects a local built-in/DataSharing/SHM transport that never
delivers large PointCloud2 samples to a normal `DrDDSSubscriber` or
`DrDDSChannel`.

The evidence is that:

- built-in transports enabled -> `matched=2`, `msgs=0`
- built-in transports disabled and explicit UDPv4 configured ->
  full PointCloud2 samples are delivered at 10 Hz
- adding explicit UDP while keeping built-ins enabled still fails

This points specifically to built-in transport selection or local
DataSharing/SHM behavior, not to topic naming, IDL type support,
reliability QoS, or buffer size.

## Requested Deep Robotics investigation

Please check the following in V1.1.8.5:

1. Does `rsdriver` publish PointCloud2 with any DataSharing/SHM-only
   expectation that a normal `DrDDSSubscriber` no longer satisfies?
2. Does `libdrdds` force or prefer FastDDS built-in transports for local
   readers in a way that can match but fail to receive large samples?
3. Is `DrDDSSubscriber(..., use_shm=true)` expected to join the same
   transport path as `rsdriver` for PointCloud2?
4. Is there an undocumented API that production services use to attach
   to `rsdriver`'s shared-memory/DataSharing path?
5. Should `libdrdds` expose a way to disable built-in transports or
   force UDP transport for a subscriber?
6. Was there an ABI/API change in the `DrDDSChannel` constructor or
   transport defaults around the March 2026 `libdrdds.so.1.1.7` build?

## Suggested fix options

Any of the following would make this easier to support:

1. Make normal `DrDDSSubscriber<PointCloud2PubSubType>` work for
   `/LIDAR/POINTS` and `/LIDAR/POINTS2` after V1.1.8.5.
2. Document the required `libdrdds` initialization and constructor flags
   for PointCloud2 consumers.
3. Provide a `libdrdds` transport option equivalent to:
   `use_builtin_transports=false` plus explicit UDPv4.
4. Provide a supported example PointCloud2 subscriber for `rsdriver`
   on M20 V1.1.8.5.
5. Clarify whether local SHM/DataSharing is supported for third-party
   PointCloud2 consumers, and if so, what API/configuration is required.
