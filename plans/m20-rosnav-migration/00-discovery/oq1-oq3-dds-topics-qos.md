# OQ1 & OQ3: DDS Topics and QoS on M20 NOS

**Date:** 2026-03-13
**Inspected by:** polecat nux (di-daa85u.6.1)
**Method:** SSH to NOS (10.21.31.106) and AOS (10.21.31.103), `ros2 topic info -v`, rsdriver config, drdds QoS XML

---

## OQ1: rsdriver DDS QoS for /lidar_points

### Topic Name Correction

The spec assumes the topic name is `/lidar_points` (lowercase). **The actual DDS topic is `/LIDAR/POINTS` (uppercase).**

- rsdriver config (`/opt/robot/share/node_driver/config/config.yaml` on AOS) specifies `dds_send_point_cloud_topic: /lidar_points`
- However, the topic visible via ROS2 discovery is `/LIDAR/POINTS`
- lio_perception config confirms: `input_lidar_topic: "/LIDAR/POINTS"` with comment: *drdds (RoboSense) uses "/LIDAR/POINTS"*
- The drdds middleware maps lowercase config names to uppercase DDS topic names

**Action required:** Update `M20ROSNavConfig.lidar_topic` from `/lidar_points` to `/LIDAR/POINTS`.

### QoS Profile (from `ros2 topic info -v`)

| Property | Value |
|----------|-------|
| Type | `sensor_msgs/msg/PointCloud2` |
| Reliability | **RELIABLE** |
| Durability | **VOLATILE** |
| Lifespan | infinite (default) |
| Deadline | infinite (default) |
| Liveliness | AUTOMATIC |
| History | KEEP_LAST (default) |

### drdds-level QoS (from `/opt/drdds/dr_qos/drqos.xml`)

The drdds QoS config sets a 50ms deadline on PointCloud2 data writers/readers:

```xml
<data_writer profile_name="sensor_msgs::msg::dds_::PointCloud2_">
    <qos>
        <reliability><kind>RELIABLE</kind></reliability>
        <deadline><period><sec>0</sec><nanosec>50000000</nanosec></period></deadline>
    </qos>
</data_writer>
```

This 50ms deadline (20Hz) is set at the drdds layer but does **not** appear in the ROS2-visible endpoint QoS (which shows infinite deadline). The container's FASTLIO2 subscriber should use RELIABLE/VOLATILE without a deadline constraint.

### Publisher Details

rsdriver publishes via bare DDS (drdds), not as a ROS2 node. Publisher count shows **0** from the ROS2 perspective on both NOS and AOS. Two bare-DDS subscribers exist (likely lio_perception instances).

### rsdriver Configuration (AOS)

| Parameter | Value |
|-----------|-------|
| Lidar type | RSAIRY (x2) |
| Front multicast | 224.10.10.201:6691 |
| Back multicast | 224.10.10.202:6692 |
| send_separately | **false** (merged dual-lidar output) |
| use_lidar_clock | true |
| dense_points | true (NaN discarded) |
| ts_first_point | true |
| min/max distance | 0.2m / 60m |
| Front IMU port | 6681 |
| Back IMU port | 6682 |

### FASTLIO2 Subscriber Recommendation

```python
QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    liveliness=QoSLivelinessPolicy.AUTOMATIC,
)
```

Subscribe to topic: `/LIDAR/POINTS`

---

## OQ3: IMU Topic Format

### Answer: Standard `sensor_msgs/msg/Imu`

**The `/IMU` topic uses standard `sensor_msgs/msg/Imu`.** FASTLIO2 can subscribe directly without any type conversion.

### QoS Profile (from `ros2 topic info -v`)

| Property | Value |
|----------|-------|
| Type | **`sensor_msgs/msg/Imu`** |
| Reliability | **RELIABLE** |
| Durability | **VOLATILE** |
| Lifespan | infinite (default) |
| Deadline | infinite (default) |
| Liveliness | AUTOMATIC |

### Other IMU-Related Topics

| Topic | Type | Notes |
|-------|------|-------|
| `/IMU` | `sensor_msgs/msg/Imu` | **Use this for FASTLIO2** — standard type, 200Hz |
| `/IMU_DATA` | `drdds/msg/ImuData` | Custom drdds type — NOT compatible with FASTLIO2 |
| `/IMU_DATA_10HZ` | (likely drdds/msg/ImuData) | Low-rate variant |
| `/IMU_YESENSE` | (sensor_msgs/msg/Imu) | Yesense IMU — used by lio_perception |

**Note:** lio_perception subscribes to `/IMU_YESENSE`, not `/IMU`. The FASTLIO2 container should subscribe to `/IMU` (200Hz from AOS).

### FASTLIO2 Subscriber Recommendation

```python
QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=200,  # Buffer ~1s at 200Hz
    liveliness=QoSLivelinessPolicy.AUTOMATIC,
)
```

Subscribe to topic: `/IMU`

---

## /NAV_CMD QoS (bonus)

| Property | Value |
|----------|-------|
| Type | `drdds/msg/NavCmd` |
| Reliability | RELIABLE |
| Durability | VOLATILE |
| Publishers | 1 (bare DDS) |
| Subscribers | 1 (bare DDS) |

---

## Critical Finding: DDS Domain ID Conflict

All topics are currently on **domain 0** (default). The spec contains conflicting guidance:

1. Design section: *"DDS domain ID: Run FASTLIO2 container on domain 0 to match rsdriver."*
2. Critical gaps section: *"DDS Domain: Container on domain 42 to isolate from rsdriver (domain 0)"*
3. M20ROSNavConfig: `ROS_DOMAIN_ID=42`

**If the container runs on domain 42, it will NOT discover rsdriver topics on domain 0.** Standard DDS discovery does not cross domain boundaries.

**Recommendation:** Use domain 0 for the FASTLIO2 container. Isolation from the Foxy stack can be achieved through topic namespacing or DDS partition QoS rather than domain separation. Alternatively, run a domain bridge — but this adds complexity and latency for high-bandwidth point cloud data.

---

## NOS Software Inventory

| Package | Version |
|---------|---------|
| OS | Ubuntu 20.04 (arm64/RK3588) |
| ROS2 | Foxy (`/opt/ros/foxy/`) |
| drdds-ros2-msgs | 1.0.4 |
| drddslib | 1.1.4 |
| fastdds CLI | `/usr/local/bin/fastdds` |

### NOS FastDDS Config (`/opt/robot/fastdds.xml`)

- Transport: localhost-only UDP + SHM (500MB segment)
- Discovery: SIMPLE protocol
- Built-in transports: disabled (custom only)

---

## Full Topic List (NOS, domain 0)

```
/ACCUMULATED_POINTS_MAP    /ALIGNED_POINTS       /BATTERY_DATA
/CANCEL_NAV                /CHARGE_CMD           /CHARGE_STATUS
/CLOUD_REGISTERED_BODY     /CPU_103              /CPU_104
/CPU_106                   /DEPTH_IMAGE          /DEPTH_POINTS
/FAULT_STATUS              /GAIT                 /GOAL
/GPS                       /HANDLE_STEER         /HEIGHT_IMAGE
/HEIGHT_MAP_STATUS         /HES_STATUS           /IMU
/IMU_DATA                  /IMU_DATA_10HZ        /IMU_YESENSE
/JOINTS_CMD                /JOINTS_DATA          /JOINTS_DATA_10HZ
/LED/STATUS                /LIDAR/POINTS         /LIDAR/STATUS
/LIO_ODOM_HIGH_FREQUENCY   /LOCATION_STATUS      /MOTION_INFO
/MOTION_STATE              /MOTION_STATUS        /NAV_CMD
/ODOM                      /OOA_STATUS           /PLANNER_STATUS
/REAL_STEER                /SEG_CLOUD            /STEER
/TERRAIN_CLASSIFIER_STATUS /TRACK_PATH           /initialpose
/parameter_events          /pose_in_apriltag_corrected  /rosout
/tag_status
```

### Additional AOS-only Topics

```
/HANDLER_POINTS_DEBUG  /LIO_ODOM   /WEIGHT_ITEMS
/cloud_nav             /free_paths /path       /tf
```
