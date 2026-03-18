# drdds-to-ROS2 Lidar Bridge Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a C++ bridge node that receives `/LIDAR/POINTS` from the M20's drdds rsdriver and republishes it as a standard ROS2 Humble PointCloud2 topic that FAST_LIO can subscribe to.

**Architecture:** Two separate C++ processes connected by POSIX shared memory. FastDDS 2.14 (drdds) and FastDDS 2.6 (ROS2 Humble) cannot coexist in one process due to symbol conflicts. Process A (`drdds_recv`) runs on the NOS host, links against drdds (`/usr/local/lib/libdrdds.so`), subscribes to `/LIDAR/POINTS` and `/IMU` via `DrDDSSubscriber`, and writes to a SHM ring buffer. Process B (`ros2_pub`) runs inside the nav container, links against ROS2 Humble rclcpp, reads from SHM, and publishes standard ROS2 topics. FAST_LIO subscribes to the republished topics. The container must run with `--ipc host` to share `/dev/shm` with the host process.

**Tech Stack:** C++17, drdds SDK (FastDDS 2.14), ROS2 Humble rclcpp, colcon build system, CMake

---

## Background

### Problem
The M20 rsdriver publishes `/LIDAR/POINTS` using Deep Robotics' `DrDDSPublisher` (built on FastDDS 2.14). This creates a DDS DataWriter that is **invisible** to ROS2's `rmw_fastrtps_cpp` — the publisher endpoint is never discovered by rclpy or rclcpp subscribers. This is confirmed by `count_publishers("/LIDAR/POINTS") == 0` on both Foxy (host) and Humble (container), even though `ros2 topic list` shows the topic.

### Root Cause
drdds uses its own `DrDDSManager` to create DomainParticipants that don't participate in ROS2's `ros_discovery_info` protocol. DDS endpoint discovery partially works (topic names visible), but publisher matching fails. This is a fundamental incompatibility between drdds's bare DDS publisher and ROS2's rmw layer — not a version mismatch.

### Evidence
- `count_publishers("/LIDAR/POINTS") == 0` on Foxy host rclpy (same FastDDS version as drdds)
- `count_publishers("/IMU") == 1` — yesense's publisher IS discoverable (different registration)
- `count_publishers("/ALIGNED_POINTS") == 1` — lio_perception's publisher IS discoverable
- Foxy rclpy on host receives `/ALIGNED_POINTS` at 10Hz (80 msgs in 8s) but 0 for `/LIDAR/POINTS`
- AOS lio_perception (drdds native) DOES receive `/LIDAR/POINTS` from rsdriver
- **Validated 2026-03-18**: Tested rclpy `spin_once` pattern (from investigation Finding #596) on BOTH Foxy host AND Humble container — both get 0 for `/LIDAR/POINTS`. The mac_bridge success in the investigation was for `/ALIGNED_POINTS` (from lio_perception), never for `/LIDAR/POINTS` (from rsdriver). Bridge is confirmed necessary.

### Verified Host Environment (NOS)
- drdds headers: `/usr/local/include/drdds/core/drdds_core.h`, `/usr/local/include/dridl/sensor_msgs/msg/`
- drdds cmake: `/usr/local/lib/cmake/drdds/drddsConfig.cmake` — exports `drdds_INCLUDE_DIRS`, `drdds_LIBRARIES`
- drdds libs: `/usr/local/lib/libdrdds.so`, `libfastrtps.so.2.14`, `libfastcdr.so.2`
- Compiler: `g++ 9.3.0`, `cmake 3.16.3` (both installed on NOS)

### Solution
A C++ bridge that uses `DrDDSSubscriber<sensor_msgs::msg::PointCloud2PubSubType>` (drdds API) to subscribe — this matches rsdriver's publisher because both use the same drdds DomainParticipant system. The bridge then republishes via standard ROS2 Humble `rclcpp::Publisher<sensor_msgs::msg::PointCloud2>`, which FAST_LIO can subscribe to normally.

## Key Constraints

1. **Latency**: `/LIDAR/POINTS` is ~80KB at 10Hz, `/IMU` is small at 200Hz. Bridge must add <1ms overhead.
2. **Memory**: NOS has 15GB RAM. Bridge + nav container + dimos container must fit. Target <100MB for bridge.
3. **Two FastDDS versions**: drdds subscriber links against FastDDS 2.14 (`/usr/local/lib/`), ROS2 publisher links against FastDDS 2.6 (`/opt/ros/humble/lib/`). These CANNOT coexist in the same process — symbol conflicts.
4. **IPC host**: Nav container MUST run with `--ipc host` so `/dev/shm` is shared between host (drdds_recv) and container (ros2_pub).
5. **Cross-compilation**: Not needed — NOS has g++ 9.3.0 and cmake 3.16.3 installed. `drdds_recv` is built directly on NOS.

## Architecture Decision: Shared Memory IPC (not dual-FastDDS)

Since two FastDDS versions can't coexist in one process, the bridge is **two processes** connected by shared memory:

```
Process A (drdds side):
  DrDDSSubscriber → receives /LIDAR/POINTS → writes to shared memory ring buffer

Process B (ROS2 side):
  Reads from shared memory ring buffer → rclcpp::Publisher → publishes /LIDAR/POINTS

  FAST_LIO subscribes to /LIDAR/POINTS from Process B's publisher
```

**IPC mechanism**: POSIX shared memory (`shm_open` + `mmap`) with a lock-free SPSC (single-producer single-consumer) ring buffer. Zero-copy for the point cloud data. The ring buffer header contains write/read indices and message metadata (timestamp, size).

**Alternative considered**: Unix domain socket, TCP loopback, or pipes. Rejected because they require serialize→copy→deserialize. Shared memory allows the ROS2 side to construct the PointCloud2 message directly from the shared buffer with one `memcpy`.

## File Structure

```
dimos/robot/deeprobotics/m20/docker/drdds_bridge/
├── CMakeLists.txt              # Two targets: drdds_recv and ros2_pub
├── package.xml                 # ROS2 package manifest (for ros2_pub only)
├── include/
│   └── shm_transport.h         # Shared memory ring buffer (header-only, shared by both)
├── src/
│   ├── drdds_recv.cpp          # Process A: drdds subscriber → SHM writer
│   └── ros2_pub.cpp            # Process B: SHM reader → ROS2 publisher
├── launch/
│   └── bridge.launch.py        # Launches both processes
└── config/
    └── bridge_params.yaml      # Topic names, SHM config
```

**On NOS host**: `/usr/local/lib/libdrdds.so`, `/usr/local/include/drdds/` — mounted into container at build time.

**In nav container Dockerfile**: New layer copies drdds headers + libs, builds the bridge.

---

## Task 1: Shared Memory Transport Header

**Files:**
- Create: `dimos/robot/deeprobotics/m20/docker/drdds_bridge/include/shm_transport.h`

- [ ] **Step 1: Write the SHM ring buffer header**

Header-only C++17 library. Lock-free SPSC ring buffer over POSIX shared memory. Supports variable-size messages (point clouds vary in size). Each slot has: message size, timestamp_sec, timestamp_nsec, then raw data bytes.

```cpp
// shm_transport.h - Lock-free SPSC ring buffer over POSIX shared memory
// Used by drdds_recv (writer) and ros2_pub (reader) to bridge point cloud data
// without linking both FastDDS versions in one process.

#pragma once
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <atomic>
#include <string>
#include <stdexcept>

namespace drdds_bridge {

// Ring buffer layout in shared memory:
//   [Header: 64 bytes]
//   [Slot 0: SlotHeader + data bytes]
//   [Slot 1: SlotHeader + data bytes]
//   ...
//   [Slot N-1: SlotHeader + data bytes]

static_assert(std::atomic<uint64_t>::is_always_lock_free,
    "atomic<uint64_t> must be lock-free for cross-process SHM on ARM64");

struct ShmHeader {
    std::atomic<uint64_t> write_idx{0};  // monotonically increasing write index
    std::atomic<uint64_t> read_idx{0};   // monotonically increasing read index
    uint32_t num_slots;                   // number of slots in ring buffer
    uint32_t slot_size;                   // max bytes per slot (including SlotHeader)
    uint32_t ready;                       // set to 0xDEADBEEF when initialized
    char padding[28];                     // pad to 64 bytes
};

struct SlotHeader {
    uint32_t data_size;       // actual payload bytes
    uint32_t msg_type;        // 0 = PointCloud2, 1 = Imu
    int32_t  stamp_sec;       // ROS timestamp seconds
    uint32_t stamp_nsec;      // ROS timestamp nanoseconds
    uint32_t height;          // PointCloud2 height
    uint32_t width;           // PointCloud2 width
    uint32_t point_step;      // bytes per point
    uint32_t row_step;        // bytes per row
    uint8_t  is_dense;        // PointCloud2 is_dense
    uint8_t  is_bigendian;    // PointCloud2 is_bigendian
    uint8_t  padding[2];
    uint32_t fields_size;     // size of serialized PointFieldArray following data
    // followed by: [data_size bytes of point data] [fields_size bytes of field descriptors]
};

static constexpr uint32_t SHM_READY_MAGIC = 0xDEADBEEF;
static constexpr const char* SHM_LIDAR_NAME = "/drdds_bridge_lidar";
static constexpr const char* SHM_IMU_NAME = "/drdds_bridge_imu";

// Slot size: 512KB per lidar message (80KB typical, 512KB max headroom)
// 8 slots = 4MB total for lidar ring buffer (extra headroom for CPU preemption on NOS)
static constexpr uint32_t LIDAR_SLOT_SIZE = 512 * 1024;
static constexpr uint32_t LIDAR_NUM_SLOTS = 8;

// IMU: 256 bytes per message, 64 slots (covers 320ms at 200Hz)
static constexpr uint32_t IMU_SLOT_SIZE = 256;
static constexpr uint32_t IMU_NUM_SLOTS = 64;

inline size_t shm_total_size(uint32_t num_slots, uint32_t slot_size) {
    return sizeof(ShmHeader) + (size_t)num_slots * slot_size;
}

// Writer side (used by drdds_recv)
class ShmWriter {
public:
    ShmWriter(const std::string& name, uint32_t num_slots, uint32_t slot_size) {
        size_t total = shm_total_size(num_slots, slot_size);
        int fd = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
        if (fd < 0) throw std::runtime_error("shm_open failed: " + name);
        ftruncate(fd, total);
        mem_ = (uint8_t*)mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        close(fd);
        if (mem_ == MAP_FAILED) throw std::runtime_error("mmap failed");

        hdr_ = reinterpret_cast<ShmHeader*>(mem_);
        hdr_->num_slots = num_slots;
        hdr_->slot_size = slot_size;
        hdr_->write_idx.store(0, std::memory_order_release);
        hdr_->read_idx.store(0, std::memory_order_release);
        hdr_->ready = SHM_READY_MAGIC;

        total_size_ = total;
        name_ = name;
    }

    ~ShmWriter() {
        if (mem_ && mem_ != MAP_FAILED) munmap(mem_, total_size_);
    }

    // Returns pointer to current write slot's SlotHeader. Caller fills in header + data.
    // Call commit() after writing to advance the write index.
    SlotHeader* acquire() {
        uint64_t wi = hdr_->write_idx.load(std::memory_order_relaxed);
        uint32_t slot_idx = wi % hdr_->num_slots;
        return reinterpret_cast<SlotHeader*>(mem_ + sizeof(ShmHeader) + (size_t)slot_idx * hdr_->slot_size);
    }

    uint8_t* slot_data(SlotHeader* slot) {
        return reinterpret_cast<uint8_t*>(slot) + sizeof(SlotHeader);
    }

    void commit() {
        hdr_->write_idx.fetch_add(1, std::memory_order_release);
    }

private:
    uint8_t* mem_ = nullptr;
    ShmHeader* hdr_ = nullptr;
    size_t total_size_ = 0;
    std::string name_;
};

// Reader side (used by ros2_pub)
class ShmReader {
public:
    ShmReader(const std::string& name) : name_(name) {}

    bool try_open() {
        int fd = shm_open(name_.c_str(), O_RDWR, 0666);
        if (fd < 0) return false;

        // Read header first to get sizes
        ShmHeader tmp;
        if (read(fd, &tmp, sizeof(tmp)) != sizeof(tmp)) { close(fd); return false; }
        if (tmp.ready != SHM_READY_MAGIC) { close(fd); return false; }

        size_t total = shm_total_size(tmp.num_slots, tmp.slot_size);
        mem_ = (uint8_t*)mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        close(fd);
        if (mem_ == MAP_FAILED) { mem_ = nullptr; return false; }

        hdr_ = reinterpret_cast<ShmHeader*>(mem_);
        total_size_ = total;
        last_read_ = hdr_->write_idx.load(std::memory_order_acquire); // start from current
        return true;
    }

    // Returns next unread slot, or nullptr if no new data
    const SlotHeader* poll() {
        if (!hdr_) return nullptr;
        uint64_t wi = hdr_->write_idx.load(std::memory_order_acquire);
        if (last_read_ >= wi) return nullptr;

        // Skip if we fell behind (overwritten slots)
        if (wi - last_read_ > hdr_->num_slots) {
            last_read_ = wi - 1; // jump to latest
        }

        uint32_t slot_idx = last_read_ % hdr_->num_slots;
        last_read_++;
        return reinterpret_cast<const SlotHeader*>(mem_ + sizeof(ShmHeader) + (size_t)slot_idx * hdr_->slot_size);
    }

    const uint8_t* slot_data(const SlotHeader* slot) {
        return reinterpret_cast<const uint8_t*>(slot) + sizeof(SlotHeader);
    }

    bool is_open() const { return hdr_ != nullptr; }

    ~ShmReader() {
        if (mem_ && mem_ != MAP_FAILED) munmap(mem_, total_size_);
    }

private:
    uint8_t* mem_ = nullptr;
    ShmHeader* hdr_ = nullptr;
    size_t total_size_ = 0;
    uint64_t last_read_ = 0;
    std::string name_;
};

} // namespace drdds_bridge
```

- [ ] **Step 2: Commit**

```bash
git add dimos/robot/deeprobotics/m20/docker/drdds_bridge/include/shm_transport.h
git commit -m "feat(m20): add shared memory transport for drdds bridge"
```

---

## Task 2: drdds Receiver Process (Process A)

**Files:**
- Create: `dimos/robot/deeprobotics/m20/docker/drdds_bridge/src/drdds_recv.cpp`

**Dependencies:** Links against host's drdds + FastDDS 2.14. Does NOT link against ROS2.

- [ ] **Step 1: Write the drdds receiver**

```cpp
// drdds_recv.cpp - Subscribes to /LIDAR/POINTS and /IMU via drdds,
// writes to shared memory for the ROS2 publisher process.
//
// Links against: libdrdds.so, libfastrtps.so.2.14, libfastcdr.so.2
// Does NOT link against ROS2 (avoids FastDDS version conflict).

#include "shm_transport.h"
#include "drdds/core/drdds_core.h"
#include "dridl/sensor_msgs/msg/PointCloud2.h"
#include "dridl/sensor_msgs/msg/PointCloud2PubSubTypes.h"
#include "dridl/sensor_msgs/msg/Imu.h"
#include "dridl/sensor_msgs/msg/ImuPubSubTypes.h"

#include <csignal>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

// Counters for logging
static std::atomic<uint64_t> lidar_count{0};
static std::atomic<uint64_t> imu_count{0};

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize drdds on domain 0
    DrDDSManager::Init(0);

    // Create shared memory writers
    drdds_bridge::ShmWriter lidar_shm(
        drdds_bridge::SHM_LIDAR_NAME,
        drdds_bridge::LIDAR_NUM_SLOTS,
        drdds_bridge::LIDAR_SLOT_SIZE);

    drdds_bridge::ShmWriter imu_shm(
        drdds_bridge::SHM_IMU_NAME,
        drdds_bridge::IMU_NUM_SLOTS,
        drdds_bridge::IMU_SLOT_SIZE);

    std::cout << "[drdds_recv] SHM initialized. Subscribing to /LIDAR/POINTS and /IMU..." << std::endl;

    // Lidar callback
    auto on_lidar = [&](const sensor_msgs::msg::PointCloud2* msg) {
        auto* slot = lidar_shm.acquire();
        uint8_t* data = lidar_shm.slot_data(slot);

        uint32_t data_size = msg->data().size();
        if (data_size + sizeof(drdds_bridge::SlotHeader) > drdds_bridge::LIDAR_SLOT_SIZE) {
            std::cerr << "[drdds_recv] WARN: lidar msg too large: " << data_size << std::endl;
            return;
        }

        slot->data_size = data_size;
        slot->msg_type = 0; // PointCloud2
        slot->stamp_sec = msg->header().stamp().sec();
        slot->stamp_nsec = msg->header().stamp().nanosec();
        slot->height = msg->height();
        slot->width = msg->width();
        slot->point_step = msg->point_step();
        slot->row_step = msg->row_step();
        slot->is_dense = msg->is_dense() ? 1 : 0;
        slot->is_bigendian = msg->is_bigendian() ? 1 : 0;
        slot->fields_size = 0; // fields serialized separately below

        // Copy point cloud data
        std::memcpy(data, msg->data().data(), data_size);

        // Serialize PointField descriptors after the data
        // Format: [count:u32] [name_len:u32 name:bytes offset:u32 datatype:u8 count:u32] ...
        uint8_t* fields_ptr = data + data_size;
        uint32_t fields_count = msg->fields().size();
        std::memcpy(fields_ptr, &fields_count, 4);
        fields_ptr += 4;
        uint32_t fields_bytes = 4;

        for (const auto& f : msg->fields()) {
            uint32_t nlen = f.name().size();
            std::memcpy(fields_ptr, &nlen, 4); fields_ptr += 4;
            std::memcpy(fields_ptr, f.name().data(), nlen); fields_ptr += nlen;
            uint32_t offset = f.offset();
            std::memcpy(fields_ptr, &offset, 4); fields_ptr += 4;
            uint8_t dtype = f.datatype();
            std::memcpy(fields_ptr, &dtype, 1); fields_ptr += 1;
            uint32_t cnt = f.count();
            std::memcpy(fields_ptr, &cnt, 4); fields_ptr += 4;
            fields_bytes += 4 + nlen + 4 + 1 + 4;
        }
        slot->fields_size = fields_bytes;

        lidar_shm.commit();

        uint64_t c = lidar_count.fetch_add(1) + 1;
        if (c % 100 == 1) {
            std::cout << "[drdds_recv] lidar #" << c
                      << " pts=" << msg->width() * msg->height()
                      << " bytes=" << data_size << std::endl;
        }
    };

    // IMU callback
    auto on_imu = [&](const sensor_msgs::msg::Imu* msg) {
        auto* slot = imu_shm.acquire();
        uint8_t* data = imu_shm.slot_data(slot);

        slot->msg_type = 1; // Imu
        slot->stamp_sec = msg->header().stamp().sec();
        slot->stamp_nsec = msg->header().stamp().nanosec();

        // Pack IMU data: orientation(4d), angular_vel(3d), linear_accel(3d) = 10 doubles = 80 bytes
        double* dp = reinterpret_cast<double*>(data);
        dp[0] = msg->orientation().x();
        dp[1] = msg->orientation().y();
        dp[2] = msg->orientation().z();
        dp[3] = msg->orientation().w();
        dp[4] = msg->angular_velocity().x();
        dp[5] = msg->angular_velocity().y();
        dp[6] = msg->angular_velocity().z();
        dp[7] = msg->linear_acceleration().x();
        dp[8] = msg->linear_acceleration().y();
        dp[9] = msg->linear_acceleration().z();
        slot->data_size = 80;

        imu_shm.commit();

        uint64_t c = imu_count.fetch_add(1) + 1;
        if (c % 1000 == 1) {
            std::cout << "[drdds_recv] imu #" << c << std::endl;
        }
    };

    // Create drdds subscribers
    // topic_prefix "" to match rsdriver's registration
    DrDDSSubscriber<sensor_msgs::msg::PointCloud2PubSubType> lidar_sub(
        on_lidar, "/LIDAR/POINTS", 0, "");

    DrDDSSubscriber<sensor_msgs::msg::ImuPubSubType> imu_sub(
        on_imu, "/IMU", 0, "");

    std::cout << "[drdds_recv] Subscriptions created. Waiting for data..." << std::endl;

    // Log match status periodically
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        std::cout << "[drdds_recv] status: lidar_matched=" << lidar_sub.GetMatchedCount()
                  << " imu_matched=" << imu_sub.GetMatchedCount()
                  << " lidar_msgs=" << lidar_count.load()
                  << " imu_msgs=" << imu_count.load() << std::endl;
    }

    std::cout << "[drdds_recv] Shutting down." << std::endl;
    DrDDSManager::Delete();
    return 0;
}
```

- [ ] **Step 2: Commit**

```bash
git add dimos/robot/deeprobotics/m20/docker/drdds_bridge/src/drdds_recv.cpp
git commit -m "feat(m20): add drdds receiver process for lidar/imu bridge"
```

---

## Task 3: ROS2 Publisher Process (Process B)

**Files:**
- Create: `dimos/robot/deeprobotics/m20/docker/drdds_bridge/src/ros2_pub.cpp`

**Dependencies:** Links against ROS2 Humble rclcpp, sensor_msgs. Does NOT link against drdds.

- [ ] **Step 1: Write the ROS2 publisher**

```cpp
// ros2_pub.cpp - Reads lidar/IMU data from shared memory and publishes
// as standard ROS2 Humble topics for FAST_LIO consumption.
//
// Links against: rclcpp, sensor_msgs (ROS2 Humble, FastDDS 2.6)
// Does NOT link against drdds (avoids FastDDS version conflict).

#include "shm_transport.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <thread>

class BridgePublisher : public rclcpp::Node {
public:
    BridgePublisher()
        : Node("drdds_bridge"),
          lidar_reader_(drdds_bridge::SHM_LIDAR_NAME),
          imu_reader_(drdds_bridge::SHM_IMU_NAME)
    {
        // QoS: sensor data profile (BEST_EFFORT for low latency)
        auto qos = rclcpp::SensorDataQoS();

        lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/LIDAR/POINTS", qos);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/IMU", qos);

        // Poll SHM at 1kHz (1ms period) — sufficient for 10Hz lidar + 200Hz IMU
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&BridgePublisher::poll_shm, this));

        RCLCPP_INFO(this->get_logger(), "Bridge publisher started. Waiting for SHM...");
    }

private:
    void poll_shm() {
        // Try to open SHM if not yet connected
        if (!lidar_reader_.is_open()) {
            if (!lidar_reader_.try_open()) return;
            RCLCPP_INFO(this->get_logger(), "Lidar SHM connected");
        }
        if (!imu_reader_.is_open()) {
            if (!imu_reader_.try_open()) return;
            RCLCPP_INFO(this->get_logger(), "IMU SHM connected");
        }

        // Drain all pending lidar messages
        while (auto* slot = lidar_reader_.poll()) {
            if (slot->msg_type != 0) continue;

            auto msg = sensor_msgs::msg::PointCloud2();
            msg.header.stamp.sec = slot->stamp_sec;
            msg.header.stamp.nanosec = slot->stamp_nsec;
            msg.header.frame_id = "lidar_link";
            msg.height = slot->height;
            msg.width = slot->width;
            msg.point_step = slot->point_step;
            msg.row_step = slot->row_step;
            msg.is_dense = slot->is_dense;
            msg.is_bigendian = slot->is_bigendian;

            // Copy point data
            const uint8_t* data = lidar_reader_.slot_data(slot);
            msg.data.assign(data, data + slot->data_size);

            // Deserialize PointField descriptors
            const uint8_t* fp = data + slot->data_size;
            uint32_t fields_count;
            std::memcpy(&fields_count, fp, 4); fp += 4;
            for (uint32_t i = 0; i < fields_count; i++) {
                sensor_msgs::msg::PointField field;
                uint32_t nlen;
                std::memcpy(&nlen, fp, 4); fp += 4;
                field.name.assign(reinterpret_cast<const char*>(fp), nlen); fp += nlen;
                std::memcpy(&field.offset, fp, 4); fp += 4;
                field.datatype = *fp; fp += 1;
                std::memcpy(&field.count, fp, 4); fp += 4;
                msg.fields.push_back(field);
            }

            lidar_pub_->publish(msg);
            lidar_count_++;
            if (lidar_count_ % 100 == 1) {
                RCLCPP_INFO(this->get_logger(), "lidar #%lu pts=%u",
                            lidar_count_, slot->width * slot->height);
            }
        }

        // Drain all pending IMU messages
        while (auto* slot = imu_reader_.poll()) {
            if (slot->msg_type != 1) continue;

            auto msg = sensor_msgs::msg::Imu();
            msg.header.stamp.sec = slot->stamp_sec;
            msg.header.stamp.nanosec = slot->stamp_nsec;
            msg.header.frame_id = "imu_link";

            const double* dp = reinterpret_cast<const double*>(imu_reader_.slot_data(slot));
            msg.orientation.x = dp[0];
            msg.orientation.y = dp[1];
            msg.orientation.z = dp[2];
            msg.orientation.w = dp[3];
            msg.angular_velocity.x = dp[4];
            msg.angular_velocity.y = dp[5];
            msg.angular_velocity.z = dp[6];
            msg.linear_acceleration.x = dp[7];
            msg.linear_acceleration.y = dp[8];
            msg.linear_acceleration.z = dp[9];

            imu_pub_->publish(msg);
            imu_count_++;
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    drdds_bridge::ShmReader lidar_reader_;
    drdds_bridge::ShmReader imu_reader_;
    uint64_t lidar_count_ = 0;
    uint64_t imu_count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BridgePublisher>());
    rclcpp::shutdown();
    return 0;
}
```

- [ ] **Step 2: Commit**

```bash
git add dimos/robot/deeprobotics/m20/docker/drdds_bridge/src/ros2_pub.cpp
git commit -m "feat(m20): add ROS2 publisher process for lidar/imu bridge"
```

---

## Task 4: CMake Build System

**Files:**
- Create: `dimos/robot/deeprobotics/m20/docker/drdds_bridge/CMakeLists.txt`
- Create: `dimos/robot/deeprobotics/m20/docker/drdds_bridge/package.xml`

- [ ] **Step 1: Write CMakeLists.txt**

Two separate targets with different link dependencies:
- `drdds_recv`: links against drdds + FastDDS 2.14 (from `/opt/drdds-host/`)
- `ros2_pub`: links against ROS2 Humble rclcpp + sensor_msgs

```cmake
cmake_minimum_required(VERSION 3.8)
project(drdds_bridge)

set(CMAKE_CXX_STANDARD 17)

# ---- Target 1: drdds_recv (drdds side, FastDDS 2.14) ----
# Built separately because it links against host's drdds libs (FastDDS 2.14)
# which conflict with ROS2 Humble's FastDDS 2.6.
#
# This target is built with a standalone cmake invocation, NOT through colcon,
# because colcon would inject ROS2's FastDDS into the link path.
#
# Build command (standalone):
#   mkdir -p build_drdds && cd build_drdds
#   cmake .. -DBUILD_DRDDS_RECV=ON -DBUILD_ROS2_PUB=OFF \
#     -Ddrdds_DIR=/opt/drdds-host/lib/cmake/drdds
#   make -j2

option(BUILD_DRDDS_RECV "Build the drdds receiver" OFF)
option(BUILD_ROS2_PUB "Build the ROS2 publisher" OFF)

if(BUILD_DRDDS_RECV)
    find_package(drdds REQUIRED)

    add_executable(drdds_recv src/drdds_recv.cpp)
    target_include_directories(drdds_recv PRIVATE
        include
        ${drdds_INCLUDE_DIRS}
    )
    target_link_libraries(drdds_recv ${drdds_LIBRARIES} -lrt)

    install(TARGETS drdds_recv DESTINATION lib/${PROJECT_NAME})
endif()

# ---- Target 2: ros2_pub (ROS2 side, FastDDS 2.6) ----
# Built through colcon as a standard ROS2 package.

if(BUILD_ROS2_PUB)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)

    add_executable(ros2_pub src/ros2_pub.cpp)
    target_include_directories(ros2_pub PRIVATE include)
    ament_target_dependencies(ros2_pub rclcpp sensor_msgs)
    target_link_libraries(ros2_pub -lrt)

    install(TARGETS ros2_pub DESTINATION lib/${PROJECT_NAME})

    # Install launch file
    install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

    ament_package()
endif()
```

- [ ] **Step 2: Write package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>drdds_bridge</name>
  <version>0.1.0</version>
  <description>Bridge between drdds (DeepRobotics DDS) and ROS2 Humble for M20 lidar/IMU</description>
  <maintainer email="dev@dimos.ai">dimos</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Commit**

```bash
git add dimos/robot/deeprobotics/m20/docker/drdds_bridge/CMakeLists.txt
git add dimos/robot/deeprobotics/m20/docker/drdds_bridge/package.xml
git commit -m "feat(m20): add CMake build system for drdds bridge"
```

---

## Task 5: Launch File

**Files:**
- Create: `dimos/robot/deeprobotics/m20/docker/drdds_bridge/launch/bridge.launch.py`

- [ ] **Step 1: Write launch file**

Launches both processes. `drdds_recv` must start first (creates SHM), then `ros2_pub` connects.

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    bridge_lib = os.path.join(
        os.environ.get('DRDDS_BRIDGE_PATH', '/opt/drdds_bridge'),
        'lib', 'drdds_bridge')

    # Process A: drdds receiver (standalone binary, NOT a ROS2 node)
    # Must set LD_LIBRARY_PATH to host drdds libs
    drdds_recv = ExecuteProcess(
        cmd=[os.path.join(bridge_lib, 'drdds_recv')],
        name='drdds_recv',
        output='screen',
        additional_env={
            'LD_LIBRARY_PATH': '/opt/drdds-host/lib:' + os.environ.get('LD_LIBRARY_PATH', ''),
        },
    )

    # Process B: ROS2 publisher (standard ROS2 node)
    # Delayed 1s to let drdds_recv create SHM
    ros2_pub = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='drdds_bridge',
                executable='ros2_pub',
                name='drdds_bridge',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([drdds_recv, ros2_pub])
```

- [ ] **Step 2: Commit**

```bash
git add dimos/robot/deeprobotics/m20/docker/drdds_bridge/launch/bridge.launch.py
git commit -m "feat(m20): add launch file for drdds bridge"
```

---

## Task 6: Dockerfile Integration

**Files:**
- Modify: `docker/navigation/Dockerfile`

The nav container Dockerfile needs two new layers:
1. Copy drdds host libs + headers into the image (for building `drdds_recv`)
2. Build the bridge (both targets)

- [ ] **Step 1: Add drdds bridge build layers to Dockerfile**

Add after the FAST_LIO clone step (step 10/12 in builder) and before the colcon build step:

```dockerfile
# --- Layer: Copy drdds host SDK for bridge compilation ---
# The drdds SDK (FastDDS 2.14) is from the M20 NOS host. It must be isolated
# from the container's ROS2 Humble (FastDDS 2.6) to avoid symbol conflicts.
# Copy to /opt/drdds-host/ — used only for compiling drdds_recv.
COPY dimos/robot/deeprobotics/m20/docker/drdds_bridge /opt/drdds_bridge/src

# In the runtime stage, after copying built workspace:

# --- Layer: Build drdds_recv (links against host drdds libs, NOT ROS2) ---
# This is built standalone (not through colcon) to avoid FastDDS version conflicts.
# Host drdds libs are mounted at runtime via docker volume.
# For now, we compile just the ROS2 side (ros2_pub) in the image.
# drdds_recv is compiled on the NOS host directly.

# --- Layer: Build ros2_pub via colcon ---
# Copy bridge source into the ROS workspace and rebuild
COPY dimos/robot/deeprobotics/m20/docker/drdds_bridge ${WORKSPACE}/src/drdds_bridge
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${WORKSPACE} && \
    colcon build --packages-select drdds_bridge \
    --cmake-args -DBUILD_ROS2_PUB=ON -DBUILD_DRDDS_RECV=OFF"
```

**Important**: `drdds_recv` is compiled directly on the NOS host (not in Docker) because it needs the host's drdds libraries at link time. The `ros2_pub` is compiled in the container via colcon.

- [ ] **Step 2: Commit**

```bash
git add docker/navigation/Dockerfile
git commit -m "feat(m20): add drdds bridge build to nav container Dockerfile"
```

---

## Task 7: Host-side Build Script for drdds_recv

**Files:**
- Create: `dimos/robot/deeprobotics/m20/docker/drdds_bridge/build_host.sh`

Since `drdds_recv` links against the host's drdds (FastDDS 2.14), it must be compiled on the NOS host (arm64, Ubuntu 20.04).

- [ ] **Step 1: Write host build script**

```bash
#!/bin/bash
# Build drdds_recv on NOS host.
# Requires: cmake, g++ (installed on NOS)
# Links against: /usr/local/lib/libdrdds.so, libfastrtps.so.2.14

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build_host"
INSTALL_DIR="/opt/drdds_bridge"

echo "=== Building drdds_recv on host ==="
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake "${SCRIPT_DIR}" \
    -DBUILD_DRDDS_RECV=ON \
    -DBUILD_ROS2_PUB=OFF \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
    -DCMAKE_BUILD_TYPE=Release

make -j2
sudo mkdir -p "${INSTALL_DIR}/lib/drdds_bridge"
sudo cp drdds_recv "${INSTALL_DIR}/lib/drdds_bridge/"

echo "=== drdds_recv installed to ${INSTALL_DIR}/lib/drdds_bridge/drdds_recv ==="
echo "Run with: LD_LIBRARY_PATH=/usr/local/lib ${INSTALL_DIR}/lib/drdds_bridge/drdds_recv"
```

- [ ] **Step 2: Commit**

```bash
git add dimos/robot/deeprobotics/m20/docker/drdds_bridge/build_host.sh
git commit -m "feat(m20): add host build script for drdds_recv"
```

---

## Task 8: deploy.sh Integration

**Files:**
- Modify: `dimos/robot/deeprobotics/m20/docker/deploy.sh`

Add a `bridge` command to deploy.sh that:
1. Syncs the bridge source to NOS
2. Builds `drdds_recv` on the host
3. Starts both processes (drdds_recv on host, ros2_pub in container)

- [ ] **Step 1: Add bridge command to deploy.sh**

Add a new case in the deploy.sh command switch:

```bash
    bridge-build)
        echo "=== Building drdds bridge on NOS host ==="
        # Sync bridge source
        rsync -avz --delete \
            -e "ssh ${SSH_OPTS}" \
            "${DIMOS_ROOT}/dimos/robot/deeprobotics/m20/docker/drdds_bridge/" \
            "${NOS_USER}@${NOS_HOST}:/tmp/drdds_bridge/"

        # Build on host
        remote_ssh "cd /tmp/drdds_bridge && chmod +x build_host.sh && ./build_host.sh"
        echo "=== Bridge build complete ==="
        ;;

    bridge-start)
        echo "=== Starting drdds bridge ==="
        # Start drdds_recv on host (background)
        remote_ssh "pkill -f drdds_recv 2>/dev/null; sleep 1; \
            nohup /opt/drdds_bridge/lib/drdds_bridge/drdds_recv \
            > /var/log/drdds_recv.log 2>&1 &"
        echo "  drdds_recv started on host"

        # Start ros2_pub in nav container
        remote_ssh "docker exec -d dimos-nav bash -c '\
            source /opt/ros/humble/setup.bash && \
            source /ros2_ws/install/setup.bash && \
            export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/m20_fastdds.xml && \
            ros2 run drdds_bridge ros2_pub'"
        echo "  ros2_pub started in container"
        echo "=== Bridge running ==="
        ;;

    bridge-stop)
        remote_ssh "pkill -f drdds_recv 2>/dev/null; \
            docker exec dimos-nav pkill -f ros2_pub 2>/dev/null"
        echo "Bridge stopped."
        ;;
```

- [ ] **Step 2: Commit**

```bash
git add dimos/robot/deeprobotics/m20/docker/deploy.sh
git commit -m "feat(m20): add bridge-build/start/stop commands to deploy.sh"
```

---

## Task 9: Integration Test

- [ ] **Step 1: Build and deploy the bridge**

```bash
# From Mac:
./deploy.sh bridge-build

# Start nav container (if not running):
# (use docker create + docker start pattern from earlier)

# Start the bridge:
./deploy.sh bridge-start
```

- [ ] **Step 2: Verify drdds_recv receives data**

```bash
# Check drdds_recv logs:
ssh -o ProxyJump=user@10.21.41.1 user@10.21.31.106 'tail -5 /var/log/drdds_recv.log'
# Expected: "[drdds_recv] lidar #1 pts=XXXX bytes=XXXXX" and imu messages
```

- [ ] **Step 3: Verify ros2_pub publishes data**

```bash
# From inside nav container:
docker exec dimos-nav bash -c "source /opt/ros/humble/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    timeout 5 ros2 topic hz /LIDAR/POINTS"
# Expected: ~10 Hz
```

- [ ] **Step 4: Start FAST_LIO and verify it receives bridged data**

```bash
# Launch FAST_LIO with the bridge running:
docker exec dimos-nav bash -c "source /opt/ros/humble/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/m20_fastdds.xml && \
    ros2 launch fast_lio mapping.launch.py \
        config_path:=/ros2_ws/src/fast_lio/config \
        config_file:=robosense.yaml \
        rviz:=false"
# Expected: FAST_LIO processes point clouds, logs pose estimates
```

- [ ] **Step 5: Commit any fixes from integration testing**

---

## Risk Mitigations

| Risk | Mitigation |
|------|-----------|
| drdds `topic_prefix` doesn't match — `DrDDSSubscriber` gets 0 matches | Try `""`, `"dr"`, `"dr/"` as prefix. Check `GetMatchedCount()` in logs. |
| drdds `DrDDSManager::Init(0)` fails in standalone process | Check if Init needs additional params. Look at how rsdriver calls it. |
| SHM `/dev/shm` not shared between host and container | Container runs with `--privileged --ipc host` or mount `/dev/shm` explicitly. |
| NOS has no C++ compiler | Install: `apt-get install g++ cmake` or cross-compile on Mac. |
| Point cloud field descriptors differ between drdds and ROS2 | Bridge serializes fields explicitly. Verify field names match FAST_LIO expectations. |
| IMU frame_id mismatch | FAST_LIO config uses `imu_topic: "/IMU"`. Check if it also matches on frame_id. |
