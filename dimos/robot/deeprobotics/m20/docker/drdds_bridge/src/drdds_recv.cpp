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

    // Initialize drdds on domain 0 using the multi-domain Init (same as rsdriver)
    std::vector<int> domains = {0};
    DrDDSManager::Init(domains, "drdds_bridge", "drdds_recv", false, false, false);

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
        // Reserve space for fields after the data: 4 + fields * (4+32+4+1+4) max ~2KB
        uint32_t max_fields_size = 4 + msg->fields().size() * 45;
        if (data_size + max_fields_size + sizeof(drdds_bridge::SlotHeader) > drdds_bridge::LIDAR_SLOT_SIZE) {
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
        if (c % 10 == 1) {
            std::cout << "[drdds_recv] lidar #" << c
                      << " stamp=" << msg->header().stamp().sec() << "." << msg->header().stamp().nanosec()
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

    // DrDDSChannel (pub+sub) — verified working in previous session
    DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType> lidar_ch(
        on_lidar, "/LIDAR/POINTS", 0);

    DrDDSChannel<sensor_msgs::msg::ImuPubSubType> imu_ch(
        on_imu, "/IMU", 0);

    std::cout << "[drdds_recv] Channels created. Waiting for data..." << std::endl;

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        std::cout << "[drdds_recv] status: lidar_matched=" << lidar_ch.GetMatchedCount()
                  << " imu_matched=" << imu_ch.GetMatchedCount()
                  << " lidar_msgs=" << lidar_count.load()
                  << " imu_msgs=" << imu_count.load() << std::endl;
    }

    std::cout << "[drdds_recv] Shutting down." << std::endl;
    DrDDSManager::Delete();
    return 0;
}
