// drdds_recv.cpp - Subscribes to /LIDAR/POINTS and /IMU via drdds,
// writes to shared memory for the ROS2 publisher process.
//
// Links against: libdrdds.so, libfastrtps.so.2.14, libfastcdr.so.2
// Does NOT link against ROS2 (avoids FastDDS version conflict).

#include "shm_transport.h"
#include "drdds/core/drdds_core.h"
#include "dridl/sensor_msgs/msg/PointCloud2.h"
#include "dridl/sensor_msgs/msg/PointCloud2PubSubTypes.h"
#include <csignal>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <semaphore.h>

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

static std::atomic<uint64_t> lidar_count{0};

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

    // Semaphore for notifying ros2_pub of new data (replaces 1ms sleep polling)
    sem_unlink(drdds_bridge::SHM_NOTIFY_NAME); // clear stale count from previous run
    sem_t* notify_sem = sem_open(drdds_bridge::SHM_NOTIFY_NAME, O_CREAT, 0666, 0);
    if (notify_sem == SEM_FAILED) {
        std::cerr << "[drdds_recv] WARN: sem_open failed, ros2_pub will fall back to polling" << std::endl;
        notify_sem = nullptr;
    }

    std::cout << "[drdds_recv] SHM initialized. Subscribing to /LIDAR/POINTS..." << std::endl;

    // Lidar callback — only passes raw point data + header metadata.
    // PointField descriptors are hardcoded in ros2_pub (RSAIRY fields never change).
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
        slot->fields_size = 0; // fields hardcoded in ros2_pub

        std::memcpy(data, msg->data().data(), data_size);
        lidar_shm.commit();
        if (notify_sem) sem_post(notify_sem);

        uint64_t c = lidar_count.fetch_add(1) + 1;
        if (c % 10 == 1) {
            std::cout << "[drdds_recv] lidar #" << c
                      << " stamp=" << msg->header().stamp().sec() << "." << msg->header().stamp().nanosec()
                      << " pts=" << msg->width() * msg->height()
                      << " bytes=" << data_size << std::endl;
        }
    };

    // DrDDSChannel (pub+sub) — required for rsdriver matching
    // IMU NOT bridged — yesense /IMU is directly readable by ROS2 (verified 2026-03-18)
    DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType> lidar_ch(
        on_lidar, "/LIDAR/POINTS", 0);

    std::cout << "[drdds_recv] Channels created. Waiting for data..." << std::endl;

    // Health watchdog: warn if no data for 30s
    uint64_t last_lidar = 0;
    int no_data_cycles = 0;

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        uint64_t cur = lidar_count.load();
        std::cout << "[drdds_recv] status: lidar_matched=" << lidar_ch.GetMatchedCount()
                  << " lidar_msgs=" << cur << std::endl;

        if (cur == last_lidar) {
            no_data_cycles++;
            if (no_data_cycles >= 6) { // 30s with no data
                std::cerr << "[drdds_recv] WARNING: no lidar data for 30s. "
                          << "Restart sequence: systemctl stop rsdriver && "
                          << "systemctl restart drdds-bridge && "
                          << "systemctl start rsdriver" << std::endl;
                no_data_cycles = 0; // don't spam
            }
        } else {
            no_data_cycles = 0;
        }
        last_lidar = cur;
    }

    std::cout << "[drdds_recv] Shutting down." << std::endl;
    DrDDSManager::Delete();
    return 0;
}
