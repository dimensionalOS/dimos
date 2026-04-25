// drdds_recv.cpp - Subscribes to /LIDAR/POINTS and /IMU via drdds,
// writes to POSIX shared memory for DrddsLidarBridge to read.
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
#include <semaphore.h>

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

static std::atomic<uint64_t> lidar_count{0};
static std::atomic<uint64_t> lidar2_count{0};
static std::atomic<uint64_t> imu_count{0};
static std::atomic<uint64_t> imu_airy_front_count{0};
static std::atomic<uint64_t> imu_airy_rear_count{0};

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // 2-arg DrDDSManager::Init(domain_id, network_name) — the network_name
    // arg is a slash-separated list of interfaces to bind for UDP transport.
    // rsdriver's rodata reveals it passes "eth0/eth1" (verified via
    // `objdump -s -j .rodata /opt/robot/share/node_driver/bin/rslidar`).
    // Empty string makes the participant bind localhost-only — that's why
    // pre-fix our drdds_recv only saw matched=1 self-match for lidar:
    // rsdriver tried to deliver via UDP unicast to our 10.21.x.x announced
    // address but we only listened on 127.0.0.1. See FASTLIO2_LOG #36.
    DrDDSManager::Init(0, "eth0/eth1");

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

    drdds_bridge::ShmWriter imu_shm(
        drdds_bridge::SHM_IMU_NAME,
        drdds_bridge::IMU_NUM_SLOTS,
        drdds_bridge::IMU_SLOT_SIZE);

    // Separate-mode extras: rsdriver's send_separately=true publishes
    //   /LIDAR/POINTS2   — rear Airy cloud
    //   /LIDAR/IMU201    — front Airy integrated IMU
    //   /LIDAR/IMU202    — rear Airy integrated IMU
    // In merged mode these publishers simply aren't created — matched stays 0.
    drdds_bridge::ShmWriter lidar2_shm(
        drdds_bridge::SHM_LIDAR2_NAME,
        drdds_bridge::LIDAR_NUM_SLOTS,
        drdds_bridge::LIDAR_SLOT_SIZE);

    drdds_bridge::ShmWriter imu_airy_front_shm(
        drdds_bridge::SHM_IMU_AIRY_FRONT_NAME,
        drdds_bridge::IMU_NUM_SLOTS,
        drdds_bridge::IMU_SLOT_SIZE);

    drdds_bridge::ShmWriter imu_airy_rear_shm(
        drdds_bridge::SHM_IMU_AIRY_REAR_NAME,
        drdds_bridge::IMU_NUM_SLOTS,
        drdds_bridge::IMU_SLOT_SIZE);

    std::cout << "[drdds_recv] SHM initialized. Subscribing to /LIDAR/POINTS, "
              << "/LIDAR/POINTS2, /IMU, /LIDAR/IMU201, /LIDAR/IMU202..." << std::endl;

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

    // IMU callback — serializes IMU data into a flat buffer for SHM transport.
    // Layout: [stamp_sec(4) stamp_nsec(4) orientation(4×f64=32) angular_vel(3×f64=24) linear_acc(3×f64=24)]
    // Total: 88 bytes (fits easily in IMU_SLOT_SIZE=256)
    auto on_imu = [&](const sensor_msgs::msg::Imu* msg) {
        auto* slot = imu_shm.acquire();
        uint8_t* data = imu_shm.slot_data(slot);

        slot->data_size = 88;
        slot->msg_type = 1; // Imu
        slot->stamp_sec = msg->header().stamp().sec();
        slot->stamp_nsec = msg->header().stamp().nanosec();

        // Pack IMU data as doubles into flat buffer
        size_t off = 0;
        auto write_d = [&](double v) { std::memcpy(data + off, &v, 8); off += 8; };
        // Orientation quaternion (x, y, z, w)
        write_d(msg->orientation().x());
        write_d(msg->orientation().y());
        write_d(msg->orientation().z());
        write_d(msg->orientation().w());
        // Angular velocity (x, y, z)
        write_d(msg->angular_velocity().x());
        write_d(msg->angular_velocity().y());
        write_d(msg->angular_velocity().z());
        // Linear acceleration (x, y, z)
        write_d(msg->linear_acceleration().x());
        write_d(msg->linear_acceleration().y());
        write_d(msg->linear_acceleration().z());

        imu_shm.commit();
        if (notify_sem) sem_post(notify_sem);

        uint64_t c = imu_count.fetch_add(1) + 1;
        if (c % 200 == 1) {
            std::cout << "[drdds_recv] imu #" << c
                      << " stamp=" << msg->header().stamp().sec() << "." << msg->header().stamp().nanosec()
                      << " acc=(" << msg->linear_acceleration().x()
                      << "," << msg->linear_acceleration().y()
                      << "," << msg->linear_acceleration().z() << ")" << std::endl;
        }
    };

    // Lidar2 callback (rear lidar in separate mode, writes to SHM_LIDAR2).
    auto on_lidar2 = [&](const sensor_msgs::msg::PointCloud2* msg) {
        auto* slot = lidar2_shm.acquire();
        uint8_t* data = lidar2_shm.slot_data(slot);
        uint32_t data_size = msg->data().size();
        if (data_size + sizeof(drdds_bridge::SlotHeader) > drdds_bridge::LIDAR_SLOT_SIZE) {
            std::cerr << "[drdds_recv] WARN: lidar2 msg too large: " << data_size << std::endl;
            return;
        }
        slot->data_size = data_size;
        slot->msg_type = 0;
        slot->stamp_sec = msg->header().stamp().sec();
        slot->stamp_nsec = msg->header().stamp().nanosec();
        slot->height = msg->height();
        slot->width = msg->width();
        slot->point_step = msg->point_step();
        slot->row_step = msg->row_step();
        slot->is_dense = msg->is_dense() ? 1 : 0;
        slot->is_bigendian = msg->is_bigendian() ? 1 : 0;
        slot->fields_size = 0;
        std::memcpy(data, msg->data().data(), data_size);
        lidar2_shm.commit();
        uint64_t c = lidar2_count.fetch_add(1) + 1;
        if (c % 10 == 1) {
            std::cout << "[drdds_recv] lidar2 #" << c
                      << " stamp=" << msg->header().stamp().sec() << "." << msg->header().stamp().nanosec()
                      << " pts=" << msg->width() * msg->height()
                      << " bytes=" << data_size << std::endl;
        }
    };

    // Airy IMU callback factory — serializes to the given SHM writer and
    // bumps the named counter. Separate SHM per IMU so downstream consumers
    // can pick front-Airy vs rear-Airy vs yesense independently.
    auto make_imu_cb = [](drdds_bridge::ShmWriter* shm, std::atomic<uint64_t>* count, const char* label) {
        return [shm, count, label](const sensor_msgs::msg::Imu* msg) {
            auto* slot = shm->acquire();
            uint8_t* data = shm->slot_data(slot);
            slot->data_size = 88;
            slot->msg_type = 1;
            slot->stamp_sec = msg->header().stamp().sec();
            slot->stamp_nsec = msg->header().stamp().nanosec();
            size_t off = 0;
            auto write_d = [&](double v) { std::memcpy(data + off, &v, 8); off += 8; };
            write_d(msg->orientation().x()); write_d(msg->orientation().y());
            write_d(msg->orientation().z()); write_d(msg->orientation().w());
            write_d(msg->angular_velocity().x()); write_d(msg->angular_velocity().y()); write_d(msg->angular_velocity().z());
            write_d(msg->linear_acceleration().x()); write_d(msg->linear_acceleration().y()); write_d(msg->linear_acceleration().z());
            shm->commit();
            uint64_t c = count->fetch_add(1) + 1;
            if (c % 200 == 1) {
                std::cout << "[drdds_recv] " << label << " #" << c
                          << " stamp=" << msg->header().stamp().sec() << "." << msg->header().stamp().nanosec()
                          << " acc=(" << msg->linear_acceleration().x()
                          << "," << msg->linear_acceleration().y()
                          << "," << msg->linear_acceleration().z() << ")" << std::endl;
            }
        };
    };

    auto on_imu_airy_front = make_imu_cb(&imu_airy_front_shm, &imu_airy_front_count, "imu_airy_front");
    auto on_imu_airy_rear = make_imu_cb(&imu_airy_rear_shm, &imu_airy_rear_count, "imu_airy_rear");

    // DrDDSChannel (pub+sub) — required for rsdriver/yesense matching.
    // Creating all 5 channels BEFORE rsdriver starts ensures the SHM-based
    // FastDDS discovery can link our subscribers to rsdriver's publishers
    // in separate mode too (Finding #5 in ROSNAV_MIGRATION_LOG.md).
    // Default 3-arg ctor (use_shm=false, topic_prefix="rt") — matches
    // Finding #2's verified-working pre-OTA call. Post-Init-fix this works
    // for all IMU channels. Lidar has a separate issue (matched=1 msgs=0)
    // that's tracked in FASTLIO2_LOG Finding #36 for follow-up.
    DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType> lidar_ch(
        on_lidar, "/LIDAR/POINTS", 0);
    DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType> lidar2_ch(
        on_lidar2, "/LIDAR/POINTS2", 0);
    DrDDSChannel<sensor_msgs::msg::ImuPubSubType> imu_ch(
        on_imu, "/IMU", 0);
    DrDDSChannel<sensor_msgs::msg::ImuPubSubType> imu_airy_front_ch(
        on_imu_airy_front, "/LIDAR/IMU201", 0);
    DrDDSChannel<sensor_msgs::msg::ImuPubSubType> imu_airy_rear_ch(
        on_imu_airy_rear, "/LIDAR/IMU202", 0);

    std::cout << "[drdds_recv] 5 channels created (lidar/lidar2/IMU/IMU201/IMU202). Waiting for data..." << std::endl;

    // Health watchdog: warn if no data for 30s
    uint64_t last_lidar = 0;
    int no_data_cycles = 0;

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        uint64_t cur = lidar_count.load();
        std::cout << "[drdds_recv] status:"
                  << " lidar_front_matched=" << lidar_ch.GetMatchedCount()
                  << " lidar_front_msgs=" << cur
                  << " lidar_rear_matched=" << lidar2_ch.GetMatchedCount()
                  << " lidar_rear_msgs=" << lidar2_count.load()
                  << " imu_yesense_matched=" << imu_ch.GetMatchedCount()
                  << " imu_yesense_msgs=" << imu_count.load()
                  << " imu_airy_front_matched=" << imu_airy_front_ch.GetMatchedCount()
                  << " imu_airy_front_msgs=" << imu_airy_front_count.load()
                  << " imu_airy_rear_matched=" << imu_airy_rear_ch.GetMatchedCount()
                  << " imu_airy_rear_msgs=" << imu_airy_rear_count.load()
                  << std::endl;

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
