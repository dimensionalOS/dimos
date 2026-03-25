// ros2_pub.cpp - Reads lidar + IMU data from shared memory and publishes
// as standard ROS2 Humble topics for ARISE SLAM / FAST_LIO consumption.
//
// Uses a dedicated polling thread (bypasses rclcpp executor scheduling).

#include "shm_transport.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <thread>
#include <atomic>
#include <semaphore.h>

// RSAIRY PointCloud2 fields — hardcoded, never change.
// All 6 fields preserved: FAST_LIO ignores time/ring (lidar_type 5 reads only XYZI),
// ARISE-SLAM needs time+ring for feature extraction and motion compensation.
static std::vector<sensor_msgs::msg::PointField> make_rsairy_fields() {
    std::vector<sensor_msgs::msg::PointField> fields(6);
    fields[0].name = "x";         fields[0].offset = 0;  fields[0].datatype = 7; fields[0].count = 1;
    fields[1].name = "y";         fields[1].offset = 4;  fields[1].datatype = 7; fields[1].count = 1;
    fields[2].name = "z";         fields[2].offset = 8;  fields[2].datatype = 7; fields[2].count = 1;
    fields[3].name = "intensity"; fields[3].offset = 12; fields[3].datatype = 7; fields[3].count = 1;
    fields[4].name = "ring";      fields[4].offset = 16; fields[4].datatype = 4; fields[4].count = 1;
    fields[5].name = "time";      fields[5].offset = 18; fields[5].datatype = 7; fields[5].count = 1; // FLOAT32 (ARISE PointcloudXYZITR expects f32)
    return fields;
}

class BridgePublisher : public rclcpp::Node {
public:
    BridgePublisher()
        : Node("drdds_bridge"),
          lidar_reader_(drdds_bridge::SHM_LIDAR_NAME),
          imu_reader_(drdds_bridge::SHM_IMU_NAME),
          rsairy_fields_(make_rsairy_fields())
    {
        // RELIABLE QoS to match ARISE SLAM's default subscriber (rclcpp::QoS(2)
        // is RELIABLE). SensorDataQoS (BEST_EFFORT) is incompatible — ARISE
        // won't receive any data. RELIABLE risks ACK blocking for 3MB messages
        // but works over SHM intra-container with negligible overhead.
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
        lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/bridge/LIDAR_POINTS", qos);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/bridge/IMU", rclcpp::QoS(rclcpp::KeepLast(50)).reliable());

        // Open notification semaphore (created by drdds_recv)
        notify_sem_ = sem_open(drdds_bridge::SHM_NOTIFY_NAME, O_CREAT, 0666, 0);
        if (notify_sem_ == SEM_FAILED) {
            RCLCPP_WARN(this->get_logger(), "sem_open failed, falling back to 1ms polling");
            notify_sem_ = nullptr;
        }

        RCLCPP_INFO(this->get_logger(), "Bridge publisher started. Waiting for SHM...");
        lidar_thread_ = std::thread([this]() { lidar_poll_loop(); });
        imu_thread_ = std::thread([this]() { imu_poll_loop(); });
    }

    ~BridgePublisher() {
        running_ = false;
        if (lidar_thread_.joinable()) lidar_thread_.join();
        if (imu_thread_.joinable()) imu_thread_.join();
    }

private:
    void lidar_poll_loop() {
        while (running_) {
            if (!lidar_reader_.is_open()) {
                if (!lidar_reader_.try_open()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                RCLCPP_INFO(this->get_logger(), "Lidar SHM connected");
            }

            auto* slot = lidar_reader_.poll();
            if (!slot) {
                if (notify_sem_) {
                    struct timespec ts;
                    clock_gettime(CLOCK_REALTIME, &ts);
                    ts.tv_sec += 1; // 1s timeout fallback
                    sem_timedwait(notify_sem_, &ts);
                    while (sem_trywait(notify_sem_) == 0) {} // drain accumulated
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                continue;
            }
            if (slot->msg_type != 0) continue;

            uint32_t data_size = slot->data_size;
            if (data_size + sizeof(drdds_bridge::SlotHeader) > drdds_bridge::LIDAR_SLOT_SIZE) continue;

            auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
            msg->header.stamp.sec = slot->stamp_sec;
            msg->header.stamp.nanosec = slot->stamp_nsec;
            msg->header.frame_id = "lidar_link";
            msg->height = slot->height;
            msg->width = slot->width;
            msg->point_step = slot->point_step;
            msg->row_step = slot->row_step;
            msg->is_dense = slot->is_dense;
            msg->is_bigendian = slot->is_bigendian;
            msg->fields = rsairy_fields_;

            auto t0 = std::chrono::steady_clock::now();

            const uint8_t* data = lidar_reader_.slot_data(slot);
            msg->data.resize(data_size);
            std::memcpy(msg->data.data(), data, data_size);

            // Post-process point data for ARISE SLAM compatibility:
            // 1. Make time relative (absolute f64 → relative f64, then pcl truncates to f32)
            // 2. Remap rings from 0-191 (RSAIRY 192ch) to 0-63 (ARISE N_SCANS=64)
            {
                const uint32_t pt_step = msg->point_step; // 26 bytes
                const uint32_t n_pts = msg->width * msg->height;
                constexpr size_t ring_off = 16; // ring field offset (uint16)
                constexpr size_t time_off = 18; // time field offset (float64)
                if (n_pts > 0 && pt_step > 0) {
                    // Read first point's time for relative offset
                    double first_t;
                    std::memcpy(&first_t, msg->data.data() + time_off, sizeof(double));

                    for (uint32_t i = 0; i < n_pts; i++) {
                        uint8_t* pt = msg->data.data() + i * pt_step;

                        // Remap ring: 0-191 → 0-63 (groups of 3 original rings per bin)
                        uint16_t ring;
                        std::memcpy(&ring, pt + ring_off, sizeof(uint16_t));
                        ring = static_cast<uint16_t>(ring * 64 / 192);
                        if (ring > 63) ring = 63;
                        std::memcpy(pt + ring_off, &ring, sizeof(uint16_t));

                        // Make time relative to first point, write as float32
                        // (PCL fromROSMsg can't convert f64→f32; PointcloudXYZITR expects f32)
                        double t;
                        std::memcpy(&t, pt + time_off, sizeof(double));
                        float t_f32 = static_cast<float>(t - first_t);
                        std::memcpy(pt + time_off, &t_f32, sizeof(float));
                    }
                }
            }

            auto t1 = std::chrono::steady_clock::now();

            lidar_pub_->publish(std::move(msg));

            auto t2 = std::chrono::steady_clock::now();
            auto copy_us = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
            auto pub_us = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();

            lidar_count_++;
            if (pub_us > 50000 || lidar_count_ % 100 == 1) {
                RCLCPP_INFO(this->get_logger(), "lidar #%lu copy=%ldus pub=%ldus data=%uB pts=%u",
                            lidar_count_, copy_us, pub_us, data_size,
                            slot->width * slot->height);
            }

            // IMU handled by separate thread
        }
    }

    void imu_poll_loop() {
        while (running_) {
            if (!imu_reader_.is_open()) {
                if (!imu_reader_.try_open()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                RCLCPP_INFO(this->get_logger(), "IMU SHM connected");
            }

            auto* slot = imu_reader_.poll();
            if (!slot) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            if (slot->msg_type != 1 || slot->data_size < 80) continue;

            auto msg = std::make_unique<sensor_msgs::msg::Imu>();
            msg->header.stamp.sec = slot->stamp_sec;
            msg->header.stamp.nanosec = slot->stamp_nsec;
            msg->header.frame_id = "imu_link";

            const uint8_t* d = imu_reader_.slot_data(slot);
            size_t off = 0;
            auto read_d = [&]() -> double { double v; std::memcpy(&v, d + off, 8); off += 8; return v; };

            msg->orientation.x = read_d();
            msg->orientation.y = read_d();
            msg->orientation.z = read_d();
            msg->orientation.w = read_d();
            msg->angular_velocity.x = read_d();
            msg->angular_velocity.y = read_d();
            msg->angular_velocity.z = read_d();
            msg->linear_acceleration.x = read_d();
            msg->linear_acceleration.y = read_d();
            msg->linear_acceleration.z = read_d();

            imu_pub_->publish(std::move(msg));
            imu_count_++;
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    drdds_bridge::ShmReader lidar_reader_;
    drdds_bridge::ShmReader imu_reader_;
    const std::vector<sensor_msgs::msg::PointField> rsairy_fields_;
    sem_t* notify_sem_ = nullptr;
    std::atomic<bool> running_{true};
    std::thread lidar_thread_;
    std::thread imu_thread_;
    uint64_t lidar_count_ = 0;
    uint64_t imu_count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BridgePublisher>();
    // Spin in a background thread for DDS discovery (participant announcement).
    // Without spin, ros2_pub's publishers are invisible to other ROS2 nodes.
    // The poll thread handles actual data — spin only processes discovery.
    std::thread spin_thread([&node]() {
        rclcpp::spin(node);
    });
    spin_thread.join();
    node.reset();
    rclcpp::shutdown();
    return 0;
}
