// ros2_pub.cpp - Reads lidar data from shared memory and publishes
// as standard ROS2 Humble PointCloud2 for FAST_LIO consumption.
//
// Uses a dedicated polling thread (bypasses rclcpp executor scheduling).
// With SHM-only FastDDS transport (ros2_pub_fastdds.xml), publish() takes
// ~5-10ms instead of 30-80ms, so single-thread is sufficient for 10Hz.

#include "shm_transport.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <thread>
#include <atomic>
#include <semaphore.h>

// RSAIRY PointCloud2 fields — hardcoded, never change.
static std::vector<sensor_msgs::msg::PointField> make_rsairy_fields() {
    std::vector<sensor_msgs::msg::PointField> fields(5);
    fields[0].name = "x";         fields[0].offset = 0;  fields[0].datatype = 7; fields[0].count = 1;
    fields[1].name = "y";         fields[1].offset = 4;  fields[1].datatype = 7; fields[1].count = 1;
    fields[2].name = "z";         fields[2].offset = 8;  fields[2].datatype = 7; fields[2].count = 1;
    fields[3].name = "intensity"; fields[3].offset = 12; fields[3].datatype = 7; fields[3].count = 1;
    fields[4].name = "ring";      fields[4].offset = 16; fields[4].datatype = 4; fields[4].count = 1;
    return fields;
}

class BridgePublisher : public rclcpp::Node {
public:
    BridgePublisher()
        : Node("drdds_bridge"),
          lidar_reader_(drdds_bridge::SHM_LIDAR_NAME),
          rsairy_fields_(make_rsairy_fields())
    {
        auto qos = rclcpp::SensorDataQoS();
        lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/bridge/LIDAR_POINTS", qos);

        // Open notification semaphore (created by drdds_recv)
        notify_sem_ = sem_open(drdds_bridge::SHM_NOTIFY_NAME, O_CREAT, 0666, 0);
        if (notify_sem_ == SEM_FAILED) {
            RCLCPP_WARN(this->get_logger(), "sem_open failed, falling back to 1ms polling");
            notify_sem_ = nullptr;
        }

        RCLCPP_INFO(this->get_logger(), "Bridge publisher started. Waiting for SHM...");
        poll_thread_ = std::thread([this]() { poll_loop(); });
    }

    ~BridgePublisher() {
        running_ = false;
        if (poll_thread_.joinable()) poll_thread_.join();
    }

private:
    void poll_loop() {
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
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    drdds_bridge::ShmReader lidar_reader_;
    const std::vector<sensor_msgs::msg::PointField> rsairy_fields_;
    sem_t* notify_sem_ = nullptr;
    std::atomic<bool> running_{true};
    std::thread poll_thread_;
    uint64_t lidar_count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BridgePublisher>();
    // Don't spin executor — poll thread does all work.
    // spin() would waste a CPU core on an idle executor loop.
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    node.reset();
    rclcpp::shutdown();
    return 0;
}
