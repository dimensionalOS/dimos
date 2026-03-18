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
        // QoS: RELIABLE + KEEP_LAST to match FAST_LIO's default subscriber QoS
        auto qos = rclcpp::QoS(10).reliable();

        lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/LIDAR/POINTS", qos);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/IMU", qos);

        // Poll SHM at 2kHz (500us) — keeps IMU latency jitter <500us for FAST_LIO fusion
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(500),
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
