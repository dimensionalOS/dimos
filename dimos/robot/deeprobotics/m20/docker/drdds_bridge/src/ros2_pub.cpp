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

// RSAIRY PointCloud2 fields are fixed — hardcode them to avoid
// serializing field descriptors across SHM (eliminates corruption bugs).
// Verified layout: x(f32,0) y(f32,4) z(f32,8) intensity(f32,12) ring(u16,16) time(f64,18)
static std::vector<sensor_msgs::msg::PointField> make_rsairy_fields() {
    std::vector<sensor_msgs::msg::PointField> fields(6);
    fields[0].name = "x";         fields[0].offset = 0;  fields[0].datatype = 7; fields[0].count = 1;
    fields[1].name = "y";         fields[1].offset = 4;  fields[1].datatype = 7; fields[1].count = 1;
    fields[2].name = "z";         fields[2].offset = 8;  fields[2].datatype = 7; fields[2].count = 1;
    fields[3].name = "intensity"; fields[3].offset = 12; fields[3].datatype = 7; fields[3].count = 1;
    fields[4].name = "ring";      fields[4].offset = 16; fields[4].datatype = 4; fields[4].count = 1;
    fields[5].name = "time";      fields[5].offset = 18; fields[5].datatype = 8; fields[5].count = 1;
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
        auto qos = rclcpp::QoS(10).reliable();

        lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/bridge/LIDAR_POINTS", qos);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/bridge/IMU", qos);

        // Poll SHM at 2kHz (500us)
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(500),
            std::bind(&BridgePublisher::poll_shm, this));

        RCLCPP_INFO(this->get_logger(), "Bridge publisher started. Waiting for SHM...");
    }

private:
    void poll_shm() {
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

            uint32_t data_size = slot->data_size;
            if (data_size + sizeof(drdds_bridge::SlotHeader) > drdds_bridge::LIDAR_SLOT_SIZE) {
                RCLCPP_WARN(this->get_logger(), "slot overflow: data=%u", data_size);
                continue;
            }

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
            msg.fields = rsairy_fields_;

            const uint8_t* data = lidar_reader_.slot_data(slot);
            msg.data.assign(data, data + data_size);

            lidar_pub_->publish(msg);
            lidar_count_++;
            if (lidar_count_ % 100 == 1) {
                RCLCPP_INFO(this->get_logger(), "lidar #%lu data=%uB pts=%u",
                            lidar_count_, data_size, msg.width * msg.height);
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
    const std::vector<sensor_msgs::msg::PointField> rsairy_fields_;
    uint64_t lidar_count_ = 0;
    uint64_t imu_count_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BridgePublisher>());
    rclcpp::shutdown();
    return 0;
}
