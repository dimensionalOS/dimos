#include <dimos/native/cdr_codec.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
// C++ robot control example
// Subscribes to robot pose and publishes twist commands

#include <lcm/lcm-cpp.hpp>
#include <cmath>
#include <cstdio>
#include <thread>
#include <atomic>
#include <chrono>



class RobotController {
public:
    RobotController() : lcm_(), running_(true) {}

    void onPose(const lcm::ReceiveBuffer* buffer, const std::string&) {
        try {
        const auto decoded = dimos::native::cdr_decode<geometry_msgs::msg::PoseStamped>(
            static_cast<const uint8_t*>(buffer->data), buffer->data_size);
        const auto* msg = &decoded;
        const auto& pos = msg->pose.position;
        const auto& ori = msg->pose.orientation;
        printf("[pose] x=%.2f y=%.2f z=%.2f | qw=%.2f\n",
               pos.x, pos.y, pos.z, ori.w);
        } catch (const std::exception& error) {
            fprintf(stderr, "Invalid CDR pose: %s\n", error.what());
        }
    }

    void run() {
        lcm_.subscribe("/odom#geometry_msgs.PoseStamped", &RobotController::onPose, this);

        printf("Robot control started\n");
        printf("Subscribing to /odom, publishing to /cmd_vel\n");
        printf("Press Ctrl+C to stop.\n\n");

        // Publisher thread
        std::thread pub_thread([this]() {
            double t = 0;
            while (running_) {
                geometry_msgs::msg::Twist twist;
                twist.linear.x = 0.5;
                twist.linear.y = 0;
                twist.linear.z = 0;
                twist.angular.x = 0;
                twist.angular.y = 0;
                twist.angular.z = std::sin(t) * 0.3;

                const auto bytes = dimos::native::cdr_encode(twist);
                lcm_.publish("/cmd_vel#geometry_msgs.Twist", bytes.data(), bytes.size());
                printf("[twist] linear=%.2f angular=%.2f\n", twist.linear.x, twist.angular.z);
                t += 0.1;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });

        // Handle incoming messages
        while (lcm_.handle() == 0) {}

        running_ = false;
        pub_thread.join();
    }

private:
    lcm::LCM lcm_;
    std::atomic<bool> running_;
};

int main() {
    RobotController controller;
    controller.run();
    return 0;
}
