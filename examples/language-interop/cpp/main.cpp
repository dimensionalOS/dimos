// C++ robot control example
// Subscribes to robot odometry and publishes twist commands

#include <lcm/lcm-cpp.hpp>
#include <cmath>
#include <cstdio>
#include <thread>
#include <atomic>
#include <chrono>

#include "geometry_msgs/Twist.hpp"
#include "nav_msgs/Odometry.hpp"

class RobotController {
public:
    RobotController() : lcm_(), running_(true) {}

    void onOdometry(const lcm::ReceiveBuffer*, const std::string&,
                    const nav_msgs::Odometry* msg) {
        const auto& pos = msg->pose.pose.position;
        const auto& ori = msg->pose.pose.orientation;
        const auto& lin = msg->twist.twist.linear;
        printf("[odometry] x=%.2f y=%.2f z=%.2f | qw=%.2f | vx=%.2f\n",
               pos.x, pos.y, pos.z, ori.w, lin.x);
    }

    void run() {
        lcm_.subscribe("/odometry#nav_msgs.Odometry", &RobotController::onOdometry, this);

        printf("Robot control started\n");
        printf("Subscribing to /odometry, publishing to /cmd_vel\n");
        printf("Press Ctrl+C to stop.\n\n");

        // Publisher thread
        std::thread pub_thread([this]() {
            double t = 0;
            while (running_) {
                geometry_msgs::Twist twist;
                twist.linear.x = 0.5;
                twist.linear.y = 0;
                twist.linear.z = 0;
                twist.angular.x = 0;
                twist.angular.y = 0;
                twist.angular.z = std::sin(t) * 0.3;

                lcm_.publish("/cmd_vel#geometry_msgs.Twist", &twist);
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
