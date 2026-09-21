#pragma once
#include "pointlio.hpp"
#include "publication_gate.hpp"

// Main-thread only; callers hold the same estimator mutex as process_updated.
// get_body_cloud returns a newly allocated cloud; retain it as const, not the
// estimator's mutable internal storage. There is at most one pending generation.
class PublicationSnapshot {
public:
    bool update(PointLio& core, bool capture_cloud) {
        if (!core.process_updated()) return false;
        auto next_pose = core.get_pose();
        auto next_odom = core.get_odometry();
        PointCloudXYZI::ConstPtr next_cloud;
        if (capture_cloud) next_cloud = core.get_body_cloud();
        pose_ = std::move(next_pose);
        odom_ = std::move(next_odom);
        cloud_ = std::move(next_cloud);
        gate_.observe(true);
        ++generation_;
        return true;
    }
    const std::vector<double>& pose() const { return pose_; }
    const custom_messages::Odometry& odometry() const { return odom_; }
    PointCloudXYZI::ConstPtr cloud() const { return cloud_; }
    unsigned long long generation() const { return generation_; }
    bool lidar_pending() const { return gate_.lidar_pending(); }
    bool odom_pending() const { return gate_.odom_pending(); }
    void lidar_published() { gate_.lidar_published(); }
    void odom_published() { gate_.odom_published(); }
private:
    PublicationGate gate_;
    std::vector<double> pose_;
    custom_messages::Odometry odom_{};
    PointCloudXYZI::ConstPtr cloud_;
    unsigned long long generation_ = 0;
};
