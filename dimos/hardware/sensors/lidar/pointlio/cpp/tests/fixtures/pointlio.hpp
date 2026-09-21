#pragma once
// Test-only API double. Selected exclusively by the standalone test include path.
#include <memory>
#include <utility>
#include <vector>

namespace custom_messages { struct Odometry { std::vector<double> values; }; }
struct PointCloudXYZI {
    using ConstPtr = std::shared_ptr<const PointCloudXYZI>;
    std::vector<double> points;
};
class PointLio {
public:
    bool next_success = false;
    bool null_cloud = false;
    unsigned cloud_reads = 0;
    std::vector<double> pose;
    custom_messages::Odometry odom;
    PointCloudXYZI body;
    bool process_updated() { return std::exchange(next_success, false); }
    std::vector<double> get_pose() const { return pose; }
    const custom_messages::Odometry& get_odometry() const { return odom; }
    PointCloudXYZI::ConstPtr get_body_cloud() {
        ++cloud_reads;
        if (null_cloud) return {};
        return std::make_shared<const PointCloudXYZI>(body);
    }
};
