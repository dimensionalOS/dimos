#pragma once

// Main-thread only. Preserve pending updates across rate-limited iterations;
// each output independently consumes the newest available estimate once.
class PublicationGate {
public:
    void observe(bool updated) {
        if (updated) lidar_pending_ = odom_pending_ = true;
    }
    bool lidar_pending() const { return lidar_pending_; }
    bool odom_pending() const { return odom_pending_; }
    void lidar_published() { lidar_pending_ = false; }
    void odom_published() { odom_pending_ = false; }
private:
    bool lidar_pending_ = false;
    bool odom_pending_ = false;
};
