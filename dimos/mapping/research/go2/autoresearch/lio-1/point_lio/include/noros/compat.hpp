// noros/compat.hpp — minimal stand-ins for ros::, sensor_msgs::,
// nav_msgs::, geometry_msgs::, visualization_msgs::, std_msgs::, tf::
// and pcl_conversions:: just enough surface that point_lio_unilidar's
// .cpp/.hpp compiles without ROS installed.
//
// Design notes:
//
// * Almost everything is a POD with public data members exactly named
//   like the ROS message field (header.stamp, linear_acceleration.x, …).
//   The point-LIO source pokes those fields directly.
// * ros::Publisher::publish(...) is a no-op. The point-LIO outputs we
//   actually want (the registered cloud → PCD file, the per-frame odom
//   → ASCII log) are written from the offline main loop, not via these
//   publishers.
// * ros::Subscriber is a tag struct. Our offline main bypasses the
//   subscriber→callback path entirely and pushes data into
//   lidar_buffer/time_buffer/imu_deque directly. So nh.subscribe(...)
//   returns an empty Subscriber and that's fine.
// * pcl::fromROSMsg / pcl::toROSMsg are stubs that do nothing — they
//   only need to compile, since at runtime we never call them (the
//   ingestion path that would invoke them, standard_pcl_cbk →
//   Preprocess::unilidar_handler, is bypassed by the offline main).
// * ros::ok() flips false when our signal handler fires.
// * Headers under include/ros/ros.h, include/sensor_msgs/Imu.h, etc.
//   each just #include this file so the existing source's #include
//   lines work unchanged.

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// In real ROS, <pcl_conversions/pcl_conversions.h> transitively pulls
// these in. We don't, so make sure the upstream code sees PCL types
// through any of our shims.
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace ros {

inline std::atomic<bool>& _ok_flag() {
    static std::atomic<bool> v{true};
    return v;
}

struct Time {
    int32_t sec = 0;
    uint32_t nsec = 0;
    Time() = default;
    explicit Time(double s) { fromSec_(s); }
    static Time fromSec(double s) { Time t; t.fromSec_(s); return t; }
    void fromSec_(double s) {
        sec = (int32_t)std::floor(s);
        nsec = (uint32_t)std::llround((s - sec) * 1e9);
        if (nsec >= 1000000000u) { sec++; nsec -= 1000000000u; }
    }
    double toSec() const { return (double)sec + (double)nsec * 1e-9; }
};

struct Duration {
    double sec = 0.0;
    Duration() = default;
    Duration(double s) : sec(s) {}
    double toSec() const { return sec; }
};

struct Subscriber {};
struct Publisher {
    template <typename M>
    void publish(const M&) const {}
};

struct NodeHandle {
    NodeHandle() = default;
    explicit NodeHandle(const std::string&) {}
    // param<T>(name, var, default) — no-op; in the offline binary the
    // values are pre-loaded via readParametersYaml() before the original
    // readParameters(nh) wrapper is called, so nothing flows through nh.
    template <typename T>
    bool param(const std::string&, T&, const T&) const { return false; }
    template <typename T>
    bool param(const std::string&, T&, const T&) { return false; }
    template <typename T>
    bool getParam(const std::string&, T&) const { return false; }
    // subscribe / advertise return inert tags. The offline main bypasses
    // the callback layer and pushes data directly into the global
    // buffers.
    template <typename M, typename F>
    Subscriber subscribe(const std::string&, uint32_t, F) const { return {}; }
    template <typename M>
    Publisher advertise(const std::string&, uint32_t) const { return {}; }
};

template <typename M> struct MessageEvent {};

inline void init(int&, char**, const std::string&) {}
inline bool ok() { return _ok_flag().load(); }
inline void shutdown() { _ok_flag().store(false); }
inline void spinOnce() {}

struct Rate {
    std::chrono::steady_clock::time_point next;
    std::chrono::nanoseconds period;
    explicit Rate(double hz)
        : next(std::chrono::steady_clock::now()),
          period(std::chrono::nanoseconds((long long)(1e9 / hz))) {
        next += period;
    }
    void sleep() {
        std::this_thread::sleep_until(next);
        next += period;
    }
    void reset() {
        next = std::chrono::steady_clock::now() + period;
    }
};

namespace console { enum { Info, Warn, Error, Debug, Fatal }; }

}  // namespace ros

// ROS_* macros → printf
#define ROS_INFO(fmt, ...)  do { std::printf("[info] " fmt "\n", ##__VA_ARGS__); } while (0)
#define ROS_WARN(fmt, ...)  do { std::printf("[warn] " fmt "\n", ##__VA_ARGS__); } while (0)
#define ROS_ERROR(fmt, ...) do { std::fprintf(stderr, "[error] " fmt "\n", ##__VA_ARGS__); } while (0)
#define ROS_FATAL(fmt, ...) do { std::fprintf(stderr, "[fatal] " fmt "\n", ##__VA_ARGS__); } while (0)
#define ROS_ASSERT(cond)    do { if (!(cond)) { std::fprintf(stderr, "[assert] %s\n", #cond); std::abort(); } } while (0)
#define ROS_INFO_STREAM(x)  do { std::cout << "[info] " << x << "\n"; } while (0)


namespace std_msgs {

struct Header {
    uint32_t seq = 0;
    ros::Time stamp;
    std::string frame_id;
};

}  // namespace std_msgs


namespace geometry_msgs {

struct Vector3 { double x = 0, y = 0, z = 0; };
struct Point   { double x = 0, y = 0, z = 0; };
struct Quaternion { double x = 0, y = 0, z = 0, w = 1; };

struct Pose {
    Point position;
    Quaternion orientation;
};

struct PoseStamped {
    std_msgs::Header header;
    Pose pose;
};

struct Twist {
    Vector3 linear;
    Vector3 angular;
};

struct PoseWithCovariance {
    Pose pose;
    std::array<double, 36> covariance{};
};

struct TwistWithCovariance {
    Twist twist;
    std::array<double, 36> covariance{};
};

}  // namespace geometry_msgs


namespace sensor_msgs {

struct PointField {
    static constexpr uint8_t INT8 = 1, UINT8 = 2, INT16 = 3, UINT16 = 4,
                              INT32 = 5, UINT32 = 6, FLOAT32 = 7, FLOAT64 = 8;
    std::string name;
    uint32_t offset = 0;
    uint8_t datatype = 0;
    uint32_t count = 1;
};

struct PointCloud2 {
    std_msgs::Header header;
    uint32_t height = 0;
    uint32_t width = 0;
    std::vector<PointField> fields;
    bool is_bigendian = false;
    uint32_t point_step = 0;
    uint32_t row_step = 0;
    std::vector<uint8_t> data;
    bool is_dense = false;
    typedef std::shared_ptr<PointCloud2> Ptr;
    typedef std::shared_ptr<const PointCloud2> ConstPtr;
};

struct Imu {
    std_msgs::Header header;
    geometry_msgs::Quaternion orientation;
    std::array<double, 9> orientation_covariance{};
    geometry_msgs::Vector3 angular_velocity;
    std::array<double, 9> angular_velocity_covariance{};
    geometry_msgs::Vector3 linear_acceleration;
    std::array<double, 9> linear_acceleration_covariance{};
    typedef std::shared_ptr<Imu> Ptr;
    typedef std::shared_ptr<const Imu> ConstPtr;
};

typedef Imu::ConstPtr ImuConstPtr;

}  // namespace sensor_msgs


namespace nav_msgs {

struct Odometry {
    std_msgs::Header header;
    std::string child_frame_id;
    geometry_msgs::PoseWithCovariance pose;
    geometry_msgs::TwistWithCovariance twist;
};

struct Path {
    std_msgs::Header header;
    std::vector<geometry_msgs::PoseStamped> poses;
};

}  // namespace nav_msgs


namespace visualization_msgs {

struct Marker {
    std_msgs::Header header;
    std::string ns;
    int32_t id = 0;
    int32_t type = 0;
    int32_t action = 0;
    geometry_msgs::Pose pose;
    geometry_msgs::Vector3 scale;
    struct { float r = 0, g = 0, b = 0, a = 1; } color;
    ros::Duration lifetime;
    bool frame_locked = false;
    std::vector<geometry_msgs::Point> points;
    std::vector<decltype(color)> colors;
    std::string text;
    std::string mesh_resource;
    bool mesh_use_embedded_materials = false;
};

}  // namespace visualization_msgs


// Minimal tf:: shims. point-LIO uses these only to broadcast pose for
// rviz — we drop the broadcasts entirely (br.sendTransform is a no-op).
namespace tf {

struct Vector3 {
    double x_ = 0, y_ = 0, z_ = 0;
    Vector3() = default;
    Vector3(double x, double y, double z) : x_(x), y_(y), z_(z) {}
    double x() const { return x_; } double y() const { return y_; } double z() const { return z_; }
};

struct Quaternion {
    double x_ = 0, y_ = 0, z_ = 0, w_ = 1;
    void setX(double v) { x_ = v; } void setY(double v) { y_ = v; } void setZ(double v) { z_ = v; } void setW(double v) { w_ = v; }
    void setRPY(double r, double p, double y) {
        double cr = std::cos(r * 0.5), sr = std::sin(r * 0.5);
        double cp = std::cos(p * 0.5), sp = std::sin(p * 0.5);
        double cy = std::cos(y * 0.5), sy = std::sin(y * 0.5);
        w_ = cr*cp*cy + sr*sp*sy;
        x_ = sr*cp*cy - cr*sp*sy;
        y_ = cr*sp*cy + sr*cp*sy;
        z_ = cr*cp*sy - sr*sp*cy;
    }
};

struct Transform {
    Vector3 origin_;
    Quaternion rotation_;
    void setOrigin(const Vector3& v) { origin_ = v; }
    void setRotation(const Quaternion& q) { rotation_ = q; }
};

struct StampedTransform {
    Transform tf_;
    ros::Time stamp_;
    std::string frame_id_, child_frame_id_;
    StampedTransform() = default;
    StampedTransform(const Transform& t, const ros::Time& s,
                     const std::string& f, const std::string& c)
        : tf_(t), stamp_(s), frame_id_(f), child_frame_id_(c) {}
};

struct TransformBroadcaster {
    void sendTransform(const StampedTransform&) const {}
};

inline geometry_msgs::Quaternion createQuaternionMsgFromRollPitchYaw(double r, double p, double y) {
    Quaternion q; q.setRPY(r, p, y);
    geometry_msgs::Quaternion m; m.x = q.x_; m.y = q.y_; m.z = q.z_; m.w = q.w_;
    return m;
}

}  // namespace tf


// pcl_conversions stubs. point-LIO only calls fromROSMsg in the lidar
// callback (which we bypass in the offline main) and toROSMsg in the
// publishers (which are no-ops). Both stubbed.
namespace pcl {

template <typename PCT>
inline void fromROSMsg(const sensor_msgs::PointCloud2&, PCT&) {
    // unreachable at runtime when driving from the offline main; the
    // caller (Preprocess::*_handler) is bypassed.
}

template <typename PCT>
inline void toROSMsg(const PCT&, sensor_msgs::PointCloud2&) {
    // no-op; the corresponding publish() is also a no-op.
}

}  // namespace pcl
