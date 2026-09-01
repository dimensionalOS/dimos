// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// DeepRobotics M20 DrDDS <-> DimOS Zenoh bridge.
//
// Runs on NOS. It forwards the vendor LIO body-frame cloud and odometry to
// typed DimOS Zenoh streams, and forwards the small DimOS control streams back
// to the robot's onboard Fast-DDS fork ("drdds").
//
// It also receives the small DimOS command streams over Zenoh and publishes the
// vendor DrDDS command topics. Dense clouds never cross an LCM boundary.
//
// This executable is deliberately fixed to the M20 deployment. Run it as root
// for access to the robot's root-owned Fast-DDS SHM writers.

#include "drdds/core/drdds_core.h"

#include "dridl/sensor_msgs/msg/PointCloud2.h"
#include "dridl/sensor_msgs/msg/PointCloud2PubSubTypes.h"
#include "dridl/nav_msgs/msg/Odometry.h"
#include "dridl/nav_msgs/msg/OdometryPubSubTypes.h"

#include "dridl/dr_msgs/msg/GaitPubSubTypes.h"
#include "dridl/dr_msgs/msg/MotionInfoPubSubTypes.h"
#include "dridl/dr_msgs/msg/MotionStatePubSubTypes.h"
#include "dridl/dr_msgs/msg/NavCmdPubSubTypes.h"
#include "dridl/dr_msgs/msg/StdMsgInt32PubSubTypes.h"

#include <zenoh.h>

#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Twist.hpp"
#include "geometry_msgs/Vector3.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "std_msgs/Bool.hpp"
#include "std_msgs/Int32.hpp"
#include "std_msgs/UInt32.hpp"
#include "tf2_msgs/TFMessage.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

static std::atomic<bool> g_running{true};
static void on_signal(int) { g_running.store(false); }

constexpr int kDomain = 0;
constexpr char kDrddsNetwork[] = "10.21.33.106";
constexpr char kZenohListen[] = "tcp/0.0.0.0:7447";

constexpr char kBodyTopic[] = "/SLAM_CLOUD_REGISTERED_BODY";
constexpr char kOdometryTopic[] = "/SLAM_ODOM";
constexpr char kNavCmdTopic[] = "/NAV_CMD";
constexpr char kMotionStateTopic[] = "/MOTION_STATE";
constexpr char kMotionInfoTopic[] = "/MOTION_INFO";
constexpr char kGaitTopic[] = "/GAIT";
constexpr char kHesStatusTopic[] = "/HES_STATUS";

constexpr char kBodyKey[] = "dimos/slam_body_points/sensor_msgs.PointCloud2";
constexpr char kOdometryKey[] = "dimos/slam_odom/nav_msgs.Odometry";
constexpr char kTfKey[] = "dimos/tf/tf2_msgs.TFMessage";
constexpr char kSafeCmdVelKey[] = "dimos/safe_cmd_vel/geometry_msgs.Twist";
constexpr char kMotionStateCmdKey[] = "dimos/motion_state_cmd/std_msgs.Int32";
constexpr char kGaitCmdKey[] = "dimos/gait_cmd/std_msgs.UInt32";
constexpr char kCommandReadyKey[] = "dimos/command_ready/std_msgs.Bool";
constexpr char kMotionStateKey[] = "dimos/motion_state/std_msgs.Int32";
constexpr char kGaitStateKey[] = "dimos/gait_state/std_msgs.UInt32";

constexpr double kCommandRateHz = 10.0;
constexpr double kCommandTimeoutS = 0.4;
constexpr double kStateTimeoutS = 2.5;
constexpr double kMaxLinearX = 2.0;
constexpr double kMaxLinearY = 1.0;
constexpr double kMaxAngularZ = 2.0;

// One wired output: its fixed Zenoh key plus counters shown in the status line.
struct Port {
    Port(std::string key_value, std::string label_value)
        : key(std::move(key_value)), label(std::move(label_value)) {}

    std::string key;    // zenoh key expr, e.g. "dimos/aligned_points/sensor_msgs.PointCloud2"
    std::string label;  // short name for logs
    std::atomic<long> n{0};
    std::atomic<long> bytes{0};
    std::atomic<bool> described{false};
    std::function<int()> matched;  // GetMatchedCount() of the underlying drdds reader
};

struct InputPort {
    InputPort(std::string key_value, std::string label_value,
              std::function<bool(const uint8_t*, size_t)> callback)
        : key(std::move(key_value)),
          label(std::move(label_value)),
          handler(std::move(callback)) {}

    std::string key;
    std::string label;
    std::function<bool(const uint8_t*, size_t)> handler;
    std::atomic<long> n{0};
    std::atomic<long> bytes{0};
    std::atomic<long> decode_errors{0};
};

struct TfGate {
    std::mutex mutex;
    std::condition_variable ready;
    int64_t latest_stamp_ns = std::numeric_limits<int64_t>::min();
};

// ----------------------------------------------------------------- zenoh out --
static z_owned_session_t g_session;
static std::mutex g_pub_mx;
static std::map<std::string, z_owned_publisher_t> g_pubs;  // key -> cached publisher
static std::map<std::string, z_owned_subscriber_t> g_subs;

// Get or declare the cached publisher for a key, matching DimOS's default
// topic QoS policy in core/transport_factory.py:
//   - high-rate clouds/images -> DROP congestion control, so a momentarily-slow
//     link (e.g. WiFi) drops stale frames instead of building a reliable
//     in-order backlog that makes every subscriber lag behind realtime.
//   - everything else (odometry, etc.) -> BLOCK, never drop under congestion.
// (zenoh-c 1.2 reliability is a no-op on the wire, but we set it for parity.)
static const z_loaned_publisher_t* get_pub(const std::string& key) {
    std::lock_guard<std::mutex> lk(g_pub_mx);
    auto it = g_pubs.find(key);
    if (it == g_pubs.end()) {
        z_view_keyexpr_t ke;
        if (z_view_keyexpr_from_str(&ke, key.c_str()) != Z_OK) {
            fprintf(stderr, "[bridge] bad key expr '%s'\n", key.c_str());
            return nullptr;
        }
        const bool is_stream = key.find("sensor_msgs.PointCloud2") != std::string::npos ||
                               key.find("sensor_msgs.Image") != std::string::npos;
        z_publisher_options_t opts;
        z_publisher_options_default(&opts);
        if (is_stream) {
            opts.congestion_control = Z_CONGESTION_CONTROL_DROP;
            opts.reliability = Z_RELIABILITY_BEST_EFFORT;
        } else {
            opts.congestion_control = Z_CONGESTION_CONTROL_BLOCK;
            opts.reliability = Z_RELIABILITY_RELIABLE;
        }
        z_owned_publisher_t pub;
        if (z_declare_publisher(z_loan(g_session), &pub, z_loan(ke), &opts) != Z_OK) {
            fprintf(stderr, "[bridge] declare_publisher failed for '%s'\n", key.c_str());
            return nullptr;
        }
        it = g_pubs.emplace(key, pub).first;
    }
    return z_loan(it->second);
}

// LCM-encode a dimos_lcm message and publish the raw bytes on the port's key.
template <class T>
static void publish_zenoh(Port* p, const T& msg) {
    const z_loaned_publisher_t* pub = get_pub(p->key);
    if (pub == nullptr) { return; }
    const int len = msg.getEncodedSize();
    if (len < 0) { return; }
    std::vector<uint8_t> buf(static_cast<size_t>(len));
    if (msg.encode(buf.data(), 0, len) != len) {
        fprintf(stderr, "[bridge] encode failed for '%s'\n", p->key.c_str());
        return;
    }
    z_owned_bytes_t payload;
    z_bytes_copy_from_buf(&payload, buf.data(), static_cast<size_t>(len));
    z_publisher_put_options_t po;
    z_publisher_put_options_default(&po);
    z_publisher_put(pub, z_move(payload), &po);
    p->n.fetch_add(1, std::memory_order_relaxed);
    p->bytes.fetch_add(len, std::memory_order_relaxed);
}

template <class T>
static bool decode_lcm(const uint8_t* data, size_t len, T* out) {
    if (len > static_cast<size_t>(std::numeric_limits<int>::max())) { return false; }
    return out->decode(data, 0, static_cast<int>(len)) >= 0;
}

static void on_zenoh_sample(z_loaned_sample_t* sample, void* context) {
    auto* port = static_cast<InputPort*>(context);
    if (sample == nullptr || port == nullptr || z_sample_kind(sample) != Z_SAMPLE_KIND_PUT) {
        return;
    }
    const z_loaned_bytes_t* payload = z_sample_payload(sample);
    const size_t len = z_bytes_len(payload);
    std::vector<uint8_t> buffer(len);
    z_bytes_reader_t reader = z_bytes_get_reader(payload);
    if (z_bytes_reader_read(&reader, buffer.data(), len) != len) {
        port->decode_errors.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    if (port->handler(buffer.data(), buffer.size())) {
        port->n.fetch_add(1, std::memory_order_relaxed);
        port->bytes.fetch_add(static_cast<long>(len), std::memory_order_relaxed);
    } else {
        port->decode_errors.fetch_add(1, std::memory_order_relaxed);
    }
}

static bool declare_zenoh_subscriber(InputPort* port) {
    z_view_keyexpr_t key;
    if (z_view_keyexpr_from_str(&key, port->key.c_str()) != Z_OK) {
        fprintf(stderr, "[bridge] bad subscriber key expr '%s'\n", port->key.c_str());
        return false;
    }
    z_owned_closure_sample_t callback;
    z_closure(&callback, on_zenoh_sample, nullptr, port);
    z_subscriber_options_t options;
    z_subscriber_options_default(&options);
    z_owned_subscriber_t subscriber;
    if (z_declare_subscriber(z_loan(g_session), &subscriber, z_loan(key),
                             z_move(callback), &options) != Z_OK) {
        fprintf(stderr, "[bridge] declare_subscriber failed for '%s'\n", port->key.c_str());
        return false;
    }
    g_subs.emplace(port->key, subscriber);
    fprintf(stderr, "[bridge] %s: %s -> Zenoh input\n", port->label.c_str(),
            port->key.c_str());
    return true;
}

// ----------------------------------------------------- drdds -> dimos_lcm conv --
// (identical field-for-field copies to ../../dds/cpp/main.cpp; the drdds and
// dimos_lcm ROS-message layouts match, so these are straight member copies.)

static std_msgs::Header to_lcm_header(const std_msgs::msg::Header& h) {
    static std::atomic<int32_t> seq{0};
    std_msgs::Header out;
    out.seq = seq.fetch_add(1, std::memory_order_relaxed);
    out.stamp.sec = h.stamp().sec();
    out.stamp.nsec = static_cast<int32_t>(h.stamp().nanosec());
    out.frame_id = h.frame_id();
    return out;
}

static int64_t stamp_ns(const std_msgs::msg::Header& h) {
    return static_cast<int64_t>(h.stamp().sec()) * 1'000'000'000LL +
           static_cast<int64_t>(h.stamp().nanosec());
}

static void on_pointcloud(const sensor_msgs::msg::PointCloud2* m, Port* p,
                          TfGate* tf_gate) {
    if (m == nullptr) { return; }
    if (tf_gate != nullptr) {
        const int64_t cloud_stamp_ns = stamp_ns(m->header());
        std::unique_lock<std::mutex> lock(tf_gate->mutex);
        if (!tf_gate->ready.wait_for(lock, std::chrono::milliseconds(200), [&] {
                return tf_gate->latest_stamp_ns >= cloud_stamp_ns;
            })) {
            return;
        }
    }
    if (!p->described.exchange(true)) {
        fprintf(stderr, "[bridge] first body cloud: frame=%s points=%ux%u step=%u\n",
                m->header().frame_id().c_str(), m->width(), m->height(), m->point_step());
    }
    sensor_msgs::PointCloud2 pc;
    pc.header = to_lcm_header(m->header());
    pc.height = m->height();
    pc.width = m->width();
    pc.is_bigendian = m->is_bigendian();
    pc.point_step = m->point_step();
    pc.row_step = m->row_step();
    pc.is_dense = m->is_dense();

    const auto& fields = m->fields();
    pc.fields_length = static_cast<int32_t>(fields.size());
    pc.fields.resize(fields.size());
    for (size_t i = 0; i < fields.size(); ++i) {
        pc.fields[i].name = fields[i].name();
        pc.fields[i].offset = fields[i].offset();
        pc.fields[i].datatype = static_cast<int8_t>(fields[i].datatype());
        pc.fields[i].count = fields[i].count();
    }

    const auto& data = m->data();
    pc.data.resize(data.size());
    if (!data.empty()) {
        std::memcpy(pc.data.data(), data.data(), data.size());
    }
    pc.data_length = static_cast<int32_t>(pc.data.size());
    publish_zenoh(p, pc);
}

static void on_odometry(const nav_msgs::msg::Odometry* m, Port* odometry_port,
                        Port* tf_port, TfGate* tf_gate) {
    if (m == nullptr) { return; }
    if (!odometry_port->described.exchange(true)) {
        fprintf(stderr, "[bridge] first odometry: frame=%s child=%s\n",
                m->header().frame_id().c_str(), m->child_frame_id().c_str());
    }
    nav_msgs::Odometry out;
    out.header = to_lcm_header(m->header());
    out.child_frame_id = m->child_frame_id();

    const auto& pose = m->pose().pose();
    out.pose.pose.position.x = pose.position().x();
    out.pose.pose.position.y = pose.position().y();
    out.pose.pose.position.z = pose.position().z();
    out.pose.pose.orientation.x = pose.orientation().x();
    out.pose.pose.orientation.y = pose.orientation().y();
    out.pose.pose.orientation.z = pose.orientation().z();
    out.pose.pose.orientation.w = pose.orientation().w();

    const auto& tw = m->twist().twist();
    out.twist.twist.linear.x = tw.linear().x();
    out.twist.twist.linear.y = tw.linear().y();
    out.twist.twist.linear.z = tw.linear().z();
    out.twist.twist.angular.x = tw.angular().x();
    out.twist.twist.angular.y = tw.angular().y();
    out.twist.twist.angular.z = tw.angular().z();

    const auto& pcov = m->pose().covariance();
    const auto& tcov = m->twist().covariance();
    for (int i = 0; i < 36; ++i) {
        out.pose.covariance[i] = pcov[i];
        out.twist.covariance[i] = tcov[i];
    }
    // Publish the transform from the same NOS callback, before the matching
    // body cloud can reach the mapper. Generating it one process later from
    // `slam_odom` lets the cloud handler block the mapper's dispatch loop while
    // the required TF is already queued behind it.
    tf2_msgs::TFMessage tf;
    tf.transforms_length = 1;
    tf.transforms.resize(1);
    auto& transform = tf.transforms[0];
    transform.header = out.header;
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = out.pose.pose.position.x;
    transform.transform.translation.y = out.pose.pose.position.y;
    transform.transform.translation.z = out.pose.pose.position.z;
    transform.transform.rotation = out.pose.pose.orientation;
    publish_zenoh(tf_port, tf);
    {
        std::lock_guard<std::mutex> lock(tf_gate->mutex);
        tf_gate->latest_stamp_ns = stamp_ns(m->header());
    }
    tf_gate->ready.notify_all();
    publish_zenoh(odometry_port, out);
}

// ---------------------------------------------------------- Zenoh -> DrDDS --

namespace {

using Clock = std::chrono::steady_clock;
constexpr int64_t kNanosecondsPerSecond = 1'000'000'000LL;
constexpr int kMotionRlControl = 17;

double clamp(double value, double limit) {
    return std::max(-limit, std::min(limit, value));
}

geometry_msgs::Twist zero_twist() {
    geometry_msgs::Twist result;
    result.linear.x = 0.0;
    result.linear.y = 0.0;
    result.linear.z = 0.0;
    result.angular.x = 0.0;
    result.angular.y = 0.0;
    result.angular.z = 0.0;
    return result;
}

void set_vendor_header(drdds::msg::MetaType& header, uint64_t frame_id) {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const int64_t total_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
    header.frame_id(frame_id);
    header.timestamp().sec(static_cast<int32_t>(total_ns / kNanosecondsPerSecond));
    header.timestamp().nsec(static_cast<uint32_t>(total_ns % kNanosecondsPerSecond));
}

class M20ControlBridge {
public:
    M20ControlBridge(Port* command_ready, Port* motion_state, Port* gait_state)
        : command_ready_(command_ready),
          motion_state_(motion_state),
          gait_state_(gait_state) {}

    ~M20ControlBridge() { stop(); }

    void start() {
        // Robot motion runs on AOS, so these topics cross to NOS over the
        // vendor's UDP DDS transport. The LIO outputs below remain local SHM.
        constexpr bool use_shm = false;
        hes_subscription_ = std::make_unique<
            DrDDSChannel<drdds::msg::StdMsgInt32PubSubType>>(
            [this](const drdds::msg::StdMsgInt32* msg) { on_hes_status(msg); },
            kHesStatusTopic, kDomain, use_shm, "rt");
        motion_info_subscription_ = std::make_unique<
            DrDDSChannel<drdds::msg::MotionInfoPubSubType>>(
            [this](const drdds::msg::MotionInfo* msg) { on_motion_info(msg); },
            kMotionInfoTopic, kDomain, use_shm, "rt");
        nav_cmd_publisher_ =
            std::make_unique<DrDDSChannel<drdds::msg::NavCmdPubSubType>>(
                kNavCmdTopic, kDomain, use_shm, "rt");
        motion_state_publisher_ =
            std::make_unique<DrDDSChannel<drdds::msg::MotionStatePubSubType>>(
                kMotionStateTopic, kDomain, use_shm, "rt");
        gait_publisher_ = std::make_unique<DrDDSChannel<drdds::msg::GaitPubSubType>>(
            kGaitTopic, kDomain, use_shm, "rt");
        command_ready_->matched = [this] { return nav_cmd_publisher_->GetMatchedCount(); };
        motion_state_->matched =
            [this] { return motion_info_subscription_->GetMatchedCount(); };
        gait_state_->matched =
            [this] { return motion_info_subscription_->GetMatchedCount(); };
        command_thread_ = std::thread([this]() { command_loop(); });
        fprintf(stderr, "[bridge] bidirectional M20 command/state path started\n");
    }

    void stop() {
        if (stopped_.exchange(true)) { return; }
        if (command_thread_.joinable()) { command_thread_.join(); }
        publish_cycle(true);
        nav_cmd_publisher_.reset();
        motion_state_publisher_.reset();
        gait_publisher_.reset();
        hes_subscription_.reset();
        motion_info_subscription_.reset();
    }

    bool on_command(const uint8_t* data, size_t len) {
        geometry_msgs::Twist source;
        if (!decode_lcm(data, len, &source)) { return false; }
        geometry_msgs::Twist bounded = zero_twist();
        if (std::isfinite(source.linear.x) && std::isfinite(source.linear.y) &&
            std::isfinite(source.angular.z)) {
            bounded.linear.x = clamp(source.linear.x, kMaxLinearX);
            bounded.linear.y = clamp(source.linear.y, kMaxLinearY);
            bounded.angular.z = clamp(source.angular.z, kMaxAngularZ);
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_command_ = bounded;
        command_received_at_ = Clock::now();
        command_path_active_ = true;
        return true;
    }

    bool on_motion_state_command(const uint8_t* data, size_t len) {
        std_msgs::Int32 source;
        if (!decode_lcm(data, len, &source)) { return false; }
        if (motion_state_publisher_ == nullptr) { return true; }
        drdds::msg::MotionState output;
        set_vendor_header(output.header(), command_sequence_.fetch_add(1));
        output.data().state(source.data);
        motion_state_publisher_->Write(&output);
        fprintf(stderr, "[bridge] published motion-state command: %d\n", source.data);
        return true;
    }

    bool on_gait_command(const uint8_t* data, size_t len) {
        std_msgs::UInt32 source;
        if (!decode_lcm(data, len, &source)) { return false; }
        if (gait_publisher_ == nullptr) { return true; }
        drdds::msg::Gait output;
        set_vendor_header(output.header(), command_sequence_.fetch_add(1));
        output.data().gait(source.data);
        gait_publisher_->Write(&output);
        fprintf(stderr, "[bridge] published gait command: %u\n", source.data);
        return true;
    }

private:
    void on_hes_status(const drdds::msg::StdMsgInt32* source) {
        if (source == nullptr) { return; }
        std::lock_guard<std::mutex> lock(state_mutex_);
        hes_status_ = source->value();
        have_hes_ = true;
    }

    void on_motion_info(const drdds::msg::MotionInfo* source) {
        if (source == nullptr) { return; }
        const int motion_state = source->data().motion_state().state();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            motion_state_value_ = motion_state;
            motion_info_received_at_ = Clock::now();
            have_motion_info_ = true;
        }
        std_msgs::Int32 motion;
        motion.data = motion_state;
        publish_zenoh(motion_state_, motion);
        std_msgs::UInt32 gait;
        gait.data = source->data().gait_state().gait();
        publish_zenoh(gait_state_, gait);
    }

    bool robot_ready(Clock::time_point now) const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto timeout = std::chrono::duration<double>(kStateTimeoutS);
        return have_motion_info_ && now - motion_info_received_at_ <= timeout &&
               motion_state_value_ == kMotionRlControl && (!have_hes_ || hes_status_ == 0);
    }

    void publish_nav(const geometry_msgs::Twist& command) {
        if (nav_cmd_publisher_ == nullptr) { return; }
        drdds::msg::NavCmd output;
        set_vendor_header(output.header(), command_sequence_.fetch_add(1));
        output.data().x_vel(static_cast<float>(command.linear.x));
        output.data().y_vel(static_cast<float>(command.linear.y));
        output.data().yaw_vel(static_cast<float>(command.angular.z));
        nav_cmd_publisher_->Write(&output);
    }

    void publish_final_zero() {
        const geometry_msgs::Twist zero = zero_twist();
        publish_nav(zero);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        publish_nav(zero);
    }

    void command_loop() {
        const auto period = std::chrono::duration<double>(1.0 / kCommandRateHz);
        while (!stopped_.load()) {
            const auto next = Clock::now() + period;
            publish_cycle(false);
            std::this_thread::sleep_until(next);
        }
    }

    void publish_cycle(bool stopping) {
        const auto now = Clock::now();
        const bool ready = !stopping && nav_cmd_publisher_ != nullptr &&
                           nav_cmd_publisher_->GetMatchedCount() > 0 &&
                           robot_ready(now);
        std_msgs::Bool ready_message;
        ready_message.data = static_cast<int8_t>(ready);
        publish_zenoh(command_ready_, ready_message);

        geometry_msgs::Twist command = zero_twist();
        bool publish = false;
        bool release = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (command_path_active_) {
                const auto timeout = std::chrono::duration<double>(kCommandTimeoutS);
                if (stopping || now - command_received_at_ > timeout) {
                    command_path_active_ = false;
                    release = true;
                } else {
                    command = ready ? latest_command_ : zero_twist();
                    publish = true;
                }
            }
        }
        if (release) {
            publish_final_zero();
        } else if (publish) {
            publish_nav(command);
        }
    }

    Port* command_ready_;
    Port* motion_state_;
    Port* gait_state_;
    std::unique_ptr<DrDDSChannel<drdds::msg::StdMsgInt32PubSubType>> hes_subscription_;
    std::unique_ptr<DrDDSChannel<drdds::msg::MotionInfoPubSubType>>
        motion_info_subscription_;
    std::unique_ptr<DrDDSChannel<drdds::msg::NavCmdPubSubType>> nav_cmd_publisher_;
    std::unique_ptr<DrDDSChannel<drdds::msg::MotionStatePubSubType>>
        motion_state_publisher_;
    std::unique_ptr<DrDDSChannel<drdds::msg::GaitPubSubType>> gait_publisher_;
    std::thread command_thread_;
    mutable std::mutex state_mutex_;
    geometry_msgs::Twist latest_command_ = zero_twist();
    Clock::time_point command_received_at_{};
    Clock::time_point motion_info_received_at_{};
    bool command_path_active_ = false;
    bool have_hes_ = false;
    bool have_motion_info_ = false;
    int motion_state_value_ = 0;
    int hes_status_ = 1;
    std::atomic<bool> stopped_{false};
    std::atomic<uint64_t> command_sequence_{0};
};

}  // namespace

int main() {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    // NOS is the fixed hub between robot-side peers and the operator client.
    // Router mode prevents offboard clients from gossiping directly with every
    // GOS process (and then trying those processes' loopback-only locators).
    z_owned_config_t cfg;
    z_config_default(&cfg);
    zc_config_insert_json5(z_config_loan_mut(&cfg), "mode", "\"router\"");
    zc_config_insert_json5(z_config_loan_mut(&cfg),
                           "scouting/multicast/enabled", "false");
    zc_config_insert_json5(z_config_loan_mut(&cfg),
                           "scouting/gossip/enabled", "false");
    const std::string listen = "[\"" + std::string(kZenohListen) + "\"]";
    zc_config_insert_json5(z_config_loan_mut(&cfg), "listen/endpoints", listen.c_str());
    if (z_open(&g_session, z_move(cfg), nullptr) != Z_OK) {
        fprintf(stderr, "[bridge] zenoh session open failed\n");
        return 1;
    }
    fprintf(stderr, "[bridge] listening on %s\n", kZenohListen);

    DrDDSManager::Init(kDomain, kDrddsNetwork);

    TfGate tf_gate;
    Port body(kBodyKey, "body");
    Port tf(kTfKey, "tf");
    Port odometry(kOdometryKey, "odometry");
    Port command_ready(kCommandReadyKey, "command_ready");
    Port motion_state(kMotionStateKey, "motion_state");
    Port gait_state(kGaitStateKey, "gait_state");
    std::vector<Port*> output_ports{
        &body, &tf, &odometry, &command_ready, &motion_state, &gait_state,
    };

    constexpr bool use_shm = true;
    auto body_channel =
        std::make_unique<DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>>(
            [&body, &tf_gate](const sensor_msgs::msg::PointCloud2* message) {
                on_pointcloud(message, &body, &tf_gate);
            },
            kBodyTopic, kDomain, use_shm, "rt");
    auto odometry_channel =
        std::make_unique<DrDDSChannel<nav_msgs::msg::OdometryPubSubType>>(
            [&odometry, &tf, &tf_gate](const nav_msgs::msg::Odometry* message) {
                on_odometry(message, &odometry, &tf, &tf_gate);
            },
            kOdometryTopic, kDomain, use_shm, "rt");
    body.matched = [&body_channel] { return body_channel->GetMatchedCount(); };
    odometry.matched = [&odometry_channel] {
        return odometry_channel->GetMatchedCount();
    };
    tf.matched = odometry.matched;

    M20ControlBridge control(&command_ready, &motion_state, &gait_state);
    control.start();

    InputPort safe_cmd_vel(
        kSafeCmdVelKey, "safe_cmd_vel",
        [&control](const uint8_t* data, size_t len) {
            return control.on_command(data, len);
        });
    InputPort motion_state_cmd(
        kMotionStateCmdKey, "motion_state_cmd",
        [&control](const uint8_t* data, size_t len) {
            return control.on_motion_state_command(data, len);
        });
    InputPort gait_cmd(
        kGaitCmdKey, "gait_cmd",
        [&control](const uint8_t* data, size_t len) {
            return control.on_gait_command(data, len);
        });
    std::vector<InputPort*> input_ports{&safe_cmd_vel, &motion_state_cmd, &gait_cmd};
    for (InputPort* port : input_ports) {
        if (!declare_zenoh_subscriber(port)) { g_running.store(false); }
    }

    fprintf(stderr, "[bridge] body: rt%s -> %s\n", kBodyTopic, kBodyKey);
    fprintf(stderr, "[bridge] odometry: rt%s -> %s + %s\n", kOdometryTopic,
            kOdometryKey, kTfKey);
    fprintf(stderr, "[bridge] Fast-DDS SHM enabled, domain %d\n", kDomain);
    long t = 0;
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ++t;
        std::string line = "t=" + std::to_string(t) + "s";
        for (const Port* p : output_ports) {
            char b[96];
            const int m = p->matched ? p->matched() : -1;
            snprintf(b, sizeof(b), "  %s[m=%d n=%ld %.1fMB]", p->label.c_str(), m, p->n.load(),
                     p->bytes.load() / 1e6);
            line += b;
        }
        for (const InputPort* p : input_ports) {
            char b[96];
            snprintf(b, sizeof(b), "  %s[in=%ld err=%ld]", p->label.c_str(), p->n.load(),
                     p->decode_errors.load());
            line += b;
        }
        fprintf(stderr, "%s\n", line.c_str());
    }

    fprintf(stderr, "[bridge] shutting down\n");
    for (auto& entry : g_subs) { z_drop(z_move(entry.second)); }
    g_subs.clear();
    control.stop();
    body_channel.reset();
    odometry_channel.reset();
    DrDDSManager::Delete();
    {
        std::lock_guard<std::mutex> lock(g_pub_mx);
        for (auto& entry : g_pubs) { z_drop(z_move(entry.second)); }
        g_pubs.clear();
    }
    z_drop(z_move(g_session));
    return 0;
}
