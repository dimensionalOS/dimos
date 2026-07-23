// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// DeepRobotics M20 "drdds" -> dimos Zenoh bridge (NativeModule binary).
//
// Runs *on the M20*. Subscribes to the robot's onboard Fast-DDS fork ("drdds")
// topics under the ROS `rt/` namespace, converts each sample to the matching
// dimos_lcm type, LCM-encodes it, and publishes the bytes over Zenoh on the
// dimos key for that topic (`dimos/<name>/<pkg.Type>`). dimos's Zenoh transport
// on the consumer side (`dimos.protocol.pubsub.impl.zenohpubsub.Zenoh`) decodes
// the same bytes back into typed messages.
//
// Same per-sample conversions as the sibling LCM bridge (../../dds/cpp/main.cpp);
// only the carrier differs: reliable Zenoh unicast (auto-discovered, retransmits
// at each hop) instead of LCM udpm multicast, which silently drops the dense
// cloud bursts (measured ~87% loss on the M20's NOS->GEN path).
//
// The Python NativeModule wrapper passes each output port's dimos channel as
// `--<port> <topic>#<msg_type>` and the drdds source topic as `--<port>_topic`.
// We map the LCM channel to the Zenoh key by swapping '#' -> '/' (exactly what
// dimos's `_topic_to_key_expr` does).
//
// Usage (on NOS, as root for SHM access to the robot's root-owned writers):
//   ./m20_drdds_zenoh_bridge \
//       --aligned   'dimos/aligned_points#sensor_msgs.PointCloud2' --aligned_topic /ALIGNED_POINTS \
//       --grid      'dimos/grid_map_3d#sensor_msgs.PointCloud2'    --grid_topic    /grid_map_3d \
//       --odometry  'dimos/odom#nav_msgs.Odometry'                 --odom_topic    /ODOM \
//       --iface eth1 --domain 0
//
// On networks where multicast scouting can't reach the router, dial it directly:
//       --connect tcp/10.21.31.103:7447,tcp/10.21.33.103:7447

#include "drdds/core/drdds_core.h"

#include "dridl/sensor_msgs/msg/PointCloud2.h"
#include "dridl/sensor_msgs/msg/PointCloud2PubSubTypes.h"
#include "dridl/sensor_msgs/msg/Imu.h"
#include "dridl/sensor_msgs/msg/ImuPubSubTypes.h"
#include "dridl/nav_msgs/msg/Odometry.h"
#include "dridl/nav_msgs/msg/OdometryPubSubTypes.h"

#include <zenoh.h>

#include "dimos_native_module.hpp"
#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Vector3.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

static std::atomic<bool> g_running{true};
static void on_signal(int) { g_running.store(false); }

static std::string g_frame_override;

// One wired output: its precomputed Zenoh key + running counters (read by the
// status line). Heap-allocated so drdds reader threads hold a stable pointer.
struct Port {
    std::string key;    // zenoh key expr, e.g. "dimos/aligned_points/sensor_msgs.PointCloud2"
    std::string label;  // short name for logs
    std::atomic<long> n{0};
    std::atomic<long> bytes{0};
    std::function<int()> matched;  // GetMatchedCount() of the underlying drdds reader
};

// ----------------------------------------------------------------- zenoh out --
static z_owned_session_t g_session;
static std::atomic<bool> g_zenoh_on{false};  // false = --no_zenoh diagnostic (skip publish)
static std::mutex g_pub_mx;
static std::map<std::string, z_owned_publisher_t> g_pubs;  // key -> cached publisher

// dimos Zenoh keys use '/' where LCM channels use '#'; key exprs can't start with '/'.
static std::string chan_to_key(const std::string& chan) {
    std::string k = chan;
    for (char& c : k) {
        if (c == '#') { c = '/'; }
    }
    size_t i = 0;
    while (i < k.size() && k[i] == '/') { ++i; }
    return k.substr(i);
}

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
    if (!g_zenoh_on.load()) {  // --no_zenoh: count only (isolate zenoh from FastDDS SHM)
        p->n.fetch_add(1, std::memory_order_relaxed);
        p->bytes.fetch_add(msg.getEncodedSize(), std::memory_order_relaxed);
        return;
    }
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

// ----------------------------------------------------- drdds -> dimos_lcm conv --
// (identical field-for-field copies to ../../dds/cpp/main.cpp; the drdds and
// dimos_lcm ROS-message layouts match, so these are straight member copies.)

static std_msgs::Header to_lcm_header(const std_msgs::msg::Header& h, const std::string& frame_override) {
    static std::atomic<int32_t> seq{0};
    std_msgs::Header out;
    out.seq = seq.fetch_add(1, std::memory_order_relaxed);
    out.stamp.sec = h.stamp().sec();
    out.stamp.nsec = static_cast<int32_t>(h.stamp().nanosec());
    out.frame_id = frame_override.empty() ? h.frame_id() : frame_override;
    return out;
}

static void on_pointcloud(const sensor_msgs::msg::PointCloud2* m, Port* p) {
    if (m == nullptr) { return; }
    sensor_msgs::PointCloud2 pc;
    pc.header = to_lcm_header(m->header(), g_frame_override);
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
    pc.data_length = static_cast<int32_t>(data.size());
    pc.data.resize(data.size());
    if (!data.empty()) {
        std::memcpy(pc.data.data(), data.data(), data.size());
    }
    publish_zenoh(p, pc);
}

static void on_imu(const sensor_msgs::msg::Imu* m, Port* p) {
    if (m == nullptr) { return; }
    sensor_msgs::Imu out;
    out.header = to_lcm_header(m->header(), g_frame_override);
    out.orientation.x = m->orientation().x();
    out.orientation.y = m->orientation().y();
    out.orientation.z = m->orientation().z();
    out.orientation.w = m->orientation().w();
    out.angular_velocity.x = m->angular_velocity().x();
    out.angular_velocity.y = m->angular_velocity().y();
    out.angular_velocity.z = m->angular_velocity().z();
    out.linear_acceleration.x = m->linear_acceleration().x();
    out.linear_acceleration.y = m->linear_acceleration().y();
    out.linear_acceleration.z = m->linear_acceleration().z();
    for (int i = 0; i < 9; ++i) {
        out.orientation_covariance[i] = m->orientation_covariance()[i];
        out.angular_velocity_covariance[i] = m->angular_velocity_covariance()[i];
        out.linear_acceleration_covariance[i] = m->linear_acceleration_covariance()[i];
    }
    publish_zenoh(p, out);
}

static void on_odometry(const nav_msgs::msg::Odometry* m, Port* p) {
    if (m == nullptr) { return; }
    nav_msgs::Odometry out;
    out.header = to_lcm_header(m->header(), g_frame_override);
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
    publish_zenoh(p, out);
}

// ------------------------------------------------------------------- wiring --
// PointCloud2 output ports: CLI flag name, --<topic_arg> override, drdds default.
struct PCPortDef {
    const char* port;
    const char* topic_arg;
    const char* default_src;
};
static const PCPortDef PC_PORTS[] = {
    {"lidar", "lidar_topic", "/LIDAR/POINTS"},          // raw lidar firehose (opt-in)
    {"aligned", "aligned_topic", "/ALIGNED_POINTS"},    // NOS localization world cloud (SHM-only)
    {"grid", "grid_topic", "/grid_map_3d"},             // 3D grid map
    {"locbody", "locbody_topic", "/LOC_BODY_POINTS"},   // body-frame localization cloud
};

// Subscribe-all prime set. The SHM-only writers (/ALIGNED_POINTS, /LOC_BODY_POINTS:
// PointCloud2 with unbounded data → FastDDS SHM transport, not data-sharing) only
// deliver samples to a subscriber that holds the FULL endpoint set; a partial
// subscription matches (m=1) but receives 0 samples. So when --subscribe_all is on
// (default), we also open count-and-drop "primer" readers for every candidate topic
// not already wired, completing the endpoint set. SHM receive is ~zero-copy and these
// are never forwarded, so the firehoses stay in-box (never hit the network).
static const char* PRIME_CLOUD_TOPICS[] = {
    "/LIO_ALIGNED_POINTS", "/CLOUD_REGISTERED_BODY", "/ACCUMULATED_POINTS_MAP", "/grid_map_3d",
    "/ALIGNED_POINTS", "/LOC_BODY_POINTS", "/DEPTH_POINTS", "/cloud_now_g", "/cloud_local_g",
    "/cloud_registered", "/full_cloud", "/SLAM_CLOUD_REGISTERED_BODY", "/cloud_nav",
    "/LIDAR/POINTS", "/LIDAR/POINTS2", "/SEG_CLOUD",
};
static const char* PRIME_ODOM_TOPICS[] = {
    "/LIO_ODOM", "/LIO_ODOM_HIGH_FREQUENCY", "/ODOM", "/fibo_fusion_pose",
};

int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    const int domain = mod.arg_int("domain", 0);
    const std::string network = mod.arg("network", "");
    g_frame_override = mod.arg("frame_id", "");
    const std::string iface = mod.arg("iface", "");
    // SHM transport: required for the robot's SHM-only writers (e.g. ALIGNED_POINTS)
    // and harmless for dual-transport topics. Run as root so the SHM segments match.
    const bool shm = mod.arg_bool("shm", true);
    // Open count-and-drop primer readers for the rest of the candidate topics so the
    // SHM-only writers (ALIGNED_POINTS / LOC_BODY_POINTS) deliver. Default on.
    const bool subscribe_all = mod.arg_bool("subscribe_all", true);

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    // Zenoh session: default config = peer mode + multicast scouting (auto-discovery),
    // so dimos consumers find us with no hardcoded endpoints. Pin the multicast NIC
    // when given (the M20 boxes are multi-homed; eth1 is the NOS .31 segment).
    // When multicast can't reach the router (an L2 segment that drops multicast),
    // pass --connect with explicit router endpoint(s) to dial it directly.
    z_owned_config_t cfg;
    const bool no_zenoh = mod.arg_bool("no_zenoh", false);  // diagnostic: skip zenoh entirely
    if (!no_zenoh) {
        z_config_default(&cfg);
        if (!iface.empty()) {
            const std::string v = "\"" + iface + "\"";
            zc_config_insert_json5(z_config_loan_mut(&cfg), "scouting/multicast/interface", v.c_str());
        }
        // Explicit router endpoint(s), comma-separated (e.g.
        // "tcp/10.21.31.103:7447,tcp/10.21.33.103:7447"). Multicast scouting still
        // runs alongside, so this is additive: connect directly AND auto-discover.
        const std::string connect = mod.arg("connect", "");
        if (!connect.empty()) {
            std::string eps = "[";
            bool first = true;
            for (size_t start = 0; start <= connect.size();) {
                const size_t comma = connect.find(',', start);
                const size_t len = comma == std::string::npos ? std::string::npos : comma - start;
                const std::string ep = connect.substr(start, len);
                if (!ep.empty()) {
                    eps += first ? "\"" : ",\"";
                    eps += ep + "\"";
                    first = false;
                }
                if (comma == std::string::npos) { break; }
                start = comma + 1;
            }
            eps += "]";
            zc_config_insert_json5(z_config_loan_mut(&cfg), "connect/endpoints", eps.c_str());
            fprintf(stderr, "[bridge] connect endpoints: %s\n", eps.c_str());
        }
        if (z_open(&g_session, z_move(cfg), nullptr) != Z_OK) {
            fprintf(stderr, "[bridge] zenoh session open failed\n");
            return 1;
        }
        g_zenoh_on.store(true);
    } else {
        fprintf(stderr, "[bridge] --no_zenoh: zenoh disabled (count-only, SHM isolation test)\n");
    }

    DrDDSManager::Init(domain, network);

    std::vector<std::unique_ptr<Port>> ports;
    std::vector<std::unique_ptr<DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>>> pc_chans;
    std::unique_ptr<DrDDSChannel<sensor_msgs::msg::ImuPubSubType>> imu_chan;
    std::unique_ptr<DrDDSChannel<nav_msgs::msg::OdometryPubSubType>> odom_chan;
    std::set<std::string> wired_src;  // drdds topics we already subscribe to (forwarded)

    for (const auto& def : PC_PORTS) {
        if (!mod.has(def.port)) { continue; }
        const std::string src = mod.arg(def.topic_arg, def.default_src);
        auto p = std::make_unique<Port>();
        p->key = chan_to_key(mod.topic(def.port));
        p->label = def.port;
        Port* pp = p.get();
        ports.push_back(std::move(p));
        pc_chans.push_back(std::make_unique<DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>>(
            [pp](const sensor_msgs::msg::PointCloud2* m) { on_pointcloud(m, pp); },
            src, domain, shm, "rt"));
        auto* ch = pc_chans.back().get();
        pp->matched = [ch] { return ch->GetMatchedCount(); };
        wired_src.insert(src);
        fprintf(stderr, "[bridge] %s: rt%s -> %s\n", def.port, src.c_str(), pp->key.c_str());
    }
    if (mod.has("imu")) {
        const std::string src = mod.arg("imu_topic", "/IMU");
        auto p = std::make_unique<Port>();
        p->key = chan_to_key(mod.topic("imu"));
        p->label = "imu";
        Port* pp = p.get();
        ports.push_back(std::move(p));
        imu_chan = std::make_unique<DrDDSChannel<sensor_msgs::msg::ImuPubSubType>>(
            [pp](const sensor_msgs::msg::Imu* m) { on_imu(m, pp); }, src, domain, shm, "rt");
        wired_src.insert(src);
        fprintf(stderr, "[bridge] imu: rt%s -> %s\n", src.c_str(), pp->key.c_str());
    }
    if (mod.has("odometry")) {
        const std::string src = mod.arg("odom_topic", "/ODOM");
        auto p = std::make_unique<Port>();
        p->key = chan_to_key(mod.topic("odometry"));
        p->label = "odometry";
        Port* pp = p.get();
        ports.push_back(std::move(p));
        odom_chan = std::make_unique<DrDDSChannel<nav_msgs::msg::OdometryPubSubType>>(
            [pp](const nav_msgs::msg::Odometry* m) { on_odometry(m, pp); }, src, domain, shm, "rt");
        wired_src.insert(src);
        fprintf(stderr, "[bridge] odometry: rt%s -> %s\n", src.c_str(), pp->key.c_str());
    }

    if (ports.empty()) {
        fprintf(stderr, "[bridge] no output ports wired; nothing to bridge\n");
        return 1;
    }

    // Complete the SHM endpoint set so SHM-only topics deliver: count-and-drop readers
    // for every candidate topic not already wired (never forwarded to zenoh).
    std::vector<std::unique_ptr<DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>>> prime_pc;
    std::vector<std::unique_ptr<DrDDSChannel<nav_msgs::msg::OdometryPubSubType>>> prime_od;
    if (subscribe_all) {
        for (const char* topic : PRIME_CLOUD_TOPICS) {
            if (wired_src.count(topic)) { continue; }
            prime_pc.push_back(std::make_unique<DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>>(
                [](const sensor_msgs::msg::PointCloud2*) {}, topic, domain, shm, "rt"));
        }
        for (const char* topic : PRIME_ODOM_TOPICS) {
            if (wired_src.count(topic)) { continue; }
            prime_od.push_back(std::make_unique<DrDDSChannel<nav_msgs::msg::OdometryPubSubType>>(
                [](const nav_msgs::msg::Odometry*) {}, topic, domain, shm, "rt"));
        }
        fprintf(stderr, "[bridge] subscribe_all: +%zu primer readers (count+drop) so SHM-only topics deliver\n",
                prime_pc.size() + prime_od.size());
    }

    fprintf(stderr, "[bridge] shm=%d iface=%s bridging domain %d ...\n", shm,
            iface.empty() ? "(auto)" : iface.c_str(), domain);
    long t = 0;
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ++t;
        std::string line = "t=" + std::to_string(t) + "s";
        for (const auto& p : ports) {
            char b[96];
            const int m = p->matched ? p->matched() : -1;
            snprintf(b, sizeof(b), "  %s[m=%d n=%ld %.1fMB]", p->label.c_str(), m, p->n.load(),
                     p->bytes.load() / 1e6);
            line += b;
        }
        fprintf(stderr, "%s\n", line.c_str());
    }

    fprintf(stderr, "[bridge] shutting down\n");
    prime_pc.clear();
    prime_od.clear();
    pc_chans.clear();
    imu_chan.reset();
    odom_chan.reset();
    DrDDSManager::Delete();
    if (g_zenoh_on.load()) {
        {
            std::lock_guard<std::mutex> lk(g_pub_mx);
            for (auto& kv : g_pubs) { z_drop(z_move(kv.second)); }
            g_pubs.clear();
        }
        z_drop(z_move(g_session));
    }
    return 0;
}
