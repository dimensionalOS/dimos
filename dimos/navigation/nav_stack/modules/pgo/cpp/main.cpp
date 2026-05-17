// PGO NativeModule — faithful port of pgo_node.cpp from ROS2 to LCM.
// Subscribes to registered_scan + odometry, runs SimplePGO (iSAM2 + PCL ICP),
// publishes corrected_odometry, global_map, and TF correction offset.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <queue>
#include <signal.h>
#include <thread>

#include <lcm/lcm-cpp.hpp>
#include <Eigen/Geometry>
#include <pcl/console/print.h>

#include "commons.h"
#include "simple_pgo.h"
#include "dimos_native_module.hpp"
#include "msgs/Graph3D.hpp"
#include "msgs/GraphDelta3D.hpp"
#include "point_cloud_utils.hpp"

#include "nav_msgs/Odometry.hpp"
#include "nav_msgs/Path.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "geometry_msgs/Pose.hpp"
#include "geometry_msgs/PoseStamped.hpp"
#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Point.hpp"
#include "geometry_msgs/Transform.hpp"
#include "geometry_msgs/TransformStamped.hpp"
#include "tf2_msgs/TFMessage.hpp"

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running.store(false); }

// Shared state between LCM callbacks and main loop
static std::mutex g_buffer_mutex;
static std::queue<CloudWithPose> g_cloud_buffer;
static double g_last_message_time = 0.0;

// Latest odometry for non-keyframe TF broadcasting
static std::mutex g_odom_mutex;
static M3D g_latest_r = M3D::Identity();
static V3D g_latest_t = V3D::Zero();
static double g_latest_time = 0.0;
static bool g_has_odom = false;

class Handlers {
public:
    void on_odometry(const lcm::ReceiveBuffer*, const std::string&,
                     const nav_msgs::Odometry* msg) {
        M3D r = Eigen::Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        ).toRotationMatrix();

        V3D t(msg->pose.pose.position.x,
               msg->pose.pose.position.y,
               msg->pose.pose.position.z);

        double ts = msg->header.stamp.sec + msg->header.stamp.nsec / 1e9;

        std::lock_guard<std::mutex> lock(g_odom_mutex);
        g_latest_r = r;
        g_latest_t = t;
        g_latest_time = ts;
        g_has_odom = true;
    }

    void on_registered_scan(const lcm::ReceiveBuffer*, const std::string&,
                            const sensor_msgs::PointCloud2* msg) {
        std::lock_guard<std::mutex> odom_lock(g_odom_mutex);
        if (!g_has_odom)
            return;

        double ts = g_latest_time;

        // Reject out-of-order messages
        if (ts < g_last_message_time)
            return;
        g_last_message_time = ts;

        CloudWithPose cloud_with_pose;
        cloud_with_pose.pose.r = g_latest_r;
        cloud_with_pose.pose.t = g_latest_t;
        cloud_with_pose.pose.setTime(static_cast<int32_t>(ts),
                        static_cast<uint32_t>((ts - static_cast<int32_t>(ts)) * 1e9));

        // Parse PointCloud2 to PCL
        cloud_with_pose.cloud = CloudType::Ptr(new CloudType);
        smartnav::to_pcl(*msg, *cloud_with_pose.cloud);

        std::lock_guard<std::mutex> buf_lock(g_buffer_mutex);
        g_cloud_buffer.push(cloud_with_pose);
    }
};

static geometry_msgs::TransformStamped build_tf(const M3D& r, const V3D& t, double ts,
                                                  const std::string& frame_id,
                                                  const std::string& child_frame_id) {
    geometry_msgs::TransformStamped ts_msg;
    ts_msg.header = dimos::make_header(frame_id, ts);
    ts_msg.child_frame_id = child_frame_id;
    Eigen::Quaterniond q(r);
    ts_msg.transform.translation.x = t.x();
    ts_msg.transform.translation.y = t.y();
    ts_msg.transform.translation.z = t.z();
    ts_msg.transform.rotation.x = q.x();
    ts_msg.transform.rotation.y = q.y();
    ts_msg.transform.rotation.z = q.z();
    ts_msg.transform.rotation.w = q.w();
    return ts_msg;
}

static tf2_msgs::TFMessage build_tf_message(const M3D& correction_r,
                                              const V3D& correction_t,
                                              double ts,
                                              const std::string& parent_frame,
                                              const std::string& world_frame,
                                              const std::string& local_frame) {
    tf2_msgs::TFMessage msg;
    // Identity anchor parent_frame -> world_frame.
    msg.transforms.push_back(
        build_tf(M3D::Identity(), V3D::Zero(), ts, parent_frame, world_frame));
    // SLAM correction world_frame -> local_frame.
    msg.transforms.push_back(
        build_tf(correction_r, correction_t, ts, world_frame, local_frame));
    msg.transforms_length = static_cast<int32_t>(msg.transforms.size());
    return msg;
}

static nav_msgs::Odometry build_odometry(const M3D& r, const V3D& t, double ts,
                                          const std::string& frame_id,
                                          const std::string& child_frame_id) {
    nav_msgs::Odometry odom;
    odom.header = dimos::make_header(frame_id, ts);
    odom.child_frame_id = child_frame_id;

    Eigen::Quaterniond q(r);
    odom.pose.pose.position.x = t.x();
    odom.pose.pose.position.y = t.y();
    odom.pose.pose.position.z = t.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    return odom;
}

// Pose-graph snapshot encoded as a Graph3D:
//   - one node per keyframe
static constexpr uint64_t NODE_KEYFRAME = 0;
static constexpr uint64_t EDGE_ODOMETRY = 0;
static constexpr uint64_t EDGE_LOOP_CLOSURE = 1;

static dimos::Graph3D build_pose_graph(
    const std::vector<KeyPoseWithCloud>& key_poses,
    const std::vector<std::pair<size_t, size_t>>& loop_pairs,
    double ts,
    const std::string& frame_id) {
    dimos::Graph3D msg(frame_id, ts);
    msg.reserve_nodes(key_poses.size());
    msg.reserve_edges(key_poses.size() + loop_pairs.size());
    for (size_t i = 0; i < key_poses.size(); i++) {
        const auto& kp = key_poses[i];
        Eigen::Quaterniond q(kp.r_global);
        msg.add_node(
            static_cast<uint64_t>(i),
            NODE_KEYFRAME,
            kp.time,
            kp.t_global.x(), kp.t_global.y(), kp.t_global.z(),
            q.x(), q.y(), q.z(), q.w());
    }
    for (size_t i = 1; i < key_poses.size(); i++) {
        msg.add_edge(
            static_cast<uint64_t>(i - 1),
            static_cast<uint64_t>(i),
            key_poses[i].time,
            EDGE_ODOMETRY);
    }
    for (const auto& pair : loop_pairs) {
        if (pair.first >= key_poses.size() || pair.second >= key_poses.size()) continue;
        msg.add_edge(
            static_cast<uint64_t>(pair.first),
            static_cast<uint64_t>(pair.second),
            ts,
            EDGE_LOOP_CLOSURE);
    }
    return msg;
}

// Build a GraphDelta3D from paired pre/post keyframe lists. Each
// (node, transform) pair has:
//   - node    = the keyframe BEFORE iSAM2's smoothAndUpdate, with id =
//               keyframe index and metadata_id = NODE_KEYFRAME.
//   - transform = SE(3) delta such that post = transform * pre.
// Convention matches Python's GraphDelta3D.lcm_decode.
static constexpr uint64_t NODE_KEYFRAME_DELTA = 0;

static dimos::GraphDelta3D build_loop_closure_event(
    const std::vector<std::pair<M3D, V3D>>& pre_poses,
    const std::vector<KeyPoseWithCloud>& post_poses,
    double ts,
    const std::string& frame_id) {
    dimos::GraphDelta3D msg(frame_id, ts);
    size_t count = std::min(pre_poses.size(), post_poses.size());
    msg.reserve(count);
    for (size_t i = 0; i < count; i++) {
        const M3D& pre_r = pre_poses[i].first;
        const V3D& pre_t = pre_poses[i].second;
        const M3D& post_r = post_poses[i].r_global;
        const V3D& post_t = post_poses[i].t_global;

        // SE(3) delta such that post = delta * pre.
        M3D r_delta = post_r * pre_r.transpose();
        V3D t_delta = post_t - r_delta * pre_t;
        Eigen::Quaterniond q_pre(pre_r);
        Eigen::Quaterniond q_delta(r_delta);

        msg.add(
            /* id */ static_cast<uint64_t>(i),
            /* metadata_id */ NODE_KEYFRAME_DELTA,
            /* pose_ts */ post_poses[i].time,
            /* pos_x,y,z */ pre_t.x(), pre_t.y(), pre_t.z(),
            /* quat_x,y,z,w */ q_pre.x(), q_pre.y(), q_pre.z(), q_pre.w(),
            /* translation_x,y,z */ t_delta.x(), t_delta.y(), t_delta.z(),
            /* rotation_x,y,z,w */ q_delta.x(), q_delta.y(), q_delta.z(), q_delta.w());
    }
    return msg;
}

int main(int argc, char** argv)
{
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    dimos::NativeModule native_module(argc, argv);

    // Port topics
    std::string scan_topic = native_module.topic("registered_scan");
    std::string odom_topic = native_module.topic("odometry");
    std::string corrected_odom_topic = native_module.topic("corrected_odometry");
    std::string global_map_topic = native_module.topic("global_map");
    std::string tf_channel = native_module.arg("tf_channel", "/tf#tf2_msgs.TFMessage");
    std::string pose_graph_topic = native_module.topic("pose_graph");
    std::string loop_closure_event_topic = native_module.topic("loop_closure_event");

    // Config parameters
    Config config;
    config.key_pose_delta_deg = native_module.arg_float("key_pose_delta_deg", 10.0f);
    config.key_pose_delta_trans = native_module.arg_float("key_pose_delta_trans", 0.5f);
    config.loop_search_radius = native_module.arg_float("loop_search_radius", 1.0f);
    config.loop_time_thresh = native_module.arg_float("loop_time_thresh", 60.0f);
    config.loop_score_thresh = native_module.arg_float("loop_score_thresh", 0.15f);
    config.loop_submap_half_range = native_module.arg_int("loop_submap_half_range", 5);
    config.submap_resolution = native_module.arg_float("submap_resolution", 0.1f);
    config.min_loop_detect_duration = native_module.arg_float("min_loop_detect_duration", 5.0f);
    config.use_scan_context = native_module.arg_bool("use_scan_context", true);
    config.scan_context_num_rings = native_module.arg_int("scan_context_num_rings", 20);
    config.scan_context_num_sectors = native_module.arg_int("scan_context_num_sectors", 60);
    config.scan_context_max_range_m = native_module.arg_float("scan_context_max_range_m", 80.0f);
    config.scan_context_top_k = native_module.arg_int("scan_context_top_k", 10);
    config.scan_context_match_threshold = native_module.arg_float("scan_context_match_threshold", 0.4f);
    config.scan_context_lidar_height_m = native_module.arg_float("scan_context_lidar_height_m", 2.0f);

    // Node-level config
    std::string parent_frame = native_module.arg("parent_frame", "world");
    std::string world_frame = native_module.arg("world_frame", "map");
    std::string local_frame = native_module.arg("local_frame", "odom");
    std::string body_frame = native_module.arg("body_frame", "base_link");
    float global_map_voxel_size = native_module.arg_float("global_map_voxel_size", 0.1f);
    float global_map_publish_rate = native_module.arg_float("global_map_publish_rate", 1.0f);
    double global_map_interval = global_map_publish_rate > 0
        ? 1.0 / global_map_publish_rate : 2.0;

    // Unregister mode: transform world-frame scans to body-frame
    bool unregister_input = native_module.arg_bool("unregister_input", true);

    bool debug = native_module.arg_bool("debug", false);

    pcl::console::setVerbosityLevel(
        debug ? pcl::console::L_INFO : pcl::console::L_ERROR);

    SimplePGO pgo(config);

    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "PGO: LCM init failed\n");
        return 1;
    }

    Handlers handlers;
    lcm.subscribe(odom_topic, &Handlers::on_odometry, &handlers);
    lcm.subscribe(scan_topic, &Handlers::on_registered_scan, &handlers);

    // NativeModule.start() in Python reads stderr for this marker and only
    // returns once it sees it. Without this, upstream publishers can race
    // ahead and emit messages before our LCM subscriptions are live.
    fprintf(stderr, "[DIMOS_NATIVE_READY]\n");
    fflush(stderr);

    if (debug) {
        fprintf(stderr, "PGO native module started\n");
        fprintf(stderr, "  registered_scan: %s\n", scan_topic.c_str());
        fprintf(stderr, "  odometry: %s\n", odom_topic.c_str());
        fprintf(stderr, "  corrected_odometry: %s\n", corrected_odom_topic.c_str());
        fprintf(stderr, "  global_map: %s\n", global_map_topic.c_str());
        fprintf(stderr, "  tf_channel: %s\n", tf_channel.c_str());
        fprintf(stderr, "  pose_graph: %s\n", pose_graph_topic.c_str());
        fprintf(stderr, "  loop_closure_event: %s\n", loop_closure_event_topic.c_str());
    }
    // Seed identity TF so consumers can query the chain before the first
    // odom message arrives.
    {
        double seed_ts =
            std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
        auto seed = build_tf_message(M3D::Identity(), V3D::Zero(), seed_ts,
                                     parent_frame, world_frame, local_frame);
        lcm.publish(tf_channel, &seed);
    }

    double last_global_map_time = 0.0;
    int timer_period_ms = 50;  // 20 Hz, matching original

    while (g_running.load()) {
        // Drain all pending LCM messages
        while (lcm.handleTimeout(0) > 0) {}

        // Check buffer
        CloudWithPose cloud_with_pose;
        bool has_data = false;
        {
            std::lock_guard<std::mutex> lock(g_buffer_mutex);
            if (!g_cloud_buffer.empty()) {
                cloud_with_pose = g_cloud_buffer.front();
                // Drain entire queue (matching original: process oldest, discard rest)
                while (!g_cloud_buffer.empty()) {
                    g_cloud_buffer.pop();
                }
                has_data = true;
            }
        }

        if (!has_data) {
            std::this_thread::sleep_for(std::chrono::milliseconds(timer_period_ms));
            continue;
        }

        // Optionally transform world-frame scan to body-frame
        if (unregister_input && cloud_with_pose.cloud && cloud_with_pose.cloud->size() > 0) {
            CloudType::Ptr body_cloud(new CloudType);
            // body = R_odom^T * (world_pts - t_odom)
            M3D r_inv = cloud_with_pose.pose.r.transpose();
            for (const auto& pt : *cloud_with_pose.cloud) {
                V3D world_pt(pt.x, pt.y, pt.z);
                V3D body_pt = r_inv * (world_pt - cloud_with_pose.pose.t);
                PointType bp;
                bp.x = static_cast<float>(body_pt.x());
                bp.y = static_cast<float>(body_pt.y());
                bp.z = static_cast<float>(body_pt.z());
                bp.intensity = pt.intensity;
                body_cloud->push_back(bp);
            }
            cloud_with_pose.cloud = body_cloud;
        }

        double cur_time = cloud_with_pose.pose.second;

        if (!pgo.addKeyPose(cloud_with_pose)) {
            // Not a keyframe — still broadcast TF and corrected odom
            M3D corr_r = pgo.offsetR() * cloud_with_pose.pose.r;
            V3D corr_t = pgo.offsetR() * cloud_with_pose.pose.t + pgo.offsetT();

            nav_msgs::Odometry corrected = build_odometry(
                corr_r, corr_t, cur_time, world_frame, body_frame);
            lcm.publish(corrected_odom_topic, &corrected);

            auto tf_msg = build_tf_message(
                pgo.offsetR(), pgo.offsetT(), cur_time, parent_frame, world_frame, local_frame);
            lcm.publish(tf_channel, &tf_msg);

            std::this_thread::sleep_for(std::chrono::milliseconds(timer_period_ms));
            continue;
        }

        // Keyframe added. Snapshot keyframe global poses BEFORE search +
        // smooth so we can publish the delta applied by iSAM2 if a loop
        // closure actually fires.
        pgo.searchForLoopPairs();
        bool had_loop = pgo.hasLoop();

        std::vector<std::pair<M3D, V3D>> pre_poses;
        if (had_loop) {
            pre_poses.reserve(pgo.keyPoses().size());
            for (const auto& kp : pgo.keyPoses()) {
                pre_poses.emplace_back(kp.r_global, kp.t_global);
            }
        }

        pgo.smoothAndUpdate();

        if (had_loop) {
            dimos::GraphDelta3D loop_closure_event_msg = build_loop_closure_event(
                pre_poses, pgo.keyPoses(), cur_time, world_frame);
            loop_closure_event_msg.publish(lcm, loop_closure_event_topic);
            if (debug) {
                fprintf(stderr,
                        "PGO: loop_closure_event published — %zu keyframe deltas\n",
                        pre_poses.size());
            }
        }

        if (debug) {
            fprintf(stderr, "PGO: keyframe %zu at (%.1f, %.1f, %.1f)\n",
                    pgo.keyPoses().size(),
                    cloud_with_pose.pose.t.x(), cloud_with_pose.pose.t.y(), cloud_with_pose.pose.t.z());
        }

        // Publish corrected odometry
        M3D corr_r = pgo.offsetR() * cloud_with_pose.pose.r;
        V3D corr_t = pgo.offsetR() * cloud_with_pose.pose.t + pgo.offsetT();
        nav_msgs::Odometry corrected = build_odometry(
            corr_r, corr_t, cur_time, world_frame, body_frame);
        lcm.publish(corrected_odom_topic, &corrected);

        auto tf_msg = build_tf_message(
            pgo.offsetR(), pgo.offsetT(), cur_time, parent_frame, world_frame, local_frame);
        lcm.publish(tf_channel, &tf_msg);

        // Publish pose graph (on every keyframe — iSAM2 may have
        // re-optimized prior poses on loop closure).
        dimos::Graph3D pose_graph_msg = build_pose_graph(
            pgo.keyPoses(), pgo.historyPairs(), cur_time, world_frame);
        pose_graph_msg.publish(lcm, pose_graph_topic);

        // Publish global map (throttled)
        double now = cur_time;
        if (now - last_global_map_time >= global_map_interval) {
            last_global_map_time = now;

            if (!pgo.keyPoses().empty()) {
                CloudType::Ptr global_cloud(new CloudType);
                for (size_t i = 0; i < pgo.keyPoses().size(); i++) {
                    CloudType::Ptr world_cloud(new CloudType);
                    pcl::transformPointCloud(
                        *pgo.keyPoses()[i].body_cloud,
                        *world_cloud,
                        pgo.keyPoses()[i].t_global,
                        Eigen::Quaterniond(pgo.keyPoses()[i].r_global));
                    *global_cloud += *world_cloud;
                }

                // Voxel downsample
                CloudType::Ptr filtered(new CloudType);
                pcl::VoxelGrid<PointType> voxel;
                voxel.setInputCloud(global_cloud);
                voxel.setLeafSize(global_map_voxel_size, global_map_voxel_size, global_map_voxel_size);
                voxel.filter(*filtered);

                sensor_msgs::PointCloud2 map_msg = smartnav::from_pcl(*filtered, world_frame, now);
                lcm.publish(global_map_topic, &map_msg);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(timer_period_ms));
    }

    if (debug) fprintf(stderr, "PGO native module shutting down\n");
    return 0;
}
