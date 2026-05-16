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

#include "commons.h"
#include "simple_pgo.h"
#include "dimos_native_module.hpp"
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

        CloudWithPose cp;
        cp.pose.r = g_latest_r;
        cp.pose.t = g_latest_t;
        cp.pose.setTime(static_cast<int32_t>(ts),
                        static_cast<uint32_t>((ts - static_cast<int32_t>(ts)) * 1e9));

        // Parse PointCloud2 to PCL
        cp.cloud = CloudType::Ptr(new CloudType);
        smartnav::to_pcl(*msg, *cp.cloud);

        std::lock_guard<std::mutex> buf_lock(g_buffer_mutex);
        g_cloud_buffer.push(cp);
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

// Build a Path-encoded GraphNodes3D message — one pose per keyframe,
// orientation.w encoded as node_type (1 = odom/robot — green in rerun).
static nav_msgs::Path build_graph_nodes(const std::vector<KeyPoseWithCloud>& key_poses,
                                         double ts,
                                         const std::string& frame_id) {
    nav_msgs::Path msg;
    msg.header = dimos::make_header(frame_id, ts);
    msg.poses_length = static_cast<int32_t>(key_poses.size());
    msg.poses.reserve(key_poses.size());
    for (const auto& keyframe : key_poses) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = dimos::make_header(frame_id, ts);
        pose_stamped.pose.position.x = keyframe.t_global.x();
        pose_stamped.pose.position.y = keyframe.t_global.y();
        pose_stamped.pose.position.z = keyframe.t_global.z();
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;
        msg.poses.push_back(pose_stamped);
    }
    return msg;
}

static void append_segment(nav_msgs::Path& msg,
                            const std::string& frame_id,
                            double start_ts,
                            const V3D& start,
                            const V3D& end,
                            double traversability,
                            double end_ts) {
    geometry_msgs::PoseStamped start_pose;
    start_pose.header = dimos::make_header(frame_id, start_ts);
    start_pose.pose.position.x = start.x();
    start_pose.pose.position.y = start.y();
    start_pose.pose.position.z = start.z();
    start_pose.pose.orientation.x = 0.0;
    start_pose.pose.orientation.y = 0.0;
    start_pose.pose.orientation.z = 0.0;
    // traversability is encoded on the first pose of each pair
    start_pose.pose.orientation.w = traversability;

    geometry_msgs::PoseStamped end_pose;
    end_pose.header = dimos::make_header(frame_id, end_ts);
    end_pose.pose.position.x = end.x();
    end_pose.pose.position.y = end.y();
    end_pose.pose.position.z = end.z();
    end_pose.pose.orientation.x = 0.0;
    end_pose.pose.orientation.y = 0.0;
    end_pose.pose.orientation.z = 0.0;
    end_pose.pose.orientation.w = traversability;

    msg.poses.push_back(start_pose);
    msg.poses.push_back(end_pose);
}

// Build a Path-encoded loop-closure-deltas message — one PoseStamped per
// keyframe, where position = (post - delta @ pre) translation delta and
// orientation = delta rotation quaternion. The Nth pose corresponds to
// the Nth keyframe (m_key_poses[N]).
static nav_msgs::Path build_loop_closure_deltas(
    const std::vector<std::pair<M3D, V3D>>& pre_poses,
    const std::vector<KeyPoseWithCloud>& post_poses,
    double ts,
    const std::string& frame_id) {
    nav_msgs::Path msg;
    msg.header = dimos::make_header(frame_id, ts);
    size_t count = std::min(pre_poses.size(), post_poses.size());
    msg.poses.reserve(count);
    for (size_t i = 0; i < count; i++) {
        const M3D& pre_r = pre_poses[i].first;
        const V3D& pre_t = pre_poses[i].second;
        const M3D& post_r = post_poses[i].r_global;
        const V3D& post_t = post_poses[i].t_global;

        // SE(3) delta such that post = delta * pre.
        M3D r_delta = post_r * pre_r.transpose();
        V3D t_delta = post_t - r_delta * pre_t;

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = dimos::make_header(frame_id, ts);
        pose_stamped.pose.position.x = t_delta.x();
        pose_stamped.pose.position.y = t_delta.y();
        pose_stamped.pose.position.z = t_delta.z();
        Eigen::Quaterniond q(r_delta);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();
        msg.poses.push_back(pose_stamped);
    }
    msg.poses_length = static_cast<int32_t>(msg.poses.size());
    return msg;
}

// Build a Path-encoded LineSegments3D message — pose pairs form segments.
// Odometry edges get traversability=1.0 (green); loop closures get 0.4
// (yellow) so they stand out in the rerun rendering. The header stamp
// on each endpoint is the *creation* time of that keyframe (not the
// message publish time), so downstream consumers can correlate edge
// endpoints back to the input scan that produced each keyframe.
static nav_msgs::Path build_graph_edges(const std::vector<KeyPoseWithCloud>& key_poses,
                                         const std::vector<std::pair<size_t, size_t>>& loop_pairs,
                                         double ts,
                                         const std::string& frame_id) {
    nav_msgs::Path msg;
    msg.header = dimos::make_header(frame_id, ts);

    // Odometry edges between consecutive keyframes.
    for (size_t i = 1; i < key_poses.size(); i++) {
        append_segment(msg, frame_id, key_poses[i - 1].time,
                       key_poses[i - 1].t_global,
                       key_poses[i].t_global,
                       1.0,
                       key_poses[i].time);
    }
    // Loop closure edges.
    for (const auto& pair : loop_pairs) {
        if (pair.first >= key_poses.size() || pair.second >= key_poses.size())
            continue;
        append_segment(msg, frame_id, key_poses[pair.first].time,
                       key_poses[pair.first].t_global,
                       key_poses[pair.second].t_global,
                       0.4,
                       key_poses[pair.second].time);
    }
    msg.poses_length = static_cast<int32_t>(msg.poses.size());
    return msg;
}

int main(int argc, char** argv)
{
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    dimos::NativeModule mod(argc, argv);

    // Port topics
    std::string scan_topic = mod.topic("registered_scan");
    std::string odom_topic = mod.topic("odometry");
    std::string corrected_odom_topic = mod.topic("corrected_odometry");
    std::string global_map_topic = mod.topic("global_map");
    std::string tf_channel = mod.arg("tf_channel", "/tf#tf2_msgs.TFMessage");
    std::string graph_nodes_topic = mod.topic("pgo_graph_nodes");
    std::string graph_edges_topic = mod.topic("pgo_graph_edges");
    std::string loop_closure_topic = mod.topic("pgo_loop_closure");

    // Config parameters
    Config config;
    config.key_pose_delta_deg = mod.arg_float("key_pose_delta_deg", 10.0f);
    config.key_pose_delta_trans = mod.arg_float("key_pose_delta_trans", 0.5f);
    config.loop_search_radius = mod.arg_float("loop_search_radius", 1.0f);
    config.loop_time_tresh = mod.arg_float("loop_time_thresh", 60.0f);
    config.loop_score_tresh = mod.arg_float("loop_score_thresh", 0.15f);
    config.loop_submap_half_range = mod.arg_int("loop_submap_half_range", 5);
    config.submap_resolution = mod.arg_float("submap_resolution", 0.1f);
    config.min_loop_detect_duration = mod.arg_float("min_loop_detect_duration", 5.0f);
    config.use_scan_context = mod.arg_bool("use_scan_context", true);
    config.sc_n_rings = mod.arg_int("sc_n_rings", 20);
    config.sc_n_sectors = mod.arg_int("sc_n_sectors", 60);
    config.sc_max_range_m = mod.arg_float("sc_max_range_m", 80.0f);
    config.sc_top_k = mod.arg_int("sc_top_k", 10);
    config.sc_match_threshold = mod.arg_float("sc_match_threshold", 0.4f);
    config.sc_lidar_height_m = mod.arg_float("sc_lidar_height_m", 2.0f);

    // Node-level config
    std::string parent_frame = mod.arg("parent_frame", "world");
    std::string world_frame = mod.arg("world_frame", "map");
    std::string local_frame = mod.arg("local_frame", "odom");
    std::string body_frame = mod.arg("body_frame", "base_link");
    float global_map_voxel_size = mod.arg_float("global_map_voxel_size", 0.1f);
    float global_map_publish_rate = mod.arg_float("global_map_publish_rate", 1.0f);
    double global_map_interval = global_map_publish_rate > 0
        ? 1.0 / global_map_publish_rate : 2.0;

    // Unregister mode: transform world-frame scans to body-frame
    bool unregister_input = mod.arg_bool("unregister_input", true);

    SimplePGO pgo(config);

    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "PGO: LCM init failed\n");
        return 1;
    }

    Handlers handlers;
    lcm.subscribe(odom_topic, &Handlers::on_odometry, &handlers);
    lcm.subscribe(scan_topic, &Handlers::on_registered_scan, &handlers);

    fprintf(stderr, "PGO native module started\n");
    fprintf(stderr, "  registered_scan: %s\n", scan_topic.c_str());
    fprintf(stderr, "  odometry: %s\n", odom_topic.c_str());
    fprintf(stderr, "  corrected_odometry: %s\n", corrected_odom_topic.c_str());
    fprintf(stderr, "  global_map: %s\n", global_map_topic.c_str());
    fprintf(stderr, "  tf channel: %s\n", tf_channel.c_str());
    fprintf(stderr, "  pgo_graph_nodes: %s\n", graph_nodes_topic.c_str());
    fprintf(stderr, "  pgo_graph_edges: %s\n", graph_edges_topic.c_str());
    fprintf(stderr, "  pgo_loop_closure: %s\n", loop_closure_topic.c_str());

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
        CloudWithPose cp;
        bool has_data = false;
        {
            std::lock_guard<std::mutex> lock(g_buffer_mutex);
            if (!g_cloud_buffer.empty()) {
                cp = g_cloud_buffer.front();
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
        if (unregister_input && cp.cloud && cp.cloud->size() > 0) {
            CloudType::Ptr body_cloud(new CloudType);
            // body = R_odom^T * (world_pts - t_odom)
            M3D r_inv = cp.pose.r.transpose();
            for (const auto& pt : *cp.cloud) {
                V3D world_pt(pt.x, pt.y, pt.z);
                V3D body_pt = r_inv * (world_pt - cp.pose.t);
                PointType bp;
                bp.x = static_cast<float>(body_pt.x());
                bp.y = static_cast<float>(body_pt.y());
                bp.z = static_cast<float>(body_pt.z());
                bp.intensity = pt.intensity;
                body_cloud->push_back(bp);
            }
            cp.cloud = body_cloud;
        }

        double cur_time = cp.pose.second;

        if (!pgo.addKeyPose(cp)) {
            // Not a keyframe — still broadcast TF and corrected odom
            M3D corr_r = pgo.offsetR() * cp.pose.r;
            V3D corr_t = pgo.offsetR() * cp.pose.t + pgo.offsetT();

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
            nav_msgs::Path loop_closure_msg = build_loop_closure_deltas(
                pre_poses, pgo.keyPoses(), cur_time, world_frame);
            lcm.publish(loop_closure_topic, &loop_closure_msg);
            fprintf(stderr,
                    "PGO: loop closure event published — %zu keyframe deltas\n",
                    pre_poses.size());
        }

        fprintf(stderr, "PGO: keyframe %zu at (%.1f, %.1f, %.1f)\n",
                pgo.keyPoses().size(),
                cp.pose.t.x(), cp.pose.t.y(), cp.pose.t.z());

        // Publish corrected odometry
        M3D corr_r = pgo.offsetR() * cp.pose.r;
        V3D corr_t = pgo.offsetR() * cp.pose.t + pgo.offsetT();
        nav_msgs::Odometry corrected = build_odometry(
            corr_r, corr_t, cur_time, world_frame, body_frame);
        lcm.publish(corrected_odom_topic, &corrected);

        auto tf_msg = build_tf_message(
            pgo.offsetR(), pgo.offsetT(), cur_time, parent_frame, world_frame, local_frame);
        lcm.publish(tf_channel, &tf_msg);

        // Publish pose-graph nodes + edges (on every keyframe — iSAM2
        // may have re-optimized prior poses on loop closure).
        {
            nav_msgs::Path nodes_msg = build_graph_nodes(
                pgo.keyPoses(), cur_time, world_frame);
            lcm.publish(graph_nodes_topic, &nodes_msg);

            nav_msgs::Path edges_msg = build_graph_edges(
                pgo.keyPoses(), pgo.historyPairs(), cur_time, world_frame);
            lcm.publish(graph_edges_topic, &edges_msg);
        }

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

    fprintf(stderr, "PGO native module shutting down\n");
    return 0;
}
