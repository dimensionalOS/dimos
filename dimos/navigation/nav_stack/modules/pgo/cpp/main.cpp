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
#include "sensor_msgs/PointCloud2.hpp"
#include "geometry_msgs/Pose.hpp"
#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Point.hpp"

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
    std::string tf_topic = mod.topic("pgo_tf");

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

    // Node-level config
    std::string world_frame = mod.arg("world_frame", "map");
    std::string local_frame = mod.arg("local_frame", "odom");
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
    fprintf(stderr, "  pgo_tf: %s\n", tf_topic.c_str());

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
                corr_r, corr_t, cur_time, world_frame, "base_link");
            lcm.publish(corrected_odom_topic, &corrected);

            nav_msgs::Odometry tf_msg = build_odometry(
                pgo.offsetR(), pgo.offsetT(), cur_time, world_frame, local_frame);
            lcm.publish(tf_topic, &tf_msg);

            std::this_thread::sleep_for(std::chrono::milliseconds(timer_period_ms));
            continue;
        }

        // Keyframe added
        pgo.searchForLoopPairs();
        pgo.smoothAndUpdate();

        fprintf(stderr, "PGO: keyframe %zu at (%.1f, %.1f, %.1f)\n",
                pgo.keyPoses().size(),
                cp.pose.t.x(), cp.pose.t.y(), cp.pose.t.z());

        // Publish corrected odometry
        M3D corr_r = pgo.offsetR() * cp.pose.r;
        V3D corr_t = pgo.offsetR() * cp.pose.t + pgo.offsetT();
        nav_msgs::Odometry corrected = build_odometry(
            corr_r, corr_t, cur_time, world_frame, "base_link");
        lcm.publish(corrected_odom_topic, &corrected);

        // Publish TF correction (map -> odom offset)
        nav_msgs::Odometry tf_msg = build_odometry(
            pgo.offsetR(), pgo.offsetT(), cur_time, world_frame, local_frame);
        lcm.publish(tf_topic, &tf_msg);

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
