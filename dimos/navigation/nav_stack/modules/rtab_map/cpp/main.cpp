// RtabMap NativeModule — wraps librtabmap with LCM I/O.
//
// Subscribes to:
//   - registered_scan  (sensor_msgs.PointCloud2, lidar scan in the map/world frame)
//   - odometry         (nav_msgs.Odometry, external odometry from FastLIO2)
//
// Publishes:
//   - corrected_odometry (nav_msgs.Odometry, odom rebound to the map frame)
//   - global_map         (sensor_msgs.PointCloud2, accumulated optimized cloud)
//   - rtab_tf            (nav_msgs.Odometry carrying the `map -> odom` correction)
//   - octomap            (sensor_msgs.PointCloud2, occupied voxel centroids)
//   - projected_2d_grid  (sensor_msgs.PointCloud2, 2D projection of OctoMap)
//
// Designed for **lidar-only** mode: RGBD/Enabled=true + Reg/Strategy=1.
// Defaults match the user spec: Grid/3D=true, Grid/RayTracing=true,
// Grid/MaxGroundAngle=45, Grid/CellSize=0.1, Grid/GroundIsObstacle=false.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <queue>
#include <signal.h>
#include <string>
#include <thread>

#include <Eigen/Geometry>
#include <lcm/lcm-cpp.hpp>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/LocalGrid.h>
#include <rtabmap/core/LocalGridMaker.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/global_map/OctoMap.h>

#include "dimos_native_module.hpp"
#include "point_cloud_utils.hpp"

#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/PointCloud2.hpp"

namespace {

std::atomic<bool> g_running{true};
void signal_handler(int) { g_running.store(false); }

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;

struct ScanFrame {
    CloudType::Ptr cloud_body;
    rtabmap::Transform odom_pose;
    double timestamp = 0.0;
};

rtabmap::Transform odom_from_lcm(const nav_msgs::Odometry& msg) {
    return rtabmap::Transform(
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
}

nav_msgs::Odometry odom_to_lcm(
    const rtabmap::Transform& tf,
    double ts,
    const std::string& frame_id,
    const std::string& child_frame_id) {
    nav_msgs::Odometry msg;
    msg.header = dimos::make_header(frame_id, ts);
    msg.child_frame_id = child_frame_id;

    Eigen::Matrix3f rot;
    rot << tf.r11(), tf.r12(), tf.r13(),
           tf.r21(), tf.r22(), tf.r23(),
           tf.r31(), tf.r32(), tf.r33();
    Eigen::Quaternionf q(rot);

    msg.pose.pose.position.x = tf.x();
    msg.pose.pose.position.y = tf.y();
    msg.pose.pose.position.z = tf.z();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    return msg;
}

class Handlers {
public:
    void on_odometry(
        const lcm::ReceiveBuffer*,
        const std::string&,
        const nav_msgs::Odometry* msg) {
        double ts = msg->header.stamp.sec + msg->header.stamp.nsec / 1e9;
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_ = odom_from_lcm(*msg);
        latest_odom_ts_ = ts;
        bool first = !has_odom_;
        has_odom_ = true;
        if (debug_ && (first || (++odom_count_ % 50 == 0))) {
            fprintf(stderr,
                    "[rtab DEBUG] odom #%d ts=%.3f pos=(%.2f,%.2f,%.2f)\n",
                    odom_count_, ts, latest_odom_.x(), latest_odom_.y(), latest_odom_.z());
        }
    }

    void on_registered_scan(
        const lcm::ReceiveBuffer*,
        const std::string&,
        const sensor_msgs::PointCloud2* msg) {
        const double scan_ts = msg->header.stamp.sec + msg->header.stamp.nsec / 1e9;

        rtabmap::Transform odom_pose;
        double odom_ts = 0.0;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (!has_odom_) {
                if (debug_) {
                    fprintf(stderr,
                            "[rtab DEBUG] scan dropped — no odometry received yet (waiting on odom topic)\n");
                }
                return;
            }
            odom_pose = latest_odom_;
            odom_ts = latest_odom_ts_;
        }

        // Drop scans that are temporally far from the latest odom — rtabmap's
        // ICP is sensitive to scan/odom misalignment and pairing a stale odom
        // with a fresh scan produces silently-bad corrections.
        if (scan_ts > 0.0 && std::abs(scan_ts - odom_ts) > scan_odom_max_dt_) {
            if (debug_) {
                fprintf(stderr,
                        "[rtab DEBUG] scan dropped — |scan_ts %.3f - odom_ts %.3f| = %.3f > %.3f\n",
                        scan_ts, odom_ts, std::abs(scan_ts - odom_ts), scan_odom_max_dt_);
            }
            return;
        }

        ScanFrame frame;
        frame.cloud_body = CloudType::Ptr(new CloudType);
        smartnav::to_pcl(*msg, *frame.cloud_body);
        frame.odom_pose = odom_pose;
        // Use the scan's own timestamp — downstream consumers tag
        // corrected_odometry with this, and the scan stamp is what they
        // actually want to align against.
        frame.timestamp = scan_ts > 0.0 ? scan_ts : odom_ts;

        if (unregister_input_) {
            // Input is world-frame; convert to body for rtabmap (it expects
            // body-frame scans with sensor at origin).
            CloudType::Ptr body(new CloudType);
            Eigen::Affine3f inv(frame.odom_pose.inverse().toEigen4f());
            pcl::transformPointCloud(*frame.cloud_body, *body, inv);
            frame.cloud_body = body;
        }

        std::lock_guard<std::mutex> lock(buffer_mutex_);
        // Replace-on-queue. Per-frame work (rtabmap process + LocalGridMaker
        // ray tracing + OctoMap update) can run slower than the scan rate on
        // a real LiDAR (~10 Hz). Without a bound here the buffer grows
        // unboundedly and the consumer chews through scans from many
        // seconds ago — the published map ends up reflecting poses far
        // behind the robot's true position. Drop everything queued and
        // keep just the latest scan; rtabmap's own keyframe gate
        // (RGBD/LinearUpdate / AngularUpdate) does the temporal
        // subsampling for us.
        if (drop_stale_scans_) {
            std::queue<ScanFrame> empty;
            std::swap(buffer_, empty);
            scan_drops_ += empty.size();
        }
        buffer_.push(frame);
        if (debug_ && (++scan_count_ % 20 == 1)) {
            fprintf(stderr,
                    "[rtab DEBUG] scan #%d queued — pts=%zu odom_pos=(%.2f,%.2f,%.2f) ts=%.3f buffer=%zu dropped=%d\n",
                    scan_count_, frame.cloud_body->size(),
                    frame.odom_pose.x(), frame.odom_pose.y(), frame.odom_pose.z(),
                    frame.timestamp, buffer_.size(), scan_drops_);
        }
    }

    // Pop one buffered frame; returns false if the buffer is empty.
    bool try_pop(ScanFrame& out) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (buffer_.empty()) return false;
        out = std::move(buffer_.front());
        buffer_.pop();
        return true;
    }

    bool unregister_input_ = true;
    double scan_odom_max_dt_ = 0.2;  // seconds
    bool debug_ = false;
    bool drop_stale_scans_ = true;

private:
    std::mutex buffer_mutex_;
    std::queue<ScanFrame> buffer_;

    std::mutex odom_mutex_;
    rtabmap::Transform latest_odom_;
    bool has_odom_ = false;
    double latest_odom_ts_ = 0.0;

    int odom_count_ = 0;
    int scan_count_ = 0;
    int scan_drops_ = 0;
};

struct CachedWorldCloud {
    rtabmap::Transform pose;
    CloudType::Ptr cloud;
};

// Drop entries whose ids are no longer in the optimized pose set, and add /
// refresh entries whose pose has changed (loop closures shift poses) or that
// haven't been transformed yet. Each scan is transformed at most once per
// pose update — replaces the prior O(total_scans * points_per_scan) rebuild.
void update_world_cloud_cache(
    const rtabmap::Memory& memory,
    const std::map<int, rtabmap::Transform>& opt_poses,
    std::map<int, CachedWorldCloud>& cache) {
    for (auto it = cache.begin(); it != cache.end();) {
        if (opt_poses.find(it->first) == opt_poses.end()) {
            it = cache.erase(it);
        } else {
            ++it;
        }
    }
    for (const auto& kv : opt_poses) {
        auto it = cache.find(kv.first);
        const bool needs_update = it == cache.end() || !(it->second.pose == kv.second);
        if (!needs_update) continue;

        const rtabmap::Signature* s = memory.getSignature(kv.first);
        if (!s) continue;
        const rtabmap::LaserScan& ls = s->sensorData().laserScanRaw();
        if (ls.empty()) continue;

        const cv::Mat& m = ls.data();
        const int channels = m.channels();
        CloudType::Ptr body(new CloudType);
        body->resize(m.cols);
        for (int i = 0; i < m.cols; ++i) {
            const float* p = m.ptr<float>(0) + i * channels;
            (*body)[i].x = p[0];
            (*body)[i].y = p[1];
            (*body)[i].z = p[2];
            (*body)[i].intensity = channels >= 4 ? p[3] : 0.0f;
        }

        CloudType::Ptr world(new CloudType);
        Eigen::Affine3f t(kv.second.toEigen4f());
        pcl::transformPointCloud(*body, *world, t);

        cache[kv.first] = CachedWorldCloud{kv.second, world};
    }
}

cv::Mat scan_to_cv_mat(const CloudType& cloud) {
    cv::Mat m(1, static_cast<int>(cloud.size()), CV_32FC4);
    for (size_t i = 0; i < cloud.size(); ++i) {
        float* p = m.ptr<float>(0) + i * 4;
        p[0] = cloud.points[i].x;
        p[1] = cloud.points[i].y;
        p[2] = cloud.points[i].z;
        p[3] = cloud.points[i].intensity;
    }
    return m;
}

void publish_pointcloud(
    lcm::LCM& lcm,
    const std::string& topic,
    const std::vector<smartnav::PointXYZI>& points,
    const std::string& frame_id,
    double ts) {
    auto msg = smartnav::build_pointcloud2(points, frame_id, ts);
    lcm.publish(topic, &msg);
}

}  // namespace

int main(int argc, char** argv) {
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    dimos::NativeModule mod(argc, argv);
    const bool debug = mod.arg_bool("debug", false);

    // LCM port topics.
    const std::string scan_topic = mod.topic("registered_scan");
    const std::string odom_topic = mod.topic("odometry");
    const std::string corrected_topic = mod.topic("corrected_odometry");
    const std::string global_map_topic = mod.topic("global_map");
    const std::string tf_topic = mod.topic("rtab_tf");
    const std::string octomap_topic = mod.topic("octomap");
    const std::string proj2d_topic = mod.topic("projected_2d_grid");

    // Frame names.
    const std::string world_frame = mod.arg("world_frame", "map");
    const std::string local_frame = mod.arg("local_frame", "odom");
    const std::string body_frame = mod.arg("body_frame", "body");

    // RTAB-Map parameters. Defaults match the user spec.
    rtabmap::ParametersMap params;
    params["Grid/3D"] = mod.arg("grid_3d", "true");
    params["Grid/RayTracing"] = mod.arg("grid_ray_tracing", "true");
    params["Grid/FromDepth"] = mod.arg("grid_from_depth", "false");
    params["Grid/CellSize"] = mod.arg("grid_cell_size", "0.1");
    params["Grid/MaxGroundAngle"] = mod.arg("grid_max_ground_angle", "45");
    params["Grid/GroundIsObstacle"] = mod.arg("grid_ground_is_obstacle", "false");
    params["Grid/FlatObstacleDetected"] =
        mod.arg("grid_flat_obstacle_detected", "true");
    // Height-based ground segmentation by default. Synthetic scans for tests
    // rarely have normals, and rtabmap's normal-based segmentation needs a
    // dense neighborhood; height thresholds are more robust here. Defaults
    // can still be overridden via the wrapper's config dict.
    params["Grid/NormalsSegmentation"] =
        mod.arg("grid_normals_segmentation", "false");
    params["Grid/MaxObstacleHeight"] =
        mod.arg("grid_max_obstacle_height", "2.0");
    params["Grid/MaxGroundHeight"] =
        mod.arg("grid_max_ground_height", "0.05");
    params["Grid/RangeMax"] = mod.arg("grid_range_max", "8.0");
    // Lidar-only mode.
    params["RGBD/Enabled"] = "true";
    params["Reg/Strategy"] = "1";  // ICP
    params["Mem/IncrementalMemory"] = "true";
    // Keep odometry covariance contributions modest so an externally-supplied
    // FastLIO2 odom dominates.
    params["Reg/Force3DoF"] = "false";
    params["RGBD/CreateOccupancyGrid"] = "true";
    // One-to-many proximity detection (compare current keyframe against
    // merged neighbor scans). Without this, lidar-only mode only triggers
    // closure via visual bag-of-words, which we don't have. 10 neighbors
    // is rtabmap's recommended starting point for LiDAR.
    params["RGBD/ProximityPathMaxNeighbors"] =
        mod.arg("rgbd_proximity_path_max_neighbors", "10");
    // Keyframe admission gate. Default to 10cm linear / ~6° angular —
    // plenty fine for real-time mapping on a 0.6 m/s robot while
    // letting per-frame rtabmap work (ICP, OctoMap update) run faster
    // than the scan rate. Synthetic tests with stationary input override
    // these to 0 so every frame admits.
    params["Rtabmap/DetectionRate"] =
        mod.arg("rtabmap_detection_rate", "0");  // 0 = motion-gated, not time-gated
    params["RGBD/LinearUpdate"] = mod.arg("rgbd_linear_update", "0.1");
    params["RGBD/AngularUpdate"] = mod.arg("rgbd_angular_update", "0.1");
    params["Mem/NotLinkedNodesKept"] = "false";

    rtabmap::Rtabmap rtab;
    rtab.init(params);

    rtabmap::LocalGridCache grid_cache;
    rtabmap::OctoMap octomap(&grid_cache, params);
    // Drives the per-signature local occupancy grid we feed into the OctoMap.
    // rtabmap's own pipeline only computes these grids automatically when
    // depth+camera is present; in lidar-only mode we must populate them
    // ourselves via LocalGridMaker::createLocalMap().
    rtabmap::LocalGridMaker grid_maker(params);

    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "RtabMap: LCM init failed\n");
        return 1;
    }

    Handlers handlers;
    handlers.unregister_input_ = mod.arg_bool("unregister_input", true);
    handlers.scan_odom_max_dt_ = std::stod(mod.arg("scan_odom_max_dt", "0.2"));
    handlers.debug_ = debug;
    handlers.drop_stale_scans_ = mod.arg_bool("drop_stale_scans", true);
    lcm.subscribe(odom_topic, &Handlers::on_odometry, &handlers);
    lcm.subscribe(scan_topic, &Handlers::on_registered_scan, &handlers);

    fprintf(stderr, "RtabMap native module started\n");
    fprintf(stderr, "  registered_scan: %s\n", scan_topic.c_str());
    fprintf(stderr, "  odometry: %s\n", odom_topic.c_str());
    fprintf(stderr, "  corrected_odometry: %s\n", corrected_topic.c_str());
    fprintf(stderr, "  global_map: %s\n", global_map_topic.c_str());
    fprintf(stderr, "  rtab_tf: %s\n", tf_topic.c_str());
    fprintf(stderr, "  octomap: %s\n", octomap_topic.c_str());
    fprintf(stderr, "  projected_2d_grid: %s\n", proj2d_topic.c_str());

    // Lowered defaults so a fast-moving robot sees the map refresh quickly
    // (max @ ~2 Hz). Each publish is bounded by the cost of the per-signature
    // world-cloud cache concat + voxel downsample — fine at this rate even
    // with hundreds of poses.
    const double octomap_publish_period = std::stod(mod.arg("octomap_publish_period", "0.3"));
    const double global_map_publish_period = std::stod(mod.arg("global_map_publish_period", "0.5"));

    double last_octomap_publish = 0.0;
    double last_global_map_publish = 0.0;
    int frame_id = 0;
    const int timer_period_ms = 30;
    std::map<int, CachedWorldCloud> world_cloud_cache;
    // Negative ids identify non-keyframe scans whose local grid we still
    // integrate into the OctoMap, so dynamic changes (hole in the floor,
    // moved object) propagate on every frame, not only on keyframes.
    int transient_grid_id = 0;
    std::map<int, rtabmap::Transform> transient_grid_poses;
    constexpr int kTransientGridLimit = 30;

    while (g_running.load()) {
        // Block waiting for at least one LCM event (or shutdown). When a scan
        // arrives the Handlers callback pushes it onto the buffer, and
        // handleTimeout returns >0. timer_period_ms caps the wait so we
        // still notice g_running going false.
        lcm.handleTimeout(timer_period_ms);
        while (lcm.handleTimeout(0) > 0) {}

        ScanFrame frame;
        if (!handlers.try_pop(frame)) {
            continue;
        }

        // Build SensorData with the lidar scan. rtabmap has no scan-only
        // ctor — the canonical lidar-only setup is an empty SensorData with
        // setLaserScan(scan); using one of the RGB-D+scan ctors with empty
        // rgb/depth/camera makes rtabmap treat the data as invalid and skip
        // grid generation entirely.
        cv::Mat scan_mat = scan_to_cv_mat(*frame.cloud_body);
        rtabmap::LaserScan laser_scan =
            rtabmap::LaserScan::backwardCompatibility(scan_mat);
        rtabmap::SensorData data;
        data.setLaserScan(laser_scan);
        data.setId(++frame_id);
        data.setStamp(frame.timestamp);

        bool processed = rtab.process(data, frame.odom_pose);
        if (debug) {
            if (processed) {
                fprintf(stderr,
                        "[rtab DEBUG] frame #%d processed — odom_pos=(%.2f,%.2f,%.2f) ts=%.3f\n",
                        frame_id, frame.odom_pose.x(), frame.odom_pose.y(),
                        frame.odom_pose.z(), frame.timestamp);
            } else {
                fprintf(stderr,
                        "[rtab DEBUG] frame #%d non-keyframe (motion gate)\n", frame_id);
            }
        }

        // Compute and integrate a local occupancy grid for EVERY scan
        // (not gated on keyframe admission), so dynamic obstacle changes
        // — a hole opening up in the floor, an object that wasn't there
        // last pass — show up via OctoMap's log-odds raycasting clearing.
        // Keyframes get rtabmap's own signature id (so loop-closure
        // optimization shifts them); non-keyframe scans get a private,
        // monotonically-decreasing negative id and a transient pose.
        cv::Mat ground, obstacles, empty;
        cv::Point3f view_point(0, 0, 0);
        grid_maker.createLocalMap(
            laser_scan, frame.odom_pose, ground, obstacles, empty, view_point);

        int local_grid_id = 0;
        rtabmap::Transform local_grid_pose;
        bool added_to_cache = false;
        if (!ground.empty() || !obstacles.empty() || !empty.empty()) {
            const rtabmap::Signature* sig =
                processed && rtab.getMemory()
                    ? rtab.getMemory()->getLastWorkingSignature()
                    : nullptr;
            if (sig) {
                local_grid_id = sig->id();
                local_grid_pose = frame.odom_pose;
            } else {
                local_grid_id = --transient_grid_id;
                local_grid_pose = rtab.getMapCorrection() * frame.odom_pose;
                transient_grid_poses[local_grid_id] = local_grid_pose;
                // FIFO-evict older transient grids so the cache doesn't
                // grow unboundedly across the session.
                while (
                    static_cast<int>(transient_grid_poses.size())
                    > kTransientGridLimit) {
                    int oldest = transient_grid_poses.begin()->first;
                    transient_grid_poses.erase(transient_grid_poses.begin());
                }
            }
            grid_cache.add(
                local_grid_id, ground, obstacles, empty,
                grid_maker.getCellSize(), view_point);
            added_to_cache = true;
        }
        if (debug && added_to_cache) {
            fprintf(stderr,
                    "[rtab DEBUG]   localmap id=%d kf=%d g=%d o=%d e=%d cellSize=%.3f cache=%zu transient=%zu\n",
                    local_grid_id, processed ? 1 : 0,
                    ground.cols, obstacles.cols, empty.cols,
                    grid_maker.getCellSize(),
                    grid_cache.size(), transient_grid_poses.size());
        }

        // Update OctoMap against the current optimized poses + our
        // transient (non-keyframe) poses. The OctoMap's log-odds
        // accumulation gives us dynamic-obstacle behavior across passes:
        // a voxel that the latest scan rays through gets cleared even if
        // an earlier scan marked it occupied.
        const std::map<int, rtabmap::Transform>& opt_poses =
            rtab.getLocalOptimizedPoses();
        if (!opt_poses.empty() || !transient_grid_poses.empty()) {
            std::map<int, rtabmap::Transform> combined_poses(opt_poses);
            for (const auto& kv : transient_grid_poses) {
                combined_poses.insert(kv);
            }
            octomap.update(combined_poses);
        }

        // Publish corrected odometry and map->odom correction every frame.
        rtabmap::Transform correction = rtab.getMapCorrection();
        rtabmap::Transform corrected_pose = correction * frame.odom_pose;

        auto corrected_msg = odom_to_lcm(
            corrected_pose, frame.timestamp, world_frame, body_frame);
        lcm.publish(corrected_topic, &corrected_msg);

        auto tf_msg = odom_to_lcm(
            correction, frame.timestamp, world_frame, local_frame);
        lcm.publish(tf_topic, &tf_msg);

        // Publish OctoMap-derived outputs (throttled).
        if (frame.timestamp - last_octomap_publish >= octomap_publish_period) {
            last_octomap_publish = frame.timestamp;

            // Occupied voxels.
            std::vector<int> obstacleIndices;
            auto cloud = octomap.createCloud(
                /*treeDepth=*/0, &obstacleIndices, nullptr, nullptr);
            std::vector<smartnav::PointXYZI> octo_points;
            if (cloud) {
                octo_points.reserve(obstacleIndices.size());
                for (int idx : obstacleIndices) {
                    const auto& pt = cloud->points[idx];
                    octo_points.push_back({pt.x, pt.y, pt.z, 1.0f});
                }
            }
            publish_pointcloud(
                lcm, octomap_topic, octo_points, world_frame, frame.timestamp);

            // Projected 2D map. createProjectionMap returns a single-channel
            // cv::Mat where free=0/unknown=-1/occupied=100; convert occupied
            // cells to a point cloud at z=0 so downstream consumers can plot
            // it cheaply.
            float xMin = 0, yMin = 0, cellSize = 0;
            cv::Mat proj = octomap.createProjectionMap(xMin, yMin, cellSize);
            std::vector<smartnav::PointXYZI> proj_points;
            if (!proj.empty() && cellSize > 0) {
                proj_points.reserve(proj.rows * proj.cols / 8);
                for (int r = 0; r < proj.rows; ++r) {
                    for (int c = 0; c < proj.cols; ++c) {
                        if (proj.at<int8_t>(r, c) > 0) {
                            float x = xMin + (c + 0.5f) * cellSize;
                            float y = yMin + (r + 0.5f) * cellSize;
                            proj_points.push_back({x, y, 0.0f, 1.0f});
                        }
                    }
                }
            }
            publish_pointcloud(
                lcm, proj2d_topic, proj_points, world_frame, frame.timestamp);
            if (debug) {
                fprintf(stderr,
                        "[rtab DEBUG] published octomap — voxels=%zu proj2d=%zu\n",
                        octo_points.size(), proj_points.size());
            }
        }

        // Publish accumulated global cloud (throttled).
        //
        // Built from per-signature world-frame caches: each signature's body
        // scan gets transformed once when its pose first appears (and again
        // only when a loop closure shifts that pose). The previous version
        // re-transformed every scan on every publish, which was O(N*S) per
        // tick and tanked perf on long runs.
        if (frame.timestamp - last_global_map_publish >= global_map_publish_period) {
            last_global_map_publish = frame.timestamp;

            if (rtab.getMemory()) {
                update_world_cloud_cache(*rtab.getMemory(), opt_poses, world_cloud_cache);
            }

            CloudType::Ptr global_cloud(new CloudType);
            for (const auto& kv : opt_poses) {
                auto it = world_cloud_cache.find(kv.first);
                if (it == world_cloud_cache.end()) continue;
                *global_cloud += *(it->second.cloud);
            }

            float voxel = static_cast<float>(std::stod(mod.arg("global_map_voxel_size", "0.15")));
            if (!global_cloud->empty() && voxel > 0) {
                CloudType::Ptr filtered(new CloudType);
                pcl::VoxelGrid<PointType> vg;
                vg.setInputCloud(global_cloud);
                vg.setLeafSize(voxel, voxel, voxel);
                vg.filter(*filtered);
                global_cloud = filtered;
            }

            sensor_msgs::PointCloud2 gmsg =
                smartnav::from_pcl(*global_cloud, world_frame, frame.timestamp);
            lcm.publish(global_map_topic, &gmsg);
            if (debug) {
                fprintf(stderr,
                        "[rtab DEBUG] published global_map — pts=%zu poses=%zu cache=%zu topic=%s\n",
                        global_cloud->size(), opt_poses.size(), world_cloud_cache.size(),
                        global_map_topic.c_str());
            }
        }
    }

    fprintf(stderr, "RtabMap native module shutting down\n");
    return 0;
}
