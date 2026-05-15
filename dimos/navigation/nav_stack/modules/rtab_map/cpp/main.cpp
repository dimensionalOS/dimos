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
#include <cstdio>
#include <mutex>
#include <queue>
#include <signal.h>
#include <string>

#include <Eigen/Geometry>
#include <lcm/lcm-cpp.hpp>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/LocalGrid.h>
#include <rtabmap/core/LocalGridMaker.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
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
    // Project the cloud into the world's gravity-aligned frame (z translated
    // by pose.z) before segmentation. rtabmap's default is false, which
    // applies the height threshold in the sensor's local frame — wrong for
    // any robot whose body origin sits above the floor (e.g., G1 pose.z ~=
    // 1.2 m). With this enabled, MaxGroundHeight=0.05 is measured from the
    // world's z=0 (floor), so short obstacles like chairs aren't lumped in
    // with the ground.
    params["Grid/MapFrameProjection"] =
        mod.arg("grid_map_frame_projection", "true");
    // OctoMap log-odds knobs. Lower ProbClampingMax = more dynamic
    // (cells flip back to free quickly under empty-cell observations).
    // rtabmap's default 0.971 saturates at +3.5 log-odds and takes ~9
    // misses to clear; 0.75 saturates at +1.1 and clears in ~3 misses.
    params["GridGlobal/ProbHit"] = mod.arg("octomap_prob_hit", "0.7");
    params["GridGlobal/ProbMiss"] = mod.arg("octomap_prob_miss", "0.4");
    params["GridGlobal/ProbClampingMax"] =
        mod.arg("octomap_prob_clamping_max", "0.75");
    params["GridGlobal/ProbClampingMin"] =
        mod.arg("octomap_prob_clamping_min", "0.12");
    params["GridGlobal/OccupancyThr"] =
        mod.arg("octomap_occupancy_thr", "0.5");
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

    // Single persistent global OctoMap. Every scan's local grid is added
    // with a monotonically increasing id, so OctoMap's internal log-odds
    // accumulator handles both occupancy AND clearing:
    //   - Cells hit repeatedly accumulate +log-odds (saturated occupied).
    //   - When something moves out of view (chair rolls away), the empty
    //     cells produced by LocalGridMaker for subsequent scans pass
    //     through that location → -log-odds → eventually flips to free.
    //
    // We snapshot `mapCorrection * odom_pose` once per scan at capture
    // time. That pose never updates afterward, so OctoMap's
    // fullUpdateNeeded never fires and we don't pay full rebuild cost on
    // every loop closure. Trade-off: octomap drifts slightly relative to
    // current rtabmap state, but on minute-scale operation drift is well
    // within the cell size.
    rtabmap::LocalGridCache global_grid_cache;
    rtabmap::OctoMap global_octomap(&global_grid_cache, params);
    std::map<int, rtabmap::Transform> octomap_poses;
    // Per-scan captured grid kept in a private copy so we can rebuild
    // the LocalGridCache after eviction (rtabmap's cache has no
    // per-id remove). Stores everything OctoMap needs to re-assemble
    // the scan, plus the capture timestamp used by the eviction window.
    struct ScanEntry {
        cv::Mat ground;
        cv::Mat obstacles;
        cv::Mat empty;
        cv::Point3f view_point;
        float cell_size;
        double timestamp;
    };
    std::map<int, ScanEntry> scan_history;
    int octomap_next_id = 1;
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
    // Echo the OctoMap log-odds params so we can verify at runtime that
    // the binary actually picked up the values we expect. Mis-passed
    // params silently fall back to rtabmap's defaults.
    fprintf(stderr,
            "  OctoMap params: ProbHit=%s ProbMiss=%s ClampingMax=%s ClampingMin=%s OccupancyThr=%s\n",
            params["GridGlobal/ProbHit"].c_str(),
            params["GridGlobal/ProbMiss"].c_str(),
            params["GridGlobal/ProbClampingMax"].c_str(),
            params["GridGlobal/ProbClampingMin"].c_str(),
            params["GridGlobal/OccupancyThr"].c_str());
    fprintf(stderr,
            "  Grid params: MapFrameProjection=%s MaxGroundHeight=%s MaxObstacleHeight=%s RangeMax=%s\n",
            params["Grid/MapFrameProjection"].c_str(),
            params["Grid/MaxGroundHeight"].c_str(),
            params["Grid/MaxObstacleHeight"].c_str(),
            params["Grid/RangeMax"].c_str());

    // Lowered defaults so a fast-moving robot sees the map refresh quickly
    // (max @ ~2 Hz). Each publish is bounded by the cost of the per-signature
    // world-cloud cache concat + voxel downsample — fine at this rate even
    // with hundreds of poses.
    const double octomap_publish_period = std::stod(mod.arg("octomap_publish_period", "0.3"));
    const double global_map_publish_period = std::stod(mod.arg("global_map_publish_period", "0.5"));
    const double octomap_max_age_seconds =
        std::stod(mod.arg("octomap_max_age_seconds", "5.0"));
    const double octomap_rebuild_period =
        std::stod(mod.arg("octomap_rebuild_period", "1.0"));

    double last_octomap_publish = 0.0;
    double last_global_map_publish = 0.0;
    double last_octomap_rebuild = 0.0;
    int frame_id = 0;
    const int timer_period_ms = 30;

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

        // Compute a local occupancy grid for every scan (keyframe or not).
        // Pass the *map-frame* pose (mapCorrection * odom) so that
        // segmentation's MapFrameProjection translation uses the same
        // z that the cells will be placed at by OctoMap::assemble. With
        // raw odom_pose, after a loop closure shifts mapCorrection.z
        // the segmentation reference plane drifts from the placement
        // plane and ground/obstacle classification disagrees with where
        // cells actually land.
        cv::Mat ground, obstacles, empty;
        cv::Point3f view_point(0, 0, 0);
        const rtabmap::Transform map_pose =
            rtab.getMapCorrection() * frame.odom_pose;
        grid_maker.createLocalMap(
            laser_scan, map_pose, ground, obstacles, empty, view_point);
        const float cell_size = grid_maker.getCellSize();

        // Add the scan's local grid to the rtabmap cache + our private
        // history with a fresh monotonically-increasing id. Pose is
        // `mapCorrection * odom_pose` at capture time and frozen — we
        // deliberately do not re-stamp old scans with updated corrections,
        // which would force OctoMap's fullUpdateNeeded → clear-and-rebuild
        // path on every loop closure.
        if (!ground.empty() || !obstacles.empty() || !empty.empty()) {
            int id = octomap_next_id++;
            global_grid_cache.add(id, ground, obstacles, empty, cell_size, view_point);
            octomap_poses[id] = map_pose;
            scan_history[id] = ScanEntry{
                ground, obstacles, empty, view_point, cell_size, frame.timestamp,
            };
        }
        if (debug) {
            fprintf(stderr,
                    "[rtab DEBUG]   localmap kf=%d g=%d o=%d e=%d cellSize=%.3f cache=%zu\n",
                    processed ? 1 : 0, ground.cols, obstacles.cols, empty.cols,
                    cell_size, global_grid_cache.size());
        }

        // Time-based eviction. Drop scans older than `max_age` and rebuild
        // the OctoMap from the surviving window. Without this, a cell hit
        // once stays occupied forever (drift + LiDAR angular gaps mean
        // many cells never get a clearing ray in subsequent scans). With
        // this, walls re-observed every scan stay in the map; trails
        // observed only during a brief walk-past fall out of the window
        // and disappear after ~max_age seconds.
        if (frame.timestamp - last_octomap_rebuild >= octomap_rebuild_period
            && !scan_history.empty()) {
            last_octomap_rebuild = frame.timestamp;
            const double cutoff = frame.timestamp - octomap_max_age_seconds;
            size_t evicted = 0;
            for (auto it = scan_history.begin(); it != scan_history.end();) {
                if (it->second.timestamp < cutoff) {
                    octomap_poses.erase(it->first);
                    it = scan_history.erase(it);
                    ++evicted;
                } else {
                    ++it;
                }
            }
            if (evicted > 0) {
                // Wipe both rtabmap's grid cache and the OctoMap's octree
                // state, then re-add the surviving scans. `update()`
                // below will re-assemble them.
                global_grid_cache.clear();
                global_octomap.clear();
                for (const auto& kv : scan_history) {
                    const ScanEntry& e = kv.second;
                    global_grid_cache.add(
                        kv.first, e.ground, e.obstacles, e.empty,
                        e.cell_size, e.view_point);
                }
                if (debug) {
                    fprintf(stderr,
                            "[rtab DEBUG] evicted %zu scans older than %.1fs; window now %zu scans\n",
                            evicted, octomap_max_age_seconds, scan_history.size());
                }
            }
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

        // Periodically integrate pending scans into the global OctoMap.
        // OctoMap::update only processes ids it hasn't seen, so calling
        // this with the accumulated poses map cheaply picks up everything
        // added since last call. We gate on the octomap publish period so
        // we batch multiple scans per update instead of paying the
        // per-cell octree-write cost on every frame.
        //
        // Reset last_*_publish if the frame timestamp went backward —
        // can happen on FastLIO2 restart, NTP correction, or a stream
        // that starts on wall-clock and then switches to relative. Without
        // this, a single forward-from-now ts would gate all future
        // publishes for the duration of the regression.
        if (frame.timestamp < last_octomap_publish) last_octomap_publish = 0.0;
        if (frame.timestamp < last_global_map_publish) last_global_map_publish = 0.0;
        bool octomap_due =
            frame.timestamp - last_octomap_publish >= octomap_publish_period;
        bool global_map_due =
            frame.timestamp - last_global_map_publish >= global_map_publish_period;
        if ((octomap_due || global_map_due) && !octomap_poses.empty()) {
            global_octomap.update(octomap_poses);
        }

        // Publish OctoMap-derived outputs (throttled).
        if (octomap_due && !octomap_poses.empty()) {
            last_octomap_publish = frame.timestamp;

            // Occupied voxels.
            std::vector<int> obstacleIndices;
            auto cloud = global_octomap.createCloud(
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
            cv::Mat proj = global_octomap.createProjectionMap(xMin, yMin, cellSize);
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
                        "[rtab DEBUG] published octomap — voxels=%zu proj2d=%zu poses=%zu\n",
                        octo_points.size(), proj_points.size(), octomap_poses.size());
            }
        }

        // Publish accumulated global cloud (throttled).
        //
        // Same OctoMap as the obstacle topic, but with ground cells
        // included for a dense visualization. Sharing the OctoMap means
        // the ray-traced clearing applies to the global map too — when
        // a chair rolls away, its old hits get probabilistically cleared
        // by subsequent scans' rays, so no permanent ghost points.
        if (global_map_due && !octomap_poses.empty()) {
            last_global_map_publish = frame.timestamp;

            std::vector<int> obstacleIndices;
            std::vector<int> groundIndices;
            auto cloud = global_octomap.createCloud(
                /*treeDepth=*/0, &obstacleIndices, nullptr, &groundIndices);
            std::vector<smartnav::PointXYZI> gpoints;
            if (cloud) {
                gpoints.reserve(obstacleIndices.size() + groundIndices.size());
                for (int idx : obstacleIndices) {
                    const auto& pt = cloud->points[idx];
                    gpoints.push_back({pt.x, pt.y, pt.z, 1.0f});
                }
                for (int idx : groundIndices) {
                    const auto& pt = cloud->points[idx];
                    gpoints.push_back({pt.x, pt.y, pt.z, 0.0f});
                }
            }
            publish_pointcloud(
                lcm, global_map_topic, gpoints, world_frame, frame.timestamp);
            if (debug) {
                fprintf(stderr,
                        "[rtab DEBUG] published global_map — obstacles=%zu ground=%zu poses=%zu topic=%s\n",
                        obstacleIndices.size(), groundIndices.size(),
                        octomap_poses.size(), global_map_topic.c_str());
            }
        }
    }

    fprintf(stderr, "RtabMap native module shutting down\n");
    return 0;
}
