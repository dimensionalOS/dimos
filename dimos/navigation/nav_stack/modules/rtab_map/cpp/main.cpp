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
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/LocalGrid.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/global_map/OctoMap.h>
#include <rtabmap/utilite/ULogger.h>

#include "dimos_native_module.hpp"
#include "point_cloud_utils.hpp"

#include "geometry_msgs/PoseStamped.hpp"
#include "nav_msgs/Odometry.hpp"
#include "nav_msgs/Path.hpp"
#include "sensor_msgs/Image.hpp"
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

// Decode an LCM sensor_msgs::Image into a BGR cv::Mat (rtabmap's canonical
// internal layout). Returns an empty Mat on unsupported encodings so the
// caller can skip the frame without crashing the binary.
cv::Mat decode_lcm_image(const sensor_msgs::Image& msg) {
    if (msg.height <= 0 || msg.width <= 0 || msg.data.empty()) {
        return cv::Mat();
    }
    const std::string& enc = msg.encoding;

    if (enc == "jpeg") {
        cv::Mat raw(1, static_cast<int>(msg.data.size()), CV_8UC1,
                    const_cast<uint8_t*>(msg.data.data()));
        cv::Mat decoded = cv::imdecode(raw, cv::IMREAD_COLOR);  // BGR
        return decoded;
    }

    int cv_type = -1;
    int channels = 0;
    bool input_is_rgb = false;
    bool input_is_gray = false;
    if (enc == "bgr8") {
        cv_type = CV_8UC3;
        channels = 3;
    } else if (enc == "rgb8") {
        cv_type = CV_8UC3;
        channels = 3;
        input_is_rgb = true;
    } else if (enc == "bgra8") {
        cv_type = CV_8UC4;
        channels = 4;
    } else if (enc == "rgba8") {
        cv_type = CV_8UC4;
        channels = 4;
        input_is_rgb = true;
    } else if (enc == "mono8") {
        cv_type = CV_8UC1;
        channels = 1;
        input_is_gray = true;
    } else {
        return cv::Mat();
    }

    const size_t expected = static_cast<size_t>(msg.height) *
                            static_cast<size_t>(msg.width) *
                            static_cast<size_t>(channels);
    if (msg.data.size() < expected) {
        return cv::Mat();
    }

    cv::Mat raw(msg.height, msg.width, cv_type,
                const_cast<uint8_t*>(msg.data.data()));
    cv::Mat bgr;
    if (input_is_gray) {
        cv::cvtColor(raw, bgr, cv::COLOR_GRAY2BGR);
    } else if (channels == 4 && input_is_rgb) {
        cv::cvtColor(raw, bgr, cv::COLOR_RGBA2BGR);
    } else if (channels == 4) {
        cv::cvtColor(raw, bgr, cv::COLOR_BGRA2BGR);
    } else if (input_is_rgb) {
        cv::cvtColor(raw, bgr, cv::COLOR_RGB2BGR);
    } else {
        // Already BGR — clone so we own the buffer after `msg` dies.
        bgr = raw.clone();
    }
    return bgr;
}

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

    void on_color_image(
        const lcm::ReceiveBuffer*,
        const std::string&,
        const sensor_msgs::Image* msg) {
        cv::Mat decoded = decode_lcm_image(*msg);
        if (decoded.empty()) {
            if (debug_) {
                fprintf(stderr,
                        "[rtab DEBUG] color_image dropped — unsupported encoding '%s' or empty data\n",
                        msg->encoding.c_str());
            }
            return;
        }
        const double ts = msg->header.stamp.sec + msg->header.stamp.nsec / 1e9;
        std::lock_guard<std::mutex> lock(rgb_mutex_);
        latest_rgb_ = decoded;
        latest_rgb_ts_ = ts;
        has_rgb_ = true;
        if (debug_ && (++rgb_count_ % 30 == 1)) {
            fprintf(stderr,
                    "[rtab DEBUG] color_image #%d ts=%.3f size=%dx%d enc=%s\n",
                    rgb_count_, ts, decoded.cols, decoded.rows, msg->encoding.c_str());
        }
    }

    // Return the latest RGB frame if its timestamp is within `max_dt` of
    // `ref_ts`. Returns empty Mat if no RGB has arrived yet or it's too stale.
    cv::Mat latest_rgb_within(double ref_ts, double max_dt, double* out_ts = nullptr) {
        std::lock_guard<std::mutex> lock(rgb_mutex_);
        if (!has_rgb_) return cv::Mat();
        if (max_dt > 0.0 && std::abs(ref_ts - latest_rgb_ts_) > max_dt) {
            return cv::Mat();
        }
        if (out_ts) *out_ts = latest_rgb_ts_;
        // Clone so callers can mutate / hold past the next callback.
        return latest_rgb_.clone();
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

    std::mutex rgb_mutex_;
    cv::Mat latest_rgb_;
    bool has_rgb_ = false;
    double latest_rgb_ts_ = 0.0;

    int odom_count_ = 0;
    int scan_count_ = 0;
    int scan_drops_ = 0;
    int rgb_count_ = 0;
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

// Build a single-edge pose_graph_edges message — two PoseStamped entries
// (start, end) whose orientation.w encodes edge type. The KITTI scorer
// matches edges back to source-frame ids via the PoseStamped timestamps,
// so we copy each signature's scan timestamp into the header.
//
// This mirrors PGO's `build_pose_graph_edges` wire contract:
//   orientation.w == 1.0 → odometry edge
//   orientation.w == 0.4 → loop closure (the value the scorer checks for)
nav_msgs::Path build_loop_edge(
    const rtabmap::Transform& start_pose,
    double start_ts,
    const rtabmap::Transform& end_pose,
    double end_ts,
    double traversability,
    const std::string& frame_id) {
    nav_msgs::Path msg;
    msg.header = dimos::make_header(frame_id, end_ts);
    auto pack = [&](const rtabmap::Transform& tf, double ts) {
        geometry_msgs::PoseStamped p;
        p.header = dimos::make_header(frame_id, ts);
        p.pose.position.x = tf.x();
        p.pose.position.y = tf.y();
        p.pose.position.z = tf.z();
        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;
        p.pose.orientation.z = 0.0;
        p.pose.orientation.w = traversability;
        return p;
    };
    msg.poses.push_back(pack(start_pose, start_ts));
    msg.poses.push_back(pack(end_pose, end_ts));
    msg.poses_length = static_cast<int32_t>(msg.poses.size());
    return msg;
}

// Extract local grids from any new signatures in rtabmap's optimized
// poses set, push them into `cache` keyed by signature id so `OctoMap::
// update(poses)` can look them up. rtabmap stores grids on each
// Signature's SensorData; they're populated by Memory when
// `RGBD/CreateOccupancyGrid=true`. Raw fields may be cleared after
// compression on insert, so we fall back to uncompressing when raw is
// empty.
//
// `max_synced_id` is the highest signature id we've already pulled into
// the cache. rtabmap assigns ids monotonically, so we can iterate
// opt_poses from the upper_bound of that id rather than walking every
// pose every scan — the latter would scale linearly with map size and
// dominate the per-scan cost on long runs.
void sync_signature_grids(
    const rtabmap::Rtabmap& rtab,
    rtabmap::LocalGridCache& cache,
    int& max_synced_id) {
    const rtabmap::Memory* memory = rtab.getMemory();
    if (!memory) return;
    const auto& poses = rtab.getLocalOptimizedPoses();
    for (auto it = poses.upper_bound(max_synced_id); it != poses.end(); ++it) {
        int id = it->first;
        if (id <= 0) continue;
        max_synced_id = std::max(max_synced_id, id);
        const rtabmap::Signature* sig = memory->getSignature(id);
        if (!sig) continue;
        const rtabmap::SensorData& sd = sig->sensorData();
        if (sd.gridCellSize() <= 0.0f) continue;
        cv::Mat ground = sd.gridGroundCellsRaw();
        cv::Mat obstacles = sd.gridObstacleCellsRaw();
        cv::Mat empty_cells = sd.gridEmptyCellsRaw();
        if (ground.empty() && obstacles.empty() && empty_cells.empty()) {
            // setOccupancyGrid auto-compresses and may clear the raw
            // mats. Decompress to recover them.
            ground = rtabmap::uncompressData(sd.gridGroundCellsCompressed());
            obstacles = rtabmap::uncompressData(sd.gridObstacleCellsCompressed());
            empty_cells = rtabmap::uncompressData(sd.gridEmptyCellsCompressed());
        }
        if (ground.empty() && obstacles.empty() && empty_cells.empty()) continue;
        cache.add(
            id, ground, obstacles, empty_cells,
            sd.gridCellSize(), sd.gridViewPoint());
    }
}

}  // namespace

int main(int argc, char** argv) {
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    dimos::NativeModule mod(argc, argv);
    const bool debug = mod.arg_bool("debug", false);
    // Surface rtabmap-internal warnings/errors (and debug lines when the
    // wrapper's --debug=true is set) so we can see what rtabmap thinks is
    // happening — particularly useful for chasing silently-skipped loop
    // closure / proximity detection paths.
    ULogger::setType(ULogger::kTypeConsole);
    // kDebug when --debug=true so we can see e.g. "nearestIds=X/Y" and
    // "nearestPaths=N" UDEBUG lines that pinpoint why proximity-by-space
    // candidate selection returns nothing.
    ULogger::setLevel(debug ? ULogger::kDebug : ULogger::kWarning);

    // LCM port topics.
    const std::string scan_topic = mod.topic("registered_scan");
    const std::string odom_topic = mod.topic("odometry");
    const std::string corrected_topic = mod.topic("corrected_odometry");
    const std::string global_map_topic = mod.topic("global_map");
    const std::string tf_topic = mod.topic("rtab_tf");
    const std::string octomap_topic = mod.topic("octomap");
    const std::string proj2d_topic = mod.topic("projected_2d_grid");
    const std::string pose_graph_edges_topic = mod.topic("pose_graph_edges");
    const std::string loop_closure_topic = mod.topic("loop_closure");
    // Optional RGB input. Only present if the Python wrapper connected
    // an Image source to color_image — when unconnected, NativeModule's
    // _collect_topics skips emitting --color_image, so `has("color_image")`
    // is false and we stay in lidar-only mode.
    const bool color_image_arg_present = mod.has("color_image");
    const std::string color_image_topic =
        color_image_arg_present ? mod.topic("color_image") : "";

    // Frame names.
    const std::string world_frame = mod.arg("world_frame", "map");
    const std::string local_frame = mod.arg("local_frame", "odom");
    const std::string body_frame = mod.arg("body_frame", "body");

    // RTAB-Map parameters. Defaults match the user spec.
    rtabmap::ParametersMap params;
    params["Grid/3D"] = mod.arg("grid_3d", "true");
    params["Grid/RayTracing"] = mod.arg("grid_ray_tracing", "true");
    // Grid/Sensor: 0=laser scan, 1=depth, 2=both. rtabmap's default is 1
    // (depth); we feed only laser scans, so it must be 0 or rtabmap's
    // internal LocalGridMaker silently skips every scan and signatures
    // get no attached grids. (The legacy `Grid/FromDepth` knob was
    // removed in rtabmap 0.20.15 — set this directly.)
    params["Grid/Sensor"] = mod.arg("grid_sensor", "0");
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
    // Disable visual feature extraction entirely. rtabmap's default
    // Kp/DetectorStrategy=8 (SURF) tries to pull visual descriptors out
    // of every SensorData; with no RGB attached the Bayes filter never
    // gets a likelihood vector, no hypothesis ever rises above zero,
    // and (load-bearing observation as of 2026-05-16) proximity
    // detection never even runs — we observed proximityDetectionId=0
    // on every frame of a 500-scan KITTI-360 seq 2 run. Setting this
    // to -1 takes the visual path completely out of the loop-closure
    // pipeline so the spatial-proximity branch can fire.
    params["Kp/DetectorStrategy"] = mod.arg("kp_detector_strategy", "-1");
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
    // Spatial proximity detection: search the local pose-graph for
    // candidates within RGBD/LocalRadius of the current keyframe. ON by
    // default in rtabmap, but explicit here so it's tunable.
    params["RGBD/ProximityBySpace"] =
        mod.arg("rgbd_proximity_by_space", "true");
    // Spatial search radius around the current keyframe — rtabmap default
    // is 10m. For KITTI-360-style outdoor scenes with 4m GT loops, this is
    // already generous; for tight indoor use, drop to 2-3m.
    params["RGBD/LocalRadius"] =
        mod.arg("rgbd_local_radius", "10");
    // Max pose-graph depth for proximity candidate search. Default 50;
    // raise this to find loop closures further back in the graph.
    params["RGBD/ProximityMaxGraphDepth"] =
        mod.arg("rgbd_proximity_max_graph_depth", "50");
    // ICP correspondence distance — rtabmap's default 0.05m is very tight
    // for outdoor LiDAR (~10cm voxelization is common). 0.5m is forgiving.
    params["Icp/MaxCorrespondenceDistance"] =
        mod.arg("icp_max_correspondence_distance", "0.5");
    // Keyframe admission. We bypass rtabmap's motion gate (LinearUpdate=0)
    // because for the dynamic-clearing use case we want keyframes to keep
    // arriving even on a stationary robot — so the OctoMap keeps getting
    // empty-cell evidence to probabilistically clear stale obstacles.
    //
    // Rtabmap/DetectionRate is only consumed by RtabmapThread (we call
    // Rtabmap::process directly), so we apply our own time gate at the
    // main-loop level: see `rtabmap_process_period` further down. Setting
    // the rtabmap-side knob to 0 here just to keep its internal state
    // matching ours.
    params["Rtabmap/DetectionRate"] = "0";
    // LOAD-BEARING ZERO. Not a perf knob — it's what keeps dynamic
    // clearing alive on a stationary robot. With a non-zero motion gate
    // rtabmap admits no keyframes while the robot stands still, the
    // OctoMap stops getting fresh empty-cell observations, and a chair
    // that rolls away never gets cleared. If you ever add a wrapper-side
    // motion gate to bound stationary keyframe accumulation (the
    // "memory bomb" follow-up), you MUST pair it with a force-refresh
    // timer or this clearing story dies silently.
    params["RGBD/LinearUpdate"] = mod.arg("rgbd_linear_update", "0");
    params["RGBD/AngularUpdate"] = mod.arg("rgbd_angular_update", "0");
    // Keep signatures around so their local grids stay in memory and can
    // be re-assembled into the OctoMap after loop closures shift their
    // poses. The wrapper used to disable this and maintain a parallel
    // cache; the parallel cache wasn't getting pose corrections, so old
    // map regions stayed at pre-closure poses forever. With this true,
    // `rtab.getLocalOptimizedPoses()` returns the up-to-date pose for
    // every kept signature, and `OctoMap::update` propagates corrections
    // via its fullUpdateNeeded → clear-and-rebuild path.
    params["Mem/NotLinkedNodesKept"] =
        mod.arg("mem_not_linked_nodes_kept", "true");

    rtabmap::Rtabmap rtab;
    rtab.init(params);

    // Persistent OctoMap driven by rtabmap's pose-graph-optimized poses.
    //
    // Each call to `OctoMap::update(rtab.getLocalOptimizedPoses())`:
    //   - Picks up new signatures that rtabmap admitted as keyframes
    //     since last call (and grabs their grids from our cache below).
    //   - When a loop closure shifts pre-existing signature poses,
    //     rtabmap's `fullUpdateNeeded` fires and the octree is wiped and
    //     re-assembled at the corrected poses — single coherent map,
    //     no leftover ghost chunks from pre-closure pose frames.
    //
    // `octomap_grid_cache` mirrors rtabmap's per-signature local grids
    // because rtabmap stores them on each Signature (raw or compressed),
    // not in a directly-shareable LocalGridCache. We populate the cache
    // by walking signatures whenever we see new ids in opt_poses.
    rtabmap::LocalGridCache octomap_grid_cache;
    rtabmap::OctoMap global_octomap(&octomap_grid_cache, params);
    int max_synced_id = 0;

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

    // RGB plumbing. Two gates: (a) the wrapper must have wired up a
    // color_image source (topic arg present), and (b) the user must
    // have set non-zero fx/fy so we can build a valid CameraModel.
    // Either missing → silently lidar-only.
    const bool color_image_enabled = mod.arg_bool("color_image_enabled", false);
    const double camera_fx = std::stod(mod.arg("camera_fx", "0"));
    const double camera_fy = std::stod(mod.arg("camera_fy", "0"));
    const double camera_cx = std::stod(mod.arg("camera_cx", "0"));
    const double camera_cy = std::stod(mod.arg("camera_cy", "0"));
    const int camera_image_width = std::stoi(mod.arg("camera_image_width", "0"));
    const int camera_image_height = std::stoi(mod.arg("camera_image_height", "0"));
    const double rgb_max_dt = std::stod(mod.arg("rgb_max_dt", "0.2"));
    const bool rgb_intrinsics_valid = camera_fx > 0.0 && camera_fy > 0.0;
    const bool rgb_active =
        color_image_arg_present && color_image_enabled && rgb_intrinsics_valid;

    // Camera→body rigid transform (defaults to identity).
    rtabmap::Transform camera_local_transform(
        static_cast<float>(std::stod(mod.arg("camera_local_x", "0"))),
        static_cast<float>(std::stod(mod.arg("camera_local_y", "0"))),
        static_cast<float>(std::stod(mod.arg("camera_local_z", "0"))),
        static_cast<float>(std::stod(mod.arg("camera_local_qx", "0"))),
        static_cast<float>(std::stod(mod.arg("camera_local_qy", "0"))),
        static_cast<float>(std::stod(mod.arg("camera_local_qz", "0"))),
        static_cast<float>(std::stod(mod.arg("camera_local_qw", "1"))));

    rtabmap::CameraModel camera_model;
    if (rgb_active) {
        camera_model = rtabmap::CameraModel(
            camera_fx, camera_fy, camera_cx, camera_cy,
            camera_local_transform,
            /*Tx=*/0.0,
            cv::Size(camera_image_width, camera_image_height));
        lcm.subscribe(color_image_topic, &Handlers::on_color_image, &handlers);
    } else if (color_image_arg_present && color_image_enabled && !rgb_intrinsics_valid) {
        fprintf(stderr,
                "RtabMap: color_image stream connected but camera_fx/fy not set — "
                "RGB frames will be ignored. Set camera_fx/fy/cx/cy in RtabMapConfig.\n");
    }

    fprintf(stderr, "RtabMap native module started\n");
    fprintf(stderr, "  registered_scan: %s\n", scan_topic.c_str());
    fprintf(stderr, "  odometry: %s\n", odom_topic.c_str());
    fprintf(stderr, "  corrected_odometry: %s\n", corrected_topic.c_str());
    fprintf(stderr, "  global_map: %s\n", global_map_topic.c_str());
    fprintf(stderr, "  rtab_tf: %s\n", tf_topic.c_str());
    fprintf(stderr, "  octomap: %s\n", octomap_topic.c_str());
    fprintf(stderr, "  projected_2d_grid: %s\n", proj2d_topic.c_str());
    fprintf(stderr, "  pose_graph_edges: %s\n", pose_graph_edges_topic.c_str());
    fprintf(stderr, "  loop_closure: %s\n", loop_closure_topic.c_str());
    if (rgb_active) {
        fprintf(stderr,
                "  color_image: %s (fx=%.2f fy=%.2f cx=%.2f cy=%.2f size=%dx%d max_dt=%.2fs)\n",
                color_image_topic.c_str(), camera_fx, camera_fy, camera_cx, camera_cy,
                camera_image_width, camera_image_height, rgb_max_dt);
    } else if (color_image_arg_present) {
        fprintf(stderr,
                "  color_image: %s (connected but disabled — set color_image_enabled=True and camera_fx/fy)\n",
                color_image_topic.c_str());
    }
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
    // How often we let rtabmap admit a new keyframe. With RGBD/LinearUpdate=0
    // every call to rtab.process() admits a keyframe; gating at our level
    // here keeps the keyframe rate sane on a stationary robot (and the
    // ICP / signature-creation cost bounded) while still giving regular
    // OctoMap empty-cell observations for dynamic clearing. Set to 0 to
    // process every scan as a keyframe — only useful for synthetic tests.
    const double rtabmap_process_period =
        std::stod(mod.arg("rtabmap_process_period", "0.5"));

    double last_octomap_publish = 0.0;
    double last_global_map_publish = 0.0;
    double last_rtab_process = -1.0;  // -1 sentinel forces the first frame through
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

        // Time-gate rtab.process at the main-loop level. rtabmap's own
        // Rtabmap/DetectionRate is consumed by RtabmapThread, not by
        // Rtabmap::process directly, so we do the rate-limiting here.
        // With LinearUpdate=0 every call admits a keyframe; this gate is
        // what bounds the keyframe rate (and ICP / Memory cost) on
        // stationary robots. Tests can set period=0 to drop the gate.
        const bool gate_open =
            rtabmap_process_period <= 0.0
            || last_rtab_process < 0.0
            || frame.timestamp - last_rtab_process >= rtabmap_process_period;
        if (!gate_open) {
            // Still publish corrected_odometry + rtab_tf for this frame
            // using the last-known correction (jumps to that section
            // below via a flag).
            rtabmap::Transform correction = rtab.getMapCorrection();
            rtabmap::Transform corrected_pose = correction * frame.odom_pose;
            auto corrected_msg = odom_to_lcm(
                corrected_pose, frame.timestamp, world_frame, body_frame);
            lcm.publish(corrected_topic, &corrected_msg);
            auto tf_msg = odom_to_lcm(
                correction, frame.timestamp, world_frame, local_frame);
            lcm.publish(tf_topic, &tf_msg);
            continue;
        }
        last_rtab_process = frame.timestamp;

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

        // Attach the latest RGB frame if its timestamp is close enough to
        // the scan. rtabmap stores the image on the resulting signature
        // (visible in any rtabmap-databaseViewer dump) and — when feature
        // extraction is enabled via Kp/* params — runs visual bag-of-words
        // loop closure against the kept descriptors. With Reg/Strategy=1
        // (ICP) the registration itself stays lidar-driven; RGB is purely
        // additive for loop-closure recall and texturing.
        if (rgb_active) {
            double rgb_ts = 0.0;
            cv::Mat rgb = handlers.latest_rgb_within(frame.timestamp, rgb_max_dt, &rgb_ts);
            if (!rgb.empty()) {
                // Empty depth tells rtabmap this is monocular; setRGBDImage
                // leaves the laser scan in place (it only touches imageRaw,
                // depthOrRightRaw, and cameraModels).
                data.setRGBDImage(rgb, cv::Mat(), camera_model);
                if (debug) {
                    fprintf(stderr,
                            "[rtab DEBUG] attached rgb to frame #%d — "
                            "rgb_ts=%.3f scan_ts=%.3f dt=%.3f size=%dx%d\n",
                            frame_id, rgb_ts, frame.timestamp,
                            std::abs(rgb_ts - frame.timestamp),
                            rgb.cols, rgb.rows);
                }
            } else if (debug) {
                fprintf(stderr,
                        "[rtab DEBUG] no rgb attached to frame #%d — "
                        "no fresh frame within %.3fs of scan_ts=%.3f\n",
                        frame_id, rgb_max_dt, frame.timestamp);
            }
        }

        // rtab.process internally calls LocalGridMaker (via Memory, when
        // RGBD/CreateOccupancyGrid=true), so the scan's local grid ends up
        // attached to the new Signature's SensorData. Pulled out below in
        // sync_signature_grids. No duplicate external createLocalMap call.
        bool processed = rtab.process(data, frame.odom_pose);
        sync_signature_grids(rtab, octomap_grid_cache, max_synced_id);

        // Loop-closure detection. rtab.getLoopClosureId() returns the
        // target signature id when this iteration triggered closure (0
        // otherwise). Pair (current, target) → publish edge marked
        // w=0.4 (the value the KITTI scorer matches on) plus a
        // loop_closure event so external listeners can react. PoseStamped
        // header timestamps mirror each signature's scan stamp so the
        // scorer can map back to frame ids via its send-time cache.
        if (processed) {
            const int loop_id = rtab.getLoopClosureId();
            if (loop_id > 0) {
                const rtabmap::Memory* mem = rtab.getMemory();
                const int curr_id = rtab.getLastLocationId();
                const auto& opt = rtab.getLocalOptimizedPoses();
                const auto curr_it = opt.find(curr_id);
                const auto loop_it = opt.find(loop_id);
                const rtabmap::Signature* curr_sig =
                    mem ? mem->getSignature(curr_id) : nullptr;
                const rtabmap::Signature* loop_sig =
                    mem ? mem->getSignature(loop_id) : nullptr;
                if (curr_it != opt.end() && loop_it != opt.end()
                    && curr_sig && loop_sig) {
                    const double curr_ts = curr_sig->getStamp();
                    const double loop_ts = loop_sig->getStamp();
                    auto edge = build_loop_edge(
                        loop_it->second, loop_ts,
                        curr_it->second, curr_ts,
                        /*traversability=*/0.4,
                        world_frame);
                    lcm.publish(pose_graph_edges_topic, &edge);
                    // Empty-poses Path is a sufficient event signal for
                    // the scoring module's `_on_loop_closure` counter.
                    nav_msgs::Path lc;
                    lc.header = dimos::make_header(world_frame, frame.timestamp);
                    lc.poses_length = 0;
                    lcm.publish(loop_closure_topic, &lc);
                    if (debug) {
                        fprintf(stderr,
                                "[rtab DEBUG] LOOP CLOSURE detected — curr_id=%d (ts=%.3f) ↔ loop_id=%d (ts=%.3f) score=%.3f\n",
                                curr_id, curr_ts, loop_id, loop_ts,
                                rtab.getLoopClosureValue());
                    }
                }
            }
        }
        // Canary for the stationary-robot keyframe-accumulation issue.
        // rtabmap doesn't dedup redundant keyframes in lidar-only mode,
        // so an idle robot with motion gate disabled accumulates ~2
        // signatures/sec in working memory. 5000 ≈ 40 minutes of idle —
        // long enough to be worth flagging but well short of OOM. One
        // warning per crossing.
        if (rtab.getMemory() && rtab.getMemory()->getWorkingMem().size() > 5000) {
            static bool warned_wm_size = false;
            if (!warned_wm_size) {
                fprintf(stderr,
                        "[rtab WARN] working memory exceeded 5000 signatures (%zu) — "
                        "long stationary session may exhaust memory. Consider "
                        "setting rgbd_linear_update > 0 if stationary clearing isn't required.\n",
                        rtab.getMemory()->getWorkingMem().size());
                warned_wm_size = true;
            }
        }
        if (debug) {
            const auto& stats = rtab.getStatistics();
            fprintf(stderr,
                    "[rtab DEBUG] frame #%d %s ts=%.3f odom_pos=(%.2f,%.2f,%.2f) "
                    "opt_poses=%zu cache=%zu | "
                    "loop_id=%d loop_score=%.3f highest_hyp_id=%d "
                    "wm_size=%zu refImageId=%d loopId=%d proxId=%d\n",
                    frame_id, processed ? "processed" : "rejected",
                    frame.timestamp,
                    frame.odom_pose.x(), frame.odom_pose.y(), frame.odom_pose.z(),
                    rtab.getLocalOptimizedPoses().size(), octomap_grid_cache.size(),
                    rtab.getLoopClosureId(), rtab.getLoopClosureValue(),
                    rtab.getHighestHypothesisId(),
                    rtab.getMemory() ? rtab.getMemory()->getWorkingMem().size() : 0,
                    stats.refImageId(),
                    static_cast<int>(stats.loopClosureId()),
                    static_cast<int>(stats.proximityDetectionId()));
        }

        // Publish corrected odometry and map->odom correction every
        // processed frame. (Unprocessed frames already published via the
        // continue branch above.)
        rtabmap::Transform correction = rtab.getMapCorrection();
        rtabmap::Transform corrected_pose = correction * frame.odom_pose;

        auto corrected_msg = odom_to_lcm(
            corrected_pose, frame.timestamp, world_frame, body_frame);
        lcm.publish(corrected_topic, &corrected_msg);

        auto tf_msg = odom_to_lcm(
            correction, frame.timestamp, world_frame, local_frame);
        lcm.publish(tf_topic, &tf_msg);

        // Periodically integrate the latest optimized poses into the
        // global OctoMap. Passing `rtab.getLocalOptimizedPoses()` means:
        //   - New keyframes from rtab.process above get added.
        //   - When a loop closure shifts existing keyframe poses,
        //     OctoMap::fullUpdateNeeded triggers a clear-and-rebuild so
        //     all cells land at corrected world coordinates.
        // No frozen poses, no parallel id space.
        //
        // Reset last_*_publish if the frame timestamp went backward —
        // can happen on FastLIO2 restart, NTP correction, or a stream
        // that starts on wall-clock and then switches to relative.
        if (frame.timestamp < last_octomap_publish) last_octomap_publish = 0.0;
        if (frame.timestamp < last_global_map_publish) last_global_map_publish = 0.0;
        bool octomap_due =
            frame.timestamp - last_octomap_publish >= octomap_publish_period;
        bool global_map_due =
            frame.timestamp - last_global_map_publish >= global_map_publish_period;
        const auto& opt_poses = rtab.getLocalOptimizedPoses();
        if ((octomap_due || global_map_due) && !opt_poses.empty()) {
            global_octomap.update(opt_poses);
        }

        // Publish OctoMap-derived outputs (throttled).
        if (octomap_due && !opt_poses.empty()) {
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
                        octo_points.size(), proj_points.size(), opt_poses.size());
            }
        }

        // Publish accumulated global cloud (throttled).
        //
        // Same OctoMap as the obstacle topic, but with ground cells
        // included for a dense visualization. Sharing the OctoMap means
        // the ray-traced clearing applies to the global map too — when
        // a chair rolls away, its old hits get probabilistically cleared
        // by subsequent scans' rays, so no permanent ghost points.
        if (global_map_due && !opt_poses.empty()) {
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
                        opt_poses.size(), global_map_topic.c_str());
            }
        }
    }

    fprintf(stderr, "RtabMap native module shutting down\n");
    return 0;
}
