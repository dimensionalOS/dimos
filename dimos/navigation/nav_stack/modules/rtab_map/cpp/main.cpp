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

std::mutex g_buffer_mutex;
std::queue<ScanFrame> g_buffer;

std::mutex g_odom_mutex;
rtabmap::Transform g_latest_odom;
bool g_has_odom = false;
double g_latest_odom_ts = 0.0;

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
        std::lock_guard<std::mutex> lock(g_odom_mutex);
        g_latest_odom = odom_from_lcm(*msg);
        g_latest_odom_ts = ts;
        g_has_odom = true;
    }

    void on_registered_scan(
        const lcm::ReceiveBuffer*,
        const std::string&,
        const sensor_msgs::PointCloud2* msg) {
        rtabmap::Transform odom_pose;
        double odom_ts = 0.0;
        {
            std::lock_guard<std::mutex> lock(g_odom_mutex);
            if (!g_has_odom) return;
            odom_pose = g_latest_odom;
            odom_ts = g_latest_odom_ts;
        }

        ScanFrame frame;
        frame.cloud_body = CloudType::Ptr(new CloudType);
        smartnav::to_pcl(*msg, *frame.cloud_body);
        frame.odom_pose = odom_pose;
        frame.timestamp = odom_ts;

        if (unregister_input_) {
            // Input is world-frame; convert to body for rtabmap (it expects
            // body-frame scans with sensor at origin).
            CloudType::Ptr body(new CloudType);
            Eigen::Affine3f w_to_b = Eigen::Affine3f::Identity();
            w_to_b.translation() << frame.odom_pose.x(),
                                    frame.odom_pose.y(),
                                    frame.odom_pose.z();
            Eigen::Matrix3f r;
            r << frame.odom_pose.r11(), frame.odom_pose.r12(), frame.odom_pose.r13(),
                 frame.odom_pose.r21(), frame.odom_pose.r22(), frame.odom_pose.r23(),
                 frame.odom_pose.r31(), frame.odom_pose.r32(), frame.odom_pose.r33();
            w_to_b.linear() = r;
            Eigen::Affine3f inv = w_to_b.inverse();
            pcl::transformPointCloud(*frame.cloud_body, *body, inv);
            frame.cloud_body = body;
        }

        std::lock_guard<std::mutex> lock(g_buffer_mutex);
        g_buffer.push(frame);
    }

    bool unregister_input_ = true;
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
    // Lidar-only mode.
    params["RGBD/Enabled"] = "true";
    params["Reg/Strategy"] = "1";  // ICP
    params["Mem/IncrementalMemory"] = "true";
    // Keep odometry covariance contributions modest so an externally-supplied
    // FastLIO2 odom dominates.
    params["Reg/Force3DoF"] = "false";
    params["RGBD/CreateOccupancyGrid"] = "true";

    rtabmap::Rtabmap rtab;
    rtab.init(params);

    rtabmap::LocalGridCache grid_cache;
    rtabmap::OctoMap octomap(&grid_cache, params);

    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "RtabMap: LCM init failed\n");
        return 1;
    }

    Handlers handlers;
    handlers.unregister_input_ = mod.arg_bool("unregister_input", true);
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

    const double octomap_publish_period = std::stod(mod.arg("octomap_publish_period", "0.5"));
    const double global_map_publish_period = std::stod(mod.arg("global_map_publish_period", "1.0"));

    double last_octomap_publish = 0.0;
    double last_global_map_publish = 0.0;
    int frame_id = 0;
    const int timer_period_ms = 30;

    while (g_running.load()) {
        // Drain LCM.
        while (lcm.handleTimeout(0) > 0) {}

        ScanFrame frame;
        bool have_frame = false;
        {
            std::lock_guard<std::mutex> lock(g_buffer_mutex);
            if (!g_buffer.empty()) {
                frame = std::move(g_buffer.front());
                g_buffer.pop();
                have_frame = true;
            }
        }

        if (!have_frame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(timer_period_ms));
            continue;
        }

        // Build SensorData with the lidar scan.
        cv::Mat scan_mat = scan_to_cv_mat(*frame.cloud_body);
        rtabmap::LaserScan laser_scan =
            rtabmap::LaserScan::backwardCompatibility(scan_mat);
        rtabmap::SensorData data(laser_scan, cv::Mat(), cv::Mat(), rtabmap::CameraModel());
        data.setId(++frame_id);
        data.setStamp(frame.timestamp);

        bool processed = rtab.process(data, frame.odom_pose);
        if (!processed) {
            // Frame rejected by rtabmap (e.g. too close to last keyframe).
            std::this_thread::sleep_for(std::chrono::milliseconds(timer_period_ms));
            continue;
        }

        // Push the just-processed keyframe's local grid into the OctoMap cache.
        const rtabmap::Signature* sig = rtab.getMemory()
            ? rtab.getMemory()->getLastWorkingSignature()
            : nullptr;
        if (sig) {
            const rtabmap::SensorData& sd = sig->sensorData();
            if (!sd.gridObstacleCellsRaw().empty()
                || !sd.gridGroundCellsRaw().empty()
                || !sd.gridEmptyCellsRaw().empty()) {
                grid_cache.add(
                    sig->id(),
                    sd.gridGroundCellsRaw(),
                    sd.gridObstacleCellsRaw(),
                    sd.gridEmptyCellsRaw(),
                    sd.gridCellSize(),
                    sd.gridViewPoint());
            }
        }

        // Update OctoMap against current optimized poses.
        const std::map<int, rtabmap::Transform>& opt_poses =
            rtab.getLocalOptimizedPoses();
        if (!opt_poses.empty()) {
            octomap.update(opt_poses);
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
        }

        // Publish accumulated global cloud (throttled). Built from optimized
        // signatures' body-frame clouds transformed into the map frame.
        if (frame.timestamp - last_global_map_publish >= global_map_publish_period) {
            last_global_map_publish = frame.timestamp;

            CloudType::Ptr global_cloud(new CloudType);
            if (rtab.getMemory()) {
                for (const auto& kv : opt_poses) {
                    const rtabmap::Signature* s =
                        rtab.getMemory()->getSignature(kv.first);
                    if (!s) continue;
                    const rtabmap::LaserScan& ls = s->sensorData().laserScanRaw();
                    if (ls.empty()) continue;

                    Eigen::Affine3f t;
                    t.translation() << kv.second.x(), kv.second.y(), kv.second.z();
                    Eigen::Matrix3f r;
                    r << kv.second.r11(), kv.second.r12(), kv.second.r13(),
                         kv.second.r21(), kv.second.r22(), kv.second.r23(),
                         kv.second.r31(), kv.second.r32(), kv.second.r33();
                    t.linear() = r;

                    // Pull XYZ out of the scan cv::Mat and transform.
                    const cv::Mat& m = ls.data();
                    int channels = m.channels();
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
                    pcl::transformPointCloud(*body, *world, t);
                    *global_cloud += *world;
                }
            }

            // Voxel downsample for size sanity.
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
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(timer_period_ms));
    }

    fprintf(stderr, "RtabMap native module shutting down\n");
    return 0;
}
