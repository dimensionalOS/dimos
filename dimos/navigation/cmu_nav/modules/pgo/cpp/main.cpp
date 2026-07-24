// PGO native module on the dimos C++ SDK. Subscribes to registered_scan +
// odometry, runs SimplePGO (iSAM2 + PCL ICP), and publishes corrected_odometry,
// global_map, and the map->odom TF correction. Odometry and registered_scan
// are decoded on the transport's single receive thread in arrival order, so
// each scan is stamped with the odometry that arrived just before it, then the
// PGO work runs on the dispatch thread.

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/console/print.h>
#include <pcl/filters/voxel_grid.h>

#include "dimos/native.hpp"

#include "commons.h"
#include "point_cloud_utils.hpp"
#include "simple_pgo.h"

#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/PointCloud2.hpp"

using dimos::native::Builder;
using dimos::native::Module;
using dimos::native::Output;

struct PGOConfig {
    std::string world_frame;
    std::string local_frame;
    double key_pose_delta_deg;
    double key_pose_delta_trans;
    double loop_search_radius;
    double loop_time_thresh;
    double loop_score_thresh;
    int loop_submap_half_range;
    double submap_resolution;
    double min_loop_detect_duration;
    bool unregister_input;
    double global_map_voxel_size;
    double global_map_publish_rate;
    bool debug;
};

namespace {

nav_msgs::Odometry build_odometry(const M3D& r, const V3D& t, double ts,
                                  const std::string& frame_id,
                                  const std::string& child_frame_id) {
    static std::atomic<int32_t> seq{0};
    nav_msgs::Odometry odom;
    odom.header.seq = seq.fetch_add(1, std::memory_order_relaxed);
    odom.header.stamp.sec = static_cast<int32_t>(ts);
    odom.header.stamp.nsec = static_cast<int32_t>((ts - static_cast<int32_t>(ts)) * 1e9);
    odom.header.frame_id = frame_id;
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

}  // namespace

class PGO : public Module {
public:
    void build(Builder& builder, dimos::native::Config& config) override {
        cfg_ = config.parse<PGOConfig>();

        Config pgo_cfg;
        pgo_cfg.key_pose_delta_deg = cfg_.key_pose_delta_deg;
        pgo_cfg.key_pose_delta_trans = cfg_.key_pose_delta_trans;
        pgo_cfg.loop_search_radius = cfg_.loop_search_radius;
        pgo_cfg.loop_time_tresh = cfg_.loop_time_thresh;
        pgo_cfg.loop_score_tresh = cfg_.loop_score_thresh;
        pgo_cfg.loop_submap_half_range = cfg_.loop_submap_half_range;
        pgo_cfg.submap_resolution = cfg_.submap_resolution;
        pgo_cfg.min_loop_detect_duration = cfg_.min_loop_detect_duration;
        pgo_ = std::make_unique<SimplePGO>(pgo_cfg);

        global_map_interval_ =
            cfg_.global_map_publish_rate > 0 ? 1.0 / cfg_.global_map_publish_rate : 2.0;

        pcl::console::setVerbosityLevel(
            cfg_.debug ? pcl::console::L_INFO : pcl::console::L_ERROR);

        corrected_odometry_ = builder.output<nav_msgs::Odometry>("corrected_odometry");
        global_map_ = builder.output<sensor_msgs::PointCloud2>("global_map");
        pgo_tf_ = builder.output<nav_msgs::Odometry>("pgo_tf");

        builder.input<nav_msgs::Odometry>(
            "odometry",
            [this](const uint8_t* data, std::size_t len) {
                nav_msgs::Odometry msg = dimos::native::lcm_decode<nav_msgs::Odometry>(data, len);
                capture_odometry(msg);
                return msg;
            },
            [](nav_msgs::Odometry) {});
        builder.input<CloudWithPose>(
            "registered_scan", &PGO::on_scan, this,
            [this](const uint8_t* data, std::size_t len) {
                return decode_scan(
                    dimos::native::lcm_decode<sensor_msgs::PointCloud2>(data, len));
            });
    }

private:
    // Receive thread: record the latest odometry as each message is decoded.
    void capture_odometry(const nav_msgs::Odometry& msg) {
        latest_odom_.r = Eigen::Quaterniond(msg.pose.pose.orientation.w,
                                            msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z)
                             .toRotationMatrix();
        latest_odom_.t = V3D(msg.pose.pose.position.x, msg.pose.pose.position.y,
                             msg.pose.pose.position.z);
        latest_odom_.time = msg.header.stamp.sec + msg.header.stamp.nsec / 1e9;
        latest_odom_.has = true;
    }

    // Receive thread: snapshot the odometry that arrived just before this scan.
    // Same thread as capture_odometry, so latest_odom_ needs no lock. A null
    // cloud (no odometry seen yet) tells on_scan to skip.
    CloudWithPose decode_scan(const sensor_msgs::PointCloud2& msg) {
        CloudWithPose cp;
        if (!latest_odom_.has) {
            return cp;
        }
        double ts = latest_odom_.time;
        cp.pose.r = latest_odom_.r;
        cp.pose.t = latest_odom_.t;
        cp.pose.setTime(static_cast<int32_t>(ts),
                        static_cast<uint32_t>((ts - static_cast<int32_t>(ts)) * 1e9));
        cp.cloud = CloudType::Ptr(new CloudType);
        smartnav::to_pcl(msg, *cp.cloud);
        return cp;
    }

    void on_scan(const CloudWithPose& decoded) {
        if (!decoded.cloud) {
            return;
        }
        double ts = decoded.pose.second;
        if (ts < last_message_time_) {  // reject out-of-order
            return;
        }
        last_message_time_ = ts;

        CloudWithPose cp = decoded;
        if (cfg_.unregister_input && cp.cloud && cp.cloud->size() > 0) {
            CloudType::Ptr body_cloud(new CloudType);
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

        if (!pgo_->addKeyPose(cp)) {
            publish_corrected_and_tf(cp, cur_time);
            return;
        }

        pgo_->searchForLoopPairs();
        pgo_->smoothAndUpdate();
        publish_corrected_and_tf(cp, cur_time);

        if (cur_time - last_global_map_time_ >= global_map_interval_) {
            last_global_map_time_ = cur_time;
            publish_global_map(cur_time);
        }
    }

    void publish_corrected_and_tf(const CloudWithPose& cp, double cur_time) {
        M3D corr_r = pgo_->offsetR() * cp.pose.r;
        V3D corr_t = pgo_->offsetR() * cp.pose.t + pgo_->offsetT();
        corrected_odometry_.publish(
            build_odometry(corr_r, corr_t, cur_time, cfg_.world_frame, "base_link"));
        pgo_tf_.publish(build_odometry(pgo_->offsetR(), pgo_->offsetT(), cur_time,
                                       cfg_.world_frame, cfg_.local_frame));
    }

    void publish_global_map(double now) {
        if (pgo_->keyPoses().empty()) {
            return;
        }
        CloudType::Ptr global_cloud(new CloudType);
        for (size_t i = 0; i < pgo_->keyPoses().size(); i++) {
            CloudType::Ptr world_cloud(new CloudType);
            pcl::transformPointCloud(*pgo_->keyPoses()[i].body_cloud, *world_cloud,
                                     pgo_->keyPoses()[i].t_global,
                                     Eigen::Quaterniond(pgo_->keyPoses()[i].r_global));
            *global_cloud += *world_cloud;
        }

        CloudType::Ptr filtered(new CloudType);
        pcl::VoxelGrid<PointType> voxel;
        voxel.setInputCloud(global_cloud);
        voxel.setLeafSize(cfg_.global_map_voxel_size, cfg_.global_map_voxel_size,
                          cfg_.global_map_voxel_size);
        voxel.filter(*filtered);

        global_map_.publish(smartnav::from_pcl(*filtered, cfg_.world_frame, now));
    }

    PGOConfig cfg_;
    std::unique_ptr<SimplePGO> pgo_;

    Output<nav_msgs::Odometry> corrected_odometry_;
    Output<sensor_msgs::PointCloud2> global_map_;
    Output<nav_msgs::Odometry> pgo_tf_;

    struct StampedPose {
        M3D r = M3D::Identity();
        V3D t = V3D::Zero();
        double time = 0.0;
        bool has = false;
    };
    // Written by capture_odometry and read by decode_scan, both on the transport
    // receive thread, so this needs no lock.
    StampedPose latest_odom_;

    double last_message_time_ = 0.0;
    double last_global_map_time_ = 0.0;
    double global_map_interval_ = 2.0;
};

int main() {
    dimos::native::run_with_transport<PGO>();
    return 0;
}
