// FAST-LIO2 + Livox Mid-360 native module on the dimos C++ SDK. A pure source:
// it binds Livox SDK2 directly into FAST-LIO-NON-ROS. SDK callbacks feed
// CustomMsg/Imu to FastLio, which performs EKF-LOAM SLAM. Publishes
// sensor-frame point clouds on `lidar` and odometry with covariance on
// `odometry` (consumers register the cloud via the odometry pose). No inputs,
// so it overrides handle() with its own processing loop.

#include <atomic>
#include <boost/make_shared.hpp>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "livox_sdk_config.hpp"
#include "point_cloud_utils.hpp"

#include "dimos/native.hpp"

#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "std_msgs/Header.hpp"

// FAST-LIO (header-only core, compiled sources linked via CMake)
#include "fast_lio.hpp"
#include "fast_lio_debug.hpp"

using dimos::native::Builder;
using dimos::native::Config;
using dimos::native::Module;
using dimos::native::Output;
namespace logging = dimos::native::log;

using livox_common::GRAVITY_MS2;
using livox_common::DATA_TYPE_CARTESIAN_HIGH;
using livox_common::DATA_TYPE_CARTESIAN_LOW;

struct FastLio2Config {
    std::string host_ip;
    std::string lidar_ip;
    double frequency;
    std::string frame_id;
    std::string sensor_frame_id;
    double msr_freq;
    double main_freq;
    double pointcloud_freq;
    double odom_freq;
    bool debug;
    bool time_sync_en;
    double time_offset_lidar_to_imu;
    std::string lidar_type;
    int scan_line;
    int scan_rate;
    std::string timestamp_unit;
    double blind;
    double acc_cov;
    double gyr_cov;
    double b_acc_cov;
    double b_gyr_cov;
    double filter_size_surf;
    double filter_size_map;
    int fov_degree;
    double det_range;
    bool extrinsic_est_en;
    std::vector<double> extrinsic_t;
    std::vector<double> extrinsic_r;
    bool scan_publish_en;
    bool dense_publish_en;
    int cmd_data_port;
    int push_msg_port;
    int point_data_port;
    int imu_data_port;
    int log_data_port;
    int host_cmd_data_port;
    int host_push_msg_port;
    int host_point_data_port;
    int host_imu_data_port;
    int host_log_data_port;
};

namespace {

using dimos::make_header;
using dimos::make_xyzi_cloud;

uint64_t packet_timestamp_ns(const LivoxLidarEthernetPacket* pkt) {
    uint64_t ns = 0;
    std::memcpy(&ns, pkt->timestamp, sizeof(uint64_t));
    return ns;
}

}  // namespace

class FastLio2 : public Module {
public:
    void build(Builder& builder, Config& config) override {
        cfg_ = config.parse<FastLio2Config>();
        lidar_ = builder.output<sensor_msgs::PointCloud2>("lidar");
        odometry_ = builder.output<nav_msgs::Odometry>("odometry");

        frame_interval_ =
            std::chrono::microseconds(static_cast<int64_t>(1e6 / cfg_.frequency));
        pc_interval_ =
            std::chrono::microseconds(static_cast<int64_t>(1e6 / cfg_.pointcloud_freq));
        odom_interval_ =
            std::chrono::microseconds(static_cast<int64_t>(1e6 / cfg_.odom_freq));
        process_period_ =
            std::chrono::microseconds(static_cast<int64_t>(1e6 / cfg_.main_freq));
    }

    void setup() override {
        // Propagates to the FAST-LIO C++ core. false -> only real errors print.
        fastlio_debug = cfg_.debug;

        FastLioParams params;
        params.acc_cov = cfg_.acc_cov;
        params.gyr_cov = cfg_.gyr_cov;
        params.b_acc_cov = cfg_.b_acc_cov;
        params.b_gyr_cov = cfg_.b_gyr_cov;
        params.filter_size_surf = cfg_.filter_size_surf;
        params.filter_size_map = cfg_.filter_size_map;
        params.det_range = cfg_.det_range;
        params.blind = cfg_.blind;
        params.time_offset_lidar_to_imu = cfg_.time_offset_lidar_to_imu;
        params.fov_degree = cfg_.fov_degree;
        params.scan_line = cfg_.scan_line;
        params.scan_rate = cfg_.scan_rate;
        params.time_sync_en = cfg_.time_sync_en;
        params.extrinsic_est_en = cfg_.extrinsic_est_en;
        params.lidar_type =
            cfg_.lidar_type == "velodyne" ? 2 : cfg_.lidar_type == "ouster" ? 3 : 1;
        params.timestamp_unit = cfg_.timestamp_unit == "second"        ? 0
                                : cfg_.timestamp_unit == "millisecond" ? 1
                                : cfg_.timestamp_unit == "nanosecond"  ? 3
                                                                       : 2;
        params.extrinsic_T = cfg_.extrinsic_t;
        params.extrinsic_R = cfg_.extrinsic_r;

        fast_lio_ = std::make_unique<FastLio>(params, cfg_.msr_freq, cfg_.main_freq);

        livox_common::SdkPorts ports;
        ports.cmd_data = cfg_.cmd_data_port;
        ports.push_msg = cfg_.push_msg_port;
        ports.point_data = cfg_.point_data_port;
        ports.imu_data = cfg_.imu_data_port;
        ports.log_data = cfg_.log_data_port;
        ports.host_cmd_data = cfg_.host_cmd_data_port;
        ports.host_push_msg = cfg_.host_push_msg_port;
        ports.host_point_data = cfg_.host_point_data_port;
        ports.host_imu_data = cfg_.host_imu_data_port;
        ports.host_log_data = cfg_.host_log_data_port;

        if (!livox_common::init_livox_sdk(cfg_.host_ip, cfg_.lidar_ip, ports, cfg_.debug)) {
            throw std::runtime_error("init_livox_sdk failed");
        }

        SetLivoxLidarPointCloudCallBack(&FastLio2::point_cloud_cb, this);
        SetLivoxLidarImuDataCallback(&FastLio2::imu_cb, this);
        SetLivoxLidarInfoChangeCallback(&FastLio2::info_cb, this);

        if (!LivoxLidarSdkStart()) {
            LivoxLidarSdkUninit();
            throw std::runtime_error("LivoxLidarSdkStart failed");
        }
        logging::info("fastlio2 SDK started, waiting for device",
                      {logging::Field("lidar_ip", cfg_.lidar_ip),
                       logging::Field("host_ip", cfg_.host_ip)});
    }

    // Own processing loop at ~main_freq: drain accumulated points into a
    // CustomMsg at frame rate, run a FAST-LIO step, publish rate-limited.
    void handle() override {
        auto last_emit = std::chrono::steady_clock::now();
        auto last_pc_publish = last_emit;
        auto last_odom_publish = last_emit;
        while (!shutdown_requested()) {
            auto now = std::chrono::steady_clock::now();
            step(now, last_emit, last_pc_publish, last_odom_publish);
            auto elapsed = std::chrono::steady_clock::now() - now;
            if (elapsed < process_period_) {
                std::this_thread::sleep_for(process_period_ - elapsed);
            }
        }
    }

    // Uninit the SDK (stops + joins its callback threads) before the module is
    // destroyed, so a late callback can't race destruction.
    void teardown() override {
        LivoxLidarSdkUninit();
    }

private:
    static void point_cloud_cb(const uint32_t, const uint8_t,
                               LivoxLidarEthernetPacket* data, void* ctx) {
        static_cast<FastLio2*>(ctx)->on_point_cloud(data);
    }

    static void imu_cb(const uint32_t, const uint8_t, LivoxLidarEthernetPacket* data,
                       void* ctx) {
        static_cast<FastLio2*>(ctx)->on_imu(data);
    }

    static void info_cb(const uint32_t handle, const LivoxLidarInfo* info, void* ctx) {
        static_cast<FastLio2*>(ctx)->on_info(handle, info);
    }

    // Runs on a Livox SDK callback thread: accumulate raw points into the
    // current frame (Livox SDK raw -> CustomPoint).
    void on_point_cloud(LivoxLidarEthernetPacket* data) {
        if (shutdown_requested() || data == nullptr) return;

        uint64_t ts_ns = packet_timestamp_ns(data);
        uint16_t dot_num = data->dot_num;

        std::lock_guard<std::mutex> lock(pc_mutex_);
        if (!frame_has_ts_) {
            frame_start_ns_ = ts_ns;
            frame_has_ts_ = true;
        }

        if (data->data_type == DATA_TYPE_CARTESIAN_HIGH) {
            auto* pts = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(data->data);
            for (uint16_t i = 0; i < dot_num; ++i) {
                custom_messages::CustomPoint cp;
                // High-precision coordinates are in mm.
                cp.x = static_cast<double>(pts[i].x) / 1000.0;
                cp.y = static_cast<double>(pts[i].y) / 1000.0;
                cp.z = static_cast<double>(pts[i].z) / 1000.0;
                cp.reflectivity = pts[i].reflectivity;
                cp.tag = pts[i].tag;
                cp.line = 0;  // Mid-360: non-repetitive, single "line"
                cp.offset_time = static_cast<uli>(ts_ns - frame_start_ns_);
                accumulated_points_.push_back(cp);
            }
        } else if (data->data_type == DATA_TYPE_CARTESIAN_LOW) {
            auto* pts = reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(data->data);
            for (uint16_t i = 0; i < dot_num; ++i) {
                custom_messages::CustomPoint cp;
                // Low-precision coordinates are in cm.
                cp.x = static_cast<double>(pts[i].x) / 100.0;
                cp.y = static_cast<double>(pts[i].y) / 100.0;
                cp.z = static_cast<double>(pts[i].z) / 100.0;
                cp.reflectivity = pts[i].reflectivity;
                cp.tag = pts[i].tag;
                cp.line = 0;
                cp.offset_time = static_cast<uli>(ts_ns - frame_start_ns_);
                accumulated_points_.push_back(cp);
            }
        }
    }

    // Runs on a Livox SDK callback thread: feed IMU samples straight into
    // FAST-LIO (its queues are internally synchronized).
    void on_imu(LivoxLidarEthernetPacket* data) {
        if (shutdown_requested() || data == nullptr || !fast_lio_) return;

        double ts = static_cast<double>(packet_timestamp_ns(data)) / 1e9;
        auto* imu_pts = reinterpret_cast<const LivoxLidarImuRawPoint*>(data->data);
        uint16_t dot_num = data->dot_num;

        for (uint16_t i = 0; i < dot_num; ++i) {
            auto imu_msg = boost::make_shared<custom_messages::Imu>();
            imu_msg->header.stamp = custom_messages::Time().fromSec(ts);
            imu_msg->header.seq = 0;
            imu_msg->header.frame_id = "livox_frame";

            imu_msg->orientation.x = 0.0;
            imu_msg->orientation.y = 0.0;
            imu_msg->orientation.z = 0.0;
            imu_msg->orientation.w = 1.0;
            for (int j = 0; j < 9; ++j) { imu_msg->orientation_covariance[j] = 0.0; }

            imu_msg->angular_velocity.x = static_cast<double>(imu_pts[i].gyro_x);
            imu_msg->angular_velocity.y = static_cast<double>(imu_pts[i].gyro_y);
            imu_msg->angular_velocity.z = static_cast<double>(imu_pts[i].gyro_z);
            for (int j = 0; j < 9; ++j) { imu_msg->angular_velocity_covariance[j] = 0.0; }

            imu_msg->linear_acceleration.x = static_cast<double>(imu_pts[i].acc_x) * GRAVITY_MS2;
            imu_msg->linear_acceleration.y = static_cast<double>(imu_pts[i].acc_y) * GRAVITY_MS2;
            imu_msg->linear_acceleration.z = static_cast<double>(imu_pts[i].acc_z) * GRAVITY_MS2;
            for (int j = 0; j < 9; ++j) { imu_msg->linear_acceleration_covariance[j] = 0.0; }

            fast_lio_->feed_imu(imu_msg);
        }
    }

    void on_info(uint32_t handle, const LivoxLidarInfo* info) {
        if (info == nullptr) return;

        char sn[17] = {};
        std::memcpy(sn, info->sn, 16);
        char ip[17] = {};
        std::memcpy(ip, info->lidar_ip, 16);
        logging::info("fastlio2 device connected",
                      {logging::Field("sn", std::string(sn)),
                       logging::Field("ip", std::string(ip))});

        SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
        EnableLivoxLidarImuData(handle, nullptr, nullptr);
    }

    // One iteration of the main loop, rate-limited by the bookmark arguments.
    void step(std::chrono::steady_clock::time_point now,
              std::chrono::steady_clock::time_point& last_emit,
              std::chrono::steady_clock::time_point& last_pc_publish,
              std::chrono::steady_clock::time_point& last_odom_publish) {
        // At frame rate, drain accumulated raw points into a CustomMsg and feed
        // FAST-LIO. Hold pc_mutex_ across the rate-limit check + swap so a
        // callback can't slip a packet in between the decision and the swap.
        std::vector<custom_messages::CustomPoint> points;
        uint64_t frame_start = 0;
        {
            std::lock_guard<std::mutex> lock(pc_mutex_);
            if (now - last_emit >= frame_interval_) {
                if (!accumulated_points_.empty()) {
                    points.swap(accumulated_points_);
                    frame_start = frame_start_ns_;
                    frame_has_ts_ = false;
                }
                last_emit = now;
            }
        }
        if (!points.empty()) {
            auto lidar_msg = boost::make_shared<custom_messages::CustomMsg>();
            lidar_msg->header.seq = 0;
            lidar_msg->header.stamp =
                custom_messages::Time().fromSec(static_cast<double>(frame_start) / 1e9);
            lidar_msg->header.frame_id = "livox_frame";
            lidar_msg->timebase = frame_start;
            lidar_msg->lidar_id = 0;
            for (int i = 0; i < 3; i++) { lidar_msg->rsvd[i] = 0; }
            lidar_msg->point_num = static_cast<uli>(points.size());
            lidar_msg->points = std::move(points);
            fast_lio_->feed_lidar(lidar_msg);
        }

        // Run one FAST-LIO IESKF step. Cheap when the IMU/lidar queues
        // are empty. The heavy work happens after a feed_lidar above.
        fast_lio_->process();

        // Check for new SLAM results and publish (rate-limited).
        auto pose = fast_lio_->get_pose();
        if (!pose.empty() && (pose[0] != 0.0 || pose[1] != 0.0 || pose[2] != 0.0)) {
            double ts = std::chrono::duration<double>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
            if (cfg_.scan_publish_en && now - last_pc_publish >= pc_interval_) {
                // Sensor-frame cloud. Register downstream via the odom pose.
                // dense_publish_en false -> FAST-LIO's IESKF-downsampled scan.
                auto cloud = cfg_.dense_publish_en ? fast_lio_->get_body_cloud()
                                                   : fast_lio_->get_body_cloud_down();
                if (cloud && !cloud->empty()) {
                    publish_pointcloud(cloud, ts);
                }
                last_pc_publish = now;
            }

            // Pose + covariance, rate-limited to odom_freq.
            if (now - last_odom_publish >= odom_interval_) {
                publish_odometry(fast_lio_->get_odometry(), ts);
                last_odom_publish = now;
            }
        }
    }

    // Publish a sensor-frame point cloud stamped with sensor_frame_id.
    void publish_pointcloud(const PointCloudXYZI::Ptr& cloud, double ts) {
        int num_points = static_cast<int>(cloud->size());

        sensor_msgs::PointCloud2 pc = make_xyzi_cloud(cfg_.sensor_frame_id, ts, num_points);

        for (int i = 0; i < num_points; ++i) {
            float* dst = reinterpret_cast<float*>(pc.data.data() + i * 16);
            dst[0] = cloud->points[i].x;
            dst[1] = cloud->points[i].y;
            dst[2] = cloud->points[i].z;
            dst[3] = cloud->points[i].intensity;
        }

        lidar_.publish(pc);
    }

    // Publish odometry as frame_id (fixed) -> sensor_frame_id (moving sensor).
    void publish_odometry(const custom_messages::Odometry& odom, double ts) {
        nav_msgs::Odometry msg;
        msg.header = make_header(cfg_.frame_id, ts);
        msg.child_frame_id = cfg_.sensor_frame_id;

        msg.pose.pose.position.x = odom.pose.pose.position.x;
        msg.pose.pose.position.y = odom.pose.pose.position.y;
        msg.pose.pose.position.z = odom.pose.pose.position.z;
        msg.pose.pose.orientation.x = odom.pose.pose.orientation.x;
        msg.pose.pose.orientation.y = odom.pose.pose.orientation.y;
        msg.pose.pose.orientation.z = odom.pose.pose.orientation.z;
        msg.pose.pose.orientation.w = odom.pose.pose.orientation.w;

        for (int i = 0; i < 36; ++i) {
            msg.pose.covariance[i] = odom.pose.covariance[i];
        }

        // Twist (zero, FAST-LIO doesn't output velocity directly)
        msg.twist.twist.linear.x = 0;
        msg.twist.twist.linear.y = 0;
        msg.twist.twist.linear.z = 0;
        msg.twist.twist.angular.x = 0;
        msg.twist.twist.angular.y = 0;
        msg.twist.twist.angular.z = 0;
        std::memset(msg.twist.covariance, 0, sizeof(msg.twist.covariance));

        odometry_.publish(msg);
    }

    FastLio2Config cfg_;
    Output<sensor_msgs::PointCloud2> lidar_;
    Output<nav_msgs::Odometry> odometry_;
    std::unique_ptr<FastLio> fast_lio_;

    std::chrono::microseconds frame_interval_{100000};
    std::chrono::microseconds pc_interval_{100000};
    std::chrono::microseconds odom_interval_{20000};
    std::chrono::microseconds process_period_{200};

    // Frame accumulator (Livox SDK raw -> CustomMsg)
    std::mutex pc_mutex_;
    std::vector<custom_messages::CustomPoint> accumulated_points_;
    uint64_t frame_start_ns_ = 0;
    bool frame_has_ts_ = false;
};

int main() {
    dimos::native::run_with_transport<FastLio2>();
    return 0;
}
