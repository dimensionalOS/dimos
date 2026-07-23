// Point-LIO + Livox Mid-360 native module on the dimos C++ SDK. A pure source:
// it binds Livox SDK2 directly into the Point-LIO core. SDK callbacks feed
// CustomMsg/Imu to the IESKF estimator, which performs LiDAR-inertial SLAM.
// Publishes sensor-frame point clouds on `lidar` and odometry with velocity on
// `odometry`. No inputs, so it overrides handle() with its own processing loop.

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

#include "dimos/native.hpp"

#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "std_msgs/Header.hpp"

// Point-LIO (header-only core, compiled sources linked via CMake)
#include "pointlio.hpp"
#include "pointlio_debug.hpp"

using dimos::native::Builder;
using dimos::native::Config;
using dimos::native::Module;
using dimos::native::Output;
namespace logging = dimos::native::log;

using livox_common::DATA_TYPE_CARTESIAN_HIGH;
using livox_common::DATA_TYPE_CARTESIAN_LOW;

struct PointLioConfig {
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
    bool con_frame;
    int con_frame_num;
    bool cut_frame;
    double cut_frame_time_interval;
    double time_lag_imu_to_lidar;
    std::string lidar_type;
    int scan_line;
    int scan_rate;
    std::string timestamp_unit;
    double blind;
    int point_filter_num;
    bool use_imu_as_input;
    bool prop_at_freq_of_imu;
    bool check_satu;
    int init_map_size;
    bool space_down_sample;
    double satu_acc;
    double satu_gyro;
    double acc_norm;
    double plane_thr;
    double filter_size_surf;
    double filter_size_map;
    double ivox_grid_resolution;
    std::string ivox_nearby_type;
    double cube_side_length;
    double det_range;
    double fov_degree;
    bool imu_en;
    bool start_in_aggressive_motion;
    bool extrinsic_est_en;
    double imu_time_inte;
    double lidar_meas_cov;
    double acc_cov_input;
    double vel_cov;
    double gyr_cov_input;
    double gyr_cov_output;
    double acc_cov_output;
    double b_gyr_cov;
    double b_acc_cov;
    double imu_meas_acc_cov;
    double imu_meas_omg_cov;
    double match_s;
    bool gravity_align;
    std::vector<double> gravity;
    std::vector<double> gravity_init;
    std::vector<double> extrinsic_t;
    std::vector<double> extrinsic_r;
    bool publish_odometry_without_downsample;
    bool odom_only;
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

// One entry per stdin config key. The struct is past nlohmann's 64-field macro
// limit, so DIMOS_NATIVE_CONFIG cannot be used; DIMOS_NATIVE_CONFIG_LIST
// generates the (de)serialization from this list instead, same contract.
#define POINTLIO_CONFIG_FIELDS(FIELD) \
    FIELD(host_ip)                    \
    FIELD(lidar_ip)                   \
    FIELD(frequency)                  \
    FIELD(frame_id)                   \
    FIELD(sensor_frame_id)            \
    FIELD(msr_freq)                   \
    FIELD(main_freq)                  \
    FIELD(pointcloud_freq)            \
    FIELD(odom_freq)                  \
    FIELD(debug)                      \
    FIELD(con_frame)                  \
    FIELD(con_frame_num)              \
    FIELD(cut_frame)                  \
    FIELD(cut_frame_time_interval)    \
    FIELD(time_lag_imu_to_lidar)      \
    FIELD(lidar_type)                 \
    FIELD(scan_line)                  \
    FIELD(scan_rate)                  \
    FIELD(timestamp_unit)             \
    FIELD(blind)                      \
    FIELD(point_filter_num)           \
    FIELD(use_imu_as_input)           \
    FIELD(prop_at_freq_of_imu)        \
    FIELD(check_satu)                 \
    FIELD(init_map_size)              \
    FIELD(space_down_sample)          \
    FIELD(satu_acc)                   \
    FIELD(satu_gyro)                  \
    FIELD(acc_norm)                   \
    FIELD(plane_thr)                  \
    FIELD(filter_size_surf)           \
    FIELD(filter_size_map)            \
    FIELD(ivox_grid_resolution)       \
    FIELD(ivox_nearby_type)           \
    FIELD(cube_side_length)           \
    FIELD(det_range)                  \
    FIELD(fov_degree)                 \
    FIELD(imu_en)                     \
    FIELD(start_in_aggressive_motion) \
    FIELD(extrinsic_est_en)           \
    FIELD(imu_time_inte)              \
    FIELD(lidar_meas_cov)             \
    FIELD(acc_cov_input)              \
    FIELD(vel_cov)                    \
    FIELD(gyr_cov_input)              \
    FIELD(gyr_cov_output)             \
    FIELD(acc_cov_output)             \
    FIELD(b_gyr_cov)                  \
    FIELD(b_acc_cov)                  \
    FIELD(imu_meas_acc_cov)           \
    FIELD(imu_meas_omg_cov)           \
    FIELD(match_s)                    \
    FIELD(gravity_align)              \
    FIELD(gravity)                    \
    FIELD(gravity_init)               \
    FIELD(extrinsic_t)                \
    FIELD(extrinsic_r)                \
    FIELD(publish_odometry_without_downsample) \
    FIELD(odom_only)                  \
    FIELD(cmd_data_port)              \
    FIELD(push_msg_port)              \
    FIELD(point_data_port)            \
    FIELD(imu_data_port)              \
    FIELD(log_data_port)              \
    FIELD(host_cmd_data_port)         \
    FIELD(host_push_msg_port)         \
    FIELD(host_point_data_port)       \
    FIELD(host_imu_data_port)         \
    FIELD(host_log_data_port)
DIMOS_NATIVE_CONFIG_LIST(PointLioConfig, POINTLIO_CONFIG_FIELDS)

namespace {

std_msgs::Header make_header(const std::string& frame_id, double ts) {
    static std::atomic<int32_t> seq{0};
    std_msgs::Header h;
    h.seq = seq.fetch_add(1, std::memory_order_relaxed);
    h.stamp.sec = static_cast<int32_t>(ts);
    h.stamp.nsec = static_cast<int32_t>((ts - static_cast<int32_t>(ts)) * 1e9);
    h.frame_id = frame_id;
    return h;
}

uint64_t packet_timestamp_ns(const LivoxLidarEthernetPacket* pkt) {
    uint64_t ns = 0;
    std::memcpy(&ns, pkt->timestamp, sizeof(uint64_t));
    return ns;
}

}  // namespace

class PointLioModule : public Module {
public:
    void build(Builder& builder, Config& config) override {
        cfg_ = config.parse<PointLioConfig>();
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
        // Propagates to the Point-LIO core; false -> only real errors print.
        pointlio_debug = cfg_.debug;

        PointLioParams params;
        params.con_frame = cfg_.con_frame;
        params.con_frame_num = cfg_.con_frame_num;
        params.cut_frame = cfg_.cut_frame;
        params.cut_frame_time_interval = cfg_.cut_frame_time_interval;
        params.time_lag_imu_to_lidar = cfg_.time_lag_imu_to_lidar;
        params.scan_line = cfg_.scan_line;
        params.scan_rate = cfg_.scan_rate;
        params.blind = cfg_.blind;
        params.point_filter_num = cfg_.point_filter_num;
        params.lidar_type = cfg_.lidar_type == "velodyne"   ? 2
                            : cfg_.lidar_type == "ouster"   ? 3
                            : cfg_.lidar_type == "hesai"    ? 4
                            : cfg_.lidar_type == "unilidar" ? 5
                                                            : 1;
        params.timestamp_unit = cfg_.timestamp_unit == "second"        ? 0
                                : cfg_.timestamp_unit == "millisecond" ? 1
                                : cfg_.timestamp_unit == "microsecond" ? 2
                                                                       : 3;
        params.use_imu_as_input = cfg_.use_imu_as_input;
        params.prop_at_freq_of_imu = cfg_.prop_at_freq_of_imu;
        params.check_satu = cfg_.check_satu;
        params.init_map_size = cfg_.init_map_size;
        params.space_down_sample = cfg_.space_down_sample;
        params.satu_acc = cfg_.satu_acc;
        params.satu_gyro = cfg_.satu_gyro;
        params.acc_norm = cfg_.acc_norm;
        params.plane_thr = cfg_.plane_thr;
        params.filter_size_surf = cfg_.filter_size_surf;
        params.filter_size_map = cfg_.filter_size_map;
        params.ivox_grid_resolution = cfg_.ivox_grid_resolution;
        params.ivox_nearby_type = cfg_.ivox_nearby_type == "center"     ? 0
                                  : cfg_.ivox_nearby_type == "nearby18" ? 18
                                  : cfg_.ivox_nearby_type == "nearby26" ? 26
                                                                        : 6;
        params.cube_side_length = cfg_.cube_side_length;
        params.det_range = cfg_.det_range;
        params.fov_degree = cfg_.fov_degree;
        params.imu_en = cfg_.imu_en;
        params.start_in_aggressive_motion = cfg_.start_in_aggressive_motion;
        params.extrinsic_est_en = cfg_.extrinsic_est_en;
        params.imu_time_inte = cfg_.imu_time_inte;
        params.lidar_meas_cov = cfg_.lidar_meas_cov;
        params.acc_cov_input = cfg_.acc_cov_input;
        params.vel_cov = cfg_.vel_cov;
        params.gyr_cov_input = cfg_.gyr_cov_input;
        params.gyr_cov_output = cfg_.gyr_cov_output;
        params.acc_cov_output = cfg_.acc_cov_output;
        params.b_gyr_cov = cfg_.b_gyr_cov;
        params.b_acc_cov = cfg_.b_acc_cov;
        params.imu_meas_acc_cov = cfg_.imu_meas_acc_cov;
        params.imu_meas_omg_cov = cfg_.imu_meas_omg_cov;
        params.match_s = cfg_.match_s;
        params.gravity_align = cfg_.gravity_align;
        params.gravity = cfg_.gravity;
        params.gravity_init = cfg_.gravity_init;
        params.extrinsic_T = cfg_.extrinsic_t;
        params.extrinsic_R = cfg_.extrinsic_r;
        params.publish_odometry_without_downsample =
            cfg_.publish_odometry_without_downsample;
        params.odom_only = cfg_.odom_only;

        point_lio_ = std::make_unique<PointLio>(params, cfg_.msr_freq, cfg_.main_freq);

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

        SetLivoxLidarPointCloudCallBack(&PointLioModule::point_cloud_cb, this);
        SetLivoxLidarImuDataCallback(&PointLioModule::imu_cb, this);
        SetLivoxLidarInfoChangeCallback(&PointLioModule::info_cb, this);

        if (!LivoxLidarSdkStart()) {
            LivoxLidarSdkUninit();
            throw std::runtime_error("LivoxLidarSdkStart failed");
        }
        logging::info("pointlio SDK started, waiting for device",
                      {logging::Field("lidar_ip", cfg_.lidar_ip),
                       logging::Field("host_ip", cfg_.host_ip)});
    }

    // Own processing loop at ~main_freq: drain accumulated points into a
    // CustomMsg at frame rate, run a Point-LIO step, publish rate-limited.
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
        static_cast<PointLioModule*>(ctx)->on_point_cloud(data);
    }

    static void imu_cb(const uint32_t, const uint8_t, LivoxLidarEthernetPacket* data,
                       void* ctx) {
        static_cast<PointLioModule*>(ctx)->on_imu(data);
    }

    static void info_cb(const uint32_t handle, const LivoxLidarInfo* info, void* ctx) {
        static_cast<PointLioModule*>(ctx)->on_info(handle, info);
    }

    // Runs on a Livox SDK callback thread: accumulate raw points into the
    // current frame (Livox SDK raw -> CustomPoint).
    void on_point_cloud(LivoxLidarEthernetPacket* data) {
        if (shutdown_requested() || data == nullptr) return;

        uint64_t ts_ns = packet_timestamp_ns(data);
        uint16_t dot_num = data->dot_num;

        // Per-point intra-packet offset (matches livox_ros_driver2). Without it
        // all points share one timestamp and per-point deskew is lost.
        // time_interval unit is 0.1us, so *100 -> ns.
        const uint64_t point_interval_ns =
            dot_num > 0 ? static_cast<uint64_t>(data->time_interval) * 100 / dot_num : 0;

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
                cp.line = 0;  // Mid-360: single line
                cp.offset_time =
                    static_cast<uli>((ts_ns - frame_start_ns_) + i * point_interval_ns);
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
                cp.offset_time =
                    static_cast<uli>((ts_ns - frame_start_ns_) + i * point_interval_ns);
                accumulated_points_.push_back(cp);
            }
        }
    }

    // Runs on a Livox SDK callback thread: feed IMU samples into the estimator.
    void on_imu(LivoxLidarEthernetPacket* data) {
        if (shutdown_requested() || data == nullptr || !point_lio_) return;

        double ts = static_cast<double>(packet_timestamp_ns(data)) / 1e9;
        auto* imu_pts = reinterpret_cast<const LivoxLidarImuRawPoint*>(data->data);
        uint16_t dot_num = data->dot_num;

        // Serialize EKF access against the main loop (step). Held across the
        // whole packet so its samples feed atomically.
        std::lock_guard<std::mutex> lio_lock(lio_mutex_);
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

            // Point-LIO expects accel in g (EKF does its own scaling). SDK
            // already reports g, so feed raw — scaling by GRAVITY_MS2 would
            // double-scale and trip the satu_acc check at rest.
            imu_msg->linear_acceleration.x = static_cast<double>(imu_pts[i].acc_x);
            imu_msg->linear_acceleration.y = static_cast<double>(imu_pts[i].acc_y);
            imu_msg->linear_acceleration.z = static_cast<double>(imu_pts[i].acc_z);
            for (int j = 0; j < 9; ++j) { imu_msg->linear_acceleration_covariance[j] = 0.0; }

            point_lio_->feed_imu(imu_msg);
        }
    }

    void on_info(uint32_t handle, const LivoxLidarInfo* info) {
        if (info == nullptr) return;

        char sn[17] = {};
        std::memcpy(sn, info->sn, 16);
        char ip[17] = {};
        std::memcpy(ip, info->lidar_ip, 16);
        logging::info("pointlio device connected",
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
        // Point-LIO. Hold pc_mutex_ across the rate-limit check + swap so a
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

        // Serialize EKF access against the SDK IMU callback (on_imu) for the
        // rest of the iteration — feed_lidar/process/get_* all touch the estimator.
        std::lock_guard<std::mutex> lio_lock(lio_mutex_);
        if (!points.empty()) {
            const size_t num_points = points.size();
            auto lidar_msg = boost::make_shared<custom_messages::CustomMsg>();
            lidar_msg->header.seq = 0;
            lidar_msg->header.stamp =
                custom_messages::Time().fromSec(static_cast<double>(frame_start) / 1e9);
            lidar_msg->header.frame_id = "livox_frame";
            lidar_msg->timebase = frame_start;
            lidar_msg->lidar_id = 0;
            for (int i = 0; i < 3; i++) { lidar_msg->rsvd[i] = 0; }
            lidar_msg->point_num = static_cast<uli>(num_points);
            lidar_msg->points = std::move(points);
            if (cfg_.debug) {
                logging::info("pointlio feed_lidar frame",
                              {logging::Field("points",
                                              static_cast<std::int64_t>(num_points))});
            }
            point_lio_->feed_lidar(lidar_msg);
        }

        // One Point-LIO IESKF step (cheap when queues empty).
        point_lio_->process();

        auto pose = point_lio_->get_pose();
        if (!pose.empty() && (pose[0] != 0.0 || pose[1] != 0.0 || pose[2] != 0.0)) {
            double ts = std::chrono::duration<double>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();

            // get_body_cloud is the loop's costliest step, so build it only when
            // a publish is due.
            if (now - last_pc_publish >= pc_interval_) {
                auto body_cloud = point_lio_->get_body_cloud();
                if (body_cloud && !body_cloud->empty()) {
                    publish_pointcloud(body_cloud, ts);
                    last_pc_publish = now;
                    if (cfg_.debug) {
                        logging::info(
                            "pointlio publish lidar",
                            {logging::Field("points",
                                            static_cast<std::int64_t>(body_cloud->size())),
                             logging::Field("x", pose[0]), logging::Field("y", pose[1]),
                             logging::Field("z", pose[2])});
                    }
                }
            }

            // Pose + covariance at odom_freq.
            if (now - last_odom_publish >= odom_interval_) {
                publish_odometry(point_lio_->get_odometry(), ts);
                last_odom_publish = now;
                if (cfg_.debug) {
                    logging::info("pointlio publish odom",
                                  {logging::Field("x", pose[0]),
                                   logging::Field("y", pose[1]),
                                   logging::Field("z", pose[2])});
                }
            }
        }
    }

    // Publish the undistorted scan in the sensor's own frame (get_body_cloud),
    // so points go out as-is with no world registration.
    void publish_pointcloud(const PointCloudXYZI::Ptr& cloud, double ts) {
        int num_points = static_cast<int>(cloud->size());

        sensor_msgs::PointCloud2 pc;
        pc.header = make_header(cfg_.sensor_frame_id, ts);
        pc.height = 1;
        pc.width = num_points;
        pc.is_bigendian = 0;
        pc.is_dense = 1;

        pc.fields_length = 4;
        pc.fields.resize(4);
        auto make_field = [](const std::string& name, int32_t offset) {
            sensor_msgs::PointField f;
            f.name = name;
            f.offset = offset;
            f.datatype = sensor_msgs::PointField::FLOAT32;
            f.count = 1;
            return f;
        };
        pc.fields[0] = make_field("x", 0);
        pc.fields[1] = make_field("y", 4);
        pc.fields[2] = make_field("z", 8);
        pc.fields[3] = make_field("intensity", 12);

        pc.point_step = 16;  // 4 float32
        pc.row_step = pc.point_step * num_points;
        pc.data_length = pc.row_step;
        pc.data.resize(pc.data_length);

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

        // Velocity from Point-LIO's IESKF state (its key output over FAST-LIO).
        msg.twist.twist.linear.x = odom.twist.twist.linear.x;
        msg.twist.twist.linear.y = odom.twist.twist.linear.y;
        msg.twist.twist.linear.z = odom.twist.twist.linear.z;
        msg.twist.twist.angular.x = odom.twist.twist.angular.x;
        msg.twist.twist.angular.y = odom.twist.twist.angular.y;
        msg.twist.twist.angular.z = odom.twist.twist.angular.z;
        std::memset(msg.twist.covariance, 0, sizeof(msg.twist.covariance));

        odometry_.publish(msg);
    }

    PointLioConfig cfg_;
    Output<sensor_msgs::PointCloud2> lidar_;
    Output<nav_msgs::Odometry> odometry_;
    std::unique_ptr<PointLio> point_lio_;

    std::chrono::microseconds frame_interval_{100000};
    std::chrono::microseconds pc_interval_{100000};
    std::chrono::microseconds odom_interval_{20000};
    std::chrono::microseconds process_period_{200};

    // Frame accumulator (Livox SDK raw -> CustomMsg)
    std::mutex pc_mutex_;
    std::vector<custom_messages::CustomPoint> accumulated_points_;
    uint64_t frame_start_ns_ = 0;
    bool frame_has_ts_ = false;

    // Serializes all Point-LIO EKF access. The SDK delivers IMU on its own
    // callback thread (on_imu -> feed_imu) while the main loop runs feed_lidar/
    // process/get_* — Point-LIO's estimator is not thread-safe, so without this
    // the two threads race on the EKF state and occasionally emit a corrupt 2nd
    // trajectory. Distinct from pc_mutex_ (which only guards the point
    // accumulator) so incoming point packets can still accumulate while the EKF
    // is processing.
    std::mutex lio_mutex_;
};

int main() {
    dimos::native::run_with_transport<PointLioModule>();
    return 0;
}
