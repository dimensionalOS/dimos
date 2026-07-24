// Livox Mid-360 native module on the dimos C++ SDK. A pure source: it drives the
// Livox SDK2, accumulates points from the SDK callbacks, and publishes a
// PointCloud2 at a fixed rate on `lidar` plus Imu samples as they arrive on
// `imu`. No inputs, so it overrides handle() with its own emit loop.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "livox_sdk_config.hpp"
#include "point_cloud_utils.hpp"

#include "dimos/native.hpp"

#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointField.hpp"
#include "std_msgs/Header.hpp"

using dimos::native::Builder;
using dimos::native::Config;
using dimos::native::Module;
using dimos::native::Output;
namespace logging = dimos::native::log;

using livox_common::GRAVITY_MS2;
using livox_common::DATA_TYPE_CARTESIAN_HIGH;
using livox_common::DATA_TYPE_CARTESIAN_LOW;

struct Mid360Config {
    std::string host_ip;
    std::string lidar_ip;
    double frequency;
    bool enable_imu;
    std::string frame_id;
    std::string imu_frame_id;
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

double packet_timestamp(const LivoxLidarEthernetPacket* pkt) {
    uint64_t ns = 0;
    std::memcpy(&ns, pkt->timestamp, sizeof(uint64_t));
    return static_cast<double>(ns) / 1e9;
}

}  // namespace

class Mid360 : public Module {
public:
    void build(Builder& builder, Config& config) override {
        cfg_ = config.parse<Mid360Config>();
        lidar_ = builder.output<sensor_msgs::PointCloud2>("lidar");
        if (cfg_.enable_imu) {
            imu_ = builder.output<sensor_msgs::Imu>("imu");
        }
        frame_interval_ =
            std::chrono::microseconds(static_cast<int64_t>(1e6 / cfg_.frequency));
    }

    void setup() override {
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

        if (!livox_common::init_livox_sdk(cfg_.host_ip, cfg_.lidar_ip, ports)) {
            throw std::runtime_error("init_livox_sdk failed");
        }

        SetLivoxLidarPointCloudCallBack(&Mid360::point_cloud_cb, this);
        if (cfg_.enable_imu) {
            SetLivoxLidarImuDataCallback(&Mid360::imu_cb, this);
        }
        SetLivoxLidarInfoChangeCallback(&Mid360::info_cb, this);

        if (!LivoxLidarSdkStart()) {
            LivoxLidarSdkUninit();
            throw std::runtime_error("LivoxLidarSdkStart failed");
        }
        logging::info("mid360 SDK started, waiting for device",
                      {logging::Field("lidar_ip", cfg_.lidar_ip),
                       logging::Field("host_ip", cfg_.host_ip)});
    }

    // Own emit loop: swap out the accumulated frame and publish at `frequency`.
    void handle() override {
        auto last_emit = std::chrono::steady_clock::now();
        while (!shutdown_requested()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            auto now = std::chrono::steady_clock::now();
            if (now - last_emit >= frame_interval_) {
                emit_frame();
                last_emit = now;
            }
        }
    }

    void teardown() override {
        LivoxLidarSdkUninit();
    }

private:
    static void point_cloud_cb(const uint32_t, const uint8_t,
                               LivoxLidarEthernetPacket* data, void* ctx) {
        static_cast<Mid360*>(ctx)->on_point_cloud(data);
    }

    static void imu_cb(const uint32_t, const uint8_t, LivoxLidarEthernetPacket* data,
                       void* ctx) {
        static_cast<Mid360*>(ctx)->on_imu(data);
    }

    static void info_cb(const uint32_t handle, const LivoxLidarInfo* info, void* ctx) {
        static_cast<Mid360*>(ctx)->on_info(handle, info);
    }

    void on_point_cloud(LivoxLidarEthernetPacket* data) {
        if (shutdown_requested() || data == nullptr) return;

        double ts = packet_timestamp(data);
        uint16_t dot_num = data->dot_num;

        std::lock_guard<std::mutex> lock(pc_mutex_);
        if (!frame_has_ts_) {
            frame_ts_ = ts;
            frame_has_ts_ = true;
        }

        if (data->data_type == DATA_TYPE_CARTESIAN_HIGH) {
            auto* pts = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(data->data);
            for (uint16_t i = 0; i < dot_num; ++i) {
                // High-precision coordinates are in mm.
                xyz_.push_back(static_cast<float>(pts[i].x) / 1000.0f);
                xyz_.push_back(static_cast<float>(pts[i].y) / 1000.0f);
                xyz_.push_back(static_cast<float>(pts[i].z) / 1000.0f);
                intensity_.push_back(static_cast<float>(pts[i].reflectivity) / 255.0f);
            }
        } else if (data->data_type == DATA_TYPE_CARTESIAN_LOW) {
            auto* pts = reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(data->data);
            for (uint16_t i = 0; i < dot_num; ++i) {
                // Low-precision coordinates are in cm.
                xyz_.push_back(static_cast<float>(pts[i].x) / 100.0f);
                xyz_.push_back(static_cast<float>(pts[i].y) / 100.0f);
                xyz_.push_back(static_cast<float>(pts[i].z) / 100.0f);
                intensity_.push_back(static_cast<float>(pts[i].reflectivity) / 255.0f);
            }
        }
    }

    void on_imu(LivoxLidarEthernetPacket* data) {
        if (shutdown_requested() || data == nullptr) return;

        double ts = packet_timestamp(data);
        auto* imu_pts = reinterpret_cast<const LivoxLidarImuRawPoint*>(data->data);
        uint16_t dot_num = data->dot_num;

        for (uint16_t i = 0; i < dot_num; ++i) {
            sensor_msgs::Imu msg;
            msg.header = make_header(cfg_.imu_frame_id, ts);

            // Orientation unknown: identity with -1 covariance to flag it.
            msg.orientation.x = 0.0;
            msg.orientation.y = 0.0;
            msg.orientation.z = 0.0;
            msg.orientation.w = 1.0;
            msg.orientation_covariance[0] = -1.0;

            msg.angular_velocity.x = static_cast<double>(imu_pts[i].gyro_x);
            msg.angular_velocity.y = static_cast<double>(imu_pts[i].gyro_y);
            msg.angular_velocity.z = static_cast<double>(imu_pts[i].gyro_z);

            msg.linear_acceleration.x = static_cast<double>(imu_pts[i].acc_x) * GRAVITY_MS2;
            msg.linear_acceleration.y = static_cast<double>(imu_pts[i].acc_y) * GRAVITY_MS2;
            msg.linear_acceleration.z = static_cast<double>(imu_pts[i].acc_z) * GRAVITY_MS2;

            imu_.publish(msg);
        }
    }

    void on_info(uint32_t handle, const LivoxLidarInfo* info) {
        if (info == nullptr) return;

        char sn[17] = {};
        std::memcpy(sn, info->sn, 16);
        char ip[17] = {};
        std::memcpy(ip, info->lidar_ip, 16);
        logging::info("mid360 device connected",
                      {logging::Field("sn", std::string(sn)),
                       logging::Field("ip", std::string(ip))});

        SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
        if (cfg_.enable_imu) {
            EnableLivoxLidarImuData(handle, nullptr, nullptr);
        }
    }

    void emit_frame() {
        std::vector<float> xyz;
        std::vector<float> intensity;
        double ts = 0.0;
        {
            std::lock_guard<std::mutex> lock(pc_mutex_);
            if (xyz_.empty()) return;
            xyz.swap(xyz_);
            intensity.swap(intensity_);
            ts = frame_ts_;
            frame_has_ts_ = false;
        }
        publish_pointcloud(xyz, intensity, ts);
    }

    void publish_pointcloud(const std::vector<float>& xyz,
                            const std::vector<float>& intensity, double ts) {
        int num_points = static_cast<int>(xyz.size()) / 3;

        sensor_msgs::PointCloud2 pc = make_xyzi_cloud(cfg_.frame_id, ts, num_points);

        for (int i = 0; i < num_points; ++i) {
            float* dst = reinterpret_cast<float*>(pc.data.data() + i * 16);
            dst[0] = xyz[i * 3 + 0];
            dst[1] = xyz[i * 3 + 1];
            dst[2] = xyz[i * 3 + 2];
            dst[3] = intensity[i];
        }

        lidar_.publish(pc);
    }

    Mid360Config cfg_;
    Output<sensor_msgs::PointCloud2> lidar_;
    Output<sensor_msgs::Imu> imu_;
    std::chrono::microseconds frame_interval_{100000};

    std::mutex pc_mutex_;
    std::vector<float> xyz_;
    std::vector<float> intensity_;
    double frame_ts_ = 0.0;
    bool frame_has_ts_ = false;
};

int main() {
    dimos::native::run_with_transport<Mid360>();
    return 0;
}
