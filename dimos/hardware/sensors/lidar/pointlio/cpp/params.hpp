// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// PointLioConfig (the stdin JSON, same keys as the Python PointLioConfig) and
// its mapping onto the core's PointLioParams. Shared by main.cpp and the
// golden harness so both feed the estimator identically.

#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#include "dimos/native/config.hpp"
#include "parameters.h"

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
    int scan_line;
    int scan_rate;
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

    void validate() const {
        dimos::native::require_positive(frequency, "frequency");
        dimos::native::require_positive(msr_freq, "msr_freq");
        dimos::native::require_positive(main_freq, "main_freq");
        dimos::native::require_positive(pointcloud_freq, "pointcloud_freq");
        dimos::native::require_positive(odom_freq, "odom_freq");
    }
};

// iVox neighbour stencil codes, from Point-LIO's parameters.cpp.
inline int ivox_nearby_code(const std::string& name) {
    if (name == "center") return 0;
    if (name == "nearby6") return 6;
    if (name == "nearby18") return 18;
    if (name == "nearby26") return 26;
    throw std::runtime_error(
        "ivox_nearby_type must be one of: center nearby6 nearby18 nearby26, got '" +
        name + "'");
}

inline PointLioParams to_params(const PointLioConfig& cfg) {
    PointLioParams params;
    params.con_frame = cfg.con_frame;
    params.con_frame_num = cfg.con_frame_num;
    params.cut_frame = cfg.cut_frame;
    params.cut_frame_time_interval = cfg.cut_frame_time_interval;
    params.time_lag_imu_to_lidar = cfg.time_lag_imu_to_lidar;
    params.scan_line = cfg.scan_line;
    params.scan_rate = cfg.scan_rate;
    params.blind = cfg.blind;
    params.point_filter_num = cfg.point_filter_num;
    params.use_imu_as_input = cfg.use_imu_as_input;
    params.prop_at_freq_of_imu = cfg.prop_at_freq_of_imu;
    params.check_satu = cfg.check_satu;
    params.init_map_size = cfg.init_map_size;
    params.space_down_sample = cfg.space_down_sample;
    params.satu_acc = cfg.satu_acc;
    params.satu_gyro = cfg.satu_gyro;
    params.acc_norm = cfg.acc_norm;
    params.plane_thr = cfg.plane_thr;
    params.filter_size_surf = cfg.filter_size_surf;
    params.filter_size_map = cfg.filter_size_map;
    params.ivox_grid_resolution = cfg.ivox_grid_resolution;
    params.ivox_nearby_type = ivox_nearby_code(cfg.ivox_nearby_type);
    params.cube_side_length = cfg.cube_side_length;
    params.det_range = cfg.det_range;
    params.fov_degree = cfg.fov_degree;
    params.imu_en = cfg.imu_en;
    params.start_in_aggressive_motion = cfg.start_in_aggressive_motion;
    params.extrinsic_est_en = cfg.extrinsic_est_en;
    params.imu_time_inte = cfg.imu_time_inte;
    params.lidar_meas_cov = cfg.lidar_meas_cov;
    params.acc_cov_input = cfg.acc_cov_input;
    params.vel_cov = cfg.vel_cov;
    params.gyr_cov_input = cfg.gyr_cov_input;
    params.gyr_cov_output = cfg.gyr_cov_output;
    params.acc_cov_output = cfg.acc_cov_output;
    params.b_gyr_cov = cfg.b_gyr_cov;
    params.b_acc_cov = cfg.b_acc_cov;
    params.imu_meas_acc_cov = cfg.imu_meas_acc_cov;
    params.imu_meas_omg_cov = cfg.imu_meas_omg_cov;
    params.match_s = cfg.match_s;
    params.gravity_align = cfg.gravity_align;
    params.gravity = cfg.gravity;
    params.gravity_init = cfg.gravity_init;
    params.extrinsic_T = cfg.extrinsic_t;
    params.extrinsic_R = cfg.extrinsic_r;
    params.publish_odometry_without_downsample = cfg.publish_odometry_without_downsample;
    params.odom_only = cfg.odom_only;
    return params;
}
