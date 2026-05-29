#include "parameters.h"

bool is_first_frame = true;

double lidar_end_time = 0.0;
double first_lidar_time = 0.0;
double time_con = 0.0;

double last_timestamp_lidar = -1.0;
double last_timestamp_imu = -1.0;

int pcd_index = 0;

std::string lid_topic;
std::string imu_topic;

bool prop_at_freq_of_imu;
bool check_satu;
bool con_frame;
bool cut_frame;

bool use_imu_as_input;
bool space_down_sample;
bool publish_odometry_without_downsample;

int init_map_size;
int con_frame_num;

double match_s;
double satu_acc;
double satu_gyro;
double cut_frame_time_interval;

float plane_thr;

double filter_size_surf_min;
double filter_size_map_min;
double fov_deg;

double cube_len;
float DET_RANGE;

bool imu_en;
bool gravity_align;
bool non_station_start;

double imu_time_inte;

double laser_point_cov;
double acc_norm;

double vel_cov;
double acc_cov_input;
double gyr_cov_input;

double gyr_cov_output;
double acc_cov_output;
double b_gyr_cov;
double b_acc_cov;

double imu_meas_acc_cov;
double imu_meas_omg_cov;

int lidar_type;
int pcd_save_interval;

std::vector<double> gravity_init;
std::vector<double> gravity;

std::vector<double> extrinT;
std::vector<double> extrinR;

bool runtime_pos_log;
bool pcd_save_en;
bool path_en;
bool extrinsic_est_en = true;

bool scan_pub_en;
bool scan_body_pub_en;

shared_ptr<Preprocess> p_pre;
double time_lag_imu_to_lidar = 0.0;

// yaml-cpp drives readParameters in the no-ROS port. The launch file's
// <param name=…/> values (which were never in the yaml) are baked in
// as our defaults here, matching launch/mapping_unilidar_l1.launch.
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace {

template <typename T>
T y_get(const YAML::Node& root, const std::string& dotted, const T& def) {
    YAML::Node n = YAML::Clone(root);
    size_t p = 0;
    while (true) {
        size_t q = dotted.find('/', p);
        std::string key = dotted.substr(p, q - p);
        if (!n.IsMap() || !n[key]) return def;
        n = n[key];
        if (q == std::string::npos) break;
        p = q + 1;
    }
    try { return n.as<T>(); } catch (...) { return def; }
}

}  // namespace

void readParametersYaml(const std::string& yaml_path) {
    p_pre.reset(new Preprocess());

    YAML::Node cfg;
    if (!yaml_path.empty()) {
        try {
            cfg = YAML::LoadFile(yaml_path);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "readParametersYaml: could not load %s: %s\n",
                         yaml_path.c_str(), e.what());
        }
    }

    // Launch-file values (mapping_unilidar_l1.launch). These were never
    // in the yaml; bake them in here as our defaults so the binary runs
    // out-of-the-box on L1 data.
    prop_at_freq_of_imu = true;
    use_imu_as_input    = false;
    check_satu          = true;
    init_map_size       = 10;
    space_down_sample   = true;
    p_pre->point_filter_num = 1;
    filter_size_surf_min = y_get<double>(cfg, "mapping/filter_size_surf", 0.1);
    filter_size_map_min  = y_get<double>(cfg, "mapping/filter_size_map",  0.1);
    cube_len             = 1000.0;
    runtime_pos_log      = true;   // enable Log/mat_out.txt trajectory dump (offline analysis)

    // Yaml-driven values (config/unilidar_l1.yaml).
    satu_acc      = y_get<double>(cfg, "mapping/satu_acc",      30.0);
    satu_gyro     = y_get<double>(cfg, "mapping/satu_gyro",     35.0);
    acc_norm      = y_get<double>(cfg, "mapping/acc_norm",      9.81);
    plane_thr     = y_get<float>(cfg,  "mapping/plane_thr",     0.1f);
    lid_topic     = y_get<std::string>(cfg, "common/lid_topic", "/unilidar/cloud");
    imu_topic     = y_get<std::string>(cfg, "common/imu_topic", "/unilidar/imu");
    con_frame     = y_get<bool>(cfg,   "common/con_frame",      false);
    con_frame_num = y_get<int>(cfg,    "common/con_frame_num",  1);
    cut_frame     = y_get<bool>(cfg,   "common/cut_frame",      false);
    cut_frame_time_interval = y_get<double>(cfg, "common/cut_frame_time_interval", 0.1);
    time_lag_imu_to_lidar   = y_get<double>(cfg, "common/time_lag_imu_to_lidar",   0.0);
    DET_RANGE     = y_get<float>(cfg,  "mapping/det_range",     100.0f);
    fov_deg       = y_get<double>(cfg, "mapping/fov_degree",    180.0);
    imu_en        = y_get<bool>(cfg,   "mapping/imu_en",        true);
    non_station_start = y_get<bool>(cfg, "mapping/start_in_aggressive_motion", false);
    extrinsic_est_en  = y_get<bool>(cfg, "mapping/extrinsic_est_en", false);
    imu_time_inte = y_get<double>(cfg, "mapping/imu_time_inte", 0.004);
    laser_point_cov = y_get<double>(cfg, "mapping/lidar_meas_cov", 0.01);
    acc_cov_input  = y_get<double>(cfg, "mapping/acc_cov_input",  0.1);
    vel_cov        = y_get<double>(cfg, "mapping/vel_cov",        20.0);
    gyr_cov_input  = y_get<double>(cfg, "mapping/gyr_cov_input",  0.01);
    gyr_cov_output = y_get<double>(cfg, "mapping/gyr_cov_output", 1000.0);
    acc_cov_output = y_get<double>(cfg, "mapping/acc_cov_output", 500.0);
    b_gyr_cov      = y_get<double>(cfg, "mapping/b_gyr_cov",      0.0001);
    b_acc_cov      = y_get<double>(cfg, "mapping/b_acc_cov",      0.0001);
    imu_meas_acc_cov = y_get<double>(cfg, "mapping/imu_meas_acc_cov", 0.1);
    imu_meas_omg_cov = y_get<double>(cfg, "mapping/imu_meas_omg_cov", 0.1);
    p_pre->blind = y_get<double>(cfg, "preprocess/blind", 0.5);
    lidar_type   = y_get<int>(cfg, "preprocess/lidar_type", 5);
    p_pre->N_SCANS  = y_get<int>(cfg, "preprocess/scan_line", 18);
    p_pre->SCAN_RATE = y_get<int>(cfg, "preprocess/scan_rate", 10);
    p_pre->time_unit = y_get<int>(cfg, "preprocess/timestamp_unit", 0);
    match_s      = y_get<double>(cfg, "mapping/match_s", 81.0);
    gravity_align = y_get<bool>(cfg, "mapping/gravity_align", true);
    gravity      = y_get<std::vector<double>>(cfg, "mapping/gravity",      {0.0, 0.0, -9.81});
    gravity_init = y_get<std::vector<double>>(cfg, "mapping/gravity_init", {0.0, 0.0, -9.81});
    extrinT      = y_get<std::vector<double>>(cfg, "mapping/extrinsic_T",  {0.007698, 0.014655, -0.00667});
    extrinR      = y_get<std::vector<double>>(cfg, "mapping/extrinsic_R",  {1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0});
    publish_odometry_without_downsample =
        y_get<bool>(cfg, "odometry/publish_odometry_without_downsample", true);
    path_en      = y_get<bool>(cfg, "publish/path_en", true);
    scan_pub_en  = y_get<bool>(cfg, "publish/scan_publish_en", true);
    scan_body_pub_en = y_get<bool>(cfg, "publish/scan_bodyframe_pub_en", false);
    pcd_save_en  = y_get<bool>(cfg, "pcd_save/pcd_save_en", true);
    pcd_save_interval = y_get<int>(cfg, "pcd_save/interval", -1);
}

// Kept for source-compat with laserMapping.cpp's main(); the offline
// main() calls readParametersYaml(path) BEFORE this, so by the time
// readParameters runs, all globals are set and the NodeHandle ref is
// unused.
void readParameters(ros::NodeHandle& /*nh*/) {
    if (p_pre == nullptr) {
        readParametersYaml(std::string());  // last-resort defaults
    }
}
