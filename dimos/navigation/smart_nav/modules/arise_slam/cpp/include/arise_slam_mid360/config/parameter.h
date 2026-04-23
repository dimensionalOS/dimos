#pragma once

#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "arise_slam_mid360/utils/Twist.h"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

enum class SensorType {VELODYNE, OUSTER, LIVOX};
extern std::string IMU_TOPIC;
extern std::string LASER_TOPIC;
extern std::string ODOM_TOPIC;
extern std::string DepthUP_TOPIC;
extern std::string DepthDown_TOPIC;
extern std::string ProjectName;

extern std::string WORLD_FRAME;
extern std::string WORLD_FRAME_ROT;
extern std::string SENSOR_FRAME;
extern std::string SENSOR_FRAME_ROT;
extern SensorType sensor;

extern int PROVIDE_IMU_LASER_EXTRINSIC;

extern std::vector<Eigen::Matrix3d> RIC;

extern std::vector<Eigen::Vector3d> TIC;

extern Eigen::Matrix3d imu_laser_R;

extern Eigen::Vector3d imu_laser_T;

extern Eigen::Matrix3d cam_laser_R;

extern Eigen::Vector3d cam_laser_T;

extern Eigen::Matrix3d imu_camera_R;

extern Eigen::Vector3d imu_camera_T;

extern Eigen::Vector3d imu_laser_offset;

extern Transformd Tcam_lidar;

extern Transformd T_i_c;

extern Transformd T_i_l;

extern Transformd T_l_i;

extern float up_realsense_roll;

extern float up_realsense_pitch;

extern float up_realsense_yaw;

extern float up_realsense_x;

extern float up_realsense_y;

extern float up_realsense_z;

extern float down_realsense_roll;

extern float down_realsense_pitch;

extern float down_realsense_yaw;

extern float down_realsense_x;

extern float down_realsense_y;

extern float down_realsense_z;

extern float yaw_ratio;

// extern float min_range;

// extern float max_range;

// extern float blindFront;

// extern float blindBack;

// extern float blindRight;

// extern float blindLeft;

// extern int skipFrame;


//extern float scan_registration_voxel_size;

//extern float lidar_correction_noise;

// extern bool use_no_motion_prior;



//extern float smooth_factor;

//extern bool use_imu_roll_pitch;

// extern bool start_from_previous_map;

// These now take no parameters -- main.cpp sets the globals directly via CLI args
bool readGlobalparam();
bool readCalibration();

// the following are UBUNTU/LINUX ONLY terminal color codes.
// #define RESET "\033[0m"
// #define BLACK "\033[0;30m"   /* Black */
// #define RED "\033[0;31m"     /* Red */
// #define GREEN "\033[0;32m"   /* Green */
// #define YELLOW "\033[0;33m"  /* Yellow */
// #define BLUE "\033[0;34m"    /* Blue */
// #define MAGENTA "\033[0;35m" /* Magenta */
// #define CYAN "\033[0;36m"    /* Cyan */
// #define WHITE "\033[0;37m"   /* White */
