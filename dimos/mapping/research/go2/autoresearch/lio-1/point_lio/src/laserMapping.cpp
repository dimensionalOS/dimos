#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
// (dropped <Python.h> — was only here for matplotlibcpp, unused)
#include <iomanip>
#include <so3_math.h>
#include <Eigen/Core>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
// #include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include "IMU_Processing.hpp"
#include "parameters.h"
#include "Estimator.h"

#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;

int feats_down_size = 0;
int time_log_counter = 0;
int scan_count = 0;
int publish_count = 0;

int frame_ct = 0;
double time_update_last = 0.0;
double time_current = 0.0;
double time_predict_last_const = 0.0;
double t_last = 0.0;

shared_ptr<ImuProcess> p_imu(new ImuProcess());
bool init_map = false;
bool flg_first_scan = true;
PointCloudXYZI::Ptr ptr_con(new PointCloudXYZI());

double T1[MAXN];
double s_plot[MAXN];
double s_plot2[MAXN];
double s_plot3[MAXN];
double s_plot11[MAXN];

double match_time = 0;
double solve_time = 0;
double propag_time = 0;
double update_time = 0;

bool lidar_pushed = false;
bool flg_reset = false;
bool flg_exit = false;

vector<BoxPointType> cub_needrm;

deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<double> time_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_deque;

PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;

MeasureGroup Measures;

sensor_msgs::Imu imu_last, imu_next;
sensor_msgs::Imu::ConstPtr imu_last_ptr;
nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::PoseStamped msg_body_pose;


void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}


inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang;
    if (!use_imu_as_input)
    {
        rot_ang = SO3ToEuler(kf_output.x_.rot);
    }
    else
    {
        rot_ang = SO3ToEuler(kf_input.x_.rot);
    }

    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));
    if (use_imu_as_input)
    {

        fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2));
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2));
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1), kf_input.x_.gravity(2));
    }
    else
    {
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2));
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2));
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1), kf_output.x_.gravity(2));
    }
    fprintf(fp, "\r\n");
    fflush(fp);
}


void pointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{

    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu;
    if (extrinsic_est_en)
    {
        if (!use_imu_as_input)
        {
            p_body_imu = kf_output.x_.offset_R_L_I.normalized() * p_body_lidar + kf_output.x_.offset_T_L_I;
        }
        else
        {
            p_body_imu = kf_input.x_.offset_R_L_I.normalized() * p_body_lidar + kf_input.x_.offset_T_L_I;
        }
    }
    else
    {
        p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
    }

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);

    po->intensity = pi->intensity;
}

int points_cache_size = 0;
void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
}


BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.shrink_to_fit();

    V3D pos_LiD;
    if (use_imu_as_input)
    {
        pos_LiD = kf_input.x_.pos + kf_input.x_.rot.normalized() * Lidar_T_wrt_IMU;
    }
    else
    {
        pos_LiD = kf_output.x_.pos + kf_output.x_.rot.normalized() * Lidar_T_wrt_IMU;
    }

    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }

    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }

    if (!need_move)
        return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    if (cub_needrm.size() > 0)
        int kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
}


void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // std::cout << "standard_pcl_cbk() run once!\n";

    mtx_buffer.lock();

    scan_count++;

    double preprocess_start_time = omp_get_wtime();

    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");

        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    last_timestamp_lidar = msg->header.stamp.toSec();

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr ptr_div(new PointCloudXYZI());

    double time_div = msg->header.stamp.toSec();

    p_pre->process(msg, ptr);

    // std::cout << "ptr->points.size() = " << ptr->points.size() << std::endl;

    if (cut_frame)
    {

        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++)
        {

            ptr_div->push_back(ptr->points[i]);

            if (ptr->points[i].curvature / double(1000) + msg->header.stamp.toSec() - time_div > cut_frame_time_interval)
            {
                if (ptr_div->size() < 1)
                    continue;

                PointCloudXYZI::Ptr ptr_div_i(new PointCloudXYZI());
                *ptr_div_i = *ptr_div;

                lidar_buffer.push_back(ptr_div_i);

                time_buffer.push_back(time_div);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }

        if (!ptr_div->empty())
        {
            lidar_buffer.push_back(ptr_div);

            time_buffer.push_back(time_div);
        }
    }
    else if (con_frame)
    {

        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar;
        }

        if (frame_ct < con_frame_num)
        {
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct++;
        }

        else
        {
            PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            lidar_buffer.push_back(ptr_con_i);
            double time_con_i = time_con;
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    }
    else
    {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(msg->header.stamp.toSec());
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}


// void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
// {

//     mtx_buffer.lock();

//     double preprocess_start_time = omp_get_wtime();

//     scan_count++;

//     if (msg->header.stamp.toSec() < last_timestamp_lidar)
//     {
//         ROS_ERROR("lidar loop back, clear buffer");

//         mtx_buffer.unlock();
//         sig_buffer.notify_all();
//         return;
//     }

//     last_timestamp_lidar = msg->header.stamp.toSec();

//     PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
//     PointCloudXYZI::Ptr ptr_div(new PointCloudXYZI());

//     p_pre->process(msg, ptr);
//     double time_div = msg->header.stamp.toSec();

//     if (cut_frame)
//     {

//         sort(ptr->points.begin(), ptr->points.end(), time_list);

//         for (int i = 0; i < ptr->size(); i++)
//         {

//             ptr_div->push_back(ptr->points[i]);
//             if (ptr->points[i].curvature / double(1000) + msg->header.stamp.toSec() - time_div > cut_frame_time_interval)
//             {
//                 if (ptr_div->size() < 1)
//                     continue;
//                 PointCloudXYZI::Ptr ptr_div_i(new PointCloudXYZI());

//                 *ptr_div_i = *ptr_div;

//                 lidar_buffer.push_back(ptr_div_i);
//                 time_buffer.push_back(time_div);
//                 time_div += ptr->points[i].curvature / double(1000);
//                 ptr_div->clear();
//             }
//         }
//         if (!ptr_div->empty())
//         {
//             lidar_buffer.push_back(ptr_div);

//             time_buffer.push_back(time_div);
//         }
//     }
//     else if (con_frame)
//     {
//         if (frame_ct == 0)
//         {

//             time_con = last_timestamp_lidar;
//         }
//         if (frame_ct < con_frame_num)
//         {
//             for (int i = 0; i < ptr->size(); i++)
//             {
//                 ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
//                 ptr_con->push_back(ptr->points[i]);
//             }
//             frame_ct++;
//         }
//         else
//         {
//             PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI());
//             *ptr_con_i = *ptr_con;
//             double time_con_i = time_con;
//             lidar_buffer.push_back(ptr_con_i);
//             time_buffer.push_back(time_con_i);
//             ptr_con->clear();
//             frame_ct = 0;
//         }
//     }
//     else
//     {
//         lidar_buffer.emplace_back(ptr);
//         time_buffer.emplace_back(msg->header.stamp.toSec());
//     }
//     s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
//     mtx_buffer.unlock();
//     sig_buffer.notify_all();
// }

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{

    publish_count++;

    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_lag_imu_to_lidar);

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_ERROR("imu loop back, clear deque");

        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    imu_deque.emplace_back(msg);

    last_timestamp_imu = timestamp;

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}


bool sync_packages(MeasureGroup &meas)
{

    if (!imu_en)
    {
        if (!lidar_buffer.empty())
        {

            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            time_buffer.pop_front();
            lidar_buffer.pop_front();

            if (meas.lidar->points.size() < 1)
            {
                cout << "lose lidar" << std::endl;
                return false;
            }

            double end_time = meas.lidar->points.back().curvature;
            for (auto pt : meas.lidar->points)
            {
                if (pt.curvature > end_time)
                {
                    end_time = pt.curvature;
                }
            }
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);

            meas.lidar_last_time = lidar_end_time;

            return true;
        }
        return false;
    }

    if (lidar_buffer.empty() || imu_deque.empty())
    {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed)
    {

        meas.lidar = lidar_buffer.front();

        if (meas.lidar->points.size() < 1)
        {
            cout << "lose lidar" << endl;
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            return false;
        }

        meas.lidar_beg_time = time_buffer.front();

        double end_time = meas.lidar->points.back().curvature;
        for (auto pt : meas.lidar->points)
        {
            if (pt.curvature > end_time)
            {
                end_time = pt.curvature;
            }
        }
        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);

        meas.lidar_last_time = lidar_end_time;
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    if (p_imu->imu_need_init_)
    {
        double imu_time = imu_deque.front()->header.stamp.toSec();
        meas.imu.shrink_to_fit();
        while ((!imu_deque.empty()) && (imu_time < lidar_end_time))
        {
            imu_time = imu_deque.front()->header.stamp.toSec();
            if (imu_time > lidar_end_time)
                break;
            meas.imu.emplace_back(imu_deque.front());
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    }
    else if (!init_map)
    {
        double imu_time = imu_deque.front()->header.stamp.toSec();
        meas.imu.shrink_to_fit();
        meas.imu.emplace_back(imu_last_ptr);

        while ((!imu_deque.empty()) && (imu_time < lidar_end_time))
        {
            imu_time = imu_deque.front()->header.stamp.toSec();
            if (imu_time > lidar_end_time)
                break;
            meas.imu.emplace_back(imu_deque.front());
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}


int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    for (int i = 0; i < feats_down_size; i++)
    {
        if (!Nearest_Points[i].empty())
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            /* If the nearest points is definitely outside the downsample box */
            if (fabs(points_near[0].x - mid_point.x) > 1.732 * filter_size_map_min || fabs(points_near[0].y - mid_point.y) > 1.732 * filter_size_map_min || fabs(points_near[0].z - mid_point.z) > 1.732 * filter_size_map_min)
            {
                PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
                continue;
            }
            /* Check if there is a point already in the downsample box */
            float dist = calc_dist<float>(feats_down_world->points[i], mid_point);
            for (int readd_i = 0; readd_i < points_near.size(); readd_i++)
            {
                /* Those points which are outside the downsample box should not be considered. */
                if (fabs(points_near[readd_i].x - mid_point.x) < 0.5 * filter_size_map_min && fabs(points_near[readd_i].y - mid_point.y) < 0.5 * filter_size_map_min && fabs(points_near[readd_i].z - mid_point.z) < 0.5 * filter_size_map_min)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                PointToAdd.emplace_back(feats_down_world->points[i]);
        }
        else
        {

            PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
        }
    }
    int add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
}

void publish_init_kdtree(const ros::Publisher &pubLaserCloudFullRes)
{
    int size_init_ikdtree = ikdtree.size();
    PointCloudXYZI::Ptr laserCloudInit(new PointCloudXYZI(size_init_ikdtree, 1));

    sensor_msgs::PointCloud2 laserCloudmsg;
    PointVector().swap(ikdtree.PCL_Storage);
    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);

    laserCloudInit->points = ikdtree.PCL_Storage;
    pcl::toROSMsg(*laserCloudInit, laserCloudmsg);

    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes.publish(laserCloudmsg);
}


PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {

            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);

        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFullRes.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_down_world->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }

        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
        {
            pcd_index++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}


void publish_frame_body(const ros::Publisher &pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        pointBodyLidarToIMU(&feats_undistort->points[i],
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

template <typename T>
void set_posestamp(T &out)
{
    if (!use_imu_as_input)
    {
        out.position.x = kf_output.x_.pos(0);
        out.position.y = kf_output.x_.pos(1);
        out.position.z = kf_output.x_.pos(2);
        out.orientation.x = kf_output.x_.rot.coeffs()[0];
        out.orientation.y = kf_output.x_.rot.coeffs()[1];
        out.orientation.z = kf_output.x_.rot.coeffs()[2];
        out.orientation.w = kf_output.x_.rot.coeffs()[3];
    }
    else
    {
        out.position.x = kf_input.x_.pos(0);
        out.position.y = kf_input.x_.pos(1);
        out.position.z = kf_input.x_.pos(2);
        out.orientation.x = kf_input.x_.rot.coeffs()[0];
        out.orientation.y = kf_input.x_.rot.coeffs()[1];
        out.orientation.z = kf_input.x_.rot.coeffs()[2];
        out.orientation.w = kf_input.x_.rot.coeffs()[3];
    }
}

void publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "aft_mapped";
    if (publish_odometry_without_downsample)
    {
        odomAftMapped.header.stamp = ros::Time().fromSec(time_current);
    }
    else
    {
        odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    }
    set_posestamp(odomAftMapped.pose.pose);

    pubOdomAftMapped.publish(odomAftMapped);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                    odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped"));
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose.pose);

    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";
    static int jjj = 0;
    jjj++;

    {
        path.poses.emplace_back(msg_body_pose);
        pubPath.publish(path);
    }
}

// ----------------- offline binary stream driver -----------------
//
// Replaces ros::Subscriber/spin: reads the binary file produced by
// work-20260528/scripts/extract_to_bin.py and pushes lidar frames /
// imu samples into the same global buffers (lidar_buffer,
// time_buffer, imu_deque) that the original ROS callbacks would have
// populated. Bypasses Preprocess::unilidar_handler entirely — we
// have the (x,y,z,intensity,ring,time) tuples in the file already and
// can construct PointCloudXYZI directly, skipping the ROS PointCloud2
// round-trip and the pcl::fromROSMsg dependency.
//
// See dim/work-20260528/scripts/extract_to_bin.py for the format spec.

#pragma pack(push, 1)
struct BinPoint {
    float x, y, z, _pad_xyz;
    float intensity;
    uint16_t ring;
    uint16_t _pad_ring;
    float time;
    float _pad_time;
};
#pragma pack(pop)
static_assert(sizeof(BinPoint) == 32, "BinPoint must be 32 bytes (L1 unilidar layout)");

static constexpr uint8_t BIN_TYPE_LIDAR = 0;
static constexpr uint8_t BIN_TYPE_IMU   = 1;
static constexpr char    BIN_MAGIC[16]  = {'P','L','N','R','1',0,0,0,0,0,0,0,0,0,0,0};

// Reads one record from the binary stream. Returns:
//   1 = record consumed, buffers may have grown
//   0 = EOF reached cleanly
//  -1 = read error
static int pump_one_record(std::FILE* fp) {
    uint8_t type = 0;
    uint8_t pad[7];
    double ts = 0.0;
    if (std::fread(&type, 1, 1, fp) != 1) return 0;  // EOF
    if (std::fread(pad, 1, 7, fp) != 7) return -1;
    if (std::fread(&ts, 8, 1, fp) != 1) return -1;

    if (type == BIN_TYPE_LIDAR) {
        uint32_t n = 0, pad32 = 0;
        if (std::fread(&n, 4, 1, fp) != 1) return -1;
        if (std::fread(&pad32, 4, 1, fp) != 1) return -1;
        std::vector<BinPoint> pts(n);
        if (n && std::fread(pts.data(), sizeof(BinPoint), n, fp) != n) return -1;

        PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
        cloud->points.reserve(n);
        const double blind2 = p_pre->blind * p_pre->blind;
        const int filter = std::max(1, p_pre->point_filter_num);
        // Curvature is in MILLISECONDS downstream (laserMapping.cpp does
        // `curvature / 1000.0` to convert back to seconds, matching
        // preprocess.cpp's invariant). The previous table here was 1000×
        // too small: SEC→1.0 left curvature in seconds, then /1000.0
        // squashed a full 65 ms sweep into 65 μs and degenerated Point-LIO's
        // per-point predict into a batch-at-frame-boundary update. The L1
        // recording uses timestamp_unit=0 (SEC) so this fix is what reaches
        // the running config; the other rows are kept consistent with
        // preprocess.cpp:51-68.
        const double time_unit_scale =
            (p_pre->time_unit == 0)   ? 1e3 :        // seconds → ms
            (p_pre->time_unit == 1)   ? 1.0 :        // ms → ms
            (p_pre->time_unit == 2)   ? 1e-3 :       // us → ms
            (p_pre->time_unit == 3)   ? 1e-6 : 1.0;  // ns → ms
        for (uint32_t i = 0; i < n; i++) {
            if ((int)(i % filter) != 0) continue;
            const BinPoint& q = pts[i];
            double r2 = (double)q.x * q.x + (double)q.y * q.y + (double)q.z * q.z;
            if (r2 <= blind2) continue;
            PointType p;
            p.normal_x = p.normal_y = p.normal_z = 0;
            p.x = q.x; p.y = q.y; p.z = q.z;
            p.intensity = q.intensity;
            p.curvature = q.time * time_unit_scale;  // see preprocess.cpp::unilidar_handler
            cloud->points.push_back(p);
        }
        if (!cloud->points.empty()) {
            std::lock_guard<std::mutex> lk(mtx_buffer);
            if (ts < last_timestamp_lidar) {
                ROS_ERROR("lidar loop back, clear buffer");
                lidar_buffer.clear();
                time_buffer.clear();
            }
            lidar_buffer.emplace_back(cloud);
            time_buffer.emplace_back(ts);
            last_timestamp_lidar = ts;
        }
        return 1;
    } else if (type == BIN_TYPE_IMU) {
        double v[6];
        if (std::fread(v, 8, 6, fp) != 6) return -1;
        sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu());
        msg->header.stamp = ros::Time::fromSec(ts - time_lag_imu_to_lidar);
        msg->header.frame_id = "unilidar_imu";
        msg->angular_velocity.x = v[0];
        msg->angular_velocity.y = v[1];
        msg->angular_velocity.z = v[2];
        msg->linear_acceleration.x = v[3];
        msg->linear_acceleration.y = v[4];
        msg->linear_acceleration.z = v[5];
        double ts_aligned = msg->header.stamp.toSec();
        std::lock_guard<std::mutex> lk(mtx_buffer);
        if (ts_aligned < last_timestamp_imu) {
            // Same behaviour as the original imu_cbk on time monotonicity violation.
            return 1;
        }
        imu_deque.emplace_back(msg);
        last_timestamp_imu = ts_aligned;
        return 1;
    }
    ROS_ERROR("unknown record type %u in binary stream", (unsigned)type);
    return -1;
}

int main(int argc, char **argv)
{
    // --yaml <unilidar_l1.yaml>  --bin <recording.bin>
    std::string yaml_path;
    std::string bin_path;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--yaml" && i + 1 < argc) yaml_path = argv[++i];
        else if (a == "--bin" && i + 1 < argc) bin_path = argv[++i];
    }
    if (bin_path.empty()) {
        std::fprintf(stderr,
            "usage: %s --bin <recording.bin> [--yaml <unilidar_l1.yaml>]\n",
            argv[0]);
        return 2;
    }
    readParametersYaml(yaml_path);
    cout << "lidar_type: " << lidar_type << endl;

    path.header.stamp = ros::Time().fromSec(lidar_end_time);
    path.header.frame_id = "camera_init";

    int frame_num = 0;
    double aver_time_consu = 0,
           aver_time_icp = 0,
           aver_time_match = 0,
           aver_time_incre = 0,
           aver_time_solve = 0,
           aver_time_propag = 0;
    std::time_t startTime, endTime;

    double FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    double HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    memset(point_selected_surf, true, sizeof(point_selected_surf));

    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

    if (extrinsic_est_en)
    {

        if (!use_imu_as_input)
        {
            kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }

        else
        {
            kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
    }

    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;

    kf_input.init_dyn_share_modified(get_f_input, df_dx_input, h_model_input);
    kf_output.init_dyn_share_modified_2h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);

    Eigen::Matrix<double, 24, 24> P_init = MD(24, 24)::Identity() * 0.01;
    P_init.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    P_init.block<6, 6>(15, 15) = MD(6, 6)::Identity() * 0.001;
    P_init.block<6, 6>(6, 6) = MD(6, 6)::Identity() * 0.0001;
    kf_input.change_P(P_init);

    Eigen::Matrix<double, 30, 30> P_init_output = MD(30, 30)::Identity() * 0.01;
    P_init_output.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    P_init_output.block<6, 6>(6, 6) = MD(6, 6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(24, 24) = MD(6, 6)::Identity() * 0.001;
    kf_input.change_P(P_init);
    kf_output.change_P(P_init_output);

    Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
    Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(), "w");

    ofstream fout_out, fout_imu_pbp;
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
    fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"), ios::out);
    if (fout_out && fout_imu_pbp)
        cout << "~~~~" << ROOT_DIR << " file opened" << endl;
    else
        cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

    // Subscriber / Publisher tags — no-ops in the offline build, but the
    // publishers are passed by reference into helpers like
    // publish_frame_world(pubLaserCloudFullRes) so we still need lvalues
    // with the right type. Construct them as default-init tags.
    ros::Subscriber sub_pcl, sub_imu;
    ros::Publisher pubLaserCloudFullRes;
    ros::Publisher pubLaserCloudFullRes_body;
    ros::Publisher pubLaserCloudEffect;
    ros::Publisher pubLaserCloudMap;
    ros::Publisher pubOdomAftMapped;
    ros::Publisher pubPath;
    ros::Publisher plane_pub;

    signal(SIGINT, SigHandle);

    std::FILE* fp_bin = std::fopen(bin_path.c_str(), "rb");
    if (!fp_bin) {
        std::fprintf(stderr, "could not open %s: %s\n", bin_path.c_str(), strerror(errno));
        return 1;
    }
    char magic[16];
    if (std::fread(magic, 1, 16, fp_bin) != 16 || std::memcmp(magic, BIN_MAGIC, 16) != 0) {
        std::fprintf(stderr, "%s: bad magic, not a PLNR1 binary stream\n", bin_path.c_str());
        return 1;
    }
    cout << "reading binary stream from " << bin_path << endl;
    bool eof_seen = false;

    while (ros::ok())
    {

        if (flg_exit)
            break;

        // Replaces ros::spinOnce(): pump records until sync_packages has
        // enough data or we hit EOF.
        if (!eof_seen) {
            int rc = pump_one_record(fp_bin);
            if (rc == 0) {
                eof_seen = true;
                cout << "binary stream EOF" << endl;
            } else if (rc < 0) {
                std::fprintf(stderr, "binary stream read error\n");
                flg_exit = true;
                break;
            }
        }

        if (sync_packages(Measures) == false)
        {
            // No rate limit in offline mode — let the pump run as fast
            // as the algorithm can consume.
            if (eof_seen && lidar_buffer.empty()) {
                flg_exit = true;
            }
            continue;
        }

        if (flg_first_scan)
        {
            first_lidar_time = Measures.lidar_beg_time;
            flg_first_scan = false;
            cout << "first lidar time" << first_lidar_time << endl;
        }

        if (flg_reset)
        {
            ROS_WARN("reset when rosbag play back");
            p_imu->Reset();
            flg_reset = false;
            continue;
        }

        double t0, t1, t2, t3, t4, t5, match_start, solve_start;
        match_time = 0;
        solve_time = 0;
        propag_time = 0;
        update_time = 0;
        t0 = omp_get_wtime();

        p_imu->Process(Measures, feats_undistort);

        if (feats_undistort->empty() || feats_undistort == NULL)
        {
            continue;
        }

        if (imu_en)
        {
            if (!p_imu->gravity_align_)
            {
                while (Measures.lidar_beg_time > imu_next.header.stamp.toSec())
                {
                    imu_last = imu_next;
                    imu_next = *(imu_deque.front());
                    imu_deque.pop_front();
                }
                if (non_station_start)
                {
                    state_in.gravity << VEC_FROM_ARRAY(gravity_init);
                    state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc *= -1;
                }
                else
                {
                    state_in.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm;
                    state_out.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm;
                    state_out.acc = p_imu->mean_acc * G_m_s2 / acc_norm;
                }
                if (gravity_align)
                {
                    Eigen::Matrix3d rot_init;
                    p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
                    p_imu->Set_init(state_in.gravity, rot_init);
                    state_in.gravity = state_out.gravity = p_imu->gravity_;
                    state_in.rot = state_out.rot = rot_init;
                    state_in.rot.normalize();
                    state_out.rot.normalize();
                    state_out.acc = -rot_init.transpose() * state_out.gravity;
                }
                kf_input.change_x(state_in);
                kf_output.change_x(state_out);
            }
        }
        else
        {
            if (!p_imu->gravity_align_)
            {
                state_in.gravity << VEC_FROM_ARRAY(gravity_init);
                state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                state_out.acc << VEC_FROM_ARRAY(gravity_init);
                state_out.acc *= -1;
            }
        }

        /*** Segment the map in lidar FOV ***/
        lasermap_fov_segment();

        t1 = omp_get_wtime();
        if (space_down_sample)
        {
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
        }
        else
        {
            feats_down_body = Measures.lidar;

            sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
        }
        time_seq = time_compressing<int>(feats_down_body);
        feats_down_size = feats_down_body->points.size();

        /*** initialize the map kdtree ***/
        if (!init_map)
        {
            if (ikdtree.Root_Node == nullptr)

            {
                ikdtree.set_downsample_param(filter_size_map_min);
            }

            feats_down_world->resize(feats_down_size);
            for (int i = 0; i < feats_down_size; i++)
            {
                pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
            }

            for (size_t i = 0; i < feats_down_world->size(); i++)
            {
                init_feats_world->points.emplace_back(feats_down_world->points[i]);
            }

            if (init_feats_world->size() < init_map_size)
                continue;

            ikdtree.Build(init_feats_world->points);
            init_map = true;

            publish_init_kdtree(pubLaserCloudMap);
            continue;
        }

        /*** ICP and Kalman filter update ***/

        normvec->resize(feats_down_size);
        feats_down_world->resize(feats_down_size);

        Nearest_Points.resize(feats_down_size);

        t2 = omp_get_wtime();

        /*** iterated state estimation ***/

        crossmat_list.reserve(feats_down_size);
        pbody_list.reserve(feats_down_size);

        for (size_t i = 0; i < feats_down_body->size(); i++)
        {

            V3D point_this(feats_down_body->points[i].x,
                           feats_down_body->points[i].y,
                           feats_down_body->points[i].z);
            pbody_list[i] = point_this;

            if (extrinsic_est_en)
            {
                if (!use_imu_as_input)
                {

                    point_this = kf_output.x_.offset_R_L_I.normalized() * point_this + kf_output.x_.offset_T_L_I;
                }
                else
                {

                    point_this = kf_input.x_.offset_R_L_I.normalized() * point_this + kf_input.x_.offset_T_L_I;
                }
            }
            else
            {
                point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
            }


            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRX(point_this);
            crossmat_list[i]=point_crossmat;
        }

        if (!use_imu_as_input)
        {
            bool imu_upda_cov = false;
            effct_feat_num = 0;

            /**** point by point update ****/

            double pcl_beg_time = Measures.lidar_beg_time;
            idx = -1;
            for (k = 0; k < time_seq.size(); k++)
            {

                PointType &point_body = feats_down_body->points[idx + time_seq[k]];

                time_current = point_body.curvature / 1000.0 + pcl_beg_time;

                if (is_first_frame)
                {
                    if (imu_en)
                    {
                        while (time_current > imu_next.header.stamp.toSec())
                        {
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_deque.pop_front();
                        }

                        angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                        acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                    }
                    is_first_frame = false;
                    imu_upda_cov = true;
                    time_update_last = time_current;
                    time_predict_last_const = time_current;
                }

                if (imu_en)
                {
                    bool imu_comes = time_current > imu_next.header.stamp.toSec();
                    while (imu_comes)
                    {
                        imu_upda_cov = true;
                        angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                        acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;

                        /*** covariance update ***/
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                        double dt = imu_last.header.stamp.toSec() - time_predict_last_const;
                        kf_output.predict(dt, Q_output, input_in, true, false);
                        time_predict_last_const = imu_last.header.stamp.toSec();
                        imu_comes = time_current > imu_next.header.stamp.toSec();

                        {
                            double dt_cov = imu_last.header.stamp.toSec() - time_update_last;

                            if (dt_cov > 0.0)
                            {
                                time_update_last = imu_last.header.stamp.toSec();
                                double propag_imu_start = omp_get_wtime();

                                kf_output.predict(dt_cov, Q_output, input_in, false, true);

                                propag_time += omp_get_wtime() - propag_imu_start;
                                double solve_imu_start = omp_get_wtime();
                                kf_output.update_iterated_dyn_share_IMU();
                                solve_time += omp_get_wtime() - solve_imu_start;
                            }
                        }
                    }
                }

                double dt = time_current - time_predict_last_const;
                double propag_state_start = omp_get_wtime();
                if (!prop_at_freq_of_imu)
                {
                    double dt_cov = time_current - time_update_last;
                    if (dt_cov > 0.0)
                    {
                        kf_output.predict(dt_cov, Q_output, input_in, false, true);
                        time_update_last = time_current;
                    }
                }
                kf_output.predict(dt, Q_output, input_in, true, false);
                propag_time += omp_get_wtime() - propag_state_start;
                time_predict_last_const = time_current;

                double t_update_start = omp_get_wtime();

                if (feats_down_size < 1)
                {
                    ROS_WARN("No point, skip this scan!\n");
                    idx += time_seq[k];
                    continue;
                }
                if (!kf_output.update_iterated_dyn_share_modified())
                {
                    idx = idx + time_seq[k];
                    continue;
                }

                if (prop_at_freq_of_imu)
                {
                    double dt_cov = time_current - time_update_last;
                    if (!imu_en && (dt_cov >= imu_time_inte))
                    {
                        double propag_cov_start = omp_get_wtime();
                        kf_output.predict(dt_cov, Q_output, input_in, false, true);
                        imu_upda_cov = false;
                        time_update_last = time_current;
                        propag_time += omp_get_wtime() - propag_cov_start;
                    }
                }

                solve_start = omp_get_wtime();

                if (publish_odometry_without_downsample)
                {
                    /******* Publish odometry *******/

                    publish_odometry(pubOdomAftMapped);
                    if (runtime_pos_log)
                    {
                        state_out = kf_output.x_;
                        euler_cur = SO3ToEuler(state_out.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_out.pos.transpose() << " " << state_out.vel.transpose()
                                 << " " << state_out.omg.transpose() << " " << state_out.acc.transpose() << " " << state_out.gravity.transpose() << " " << state_out.bg.transpose() << " " << state_out.ba.transpose() << " " << feats_undistort->points.size() << endl;
                    }
                }

                for (int j = 0; j < time_seq[k]; j++)
                {
                    PointType &point_body_j = feats_down_body->points[idx + j + 1];
                    PointType &point_world_j = feats_down_world->points[idx + j + 1];
                    pointBodyToWorld(&point_body_j, &point_world_j);
                }

                solve_time += omp_get_wtime() - solve_start;

                update_time += omp_get_wtime() - t_update_start;
                idx += time_seq[k];
            }
        }
        else
        {
            bool imu_prop_cov = false;
            effct_feat_num = 0;

            double pcl_beg_time = Measures.lidar_beg_time;
            idx = -1;
            for (k = 0; k < time_seq.size(); k++)
            {
                PointType &point_body = feats_down_body->points[idx + time_seq[k]];
                time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                if (is_first_frame)
                {
                    while (time_current > imu_next.header.stamp.toSec())
                    {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                    }
                    imu_prop_cov = true;

                    is_first_frame = false;
                    t_last = time_current;
                    time_update_last = time_current;

                    {
                        input_in.gyro << imu_last.angular_velocity.x,
                            imu_last.angular_velocity.y,
                            imu_last.angular_velocity.z;

                        input_in.acc << imu_last.linear_acceleration.x,
                            imu_last.linear_acceleration.y,
                            imu_last.linear_acceleration.z;

                        input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                    }
                }

                while (time_current > imu_next.header.stamp.toSec())
                {
                    imu_last = imu_next;
                    imu_next = *(imu_deque.front());
                    imu_deque.pop_front();
                    input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                    input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;

                    input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                    double dt = imu_last.header.stamp.toSec() - t_last;

                    double dt_cov = imu_last.header.stamp.toSec() - time_update_last;
                    if (dt_cov > 0.0)
                    {
                        kf_input.predict(dt_cov, Q_input, input_in, false, true);
                        time_update_last = imu_last.header.stamp.toSec();
                    }
                    kf_input.predict(dt, Q_input, input_in, true, false);
                    t_last = imu_last.header.stamp.toSec();
                    imu_prop_cov = true;
                }

                double dt = time_current - t_last;
                t_last = time_current;
                double propag_start = omp_get_wtime();

                if (!prop_at_freq_of_imu)
                {
                    double dt_cov = time_current - time_update_last;
                    if (dt_cov > 0.0)
                    {
                        kf_input.predict(dt_cov, Q_input, input_in, false, true);
                        time_update_last = time_current;
                    }
                }
                kf_input.predict(dt, Q_input, input_in, true, false);

                propag_time += omp_get_wtime() - propag_start;

                double t_update_start = omp_get_wtime();

                if (feats_down_size < 1)
                {
                    ROS_WARN("No point, skip this scan!\n");

                    idx += time_seq[k];
                    continue;
                }
                if (!kf_input.update_iterated_dyn_share_modified())
                {
                    idx = idx + time_seq[k];
                    continue;
                }

                solve_start = omp_get_wtime();

                if (publish_odometry_without_downsample)
                {
                    /******* Publish odometry *******/

                    publish_odometry(pubOdomAftMapped);
                    if (runtime_pos_log)
                    {
                        state_in = kf_input.x_;
                        euler_cur = SO3ToEuler(state_in.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_in.pos.transpose() << " " << state_in.vel.transpose()
                                 << " " << state_in.bg.transpose() << " " << state_in.ba.transpose() << " " << state_in.gravity.transpose() << " " << feats_undistort->points.size() << endl;
                    }
                }

                for (int j = 0; j < time_seq[k]; j++)
                {
                    PointType &point_body_j = feats_down_body->points[idx + j + 1];
                    PointType &point_world_j = feats_down_world->points[idx + j + 1];
                    pointBodyToWorld(&point_body_j, &point_world_j);
                }
                solve_time += omp_get_wtime() - solve_start;

                update_time += omp_get_wtime() - t_update_start;
                idx = idx + time_seq[k];
            }
        }

        /******* Publish odometry downsample *******/
        if (!publish_odometry_without_downsample)
        {
            publish_odometry(pubOdomAftMapped);
        }

        /*** add the feature points to map kdtree ***/
        t3 = omp_get_wtime();

        if (feats_down_size > 4)
        {
            map_incremental();
        }

        t5 = omp_get_wtime();

        /******* Publish points *******/

        if (path_en)
            publish_path(pubPath);

        if (scan_pub_en || pcd_save_en)
            publish_frame_world(pubLaserCloudFullRes);

        if (scan_pub_en && scan_body_pub_en)
            publish_frame_body(pubLaserCloudFullRes_body);

        /*** Debug variables Logging ***/
        if (runtime_pos_log)
        {
            frame_num++;
            aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
            {
                aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num;
            }
            aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
            aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
            aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;
            T1[time_log_counter] = Measures.lidar_beg_time;
            s_plot[time_log_counter] = t5 - t0;
            s_plot2[time_log_counter] = feats_undistort->points.size();
            s_plot3[time_log_counter] = aver_time_consu;
            time_log_counter++;
            printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n", t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp, aver_time_propag);
            if (!publish_odometry_without_downsample)
            {
                if (!use_imu_as_input)
                {
                    state_out = kf_output.x_;
                    euler_cur = SO3ToEuler(state_out.rot);
                    fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_out.pos.transpose() << " " << state_out.vel.transpose()
                             << " " << state_out.omg.transpose() << " " << state_out.acc.transpose() << " " << state_out.gravity.transpose() << " " << state_out.bg.transpose() << " " << state_out.ba.transpose() << " " << feats_undistort->points.size() << endl;
                }
                else
                {
                    state_in = kf_input.x_;
                    euler_cur = SO3ToEuler(state_in.rot);
                    fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_in.pos.transpose() << " " << state_in.vel.transpose()
                             << " " << state_in.bg.transpose() << " " << state_in.ba.transpose() << " " << state_in.gravity.transpose() << " " << feats_undistort->points.size() << endl;
                }
            }
            dump_lio_state_to_log(fp);
        }

        // (offline build: no rate limit — run as fast as the algorithm
        // can consume records from the binary stream)
    }

    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/

    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        std::cout << "Saving map to file: " << all_points_dir << std::endl;
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
    fout_out.close();
    fout_imu_pbp.close();

    return 0;
}
