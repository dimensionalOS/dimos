// Terrain Analysis Ext — dimos NativeModule port
// Ported from ROS2: src/base_autonomy/terrain_analysis_ext/src/terrainAnalysisExt.cpp
//
// Accumulates registered_scan into a rolling 41x41 terrain voxel grid (2m cells),
// estimates ground elevation per 101x101 planar grid (0.4m cells), runs BFS
// connectivity check, and publishes filtered terrain_map_ext with intensity =
// elevation distance from ground. Merges local terrain_map within localTerrainMapRadius.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <chrono>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <lcm/lcm-cpp.hpp>

#include "dimos_native_module.hpp"
#include "point_cloud_utils.hpp"

#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/PointCloud2.hpp"

#ifdef USE_PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#endif

using namespace std;

const double PI = 3.1415926;

// --- Configuration (populated from CLI) ---
double scanVoxelSize = 0.1;
double decayTime = 10.0;
double noDecayDis = 0;
double clearingDis = 30.0;
bool clearingCloud = false;
bool useSorting = false;
double quantileZ = 0.25;
double vehicleHeight = 1.5;
int voxelPointUpdateThre = 100;
double voxelTimeUpdateThre = 2.0;
double lowerBoundZ = -1.5;
double upperBoundZ = 1.0;
double disRatioZ = 0.1;
bool checkTerrainConn = true;
double terrainUnderVehicle = -0.75;
double terrainConnThre = 0.5;
double ceilingFilteringThre = 2.0;
double localTerrainMapRadius = 4.0;
bool mergeLocalTerrain = true;

// --- Terrain voxel grid parameters ---
float terrainVoxelSize = 2.0;
int terrainVoxelShiftX = 0;
int terrainVoxelShiftY = 0;
const int terrainVoxelWidth = 41;
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;

// --- Planar voxel parameters ---
float planarVoxelSize = 0.4;
const int planarVoxelWidth = 101;
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth;

// --- State ---
#ifdef USE_PCL
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudLocal(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[terrainVoxelNum];

int terrainVoxelUpdateNum[terrainVoxelNum] = {0};
float terrainVoxelUpdateTime[terrainVoxelNum] = {0};
float planarVoxelElev[planarVoxelNum] = {0};
int planarVoxelConn[planarVoxelNum] = {0};
vector<float> planarPointElev[planarVoxelNum];
queue<int> planarVoxelQueue;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
#endif

double laserCloudTime = 0;
bool newlaserCloud = false;
double systemInitTime = 0;
bool systemInited = false;

float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

// --- Callbacks ---
void odometryHandler(const lcm::ReceiveBuffer*, const std::string&,
                     const nav_msgs::Odometry* msg) {
    double roll, pitch, yaw;
    smartnav::quat_to_rpy(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w,
        roll, pitch, yaw);
    vehicleX = msg->pose.pose.position.x;
    vehicleY = msg->pose.pose.position.y;
    vehicleZ = msg->pose.pose.position.z;
}

void laserCloudHandler(const lcm::ReceiveBuffer*, const std::string&,
                       const sensor_msgs::PointCloud2* msg) {
    laserCloudTime = smartnav::get_timestamp(*msg);

    if (!systemInited) {
        systemInitTime = laserCloudTime;
        systemInited = true;
    }

#ifdef USE_PCL
    laserCloud->clear();
    smartnav::to_pcl(*msg, *laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
        point = laserCloud->points[i];
        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                         (point.y - vehicleY) * (point.y - vehicleY));
        if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
            point.z - vehicleZ < upperBoundZ + disRatioZ * dis &&
            dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {
            point.intensity = laserCloudTime - systemInitTime;
            laserCloudCrop->push_back(point);
        }
    }
#endif

    newlaserCloud = true;
}

void terrainCloudLocalHandler(const lcm::ReceiveBuffer*, const std::string&,
                              const sensor_msgs::PointCloud2* msg) {
#ifdef USE_PCL
    terrainCloudLocal->clear();
    smartnav::to_pcl(*msg, *terrainCloudLocal);
#endif
}

int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    // Parse config from CLI
    scanVoxelSize = mod.arg_float("scanVoxelSize", scanVoxelSize);
    decayTime = mod.arg_float("decayTime", decayTime);
    noDecayDis = mod.arg_float("noDecayDis", noDecayDis);
    clearingDis = mod.arg_float("clearingDis", clearingDis);
    useSorting = mod.arg_bool("useSorting", useSorting);
    quantileZ = mod.arg_float("quantileZ", quantileZ);
    vehicleHeight = mod.arg_float("vehicleHeight", vehicleHeight);
    voxelPointUpdateThre = mod.arg_int("voxelPointUpdateThre", voxelPointUpdateThre);
    voxelTimeUpdateThre = mod.arg_float("voxelTimeUpdateThre", voxelTimeUpdateThre);
    lowerBoundZ = mod.arg_float("lowerBoundZ", lowerBoundZ);
    upperBoundZ = mod.arg_float("upperBoundZ", upperBoundZ);
    disRatioZ = mod.arg_float("disRatioZ", disRatioZ);
    checkTerrainConn = mod.arg_bool("checkTerrainConn", checkTerrainConn);
    terrainUnderVehicle = mod.arg_float("terrainUnderVehicle", terrainUnderVehicle);
    terrainConnThre = mod.arg_float("terrainConnThre", terrainConnThre);
    ceilingFilteringThre = mod.arg_float("ceilingFilteringThre", ceilingFilteringThre);
    localTerrainMapRadius = mod.arg_float("localTerrainMapRadius", localTerrainMapRadius);
    mergeLocalTerrain = mod.arg_bool("mergeLocalTerrain", mergeLocalTerrain);

    auto lcm = std::make_shared<lcm::LCM>();
    if (!lcm->good()) {
        fprintf(stderr, "terrain_map_ext: LCM init failed\n");
        return 1;
    }

    lcm->subscribe(mod.topic("registered_scan"), &laserCloudHandler);
    lcm->subscribe(mod.topic("odometry"), &odometryHandler);
    lcm->subscribe(mod.topic("terrain_map"), &terrainCloudLocalHandler);

    std::string topic_out = mod.topic("terrain_map_ext");

#ifdef USE_PCL
    for (int i = 0; i < terrainVoxelNum; i++) {
        terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);
#endif

    // Main loop at 100Hz
    while (true) {
        lcm->handleTimeout(10);

        if (!newlaserCloud) continue;
        newlaserCloud = false;

#ifdef USE_PCL
        // --- Terrain voxel roll over ---
        float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
        float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;

        while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
            for (int indY = 0; indY < terrainVoxelWidth; indY++) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr ptr =
                    terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY];
                for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--)
                    terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                        terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
                terrainVoxelCloud[indY] = ptr;
                terrainVoxelCloud[indY]->clear();
            }
            terrainVoxelShiftX--;
            terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
        }
        while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
            for (int indY = 0; indY < terrainVoxelWidth; indY++) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr ptr = terrainVoxelCloud[indY];
                for (int indX = 0; indX < terrainVoxelWidth - 1; indX++)
                    terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                        terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
                terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY] = ptr;
                terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]->clear();
            }
            terrainVoxelShiftX++;
            terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
        }
        while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) {
            for (int indX = 0; indX < terrainVoxelWidth; indX++) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr ptr =
                    terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)];
                for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--)
                    terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                        terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
                terrainVoxelCloud[terrainVoxelWidth * indX] = ptr;
                terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
            }
            terrainVoxelShiftY--;
            terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
        }
        while (vehicleY - terrainVoxelCenY > terrainVoxelSize) {
            for (int indX = 0; indX < terrainVoxelWidth; indX++) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr ptr =
                    terrainVoxelCloud[terrainVoxelWidth * indX];
                for (int indY = 0; indY < terrainVoxelWidth - 1; indY++)
                    terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                        terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
                terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)] = ptr;
                terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]->clear();
            }
            terrainVoxelShiftY++;
            terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
        }

        // --- Stack registered laser scans ---
        pcl::PointXYZI point;
        int laserCloudCropSize = laserCloudCrop->points.size();
        for (int i = 0; i < laserCloudCropSize; i++) {
            point = laserCloudCrop->points[i];
            int indX = int((point.x - vehicleX + terrainVoxelSize / 2) / terrainVoxelSize) +
                       terrainVoxelHalfWidth;
            int indY = int((point.y - vehicleY + terrainVoxelSize / 2) / terrainVoxelSize) +
                       terrainVoxelHalfWidth;
            if (point.x - vehicleX + terrainVoxelSize / 2 < 0) indX--;
            if (point.y - vehicleY + terrainVoxelSize / 2 < 0) indY--;

            if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 && indY < terrainVoxelWidth) {
                terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
                terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
            }
        }

        // --- Downsample / evict ---
        for (int ind = 0; ind < terrainVoxelNum; ind++) {
            if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
                laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >=
                    voxelTimeUpdateThre ||
                clearingCloud) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr = terrainVoxelCloud[ind];

                laserCloudDwz->clear();
                downSizeFilter.setInputCloud(cloudPtr);
                downSizeFilter.filter(*laserCloudDwz);

                cloudPtr->clear();
                int dwzSize = laserCloudDwz->points.size();
                for (int i = 0; i < dwzSize; i++) {
                    point = laserCloudDwz->points[i];
                    float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                                     (point.y - vehicleY) * (point.y - vehicleY));
                    if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
                        point.z - vehicleZ < upperBoundZ + disRatioZ * dis &&
                        (laserCloudTime - systemInitTime - point.intensity < decayTime ||
                         dis < noDecayDis) &&
                        !(dis < clearingDis && clearingCloud)) {
                        cloudPtr->push_back(point);
                    }
                }
                terrainVoxelUpdateNum[ind] = 0;
                terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
            }
        }

        // --- Gather terrain cloud from central 21x21 ---
        terrainCloud->clear();
        for (int indX = terrainVoxelHalfWidth - 10; indX <= terrainVoxelHalfWidth + 10; indX++)
            for (int indY = terrainVoxelHalfWidth - 10; indY <= terrainVoxelHalfWidth + 10; indY++)
                *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];

        // --- Ground elevation estimation ---
        for (int i = 0; i < planarVoxelNum; i++) {
            planarVoxelElev[i] = 0;
            planarVoxelConn[i] = 0;
            planarPointElev[i].clear();
        }

        int terrainCloudSize = terrainCloud->points.size();
        for (int i = 0; i < terrainCloudSize; i++) {
            point = terrainCloud->points[i];
            float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                             (point.y - vehicleY) * (point.y - vehicleY));
            if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
                point.z - vehicleZ < upperBoundZ + disRatioZ * dis) {
                int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
                           planarVoxelHalfWidth;
                int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
                           planarVoxelHalfWidth;
                if (point.x - vehicleX + planarVoxelSize / 2 < 0) indX--;
                if (point.y - vehicleY + planarVoxelSize / 2 < 0) indY--;

                for (int dX = -1; dX <= 1; dX++)
                    for (int dY = -1; dY <= 1; dY++)
                        if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                            indY + dY >= 0 && indY + dY < planarVoxelWidth)
                            planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY].push_back(
                                point.z);
            }
        }

        if (useSorting) {
            for (int i = 0; i < planarVoxelNum; i++) {
                int n = planarPointElev[i].size();
                if (n > 0) {
                    sort(planarPointElev[i].begin(), planarPointElev[i].end());
                    int qid = int(quantileZ * n);
                    if (qid < 0) qid = 0;
                    else if (qid >= n) qid = n - 1;
                    planarVoxelElev[i] = planarPointElev[i][qid];
                }
            }
        } else {
            for (int i = 0; i < planarVoxelNum; i++) {
                int n = planarPointElev[i].size();
                if (n > 0) {
                    float minZ = 1000.0;
                    for (int j = 0; j < n; j++)
                        if (planarPointElev[i][j] < minZ) minZ = planarPointElev[i][j];
                    planarVoxelElev[i] = minZ;
                }
            }
        }

        // --- BFS connectivity check ---
        if (checkTerrainConn) {
            int ind = planarVoxelWidth * planarVoxelHalfWidth + planarVoxelHalfWidth;
            if (planarPointElev[ind].size() == 0)
                planarVoxelElev[ind] = vehicleZ + terrainUnderVehicle;

            planarVoxelQueue.push(ind);
            planarVoxelConn[ind] = 1;
            while (!planarVoxelQueue.empty()) {
                int front = planarVoxelQueue.front();
                planarVoxelConn[front] = 2;
                planarVoxelQueue.pop();

                int fX = int(front / planarVoxelWidth);
                int fY = front % planarVoxelWidth;
                for (int dX = -10; dX <= 10; dX++) {
                    for (int dY = -10; dY <= 10; dY++) {
                        if (fX + dX >= 0 && fX + dX < planarVoxelWidth && fY + dY >= 0 &&
                            fY + dY < planarVoxelWidth) {
                            ind = planarVoxelWidth * (fX + dX) + fY + dY;
                            if (planarVoxelConn[ind] == 0 && planarPointElev[ind].size() > 0) {
                                if (fabs(planarVoxelElev[front] - planarVoxelElev[ind]) <
                                    terrainConnThre) {
                                    planarVoxelQueue.push(ind);
                                    planarVoxelConn[ind] = 1;
                                } else if (fabs(planarVoxelElev[front] - planarVoxelElev[ind]) >
                                           ceilingFilteringThre) {
                                    planarVoxelConn[ind] = -1;
                                }
                            }
                        }
                    }
                }
            }
        }

        // --- Build output: beyond localTerrainMapRadius with ground/connectivity filter ---
        terrainCloudElev->clear();
        int terrainCloudElevSize = 0;
        for (int i = 0; i < terrainCloudSize; i++) {
            point = terrainCloud->points[i];
            float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                             (point.y - vehicleY) * (point.y - vehicleY));
            if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis &&
                point.z - vehicleZ < upperBoundZ + disRatioZ * dis &&
                dis > localTerrainMapRadius) {
                int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
                           planarVoxelHalfWidth;
                int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
                           planarVoxelHalfWidth;
                if (point.x - vehicleX + planarVoxelSize / 2 < 0) indX--;
                if (point.y - vehicleY + planarVoxelSize / 2 < 0) indY--;

                if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
                    indY < planarVoxelWidth) {
                    int ind = planarVoxelWidth * indX + indY;
                    float disZ = fabs(point.z - planarVoxelElev[ind]);
                    if (disZ < vehicleHeight &&
                        (planarVoxelConn[ind] == 2 || !checkTerrainConn)) {
                        terrainCloudElev->push_back(point);
                        terrainCloudElev->points[terrainCloudElevSize].x = point.x;
                        terrainCloudElev->points[terrainCloudElevSize].y = point.y;
                        terrainCloudElev->points[terrainCloudElevSize].z = point.z;
                        terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;
                        terrainCloudElevSize++;
                    }
                }
            }
        }

        // --- Merge local terrain map within radius ---
        if (mergeLocalTerrain) {
            int localSize = terrainCloudLocal->points.size();
            for (int i = 0; i < localSize; i++) {
                point = terrainCloudLocal->points[i];
                float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                                 (point.y - vehicleY) * (point.y - vehicleY));
                if (dis <= localTerrainMapRadius) {
                    terrainCloudElev->push_back(point);
                }
            }
        }

        clearingCloud = false;

        // --- Publish ---
        auto outMsg = smartnav::from_pcl(*terrainCloudElev, "map", laserCloudTime);
        lcm->publish(topic_out, &outMsg);
#endif  // USE_PCL
    }

    return 0;
}
