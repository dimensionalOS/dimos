// The vendor toolchain, to check src/dock against: docker run -it ubuntu:20.04 (arm64), apt-get install g++ libpcl-dev
// (g++ 9.4.0-1ubuntu1~20.04.2, PCL 1.10, glibc 2.31: reflective_column_node's own).
// reflective_column_node's ProcessPointCloudData rebuilt from its decompile on the real PCL and libm:
//   g++ -O2 -ffp-contract=off -std=c++14 -I/usr/include/pcl-1.10 -I/usr/include/eigen3 reflective_column_harness.cpp \
//     -lpcl_filters -lpcl_segmentation -lpcl_search -lpcl_kdtree -lpcl_common -lflann_cpp -lboost_system
// Input: sweeps of u32 n + n x f32 (x, y, z, intensity) in the front lidar's frame; output: a line per sweep with every
// logged value at full precision (argv[2] = a sweep index prints its transformed, voxel and outlier-removal clouds).
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <map>
#include <vector>
#include <cstring>
#include <cstdlib>

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

static Eigen::Vector4f centroid(const Cloud &c) {
  float s[3] = {0, 0, 0};
  for (auto &p : c.points) { s[0] = s[0] + p.x; s[1] = s[1] + p.y; s[2] = s[2] + p.z; }
  float n = (float)c.points.size();
  return Eigen::Vector4f(s[0] / n, s[1] / n, s[2] / n, 1.0f);
}

static unsigned bits(float f) { unsigned u; memcpy(&u, &f, 4); return u; }
int main(int argc, char **argv) {
  FILE *in = fopen(argv[1], "rb");
  int want = argc > 2 ? atoi(argv[2]) : -1, k = -1;
  // the driver's transform (rs_driver ENABLE_TRANSFORM, RSTransformParam floats)
  float tx = 0.32028f, ty = 0.0f, tz = -0.013f, roll = 0.0f, pitch = -1.57079f, yaw = -3.14159f;
  Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX()), ry(pitch, Eigen::Vector3d::UnitY()), rz(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Translation3d tr(tx, ty, tz);
  Eigen::Matrix4d trans = (tr * rz * ry * rx).matrix();
  uint32_t n;
  while (fread(&n, 4, 1, in) == 1) {
    std::vector<float> raw(4 * (size_t)n);
    if (n && fread(raw.data(), 4, raw.size(), in) != raw.size()) break;
    k++;
    Cloud::Ptr cloud(new Cloud);
    for (uint32_t i = 0; i < n; i++) {
      Eigen::Vector4d o(raw[4 * i], raw[4 * i + 1], raw[4 * i + 2], 1);
      Eigen::Vector4d r = trans * o;
      float x = r(0), y = r(1), z = r(2), in_ = raw[4 * i + 3];
      if (k == want) printf("T %08x %08x %08x\n", bits(x), bits(y), bits(z));
      if ((float)230 < in_ && 0.4 < (double)x && x < 4.0f && fabsf(y) < 2.0f && -1.1 < (double)z && (double)z < 0.1 &&
          fabs((double)atan2f(y, x)) <= 0.7853981633974483) {
        pcl::PointXYZ p; p.x = x; p.y = y; p.z = z; cloud->points.push_back(p);
      }
    }
    cloud->width = cloud->points.size(); cloud->height = 1;
    {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(0.01f, 0.01f, 0.01f);
      vg.filter(*cloud);
    }
    if (k == want) for (auto &p : cloud->points) printf("V %08x %08x %08x\n", bits(p.x), bits(p.y), bits(p.z));
    if (cloud->points.size() * 16 > 0x13f) {
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
      ror.setInputCloud(cloud);
      ror.setRadiusSearch((double)0.3f);
      ror.setMinNeighborsInRadius(15);
      ror.filter(*cloud);
    }
    if (k == want) { for (auto &p : cloud->points) printf("O %08x %08x %08x\n", bits(p.x), bits(p.y), bits(p.z)); return 0; }
    printf("S %zu", cloud->points.size());
    if (cloud->points.empty()) { printf(" E nocloud\n"); continue; }
    std::vector<pcl::PointIndices> clusters;
    {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>(true));
      tree->setInputCloud(cloud);
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance((double)0.05f);
      ec.setMinClusterSize(30);
      ec.setMaxClusterSize(20000);
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud);
      ec.extract(clusters);
    }
    if (clusters.size() < 2) { printf(" E clusters %zu\n", clusters.size()); continue; }
    std::vector<Cloud::Ptr> cols;
    printf(" R");
    for (auto &ci : clusters) {
      Cloud::Ptr c(new Cloud);
      for (int i : ci.indices) c->points.push_back(cloud->points[i]);
      c->width = c->points.size(); c->height = 1; c->is_dense = true;
      pcl::PointXYZ lo, hi;
      pcl::getMinMax3D(*c, lo, hi);
      float dy = hi.y - lo.y;
      if (dy != 0.0f) {
        float ratio = fabsf(hi.z - lo.z) / fabsf(dy);
        printf(" %.9g", ratio);
        if (1.8f < ratio) cols.push_back(c);
      }
    }
    if (cols.size() < 2) { printf(" E columns\n"); continue; }
    printf(" C %zu", cols.size());
    // FindTargetRefleColu
    size_t nc = cols.size();
    std::vector<bool> used(nc, false);
    std::vector<Eigen::Vector4f> cent(nc);
    std::map<int, int> pairs;
    bool any = false;
    double best = 0;
    unsigned long bi = 0, bj = 0;
    for (size_t i = 0; i + 1 < nc; i++) {
      if (used[i]) continue;
      cent[i] = centroid(*cols[i]);
      used[i] = true;
      best = 1.7976931348623157e+308;
      for (size_t j = i + 1; j < nc; j++) {
        if (used[j]) continue;
        cent[j] = centroid(*cols[j]);
        Eigen::Vector4f d = cent[i] - cent[j];
        Eigen::Vector4f q = d.cwiseProduct(d);
        float dist = sqrtf((q[0] + q[1]) + (q[2] + q[3]));
        double off = fabs((double)dist - 0.4);
        double deg = ((double)atan2f(cent[j][1] - cent[i][1], cent[j][0] - cent[i][0]) * 180.0) / 3.141592653589793;
        if (off < 0.18 && fabs(deg) < 120.0 && 60.0 < fabs(deg)) {
          any = true;
          if (!(best <= off)) { best = off; bi = i; bj = j; }
        } else {
          printf(" X %.17g %.17g", off, deg);
        }
      }
      if (best < 0.1) { pairs.emplace((int)bi, (int)bj); used[bj] = true; }
    }
    if (!any) { printf(" E nopair\n"); continue; }
    if (pairs.empty()) { printf(" W %.17g E nopair\n", best); continue; }
    if (pairs.size() >= 2) {
      float least = INFINITY;
      for (auto &kv : pairs) {
        float a = fabsf(atan2f(cent[kv.first][0] - cent[kv.second][0], cent[kv.first][1] - cent[kv.second][1]));
        if ((double)a < (double)least) { bi = kv.first; bj = kv.second; least = a; }
      }
    }
    int lo_i, hi_i;
    if ((cent[bi][1] + cent[bj][1]) * 0.5f <= cent[bi][1]) { lo_i = bj; hi_i = bi; } else { lo_i = bi; hi_i = bj; }
    // FilterOutSuctionPoints, CalculateCenter
    double ctr[2][2];
    size_t cnt[2];
    int which[2] = {lo_i, hi_i};
    for (int k = 0; k < 2; k++) {
      Cloud &c = *cols[which[k]];
      Eigen::Vector4f m = centroid(c);
      Cloud f;
      for (auto &p : c.points)
        if ((double)p.x < (double)m[0] + 0.0 && (double)p.y < (double)m[1] + 0.02 && (double)m[1] - 0.02 < (double)p.y &&
            (double)p.z < (double)m[2] + 0.08 && (double)m[2] - 0.08 < (double)p.z)
          f.points.push_back(p);
      cnt[k] = f.points.size();
      Eigen::Vector4f cc = centroid(f);
      float s = sinf(fabsf(atan2f(cc[0], cc[1])));
      ctr[k][0] = std::fma((double)s, 0.045, (double)cc[0]);
      ctr[k][1] = (double)cc[1];
    }
    printf(" N %zu %zu P %.17g %.17g %.17g %.17g", cnt[0], cnt[1], ctr[0][0], ctr[0][1], ctr[1][0], ctr[1][1]);
    // calculateLidarPosition(high-y, low-y), z = 0
    double a[2] = {ctr[1][0], ctr[1][1]}, b[2] = {ctr[0][0], ctr[0][1]};
    double dx = a[0] - b[0], dy = a[1] - b[1];
    double n2 = dy * dy + dx * dx;
    double ux = dx, uy = dy;
    if (n2 > 0.0) { double nn = sqrt(n2); ux = dx / nn; uy = dy / nn; }
    double sx = a[0] + b[0], sy = a[1] + b[1];
    double px = ((-uy) * sx + ux * sy) / 2.0;
    double py = ((-ux) * sx + (-uy) * sy) / 2.0;
    double yw = atan2(a[0] - b[0], a[1] - b[1]);
    printf(" L %.17g %.17g %.17g\n", px, py, yw);
  }
  return 0;
}
