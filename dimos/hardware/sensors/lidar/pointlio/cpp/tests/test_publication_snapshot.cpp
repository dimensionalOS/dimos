#include "publication_snapshot.hpp"
#include <stdexcept>
#include <type_traits>

static void check(bool condition) {
    if (!condition) throw std::runtime_error("snapshot contract failed");
}
int main() {
    static_assert(std::is_const_v<std::remove_reference_t<decltype(*std::declval<PublicationSnapshot>().cloud())>>);
    PointLio core;
    PublicationSnapshot snapshot;
    check(!snapshot.update(core, true));
    check(snapshot.generation() == 0 && !snapshot.cloud());
    check(!snapshot.lidar_pending() && !snapshot.odom_pending());
    check(core.cloud_reads == 0);

    core.pose = {1, 2, 3}; core.odom.values = {4, 5}; core.body.points = {6, 7};
    core.next_success = true;
    check(snapshot.update(core, true));
    auto first_cloud = snapshot.cloud();
    check(snapshot.generation() == 1 && snapshot.pose() == core.pose);
    check(snapshot.odometry().values == core.odom.values);
    check(first_cloud->points == core.body.points);
    // A failed processing step may mutate the core. Retained output must not change.
    core.pose = {90}; core.odom.values = {91}; core.body.points = {92};
    check(!snapshot.update(core, true));
    check(snapshot.pose() == std::vector<double>({1, 2, 3}));
    check(snapshot.odometry().values == std::vector<double>({4, 5}));
    check(snapshot.cloud() == first_cloud && first_cloud->points == std::vector<double>({6, 7}));
    check(snapshot.generation() == 1 && core.cloud_reads == 1);
    snapshot.odom_published();
    check(!snapshot.odom_pending() && snapshot.lidar_pending());
    check(!snapshot.update(core, true));
    check(!snapshot.odom_pending() && snapshot.lidar_pending());
    snapshot.lidar_published();
    for (int i = 0; i != 100; ++i) check(!snapshot.update(core, true));
    check(!snapshot.odom_pending() && !snapshot.lidar_pending());
    // Successful updates re-arm both outputs, including unchanged static poses.
    core.next_success = true; check(snapshot.update(core, true));
    check(snapshot.generation() == 2 && snapshot.cloud()->points == core.body.points);
    core.next_success = true; check(snapshot.update(core, true));
    check(snapshot.generation() == 3 && snapshot.lidar_pending() && snapshot.odom_pending());
    snapshot.lidar_published();
    check(!snapshot.lidar_pending() && snapshot.odom_pending());
    check(first_cloud->points == std::vector<double>({6, 7}));
    // No-lidar mode must not call conversion or retain an older cloud.
    const auto reads = core.cloud_reads;
    core.next_success = true; check(snapshot.update(core, false));
    check(core.cloud_reads == reads && !snapshot.cloud());
    check(snapshot.pose() == core.pose && snapshot.odometry().values == core.odom.values);
    // A successful null-cloud result must not reuse the previous cloud.
    core.null_cloud = true; core.next_success = true;
    check(snapshot.update(core, true) && !snapshot.cloud());
    check(snapshot.generation() == 5);
}
