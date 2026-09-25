#include "publication_gate.hpp"
#include <cassert>
#include <stdexcept>
#undef assert
#define assert(condition) do { if (!(condition)) throw std::runtime_error("check failed: " #condition); } while (false)

int main() {
    PublicationGate gate;
    assert(!gate.lidar_pending() && !gate.odom_pending());
    gate.observe(false);
    assert(!gate.lidar_pending() && !gate.odom_pending());
    gate.observe(true); // a valid static pose is still an update
    for (int i = 0; i < 100; ++i) gate.observe(false);
    assert(gate.lidar_pending() && gate.odom_pending()); // rate-limit deferral
    gate.odom_published();
    assert(gate.lidar_pending() && !gate.odom_pending());
    gate.observe(false);
    assert(!gate.odom_pending()); // no duplicate after dropout
    gate.lidar_published();
    assert(!gate.lidar_pending());
    gate.observe(true); // recovery
    gate.observe(true); // latest-value coalescing at a lower publication rate
    gate.lidar_published();
    assert(!gate.lidar_pending() && gate.odom_pending());
    gate.odom_published();
    for (int i = 0; i < 1000; ++i) gate.observe(false);
    assert(!gate.lidar_pending() && !gate.odom_pending());
}
