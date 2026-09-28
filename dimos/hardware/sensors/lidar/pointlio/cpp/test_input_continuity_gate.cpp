#include "input_continuity_gate.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using Gate = InputContinuityGate;
constexpr auto L = Gate::Stream::lidar;
constexpr auto I = Gate::Stream::imu;
void require(bool ok) { if (!ok) throw std::runtime_error("test failed"); }
int main() {
    Gate normal(.5);
    // Interleaved independent streams, normal 200Hz IMU and 10Hz scan times.
    for (uint64_t n = 0; n < 10000; ++n) {
        require(normal.observe(I, n * 5000000));
        if (n % 20 == 0) require(normal.observe(L, n * 5000000));
    }
    require(!normal.fault());
    Gate exact(.5);
    require(exact.observe(I, 0));
    require(exact.observe(I, 500000000));
    require(exact.observe(I, 500000000)); // duplicate preserves baseline behavior
    require(!exact.observe(I, 1000000001));
    require(exact.fault()->reason == Gate::Reason::gap);
    require(!exact.observe(L, 0)); // cannot rearm through another stream
    require(!exact.observe(I, 500000001));
    for (auto stream : {I, L}) {
        Gate loss(.5);
        require(loss.observe(stream, 1000000000));
        require(!loss.observe(stream, 21000000000));
        require(loss.fault()->stream == stream);
        Gate backwards(.5);
        require(backwards.observe(stream, 100));
        require(!backwards.observe(stream, 99));
        require(backwards.fault()->reason == Gate::Reason::backwards);
    }
    Gate overflow(.5);
    require(overflow.observe(I, UINT64_MAX-100));
    require(overflow.observe(I, UINT64_MAX));
    require(!overflow.observe(I, 0));
    Gate disabled(0);
    require(disabled.observe(I, UINT64_MAX));
    require(disabled.observe(I, 0));
    require(!disabled.fault());
    for (double invalid : {-1., 1e-12, 86401.,
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try { Gate gate(invalid); } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected);
    }
    std::cout << "PASS: continuity, independent streams, boundary, latch, backwards, overflow, opt-out, config\n";
}
