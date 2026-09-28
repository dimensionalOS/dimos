// Deterministic lock-order contract test, not a transport-queue saturation test.
#include "input_continuity_gate.hpp"
#include <atomic>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <iostream>

void require(bool value) { if (!value) throw std::runtime_error("contract failed"); }
int main() {
    using Gate = InputContinuityGate;
    for (int iteration=0; iteration<100; ++iteration) {
        Gate gate(1.);
        std::mutex mutex;
        std::atomic<bool> failed{false};
        unsigned published=0;
        require(gate.observe(Gate::Stream::imu,0));
        // Publish owns the lock first: one pre-fault item may already be queued.
        std::promise<void> publishing, detector_ready;
        auto publishing_future=publishing.get_future();
        auto detector_future=detector_ready.get_future();
        std::thread publisher([&] {
            std::lock_guard<std::mutex> lock(mutex);
            publishing.set_value();
            detector_future.wait();
            if (!failed.load()) ++published;
        });
        std::thread detector([&] {
            publishing_future.wait();
            detector_ready.set_value();
            std::lock_guard<std::mutex> lock(mutex);
            if (!gate.observe(Gate::Stream::imu,2000000000)) failed.store(true);
        });
        publisher.join(); detector.join();
        require(failed && published==1);
        // Fault owns lock first: no subsequent publication or rearming.
        std::thread after([&] {
            std::lock_guard<std::mutex> lock(mutex);
            if (!failed.load()) ++published;
            require(!gate.observe(Gate::Stream::lidar,0));
            require(!gate.observe(Gate::Stream::imu,2005000000));
        });
        after.join();
        require(published==1);
    }
    std::cout << "PASS: 100 deterministic publication-before-fault / fault-before-publication contracts\n";
}
