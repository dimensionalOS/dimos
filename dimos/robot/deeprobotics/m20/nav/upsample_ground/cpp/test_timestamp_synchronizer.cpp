// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include "timestamp_synchronizer.hpp"

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

struct Sample {
    int id;
};

using Synchronizer =
    dimos::upsample_ground::TimestampSynchronizer<Sample, Sample, Sample>;
using dimos::upsample_ground::SyncStatus;

constexpr std::int64_t milliseconds(std::int64_t value) {
    return value * 1'000'000LL;
}

std::shared_ptr<const Sample> sample(int id) {
    return std::make_shared<const Sample>(Sample{id});
}

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

const char* statusName(SyncStatus status) {
    switch (status) {
        case SyncStatus::kSynchronized:
            return "SYNCHRONIZED";
        case SyncStatus::kWait:
            return "WAIT";
        case SyncStatus::kUnsynchronizable:
            return "UNSYNCHRONIZABLE";
    }
    return "UNKNOWN";
}

void printTest(const std::string& name, const std::string& input) {
    std::cout << "\n[TEST] " << name << '\n';
    std::cout << "  input:  " << input << '\n';
}

void printResult(const std::string& result) {
    std::cout << "  result: " << result << '\n';
    std::cout << "  PASS\n";
}

void testSelectsNearestBracketedSamplesFromOutOfOrderInput() {
    printTest(
        "nearest bracketed samples from out-of-order input",
        "global=1000ms, current(arrival)=[1060,960]ms, odometry(arrival)=[1030,980]ms");
    Synchronizer synchronizer(10, 10, 10, milliseconds(50));
    synchronizer.pushCurrentFrame(milliseconds(1060), sample(1060));
    synchronizer.pushCurrentFrame(milliseconds(960), sample(960));
    synchronizer.pushOdometry(milliseconds(1030), sample(1030));
    synchronizer.pushOdometry(milliseconds(980), sample(980));
    synchronizer.pushGlobalMap(milliseconds(1000), sample(1000));

    const auto decision = synchronizer.trySynchronize();

    require(decision.status == SyncStatus::kSynchronized, "expected synchronized decision");
    require(decision.current_frame->message->id == 960, "expected nearest current frame");
    require(decision.odometry->message->id == 980, "expected nearest odometry");
    require(decision.global_map->message->id == 1000, "expected global-map reference");
    printResult(
        std::string("status=") + statusName(decision.status) +
        ", selected current=" + std::to_string(decision.current_frame->message->id) +
        "ms, odometry=" + std::to_string(decision.odometry->message->id) +
        "ms, global=" + std::to_string(decision.global_map->message->id) + "ms");
}

void testSelectsEarlierSampleOnExactTie() {
    printTest(
        "earlier sample on exact tie",
        "global=1000ms, current=[950,1050]ms, odometry=[950,1050]ms");
    Synchronizer synchronizer(10, 10, 10, milliseconds(50));
    synchronizer.pushCurrentFrame(milliseconds(950), sample(950));
    synchronizer.pushCurrentFrame(milliseconds(1050), sample(1050));
    synchronizer.pushOdometry(milliseconds(950), sample(950));
    synchronizer.pushOdometry(milliseconds(1050), sample(1050));
    synchronizer.pushGlobalMap(milliseconds(1000), sample(1000));

    const auto decision = synchronizer.trySynchronize();

    require(decision.status == SyncStatus::kSynchronized, "expected synchronized tie");
    require(decision.current_frame->message->id == 950, "current-frame tie must choose earlier");
    require(decision.odometry->message->id == 950, "odometry tie must choose earlier");
    printResult(
        std::string("status=") + statusName(decision.status) +
        ", selected current=" + std::to_string(decision.current_frame->message->id) +
        "ms, odometry=" + std::to_string(decision.odometry->message->id) + "ms");
}

void testWaitsUntilBothQueuesContainAFutureSample() {
    printTest(
        "wait until both auxiliary queues contain a future sample",
        "global=1000ms, initial current=[900]ms, initial odometry=[900]ms; then current=1100ms, "
        "odometry=1020ms");
    Synchronizer synchronizer(10, 10, 10, milliseconds(50));
    synchronizer.pushCurrentFrame(milliseconds(900), sample(900));
    synchronizer.pushOdometry(milliseconds(900), sample(900));
    synchronizer.pushGlobalMap(milliseconds(1000), sample(1000));

    const auto initial_decision = synchronizer.trySynchronize();
    require(initial_decision.status == SyncStatus::kWait, "expected initial wait");
    require(synchronizer.globalMapQueueSize() == 1, "wait must retain global map");

    synchronizer.pushCurrentFrame(milliseconds(1100), sample(1100));
    const auto current_ready_decision = synchronizer.trySynchronize();
    require(
        current_ready_decision.status == SyncStatus::kWait,
        "expected wait for odometry future sample");
    require(synchronizer.globalMapQueueSize() == 1, "second wait must retain global map");

    synchronizer.pushOdometry(milliseconds(1020), sample(1020));
    const auto decision = synchronizer.trySynchronize();
    require(decision.status == SyncStatus::kSynchronized, "expected synchronization after future data");
    require(decision.current_frame->message->id == 900, "expected nearer earlier current frame");
    require(decision.odometry->message->id == 1020, "expected nearer later odometry");
    require(synchronizer.globalMapQueueSize() == 0, "matched global map must be consumed");
    printResult(
        std::string("initial status=") + statusName(initial_decision.status) +
        ", after current status=" + statusName(current_ready_decision.status) +
        ", final status=" + statusName(decision.status) +
        ", selected current=" + std::to_string(decision.current_frame->message->id) +
        "ms, odometry=" + std::to_string(decision.odometry->message->id) +
        "ms, remaining global maps=" + std::to_string(synchronizer.globalMapQueueSize()));
}

void testAcceptsFutureQueueHeadsWithinTolerance() {
    printTest(
        "future queue heads within 50ms tolerance",
        "global=1000ms, current head=1040ms, odometry head=1050ms, tolerance=50ms");
    Synchronizer synchronizer(10, 10, 10, milliseconds(50));
    synchronizer.pushCurrentFrame(milliseconds(1040), sample(1040));
    synchronizer.pushOdometry(milliseconds(1050), sample(1050));
    synchronizer.pushGlobalMap(milliseconds(1000), sample(1000));

    const auto decision = synchronizer.trySynchronize();

    require(decision.status == SyncStatus::kSynchronized, "future heads within tolerance must match");
    require(decision.current_frame->message->id == 1040, "wrong current-frame head selected");
    require(decision.odometry->message->id == 1050, "wrong odometry head selected");
    printResult(
        std::string("status=") + statusName(decision.status) +
        ", selected current=" + std::to_string(decision.current_frame->message->id) +
        "ms, odometry=" + std::to_string(decision.odometry->message->id) + "ms");
}

void testRejectsFutureQueueHeadBeyondTolerance() {
    printTest(
        "future queue head beyond 50ms tolerance",
        "global=1000ms, current head=1051ms, odometry=[900]ms, tolerance=50ms");
    Synchronizer synchronizer(10, 10, 10, milliseconds(50));
    synchronizer.pushCurrentFrame(milliseconds(1051), sample(1051));
    synchronizer.pushOdometry(milliseconds(900), sample(900));
    synchronizer.pushGlobalMap(milliseconds(1000), sample(1000));

    const auto decision = synchronizer.trySynchronize();

    require(
        decision.status == SyncStatus::kUnsynchronizable,
        "future head beyond tolerance must be unsynchronizable");
    require(decision.global_map->message->id == 1000, "fallback must retain reference map");
    require(synchronizer.globalMapQueueSize() == 0, "unsynchronizable map must be consumed");
    printResult(
        std::string("status=") + statusName(decision.status) +
        ", fallback global=" + std::to_string(decision.global_map->message->id) +
        "ms, remaining global maps=" + std::to_string(synchronizer.globalMapQueueSize()));
}

void testGlobalMapQueueEvictsOldestReferenceAtCapacity() {
    printTest(
        "global-map queue capacity",
        "capacity=2, global arrival=[2000,3000,1000]ms");
    Synchronizer synchronizer(10, 2, 10, milliseconds(50));
    require(
        !synchronizer.pushGlobalMap(milliseconds(2000), sample(2000)).has_value(),
        "first insert must not evict");
    require(
        !synchronizer.pushGlobalMap(milliseconds(3000), sample(3000)).has_value(),
        "second insert must not evict");

    const auto evicted = synchronizer.pushGlobalMap(milliseconds(1000), sample(1000));

    require(evicted.has_value(), "capacity overflow must return an evicted map");
    require(evicted->message->id == 1000, "oldest timestamp must be evicted");
    require(synchronizer.globalMapQueueSize() == 2, "queue must remain bounded");
    printResult(
        "evicted global=" + std::to_string(evicted->message->id) +
        "ms, queue size=" + std::to_string(synchronizer.globalMapQueueSize()));
}

void testQueuesUseIndependentCapacities() {
    printTest(
        "independent queue capacities",
        "current capacity=2, global capacity=3, odometry capacity=4");
    Synchronizer synchronizer(2, 3, 4, milliseconds(50));
    synchronizer.pushCurrentFrame(milliseconds(1000), sample(1000));
    synchronizer.pushCurrentFrame(milliseconds(2000), sample(2000));
    synchronizer.pushCurrentFrame(milliseconds(3000), sample(3000));
    synchronizer.pushGlobalMap(milliseconds(1000), sample(1000));
    synchronizer.pushGlobalMap(milliseconds(2000), sample(2000));
    synchronizer.pushGlobalMap(milliseconds(3000), sample(3000));
    synchronizer.pushGlobalMap(milliseconds(4000), sample(4000));
    synchronizer.pushOdometry(milliseconds(1000), sample(1000));
    synchronizer.pushOdometry(milliseconds(2000), sample(2000));
    synchronizer.pushOdometry(milliseconds(3000), sample(3000));
    synchronizer.pushOdometry(milliseconds(4000), sample(4000));
    synchronizer.pushOdometry(milliseconds(5000), sample(5000));

    require(synchronizer.currentFrameQueueSize() == 2, "current-frame queue must remain bounded");
    require(synchronizer.globalMapQueueSize() == 3, "global-map queue must remain bounded");
    require(synchronizer.odometryQueueSize() == 4, "odometry queue must remain bounded");
    printResult(
        "current queue size=" + std::to_string(synchronizer.currentFrameQueueSize()) +
        ", global queue size=" + std::to_string(synchronizer.globalMapQueueSize()) +
        ", odometry queue size=" + std::to_string(synchronizer.odometryQueueSize()));
}

void testGlobalMapReferenceOrderUsesTimestampsNotArrivalOrder() {
    printTest(
        "global-map reference ordered by timestamp",
        "global arrival=[2000,1000]ms, current=[950,1050]ms, odometry=[950,1050]ms");
    Synchronizer synchronizer(10, 10, 10, milliseconds(50));
    synchronizer.pushGlobalMap(milliseconds(2000), sample(2000));
    synchronizer.pushGlobalMap(milliseconds(1000), sample(1000));
    synchronizer.pushCurrentFrame(milliseconds(950), sample(950));
    synchronizer.pushCurrentFrame(milliseconds(1050), sample(1050));
    synchronizer.pushOdometry(milliseconds(950), sample(950));
    synchronizer.pushOdometry(milliseconds(1050), sample(1050));

    const auto decision = synchronizer.trySynchronize();

    require(decision.status == SyncStatus::kSynchronized, "expected first reference to synchronize");
    require(decision.global_map->message->id == 1000, "oldest timestamp must be reference");
    require(synchronizer.globalMapQueueSize() == 1, "newer global map must remain queued");
    printResult(
        std::string("status=") + statusName(decision.status) +
        ", processed global=" + std::to_string(decision.global_map->message->id) +
        "ms, remaining global maps=" + std::to_string(synchronizer.globalMapQueueSize()));
}

}  // namespace

int main() {
    std::cout << "TimestampSynchronizer test data and results\n";
    try {
        testSelectsNearestBracketedSamplesFromOutOfOrderInput();
        testSelectsEarlierSampleOnExactTie();
        testWaitsUntilBothQueuesContainAFutureSample();
        testAcceptsFutureQueueHeadsWithinTolerance();
        testRejectsFutureQueueHeadBeyondTolerance();
        testGlobalMapQueueEvictsOldestReferenceAtCapacity();
        testQueuesUseIndependentCapacities();
        testGlobalMapReferenceOrderUsesTimestampsNotArrivalOrder();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "\nAll synchronization scenarios passed.\n";
    return EXIT_SUCCESS;
}
