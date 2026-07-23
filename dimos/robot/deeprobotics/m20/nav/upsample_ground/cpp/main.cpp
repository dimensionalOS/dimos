// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <lcm/lcm-cpp.hpp>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <csignal>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "dimos_native_module.hpp"
#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "timestamp_synchronizer.hpp"

namespace {

constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;

std::atomic<bool> running{true};

void signalHandler(int) {
    running.store(false);
}

template <typename Message>
std::int64_t timestampNs(const Message& message) {
    return static_cast<std::int64_t>(message.header.stamp.sec) * kNanosecondsPerSecond +
           static_cast<std::int64_t>(message.header.stamp.nsec);
}

const char* syncStatusName(dimos::upsample_ground::SyncStatus status) {
    switch (status) {
        case dimos::upsample_ground::SyncStatus::kSynchronized:
            return "SYNCHRONIZED";
        case dimos::upsample_ground::SyncStatus::kWait:
            return "WAIT";
        case dimos::upsample_ground::SyncStatus::kUnsynchronizable:
            return "UNSYNCHRONIZABLE";
    }
    return "UNKNOWN";
}

template <typename Message>
std::string timestampText(
    const std::optional<dimos::upsample_ground::TimedMessage<Message>>& message) {
    if (!message.has_value()) {
        return "none";
    }
    return std::to_string(message->timestamp_ns);
}

template <typename Message>
std::string deltaMillisecondsText(
    const std::optional<dimos::upsample_ground::TimedMessage<Message>>& message,
    std::int64_t reference_ns) {
    if (!message.has_value()) {
        return "none";
    }
    const double delta_ms =
        static_cast<double>(message->timestamp_ns - reference_ns) / 1'000'000.0;
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.3f", delta_ms);
    return buffer;
}

class UpsampleGroundApplication {
public:
    UpsampleGroundApplication(
        lcm::LCM& lcm,
        std::string output_topic,
        std::size_t current_frame_queue_capacity,
        std::size_t global_map_queue_capacity,
        std::size_t odometry_queue_capacity,
        std::int64_t max_future_delta_ns,
        bool debug)
        : lcm_(lcm),
          output_topic_(std::move(output_topic)),
          synchronizer_(
              current_frame_queue_capacity,
              global_map_queue_capacity,
              odometry_queue_capacity,
              max_future_delta_ns),
          debug_(debug) {}

    void onCurrentFrame(
        const lcm::ReceiveBuffer*,
        const std::string&,
        const sensor_msgs::PointCloud2* message) {
        synchronizer_.pushCurrentFrame(
            timestampNs(*message), std::make_shared<sensor_msgs::PointCloud2>(*message));
        processAvailable();
    }

    void onGlobalMap(
        const lcm::ReceiveBuffer*,
        const std::string&,
        const sensor_msgs::PointCloud2* message) {
        const auto evicted = synchronizer_.pushGlobalMap(
            timestampNs(*message), std::make_shared<sensor_msgs::PointCloud2>(*message));
        if (evicted.has_value()) {
            ++queue_overflow_count_;
            publishFallback(*evicted->message);
            if (debug_) {
                std::fprintf(
                    stdout,
                    "UpsampleGround sync result: status=QUEUE_OVERFLOW global_ns=%lld "
                    "queues=(%zu,%zu,%zu) published=%llu\n",
                    static_cast<long long>(evicted->timestamp_ns),
                    synchronizer_.currentFrameQueueSize(),
                    synchronizer_.globalMapQueueSize(),
                    synchronizer_.odometryQueueSize(),
                    static_cast<unsigned long long>(published_count_));
            }
        }
        processAvailable();
    }

    void onOdometry(
        const lcm::ReceiveBuffer*,
        const std::string&,
        const nav_msgs::Odometry* message) {
        synchronizer_.pushOdometry(
            timestampNs(*message), std::make_shared<nav_msgs::Odometry>(*message));
        processAvailable();
    }

private:
    using Synchronizer = dimos::upsample_ground::TimestampSynchronizer<
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        nav_msgs::Odometry>;

    void processAvailable() {
        while (true) {
            const auto decision = synchronizer_.trySynchronize();
            if (decision.status == dimos::upsample_ground::SyncStatus::kWait) {
                ++wait_count_;
                logDecision(decision);
                return;
            }

            if (decision.status == dimos::upsample_ground::SyncStatus::kUnsynchronizable) {
                ++unsynchronizable_count_;
                logDecision(decision);
                publishFallback(*decision.global_map->message);
                continue;
            }

            ++synchronized_count_;
            logDecision(decision);
            const auto& current_frame = *decision.current_frame->message;
            const auto& global_map = *decision.global_map->message;
            if (current_frame.header.frame_id != global_map.header.frame_id) {
                std::fprintf(
                    stderr,
                    "UpsampleGround: frame mismatch current_frame='%s' global_map='%s'\n",
                    current_frame.header.frame_id.c_str(),
                    global_map.header.frame_id.c_str());
                publishFallback(global_map);
                continue;
            }

            // Ground estimation is intentionally deferred to the next implementation stage.
            publishFallback(global_map);
        }
    }

    void publishFallback(const sensor_msgs::PointCloud2& global_map) {
        lcm_.publish(output_topic_, &global_map);
        ++published_count_;
    }

    void logDecision(
        const dimos::upsample_ground::SyncDecision<
            sensor_msgs::PointCloud2,
            sensor_msgs::PointCloud2,
            nav_msgs::Odometry>& decision) const {
        if (!debug_ || !decision.global_map.has_value()) {
            return;
        }
        const std::int64_t reference_ns = decision.global_map->timestamp_ns;
        const std::string current_ns = timestampText(decision.current_frame);
        const std::string odometry_ns = timestampText(decision.odometry);
        const std::string current_delta_ms =
            deltaMillisecondsText(decision.current_frame, reference_ns);
        const std::string odometry_delta_ms = deltaMillisecondsText(decision.odometry, reference_ns);
        std::fprintf(
            stdout,
            "UpsampleGround sync result: status=%s global_ns=%lld current_ns=%s odometry_ns=%s "
            "current_delta_ms=%s odometry_delta_ms=%s queues=(%zu,%zu,%zu) "
            "counts=(sync:%llu,wait:%llu,unsync:%llu,overflow:%llu)\n",
            syncStatusName(decision.status),
            static_cast<long long>(reference_ns),
            current_ns.c_str(),
            odometry_ns.c_str(),
            current_delta_ms.c_str(),
            odometry_delta_ms.c_str(),
            synchronizer_.currentFrameQueueSize(),
            synchronizer_.globalMapQueueSize(),
            synchronizer_.odometryQueueSize(),
            static_cast<unsigned long long>(synchronized_count_),
            static_cast<unsigned long long>(wait_count_),
            static_cast<unsigned long long>(unsynchronizable_count_),
            static_cast<unsigned long long>(queue_overflow_count_));
    }

    lcm::LCM& lcm_;
    std::string output_topic_;
    Synchronizer synchronizer_;
    bool debug_;
    std::uint64_t synchronized_count_ = 0;
    std::uint64_t wait_count_ = 0;
    std::uint64_t unsynchronizable_count_ = 0;
    std::uint64_t queue_overflow_count_ = 0;
    std::uint64_t published_count_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    std::signal(SIGTERM, signalHandler);
    std::signal(SIGINT, signalHandler);

    dimos::NativeModule module(argc, argv);
    const std::string current_frame_topic = module.topic("current_frame");
    const std::string global_map_topic = module.topic("global_map");
    const std::string odometry_topic = module.topic("odometry");
    const std::string output_topic = module.topic("global_map_upsample_ground");

    const int current_frame_queue_size = module.arg_int("current_frame_queue_size", 20);
    const int global_map_queue_size = module.arg_int("global_map_queue_size", 20);
    const int odometry_queue_size = module.arg_int("odometry_queue_size", 200);
    const double max_future_delta_s = module.arg_float("max_future_sync_delta_s", 0.05F);
    if (current_frame_queue_size <= 0 || global_map_queue_size <= 0 ||
        odometry_queue_size <= 0 || max_future_delta_s < 0.0) {
        std::fprintf(stderr, "UpsampleGround: invalid synchronization configuration\n");
        return 1;
    }

    const auto max_future_delta_ns = static_cast<std::int64_t>(
        max_future_delta_s * static_cast<double>(kNanosecondsPerSecond));
    const bool debug = module.arg_bool("debug", false);

    lcm::LCM lcm;
    if (!lcm.good()) {
        std::fprintf(stderr, "UpsampleGround: LCM initialization failed\n");
        return 1;
    }

    UpsampleGroundApplication application(
        lcm,
        output_topic,
        static_cast<std::size_t>(current_frame_queue_size),
        static_cast<std::size_t>(global_map_queue_size),
        static_cast<std::size_t>(odometry_queue_size),
        max_future_delta_ns,
        debug);
    lcm.subscribe(
        current_frame_topic,
        &UpsampleGroundApplication::onCurrentFrame,
        &application);
    lcm.subscribe(global_map_topic, &UpsampleGroundApplication::onGlobalMap, &application);
    lcm.subscribe(odometry_topic, &UpsampleGroundApplication::onOdometry, &application);

    while (running.load()) {
        const int result = lcm.handleTimeout(100);
        if (result < 0) {
            if (!running.load()) {
                break;
            }
            std::fprintf(stderr, "UpsampleGround: LCM receive failed\n");
            return 1;
        }
    }

    return 0;
}
