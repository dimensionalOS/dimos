// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

namespace dimos::upsample_ground {

using TimestampNs = std::int64_t;

template <typename Message>
struct TimedMessage {
    TimestampNs timestamp_ns;
    std::shared_ptr<const Message> message;
};

enum class SelectionStatus {
    kSelected,
    kWait,
    kUnsynchronizable,
};

template <typename Message>
struct Selection {
    SelectionStatus status = SelectionStatus::kWait;
    std::optional<TimedMessage<Message>> value;
};

template <typename Message>
class OrderedTimestampQueue {
public:
    explicit OrderedTimestampQueue(std::size_t capacity) : capacity_(capacity) {
        if (capacity_ == 0) {
            throw std::invalid_argument("timestamp queue capacity must be greater than zero");
        }
    }

    std::optional<TimedMessage<Message>> push(
        TimestampNs timestamp_ns,
        std::shared_ptr<const Message> message) {
        TimedMessage<Message> entry{timestamp_ns, std::move(message)};
        const auto position = std::upper_bound(
            entries_.begin(), entries_.end(), timestamp_ns,
            [](TimestampNs timestamp, const TimedMessage<Message>& queued) {
                return timestamp < queued.timestamp_ns;
            });
        entries_.insert(position, std::move(entry));

        if (entries_.size() <= capacity_) {
            return std::nullopt;
        }

        TimedMessage<Message> evicted = std::move(entries_.front());
        entries_.pop_front();
        return evicted;
    }

    Selection<Message> selectNearest(
        TimestampNs reference_ns,
        TimestampNs max_future_delta_ns) const {
        if (entries_.empty()) {
            return {SelectionStatus::kWait, std::nullopt};
        }

        const auto& head = entries_.front();
        if (head.timestamp_ns == reference_ns) {
            return {SelectionStatus::kSelected, head};
        }

        if (head.timestamp_ns > reference_ns) {
            const TimestampNs lead_ns = head.timestamp_ns - reference_ns;
            if (lead_ns <= max_future_delta_ns) {
                return {SelectionStatus::kSelected, head};
            }
            return {SelectionStatus::kUnsynchronizable, std::nullopt};
        }

        const auto first_after = std::upper_bound(
            entries_.begin(), entries_.end(), reference_ns,
            [](TimestampNs timestamp, const TimedMessage<Message>& queued) {
                return timestamp < queued.timestamp_ns;
            });
        if (first_after == entries_.end()) {
            return {SelectionStatus::kWait, std::nullopt};
        }

        const auto before = std::prev(first_after);
        const TimestampNs before_delta_ns = reference_ns - before->timestamp_ns;
        const TimestampNs after_delta_ns = first_after->timestamp_ns - reference_ns;
        const auto& nearest = before_delta_ns <= after_delta_ns ? *before : *first_after;
        return {SelectionStatus::kSelected, nearest};
    }

    void pruneAfterProcessedReference(TimestampNs reference_ns) {
        while (entries_.size() > 1 && entries_[1].timestamp_ns <= reference_ns) {
            entries_.pop_front();
        }
    }

    const TimedMessage<Message>& front() const {
        return entries_.front();
    }

    void popFront() {
        entries_.pop_front();
    }

    [[nodiscard]] bool empty() const {
        return entries_.empty();
    }

    [[nodiscard]] std::size_t size() const {
        return entries_.size();
    }

private:
    std::size_t capacity_;
    std::deque<TimedMessage<Message>> entries_;
};

enum class SyncStatus {
    kSynchronized,
    kWait,
    kUnsynchronizable,
};

template <typename CurrentFrame, typename GlobalMap, typename Odometry>
struct SyncDecision {
    SyncStatus status = SyncStatus::kWait;
    std::optional<TimedMessage<CurrentFrame>> current_frame;
    std::optional<TimedMessage<GlobalMap>> global_map;
    std::optional<TimedMessage<Odometry>> odometry;
};

template <typename CurrentFrame, typename GlobalMap, typename Odometry>
class TimestampSynchronizer {
public:
    TimestampSynchronizer(
        std::size_t current_frame_capacity,
        std::size_t global_map_capacity,
        std::size_t odometry_capacity,
        TimestampNs max_future_delta_ns)
        : current_frames_(current_frame_capacity),
          global_maps_(global_map_capacity),
          odometry_(odometry_capacity),
          max_future_delta_ns_(max_future_delta_ns) {
        if (max_future_delta_ns_ < 0) {
            throw std::invalid_argument("future synchronization tolerance must not be negative");
        }
    }

    void pushCurrentFrame(TimestampNs timestamp_ns, std::shared_ptr<const CurrentFrame> message) {
        static_cast<void>(current_frames_.push(timestamp_ns, std::move(message)));
    }

    std::optional<TimedMessage<GlobalMap>> pushGlobalMap(
        TimestampNs timestamp_ns,
        std::shared_ptr<const GlobalMap> message) {
        return global_maps_.push(timestamp_ns, std::move(message));
    }

    void pushOdometry(TimestampNs timestamp_ns, std::shared_ptr<const Odometry> message) {
        static_cast<void>(odometry_.push(timestamp_ns, std::move(message)));
    }

    SyncDecision<CurrentFrame, GlobalMap, Odometry> trySynchronize() {
        if (global_maps_.empty()) {
            return {};
        }

        const auto reference = global_maps_.front();
        const auto current_selection =
            current_frames_.selectNearest(reference.timestamp_ns, max_future_delta_ns_);
        const auto odometry_selection =
            odometry_.selectNearest(reference.timestamp_ns, max_future_delta_ns_);

        if (current_selection.status == SelectionStatus::kUnsynchronizable ||
            odometry_selection.status == SelectionStatus::kUnsynchronizable) {
            consumeReference(reference.timestamp_ns);
            return {
                SyncStatus::kUnsynchronizable,
                current_selection.value,
                reference,
                odometry_selection.value,
            };
        }

        if (current_selection.status == SelectionStatus::kWait ||
            odometry_selection.status == SelectionStatus::kWait) {
            return {
                SyncStatus::kWait,
                current_selection.value,
                reference,
                odometry_selection.value,
            };
        }

        consumeReference(reference.timestamp_ns);
        return {
            SyncStatus::kSynchronized,
            current_selection.value,
            reference,
            odometry_selection.value,
        };
    }

    [[nodiscard]] std::size_t currentFrameQueueSize() const {
        return current_frames_.size();
    }

    [[nodiscard]] std::size_t globalMapQueueSize() const {
        return global_maps_.size();
    }

    [[nodiscard]] std::size_t odometryQueueSize() const {
        return odometry_.size();
    }

private:
    void consumeReference(TimestampNs reference_ns) {
        global_maps_.popFront();
        current_frames_.pruneAfterProcessedReference(reference_ns);
        odometry_.pruneAfterProcessedReference(reference_ns);
    }

    OrderedTimestampQueue<CurrentFrame> current_frames_;
    OrderedTimestampQueue<GlobalMap> global_maps_;
    OrderedTimestampQueue<Odometry> odometry_;
    TimestampNs max_future_delta_ns_;
};

}  // namespace dimos::upsample_ground
