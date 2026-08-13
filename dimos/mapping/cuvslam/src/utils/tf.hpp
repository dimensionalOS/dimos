// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Transform client, a C++ mirror of the rust one (native/rust/dimos-module/src/tf.rs).
//
// Each tf edge is buffered per (parent, child), and Tf::lookup composes the shortest
// path through the frame graph. Lookups are nearest-in-time within a tolerance, not
// interpolated. Tf::publish feeds the local graph and forwards to a caller-provided
// sink (the module wires its tf output port there). Depends on Eigen and the standard
// library only; warnings go through an injectable sink.

#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

namespace tf_client {

/// How many seconds of history each edge keeps.
constexpr double DEFAULT_TF_WINDOW_SECS = 10.0;

inline double now_secs() {
    return std::chrono::duration<double>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

/// A rigid transform from `parent` to `child` at a point in time. It maps a point
/// expressed in `child` coordinates into `parent` coordinates.
struct Transform {
    std::string parent;
    std::string child;
    double ts = 0.0;
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();

    Eigen::Vector3d translation() const { return iso.translation(); }
    Eigen::Quaterniond rotation() const { return Eigen::Quaterniond(iso.linear()); }

    Transform inverse() const { return Transform{child, parent, ts, iso.inverse()}; }

    Transform compose(const Transform& other) const {
        return Transform{parent, other.child, ts, iso * other.iso};
    }
};

/// One edge's time-sorted history, capped to a fixed-duration window. Public so its
/// windowing behaviour is directly testable, like the rust module's own tests do.
class TBuffer {
public:
    struct Sample {
        double ts;
        Eigen::Isometry3d iso;
    };

    explicit TBuffer(double window_secs) : window_secs_(window_secs) {}

    void add(double ts, const Eigen::Isometry3d& iso) {
        // A stamp a whole window behind the newest is a clock reset, not jitter.
        if (!samples_.empty() && ts < samples_.back().ts - window_secs_) {
            samples_.clear();
        }
        const auto position = std::upper_bound(
            samples_.begin(), samples_.end(), ts,
            [](double value, const Sample& sample) { return value < sample.ts; });
        samples_.insert(position, Sample{ts, iso});
        // Anchored to the newest sample so a late message cannot widen the window.
        const double newest = samples_.back().ts;
        prune(newest - window_secs_);
    }

    const std::deque<Sample>& samples() const { return samples_; }

    const Sample* last() const { return samples_.empty() ? nullptr : &samples_.back(); }

    /// Nearest sample in time. On a tie, prefer the later sample. Null when the
    /// closest sample is further than tolerance from ts.
    const Sample* find_closest(double ts, std::optional<double> tolerance) const {
        const auto position = std::lower_bound(
            samples_.begin(), samples_.end(), ts,
            [](const Sample& sample, double value) { return sample.ts < value; });
        const Sample* next = position == samples_.end() ? nullptr : &*position;
        const Sample* previous = position == samples_.begin() ? nullptr : &*(position - 1);
        const Sample* best = nullptr;
        if (previous != nullptr && next != nullptr) {
            best = std::abs(next->ts - ts) <= std::abs(ts - previous->ts) ? next : previous;
        } else if (previous != nullptr) {
            best = previous;
        } else if (next != nullptr) {
            best = next;
        } else {
            return nullptr;
        }
        if (tolerance.has_value() && std::abs(best->ts - ts) > *tolerance) {
            return nullptr;
        }
        return best;
    }

    std::optional<Transform> sample(const std::string& parent, const std::string& child,
                                    std::optional<double> time,
                                    std::optional<double> tolerance) const {
        const Sample* found = nullptr;
        if (!time.has_value()) {
            found = last();
        } else {
            found = find_closest(*time, tolerance.value_or(window_secs_));
        }
        if (found == nullptr) {
            return std::nullopt;
        }
        return Transform{parent, child, found->ts, found->iso};
    }

private:
    void prune(double min_ts) {
        while (!samples_.empty() && samples_.front().ts < min_ts) {
            samples_.pop_front();
        }
    }

    double window_secs_;
    std::deque<Sample> samples_;
};

/// A cheap handle for querying and publishing transforms.
///
/// Mirrors the rust Tf: `lookup(parent, child)` starts a builder refined with `at()`
/// and `tolerance()` and finished with `get()` or a blocking `within()`;
/// `get_latest()` is the no-refinement shorthand; `receive()` ingests wire traffic;
/// `publish()` stores and forwards through the publish sink. Thread-safe.
class Tf {
public:
    /// Receives everything publish() sends, e.g. to encode onto the module's tf port.
    using PublishSink = std::function<void(const std::vector<Transform>&)>;
    /// Receives unresolved-lookup warnings (throttled to one per second per pair).
    using WarnSink = std::function<void(const std::string& message)>;

    explicit Tf(double window_secs = DEFAULT_TF_WINDOW_SECS) : window_secs_(window_secs) {}

    void set_publish_sink(PublishSink sink) {
        const std::lock_guard<std::mutex> guard(mutex_);
        publish_sink_ = std::move(sink);
    }

    void set_warn_sink(WarnSink sink) {
        const std::lock_guard<std::mutex> guard(mutex_);
        warn_sink_ = std::move(sink);
    }

    /// Ingest one wire edge. A zero or non-finite rotation would normalize to NaN and
    /// poison every lookup through it, so it is dropped instead of stored.
    void receive(const std::string& parent, const std::string& child, double ts,
                 const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation) {
        const double norm = rotation.norm();
        if (!std::isfinite(norm) || norm < 1e-12 || !translation.allFinite()) {
            return;
        }
        Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
        iso.linear() = rotation.normalized().toRotationMatrix();
        iso.translation() = translation;
        {
            const std::lock_guard<std::mutex> guard(mutex_);
            edge_buffer(parent, child).add(ts, iso);
        }
        changed_.notify_all();
    }

    /// The latest transform from `parent` to `child`; shorthand for lookup().get().
    std::optional<Transform> get_latest(const std::string& parent, const std::string& child);

    class Lookup;
    /// Start a lookup of the transform from `parent` to `child`. Refine it with
    /// Lookup::at and Lookup::tolerance, then finish with Lookup::get.
    Lookup lookup(const std::string& parent, const std::string& child);

    /// Publish transforms: they feed the local graph, so a lookup right after sees
    /// them without waiting for a transport round trip, and go to the publish sink.
    void publish(const std::vector<Transform>& transforms) {
        PublishSink sink;
        {
            const std::lock_guard<std::mutex> guard(mutex_);
            for (const Transform& transform : transforms) {
                edge_buffer(transform.parent, transform.child).add(transform.ts, transform.iso);
            }
            sink = publish_sink_;
        }
        changed_.notify_all();
        if (sink) {
            sink(transforms);
        }
    }

private:
    friend class Lookup;

    TBuffer& edge_buffer(const std::string& parent, const std::string& child) {
        const auto key = std::make_pair(parent, child);
        auto found = buffers_.find(key);
        if (found == buffers_.end()) {
            found = buffers_.emplace(key, TBuffer(window_secs_)).first;
        }
        return found->second;
    }

    std::vector<std::string> connections(const std::string& frame) const {
        std::vector<std::string> out;
        for (const auto& [key, buffer] : buffers_) {
            if (key.first == frame) {
                out.push_back(key.second);
            }
            if (key.second == frame) {
                out.push_back(key.first);
            }
        }
        return out;
    }

    std::optional<Transform> edge(const std::string& parent, const std::string& child,
                                  std::optional<double> time,
                                  std::optional<double> tolerance) const {
        if (parent == child) {
            return Transform{parent, child, time.value_or(now_secs()),
                             Eigen::Isometry3d::Identity()};
        }
        auto direct = buffers_.find(std::make_pair(parent, child));
        if (direct != buffers_.end()) {
            return direct->second.sample(parent, child, time, tolerance);
        }
        auto reversed = buffers_.find(std::make_pair(child, parent));
        if (reversed != buffers_.end()) {
            auto sample = reversed->second.sample(child, parent, time, tolerance);
            if (sample.has_value()) {
                return sample->inverse();
            }
        }
        return std::nullopt;
    }

    std::optional<std::vector<Transform>> bfs(const std::string& parent,
                                              const std::string& child,
                                              std::optional<double> time,
                                              std::optional<double> tolerance) const {
        std::deque<std::pair<std::string, std::vector<Transform>>> queue{{parent, {}}};
        std::set<std::string> visited{parent};
        while (!queue.empty()) {
            auto [frame, path] = std::move(queue.front());
            queue.pop_front();
            if (frame == child) {
                return path;
            }
            for (const std::string& next : connections(frame)) {
                if (visited.count(next) != 0) {
                    continue;
                }
                const auto step = edge(frame, next, time, tolerance);
                if (!step.has_value()) {
                    continue;
                }
                visited.insert(next);
                std::vector<Transform> extended = path;
                extended.push_back(*step);
                queue.emplace_back(next, std::move(extended));
            }
        }
        return std::nullopt;
    }

    std::optional<Transform> resolve_locked(const std::string& parent,
                                            const std::string& child,
                                            std::optional<double> time,
                                            std::optional<double> tolerance) const {
        const auto direct = edge(parent, child, time, tolerance);
        if (direct.has_value()) {
            return direct;
        }
        const auto path = bfs(parent, child, time, tolerance);
        if (!path.has_value() || path->empty()) {
            return std::nullopt;
        }
        // A composition is only as fresh as its stalest edge.
        double oldest = path->front().ts;
        Transform composed = path->front();
        for (std::size_t i = 1; i < path->size(); ++i) {
            oldest = std::min(oldest, (*path)[i].ts);
            composed = composed.compose((*path)[i]);
        }
        composed.ts = oldest;
        return composed;
    }

    /// Once per second per pair, so one noisy lookup cannot mute another pair's miss.
    bool should_warn(const std::string& parent, const std::string& child) {
        const auto now = std::chrono::steady_clock::now();
        auto& last = warned_[std::make_pair(parent, child)];
        if (last.has_value() && now - *last < std::chrono::seconds(1)) {
            return false;
        }
        last = now;
        return true;
    }

    void warn_unresolved(const std::string& parent, const std::string& child,
                         std::optional<double> time, std::optional<double> tolerance) {
        WarnSink sink;
        {
            const std::lock_guard<std::mutex> guard(mutex_);
            if (!should_warn(parent, child)) {
                return;
            }
            sink = warn_sink_;
        }
        if (sink) {
            sink("No transform found between frames '" + parent + "' and '" + child +
                 "' at " + std::to_string(time.value_or(now_secs())) + " tolerance " +
                 (tolerance.has_value() ? std::to_string(*tolerance) : "none"));
        }
    }

    double window_secs_;
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    std::map<std::pair<std::string, std::string>, TBuffer> buffers_;
    std::map<std::pair<std::string, std::string>,
             std::optional<std::chrono::steady_clock::time_point>>
        warned_;
    PublishSink publish_sink_;
    WarnSink warn_sink_;
};

/// A transform lookup being built. Created by Tf::lookup.
class Tf::Lookup {
public:
    /// Take the sample nearest `time` rather than the latest one.
    Lookup& at(double time) {
        time_ = time;
        return *this;
    }

    /// Bound how far, in seconds, the chosen sample may sit from at().
    Lookup& tolerance(double tolerance) {
        tolerance_ = tolerance;
        return *this;
    }

    /// Resolve against the transforms buffered so far. Nullopt when no path connects
    /// the frames, or when the nearest sample is outside the tolerance.
    std::optional<Transform> get() {
        std::optional<Transform> found;
        {
            const std::lock_guard<std::mutex> guard(tf_.mutex_);
            found = tf_.resolve_locked(parent_, child_, time_, tolerance_);
        }
        if (!found.has_value()) {
            tf_.warn_unresolved(parent_, child_, time_, tolerance_);
        }
        return found;
    }

    /// Resolve the lookup, blocking up to `timeout` for a late transform. Returns as
    /// soon as the lookup succeeds, or nullopt at the deadline. Calling this on the
    /// module's dispatch thread parks the whole dispatch loop; prefer get() there.
    std::optional<Transform> within(std::chrono::steady_clock::duration timeout) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        std::unique_lock<std::mutex> lock(tf_.mutex_);
        while (true) {
            auto found = tf_.resolve_locked(parent_, child_, time_, tolerance_);
            if (found.has_value()) {
                return found;
            }
            // Only the deadline warns. An intermediate miss is the normal state of a
            // wait, not a failure.
            if (tf_.changed_.wait_until(lock, deadline) == std::cv_status::timeout) {
                auto last = tf_.resolve_locked(parent_, child_, time_, tolerance_);
                lock.unlock();
                if (!last.has_value()) {
                    tf_.warn_unresolved(parent_, child_, time_, tolerance_);
                }
                return last;
            }
        }
    }

private:
    friend class Tf;
    Lookup(Tf& tf, std::string parent, std::string child)
        : tf_(tf), parent_(std::move(parent)), child_(std::move(child)) {}

    Tf& tf_;
    std::string parent_;
    std::string child_;
    std::optional<double> time_;
    std::optional<double> tolerance_;
};

inline Tf::Lookup Tf::lookup(const std::string& parent, const std::string& child) {
    return Lookup(*this, parent, child);
}

inline std::optional<Transform> Tf::get_latest(const std::string& parent,
                                               const std::string& child) {
    return lookup(parent, child).get();
}

}  // namespace tf_client
