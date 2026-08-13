// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Rigid tf tree: frame-name lookup and publishing over a static mount tree.

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

#include <Eigen/Geometry>

namespace dimos::native {

/// A rigid (time-invariant) tf tree mirroring the shape of python's `self.tf`: `get()`
/// resolves parent_from_child across the tree, `publish()` stores an edge and forwards it
/// to the wire via a caller-provided sink (the module wires its tf output port there).
/// Each child has one parent; re-setting an edge overwrites it, which is the right
/// semantics for a mount tree that is static per session but may be republished. Not
/// time-aware.
class TfTree {
public:
    /// Deeper than any physical mount tree; guards get() against tf cycles.
    static constexpr std::size_t kMaxDepth = 32;

    /// Receives every published edge, e.g. to put it on the module's tf output.
    using PublishSink = std::function<void(const std::string& parent, const std::string& child,
                                           const Eigen::Isometry3d& parent_from_child,
                                           std::int64_t timestamp_ns)>;

    void set_publish_sink(PublishSink sink) { sink_ = std::move(sink); }

    /// Store an edge without publishing (tf received from the wire).
    void set(const std::string& parent, const std::string& child,
             const Eigen::Isometry3d& parent_from_child) {
        edges_[child] = {parent, parent_from_child};
    }

    /// Store an edge and forward it to the publish sink, like python's `self.tf.publish`.
    void publish(const std::string& parent, const std::string& child,
                 const Eigen::Isometry3d& parent_from_child, std::int64_t timestamp_ns) {
        set(parent, child, parent_from_child);
        if (sink_) {
            sink_(parent, child, parent_from_child, timestamp_ns);
        }
    }

    bool empty() const { return edges_.empty(); }

    /// parent_from_child through the nearest common ancestor, or nullopt when the two
    /// frames are not connected. The C++ shape of python's `self.tf.get`.
    std::optional<Eigen::Isometry3d> get(const std::string& parent,
                                         const std::string& child) const {
        std::unordered_map<std::string, Eigen::Isometry3d> from_child{
            {child, Eigen::Isometry3d::Identity()}};
        std::string frame = child;
        Eigen::Isometry3d ancestor_from_child = Eigen::Isometry3d::Identity();
        for (std::size_t step = 0; step < kMaxDepth; ++step) {
            auto edge = edges_.find(frame);
            if (edge == edges_.end()) {
                break;
            }
            ancestor_from_child = edge->second.second * ancestor_from_child;
            frame = edge->second.first;
            from_child[frame] = ancestor_from_child;
        }

        frame = parent;
        Eigen::Isometry3d ancestor_from_parent = Eigen::Isometry3d::Identity();
        for (std::size_t step = 0; step <= kMaxDepth; ++step) {
            auto shared = from_child.find(frame);
            if (shared != from_child.end()) {
                return ancestor_from_parent.inverse() * shared->second;
            }
            auto edge = edges_.find(frame);
            if (edge == edges_.end()) {
                return std::nullopt;  // the two are not connected
            }
            ancestor_from_parent = edge->second.second * ancestor_from_parent;
            frame = edge->second.first;
        }
        return std::nullopt;
    }

private:
    /// child -> (parent, parent_from_child)
    std::unordered_map<std::string, std::pair<std::string, Eigen::Isometry3d>> edges_;
    PublishSink sink_;
};

}  // namespace dimos::native
