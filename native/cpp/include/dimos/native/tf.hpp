// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Static tf tree: frame-name lookup over a rigid mount tree.

#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

#include <Eigen/Geometry>

namespace dimos::native {

/// A rigid (time-invariant) tf tree fed from TransformStamped edges. Each child has one
/// parent; lookup walks both frames to their nearest common ancestor. Not time-aware:
/// re-setting an edge overwrites it, which is the right semantics for a mount tree that
/// is static per session but may be republished.
class StaticTfTree {
public:
    /// Deeper than any physical mount tree; guards lookup against tf cycles.
    static constexpr std::size_t kMaxDepth = 32;

    void set(const std::string& parent, const std::string& child,
             const Eigen::Isometry3d& parent_from_child) {
        edges_[child] = {parent, parent_from_child};
    }

    bool empty() const { return edges_.empty(); }

    /// parent_from_child through the nearest common ancestor, or nullopt when the two
    /// frames are not connected.
    std::optional<Eigen::Isometry3d> lookup(const std::string& parent,
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
};

}  // namespace dimos::native
