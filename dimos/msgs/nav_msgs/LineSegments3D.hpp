// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Typed C++ helper mirroring the Python `dimos.msgs.nav_msgs.LineSegments3D`
// wrapper. Wire format is `nav_msgs::Path` where consecutive `PoseStamped`
// pairs form line segments; `orientation.w` on the first pose of each
// pair carries the segment's `traversability`. The Python
// `LineSegments3D.lcm_decode` reads exactly this layout — keep the two
// in sync.
//
// This type is for *standalone* line segments (e.g., collision-boundary
// polylines). Graph-structured edges with node-id references live in
// `Graph3D` instead.

#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <lcm/lcm-cpp.hpp>

#include "geometry_msgs/PoseStamped.hpp"
#include "nav_msgs/Path.hpp"

#include "dimos_native_module.hpp"

namespace dimos {

class LineSegments3D {
public:
    LineSegments3D(std::string frame_id, double ts)
        : frame_id_(std::move(frame_id)), ts_(ts) {}

    void reserve(size_t capacity) { segments_.reserve(capacity); }

    void add(float x1, float y1, float z1,
             float x2, float y2, float z2,
             float traversability = 1.0f) {
        segments_.push_back({x1, y1, z1, x2, y2, z2, traversability});
    }

    size_t size() const { return segments_.size(); }
    bool empty() const { return segments_.empty(); }

    nav_msgs::Path to_lcm_path() const {
        nav_msgs::Path msg;
        msg.header = make_header(frame_id_, ts_);
        msg.poses_length = static_cast<int32_t>(segments_.size() * 2);
        msg.poses.resize(segments_.size() * 2);
        for (size_t i = 0; i < segments_.size(); ++i) {
            const auto& s = segments_[i];
            auto& p1 = msg.poses[i * 2];
            auto& p2 = msg.poses[i * 2 + 1];
            p1.header = msg.header;
            p2.header = msg.header;
            p1.pose.position.x = s.x1;
            p1.pose.position.y = s.y1;
            p1.pose.position.z = s.z1;
            p1.pose.orientation.x = 0.0;
            p1.pose.orientation.y = 0.0;
            p1.pose.orientation.z = 0.0;
            // orientation.w on the first endpoint carries traversability
            // (see LineSegments3D.py).
            p1.pose.orientation.w = s.traversability;
            p2.pose.position.x = s.x2;
            p2.pose.position.y = s.y2;
            p2.pose.position.z = s.z2;
            p2.pose.orientation.x = 0.0;
            p2.pose.orientation.y = 0.0;
            p2.pose.orientation.z = 0.0;
            p2.pose.orientation.w = s.traversability;
        }
        return msg;
    }

    int publish(lcm::LCM& lcm, const std::string& channel) const {
        nav_msgs::Path msg = to_lcm_path();
        return lcm.publish(channel, &msg);
    }

private:
    struct Segment {
        float x1, y1, z1;
        float x2, y2, z2;
        float traversability;
    };

    std::string frame_id_;
    double ts_;
    std::vector<Segment> segments_;
};

}  // namespace dimos
