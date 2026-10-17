// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Typed C++ helper mirroring the Python `dimos.msgs.nav_msgs.GraphDelta3D`.
//
// Wire format (big-endian):
//
//   uint64 node_count
//   double timestamp                 // seconds since epoch
//   per node (node_count):
//     pose_stamped:                  // (same as Graph3D's node3d pose)
//       double ts
//       uint32 frame_id_len
//       bytes  frame_id (utf-8, no terminator)
//       7×double pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w
//     uint64 id
//     uint64 metadata_id
//   per transform (node_count):
//     7×double translation_x, translation_y, translation_z,
//              rotation_x, rotation_y, rotation_z, rotation_w
//
// Two aligned arrays: ``transforms[i]`` is the SE(3) delta about to
// be applied to ``nodes[i]``. ``post_pose = transforms[i] * nodes[i].pose``
// is the convention (left-multiply).
//
// `GraphDelta3D.py.lcm_decode` reads exactly this layout — keep in sync.

#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include <lcm/lcm-cpp.hpp>

namespace dimos {

namespace graph_delta3d_detail {

inline void write_u32_be(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >>  8) & 0xFF));
    out.push_back(static_cast<uint8_t>( v        & 0xFF));
}

inline void write_u64_be(std::vector<uint8_t>& out, uint64_t v) {
    for (int shift = 56; shift >= 0; shift -= 8) {
        out.push_back(static_cast<uint8_t>((v >> shift) & 0xFF));
    }
}

inline void write_double_be(std::vector<uint8_t>& out, double v) {
    uint64_t bits;
    std::memcpy(&bits, &v, sizeof(bits));
    write_u64_be(out, bits);
}

inline void write_bytes(std::vector<uint8_t>& out, const std::string& s) {
    out.insert(out.end(), s.begin(), s.end());
}

}  // namespace graph_delta3d_detail

class GraphDelta3D {
public:
    struct PoseStamped {
        double ts = 0.0;
        std::string frame_id;
        double pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;
        double quat_x = 0.0, quat_y = 0.0, quat_z = 0.0, quat_w = 1.0;
    };

    struct Node3D {
        PoseStamped pose;
        uint64_t id = 0;
        uint64_t metadata_id = 0;
    };

    struct Transform {
        double translation_x = 0.0, translation_y = 0.0, translation_z = 0.0;
        double rotation_x = 0.0, rotation_y = 0.0, rotation_z = 0.0, rotation_w = 1.0;
    };

    GraphDelta3D(std::string frame_id, double timestamp)
        : frame_id_(std::move(frame_id)), timestamp_(timestamp) {}

    void reserve(size_t capacity) {
        nodes_.reserve(capacity);
        transforms_.reserve(capacity);
    }

    // Add a node + its SE(3) delta. Pass empty `node_frame_id` to inherit
    // the graph's frame_id.
    void add(uint64_t id, uint64_t metadata_id, double pose_ts,
             double pos_x, double pos_y, double pos_z,
             double quat_x, double quat_y, double quat_z, double quat_w,
             double translation_x, double translation_y, double translation_z,
             double rotation_x, double rotation_y, double rotation_z, double rotation_w,
             std::string node_frame_id = "") {
        Node3D node;
        node.id = id;
        node.metadata_id = metadata_id;
        node.pose.ts = pose_ts;
        node.pose.frame_id = node_frame_id.empty() ? frame_id_ : std::move(node_frame_id);
        node.pose.pos_x = pos_x; node.pose.pos_y = pos_y; node.pose.pos_z = pos_z;
        node.pose.quat_x = quat_x; node.pose.quat_y = quat_y;
        node.pose.quat_z = quat_z; node.pose.quat_w = quat_w;
        nodes_.push_back(node);

        Transform tf;
        tf.translation_x = translation_x; tf.translation_y = translation_y; tf.translation_z = translation_z;
        tf.rotation_x = rotation_x; tf.rotation_y = rotation_y;
        tf.rotation_z = rotation_z; tf.rotation_w = rotation_w;
        transforms_.push_back(tf);
    }

    size_t size() const { return nodes_.size(); }
    bool empty() const { return nodes_.empty(); }
    const std::string& frame_id() const { return frame_id_; }

    std::vector<uint8_t> encode() const {
        using namespace graph_delta3d_detail;
        std::vector<uint8_t> out;
        out.reserve(16 + nodes_.size() * (84 + 56));
        write_u64_be(out, static_cast<uint64_t>(nodes_.size()));
        write_double_be(out, timestamp_);
        for (const auto& n : nodes_) {
            write_double_be(out, n.pose.ts);
            write_u32_be(out, static_cast<uint32_t>(n.pose.frame_id.size()));
            write_bytes(out, n.pose.frame_id);
            write_double_be(out, n.pose.pos_x);
            write_double_be(out, n.pose.pos_y);
            write_double_be(out, n.pose.pos_z);
            write_double_be(out, n.pose.quat_x);
            write_double_be(out, n.pose.quat_y);
            write_double_be(out, n.pose.quat_z);
            write_double_be(out, n.pose.quat_w);
            write_u64_be(out, n.id);
            write_u64_be(out, n.metadata_id);
        }
        for (const auto& t : transforms_) {
            write_double_be(out, t.translation_x);
            write_double_be(out, t.translation_y);
            write_double_be(out, t.translation_z);
            write_double_be(out, t.rotation_x);
            write_double_be(out, t.rotation_y);
            write_double_be(out, t.rotation_z);
            write_double_be(out, t.rotation_w);
        }
        return out;
    }

    int publish(lcm::LCM& lcm, const std::string& channel) const {
        std::vector<uint8_t> bytes = encode();
        return lcm.publish(channel, bytes.data(), static_cast<int>(bytes.size()));
    }

private:
    std::string frame_id_;
    double timestamp_;
    std::vector<Node3D> nodes_;
    std::vector<Transform> transforms_;
};

}  // namespace dimos
