// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Typed C++ helper mirroring the Python `dimos.msgs.nav_msgs.Graph3D`.
// Canonical schema lives in `dimos/msgs/nav_msgs/Graph3D.ksy` — keep
// encode() in sync with that file (and with Graph3D.py.lcm_decode).
//
// Wire format (big-endian):
//
//   uint64 edge_count
//   uint64 node_count
//   double timestamp                 // seconds since epoch
//   per node (node_count):
//     pose_stamped:
//       double ts
//       uint32 frame_id_len
//       bytes  frame_id (utf-8, no terminator)
//       7×double pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w
//     uint64 id
//     uint64 metadata_id
//   per edge (edge_count):
//     uint64 start_id
//     uint64 end_id
//     double timestamp
//     uint64 metadata_id
//
// Edges reference nodes by `id`, not by index.

#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include <lcm/lcm-cpp.hpp>

namespace dimos {

namespace graph3d_detail {

// Host-order → big-endian byte writers. Avoid <endian.h> for portability
// (macOS uses different names) — write byte-by-byte from the top.

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

}  // namespace graph3d_detail

class Graph3D {
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

    struct Edge {
        uint64_t start_id = 0;
        uint64_t end_id = 0;
        double timestamp = 0.0;
        uint64_t metadata_id = 0;
    };

    Graph3D(std::string frame_id, double timestamp)
        : frame_id_(std::move(frame_id)), timestamp_(timestamp) {}

    void reserve_nodes(size_t capacity) { nodes_.reserve(capacity); }
    void reserve_edges(size_t capacity) { edges_.reserve(capacity); }

    // Add a node. The pose's frame_id defaults to the graph's frame_id —
    // override per-node only if a node lives in a different frame.
    void add_node(uint64_t id, uint64_t metadata_id, double pose_ts,
                  double pos_x, double pos_y, double pos_z,
                  double quat_x, double quat_y, double quat_z, double quat_w,
                  std::string node_frame_id = "") {
        PoseStamped pose;
        pose.ts = pose_ts;
        pose.frame_id = node_frame_id.empty() ? frame_id_ : std::move(node_frame_id);
        pose.pos_x = pos_x; pose.pos_y = pos_y; pose.pos_z = pos_z;
        pose.quat_x = quat_x; pose.quat_y = quat_y; pose.quat_z = quat_z; pose.quat_w = quat_w;
        nodes_.push_back({pose, id, metadata_id});
    }

    // Position-only convenience (orientation defaults to identity).
    void add_node_xyz(uint64_t id, uint64_t metadata_id, double pose_ts,
                      double pos_x, double pos_y, double pos_z) {
        add_node(id, metadata_id, pose_ts, pos_x, pos_y, pos_z, 0.0, 0.0, 0.0, 1.0);
    }

    void add_edge(uint64_t start_id, uint64_t end_id, double edge_ts,
                  uint64_t metadata_id = 0) {
        edges_.push_back({start_id, end_id, edge_ts, metadata_id});
    }

    size_t node_count() const { return nodes_.size(); }
    size_t edge_count() const { return edges_.size(); }
    const std::string& frame_id() const { return frame_id_; }

    std::vector<uint8_t> encode() const {
        using namespace graph3d_detail;
        std::vector<uint8_t> out;
        // Conservative reservation: header + per-node fixed bytes + per-edge.
        // frame_id strings add variable length on top — that just causes a
        // realloc, not correctness issues.
        out.reserve(24 + nodes_.size() * 84 + edges_.size() * 32);
        write_u64_be(out, static_cast<uint64_t>(edges_.size()));
        write_u64_be(out, static_cast<uint64_t>(nodes_.size()));
        write_double_be(out, timestamp_);
        for (const auto& n : nodes_) {
            // pose_stamped first (per Graph3D.ksy)
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
            // then id, metadata_id
            write_u64_be(out, n.id);
            write_u64_be(out, n.metadata_id);
        }
        for (const auto& e : edges_) {
            write_u64_be(out, e.start_id);
            write_u64_be(out, e.end_id);
            write_double_be(out, e.timestamp);
            write_u64_be(out, e.metadata_id);
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
    std::vector<Edge> edges_;
};

}  // namespace dimos
