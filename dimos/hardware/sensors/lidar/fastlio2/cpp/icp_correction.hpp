// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// ICP cross-check rollback: when the IESKF state's |v| disagrees with
// scan-to-scan ICP by more than a configurable percentage, replay the
// last N ms of ICP body-frame velocities (rotated to world via the
// per-scan IESKF orientation captured at the time) forward from an
// old known-good anchor pose and overwrite the IESKF state.
//
// Maintains a ring buffer of per-scan history. Push on every scan,
// pop oldest entries beyond max_age. Correction triggers off the
// current scan's IESKF vs ICP comparison.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <deque>

namespace icp_correction {

struct ScanEntry {
    double ts;                        // scan timestamp (sensor-boot s for replay)
    Eigen::Vector3d ieskf_pos;        // world-frame pose right AFTER this scan's IESKF update
    Eigen::Quaterniond ieskf_quat;    // world-frame orientation right AFTER this scan's IESKF update
    Eigen::Vector3d icp_v_body;       // ICP-derived body-frame velocity for this scan
    bool icp_valid = false;           // false on the first scan (no previous to ICP against)
};

struct Config {
    // Only fire a correction once the IESKF's |v| exceeds this.
    double only_correct_above_speed_ms = 5.0;
    // Trust ICP and roll back only when its |v| is at least this much less
    // than the IESKF's |v|, expressed as a percentage.
    double only_correct_when_icp_slower_by_pct = 80.0;
    // How far back in time to find the anchor pose we roll back to.
    // Also bounds buffer retention: anything older than this is evicted.
    double rewind_window_ms = 1000.0;
};

struct Result {
    bool corrected = false;
    Eigen::Vector3d new_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d new_vel = Eigen::Vector3d::Zero();
    double anchor_ts = 0.0;
    double anchor_age_ms = 0.0;
    double ieskf_v = 0.0;
    double icp_v = 0.0;
};

class Corrector {
public:
    Config cfg;
    std::deque<ScanEntry> history;

    void push(const ScanEntry& e) {
        history.push_back(e);
        // Evict entries older than the rewind window. Keep at least 2 so we
        // can always look back one step (for the per-step IESKF velocity
        // estimate in check_and_compute).
        const double window_s = cfg.rewind_window_ms / 1000.0;
        while (history.size() > 2 && (e.ts - history.front().ts) > window_s) {
            history.pop_front();
        }
    }

    // Returns a correction if triggered. Caller applies via
    // fast_lio.set_world_pose_vel(new_pos.., new_vel..).
    Result check_and_compute() {
        Result r;
        if (history.size() < 2) return r;
        const ScanEntry& cur = history.back();
        if (!cur.icp_valid) return r;

        // Compute world-frame |v| from IESKF state already captured.
        // (We could pass it in but the caller passes |v| via the last
        // entry's ieskf_pos delta — instead pass it explicitly.)
        // For now, use the per-step IESKF pose delta:
        const ScanEntry& prev = history[history.size() - 2];
        const double dt_now = cur.ts - prev.ts;
        const double ieskf_v = (dt_now > 0)
            ? (cur.ieskf_pos - prev.ieskf_pos).norm() / dt_now
            : 0.0;
        const double icp_v = cur.icp_v_body.norm();
        r.ieskf_v = ieskf_v;
        r.icp_v = icp_v;

        if (ieskf_v <= cfg.only_correct_above_speed_ms) return r;
        // ICP must be at least N% slower than IESKF for us to trust it.
        const double threshold = ieskf_v * (1.0 - cfg.only_correct_when_icp_slower_by_pct / 100.0);
        if (icp_v >= threshold) return r;

        // Find anchor: oldest entry within the rewind window.
        const double rollback_s = cfg.rewind_window_ms / 1000.0;
        size_t anchor_idx = history.size() - 1;
        for (size_t i = 0; i < history.size(); i++) {
            if (cur.ts - history[i].ts <= rollback_s) {
                anchor_idx = i;
                break;
            }
        }
        if (anchor_idx >= history.size() - 1) {
            // Not enough history yet — anchor would be ourselves.
            return r;
        }

        // Integrate ICP body-frame velocities from anchor+1 to now,
        // rotating each by the IESKF world orientation at that scan's time.
        // Use the ieskf_quat captured AT THAT scan as the body→world rotation.
        Eigen::Vector3d disp = Eigen::Vector3d::Zero();
        for (size_t i = anchor_idx + 1; i < history.size(); i++) {
            const ScanEntry& e = history[i];
            if (!e.icp_valid) continue;
            const double dt = e.ts - history[i - 1].ts;
            if (dt <= 0) continue;
            Eigen::Vector3d v_world = e.ieskf_quat * e.icp_v_body;
            disp += v_world * dt;
        }
        const ScanEntry& anchor = history[anchor_idx];
        r.new_pos = anchor.ieskf_pos + disp;
        // New velocity = current ICP body velocity rotated to world
        // using the current orientation (best available).
        r.new_vel = cur.ieskf_quat * cur.icp_v_body;
        r.corrected = true;
        r.anchor_ts = anchor.ts;
        r.anchor_age_ms = (cur.ts - anchor.ts) * 1000.0;
        return r;
    }
};

}  // namespace icp_correction
