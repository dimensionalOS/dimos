// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// ICP cross-check rollback. Triggers on EITHER linear-velocity disagreement
// OR rotational-velocity disagreement between the IESKF and scan-to-scan
// ICP. On trigger: rewind to an anchor pose `rewind_window_ms` back, then
// integrate ICP body-frame linear AND angular velocities forward from there
// — pos = anchor.pos + Σ R_step · v_body·dt, quat = anchor.quat · Π exp(ω_body·dt).
// Overwrite the IESKF state (pos, quat, vel) with the result.
//
// Maintains a ring buffer of per-scan history. Push on every scan.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <deque>

namespace icp_correction {

struct ScanEntry {
    double ts;                          // scan timestamp (sensor-boot s for replay)
    Eigen::Vector3d ieskf_pos;          // IESKF world pos AFTER this scan's update
    Eigen::Quaterniond ieskf_quat;      // IESKF world orientation AFTER this scan's update
    Eigen::Vector3d ieskf_omega_body;   // IESKF body-frame ω this scan (from quat delta)
    Eigen::Vector3d icp_v_body;         // ICP body-frame linear velocity for this scan
    Eigen::Vector3d icp_omega_body;     // ICP body-frame angular velocity for this scan
    bool icp_valid = false;
};

struct Config {
    // Linear trigger: fire when IESKF |v| exceeds this AND ICP is ≥pct slower.
    double only_correct_above_speed_ms = 5.0;
    double only_correct_when_icp_slower_by_pct = 80.0;
    // Angular trigger: fire when |ω_ieskf − ω_icp| exceeds this (deg/s).
    // Zero disables the angular trigger.
    double angular_trigger_gap_deg_s = 30.0;
    // Rewind window for the anchor.
    double rewind_window_ms = 500.0;
};

struct Result {
    bool corrected = false;
    Eigen::Vector3d new_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond new_quat = Eigen::Quaterniond::Identity();
    Eigen::Vector3d new_vel = Eigen::Vector3d::Zero();
    double anchor_ts = 0.0;
    double anchor_age_ms = 0.0;
    double ieskf_v = 0.0;
    double icp_v = 0.0;
    double omega_gap_deg_s = 0.0;
    const char* trigger = "";   // "linear" or "angular" — for the stderr log
};

class Corrector {
public:
    Config cfg;
    std::deque<ScanEntry> history;

    void push(const ScanEntry& e) {
        history.push_back(e);
        const double window_s = cfg.rewind_window_ms / 1000.0;
        while (history.size() > 2 && (e.ts - history.front().ts) > window_s) {
            history.pop_front();
        }
    }

    Result check_and_compute() {
        Result r;
        if (history.size() < 2) return r;
        const ScanEntry& cur = history.back();
        if (!cur.icp_valid) return r;

        const ScanEntry& prev = history[history.size() - 2];
        const double dt_now = cur.ts - prev.ts;
        const double ieskf_v = (dt_now > 0)
            ? (cur.ieskf_pos - prev.ieskf_pos).norm() / dt_now
            : 0.0;
        const double icp_v = cur.icp_v_body.norm();
        r.ieskf_v = ieskf_v;
        r.icp_v = icp_v;
        r.omega_gap_deg_s = (cur.ieskf_omega_body - cur.icp_omega_body).norm()
                            * 180.0 / M_PI;

        bool linear_trigger = false;
        if (ieskf_v > cfg.only_correct_above_speed_ms) {
            const double threshold = ieskf_v * (1.0 - cfg.only_correct_when_icp_slower_by_pct / 100.0);
            if (icp_v < threshold) linear_trigger = true;
        }
        bool angular_trigger = false;
        if (cfg.angular_trigger_gap_deg_s > 0
            && r.omega_gap_deg_s > cfg.angular_trigger_gap_deg_s) {
            angular_trigger = true;
        }
        if (!linear_trigger && !angular_trigger) return r;
        r.trigger = linear_trigger ? "linear" : "angular";

        const double rollback_s = cfg.rewind_window_ms / 1000.0;
        size_t anchor_idx = history.size() - 1;
        for (size_t i = 0; i < history.size(); i++) {
            if (cur.ts - history[i].ts <= rollback_s) {
                anchor_idx = i;
                break;
            }
        }
        if (anchor_idx >= history.size() - 1) {
            // No anchor far enough back — bail.
            return r;
        }
        const ScanEntry& anchor = history[anchor_idx];

        // Integrate ICP angular velocity forward from anchor's orientation
        // (body-frame, post-multiplied). Then integrate ICP linear velocity
        // using the rolling reconstructed orientation.
        Eigen::Quaterniond q = anchor.ieskf_quat;
        Eigen::Vector3d disp = Eigen::Vector3d::Zero();
        for (size_t i = anchor_idx + 1; i < history.size(); i++) {
            const ScanEntry& e = history[i];
            if (!e.icp_valid) continue;
            const double dt = e.ts - history[i - 1].ts;
            if (dt <= 0) continue;
            // Body-frame angular velocity → small-angle quaternion step.
            const Eigen::Vector3d w = e.icp_omega_body * dt;
            const double wn = w.norm();
            Eigen::Quaterniond dq = (wn > 1e-9)
                ? Eigen::Quaterniond(Eigen::AngleAxisd(wn, w / wn))
                : Eigen::Quaterniond::Identity();
            q = q * dq;
            q.normalize();
            disp += q * e.icp_v_body * dt;
        }
        r.new_pos = anchor.ieskf_pos + disp;
        r.new_quat = q;
        // Current world-frame velocity = current ICP body v rotated by the
        // freshly reconstructed orientation.
        r.new_vel = q * cur.icp_v_body;
        r.corrected = true;
        r.anchor_ts = anchor.ts;
        r.anchor_age_ms = (cur.ts - anchor.ts) * 1000.0;
        return r;
    }
};

}  // namespace icp_correction
