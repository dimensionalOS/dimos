// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Init-pose value type and quaternion helpers used by FastlioPublisher to
// transform SLAM-origin outputs into the user's chosen world frame.

#ifndef POSE_TRANSFORM_HPP_
#define POSE_TRANSFORM_HPP_

#include <cstddef>
#include <string>

// Hamilton product q_out = q1 * q2.
inline void quat_mul(double ax, double ay, double az, double aw,
                     double bx, double by, double bz, double bw,
                     double& ox, double& oy, double& oz, double& ow) {
    ow = aw*bw - ax*bx - ay*by - az*bz;
    ox = aw*bx + ax*bw + ay*bz - az*by;
    oy = aw*by - ax*bz + ay*bw + az*bx;
    oz = aw*bz + ax*by - ay*bx + az*bw;
}

// Rotate a vector by a unit quaternion: v_out = q * v * q_inv.
inline void quat_rotate(double qx, double qy, double qz, double qw,
                        double vx, double vy, double vz,
                        double& ox, double& oy, double& oz) {
    double tx = 2.0 * (qy*vz - qz*vy);
    double ty = 2.0 * (qz*vx - qx*vz);
    double tz = 2.0 * (qx*vy - qy*vx);
    ox = vx + qw*tx + (qy*tz - qz*ty);
    oy = vy + qw*ty + (qz*tx - qx*tz);
    oz = vz + qw*tz + (qx*ty - qy*tx);
}

struct InitPose {
    double x  = 0.0, y  = 0.0, z  = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;

    bool has_offset() const {
        return x != 0.0 || y != 0.0 || z != 0.0 ||
               qx != 0.0 || qy != 0.0 || qz != 0.0 || qw != 1.0;
    }

    // Parse "x,y,z,qx,qy,qz,qw" CSV. Missing fields keep their defaults
    // (identity rotation, zero translation). Empty input is a no-op.
    void parse(const std::string& csv) {
        if (csv.empty()) return;
        double vals[7] = {0, 0, 0, 0, 0, 0, 1};
        int n = 0;
        std::size_t pos = 0;
        while (pos < csv.size() && n < 7) {
            std::size_t comma = csv.find(',', pos);
            if (comma == std::string::npos) comma = csv.size();
            vals[n++] = std::stod(csv.substr(pos, comma - pos));
            pos = comma + 1;
        }
        x  = vals[0]; y  = vals[1]; z  = vals[2];
        qx = vals[3]; qy = vals[4]; qz = vals[5]; qw = vals[6];
    }
};

#endif  // POSE_TRANSFORM_HPP_
