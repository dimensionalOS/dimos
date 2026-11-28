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

//! Robot state in, joint commands out, operator commands; all in robot joint order
//! (fl, fr, hl, hr x hipx, hipy, knee, wheel).

use std::collections::BTreeMap;

use serde::Deserialize;

use crate::config::JOINTS;

#[derive(Debug, Clone, Default)]
pub struct RobotState {
    /// body orientation w, x, y, z
    pub quat: [f64; 4],
    /// body angular velocity (rad/s), body frame
    pub gyro: [f64; 3],
    /// accelerometer specific force (m/s^2), body frame; +9.81 on z at rest
    pub accel: [f64; 3],
    pub q: [f64; JOINTS],
    pub dq: [f64; JOINTS],
    pub tau: [f64; JOINTS],
    /// rows * cols relative terrain heights, for height_map observations
    pub height_map: Option<Vec<f64>>,
    /// seconds, monotonic, from the hardware
    pub stamp: f64,
    /// the IMU's own roll / pitch / yaw (degrees, f32) when the source has them (yesense's ImuData): the gravity
    /// observation is then rl_deploy's f32 rotation of them ([`vendor_projected_gravity`]), not `quat`'s
    pub imu_rpy_deg: Option<[f32; 3]>,
    /// the drivers' status words and motor / driver temperatures (deg C) when the source has them (the motor bus):
    /// rl_deploy's SafeController faults on them
    pub drivers: Option<Drivers>,
    /// how old the IMU sample in this state is (s), when the source knows
    pub imu_age_s: Option<f64>,
    /// the battery level from the newest battery report, when the source has them
    pub battery: Option<crate::battery::BatteryLevel>,
}

/// Per joint (robot order) what the motor board reports besides the joint state.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Drivers {
    pub status_word: [u16; JOINTS],
    pub motor_temperature_c: [f32; JOINTS],
    pub driver_temperature_c: [f32; JOINTS],
}

/// Per joint the drivers run tau = kp * (q - q_meas) + kd * (dq - dq_meas) + tau.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct JointCommand {
    pub kp: [f64; JOINTS],
    pub q: [f64; JOINTS],
    pub kd: [f64; JOINTS],
    pub dq: [f64; JOINTS],
    pub tau: [f64; JOINTS],
    /// rl_deploy's SetJointEnable(): an all-zero frame with the enable control word (3 on entering some states)
    pub enable: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum Target {
    #[default]
    Passive,
    Stand,
    RlControl,
    LieDown,
    JointDamping,
}

/// What the operator asks for. axes are normalized [-1, 1] (vx, vy, wz), scaled by each runner's command_max.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct UserCommand {
    #[serde(default)]
    pub target: Target,
    #[serde(default)]
    pub gait: u32,
    #[serde(default)]
    pub axes: [f64; 3],
    /// metric velocity (m/s, m/s, rad/s) in place of axes: normalized by [real_steer] for the commanded gait
    #[serde(default)]
    pub velocity: Option<[f64; 3]>,
    /// `velocity` is basic_server's normalized /STEER (the vendor app's joystick), not a metric /REAL_STEER
    #[serde(default)]
    pub steer: bool,
    #[serde(default)]
    pub params: BTreeMap<String, f64>,
    /// force a runner by name (for runners no gait code selects)
    #[serde(default)]
    pub runner: Option<String>,
    /// counts the operator's motion and gait commands: a change says this command asks for a new target or gait
    /// (velocity-only commands keep it), which is what cancels a docking sequence
    #[serde(default)]
    pub motion_seq: u64,
    /// counts gait commands: a gait sent again (same code) is a new write into rl_deploy's gait slot
    #[serde(default)]
    pub gait_seq: u64,
}

pub fn rotation(q: &[f64; 4]) -> [[f64; 3]; 3] {
    let [w, x, y, z] = *q;
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

/// R^T (0, 0, -1): gravity direction in the body frame.
pub fn projected_gravity(q: &[f64; 4]) -> [f64; 3] {
    let r = rotation(q);
    [-r[2][0], -r[2][1], -r[2][2]]
}

/// rl_deploy's gravity observation from yesense's roll / pitch / yaw (degrees), in its f32 arithmetic:
/// EcanHardwareInterface::HandlerIMU's radians f32(f64(f32(deg / 180)) x pi); RLControlState::UpdateRobotObservation's
/// Eigen AngleAxis(yaw, Z) x AngleAxis(pitch, Y) x AngleAxis(roll, X) (glibc sincosf of half angles, its NEON quaternion
/// product with fused multiply-adds) and toRotationMatrix; the runners' R^T (0, 0, -1) = -(R20, R21, R22).
pub fn vendor_projected_gravity(rpy_deg: [f32; 3]) -> [f32; 3] {
    let rad = |d: f32| ((d / 180.0) as f64 * std::f64::consts::PI) as f32;
    let [r, p, y] = rpy_deg.map(rad);
    // (x, y, z, w) of AngleAxis(angle, axis)
    let aa = |ang: f32, axis: usize| {
        let (s, c) = crate::dock::libm::sincosf(ang * 0.5);
        let mut q = [0.0 * s, 0.0 * s, 0.0 * s, c];
        q[axis] = 1.0 * s;
        q
    };
    // Eigen's quat product l * r as compiled: lanes by TBL masks, fmla / fmls, the w lane's sign flipped
    let mul = |l: [f32; 4], rr: [f32; 4]| {
        let t = |v: [f32; 4], m: [usize; 4]| m.map(|i| v[i]);
        let (m70, m80, m90, ma0, mb0, mc0) = (
            [0, 1, 2, 1],
            [3, 3, 3, 1],
            [2, 0, 1, 2],
            [1, 2, 0, 2],
            [1, 2, 0, 0],
            [2, 0, 1, 0],
        );
        let (r70, l80, la0, r90, rb0, lc0) = (
            t(rr, m70),
            t(l, m80),
            t(l, ma0),
            t(rr, m90),
            t(rr, mb0),
            t(l, mc0),
        );
        let mut out = [0.0f32; 4];
        for i in 0..4 {
            let mut v4 = la0[i].mul_add(r90[i], r70[i] * l80[i]);
            if i == 3 {
                v4 = -v4;
            }
            let v1 = (-rb0[i]).mul_add(lc0[i], l[i] * rr[3]);
            out[i] = v1 + v4;
        }
        out
    };
    let [x, yq, z, w] = mul(mul(aa(y, 2), aa(p, 1)), aa(r, 0));
    let (ty, tx, tz) = (yq + yq, x + x, z + z);
    let r20 = (-ty).mul_add(w, x * tz);
    let r21 = tx.mul_add(w, yq * tz);
    let r22 = 1.0 - x.mul_add(tx, yq * ty);
    [-r20, -r21, -r22]
}

/// Roll and pitch (rad, ZYX Euler) of the body.
pub fn roll_pitch(q: &[f64; 4]) -> (f64, f64) {
    let [w, x, y, z] = *q;
    let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin();
    (roll, pitch)
}

/// Angle between the body z axis and up (rad).
pub fn tilt(q: &[f64; 4]) -> f64 {
    (-projected_gravity(q)[2]).clamp(-1.0, 1.0).acos()
}

/// rl_deploy's accelerometer filter (RLControlState::UpdateRobotObservation, every rl_control tick): each tick's
/// sample into a 20-slot ring (zeros at start), the observation is the ring's mean, in f32: Eigen's sum over the slots
/// in ring order (four lanes, two accumulators, then the lanes pairwise) / 20.
#[derive(Debug, Clone, Default)]
pub struct AccelAverage {
    ring: [[f32; 20]; 3],
    next: usize,
}

impl AccelAverage {
    /// Add this tick's sample; the mean.
    pub fn push(&mut self, accel: [f64; 3]) -> [f64; 3] {
        for (c, ring) in self.ring.iter_mut().enumerate() {
            ring[self.next] = accel[c] as f32;
        }
        self.next = (self.next + 1) % 20;
        let mut out = [0.0; 3];
        for (c, v) in self.ring.iter().enumerate() {
            let lanes = |a: usize| [v[a], v[a + 1], v[a + 2], v[a + 3]];
            let add = |p: [f32; 4], q: [f32; 4]| std::array::from_fn::<f32, 4, _>(|i| p[i] + q[i]);
            let mut p0 = lanes(0);
            let mut p1 = lanes(4);
            p0 = add(p0, lanes(8));
            p1 = add(p1, lanes(12));
            p0 = add(p0, p1);
            p0 = add(p0, lanes(16));
            out[c] = (((p0[0] + p0[1]) + (p0[2] + p0[3])) / 20.0) as f64;
        }
        out
    }
}
