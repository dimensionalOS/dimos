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
}

/// Per joint the drivers run tau = kp * (q - q_meas) + kd * (dq - dq_meas) + tau.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct JointCommand {
    pub kp: [f64; JOINTS],
    pub q: [f64; JOINTS],
    pub kd: [f64; JOINTS],
    pub dq: [f64; JOINTS],
    pub tau: [f64; JOINTS],
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
    #[serde(default)]
    pub params: BTreeMap<String, f64>,
    /// force a runner by name (for runners no gait code selects)
    #[serde(default)]
    pub runner: Option<String>,
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
