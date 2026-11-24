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

//! The Python MuJoCo sim (dimos/simulation/m20_policy_sim/server.py) as a motor bus over TCP, one physics step
//! per command. Protocol in server.py's docstring.

use std::io::{Read, Write};
use std::net::TcpStream;

use anyhow::{bail, Context, Result};

use super::Hardware;
use crate::config::JOINTS;
use crate::types::{JointCommand, RobotState};

const STATE_FIXED: usize = 1 + 4 + 3 + 3 + 3 * JOINTS;

/// Ground truth from the sim, for scoring tests.
#[derive(Debug, Clone, Copy, Default)]
pub struct Truth {
    pub position: [f64; 3],
    /// body vx, vy and yaw rate
    pub velocity: [f64; 3],
    pub tilt: f64,
    pub trunk_contact: bool,
    pub time: f64,
    /// heading (rad, world z)
    pub yaw: f64,
}

pub struct MujocoSim {
    stream: TcpStream,
    /// the motor bus's timing: joints from the newest reply before the previous send, the IMU the newest (default on)
    pub reply_lag: bool,
    lag: crate::hardware::bus::ReplyLag<RobotState>,
    last: Option<RobotState>,
}

impl MujocoSim {
    pub fn connect(addr: &str) -> Result<MujocoSim> {
        let stream =
            TcpStream::connect(addr).with_context(|| format!("connecting to the sim at {addr}"))?;
        stream.set_nodelay(true)?;
        Ok(MujocoSim {
            stream,
            reply_lag: true,
            lag: Default::default(),
            last: None,
        })
    }

    fn read_doubles(&mut self) -> Result<Vec<f64>> {
        let mut len = [0u8; 4];
        self.stream.read_exact(&mut len)?;
        let n = u32::from_le_bytes(len) as usize;
        let mut buf = vec![0u8; n * 8];
        self.stream.read_exact(&mut buf)?;
        Ok(buf
            .as_chunks::<8>()
            .0
            .iter()
            .map(|c| f64::from_le_bytes(*c))
            .collect())
    }

    fn read_robot_state(&mut self) -> Result<RobotState> {
        let v = self.read_doubles()?;
        if v.len() < STATE_FIXED {
            bail!("sim state has {} values", v.len());
        }
        let mut s = RobotState {
            stamp: v[0],
            ..Default::default()
        };
        s.quat.copy_from_slice(&v[1..5]);
        s.gyro.copy_from_slice(&v[5..8]);
        s.accel.copy_from_slice(&v[8..11]);
        s.q.copy_from_slice(&v[11..11 + JOINTS]);
        s.dq.copy_from_slice(&v[11 + JOINTS..11 + 2 * JOINTS]);
        s.tau.copy_from_slice(&v[11 + 2 * JOINTS..STATE_FIXED]);
        if v.len() > STATE_FIXED {
            s.height_map = Some(v[STATE_FIXED..].to_vec());
        }
        Ok(s)
    }

    /// Rebuild the world: request is server.py's RESET JSON (terrain, pose, height_map).
    pub fn reset(&mut self, request: &serde_json::Value) -> Result<RobotState> {
        let body = serde_json::to_vec(request)?;
        self.stream.write_all(&[3])?;
        self.stream.write_all(&(body.len() as u32).to_le_bytes())?;
        self.stream.write_all(&body)?;
        // a reset world has no replies in flight
        self.lag = Default::default();
        self.last = None;
        self.read_robot_state()
    }

    pub fn truth(&mut self) -> Result<Truth> {
        self.stream.write_all(&[4])?;
        let v = self.read_doubles()?;
        if v.len() != 10 {
            bail!("sim truth has {} values", v.len());
        }
        Ok(Truth {
            position: [v[0], v[1], v[2]],
            velocity: [v[3], v[4], v[5]],
            tilt: v[6],
            trunk_contact: v[7] > 0.5,
            time: v[8],
            yaw: v[9],
        })
    }

    /// One lidar sweep (server.py's LIDAR request: mount, beam pattern, noise, seed) as (x, y, z, intensity) in the
    /// lidar's frame.
    pub fn lidar(&mut self, request: &serde_json::Value) -> Result<Vec<[f32; 4]>> {
        let body = serde_json::to_vec(request)?;
        self.stream.write_all(&[5])?;
        self.stream.write_all(&(body.len() as u32).to_le_bytes())?;
        self.stream.write_all(&body)?;
        let v = self.read_doubles()?;
        if v.len() % 4 != 0 {
            bail!("sim lidar sent {} values", v.len());
        }
        Ok(v.as_chunks::<4>()
            .0
            .iter()
            .map(|p| [p[0] as f32, p[1] as f32, p[2] as f32, p[3] as f32])
            .collect())
    }
}

impl Hardware for MujocoSim {
    fn read_state(&mut self) -> Result<RobotState> {
        self.stream.write_all(&[2])?;
        self.read_robot_state()
    }

    fn exchange(&mut self, cmd: &JointCommand) -> Result<RobotState> {
        let mut buf = Vec::with_capacity(1 + 80 * 8);
        buf.push(1u8);
        for arr in [&cmd.kp, &cmd.q, &cmd.kd, &cmd.dq, &cmd.tau] {
            for v in arr.iter() {
                buf.extend_from_slice(&v.to_le_bytes());
            }
        }
        self.stream.write_all(&buf)?;
        if !self.reply_lag {
            return self.read_robot_state();
        }
        if let Some(s) = self.last.take() {
            self.lag.reply(s);
        }
        let before = self.lag.send();
        let now = self.read_robot_state()?;
        self.last = Some(now.clone());
        Ok(match before {
            Some(b) => RobotState {
                quat: now.quat,
                gyro: now.gyro,
                accel: now.accel,
                imu_rpy_deg: now.imu_rpy_deg,
                stamp: now.stamp,
                height_map: now.height_map,
                ..b
            },
            None => now,
        })
    }
}
