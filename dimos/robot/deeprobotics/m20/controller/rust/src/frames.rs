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

//! The eth1 motor bus (ECAN) wire format as pure byte codecs: no sockets, nothing that can reach the bus. The
//! sending backend (hardware::ecan, feature "ecan") and the passive shadow (feature "shadow") both build on this.
//!
//! One raw Ethernet frame (ethertype 0x4452) per 1 ms tick: a 10-byte header and 16 x 24-byte JointUnit commands;
//! the board answers ~250 us later with 40 bytes of (unused, zero) IMU data and 16 JointUnit states.

use anyhow::{bail, Result};

use crate::config::{tile, Ecan, JOINTS};
use crate::types::{JointCommand, RobotState};

// protocol constants of the ECAN wire format (decoded from captures, scratchpad ecan/NOTES.md and SPEC.md)
pub const ETH_HEADER: usize = 14;
pub const ECAN_HEADER: usize = 10;
pub const JOINT_UNIT: usize = 24;
pub const MAGIC: u16 = 0x5244;
pub const TYPE_JOINTS: u8 = 0x89;
pub const CODE_COMMAND: u8 = 0x12;
pub const CODE_STATE: u8 = 0x21;
pub const B3_COMMAND: u8 = 1;
pub const STATE_IMU_BYTES: usize = 40;

/// The vendor calibration (DeepRoboticsLab/sdk_deploy m20_interface.hpp init_pos_offset, as m20-ecan has it):
/// URDF angle = motor angle x direction + offset; per joint fl, fr, hl, hr x hipx, hipy, knee, wheel.
pub const VENDOR_OFFSET_DEG: [f64; JOINTS] = [
    -25., -131., 160., 0., 25., -131., 160., 0., -25., 131., -160., 0., 25., 131., -160., 0.,
];
pub const VENDOR_DIRECTION: [f64; JOINTS] = [
    1., 1., -1., 1., 1., -1., 1., -1., -1., 1., -1., 1., -1., -1., 1., -1.,
];

/// (position_offset, direction) in m20_controller.toml's convention (joint = direction x (motor - offset)).
pub fn vendor_calibration() -> (Vec<f64>, Vec<f64>) {
    let offset = (0..JOINTS)
        .map(|i| -(VENDOR_OFFSET_DEG[i] as f32).to_radians() as f64 * VENDOR_DIRECTION[i])
        .collect();
    (offset, VENDOR_DIRECTION.to_vec())
}

/// rl_deploy's joint angle from a motor angle (EcanHardwareInterface::Handler, its FMA): f32(deg / 180 x pi +
/// motor x direction), the sum in f64; hips and knees left as they are (rl_deploy does not wrap them).
pub fn vendor_joint_position(i: usize, motor: f32) -> f32 {
    let deg = VENDOR_OFFSET_DEG[i] as f32 as f64;
    (deg / 180.0).mul_add(
        std::f64::consts::PI,
        (motor * VENDOR_DIRECTION[i] as f32) as f64,
    ) as f32
}

/// rl_deploy's motor angle for a joint target (EcanHardwareInterface::SetJointCommand): (q - offset) x direction in
/// f32, its offset f32(f32(deg / 180) x pi).
pub fn vendor_motor_position(i: usize, q: f32) -> f32 {
    let offset = ((VENDOR_OFFSET_DEG[i] as f32 / 180.0) as f64 * std::f64::consts::PI) as f32;
    (q - offset) * VENDOR_DIRECTION[i] as f32
}

/// Joint index (robot order) of a unit: can_ch = leg 0..3, can_id = 2 hipx, 3 hipy, 4 knee, 5 wheel.
pub fn joint_index(can_ch: u8, can_id: u16) -> Option<usize> {
    if can_ch < 4 && (2..=5).contains(&can_id) {
        Some(can_ch as usize * 4 + (can_id as usize - 2))
    } else {
        None
    }
}

pub struct Mapping {
    pub offset: Vec<f64>,
    pub direction: Vec<f64>,
    pub kp_scale: f64,
    pub kd_scale: f64,
    pub board_id: u8,
    pub control_word: u16,
    pub enable_control_word: u16,
    /// the vendor calibration: positions convert in f32 as rl_deploy converts them
    pub vendor: bool,
}

impl Mapping {
    pub fn new(cfg: &Ecan) -> Result<Mapping> {
        Ok(Mapping {
            offset: tile(&cfg.position_offset, JOINTS)?,
            direction: tile(&cfg.direction, JOINTS)?,
            kp_scale: cfg.kp_scale,
            kd_scale: cfg.kd_scale,
            board_id: cfg.board_id,
            control_word: cfg.control_word,
            enable_control_word: cfg.enable_control_word,
            vendor: {
                let (offset, direction) = vendor_calibration();
                tile(&cfg.position_offset, JOINTS)? == offset
                    && tile(&cfg.direction, JOINTS)? == direction
            },
        })
    }

    fn to_joint(&self, i: usize, motor: f64) -> f64 {
        let q = if self.vendor {
            vendor_joint_position(i, motor as f32) as f64
        } else {
            self.direction[i] * (motor - self.offset[i])
        };
        if self.vendor && (i % 4 == 3 || (-std::f64::consts::PI < q && q <= std::f64::consts::PI)) {
            return q;
        }
        // single-turn hip / knee encoders: the (-pi, pi] branch, as the vendor picks at startup; wheels accumulate
        if i % 4 == 3 {
            q
        } else {
            (q + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
        }
    }
}

/// Like rl_deploy (ECAN2DDS::Handler): f32 multiply, truncate toward zero, saturating.
pub fn gain_i16(v: f64, scale: f64) -> i16 {
    (v as f32 * scale as f32) as i16
}

/// The full Ethernet frame for one joint command (an enable command is raw zeros with the enable control word).
pub fn encode_command(
    cmd: &JointCommand,
    seq: u8,
    src_mac: [u8; 6],
    cfg: &Ecan,
    map: &Mapping,
) -> Vec<u8> {
    let mut payload = Vec::with_capacity(JOINTS * JOINT_UNIT);
    for i in 0..JOINTS {
        let (leg, part) = (i / 4, i % 4);
        let dir = map.direction[i];
        payload.push(map.board_id);
        payload.push(leg as u8);
        payload.extend_from_slice(&(part as u16 + 2).to_le_bytes());
        if cmd.enable {
            payload.extend_from_slice(&[0u8; 16]);
            payload.extend_from_slice(&map.enable_control_word.to_le_bytes());
        } else {
            let motor = if map.vendor {
                vendor_motor_position(i, cmd.q[i] as f32)
            } else {
                (cmd.q[i] / dir + map.offset[i]) as f32
            };
            payload.extend_from_slice(&motor.to_le_bytes());
            payload.extend_from_slice(&((cmd.dq[i] / dir) as f32).to_le_bytes());
            payload.extend_from_slice(&((cmd.tau[i] / dir) as f32).to_le_bytes());
            payload.extend_from_slice(&gain_i16(cmd.kp[i], map.kp_scale).to_le_bytes());
            payload.extend_from_slice(&gain_i16(cmd.kd[i], map.kd_scale).to_le_bytes());
            payload.extend_from_slice(&map.control_word.to_le_bytes());
        }
        payload.extend_from_slice(&0u16.to_le_bytes());
    }
    let checksum = payload.iter().fold(0u32, |a, &b| a + b as u32) as u16;
    let mut frame = Vec::with_capacity(ETH_HEADER + ECAN_HEADER + payload.len());
    frame.extend_from_slice(&cfg.destination_mac);
    frame.extend_from_slice(&src_mac);
    frame.extend_from_slice(&cfg.ethertype.to_be_bytes());
    frame.extend_from_slice(&MAGIC.to_le_bytes());
    frame.extend_from_slice(&[seq, B3_COMMAND, CODE_COMMAND, TYPE_JOINTS]);
    frame.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    frame.extend_from_slice(&checksum.to_le_bytes());
    frame.extend_from_slice(&payload);
    frame
}

/// The ECAN payload of a joint frame with this code, checked; None for other frames.
fn joint_payload(frame: &[u8], ethertype: u16, code: u8, min_len: usize) -> Result<Option<&[u8]>> {
    if frame.len() < ETH_HEADER + ECAN_HEADER
        || u16::from_be_bytes([frame[12], frame[13]]) != ethertype
    {
        return Ok(None);
    }
    let h = &frame[ETH_HEADER..];
    if u16::from_le_bytes([h[0], h[1]]) != MAGIC || h[4] != code || h[5] != TYPE_JOINTS {
        return Ok(None);
    }
    let len = u16::from_le_bytes([h[6], h[7]]) as usize;
    let payload = &h[ECAN_HEADER..];
    if payload.len() < len || len < min_len {
        bail!(
            "short joint frame: {} bytes, header says {len}",
            payload.len()
        );
    }
    let sum = payload[..len].iter().fold(0u32, |a, &b| a + b as u32) as u16;
    if sum != u16::from_le_bytes([h[8], h[9]]) {
        bail!("joint frame checksum mismatch");
    }
    Ok(Some(&payload[..len]))
}

fn f32_at(u: &[u8], o: usize) -> f64 {
    f32::from_le_bytes([u[o], u[o + 1], u[o + 2], u[o + 3]]) as f64
}

/// Joint state from a received frame, or None if it is not a joint-state frame.
pub fn decode_state(frame: &[u8], ethertype: u16, map: &Mapping) -> Result<Option<RobotState>> {
    let Some(payload) = joint_payload(
        frame,
        ethertype,
        CODE_STATE,
        STATE_IMU_BYTES + JOINTS * JOINT_UNIT,
    )?
    else {
        return Ok(None);
    };
    let mut s = RobotState::default();
    let mut seen = 0u32;
    for k in 0..JOINTS {
        let u = &payload[STATE_IMU_BYTES + k * JOINT_UNIT..][..JOINT_UNIT];
        let Some(i) = joint_index(u[1], u16::from_le_bytes([u[2], u[3]])) else {
            continue;
        };
        s.q[i] = map.to_joint(i, f32_at(u, 4));
        s.dq[i] = map.direction[i] * f32_at(u, 8);
        s.tau[i] = map.direction[i] * f32_at(u, 12);
        seen |= 1 << i;
    }
    if seen != 0xffff {
        bail!("joint-state frame is missing joints (mask {seen:#06x})");
    }
    Ok(Some(s))
}

/// A joint command as it went out on the bus, in joint space.
#[derive(Debug, Clone, Copy, Default)]
pub struct SentCommand {
    pub command: JointCommand,
    /// control word of the first joint (4 = command, 2 = enable)
    pub word: u16,
}

/// The joint command of a sent frame, or None if it is not one. An enable frame (raw zeros) comes back with
/// enable = true and zeros, like the controller's own enable command.
pub fn decode_command(frame: &[u8], ethertype: u16, map: &Mapping) -> Result<Option<SentCommand>> {
    let Some(payload) = joint_payload(frame, ethertype, CODE_COMMAND, JOINTS * JOINT_UNIT)? else {
        return Ok(None);
    };
    let mut c = JointCommand::default();
    let mut word = 0u16;
    let mut seen = 0u32;
    for k in 0..JOINTS {
        let u = &payload[k * JOINT_UNIT..][..JOINT_UNIT];
        let Some(i) = joint_index(u[1], u16::from_le_bytes([u[2], u[3]])) else {
            continue;
        };
        let w = u16::from_le_bytes([u[20], u[21]]);
        if k == 0 {
            word = w;
        }
        let dir = map.direction[i];
        c.q[i] = dir * (f32_at(u, 4) - map.offset[i]);
        c.dq[i] = dir * f32_at(u, 8);
        c.tau[i] = dir * f32_at(u, 12);
        c.kp[i] = i16::from_le_bytes([u[16], u[17]]) as f64 / map.kp_scale;
        c.kd[i] = i16::from_le_bytes([u[18], u[19]]) as f64 / map.kd_scale;
        seen |= 1 << i;
    }
    if seen != 0xffff {
        bail!("joint-command frame is missing joints (mask {seen:#06x})");
    }
    if word == map.enable_control_word {
        c = JointCommand {
            enable: true,
            ..Default::default()
        };
    }
    Ok(Some(SentCommand { command: c, word }))
}

/// One frame the ecan_tap.so shim saw inside rl_deploy (m20-ecan's tap record: 32-byte header + frame).
#[derive(Debug, Clone)]
pub struct TapRecord {
    pub timestamp_ns: u64,
    /// true = sent by rl_deploy (to the board), false = received
    pub sent: bool,
    pub ring_dropped: u32,
    pub send_dropped: u32,
    pub frame: Vec<u8>,
}

pub const TAP_MAGIC: &[u8; 4] = b"ETAP";
pub const TAP_HEADER: usize = 32;

/// Every record in one live-stream datagram; stops at the first malformed one.
pub fn decode_tap_datagram(datagram: &[u8]) -> Vec<TapRecord> {
    let mut out = Vec::new();
    let mut at = 0;
    while let Some(h) = datagram.get(at..at + TAP_HEADER) {
        if &h[0..4] != TAP_MAGIC || u16::from_le_bytes([h[4], h[5]]) != 1 {
            break;
        }
        let len = u16::from_le_bytes([h[6], h[7]]) as usize;
        let word = |i: usize| u32::from_le_bytes([h[i], h[i + 1], h[i + 2], h[i + 3]]);
        let sent = match word(8) {
            1 => false,
            2 => true,
            _ => break,
        };
        let Some(frame) = datagram.get(at + TAP_HEADER..at + TAP_HEADER + len) else {
            break;
        };
        out.push(TapRecord {
            timestamp_ns: u64::from(word(24)) | u64::from(word(28)) << 32,
            sent,
            ring_dropped: word(12),
            send_dropped: word(16),
            frame: frame.to_vec(),
        });
        at += TAP_HEADER + len;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_round_trip_and_enable() {
        let cfg = Ecan::default();
        let map = Mapping::new(&cfg).unwrap();
        let mut cmd = JointCommand::default();
        cmd.q[5] = -0.7075;
        cmd.kp[5] = 78.0;
        cmd.kd[5] = 1.95;
        let f = encode_command(&cmd, 7, [1, 2, 3, 4, 5, 6], &cfg, &map);
        let back = decode_command(&f, cfg.ethertype, &map).unwrap().unwrap();
        assert!((back.command.q[5] + 0.7075).abs() < 1e-5);
        assert_eq!(back.command.kp[5], 78.0);
        assert_eq!(back.command.kd[5], 1.9); // truncated like rl_deploy
        assert_eq!(back.word, 4);
        let enable = JointCommand {
            enable: true,
            ..Default::default()
        };
        let f = encode_command(&enable, 8, [1, 2, 3, 4, 5, 6], &cfg, &map);
        assert!(f[ETH_HEADER + ECAN_HEADER + 4..][..16]
            .iter()
            .all(|&b| b == 0));
        let back = decode_command(&f, cfg.ethertype, &map).unwrap().unwrap();
        assert!(back.command.enable);
        assert_eq!(back.word, 2);
    }

    #[test]
    fn tap_datagram() {
        let mut d = Vec::new();
        for (dir, frame) in [(1u32, vec![1u8, 2, 3]), (2, vec![4, 5])] {
            d.extend_from_slice(TAP_MAGIC);
            d.extend_from_slice(&1u16.to_le_bytes());
            d.extend_from_slice(&(frame.len() as u16).to_le_bytes());
            d.extend_from_slice(&dir.to_le_bytes());
            d.extend_from_slice(&[0u8; 12]);
            d.extend_from_slice(&123u64.to_le_bytes());
            d.extend_from_slice(&frame);
        }
        let r = decode_tap_datagram(&d);
        assert_eq!(r.len(), 2);
        assert!(!r[0].sent && r[1].sent);
        assert_eq!(r[1].frame, vec![4, 5]);
        assert_eq!(r[0].timestamp_ns, 123);
    }
}
