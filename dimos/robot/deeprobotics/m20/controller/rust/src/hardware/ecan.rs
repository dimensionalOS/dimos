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

//! The M20's eth1 motor bus (ECAN), as rl_deploy's libecanmaster drives it (decoded from captures; see
//! scratchpad notes summarized in dimos/simulation/m20_policy_sim/SPEC.md). Implemented, never run on the robot.
//!
//! One raw Ethernet frame (ethertype 0x4452) per 1 ms tick: a 10-byte header and 16 x 24-byte JointUnit commands;
//! the board answers ~250 us later with 40 bytes of (unused, zero) IMU data and 16 JointUnit states. The vendor
//! kernel routes these through a private path selected by the 0x80000000 send/recv flag. Body orientation comes
//! from a separate IMU (the yesense), through [ImuSource].

use anyhow::{bail, Result};

use super::Hardware;
use crate::config::{tile, Ecan, JOINTS};
use crate::types::{JointCommand, RobotState};

pub const ETH_HEADER: usize = 14;
pub const ECAN_HEADER: usize = 10;
pub const JOINT_UNIT: usize = 24;
pub const MAGIC: u16 = 0x5244;
pub const TYPE_JOINTS: u8 = 0x89;
pub const CODE_COMMAND: u8 = 0x12;
pub const CODE_STATE: u8 = 0x21;
pub const B3_COMMAND: u8 = 1;
pub const STATE_IMU_BYTES: usize = 40;

/// Body orientation and rates from the IMU, timestamped on the controller's clock.
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuSample {
    pub quat: [f64; 4],
    pub gyro: [f64; 3],
    pub accel: [f64; 3],
    pub stamp: f64,
}

pub trait ImuSource: Send {
    fn latest(&mut self) -> Option<ImuSample>;
}

/// No IMU wired: every state is reported without orientation, so the controller refuses to leave passive safely.
pub struct NoImu;

impl ImuSource for NoImu {
    fn latest(&mut self) -> Option<ImuSample> {
        None
    }
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
    offset: Vec<f64>,
    direction: Vec<f64>,
    kp_scale: f64,
    kd_scale: f64,
    board_id: u8,
    control_word: u16,
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
        })
    }
}

fn gain_i16(v: f64, scale: f64) -> i16 {
    (v * scale).round().clamp(i16::MIN as f64, i16::MAX as f64) as i16
}

/// The full Ethernet frame for one joint command.
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
        payload.extend_from_slice(&((cmd.q[i] / dir + map.offset[i]) as f32).to_le_bytes());
        payload.extend_from_slice(&((cmd.dq[i] / dir) as f32).to_le_bytes());
        payload.extend_from_slice(&((cmd.tau[i] / dir) as f32).to_le_bytes());
        payload.extend_from_slice(&gain_i16(cmd.kp[i], map.kp_scale).to_le_bytes());
        payload.extend_from_slice(&gain_i16(cmd.kd[i], map.kd_scale).to_le_bytes());
        payload.extend_from_slice(&map.control_word.to_le_bytes());
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

/// Joint state from a received frame, or None if it is not a joint-state frame.
pub fn decode_state(frame: &[u8], ethertype: u16, map: &Mapping) -> Result<Option<RobotState>> {
    if frame.len() < ETH_HEADER + ECAN_HEADER
        || u16::from_be_bytes([frame[12], frame[13]]) != ethertype
    {
        return Ok(None);
    }
    let h = &frame[ETH_HEADER..];
    if u16::from_le_bytes([h[0], h[1]]) != MAGIC || h[4] != CODE_STATE || h[5] != TYPE_JOINTS {
        return Ok(None);
    }
    let len = u16::from_le_bytes([h[6], h[7]]) as usize;
    let payload = &h[ECAN_HEADER..];
    if payload.len() < len || len < STATE_IMU_BYTES + JOINTS * JOINT_UNIT {
        bail!(
            "short joint-state frame: {} bytes, header says {len}",
            payload.len()
        );
    }
    let sum = payload[..len].iter().fold(0u32, |a, &b| a + b as u32) as u16;
    if sum != u16::from_le_bytes([h[8], h[9]]) {
        bail!("joint-state checksum mismatch");
    }
    let mut s = RobotState::default();
    let mut seen = 0u32;
    for k in 0..JOINTS {
        let u = &payload[STATE_IMU_BYTES + k * JOINT_UNIT..STATE_IMU_BYTES + (k + 1) * JOINT_UNIT];
        let Some(i) = joint_index(u[1], u16::from_le_bytes([u[2], u[3]])) else {
            continue;
        };
        let f = |o: usize| f32::from_le_bytes([u[o], u[o + 1], u[o + 2], u[o + 3]]) as f64;
        let dir = map.direction[i];
        s.q[i] = dir * (f(4) - map.offset[i]);
        s.dq[i] = dir * f(8);
        s.tau[i] = dir * f(12);
        seen |= 1 << i;
    }
    if seen != 0xffff {
        bail!("joint-state frame is missing joints (mask {seen:#06x})");
    }
    Ok(Some(s))
}

#[cfg(target_os = "linux")]
mod sys {
    use std::ffi::CString;
    use std::io;

    use anyhow::{bail, Context, Result};

    pub struct RawSocket {
        fd: i32,
        ifindex: i32,
        pub mac: [u8; 6],
    }

    fn retry_eintr<F: FnMut() -> isize>(mut f: F) -> io::Result<isize> {
        loop {
            let r = f();
            if r >= 0 {
                return Ok(r);
            }
            let e = io::Error::last_os_error();
            if e.kind() != io::ErrorKind::Interrupted {
                return Err(e);
            }
        }
    }

    impl RawSocket {
        pub fn open(interface: &str, ethertype: u16) -> Result<RawSocket> {
            let proto = (ethertype as u16).to_be() as i32;
            let fd = unsafe { libc::socket(libc::AF_PACKET, libc::SOCK_RAW, proto) };
            if fd < 0 {
                return Err(io::Error::last_os_error())
                    .context("AF_PACKET socket (needs CAP_NET_RAW)");
            }
            let name = CString::new(interface)?;
            let ifindex = unsafe { libc::if_nametoindex(name.as_ptr()) } as i32;
            if ifindex == 0 {
                bail!("no interface {interface}");
            }
            let mut ifr: libc::ifreq = unsafe { std::mem::zeroed() };
            for (d, s) in ifr.ifr_name.iter_mut().zip(interface.as_bytes()) {
                *d = *s as libc::c_char;
            }
            if unsafe { libc::ioctl(fd, libc::SIOCGIFHWADDR, &mut ifr) } < 0 {
                return Err(io::Error::last_os_error()).context("SIOCGIFHWADDR");
            }
            let hw = unsafe { ifr.ifr_ifru.ifru_hwaddr.sa_data };
            let mac = [
                hw[0] as u8,
                hw[1] as u8,
                hw[2] as u8,
                hw[3] as u8,
                hw[4] as u8,
                hw[5] as u8,
            ];
            let mut sll: libc::sockaddr_ll = unsafe { std::mem::zeroed() };
            sll.sll_family = libc::AF_PACKET as u16;
            sll.sll_protocol = (ethertype as u16).to_be();
            sll.sll_ifindex = ifindex;
            let r = unsafe {
                libc::bind(
                    fd,
                    &sll as *const _ as *const libc::sockaddr,
                    std::mem::size_of::<libc::sockaddr_ll>() as u32,
                )
            };
            if r < 0 {
                return Err(io::Error::last_os_error()).context("bind AF_PACKET");
            }
            Ok(RawSocket { fd, ifindex, mac })
        }

        pub fn send(&self, frame: &[u8], flags: u32) -> io::Result<()> {
            let mut sll: libc::sockaddr_ll = unsafe { std::mem::zeroed() };
            sll.sll_family = libc::AF_PACKET as u16;
            sll.sll_ifindex = self.ifindex;
            sll.sll_halen = 6;
            sll.sll_addr[..6].copy_from_slice(&frame[..6]);
            retry_eintr(|| unsafe {
                libc::sendto(
                    self.fd,
                    frame.as_ptr() as *const libc::c_void,
                    frame.len(),
                    flags as i32,
                    &sll as *const _ as *const libc::sockaddr,
                    std::mem::size_of::<libc::sockaddr_ll>() as u32,
                )
            })?;
            Ok(())
        }

        /// One frame, or None once the timeout passes; EINTR never ends the wait.
        pub fn recv(
            &self,
            buf: &mut [u8],
            flags: u32,
            timeout_s: f64,
        ) -> io::Result<Option<usize>> {
            let deadline =
                std::time::Instant::now() + std::time::Duration::from_secs_f64(timeout_s);
            loop {
                let left = deadline.saturating_duration_since(std::time::Instant::now());
                if left.is_zero() {
                    return Ok(None);
                }
                let mut pfd = libc::pollfd {
                    fd: self.fd,
                    events: libc::POLLIN,
                    revents: 0,
                };
                let ts = libc::timespec {
                    tv_sec: 0,
                    tv_nsec: left.as_nanos().min(999_999_999) as i64,
                };
                let r = unsafe { libc::ppoll(&mut pfd, 1, &ts, std::ptr::null()) };
                if r < 0 {
                    let e = io::Error::last_os_error();
                    if e.kind() == io::ErrorKind::Interrupted {
                        continue;
                    }
                    return Err(e);
                }
                if r == 0 {
                    continue;
                }
                let n = retry_eintr(|| unsafe {
                    libc::recvfrom(
                        self.fd,
                        buf.as_mut_ptr() as *mut libc::c_void,
                        buf.len(),
                        flags as i32,
                        std::ptr::null_mut(),
                        std::ptr::null_mut(),
                    )
                })?;
                return Ok(Some(n as usize));
            }
        }
    }

    impl Drop for RawSocket {
        fn drop(&mut self) {
            unsafe { libc::close(self.fd) };
        }
    }
}

#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
pub struct EcanBus {
    cfg: Ecan,
    map: Mapping,
    #[cfg(target_os = "linux")]
    socket: sys::RawSocket,
    imu: Box<dyn ImuSource>,
    seq: u8,
    started: std::time::Instant,
    last: Option<RobotState>,
    buf: Vec<u8>,
}

#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
impl EcanBus {
    pub fn open(cfg: &Ecan, imu: Box<dyn ImuSource>) -> Result<EcanBus> {
        #[cfg(not(target_os = "linux"))]
        {
            let _ = (cfg, imu);
            bail!("the ECAN backend needs Linux (AF_PACKET)");
        }
        #[cfg(target_os = "linux")]
        Ok(EcanBus {
            map: Mapping::new(cfg)?,
            socket: sys::RawSocket::open(&cfg.interface, cfg.ethertype)?,
            cfg: cfg.clone(),
            imu,
            seq: 0,
            started: std::time::Instant::now(),
            last: None,
            buf: vec![0u8; 2048],
        })
    }

    fn with_imu(&mut self, mut s: RobotState) -> Result<RobotState> {
        s.stamp = self.started.elapsed().as_secs_f64();
        let Some(imu) = self.imu.latest() else {
            bail!("no IMU sample")
        };
        s.quat = imu.quat;
        s.gyro = imu.gyro;
        s.accel = imu.accel;
        Ok(s)
    }
}

impl Hardware for EcanBus {
    fn read_state(&mut self) -> Result<RobotState> {
        // the board only answers commands: a zero-gain command reads the joints without moving them
        self.exchange(&JointCommand::default())
    }

    fn exchange(&mut self, cmd: &JointCommand) -> Result<RobotState> {
        #[cfg(target_os = "linux")]
        {
            let frame = encode_command(cmd, self.seq, self.socket.mac, &self.cfg, &self.map);
            self.seq = self.seq.wrapping_add(1);
            self.socket.send(&frame, self.cfg.send_flags)?;
            let deadline = std::time::Instant::now()
                + std::time::Duration::from_secs_f64(self.cfg.reply_timeout_s);
            loop {
                let left = deadline
                    .saturating_duration_since(std::time::Instant::now())
                    .as_secs_f64();
                let Some(n) = self.socket.recv(&mut self.buf, self.cfg.recv_flags, left)? else {
                    bail!("no joint-state reply within {} s", self.cfg.reply_timeout_s);
                };
                let frame = self.buf[..n].to_vec();
                if let Some(s) = decode_state(&frame, self.cfg.ethertype, &self.map)? {
                    let s = self.with_imu(s)?;
                    self.last = Some(s.clone());
                    return Ok(s);
                }
            }
        }
        #[cfg(not(target_os = "linux"))]
        {
            let _ = (cmd, &self.last, &self.seq, &self.buf);
            bail!("the ECAN backend needs Linux")
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_frame_layout() {
        let cfg = Ecan::default();
        let map = Mapping::new(&cfg).unwrap();
        let mut cmd = JointCommand::default();
        cmd.q[5] = 1.25;
        cmd.kp[5] = 60.0;
        cmd.kd[5] = 1.5;
        let f = encode_command(&cmd, 7, [1, 2, 3, 4, 5, 6], &cfg, &map);
        assert_eq!(f.len(), ETH_HEADER + ECAN_HEADER + 384);
        assert_eq!(&f[12..14], &[0x44, 0x52]);
        assert_eq!(&f[14..16], &[0x44, 0x52]);
        assert_eq!(f[16], 7);
        assert_eq!(&f[17..20], &[1, 0x12, 0x89]);
        let u = &f[24 + 5 * 24..24 + 6 * 24];
        assert_eq!((u[0], u[1], u16::from_le_bytes([u[2], u[3]])), (0x79, 1, 3));
        assert_eq!(f32::from_le_bytes([u[4], u[5], u[6], u[7]]), 1.25);
        assert_eq!(i16::from_le_bytes([u[16], u[17]]), 600);
        assert_eq!(i16::from_le_bytes([u[18], u[19]]), 15);
    }

    #[test]
    fn state_frame_round_trip() {
        let cfg = Ecan::default();
        let map = Mapping::new(&cfg).unwrap();
        let mut payload = vec![0u8; STATE_IMU_BYTES];
        for i in 0..JOINTS {
            payload.extend_from_slice(&[0x79, (i / 4) as u8]);
            payload.extend_from_slice(&((i % 4) as u16 + 2).to_le_bytes());
            payload.extend_from_slice(&(i as f32 * 0.1).to_le_bytes());
            payload.extend_from_slice(&(-(i as f32)).to_le_bytes());
            payload.extend_from_slice(&0.5f32.to_le_bytes());
            payload.extend_from_slice(&[0u8; 8]);
        }
        let sum = payload.iter().fold(0u32, |a, &b| a + b as u32) as u16;
        let mut f = vec![0xff; 12];
        f.extend_from_slice(&[0x44, 0x52]);
        f.extend_from_slice(&MAGIC.to_le_bytes());
        f.extend_from_slice(&[0, 6, CODE_STATE, TYPE_JOINTS]);
        f.extend_from_slice(&(payload.len() as u16).to_le_bytes());
        f.extend_from_slice(&sum.to_le_bytes());
        f.extend_from_slice(&payload);
        let s = decode_state(&f, 0x4452, &map).unwrap().unwrap();
        assert!((s.q[9] - 0.9).abs() < 1e-6);
        assert_eq!(s.dq[3], -3.0);
    }
}
