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
use crate::config::{Ecan, JOINTS};
use crate::types::{JointCommand, RobotState};

pub use crate::frames::*;

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
            let proto = ethertype.to_be() as i32;
            let fd = unsafe {
                libc::socket(libc::AF_PACKET, libc::SOCK_RAW | libc::SOCK_CLOEXEC, proto)
            };
            if fd < 0 {
                return Err(io::Error::last_os_error())
                    .context("AF_PACKET socket (needs CAP_NET_RAW)");
            }
            // owned from here: every error below closes the descriptor
            let mut socket = RawSocket {
                fd,
                ifindex: 0,
                mac: [0; 6],
            };
            let name = CString::new(interface)?;
            let ifindex = unsafe { libc::if_nametoindex(name.as_ptr()) } as i32;
            if ifindex == 0 {
                bail!("no interface {interface}");
            }
            if interface.len() >= libc::IFNAMSIZ {
                bail!("interface name {interface} is too long");
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
            sll.sll_protocol = ethertype.to_be();
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
            socket.ifindex = ifindex;
            socket.mac = mac;
            Ok(socket)
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
                    tv_nsec: left.as_nanos().min(999_999_999) as libc::c_long,
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
    charge: Option<Charge>,
}

/// The charge-enable request on this bus: requests from the dock sequence (charge_manager's /BATTERY_CHARGE_ENABLE,
/// which rl_deploy turns into frames with no rule of its own) and, unless turned off, m20-battery's rule fed by the
/// board's battery frames and the legs' gains.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
pub struct Charge {
    rule: m20_battery::charge::ChargeEnabler,
    rule_on: bool,
    legs_passive: bool,
    // request frames still to send, one per tick in place of a joint command
    pending: usize,
}

impl Charge {
    pub fn new() -> Self {
        Charge {
            rule: m20_battery::charge::ChargeEnabler::new(),
            rule_on: true,
            legs_passive: false,
            pending: 0,
        }
    }

    /// m20-battery's lie-down / battery-report rule on (the default) or off (requests only, as rl_deploy has it).
    pub fn set_rule(&mut self, on: bool) {
        self.rule_on = on;
    }

    /// One request (rl_deploy, per /BATTERY_CHARGE_ENABLE 1): 3 frames on the next 3 ticks, in place of their joint
    /// commands (a request during a burst restarts it).
    pub fn request(&mut self) {
        self.pending = m20_battery::charge::FRAMES_PER_REQUEST;
    }

    /// Note the command about to go out; a switch to no position gain on any leg is a lie-down.
    pub fn on_command(&mut self, cmd: &JointCommand) {
        let passive = (0..JOINTS).filter(|i| i % 4 != 3).all(|i| cmd.kp[i] == 0.0);
        if passive && !self.legs_passive {
            self.rule.lay_down();
        }
        self.legs_passive = passive;
    }

    /// A frame from the board: a battery report may ask for a request.
    pub fn on_frame(&mut self, frame: &[u8]) {
        if !self.rule_on {
            return;
        }
        if let Ok(report) = m20_battery::BatteryReport::from_ethernet(frame) {
            if self.rule.on_report(&report, self.legs_passive)
                == m20_battery::charge::Decision::Request
            {
                self.pending = m20_battery::charge::FRAMES_PER_REQUEST;
            }
        }
    }

    /// Whether this tick sends a request frame instead of the joint command.
    pub fn take_tick(&mut self) -> bool {
        if self.pending == 0 {
            return false;
        }
        self.pending -= 1;
        true
    }
}

impl Default for Charge {
    fn default() -> Self {
        Self::new()
    }
}

/// The full Ethernet frame of one charge-enable request (header only, as rl_deploy sends it).
pub fn encode_charge_enable(seq: u8, src_mac: [u8; 6], cfg: &Ecan) -> Vec<u8> {
    let mut frame = Vec::with_capacity(ETH_HEADER + ECAN_HEADER);
    frame.extend_from_slice(&cfg.destination_mac);
    frame.extend_from_slice(&src_mac);
    frame.extend_from_slice(&cfg.ethertype.to_be_bytes());
    frame.extend_from_slice(&m20_battery::charge::encode_charge_enable(seq));
    frame
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
            charge: cfg.charge_enable.then(Charge::new),
        })
    }

    /// A charge-enable request from the dock sequence (charge_manager writing /BATTERY_CHARGE_ENABLE 1): 3 frames on
    /// the next 3 ticks. Off (no frames) unless [ecan] charge_enable.
    pub fn request_charge_enable(&mut self) {
        if let Some(charge) = self.charge.as_mut() {
            charge.request();
        }
    }

    /// m20-battery's own lie-down / battery-report rule on or off (off when the dock sequence sends the requests).
    pub fn set_charge_rule(&mut self, on: bool) {
        if let Some(charge) = self.charge.as_mut() {
            charge.set_rule(on);
        }
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
            if let Some(charge) = self.charge.as_mut() {
                charge.on_command(cmd);
                // rl_deploy sends the request in place of joint commands; the board does not answer it
                if charge.take_tick() {
                    let frame = encode_charge_enable(self.seq, self.socket.mac, &self.cfg);
                    self.seq = self.seq.wrapping_add(1);
                    self.socket.send(&frame, self.cfg.send_flags)?;
                    if let Some(last) = self.last.clone() {
                        return Ok(last);
                    }
                }
            }
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
                if let Some(charge) = self.charge.as_mut() {
                    charge.on_frame(&frame);
                }
                // a malformed frame (bad checksum, short) is skipped: this tick's reply may still come
                if let Ok(Some(s)) = decode_state(&frame, self.cfg.ethertype, &self.map) {
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

    fn identity() -> Ecan {
        Ecan {
            position_offset: vec![0.0],
            direction: vec![1.0],
            ..Ecan::default()
        }
    }

    #[test]
    fn command_frame_layout() {
        let cfg = identity();
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
    fn gains_truncate_like_rl_deploy() {
        assert_eq!(gain_i16(1.5 * 1.3, 10.0), 19);
        assert_eq!(gain_i16(60.0 * 1.3, 10.0), 780);
        assert_eq!(gain_i16(0.6, 10.0), 6);
    }

    #[test]
    fn default_mapping_is_the_vendor_calibration() {
        let cfg = Ecan::default();
        let map = Mapping::new(&cfg).unwrap();
        let calibration = m20_ecan::Calibration::m20();
        let cmd = JointCommand {
            q: [
                0.1, -0.7, 1.4, 3.0, -0.1, -0.7, 1.4, 3.0, 0.1, 0.7, -1.4, 3.0, -0.1, 0.7, -1.4,
                3.0,
            ],
            ..Default::default()
        };
        let f = encode_command(&cmd, 0, [0; 6], &cfg, &map);
        for (i, joint) in m20_ecan::JointId::all().enumerate() {
            let u = &f[24 + i * 24..24 + (i + 1) * 24];
            let motor = f32::from_le_bytes([u[4], u[5], u[6], u[7]]);
            assert!(
                (calibration.urdf_position(joint, motor) as f64 - cmd.q[i]).abs() < 1e-5,
                "{joint:?}"
            );
        }
    }

    #[test]
    fn charge_enable_frame() {
        let f = encode_charge_enable(0x24, [0xf2, 0x53, 0xbb, 0xa2, 0x93, 0xad], &Ecan::default());
        assert_eq!(f.len(), 24);
        assert_eq!(
            &f[..14],
            &[0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0xf2, 0x53, 0xbb, 0xa2, 0x93, 0xad, 0x44, 0x52]
        );
        assert_eq!(&f[14..], &[0, 0, 0x24, 0, 0x11, 0xb3, 0, 0, 0, 0]);
    }

    #[test]
    fn state_frame_round_trip() {
        let cfg = identity();
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
