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

//! The body IMU and the velocity commands, read off DDS without joining it: a receive-only packet socket per
//! interface (IPv4 only), RTPS DATA submessages picked by port / size (IMU) or writer entity id (commands).
//! The motor bus carries no IPv4, and the seccomp filter (seccomp.rs) refuses any send or write on these sockets.

use std::collections::HashSet;
#[cfg(target_os = "linux")]
use std::sync::mpsc::Sender;

use anyhow::Result;

use super::config::{CommandWriter, ShadowConfig};

#[derive(Debug, Clone, Copy)]
pub struct Imu {
    pub t: f64,
    pub rpy_deg: [f64; 3],
    pub gyro: [f64; 3],
    pub accel: [f64; 3],
}

#[derive(Debug, Clone, Copy)]
pub struct Steer {
    pub t: f64,
    /// x, y, yaw, z, roll, pitch
    pub axes: [f64; 6],
    pub metric: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum Sample {
    Imu(Imu),
    Steer(Steer),
}

/// RTPS DATA serialized payloads with their writer entity id.
pub fn rtps_data(p: &[u8]) -> Vec<([u8; 4], [u8; 8], &[u8])> {
    let mut out = Vec::new();
    if p.len() < 20 || &p[..4] != b"RTPS" {
        return out;
    }
    let mut i = 20;
    while i + 4 <= p.len() {
        let (sid, flags) = (p[i], p[i + 1]);
        let n = if flags & 1 != 0 {
            u16::from_le_bytes([p[i + 2], p[i + 3]])
        } else {
            u16::from_be_bytes([p[i + 2], p[i + 3]])
        } as usize;
        let Some(body) = p.get(i + 4..i + 4 + n) else {
            break;
        };
        // RTPS DATA submessage (protocol constant): reader id, writer id, sequence number, then the payload
        if sid == 0x15 {
            if let (Some(Ok(entity)), Some(Ok(sn)), Some(data)) = (
                body.get(8..12).map(<[u8; 4]>::try_from),
                body.get(12..20).map(<[u8; 8]>::try_from),
                body.get(20..),
            ) {
                out.push((entity, sn, data));
            }
        }
        if n == 0 {
            break;
        }
        i += 4 + n;
    }
    out
}

fn f32s(d: &[u8]) -> Vec<f64> {
    d.as_chunks::<4>()
        .0
        .iter()
        .map(|c| f32::from_le_bytes(*c) as f64)
        .collect()
}

/// u32 at byte o, little-endian, if the slice is long enough.
fn u32_at(d: &[u8], o: usize) -> Option<u32> {
    let b = d.get(o..o + 4)?;
    Some(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

/// Sample ids remembered for duplicate drops (each sample reaches a few readers within microseconds).
const DEDUP_WINDOW: usize = 4096;

pub struct Parser {
    imu_port: u16,
    imu_bytes: usize,
    writers: Vec<([u8; 4], CommandWriter)>,
    seen_imu: HashSet<(u32, u32)>,
    seen_steer: HashSet<([u8; 4], [u8; 8])>,
}

impl Parser {
    pub fn new(cfg: &ShadowConfig) -> Result<Parser> {
        let mut writers = Vec::new();
        for w in &cfg.command_writers {
            let id = u32::from_str_radix(&w.entity, 16)?.to_be_bytes();
            writers.push((id, w.clone()));
        }
        Ok(Parser {
            imu_port: cfg.imu_port,
            imu_bytes: cfg.imu_sample_bytes,
            writers,
            seen_imu: HashSet::new(),
            seen_steer: HashSet::new(),
        })
    }

    /// Samples in one IPv4 packet (each DDS sample goes to several readers: duplicates are dropped).
    pub fn packet(&mut self, t: f64, ip: &[u8]) -> Vec<Sample> {
        let mut out = Vec::new();
        if ip.len() < 28 || ip[0] >> 4 != 4 || ip[9] != 17 {
            return out;
        }
        let ihl = (ip[0] & 15) as usize * 4;
        let Some(udp) = ip.get(ihl..) else { return out };
        if udp.len() < 8 {
            return out;
        }
        let dport = u16::from_be_bytes([udp[2], udp[3]]);
        for (entity, sn, data) in rtps_data(&udp[8..]) {
            if dport == self.imu_port && data.len() == self.imu_bytes {
                // stamp at 12..20, then roll pitch yaw gyro accel as 9 f32 (56 bytes in all)
                let (Some(sec), Some(nsec)) = (u32_at(data, 12), u32_at(data, 16)) else {
                    continue;
                };
                let f = f32s(data.get(20..).unwrap_or_default());
                if f.len() < 9 {
                    continue;
                }
                let stamp = (sec, nsec);
                if self.seen_imu.insert(stamp) {
                    if self.seen_imu.len() > DEDUP_WINDOW {
                        self.seen_imu.clear();
                        self.seen_imu.insert(stamp);
                    }
                    out.push(Sample::Imu(Imu {
                        t,
                        rpy_deg: [f[0], f[1], f[2]],
                        gyro: [f[3], f[4], f[5]],
                        accel: [f[6], f[7], f[8]],
                    }));
                }
                continue;
            }
            let Some((_, w)) = self.writers.iter().find(|(id, _)| *id == entity) else {
                continue;
            };
            if data.len() < 24 || !self.seen_steer.insert((entity, sn)) {
                continue;
            }
            if self.seen_steer.len() > DEDUP_WINDOW {
                self.seen_steer.clear();
            }
            let f = f32s(&data[20..]);
            let get = |i: Option<usize>| i.and_then(|i| f.get(i).copied()).unwrap_or(0.0);
            if f.len() <= w.x.max(w.y).max(w.yaw) {
                continue;
            }
            out.push(Sample::Steer(Steer {
                t,
                axes: [
                    f[w.x],
                    f[w.y],
                    f[w.yaw],
                    get(w.z),
                    get(w.roll),
                    get(w.pitch),
                ],
                metric: w.metric,
            }));
        }
        out
    }
}

#[cfg(target_os = "linux")]
pub mod socket {
    use std::ffi::CString;
    use std::io;

    use anyhow::{bail, Context, Result};

    const ETH_P_IP: u16 = 0x0800;

    /// A receive-only cooked packet socket for IPv4 on one interface.
    pub struct Sniffer {
        pub fd: i32,
    }

    impl Sniffer {
        /// Only UDP to ports lo_port..=hi_port reaches the process (a kernel filter: lidar traffic is never copied).
        pub fn open(interface: &str, lo_port: u16, hi_port: u16) -> Result<Sniffer> {
            let fd = unsafe {
                libc::socket(
                    libc::AF_PACKET,
                    libc::SOCK_DGRAM | libc::SOCK_CLOEXEC,
                    ETH_P_IP.to_be() as i32,
                )
            };
            if fd < 0 {
                return Err(io::Error::last_os_error())
                    .context("AF_PACKET socket (needs CAP_NET_RAW)");
            }
            let name = CString::new(interface)?;
            let ifindex = unsafe { libc::if_nametoindex(name.as_ptr()) } as i32;
            if ifindex == 0 {
                unsafe { libc::close(fd) };
                bail!("no interface {interface}");
            }
            let mut sll: libc::sockaddr_ll = unsafe { std::mem::zeroed() };
            sll.sll_family = libc::AF_PACKET as u16;
            sll.sll_protocol = ETH_P_IP.to_be();
            sll.sll_ifindex = ifindex;
            let r = unsafe {
                libc::bind(
                    fd,
                    &sll as *const _ as *const libc::sockaddr,
                    std::mem::size_of::<libc::sockaddr_ll>() as u32,
                )
            };
            if r < 0 {
                unsafe { libc::close(fd) };
                return Err(io::Error::last_os_error()).context("bind AF_PACKET");
            }
            let sniffer = Sniffer { fd };
            attach_udp_port_filter(fd, lo_port, hi_port)?;
            Ok(sniffer)
        }
    }

    impl Drop for Sniffer {
        fn drop(&mut self) {
            unsafe { libc::close(self.fd) };
        }
    }

    /// Classic BPF on the IP header (SOCK_DGRAM packet sockets start at it): IPv4 UDP, unfragmented, dst port in range.
    fn attach_udp_port_filter(fd: i32, lo: u16, hi: u16) -> Result<()> {
        let s = |code: u16, k: u32| libc::sock_filter {
            code,
            jt: 0,
            jf: 0,
            k,
        };
        let j = |code: u16, k: u32, jt: u8, jf: u8| libc::sock_filter { code, jt, jf, k };
        // ldb [9]; jeq 17 ? next : drop; ldh [6]; jset 0x1fff ? drop : next; ldxb 4*([0]&0xf); ldh [x+2];
        // jge lo ? next : drop; jgt hi ? drop : accept
        // 0 ldb [9]; 1 not UDP -> 8; 2 ldh [6]; 3 fragment -> 8; 4 ldxb 4*([0]&0xf); 5 ldh [x+2];
        // 6 below lo -> 8; 7 above hi -> 8 else 9; 8 drop; 9 accept
        let mut prog = [
            s(0x30, 9),
            j(0x15, 17, 0, 6),
            s(0x28, 6),
            j(0x45, 0x1fff, 4, 0),
            s(0xb1, 0),
            s(0x48, 2),
            j(0x35, lo as u32, 0, 1),
            j(0x25, hi as u32, 0, 1),
            s(0x06, 0),
            s(0x06, 0xffff),
        ];
        let fprog = libc::sock_fprog {
            len: prog.len() as u16,
            filter: prog.as_mut_ptr(),
        };
        let r = unsafe {
            libc::setsockopt(
                fd,
                libc::SOL_SOCKET,
                libc::SO_ATTACH_FILTER,
                &fprog as *const _ as *const libc::c_void,
                std::mem::size_of::<libc::sock_fprog>() as u32,
            )
        };
        if r != 0 {
            return Err(io::Error::last_os_error()).context("SO_ATTACH_FILTER");
        }
        Ok(())
    }

    /// Wait up to timeout_ms (-1: until a packet) on all sniffers; call f(packet) for each one received. EINTR
    /// just returns (the caller polls again).
    pub fn poll_all(
        sniffers: &[Sniffer],
        timeout_ms: i32,
        buf: &mut [u8],
        mut f: impl FnMut(&[u8]),
    ) {
        let mut fds: Vec<libc::pollfd> = sniffers
            .iter()
            .map(|s| libc::pollfd {
                fd: s.fd,
                events: libc::POLLIN,
                revents: 0,
            })
            .collect();
        let r = unsafe { libc::poll(fds.as_mut_ptr(), fds.len() as libc::nfds_t, timeout_ms) };
        if r <= 0 {
            return;
        }
        for p in fds.iter().filter(|p| p.revents & libc::POLLIN != 0) {
            loop {
                let n = unsafe {
                    libc::recv(
                        p.fd,
                        buf.as_mut_ptr() as *mut libc::c_void,
                        buf.len(),
                        libc::MSG_DONTWAIT,
                    )
                };
                if n <= 0 {
                    break;
                }
                f(&buf[..n as usize]);
            }
        }
    }
}

/// Run the sniffer loop forever, sending samples on `tx` (Linux).
#[cfg(target_os = "linux")]
pub fn run(
    sniffers: Vec<socket::Sniffer>,
    mut parser: Parser,
    tx: Sender<Sample>,
    now: impl Fn() -> f64,
) {
    let mut buf = vec![0u8; 65536];
    loop {
        let mut closed = false;
        // blocks until a packet: the thread ends on the first send after the engine is gone
        if sniffers.is_empty() {
            return;
        }
        socket::poll_all(&sniffers, -1, &mut buf, |p| {
            for s in parser.packet(now(), p) {
                if tx.send(s).is_err() {
                    closed = true;
                }
            }
        });
        if closed {
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn shadow_config() -> ShadowConfig {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../shadow.toml");
        ShadowConfig::load(&path).unwrap()
    }

    /// One IPv4 / UDP / RTPS packet to `port` carrying one DATA submessage with this payload.
    fn packet(port: u16, data: &[u8]) -> Vec<u8> {
        let mut rtps = b"RTPS".to_vec();
        rtps.extend_from_slice(&[0u8; 16]);
        rtps.extend_from_slice(&[0x15, 1]);
        rtps.extend_from_slice(&((20 + data.len()) as u16).to_le_bytes());
        rtps.extend_from_slice(&[0u8; 20]);
        rtps.extend_from_slice(data);
        let mut ip = vec![0x45, 0, 0, 0, 0, 0, 0, 0, 64, 17];
        ip.extend_from_slice(&[0u8; 10]);
        ip.extend_from_slice(&[0, 0]);
        ip.extend_from_slice(&port.to_be_bytes());
        ip.extend_from_slice(&[0u8; 4]);
        ip.extend_from_slice(&rtps);
        ip
    }

    #[test]
    fn imu_samples_of_a_configured_short_size_do_not_panic() {
        let mut cfg = shadow_config();
        cfg.imu_sample_bytes = 16;
        let mut p = Parser::new(&cfg).unwrap();
        assert!(p.packet(0.0, &packet(cfg.imu_port, &[0u8; 16])).is_empty());
    }

    #[test]
    fn imu_sample_decodes() {
        let cfg = shadow_config();
        let mut p = Parser::new(&cfg).unwrap();
        let mut data = vec![0u8; 20];
        for v in 1..=9 {
            data.extend_from_slice(&(v as f32).to_le_bytes());
        }
        let out = p.packet(0.0, &packet(cfg.imu_port, &data));
        let [Sample::Imu(imu)] = out.as_slice() else {
            panic!("{out:?}");
        };
        assert_eq!(imu.rpy_deg, [1.0, 2.0, 3.0]);
        assert_eq!(imu.accel, [7.0, 8.0, 9.0]);
    }
}
