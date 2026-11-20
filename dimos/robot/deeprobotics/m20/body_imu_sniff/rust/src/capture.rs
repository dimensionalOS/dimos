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

//! Receive-only raw captures (AF_PACKET) of RTPS datagrams on one interface, filtered in the kernel. They copy frames
//! as tcpdump does: the vendor's sockets still get every datagram, and nothing is ever sent.
//!
//! Fast-DDS sends every SPDP announcement to the domain's multicast port (7400 for domain 0) as well as to each
//! peer's even metatraffic port, and user data to odd ports (7411 + 2n). So one capture takes the multicast port to
//! learn the participant's GUID prefix, and the other takes odd-port datagrams from that prefix only.

use std::ffi::CString;
use std::io;

/// What a capture keeps; datagrams shorter than `min_udp_bytes` (UDP header included) never reach us.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Filter {
    pub port_min: u16,
    pub port_max: u16,
    pub odd_ports: bool,
    pub min_udp_bytes: u16,
    /// Only RTPS messages from this GUID prefix (None: any).
    pub prefix: Option<[u8; 12]>,
}

/// Classic BPF for an Ethernet frame (lo has a zeroed one): IPv4, UDP, first fragment, then `filter`.
pub fn filter_program(filter: &Filter) -> Vec<libc::sock_filter> {
    // (code, k, jumps to drop when true, jumps to drop when false); every other branch falls through
    let mut ops: Vec<(u16, u32, bool, bool)> = vec![
        (0x28, 12, false, false),                         // ldh [12] ethertype
        (0x15, 0x0800, false, true),                      // jeq IPv4
        (0x30, 23, false, false),                         // ldb [23] protocol
        (0x15, 17, false, true),                          // jeq UDP
        (0x28, 20, false, false),                         // ldh [20] fragment offset
        (0x45, 0x1fff, true, false),                      // jset: a later fragment
        (0xb1, 14, false, false),                         // ldxb 4*([14]&0xf) IP header length
        (0x48, 16, false, false),                         // ldh [x+16] destination port
        (0x35, filter.port_min as u32, false, true),      // jge port_min
        (0x25, filter.port_max as u32, true, false),      // jgt port_max
        (0x54, 1, false, false),                          // and #1
        (0x15, filter.odd_ports as u32, false, true),     // jeq parity
        (0x48, 18, false, false),                         // ldh [x+18] UDP length
        (0x35, filter.min_udp_bytes as u32, false, true), // jge min_udp_bytes
    ];
    if let Some(prefix) = filter.prefix {
        // the RTPS header's GUID prefix: UDP payload bytes 8..20
        for (i, word) in prefix.chunks(4).enumerate() {
            ops.push((0x40, 30 + 4 * i as u32, false, false)); // ld [x+30+4i]
            ops.push((
                0x15,
                u32::from_be_bytes(word.try_into().unwrap()),
                false,
                true,
            ));
        }
    }
    let drop_at = ops.len() + 1;
    let mut program: Vec<libc::sock_filter> = ops
        .iter()
        .enumerate()
        .map(|(i, &(code, k, drop_if_true, drop_if_false))| {
            let to_drop = (drop_at - i - 1) as u8;
            libc::sock_filter {
                code,
                jt: if drop_if_true { to_drop } else { 0 },
                jf: if drop_if_false { to_drop } else { 0 },
                k,
            }
        })
        .collect();
    program.push(libc::sock_filter {
        code: 0x06,
        jt: 0,
        jf: 0,
        k: 0x0004_0000,
    }); // accept
    program.push(libc::sock_filter {
        code: 0x06,
        jt: 0,
        jf: 0,
        k: 0,
    }); // drop
    program
}

pub struct Capture {
    fd: libc::c_int,
    buffer: Vec<u8>,
}

impl Capture {
    /// Needs CAP_NET_RAW. Opens on `interface` only.
    pub fn open(interface: &str, filter: &Filter, recv_buffer_bytes: usize) -> io::Result<Self> {
        let protocol = (libc::ETH_P_IP as u16).to_be() as libc::c_int;
        let fd = unsafe {
            libc::socket(
                libc::AF_PACKET,
                libc::SOCK_RAW | libc::SOCK_CLOEXEC,
                protocol,
            )
        };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        let capture = Self {
            fd,
            buffer: vec![0; 65536],
        };
        capture.set_filter(filter)?;
        // loopback hands a packet socket every datagram twice (sent and received): keep the received copy only
        capture.set(libc::SOL_PACKET, libc::PACKET_IGNORE_OUTGOING, 1)?;
        // past net.core.rmem_max only with CAP_NET_ADMIN; without it the kernel's cap applies
        let size = recv_buffer_bytes as libc::c_int;
        if capture
            .set(libc::SOL_SOCKET, libc::SO_RCVBUFFORCE, size)
            .is_err()
        {
            capture.set(libc::SOL_SOCKET, libc::SO_RCVBUF, size)?;
        }
        let name =
            CString::new(interface).map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
        let index = unsafe { libc::if_nametoindex(name.as_ptr()) };
        if index == 0 {
            return Err(io::Error::last_os_error());
        }
        let mut address: libc::sockaddr_ll = unsafe { std::mem::zeroed() };
        address.sll_family = libc::AF_PACKET as u16;
        address.sll_protocol = protocol as u16;
        address.sll_ifindex = index as i32;
        check(unsafe {
            libc::bind(
                fd,
                &address as *const _ as *const libc::sockaddr,
                std::mem::size_of::<libc::sockaddr_ll>() as libc::socklen_t,
            )
        })?;
        Ok(capture)
    }

    /// Swap the kernel filter (atomically: no datagram sees neither).
    pub fn set_filter(&self, filter: &Filter) -> io::Result<()> {
        let mut program = filter_program(filter);
        let fprog = libc::sock_fprog {
            len: program.len() as u16,
            filter: program.as_mut_ptr(),
        };
        check(unsafe {
            libc::setsockopt(
                self.fd,
                libc::SOL_SOCKET,
                libc::SO_ATTACH_FILTER,
                &fprog as *const _ as *const libc::c_void,
                std::mem::size_of::<libc::sock_fprog>() as libc::socklen_t,
            )
        })
    }

    fn set(&self, level: libc::c_int, option: libc::c_int, value: libc::c_int) -> io::Result<()> {
        check(unsafe {
            libc::setsockopt(
                self.fd,
                level,
                option,
                &value as *const _ as *const libc::c_void,
                std::mem::size_of::<libc::c_int>() as libc::socklen_t,
            )
        })
    }

    pub fn fd(&self) -> libc::c_int {
        self.fd
    }

    /// Datagrams the kernel dropped for a full receive buffer since the last call.
    pub fn take_drops(&self) -> u32 {
        let mut stats: libc::tpacket_stats = unsafe { std::mem::zeroed() };
        let mut size = std::mem::size_of::<libc::tpacket_stats>() as libc::socklen_t;
        let result = unsafe {
            libc::getsockopt(
                self.fd,
                libc::SOL_PACKET,
                libc::PACKET_STATISTICS,
                &mut stats as *mut _ as *mut libc::c_void,
                &mut size,
            )
        };
        if result < 0 {
            0
        } else {
            stats.tp_drops
        }
    }

    /// The next datagram's UDP payload (blocks).
    pub fn next_udp_payload(&mut self) -> io::Result<&[u8]> {
        loop {
            let size = unsafe {
                libc::recv(
                    self.fd,
                    self.buffer.as_mut_ptr() as *mut libc::c_void,
                    self.buffer.len(),
                    0,
                )
            };
            if size < 0 {
                return Err(io::Error::last_os_error());
            }
            if let Some((start, end)) = udp_payload_range(&self.buffer[..size as usize]) {
                return Ok(&self.buffer[start..end]);
            }
        }
    }
}

impl Drop for Capture {
    fn drop(&mut self) {
        unsafe { libc::close(self.fd) };
    }
}

fn check(result: libc::c_int) -> io::Result<()> {
    if result < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(())
    }
}

/// Blocks until one of `captures` has a datagram; returns their readiness in order.
pub fn wait_readable(captures: &[&Capture]) -> io::Result<Vec<bool>> {
    let mut fds: Vec<libc::pollfd> = captures
        .iter()
        .map(|c| libc::pollfd {
            fd: c.fd,
            events: libc::POLLIN,
            revents: 0,
        })
        .collect();
    check(unsafe { libc::poll(fds.as_mut_ptr(), fds.len() as libc::nfds_t, -1) })?;
    Ok(fds.iter().map(|f| f.revents & libc::POLLIN != 0).collect())
}

/// Where the UDP payload sits in an Ethernet + IPv4 + UDP frame.
pub fn udp_payload_range(frame: &[u8]) -> Option<(usize, usize)> {
    let ip = frame.get(14..)?;
    if ip.first()? >> 4 != 4 || *ip.get(9)? != 17 {
        return None;
    }
    let header = ((ip[0] & 0x0f) as usize) * 4;
    let udp_length = u16::from_be_bytes([*ip.get(header + 4)?, *ip.get(header + 5)?]) as usize;
    let start = 14 + header + 8;
    let end = (14 + header + udp_length).min(frame.len());
    (start <= end).then_some((start, end))
}

/// The two captures the sniffer reads: discovery (the SPDP multicast port) and data (odd ports, one prefix).
pub struct SniffCaptures {
    pub discovery: Capture,
    pub data: Capture,
    data_filter: Filter,
}

impl SniffCaptures {
    pub fn open(
        interface: &str,
        discovery_port: u16,
        port_min: u16,
        port_max: u16,
        min_udp_bytes: u16,
        recv_buffer_bytes: usize,
    ) -> io::Result<Self> {
        let discovery = Filter {
            port_min: discovery_port,
            port_max: discovery_port,
            odd_ports: discovery_port % 2 == 1,
            min_udp_bytes,
            prefix: None,
        };
        // until the participant is found the data capture takes nothing (an all-zero prefix is never used)
        let data_filter = Filter {
            port_min,
            port_max,
            odd_ports: true,
            min_udp_bytes,
            prefix: Some([0; 12]),
        };
        Ok(Self {
            discovery: Capture::open(interface, &discovery, recv_buffer_bytes)?,
            data: Capture::open(interface, &data_filter, recv_buffer_bytes)?,
            data_filter,
        })
    }

    /// Narrow the data capture to `prefix`'s messages.
    pub fn follow(&mut self, prefix: [u8; 12]) -> io::Result<()> {
        self.data_filter.prefix = Some(prefix);
        self.data.set_filter(&self.data_filter)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Run a classic BPF program over a frame (the subset of opcodes filter_program uses).
    fn run(program: &[libc::sock_filter], frame: &[u8]) -> bool {
        let (mut a, mut x, mut pc) = (0u32, 0u32, 0usize);
        let byte = |at: usize| frame.get(at).copied().unwrap_or(0) as u32;
        loop {
            let op = program[pc];
            let k = op.k as usize;
            let jump = |cond: bool| pc + 1 + if cond { op.jt as usize } else { op.jf as usize };
            pc = match op.code {
                0x28 => {
                    a = byte(k) << 8 | byte(k + 1);
                    pc + 1
                }
                0x30 => {
                    a = byte(k);
                    pc + 1
                }
                0x40 => {
                    let at = x as usize + k;
                    a = byte(at) << 24 | byte(at + 1) << 16 | byte(at + 2) << 8 | byte(at + 3);
                    pc + 1
                }
                0x48 => {
                    let at = x as usize + k;
                    a = byte(at) << 8 | byte(at + 1);
                    pc + 1
                }
                0xb1 => {
                    x = 4 * (byte(k) & 0x0f);
                    pc + 1
                }
                0x54 => {
                    a &= op.k;
                    pc + 1
                }
                0x15 => jump(a == op.k),
                0x25 => jump(a > op.k),
                0x35 => jump(a >= op.k),
                0x45 => jump(a & op.k != 0),
                0x06 => return op.k != 0,
                code => panic!("opcode {code:#x}"),
            };
        }
    }

    fn frame(port: u16, payload: &[u8]) -> Vec<u8> {
        let mut f = vec![0u8; 12];
        f.extend_from_slice(&[
            0x08, 0x00, 0x45, 0, 0, 0, 0, 0, 0, 0, 64, 17, 0, 0, 127, 0, 0, 1, 10, 21, 33, 103,
        ]);
        f.extend_from_slice(&[0x1c, 0x2b]);
        f.extend_from_slice(&port.to_be_bytes());
        f.extend_from_slice(&((payload.len() + 8) as u16).to_be_bytes());
        f.extend_from_slice(&[0, 0]);
        f.extend_from_slice(payload);
        f
    }

    #[test]
    fn filters_by_port_parity_size_and_prefix() {
        let imu = include_bytes!("../tests/fixtures/imu.bin");
        let spdp = include_bytes!("../tests/fixtures/spdp.bin");
        let prefix: [u8; 12] = imu[8..20].try_into().unwrap();
        let base = Filter {
            port_min: 7400,
            port_max: 7700,
            odd_ports: false,
            min_udp_bytes: 200,
            prefix: None,
        };
        let data = Filter {
            odd_ports: true,
            prefix: Some(prefix),
            ..base
        };
        let program = filter_program(&base);
        assert!(run(&program, &frame(7410, spdp)));
        assert!(!run(&program, &frame(7475, imu)), "odd port");
        assert!(!run(&program, &frame(7800, spdp)), "out of range");
        assert!(!run(&program, &frame(7410, &spdp[..100])), "too small");
        let program = filter_program(&data);
        assert!(run(&program, &frame(7475, imu)));
        assert!(
            !run(
                &program,
                &frame(
                    7475,
                    &[imu[..8].to_vec(), vec![0; 12], imu[20..].to_vec()].concat()
                )
            ),
            "other prefix"
        );
        assert!(!run(
            &filter_program(&Filter {
                prefix: Some([0; 12]),
                ..data
            }),
            &frame(7475, imu)
        ));
        let (start, end) = udp_payload_range(&frame(7475, imu)).unwrap();
        assert_eq!(end - start, imu.len());
    }
}
