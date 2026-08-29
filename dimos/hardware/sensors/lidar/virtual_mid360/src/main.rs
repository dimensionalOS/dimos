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

// Fake Livox Mid-360 — replays a recorded pcap over a virtual NIC and synthesizes
// the Livox SDK2 control handshake so an unmodified, live-mode pointlio ingests it
// through the real Livox SDK as if from a live sensor. Namespace-agnostic: it just
// binds lidar_ip and sends UDP, so it works wherever the host_ip/lidar_ip are
// reachable — IPs aliased on an interface (host ns, incl. macOS lo0) or a netns.

use dimos_livox::wire::{
    self, AsyncControlAck, ControlFrame, DetectionAck, InternalInfoAck, KeyValue,
};
use dimos_module::{native_config, run_with_transport, Module};
use socket2::{Domain, Protocol, Socket, Type};
use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const CMD_PORT: u16 = wire::LIDAR_CMD_PORT;
const DISCOVERY_PORT: u16 = wire::DISCOVERY_PORT;
// data plane: lidar source port -> host destination port
const PORT_POINT: u16 = wire::LIDAR_POINT_PORT;
const PORT_IMU: u16 = wire::LIDAR_IMU_PORT;
const PORT_STATUS: u16 = wire::LIDAR_PUSH_MSG_PORT;
const DST_POINT: u16 = wire::HOST_POINT_PORT;
const DST_IMU: u16 = wire::HOST_IMU_PORT;
const DST_STATUS: u16 = wire::HOST_PUSH_MSG_PORT;
// cmd_id whose ACK means the host finished configuring -> start streaming
const CMD_WORKMODE: u16 = wire::cmd_id::PARAM_SET;

// native_config: every field required + supplied by the Python wrapper over
// stdin (no Rust-side serde defaults / Option). VirtualMid360Config sends all of
// these, so each is unconditionally present. Injects the
// Deserialize/Serialize/Validate derives + deny_unknown_fields + impl NativeConfig.
#[native_config]
struct Config {
    /// Recorded Mid-360 pcap (point/IMU/status UDP). Read fully into RAM.
    pcap: String,
    /// Replay speed; 1.0 = original timing, >1 = faster.
    #[validate(range(min = 0.01, max = 1000.0))]
    rate: f64,
    /// Seconds to wait before streaming begins.
    #[validate(range(min = 0.0, max = 3600.0))]
    delay: f64,
    /// IP the fake lidar sends from.
    lidar_ip: String,
    /// Host IP the data is delivered to (where the SDK listens).
    host_ip: String,
    /// Network namespace the fake lidar runs in. Accepted for wire-config
    /// compatibility but not acted on: the process is *placed* in the netns by
    /// the launcher (`ip netns exec`), so the binary itself stays agnostic.
    #[allow(dead_code)]
    lidar_netns: String,
    /// Multicast group for point/IMU. 224.1.1.5 is the Livox default the SDK
    /// joins; override only to match a differently-configured consumer.
    mcast_data: String,
}

#[derive(Module)]
#[module(setup = start)]
struct VirtualMid360 {
    #[config]
    config: Config,
}

/// Synthesize a Livox SDK2 ACK frame for `data` (per-cmd payload).
fn build_ack(cmd_id: u16, seq: u32, data: &[u8]) -> Vec<u8> {
    wire::build_control(seq, cmd_id, wire::CMD_TYPE_ACK, wire::SENDER_LIDAR, data)
}

// ---- classic pcap (LE, magic d4c3b2a1) parser -> data-plane UDP packets ----
struct Pkt {
    ts: f64,
    src_port: u16,
    payload: Vec<u8>,
}

fn parse_pcap(path: &str) -> std::io::Result<Vec<Pkt>> {
    let buffer = std::fs::read(path)?;
    if buffer.len() < 24 || buffer[0..4] != [0xd4, 0xc3, 0xb2, 0xa1] {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!("unsupported pcap (need classic little-endian, magic d4c3b2a1) at {path}"),
        ));
    }
    let mut out = Vec::new();
    let mut offset = 24usize;
    while offset + 16 <= buffer.len() {
        let ts_sec = u32::from_le_bytes(buffer[offset..offset + 4].try_into().unwrap());
        let ts_usec = u32::from_le_bytes(buffer[offset + 4..offset + 8].try_into().unwrap());
        let captured_len =
            u32::from_le_bytes(buffer[offset + 8..offset + 12].try_into().unwrap()) as usize;
        offset += 16;
        if offset + captured_len > buffer.len() {
            break;
        }
        let frame = &buffer[offset..offset + captured_len];
        offset += captured_len;
        // Ethernet(14) -> IPv4 -> UDP
        if frame.len() < 14 + 20 + 8 || frame[12] != 0x08 || frame[13] != 0x00 {
            continue;
        }
        let ip_header_len = ((frame[14] & 0x0f) as usize) * 4;
        if frame[14 + 9] != 17 {
            continue; // not UDP
        }
        let udp_offset = 14 + ip_header_len;
        if frame.len() < udp_offset + 8 {
            continue;
        }
        let src_port = u16::from_be_bytes([frame[udp_offset], frame[udp_offset + 1]]);
        let udp_len = u16::from_be_bytes([frame[udp_offset + 4], frame[udp_offset + 5]]) as usize;
        let payload_start = udp_offset + 8;
        let payload_end = (udp_offset + udp_len).min(frame.len());
        if payload_end <= payload_start {
            continue;
        }
        out.push(Pkt {
            ts: ts_sec as f64 + ts_usec as f64 / 1e6,
            src_port,
            payload: frame[payload_start..payload_end].to_vec(),
        });
    }
    Ok(out)
}

/// Verify we're in the lidar netns with lidar_ip bindable; else return a helpful
/// error naming the exact `sudo ip netns ...` commands and to re-run.
fn ensure_interface(cfg: &Config) -> Result<Ipv4Addr, String> {
    let lidar_ip: Ipv4Addr = cfg
        .lidar_ip
        .parse()
        .map_err(|_| format!("invalid lidar_ip '{}'", cfg.lidar_ip))?;
    // If we can't bind the control port on lidar_ip, the veth/netns isn't set up
    // (or we're in the wrong namespace).
    let probe = UdpSocket::bind(SocketAddrV4::new(lidar_ip, CMD_PORT));
    if probe.is_err() {
        let lidar_addr = &cfg.lidar_ip;
        let host_addr = &cfg.host_ip;
        let mcast_group = &cfg.mcast_data;
        // The VirtualMid360 module sets the NIC up automatically (setup_network,
        // via sudo); this fires only when that was skipped/failed. Show the
        // by-hand recipe for the current platform.
        let how = if cfg!(target_os = "macos") {
            format!(
                "macOS — alias the IPs onto loopback and route the Livox multicast there:\n  \
                 sudo ifconfig lo0 alias {host_addr} netmask 255.255.255.0\n  \
                 sudo ifconfig lo0 alias {lidar_addr} netmask 255.255.255.0\n  \
                 sudo route -n add -host {mcast_group} -interface lo0\n  \
                 sudo route -n add -host 255.255.255.255 -interface lo0"
            )
        } else {
            format!(
                "Linux — alias the IPs onto a dummy interface (no netns needed):\n  \
                 sudo ip link add dimos-mid360 type dummy\n  \
                 sudo ip addr add {host_addr}/24 dev dimos-mid360\n  \
                 sudo ip addr add {lidar_addr}/24 dev dimos-mid360\n  \
                 sudo ip link set dimos-mid360 up\n  \
                 sudo ip link set dimos-mid360 multicast on\n  \
                 sudo ip route add {mcast_group}/32 dev dimos-mid360\n  \
                 sudo ip route add 255.255.255.255/32 dev dimos-mid360"
            )
        };
        return Err(format!(
            "cannot bind {lidar_addr}:{CMD_PORT} — the virtual NIC isn't set up.\n{how}"
        ));
    }
    Ok(lidar_ip)
}

impl VirtualMid360 {
    async fn start(&mut self) {
        let cfg = &self.config;
        let lidar_ip = match ensure_interface(cfg) {
            Ok(ip) => ip,
            Err(msg) => {
                // Exit non-zero so the coordinator surfaces the fix command.
                tracing::error!("{msg}");
                std::process::exit(2);
            }
        };
        let host_ip: Ipv4Addr = cfg.host_ip.parse().expect("host_ip validated bindable");
        let mcast_data: Ipv4Addr = match cfg.mcast_data.parse() {
            Ok(ip) => ip,
            Err(_) => {
                tracing::error!(
                    "[virtual_mid360] invalid mcast_data '{}' — expected an IPv4 multicast \
                     address matching the consumer's Livox multicast_ip (default 224.1.1.5).",
                    cfg.mcast_data
                );
                std::process::exit(2);
            }
        };

        let packets = match parse_pcap(&cfg.pcap) {
            Ok(parsed) if !parsed.is_empty() => Arc::new(parsed),
            Ok(_) => {
                tracing::error!(
                    "[virtual_mid360] pcap '{}' has no Livox UDP data packets. \
                     Check the path / that it's a Mid-360 capture, then re-run.",
                    cfg.pcap
                );
                std::process::exit(2);
            }
            Err(err) => {
                tracing::error!(
                    "[virtual_mid360] failed to read pcap '{}': {err}. Fix the path, then re-run.",
                    cfg.pcap
                );
                std::process::exit(2);
            }
        };

        let stop = Arc::new(AtomicBool::new(false));
        let armed = Arc::new(AtomicBool::new(false));
        let rate = cfg.rate;
        let delay = cfg.delay;

        // discovery responder (:56000) — proactively announces + answers 0x0000
        spawn_discovery(lidar_ip, host_ip, stop.clone());
        // control responder (:56100) — per-cmd ACKs; arms streaming on 0x0100
        spawn_control(lidar_ip, armed.clone(), stop.clone());
        // data streamer — point/IMU/status paced at `rate`, timestamps shifted to now
        spawn_stream(
            lidar_ip, host_ip, mcast_data, packets, rate, delay, armed, stop,
        );
        tracing::info!(lidar = %lidar_ip, host = %host_ip, rate, delay, "virtual_mid360 started");
    }
}

/// UDP socket bound with SO_REUSEADDR so it can share a port with the consumer
/// SDK's own sockets when both run in one network namespace — macOS (and Linux
/// alias mode) have no netns to separate the two endpoints.
fn reuse_bind(addr: SocketAddrV4) -> std::io::Result<UdpSocket> {
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    socket.set_reuse_address(true)?;
    // SO_REUSEPORT too: the consumer SDK opens its own :56000 sockets (one on
    // INADDR_ANY), and on macOS a wildcard bind can't be added over an existing
    // specific bind with SO_REUSEADDR alone — so without this the two race and
    // whichever loses fails to bind. REUSEPORT makes the binds order-independent.
    socket.set_reuse_port(true)?;
    let bind_addr: std::net::SocketAddr = addr.into();
    socket.bind(&bind_addr.into())?;
    Ok(socket.into())
}

fn spawn_discovery(lidar_ip: Ipv4Addr, host_ip: Ipv4Addr, stop: Arc<AtomicBool>) {
    std::thread::spawn(move || {
        // Bind the lidar's detection port (not INADDR_ANY): SO_REUSEADDR + a
        // specific source IP lets this coexist with the consumer SDK's own
        // :56000 sockets in a shared namespace, and makes our packets arrive
        // *from* lidar_ip:56000 (which is how the SDK identifies the device).
        let socket = match reuse_bind(SocketAddrV4::new(lidar_ip, DISCOVERY_PORT)) {
            Ok(socket) => socket,
            Err(err) => {
                tracing::error!("discovery bind {lidar_ip}:{DISCOVERY_PORT} failed: {err}");
                return;
            }
        };
        socket
            .set_read_timeout(Some(Duration::from_millis(200)))
            .ok();
        // The SDK solicits lidars by broadcasting to 255.255.255.255, which macOS
        // refuses to send — so it can never reach us. Instead we *proactively*
        // unicast the search-ACK to the host's detection port; the SDK accepts an
        // unsolicited detection response (it matches no request seq — none is
        // required for cmd 0x0000) and registers the device. Harmless on Linux,
        // where the broadcast path also works.
        let host_detect = SocketAddrV4::new(host_ip, DISCOVERY_PORT);
        let announce = build_ack(wire::cmd_id::SEARCH, 0, &discovery_ack_payload(lidar_ip));
        let mut buffer = [0u8; 2048];
        while !stop.load(Ordering::Relaxed) {
            let _ = socket.send_to(&announce, host_detect);
            // Also answer a real broadcast solicitation if one arrives, echoing
            // its seq (the original live/netns path).
            if let Ok((len, _)) = socket.recv_from(&mut buffer) {
                if let Ok(frame) = ControlFrame::parse(&buffer[..len]) {
                    if frame.cmd_id == wire::cmd_id::SEARCH
                        && frame.cmd_type == wire::CMD_TYPE_REQUEST
                    {
                        let ack = build_ack(
                            wire::cmd_id::SEARCH,
                            frame.seq,
                            &discovery_ack_payload(lidar_ip),
                        );
                        let _ = socket.send_to(&ack, host_detect);
                    }
                }
            }
        }
    });
}

fn spawn_control(lidar_ip: Ipv4Addr, armed: Arc<AtomicBool>, stop: Arc<AtomicBool>) {
    std::thread::spawn(move || {
        let socket = match UdpSocket::bind(SocketAddrV4::new(lidar_ip, CMD_PORT)) {
            Ok(socket) => socket,
            Err(err) => {
                tracing::error!("control bind {lidar_ip}:{CMD_PORT} failed: {err}");
                return;
            }
        };
        socket
            .set_read_timeout(Some(Duration::from_millis(500)))
            .ok();
        let mut buffer = [0u8; 2048];
        while !stop.load(Ordering::Relaxed) {
            let (len, from) = match socket.recv_from(&mut buffer) {
                Ok(received) => received,
                Err(_) => continue,
            };
            let (seq, cmd_id) = match ControlFrame::parse(&buffer[..len]) {
                Ok(frame) => (frame.seq, frame.cmd_id),
                Err(_) => continue,
            };
            // Per-cmd_id ACK data (control_ack_payload): QueryFwType echoes a
            // key-value param; the rest reply ret_code(u8)=0 (success).
            let ack = build_ack(cmd_id, seq, &control_ack_payload(cmd_id));
            let _ = socket.send_to(&ack, from);
            tracing::info!(
                cmd_id = format!("0x{cmd_id:04x}"),
                seq,
                "control REQ -> ACK"
            );
            if cmd_id == CMD_WORKMODE {
                armed.store(true, Ordering::Relaxed);
                tracing::info!("work-mode cmd 0x0100 acked -> arming data stream");
            }
        }
    });
}

#[allow(clippy::too_many_arguments)]
fn spawn_stream(
    lidar_ip: Ipv4Addr,
    host_ip: Ipv4Addr,
    mcast_data: Ipv4Addr,
    packets: Arc<Vec<Pkt>>,
    rate: f64,
    delay: f64,
    armed: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
) {
    std::thread::spawn(move || {
        let bind_port = |src_port: u16| -> std::io::Result<UdpSocket> {
            UdpSocket::bind(SocketAddrV4::new(lidar_ip, src_port))
        };
        let (point, imu, status) = match (
            bind_port(PORT_POINT),
            bind_port(PORT_IMU),
            bind_port(PORT_STATUS),
        ) {
            (Ok(point_sock), Ok(imu_sock), Ok(status_sock)) => (point_sock, imu_sock, status_sock),
            _ => {
                tracing::error!("failed to bind data-plane source ports on {lidar_ip}");
                return;
            }
        };
        // Wait for handshake to arm streaming, with `delay` as a startup floor + fallback.
        let waited = Instant::now();
        while !armed.load(Ordering::Relaxed) && !stop.load(Ordering::Relaxed) {
            if waited.elapsed().as_secs_f64() >= delay.max(0.0) && delay > 0.0 {
                tracing::warn!("no handshake within delay={delay}s — arming stream anyway");
                break;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        std::thread::sleep(Duration::from_secs_f64(delay.max(0.0)));
        tracing::info!("streaming {} packets at {rate}x", packets.len());

        // Shift every packet's sensor timestamp so the first reads ≈ now,
        // preserving inter-packet spacing — the stream looks live.
        let now_ns = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
        let first_orig = packets
            .iter()
            .find(|pkt| matches!(pkt.src_port, PORT_POINT | PORT_IMU))
            .map(|pkt| read_ts_ns(&pkt.payload))
            .unwrap_or(0);
        let ts_shift = now_ns.wrapping_sub(first_orig);

        // Linux multicasts point/IMU to mcast_data (the SDK joins the group). On
        // macOS the IPs are lo0 aliases and a multicast send source-bound to an
        // alias fails with "No route to host", so we unicast point/IMU to host_ip
        // instead — the SDK identifies the device by the packet's source IP, so
        // the source-bind to lidar_ip (which works for unicast on lo0) is what
        // matters. The consumer's SDK config drops multicast_ip on macOS so its
        // data socket binds host_ip and receives these unicasts. Loopback IPs
        // (host and lidar on 127.0.0.0/8, no NIC setup, no privileges) unicast
        // for the same reason: multicast source-bound to loopback has no route.
        let data_dest = if cfg!(target_os = "macos") || host_ip.is_loopback() {
            host_ip
        } else {
            mcast_data
        };

        let t_wall0 = Instant::now();
        let mut t_cap0: Option<f64> = None;
        for pkt in packets.iter() {
            if stop.load(Ordering::Relaxed) {
                break;
            }
            let (socket, dest_ip, dest_port) = match pkt.src_port {
                PORT_POINT => (&point, data_dest, DST_POINT),
                PORT_IMU => (&imu, data_dest, DST_IMU),
                PORT_STATUS => (&status, host_ip, DST_STATUS),
                _ => continue,
            };
            let t0 = *t_cap0.get_or_insert(pkt.ts);
            let target = (pkt.ts - t0) / rate;
            let elapsed = t_wall0.elapsed().as_secs_f64();
            if target > elapsed {
                std::thread::sleep(Duration::from_secs_f64(target - elapsed));
            }
            let mut out = pkt.payload.clone();
            if matches!(pkt.src_port, PORT_POINT | PORT_IMU) {
                rewrite_ts(&mut out, ts_shift);
            }
            let _ = socket.send_to(&out, SocketAddrV4::new(dest_ip, dest_port));
        }
        tracing::info!("data stream finished");
    });
}

// ---- control ACK payloads ----

/// Detection/search (0x0000) ACK body. The SDK's VerifyNetSegment requires
/// lidar_ip on the host's /24 (192.168.1.x).
fn discovery_ack_payload(lidar_ip: Ipv4Addr) -> Vec<u8> {
    // sn[16] MUST be null-terminated within 16 bytes — the SDK treats it as a
    // C-string (strcpy), so a full-16 SN with no NUL overruns its buffer.
    let mut sn = [0u8; 16];
    sn[..10].copy_from_slice(b"FAKEMID360"); // sn[10..]=0 -> NUL-terminated
    DetectionAck {
        ret_code: 0,
        dev_type: wire::DEVICE_TYPE_MID360,
        sn,
        lidar_ip,
        cmd_port: CMD_PORT,
    }
    .build()
}

/// Control-plane ACK bodies. QueryFwType (0x0101) wants fw_type != 0 => app
/// firmware (not loader/upgrade mode), so the SDK proceeds to normal
/// operation; the rest reply ret_code=0 (success).
fn control_ack_payload(cmd_id: u16) -> Vec<u8> {
    match cmd_id {
        wire::cmd_id::GET_INTERNAL_INFO => InternalInfoAck {
            ret_code: 0,
            params: vec![KeyValue {
                key: wire::param_key::FW_TYPE,
                value: &[wire::FW_TYPE_APP],
            }],
        }
        .build(),
        _ => AsyncControlAck {
            ret_code: 0,
            error_key: 0,
        }
        .build(),
    }
}

fn read_ts_ns(payload: &[u8]) -> u64 {
    wire::read_timestamp_ns(payload).unwrap_or(0)
}

fn rewrite_ts(payload: &mut [u8], shift: u64) {
    wire::shift_timestamp_ns(payload, shift);
}

#[tokio::main]
async fn main() {
    run_with_transport::<VirtualMid360>().await;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn announce_parses_as_a_host_would() {
        let lidar_ip = Ipv4Addr::new(192, 168, 1, 171);
        let announce = build_ack(wire::cmd_id::SEARCH, 0, &discovery_ack_payload(lidar_ip));
        let frame = ControlFrame::parse(&announce).unwrap();
        assert_eq!(frame.cmd_id, wire::cmd_id::SEARCH);
        assert_eq!(frame.cmd_type, wire::CMD_TYPE_ACK);
        assert_eq!(frame.sender_type, wire::SENDER_LIDAR);
        let detection = DetectionAck::parse(frame.data).unwrap();
        assert_eq!(detection.dev_type, wire::DEVICE_TYPE_MID360);
        assert_eq!(detection.lidar_ip, lidar_ip);
        assert_eq!(detection.cmd_port, CMD_PORT);
        assert!(detection.sn.contains(&0), "sn must be NUL-terminated");
    }

    #[test]
    fn fw_type_ack_matches_hand_rolled_layout() {
        // Byte-identical to the previous hand-built GetInternalInfo payload.
        assert_eq!(
            control_ack_payload(wire::cmd_id::GET_INTERNAL_INFO),
            vec![0, 1, 0, 0x10, 0x80, 1, 0, wire::FW_TYPE_APP]
        );
        let payload = control_ack_payload(wire::cmd_id::GET_INTERNAL_INFO);
        let info = InternalInfoAck::parse(&payload).unwrap();
        assert_eq!(info.params[0].key, wire::param_key::FW_TYPE);
        assert_eq!(info.params[0].value, &[wire::FW_TYPE_APP]);
    }

    #[test]
    fn generic_ack_matches_hand_rolled_layout() {
        assert_eq!(control_ack_payload(wire::cmd_id::PARAM_SET), vec![0, 0, 0]);
    }

    #[test]
    fn workmode_request_from_host_is_recognized() {
        let request = wire::build_control(
            7,
            wire::cmd_id::PARAM_SET,
            wire::CMD_TYPE_REQUEST,
            wire::SENDER_HOST,
            &[],
        );
        let frame = ControlFrame::parse(&request).unwrap();
        assert_eq!(frame.cmd_id, CMD_WORKMODE);
        assert_eq!(frame.seq, 7);
    }
}
