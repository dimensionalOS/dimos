// Fake Livox Mid-360 — replays a recorded pcap over a virtual NIC and synthesizes
// the Livox SDK2 control handshake so an unmodified, live-mode pointlio ingests it
// through the real Livox SDK as if from a live sensor.
//
// Inverse of pointlio's in-process `cpp/pcap_replay.hpp` (--replay_pcap), which
// bypasses the network. This exercises the full live stack: SDK discovery +
// control handshake, then point/IMU UDP off a (virtual) wire.
//
// Runs inside the "lidar" network namespace (see setup_commands()); the unmodified
// pointlio runs in the peer "drv" namespace. On any failure the error names the
// exact command to run, then asks the user to re-run the module.

use dimos_module::{run, LcmTransport, Module};
use serde::Deserialize;
use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use validator::Validate;

// ---- Livox SDK2 control wire format (SdkPacket) ----
const SOF: u8 = 0xAA;
const WRAPPER: usize = 24; // bytes before data[]
const CMD_PORT: u16 = 56100;
const DISCOVERY_PORT: u16 = 56000;
// data plane: lidar src port -> host dst port
const PORT_POINT: u16 = 56300;
const PORT_IMU: u16 = 56400;
const PORT_STATUS: u16 = 56200;
const DST_POINT: u16 = 56301;
const DST_IMU: u16 = 56401;
const DST_STATUS: u16 = 56201;
// Mid-360 multicasts point/IMU data to this group (the SDK joins it). Add a
// route (224.1.1.5/32 dev <lidar-veth>) so it egresses the virtual NIC.
const MCAST_DATA: Ipv4Addr = Ipv4Addr::new(224, 1, 1, 5);
// cmd_id whose ACK means the host finished configuring -> start streaming
const CMD_WORKMODE: u16 = 0x0100;

#[derive(Debug, Deserialize, Validate)]
#[serde(deny_unknown_fields)]
struct Config {
    /// Recorded Mid-360 pcap (data plane: point/IMU/status UDP). Read fully into RAM.
    pcap: String,
    /// Replay-speed multiplier; 1.0 = original inter-packet timing, >1 = faster.
    #[serde(default = "one")]
    #[validate(range(min = 0.01, max = 1000.0))]
    rate: f64,
    /// Seconds to wait after start before streaming begins.
    #[serde(default)]
    #[validate(range(min = 0.0, max = 3600.0))]
    delay: f64,
    /// IP the fake lidar sends from (must be assigned to this netns's veth).
    #[serde(default = "default_lidar_ip")]
    lidar_ip: String,
    /// Host IP the recorded data is delivered to (where pointlio's SDK listens).
    #[serde(default = "default_host_ip")]
    host_ip: String,
    /// Network namespace the fake lidar must run inside.
    #[serde(default = "default_netns")]
    lidar_netns: String,
}

fn one() -> f64 {
    1.0
}
fn default_lidar_ip() -> String {
    "192.168.1.155".into()
}
fn default_host_ip() -> String {
    "192.168.1.5".into()
}
fn default_netns() -> String {
    "lidar".into()
}

#[derive(Module)]
#[module(setup = start)]
struct VirtualMid360 {
    #[config]
    config: Config,
}

// ---- CRCs (Livox SDK2: CRC16-CCITT-FALSE over header[0:18], CRC32/IEEE over data[]) ----
fn crc16_ccitt_false(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

fn crc32_ieee(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

/// Build a Livox SDK2 ACK frame from scratch (synthesized, not replayed):
/// header[0:18] (SOF, version=0, length, seq, cmd_id, cmd_type=1 ACK, sender_type=1)
/// + crc16_h@18 + data[] + crc32_d. `data` is the per-cmd ACK payload.
fn build_ack(cmd_id: u16, seq: u32, data: &[u8]) -> Vec<u8> {
    let length = (WRAPPER + data.len()) as u16;
    let mut f = vec![0u8; WRAPPER + data.len()];
    f[0] = SOF;
    f[1] = 0; // version
    f[2..4].copy_from_slice(&length.to_le_bytes());
    f[4..8].copy_from_slice(&seq.to_le_bytes());
    f[8..10].copy_from_slice(&cmd_id.to_le_bytes());
    f[10] = 1; // cmd_type = ACK
    f[11] = 1; // sender_type = lidar
               // f[12..18] reserved (0)
    let crc16 = crc16_ccitt_false(&f[0..18]);
    f[18..20].copy_from_slice(&crc16.to_le_bytes());
    // f[20..24] = crc32 of data[]
    f[24..].copy_from_slice(data);
    let crc32 = crc32_ieee(data);
    f[20..24].copy_from_slice(&crc32.to_le_bytes());
    f
}

// ---- classic pcap (LE, magic d4c3b2a1) parser -> data-plane UDP packets ----
struct Pkt {
    ts: f64,
    src_port: u16,
    payload: Vec<u8>,
}

fn parse_pcap(path: &str) -> std::io::Result<Vec<Pkt>> {
    let buf = std::fs::read(path)?;
    if buf.len() < 24 || buf[0..4] != [0xd4, 0xc3, 0xb2, 0xa1] {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!("unsupported pcap (need classic little-endian, magic d4c3b2a1) at {path}"),
        ));
    }
    let mut out = Vec::new();
    let mut off = 24usize;
    while off + 16 <= buf.len() {
        let ts_sec = u32::from_le_bytes(buf[off..off + 4].try_into().unwrap());
        let ts_usec = u32::from_le_bytes(buf[off + 4..off + 8].try_into().unwrap());
        let incl = u32::from_le_bytes(buf[off + 8..off + 12].try_into().unwrap()) as usize;
        off += 16;
        if off + incl > buf.len() {
            break;
        }
        let frame = &buf[off..off + incl];
        off += incl;
        // Ethernet(14) -> IPv4 -> UDP
        if frame.len() < 14 + 20 + 8 || frame[12] != 0x08 || frame[13] != 0x00 {
            continue;
        }
        let ihl = ((frame[14] & 0x0f) as usize) * 4;
        if frame[14 + 9] != 17 {
            continue; // not UDP
        }
        let udp = 14 + ihl;
        if frame.len() < udp + 8 {
            continue;
        }
        let src_port = u16::from_be_bytes([frame[udp], frame[udp + 1]]);
        let udp_len = u16::from_be_bytes([frame[udp + 4], frame[udp + 5]]) as usize;
        let payload_start = udp + 8;
        let payload_end = (udp + udp_len).min(frame.len());
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
    // Probe: can we bind the lidar control port on lidar_ip? If not, the veth/netns
    // isn't set up (or we're in the wrong namespace).
    let probe = UdpSocket::bind(SocketAddrV4::new(lidar_ip, CMD_PORT));
    if probe.is_err() {
        let ns = &cfg.lidar_netns;
        let lip = &cfg.lidar_ip;
        let hip = &cfg.host_ip;
        return Err(format!(
            "cannot bind {lip}:{CMD_PORT} — the virtual network interface isn't set up \
             (or this process isn't in the '{ns}' netns).\n\
             Run this once (creates the lidar/drv veth pair), then re-run the module:\n\
             \n  sudo ip netns add drv\n  sudo ip netns add {ns}\n  \
             sudo ip link add veth-drv type veth peer name veth-lidar\n  \
             sudo ip link set veth-drv netns drv\n  \
             sudo ip link set veth-lidar netns {ns}\n  \
             sudo ip netns exec drv   ip addr add {hip}/24 dev veth-drv\n  \
             sudo ip netns exec {ns} ip addr add {lip}/24 dev veth-lidar\n  \
             sudo ip netns exec drv   ip link set veth-drv up\n  \
             sudo ip netns exec {ns} ip link set veth-lidar up\n  \
             sudo ip netns exec drv   ip link set lo up\n  \
             sudo ip netns exec {ns} ip link set lo up\n  \
             sudo ip netns exec drv   ip link set veth-drv multicast on\n  \
             sudo ip netns exec {ns} ip link set veth-lidar multicast on\n  \
             sudo ip netns exec {ns} ip route add 255.255.255.255/32 dev veth-lidar\n  \
             sudo ip netns exec {ns} ip route add 224.1.1.5/32 dev veth-lidar  # point/IMU multicast\n  \
             sudo ip netns exec drv   ip route add 224.0.0.0/4 dev lo  # LCM (dimos transport)\n  \
             sudo ip netns exec {ns} ip route add 224.0.0.0/4 dev lo  # LCM (dimos transport)\n\
             \nThen launch this module inside the lidar netns:\n  \
             sudo ip netns exec {ns} <run the blueprint / binary>"
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
                // Actionable error: print the fix command, then exit non-zero so the
                // coordinator surfaces it and the user can re-run after setup.
                tracing::error!("{msg}");
                eprintln!("\n[virtual_mid360] {msg}\n");
                std::process::exit(2);
            }
        };
        let host_ip: Ipv4Addr = cfg.host_ip.parse().expect("host_ip validated bindable");

        let packets = match parse_pcap(&cfg.pcap) {
            Ok(p) if !p.is_empty() => Arc::new(p),
            Ok(_) => {
                eprintln!(
                    "[virtual_mid360] pcap '{}' has no Livox UDP data packets. \
                     Check the path / that it's a Mid-360 capture, then re-run.",
                    cfg.pcap
                );
                std::process::exit(2);
            }
            Err(e) => {
                eprintln!(
                    "[virtual_mid360] failed to read pcap '{}': {e}. Fix the path, then re-run.",
                    cfg.pcap
                );
                std::process::exit(2);
            }
        };

        let stop = Arc::new(AtomicBool::new(false));
        let armed = Arc::new(AtomicBool::new(false));
        let rate = cfg.rate;
        let delay = cfg.delay;

        // Role 1: discovery responder (:56000 broadcast) — synthesize the 0x0000 ACK.
        spawn_discovery(lidar_ip, stop.clone());
        // Role 2: control responder (:56100) — synthesize per-cmd ACKs; arm streaming
        // when the host issues the work-mode/config command (0x0100).
        spawn_control(lidar_ip, armed.clone(), stop.clone());
        // Role 3: data streamer — point/IMU/status, paced at `rate`, timestamps rewritten
        // to now, armed by the handshake (with `delay` as a startup floor / fallback).
        spawn_stream(lidar_ip, host_ip, packets, rate, delay, armed, stop);
        tracing::info!(lidar = %lidar_ip, host = %host_ip, rate, delay, "virtual_mid360 started");
    }
}

fn spawn_discovery(lidar_ip: Ipv4Addr, stop: Arc<AtomicBool>) {
    std::thread::spawn(move || {
        let sock = match UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, DISCOVERY_PORT)) {
            Ok(s) => s,
            Err(e) => {
                tracing::error!("discovery bind :{DISCOVERY_PORT} failed: {e}");
                return;
            }
        };
        let _ = sock.set_broadcast(true);
        sock.set_read_timeout(Some(Duration::from_millis(500))).ok();
        let bcast = SocketAddrV4::new(Ipv4Addr::BROADCAST, DISCOVERY_PORT);
        let mut buf = [0u8; 2048];
        while !stop.load(Ordering::Relaxed) {
            let n = match sock.recv_from(&mut buf) {
                Ok((n, _)) => n,
                Err(_) => continue,
            };
            if n < WRAPPER || buf[0] != SOF {
                continue;
            }
            let cmd_id = u16::from_le_bytes([buf[8], buf[9]]);
            let cmd_type = buf[10];
            if cmd_id != 0x0000 || cmd_type != 0 {
                continue;
            }
            let seq = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
            // TODO(payload): discovery ACK data describes the device (dev_type, serial,
            // lidar_ip, cmd port). Enumerate the exact layout from livox-sdk2 source.
            let ack = build_ack(0x0000, seq, &discovery_ack_payload(lidar_ip));
            let _ = sock.send_to(&ack, bcast);
        }
    });
}

fn spawn_control(lidar_ip: Ipv4Addr, armed: Arc<AtomicBool>, stop: Arc<AtomicBool>) {
    std::thread::spawn(move || {
        let sock = match UdpSocket::bind(SocketAddrV4::new(lidar_ip, CMD_PORT)) {
            Ok(s) => s,
            Err(e) => {
                tracing::error!("control bind {lidar_ip}:{CMD_PORT} failed: {e}");
                return;
            }
        };
        sock.set_read_timeout(Some(Duration::from_millis(500))).ok();
        let mut buf = [0u8; 2048];
        while !stop.load(Ordering::Relaxed) {
            let (n, from) = match sock.recv_from(&mut buf) {
                Ok(x) => x,
                Err(_) => continue,
            };
            if n < WRAPPER || buf[0] != SOF {
                continue;
            }
            let seq = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
            let cmd_id = u16::from_le_bytes([buf[8], buf[9]]);
            // TODO(payload): per-cmd_id ACK data. Most replies = ret_code(u8)=0 (success);
            // queries echo the requested fields. Enumerate cmd_ids + payloads from
            // livox-sdk2 source (comm/command_impl) or one captured real handshake.
            let ack = build_ack(cmd_id, seq, &control_ack_payload(cmd_id));
            let _ = sock.send_to(&ack, from);
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

fn spawn_stream(
    lidar_ip: Ipv4Addr,
    host_ip: Ipv4Addr,
    packets: Arc<Vec<Pkt>>,
    rate: f64,
    delay: f64,
    armed: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
) {
    std::thread::spawn(move || {
        let mk = |sport: u16| -> std::io::Result<UdpSocket> {
            UdpSocket::bind(SocketAddrV4::new(lidar_ip, sport))
        };
        let (point, imu, status) = match (mk(PORT_POINT), mk(PORT_IMU), mk(PORT_STATUS)) {
            (Ok(a), Ok(b), Ok(c)) => (a, b, c),
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

        // Shift every packet's Livox sensor timestamp by a constant so the first
        // emitted packet reads ≈ now and the original inter-packet spacing (used for
        // intra-scan deskew) is preserved — the stream looks current/live.
        let now_ns = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
        let first_orig = packets
            .iter()
            .find(|p| matches!(p.src_port, PORT_POINT | PORT_IMU))
            .map(|p| read_ts_ns(&p.payload))
            .unwrap_or(0);
        let ts_shift = now_ns.wrapping_sub(first_orig);

        let t_wall0 = Instant::now();
        let mut t_cap0: Option<f64> = None;
        for p in packets.iter() {
            if stop.load(Ordering::Relaxed) {
                break;
            }
            // The Mid-360 MULTICASTS point/IMU to MCAST_DATA:port (the SDK joins that
            // group — confirmed via `ss -ulnp` showing 224.1.1.5:56301/56401); status
            // is unicast to the host. Sending point/IMU unicast is silently dropped.
            let (sock, dst_ip, dst) = match p.src_port {
                PORT_POINT => (&point, MCAST_DATA, DST_POINT),
                PORT_IMU => (&imu, MCAST_DATA, DST_IMU),
                PORT_STATUS => (&status, host_ip, DST_STATUS),
                _ => continue,
            };
            let t0 = *t_cap0.get_or_insert(p.ts);
            let target = (p.ts - t0) / rate;
            let elapsed = t_wall0.elapsed().as_secs_f64();
            if target > elapsed {
                std::thread::sleep(Duration::from_secs_f64(target - elapsed));
            }
            let mut out = p.payload.clone();
            if matches!(p.src_port, PORT_POINT | PORT_IMU) {
                rewrite_ts(&mut out, ts_shift);
            }
            let _ = sock.send_to(&out, SocketAddrV4::new(dst_ip, dst));
        }
        tracing::info!("data stream finished");
    });
}

// ---- payload synthesizers (layouts from Livox-SDK2 sdk_core/comm/define.h) ----
// Mid-360 device type (livox_lidar_def.h: kLivoxLidarTypeMid360 = 9).
const DEV_TYPE_MID360: u8 = 9;

/// Detection/search (0x0000) ACK body == `DetectionData`:
///   ret_code:u8, dev_type:u8, sn[16], lidar_ip[4], cmd_port:u16 LE.
/// The SDK's VerifyNetSegment requires lidar_ip on the host's /24 (192.168.1.x).
fn discovery_ack_payload(lidar_ip: Ipv4Addr) -> Vec<u8> {
    let mut d = Vec::with_capacity(24);
    d.push(0); // ret_code = success
    d.push(DEV_TYPE_MID360);
    // sn[16] MUST be null-terminated within 16 bytes — the SDK treats it as a
    // C-string (strcpy), so a full-16 SN with no NUL overruns its buffer.
    let mut sn = [0u8; 16];
    sn[..10].copy_from_slice(b"FAKEMID360"); // sn[10..]=0 -> NUL-terminated
    d.extend_from_slice(&sn);
    d.extend_from_slice(&lidar_ip.octets());
    d.extend_from_slice(&CMD_PORT.to_le_bytes());
    d
}

// kKeyFwType (livox_lidar_def.h ParamKeyName = 0x8010); fw_type != 0 => app
// firmware (not loader/upgrade mode), so the SDK proceeds to normal operation.
const KEY_FW_TYPE: u16 = 0x8010;
const FW_TYPE_APP: u8 = 1;

/// Control-plane ACK bodies. The SDK casts the SdkPacket data[] directly to the
/// per-cmd response struct, which are #pragma pack(1) (packed, no padding).
fn control_ack_payload(cmd_id: u16) -> Vec<u8> {
    match cmd_id {
        // GetInternalInfo (0x0101): LivoxLidarDiagInternalInfoResponse (packed) —
        //   ret_code:u8 @0, param_num:u16 @1, data @3 (= LivoxLidarKeyValueParam:
        //   key:u16 @0, length:u16 @2, value @4). QueryFwType expects one param
        //   keyed kKeyFwType (0x8010) with a 1-byte fw_type value (non-zero = app).
        0x0101 => {
            let mut d = vec![0u8; 8];
            // d[0] ret_code = 0
            d[1..3].copy_from_slice(&1u16.to_le_bytes()); // param_num = 1
            d[3..5].copy_from_slice(&KEY_FW_TYPE.to_le_bytes());
            d[5..7].copy_from_slice(&1u16.to_le_bytes()); // value length = 1
            d[7] = FW_TYPE_APP;
            d
        }
        // Others: LivoxLidarAsyncControlResponse (packed) { ret_code:u8 @0,
        // error_key:u16 @1 } = 3 bytes. ret_code=0 (success), error_key=0.
        _ => vec![0u8; 3],
    }
}
// LivoxLidarEthernetPacket.timestamp[8] sits at payload offset 28 (packed:
// version@0,len@1,time_interval@3,dot_num@5,udp_cnt@7,frame_cnt@9,data_type@10,
// time_type@11,rsvd@12,crc32@24,timestamp@28). The SDK casts the UDP payload
// directly to LivoxLidarEthernetPacket*, so offset 28 is in the payload.
const TS_OFFSET: usize = 28;

fn read_ts_ns(payload: &[u8]) -> u64 {
    if payload.len() >= TS_OFFSET + 8 {
        u64::from_le_bytes(payload[TS_OFFSET..TS_OFFSET + 8].try_into().unwrap())
    } else {
        0
    }
}

fn rewrite_ts(payload: &mut [u8], shift: u64) {
    if payload.len() >= TS_OFFSET + 8 {
        let orig = u64::from_le_bytes(payload[TS_OFFSET..TS_OFFSET + 8].try_into().unwrap());
        let new = orig.wrapping_add(shift);
        payload[TS_OFFSET..TS_OFFSET + 8].copy_from_slice(&new.to_le_bytes());
    }
}

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new()
        .await
        .expect("Failed to create transport");
    run::<VirtualMid360, _>(transport).await;
}
