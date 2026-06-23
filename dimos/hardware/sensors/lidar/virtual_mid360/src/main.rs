// Fake Livox Mid-360 — replays a recorded pcap over a virtual NIC and synthesizes
// the Livox SDK2 control handshake so an unmodified, live-mode pointlio ingests it
// through the real Livox SDK as if from a live sensor. Namespace-agnostic: it just
// binds lidar_ip and sends UDP, so it works wherever the host_ip/lidar_ip are
// reachable — IPs aliased on an interface (host ns, incl. macOS lo0) or a netns.

use dimos_module::{native_config, run, LcmTransport, Module};
use socket2::{Domain, Protocol, Socket, Type};
use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

// ---- Livox SDK2 control wire format (SdkPacket) ----
const SOF: u8 = 0xAA;
const WRAPPER: usize = 24; // bytes before data[]
const CMD_PORT: u16 = 56100;
const DISCOVERY_PORT: u16 = 56000;
// data plane: lidar source port -> host destination port
const PORT_POINT: u16 = 56300;
const PORT_IMU: u16 = 56400;
const PORT_STATUS: u16 = 56200;
const DST_POINT: u16 = 56301;
const DST_IMU: u16 = 56401;
const DST_STATUS: u16 = 56201;
// cmd_id whose ACK means the host finished configuring -> start streaming
const CMD_WORKMODE: u16 = 0x0100;

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

// ---- CRCs (Livox SDK2: CRC16-CCITT-FALSE over header[0:18], CRC32/IEEE over data[]) ----
fn crc16_ccitt_false(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
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
    for &byte in data {
        crc ^= byte as u32;
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

/// Synthesize a Livox SDK2 ACK frame: 18-byte header (SOF, ver, len, seq, cmd_id,
/// cmd_type=1, sender=1) + crc16@18 + `data` (per-cmd payload) + crc32@20.
fn build_ack(cmd_id: u16, seq: u32, data: &[u8]) -> Vec<u8> {
    let length = (WRAPPER + data.len()) as u16;
    let mut frame = vec![0u8; WRAPPER + data.len()];
    frame[0] = SOF;
    frame[1] = 0; // version
    frame[2..4].copy_from_slice(&length.to_le_bytes());
    frame[4..8].copy_from_slice(&seq.to_le_bytes());
    frame[8..10].copy_from_slice(&cmd_id.to_le_bytes());
    frame[10] = 1; // cmd_type = ACK
    frame[11] = 1; // sender_type = lidar
                   // frame[12..18] reserved (0)
    let crc16 = crc16_ccitt_false(&frame[0..18]);
    frame[18..20].copy_from_slice(&crc16.to_le_bytes());
    // frame[20..24] = crc32 of data[]
    frame[24..].copy_from_slice(data);
    let crc32 = crc32_ieee(data);
    frame[20..24].copy_from_slice(&crc32.to_le_bytes());
    frame
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
        let announce = build_ack(0x0000, 0, &discovery_ack_payload(lidar_ip));
        let mut buffer = [0u8; 2048];
        while !stop.load(Ordering::Relaxed) {
            let _ = socket.send_to(&announce, host_detect);
            // Also answer a real broadcast solicitation if one arrives, echoing
            // its seq (the original live/netns path).
            if let Ok((len, _)) = socket.recv_from(&mut buffer) {
                if len >= WRAPPER
                    && buffer[0] == SOF
                    && u16::from_le_bytes([buffer[8], buffer[9]]) == 0x0000
                    && buffer[10] == 0
                {
                    let seq = u32::from_le_bytes([buffer[4], buffer[5], buffer[6], buffer[7]]);
                    let ack = build_ack(0x0000, seq, &discovery_ack_payload(lidar_ip));
                    let _ = socket.send_to(&ack, host_detect);
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
            if len < WRAPPER || buffer[0] != SOF {
                continue;
            }
            let seq = u32::from_le_bytes([buffer[4], buffer[5], buffer[6], buffer[7]]);
            let cmd_id = u16::from_le_bytes([buffer[8], buffer[9]]);
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

        let t_wall0 = Instant::now();
        let mut t_cap0: Option<f64> = None;
        for pkt in packets.iter() {
            if stop.load(Ordering::Relaxed) {
                break;
            }
            // Mid-360 multicasts point/IMU to mcast_data:port (the SDK joins it);
            // status is unicast. Unicasting point/IMU is silently dropped.
            let (socket, dest_ip, dest_port) = match pkt.src_port {
                PORT_POINT => (&point, mcast_data, DST_POINT),
                PORT_IMU => (&imu, mcast_data, DST_IMU),
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

// ---- payload synthesizers (layouts from Livox-SDK2 sdk_core/comm/define.h) ----
// Mid-360 device type (livox_lidar_def.h: kLivoxLidarTypeMid360 = 9).
const DEV_TYPE_MID360: u8 = 9;

/// Detection/search (0x0000) ACK body == `DetectionData`:
///   ret_code:u8, dev_type:u8, sn[16], lidar_ip[4], cmd_port:u16 LE.
/// The SDK's VerifyNetSegment requires lidar_ip on the host's /24 (192.168.1.x).
fn discovery_ack_payload(lidar_ip: Ipv4Addr) -> Vec<u8> {
    let mut payload = Vec::with_capacity(24);
    payload.push(0); // ret_code = success
    payload.push(DEV_TYPE_MID360);
    // sn[16] MUST be null-terminated within 16 bytes — the SDK treats it as a
    // C-string (strcpy), so a full-16 SN with no NUL overruns its buffer.
    let mut sn = [0u8; 16];
    sn[..10].copy_from_slice(b"FAKEMID360"); // sn[10..]=0 -> NUL-terminated
    payload.extend_from_slice(&sn);
    payload.extend_from_slice(&lidar_ip.octets());
    payload.extend_from_slice(&CMD_PORT.to_le_bytes());
    payload
}

// kKeyFwType (livox_lidar_def.h ParamKeyName = 0x8010); fw_type != 0 => app
// firmware (not loader/upgrade mode), so the SDK proceeds to normal operation.
const KEY_FW_TYPE: u16 = 0x8010;
const FW_TYPE_APP: u8 = 1;

/// Control-plane ACK bodies. The SDK casts the SdkPacket data[] directly to the
/// per-cmd response struct, which are #pragma pack(1) (packed, no padding).
fn control_ack_payload(cmd_id: u16) -> Vec<u8> {
    match cmd_id {
        // GetInternalInfo (0x0101), packed: ret_code:u8 @0, param_num:u16 @1, then
        // one KeyValueParam (key:u16, len:u16, value). QueryFwType wants kKeyFwType
        // (0x8010) -> 1-byte fw_type != 0.
        0x0101 => {
            let mut payload = vec![0u8; 8];
            // payload[0] ret_code = 0
            payload[1..3].copy_from_slice(&1u16.to_le_bytes()); // param_num = 1
            payload[3..5].copy_from_slice(&KEY_FW_TYPE.to_le_bytes());
            payload[5..7].copy_from_slice(&1u16.to_le_bytes()); // value length = 1
            payload[7] = FW_TYPE_APP;
            payload
        }
        // Others: LivoxLidarAsyncControlResponse (packed) { ret_code:u8 @0,
        // error_key:u16 @1 } = 3 bytes. ret_code=0 (success), error_key=0.
        _ => vec![0u8; 3],
    }
}
// LivoxLidarEthernetPacket.timestamp[8] is at payload offset 28 (the SDK casts
// the UDP payload straight to that packed struct), so rewrite 8 bytes there.
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
