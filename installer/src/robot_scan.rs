//! `dimos robot scan` — three read-only probes that print what is on the network. v1 saves
//! nothing; the identity and the address are separate fields so v2 can key a registry by identity.

use std::io::ErrorKind;
use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::time::{Duration, Instant};

use anyhow::Result;
use serde::Serialize;
use socket2::{Domain, Protocol, Socket, Type};

use crate::cli::ScanArgs;
use crate::plan::text;
use crate::probe::{capture, Os, Probes};
use crate::run_context::{Ctx, Mode};
use crate::say;

const GROUP: Ipv4Addr = Ipv4Addr::new(231, 1, 1, 1);
const QUERY_PORT: u16 = 10131;
const REPLY_PORT: u16 = 10134;
const QUERY: &[u8] = br#"{"name": "unitree_dapengche"}"#;
/// landiscovery.py re-sends every 2 s; a robot that finishes booting mid-window still answers.
const RESEND_EVERY: Duration = Duration::from_secs(1);
const PING_TIMEOUT_S: u64 = 5;
const SSH_TIMEOUT_S: u64 = 10;
/// The BLE scan's own `--timeout`, plus the Python CLI's start-up.
const BLE_STARTUP_S: u64 = 60;

/// VPN and container devices install a 224.0.0.0/4 route in a table that swallows the probe.
const SKIP_IFACES: [&str; 8] = [
    "lo",
    "tailscale",
    "wg",
    "tun",
    "docker",
    "br-",
    "veth",
    "Meta",
];

/// The G1's Jetson and the Unitree control computer on the wired link; only the Jetson takes ssh.
const WIRED_HOSTS: [(Ipv4Addr, &str, bool); 2] = [
    (Ipv4Addr::new(192, 168, 123, 164), "jetson", true),
    (
        Ipv4Addr::new(192, 168, 123, 161),
        "control computer (no ssh)",
        false,
    ),
];
const WIRED_NET: [u8; 3] = [192, 168, 123];

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    Lan,
    Wired,
    Ble,
}

impl Kind {
    pub fn name(self) -> &'static str {
        match self {
            Kind::Lan => "lan",
            Kind::Wired => "wired",
            Kind::Ble => "ble",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Identity {
    Serial(String),
    Mac(String),
    Unknown,
}

impl Identity {
    pub fn text(&self) -> &str {
        match self {
            Identity::Serial(value) | Identity::Mac(value) => value,
            Identity::Unknown => "-",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Found {
    pub kind: Kind,
    pub vendor: String,
    pub model: Option<String>,
    pub identity: Identity,
    pub addr: String,
    pub iface: String,
    pub note: Option<String>,
}

/// Print what is on the network and exit; scanning changes nothing, so it always exits 0.
pub fn run(args: &ScanArgs, ctx: &Ctx, probes: &Probes) -> Result<i32> {
    let found = scan(args, probes);
    if ctx.mode == Mode::Agent {
        println!("{}", serde_json::to_string(&found)?);
    } else if found.is_empty() {
        say::info("no robots found");
    } else {
        print!("{}", table(&found));
    }
    Ok(0)
}

pub fn scan(args: &ScanArgs, probes: &Probes) -> Vec<Found> {
    let all = args.all_kinds();
    let mut found = Vec::new();
    if all || args.lan {
        found.extend(lan(&probes.ifaces, args.timeout_s));
    }
    if all || args.wired {
        found.extend(wired(&probes.ifaces, &probes.platform.os));
    }
    if all || args.ble {
        found.extend(ble(probes, args.timeout_s));
    }
    found
}

/// One probe per usable interface, keeping the first sighting of each serial.
fn lan(ifaces: &[(String, Ipv4Addr)], timeout_s: u64) -> Vec<Found> {
    let mut found: Vec<Found> = Vec::new();
    for (name, ip) in lan_ifaces(ifaces) {
        for robot in probe_iface(&name, ip, timeout_s) {
            if !found.iter().any(|seen| seen.identity == robot.identity) {
                found.push(robot);
            }
        }
    }
    found
}

/// Non-loopback, non-VPN interfaces, first IPv4 address each.
pub fn lan_ifaces(ifaces: &[(String, Ipv4Addr)]) -> Vec<(String, Ipv4Addr)> {
    let mut usable: Vec<(String, Ipv4Addr)> = Vec::new();
    for (name, ip) in ifaces {
        let skipped = SKIP_IFACES.iter().any(|p| name.starts_with(p)) || ip.is_loopback();
        if !skipped && !usable.iter().any(|(seen, _)| seen == name) {
            usable.push((name.clone(), *ip));
        }
    }
    usable
}

fn probe_iface(name: &str, iface_ip: Ipv4Addr, timeout_s: u64) -> Vec<Found> {
    let sock = match reply_socket(iface_ip) {
        Ok(sock) => sock,
        Err(e) => {
            say::warn(&format!("{name}: no UDP {REPLY_PORT} on {iface_ip}: {e}"));
            return Vec::new();
        }
    };
    if let Err(e) = sock.send_to(QUERY, SocketAddrV4::new(GROUP, QUERY_PORT)) {
        say::warn(&format!("{name}: multicast send failed: {e}"));
        return Vec::new();
    }
    collect_replies(&sock, name, Duration::from_secs(timeout_s))
}

/// Pinned to one interface: the multicast route alone sends every probe out of the VPN.
fn reply_socket(iface_ip: Ipv4Addr) -> std::io::Result<UdpSocket> {
    let sock = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    sock.set_reuse_address(true)?;
    sock.bind(&SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, REPLY_PORT).into())?;
    sock.set_multicast_if_v4(&iface_ip)?;
    sock.join_multicast_v4(&GROUP, &iface_ip)?;
    Ok(sock.into())
}

fn collect_replies(sock: &UdpSocket, iface: &str, window: Duration) -> Vec<Found> {
    let deadline = Instant::now() + window;
    let mut found = Vec::new();
    let mut buf = [0u8; 1024];
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        if left.is_zero() || sock.set_read_timeout(Some(left.min(RESEND_EVERY))).is_err() {
            return found;
        }
        match sock.recv_from(&mut buf) {
            Ok((n, src)) => {
                let payload = String::from_utf8_lossy(&buf[..n]);
                found.extend(parse_reply(&payload, &src.ip().to_string(), iface));
            }
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {
                let _ = sock.send_to(QUERY, SocketAddrV4::new(GROUP, QUERY_PORT));
            }
            Err(_) => return found,
        }
    }
}

/// A reply carries `sn`; `ip` is optional and falls back to the datagram's source address.
pub fn parse_reply(payload: &str, src: &str, iface: &str) -> Option<Found> {
    let msg: serde_json::Value = serde_json::from_str(payload).ok()?;
    let serial = msg.get("sn")?.as_str().filter(|s| !s.is_empty())?;
    Some(Found {
        kind: Kind::Lan,
        vendor: "unitree".to_string(),
        model: None,
        identity: Identity::Serial(serial.to_string()),
        addr: msg
            .get("ip")
            .and_then(|v| v.as_str())
            .unwrap_or(src)
            .to_string(),
        iface: iface.to_string(),
        note: None,
    })
}

/// One interface is enough: ping follows the route table, so a second probe repeats the first.
fn wired(ifaces: &[(String, Ipv4Addr)], os: &Os) -> Vec<Found> {
    let Some((name, _)) = wired_ifaces(ifaces).into_iter().next() else {
        return Vec::new();
    };
    WIRED_HOSTS
        .iter()
        .filter(|(addr, _, _)| alive(os, *addr))
        .map(|(addr, label, ssh)| wired_found(&name, *addr, label, *ssh))
        .collect()
}

/// Interfaces holding a 192.168.123.0/24 address — the wired link to a Unitree robot.
pub fn wired_ifaces(ifaces: &[(String, Ipv4Addr)]) -> Vec<(String, Ipv4Addr)> {
    ifaces
        .iter()
        .filter(|(_, ip)| ip.octets()[..3] == WIRED_NET)
        .cloned()
        .collect()
}

fn wired_found(iface: &str, addr: Ipv4Addr, label: &str, ssh: bool) -> Found {
    let note = if ssh {
        format!("{label}, {}", ssh_note(addr))
    } else {
        label.to_string()
    };
    Found {
        kind: Kind::Wired,
        vendor: "unitree".to_string(),
        model: None,
        identity: Identity::Unknown,
        addr: addr.to_string(),
        iface: iface.to_string(),
        note: Some(note),
    }
}

fn alive(os: &Os, addr: Ipv4Addr) -> bool {
    let args = ["-c", "1", "-W", ping_wait_arg(os), &addr.to_string()];
    capture("ping", &args, &[], PING_TIMEOUT_S).is_some()
}

/// `-W` is whole seconds on iputils and milliseconds on macOS.
pub fn ping_wait_arg(os: &Os) -> &'static str {
    match os {
        Os::Linux { .. } => "1",
        Os::MacOs { .. } => "1000",
    }
}

/// Reports reachability only; `true` is the whole remote command.
fn ssh_note(addr: Ipv4Addr) -> &'static str {
    let args = [
        "-o",
        "BatchMode=yes",
        "-o",
        "ConnectTimeout=3",
        &format!("unitree@{addr}"),
        "true",
    ];
    if capture("ssh", &args, &[], SSH_TIMEOUT_S).is_some() {
        "ssh: key ok"
    } else {
        "ssh: password needed"
    }
}

/// BLE stays in Python (bleak); the installed venv runs it and we reprint its rows.
fn ble(probes: &Probes, timeout_s: u64) -> Vec<Found> {
    let Some(dimos) = probes
        .installed
        .as_ref()
        .map(|i| i.venv_dimos())
        .filter(|p| p.exists())
    else {
        say::warn("ble: install DimOS first (dimos setup)");
        return Vec::new();
    };
    let args = [
        "go2tool",
        "discover",
        "--ble",
        "--timeout",
        &timeout_s.to_string(),
    ];
    match capture(&text(&dimos), &args, &[], timeout_s + BLE_STARTUP_S) {
        Some(rows) => rows.lines().filter_map(parse_ble_row).collect(),
        None => {
            say::warn(&format!("ble: {} did not run", dimos.display()));
            Vec::new()
        }
    }
}

/// `dimos go2tool discover` prints `SOURCE NAME IP MAC SERIAL`; only its BLE rows are ours.
pub fn parse_ble_row(line: &str) -> Option<Found> {
    let fields: Vec<&str> = line.split_whitespace().collect();
    let ["BLE", name, _ip, mac, serial] = fields[..] else {
        return None;
    };
    Some(Found {
        kind: Kind::Ble,
        vendor: "unitree".to_string(),
        model: (name != "-").then(|| name.to_string()),
        identity: match serial {
            "?" => Identity::Mac(mac.to_string()),
            found => Identity::Serial(found.to_string()),
        },
        addr: mac.to_string(),
        iface: "ble".to_string(),
        note: None,
    })
}

pub fn table(found: &[Found]) -> String {
    let mut out = row(
        "kind", "vendor", "model", "identity", "address", "iface", "note",
    );
    for f in found {
        out.push_str(&row(
            f.kind.name(),
            &f.vendor,
            f.model.as_deref().unwrap_or("-"),
            f.identity.text(),
            &f.addr,
            &f.iface,
            f.note.as_deref().unwrap_or("-"),
        ));
    }
    out
}

fn row(
    kind: &str,
    vendor: &str,
    model: &str,
    identity: &str,
    addr: &str,
    iface: &str,
    note: &str,
) -> String {
    format!("{kind:<6} {vendor:<8} {model:<14} {identity:<20} {addr:<16} {iface:<9} {note}\n")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ifaces(pairs: &[(&str, [u8; 4])]) -> Vec<(String, Ipv4Addr)> {
        pairs
            .iter()
            .map(|(name, ip)| (name.to_string(), Ipv4Addr::from(*ip)))
            .collect()
    }

    #[test]
    fn lan_ifaces_drop_loopback_vpn_and_container_devices() {
        let all = ifaces(&[
            ("lo", [127, 0, 0, 1]),
            ("en0", [10, 0, 0, 7]),
            ("tailscale0", [100, 64, 0, 1]),
            ("docker0", [172, 17, 0, 1]),
            ("br-abc", [172, 18, 0, 1]),
            ("eth0", [192, 168, 123, 51]),
        ]);
        assert_eq!(
            lan_ifaces(&all),
            ifaces(&[("en0", [10, 0, 0, 7]), ("eth0", [192, 168, 123, 51])])
        );
    }

    #[test]
    fn lan_ifaces_keep_only_the_first_address_of_an_interface() {
        let all = ifaces(&[("en0", [10, 0, 0, 7]), ("en0", [10, 0, 0, 8])]);
        assert_eq!(lan_ifaces(&all), ifaces(&[("en0", [10, 0, 0, 7])]));
    }

    #[test]
    fn wired_ifaces_are_the_ones_carrying_a_192_168_123_address() {
        let all = ifaces(&[
            ("en0", [10, 0, 0, 7]),
            ("eth0", [192, 168, 123, 51]),
            ("eth1", [192, 168, 124, 51]),
        ]);
        assert_eq!(wired_ifaces(&all), ifaces(&[("eth0", [192, 168, 123, 51])]));
    }

    #[test]
    fn a_reply_reports_the_serial_and_prefers_its_own_ip_field() {
        let found = parse_reply(
            r#"{"sn":"B42D2000ABC","ip":"10.0.0.104"}"#,
            "10.0.0.9",
            "en0",
        )
        .expect("a reply carrying sn is a robot");
        assert_eq!(found.identity, Identity::Serial("B42D2000ABC".to_string()));
        assert_eq!(found.addr, "10.0.0.104");
        assert_eq!(found.iface, "en0");
    }

    #[test]
    fn a_reply_without_an_ip_field_falls_back_to_the_datagram_source() {
        let found = parse_reply(r#"{"sn":"B42D2000ABC"}"#, "10.0.0.104", "en0")
            .expect("a reply carrying sn is a robot");
        assert_eq!(found.addr, "10.0.0.104");
    }

    #[test]
    fn a_reply_without_a_serial_is_not_a_robot() {
        assert_eq!(
            parse_reply(r#"{"ip":"10.0.0.104"}"#, "10.0.0.9", "en0"),
            None
        );
        assert_eq!(parse_reply(r#"{"sn":""}"#, "10.0.0.9", "en0"), None);
        assert_eq!(parse_reply("not json", "10.0.0.9", "en0"), None);
    }

    #[test]
    fn a_reply_datagram_on_the_wire_becomes_a_row() {
        let sock = UdpSocket::bind("127.0.0.1:0").expect("bind the listener");
        let addr = sock.local_addr().expect("read the listener port");
        let responder = UdpSocket::bind("127.0.0.1:0").expect("bind the responder");
        responder
            .send_to(br#"{"sn":"B42D2000ABC","ip":"10.0.0.104"}"#, addr)
            .expect("send one reply");
        let found = collect_replies(&sock, "en0", Duration::from_millis(300));
        assert_eq!(found.len(), 1);
        assert_eq!(
            found[0].identity,
            Identity::Serial("B42D2000ABC".to_string())
        );
    }

    #[test]
    fn a_silent_interface_yields_no_rows_and_returns_at_the_deadline() {
        let sock = UdpSocket::bind("127.0.0.1:0").expect("bind the listener");
        let started = Instant::now();
        assert!(collect_replies(&sock, "en0", Duration::from_millis(100)).is_empty());
        assert!(started.elapsed() < Duration::from_secs(2));
    }

    #[test]
    fn a_ble_row_keeps_the_mac_as_identity_when_the_serial_is_unknown() {
        let found = parse_ble_row("BLE    GO2_1234       -               AA:BB:CC:DD:EE:FF   ?")
            .expect("a BLE row is a robot");
        assert_eq!(
            found.identity,
            Identity::Mac("AA:BB:CC:DD:EE:FF".to_string())
        );
        assert_eq!(found.model, Some("GO2_1234".to_string()));
        assert_eq!(found.addr, "AA:BB:CC:DD:EE:FF");
    }

    #[test]
    fn a_ble_row_with_a_serial_is_identified_by_it() {
        let found = parse_ble_row("BLE GO2_1234 - AA:BB:CC:DD:EE:FF B42D2000ABC")
            .expect("a BLE row is a robot");
        assert_eq!(found.identity, Identity::Serial("B42D2000ABC".to_string()));
    }

    #[test]
    fn only_ble_rows_are_parsed_out_of_go2tool_output() {
        assert_eq!(
            parse_ble_row("SOURCE NAME           IP              MAC                 SERIAL"),
            None
        );
        assert_eq!(
            parse_ble_row("LAN    -              10.0.0.104      -                   B42D2000ABC"),
            None
        );
        assert_eq!(parse_ble_row(""), None);
        assert_eq!(parse_ble_row("Stopped."), None);
    }

    #[test]
    fn the_control_computer_is_labelled_and_never_probed_for_ssh() {
        let found = wired_found(
            "eth0",
            Ipv4Addr::new(192, 168, 123, 161),
            "control computer (no ssh)",
            false,
        );
        assert_eq!(found.note.as_deref(), Some("control computer (no ssh)"));
        assert_eq!(found.addr, "192.168.123.161");
        assert_eq!(WIRED_HOSTS[0].1, "jetson");
        assert!(WIRED_HOSTS[0].2 && !WIRED_HOSTS[1].2);
    }

    #[test]
    fn ping_waits_milliseconds_on_macos_and_seconds_on_linux() {
        let linux = Os::Linux {
            id: "ubuntu".to_string(),
            version: "22.04".to_string(),
        };
        let macos = Os::MacOs {
            version: "15.0".to_string(),
        };
        assert_eq!(ping_wait_arg(&linux), "1");
        assert_eq!(ping_wait_arg(&macos), "1000");
    }

    #[test]
    fn the_table_prints_a_header_and_one_line_per_robot() {
        let found = [Found {
            kind: Kind::Lan,
            vendor: "unitree".to_string(),
            model: None,
            identity: Identity::Serial("B42D2000ABC".to_string()),
            addr: "10.0.0.104".to_string(),
            iface: "en0".to_string(),
            note: None,
        }];
        let text = table(&found);
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 2);
        assert!(lines[0].starts_with("kind   vendor"));
        assert_eq!(
            lines[1].split_whitespace().collect::<Vec<_>>(),
            [
                "lan",
                "unitree",
                "-",
                "B42D2000ABC",
                "10.0.0.104",
                "en0",
                "-"
            ]
        );
    }
}
