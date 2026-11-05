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

//! Readers for what the kernel and a few tools print, kept pure so they're tested on fixed text and directories.

use std::fs;
use std::path::Path;

use serde_json::{json, Map, Value};

pub fn read(path: &Path) -> Option<String> {
    fs::read_to_string(path)
        .ok()
        .map(|text| text.trim().to_string())
}

fn read_i64(path: &Path) -> Option<i64> {
    read(path)?.parse().ok()
}

pub fn round(value: f64, digits: i32) -> f64 {
    let scale = 10f64.powi(digits);
    (value * scale).round() / scale
}

/// /proc/net/wireless → {iface: {"quality", "level_dbm"}}
pub fn proc_net_wireless(text: &str) -> Map<String, Value> {
    let mut wireless = Map::new();
    for line in text.lines().skip(2) {
        let Some((iface, rest)) = line.split_once(':') else {
            continue;
        };
        let fields: Vec<f64> = rest
            .split_whitespace()
            .skip(1)
            .take(2)
            .filter_map(|f| f.trim_end_matches('.').parse().ok())
            .collect();
        if let [quality, level] = fields[..] {
            wireless.insert(
                iface.trim().to_string(),
                json!({ "quality": quality, "level_dbm": level }),
            );
        }
    }
    wireless
}

/// `iw dev <iface> link` → ssid, signal, bitrates and frequency
pub fn iw_link(text: &str) -> Map<String, Value> {
    let mut link = Map::new();
    if text.starts_with("Not connected") {
        link.insert("connected".into(), json!(false));
        return link;
    }
    link.insert("connected".into(), json!(true));
    for line in text.lines() {
        let line = line.trim();
        for (prefix, key) in [
            ("SSID:", "ssid"),
            ("signal:", "signal"),
            ("tx bitrate:", "tx_bitrate"),
            ("rx bitrate:", "rx_bitrate"),
            ("freq:", "freq"),
        ] {
            if let Some(value) = line.strip_prefix(prefix) {
                link.insert(key.into(), json!(value.trim()));
            }
        }
    }
    link
}

/// /proc/net/route → the default routes, as "wlan0 via 192.168.12.1 metric 600"
pub fn proc_net_route(text: &str) -> Vec<String> {
    text.lines()
        .skip(1)
        .filter_map(|line| {
            let f: Vec<&str> = line.split_whitespace().collect();
            if f.len() < 7 || f[1] != "00000000" {
                return None;
            }
            let gw = u32::from_str_radix(f[2], 16).ok()?.to_le_bytes();
            Some(format!(
                "{} via {}.{}.{}.{} metric {}",
                f[0], gw[0], gw[1], gw[2], gw[3], f[6]
            ))
        })
        .collect()
}

/// `route -n get default` (macOS) → (interface, "en0 via 192.168.1.1")
pub fn macos_route(text: &str) -> Option<(String, String)> {
    let field = |key: &str| {
        text.lines()
            .find_map(|l| l.trim().strip_prefix(key).map(|v| v.trim().to_string()))
    };
    let iface = field("interface:")?;
    let route = format!(
        "{iface} via {}",
        field("gateway:").unwrap_or_else(|| "?".into())
    );
    Some((iface, route))
}

/// /sys/class/thermal/thermal_zone*: {"thermal_zone0/cpu-thermal": 48.3}
pub fn thermal_zones(root: &Path) -> Map<String, Value> {
    let mut temps = Map::new();
    for zone in sorted_entries(root, "thermal_zone") {
        let name = zone
            .file_name()
            .unwrap_or_default()
            .to_string_lossy()
            .to_string();
        if let (Some(kind), Some(millidegrees)) =
            (read(&zone.join("type")), read_i64(&zone.join("temp")))
        {
            temps.insert(
                format!("{name}/{kind}"),
                json!(round(millidegrees as f64 / 1000.0, 1)),
            );
        }
    }
    temps
}

/// hwmon (/sys/class/hwmon): watts per rail — power*_input (µW), or in*_input (mV) × curr*_input (mA) like a
/// Jetson's INA3221 — and fan speeds from fan*_input (RPM).
pub fn hwmon(root: &Path) -> (Map<String, Value>, Map<String, Value>) {
    let (mut watts, mut fans) = (Map::new(), Map::new());
    for hwmon in sorted_entries(root, "hwmon") {
        let chip = read(&hwmon.join("name")).unwrap_or_else(|| {
            hwmon
                .file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .into()
        });
        let label = |prefix: &str, channel: &str| {
            read(&hwmon.join(format!("{prefix}{channel}_label")))
                .unwrap_or(format!("{prefix}{channel}"))
        };
        for input in sorted_entries(&hwmon, "") {
            let file = input
                .file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .to_string();
            let Some(stem) = file.strip_suffix("_input") else {
                continue;
            };
            if let Some(channel) = stem.strip_prefix("power") {
                if let Some(microwatts) = read_i64(&input) {
                    watts.insert(
                        format!("{chip}/{}", label("power", channel)),
                        json!(round(microwatts as f64 / 1e6, 2)),
                    );
                }
            } else if let Some(channel) = stem.strip_prefix("in") {
                if let (Some(mv), Some(ma)) = (
                    read_i64(&input),
                    read_i64(&hwmon.join(format!("curr{channel}_input"))),
                ) {
                    watts.insert(
                        format!("{chip}/{}", label("in", channel)),
                        json!(round(mv as f64 * ma as f64 / 1e6, 2)),
                    );
                }
            } else if let Some(channel) = stem.strip_prefix("fan") {
                if let Some(rpm) = read_i64(&input) {
                    fans.insert(format!("{chip}/{}", label("fan", channel)), json!(rpm));
                }
            }
        }
    }
    (watts, fans)
}

/// /sys/class/power_supply: each battery or adapter's type, status, capacity and watts
pub fn power_supplies(root: &Path) -> Map<String, Value> {
    let mut supplies = Map::new();
    for supply in sorted_entries(root, "") {
        let mut entry = Map::new();
        for key in ["type", "status"] {
            if let Some(value) = read(&supply.join(key)) {
                entry.insert(key.into(), json!(value));
            }
        }
        if let Some(capacity) = read_i64(&supply.join("capacity")) {
            entry.insert("capacity".into(), json!(capacity));
        }
        let watts = read_i64(&supply.join("power_now"))
            .map(|uw| uw as f64 / 1e6)
            .or_else(|| {
                Some(
                    read_i64(&supply.join("voltage_now"))? as f64
                        * read_i64(&supply.join("current_now"))? as f64
                        / 1e12,
                )
            });
        if let Some(watts) = watts {
            entry.insert("watts".into(), json!(round(watts, 2)));
        }
        if !entry.is_empty() {
            supplies.insert(
                supply
                    .file_name()
                    .unwrap_or_default()
                    .to_string_lossy()
                    .into(),
                Value::Object(entry),
            );
        }
    }
    supplies
}

/// A Raspberry Pi's throttled bits (`vcgencmd get_throttled`, or the firmware's sysfs file) → what they mean
pub fn pi_throttled(text: &str) -> Option<Value> {
    let value = text.trim().rsplit('=').next()?.trim_start_matches("0x");
    let bits = u32::from_str_radix(value, 16).ok()?;
    let names = [
        "undervoltage",
        "freq_capped",
        "throttled",
        "soft_temp_limit",
    ];
    let set = |offset: u32| {
        names
            .iter()
            .enumerate()
            .filter(|(i, _)| bits & (1 << (offset + *i as u32)) != 0)
            .map(|(_, n)| *n)
            .collect::<Vec<_>>()
    };
    Some(json!({ "raw": format!("0x{bits:x}"), "now": set(0), "since_boot": set(16) }))
}

/// `nvidia-smi --query-gpu=name,utilization.gpu,memory.used,memory.total,temperature.gpu,power.draw
/// --format=csv,noheader,nounits` → one entry per GPU
pub fn nvidia_smi(text: &str) -> Vec<Value> {
    text.lines()
        .filter_map(|line| {
            let fields: Vec<&str> = line.split(',').map(str::trim).collect();
            let [name, util, used, total, temp, power] = fields[..] else {
                return None;
            };
            let number = |v: &str| v.parse::<f64>().ok();
            Some(
                json!({ "name": name, "util_percent": number(util), "mem_used_mb": number(used),
                "mem_total_mb": number(total), "temp_c": number(temp), "power_w": number(power) }),
            )
        })
        .collect()
}

/// `pmset -g batt` (macOS) → {"percent", "state", "remaining"}; None without a battery
pub fn pmset_batt(text: &str) -> Option<Value> {
    let line = text.lines().find(|l| l.contains('%'))?;
    let after_tab = line.split('\t').nth(1)?;
    let mut parts = after_tab.split(';').map(str::trim);
    let percent: f64 = parts.next()?.trim_end_matches('%').parse().ok()?;
    let state = parts.next().unwrap_or("unknown");
    let remaining = parts
        .next()
        .and_then(|r| r.split_whitespace().next())
        .filter(|r| r.contains(':'));
    Some(json!({ "percent": percent, "state": state, "remaining": remaining }))
}

/// /proc/diskstats → total (sectors read, sectors written) over whole disks (those in /sys/block, minus loop/ram)
pub fn diskstats(text: &str, is_disk: impl Fn(&str) -> bool) -> (u64, u64) {
    text.lines().fold((0, 0), |(read, written), line| {
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() < 10 || !is_disk(f[2]) {
            return (read, written);
        }
        (
            read + f[5].parse::<u64>().unwrap_or(0),
            written + f[9].parse::<u64>().unwrap_or(0),
        )
    })
}

fn sorted_entries(dir: &Path, prefix: &str) -> Vec<std::path::PathBuf> {
    let mut entries: Vec<_> = fs::read_dir(dir)
        .into_iter()
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| {
            p.file_name()
                .is_some_and(|n| n.to_string_lossy().starts_with(prefix))
        })
        .collect();
    entries.sort();
    entries
}

/// `id` → {"uid", "gid", "groups"}, each as "501(jeff)"
pub fn id(text: &str) -> Value {
    let mut id = Map::new();
    for field in text.split_whitespace() {
        if let Some((key, value)) = field.split_once('=') {
            let value = if key == "groups" {
                json!(value.split(',').collect::<Vec<_>>())
            } else {
                json!(value)
            };
            id.insert(key.into(), value);
        }
    }
    Value::Object(id)
}

/// A command-line argument with any `NAME=value` (an env var set for the command, as `env FOO=x cmd` does) or
/// `--secret-ish=value` value replaced, so the variables and secrets a process was started with stay off the log.
pub fn redact_assignment(arg: &str) -> String {
    let Some((name, value)) = arg.split_once('=') else {
        return arg.to_string();
    };
    if value.starts_with('=') {
        return arg.to_string(); // a comparison ("a==b"), not an assignment
    }
    let flag = name.trim_start_matches('-');
    let env_name = !name.starts_with('-')
        && !flag.is_empty()
        && !flag.starts_with(|c: char| c.is_ascii_digit())
        && flag.chars().all(|c| c.is_ascii_alphanumeric() || c == '_');
    let lower = flag.to_ascii_lowercase();
    let secret = [
        "key",
        "token",
        "secret",
        "password",
        "passwd",
        "auth",
        "credential",
    ]
    .iter()
    .any(|word| lower.contains(word));
    if env_name || secret {
        format!("{name}=<redacted>")
    } else {
        arg.to_string()
    }
}

/// The user's name from `id` ("uid=501(jeff) ...") → "jeff"
pub fn id_name(text: &str) -> Option<String> {
    let uid = text.split_whitespace().next()?.strip_prefix("uid=")?;
    Some(uid.split_once('(')?.1.trim_end_matches(')').to_string())
}

/// /etc/resolv.conf → {"nameservers", "search"}
pub fn resolv_conf(text: &str) -> Value {
    let (mut nameservers, mut search) = (Vec::new(), Vec::new());
    for line in text.lines() {
        let mut words = line.split_whitespace();
        match words.next() {
            Some("nameserver") => nameservers.extend(words),
            Some("search") | Some("domain") => search.extend(words),
            _ => {}
        }
    }
    json!({ "nameservers": nameservers, "search": search })
}

/// `arp -an` → [{"ip", "mac", "iface"}], skipping entries still unresolved
pub fn arp(text: &str) -> Vec<Value> {
    text.lines()
        .filter_map(|line| {
            let ip = line.split_once('(')?.1.split_once(')')?.0;
            let mut after = line.split_once(") at ")?.1.split_whitespace();
            let mac = after.next().filter(|m| m.contains(':'))?;
            let iface = after.skip_while(|w| *w != "on").nth(1);
            Some(json!({ "ip": ip, "mac": mac, "iface": iface }))
        })
        .collect()
}

/// The value of `key` in ioreg's `"key" = "value"` / `<"value">` lines, or ipconfig's and lscpu's `key : value` ones.
pub fn key_value(text: &str, key: &str) -> Option<String> {
    text.lines().find_map(|line| {
        let line = line.trim();
        let rest = line
            .strip_prefix(&format!("\"{key}\""))
            .or_else(|| line.strip_prefix(key))?
            .trim_start();
        let value = rest.strip_prefix('=').or_else(|| rest.strip_prefix(':'))?;
        Some(
            value
                .trim()
                .trim_start_matches('<')
                .trim_end_matches('>')
                .trim_matches('"')
                .to_string(),
        )
    })
}

/// `networksetup -listallhardwareports` → the Wi-Fi devices ("en0")
pub fn macos_wifi_devices(text: &str) -> Vec<String> {
    let mut devices = Vec::new();
    let mut wifi = false;
    for line in text.lines() {
        if let Some(port) = line.strip_prefix("Hardware Port: ") {
            wifi = port == "Wi-Fi" || port == "AirPort";
        } else if let (true, Some(device)) = (wifi, line.strip_prefix("Device: ")) {
            devices.push(device.trim().to_string());
        }
    }
    devices
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write(dir: &Path, files: &[(&str, &str)]) {
        for (name, text) in files {
            let path = dir.join(name);
            fs::create_dir_all(path.parent().unwrap()).unwrap();
            fs::write(path, text).unwrap();
        }
    }

    fn tempdir(name: &str) -> std::path::PathBuf {
        let dir =
            std::env::temp_dir().join(format!("system_log_test_{name}_{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn reads_wireless_iw_and_routes() {
        let wireless = "Inter-| sta-|   Quality        |   Discarded packets\n face | tus | link level noise |  nwid\n wlan0: 0000   54.  -56.  -256        0      0\n";
        assert_eq!(
            Value::Object(proc_net_wireless(wireless)),
            json!({ "wlan0": { "quality": 54.0, "level_dbm": -56.0 } })
        );
        let iw = "Connected to aa:bb (on wlan0)\n\tSSID: dimos\n\tfreq: 5180\n\tsignal: -56 dBm\n\ttx bitrate: 433.3 MBit/s\n";
        assert_eq!(
            Value::Object(iw_link(iw)),
            json!({ "connected": true, "ssid": "dimos", "freq": "5180", "signal": "-56 dBm", "tx_bitrate": "433.3 MBit/s" })
        );
        assert_eq!(
            Value::Object(iw_link("Not connected.")),
            json!({ "connected": false })
        );
        let route = "Iface\tDestination\tGateway\tFlags\tRefCnt\tUse\tMetric\tMask\nwlan0\t00000000\t010CA8C0\t0003\t0\t0\t600\t00000000\nwlan0\t000CA8C0\t00000000\t0001\t0\t0\t600\t00FFFFFF\n";
        assert_eq!(
            proc_net_route(route),
            vec!["wlan0 via 192.168.12.1 metric 600"]
        );
        let mac = "   route to: default\n    gateway: 192.168.1.1\n  interface: en0\n";
        assert_eq!(
            macos_route(mac),
            Some(("en0".into(), "en0 via 192.168.1.1".into()))
        );
    }

    #[test]
    fn reads_jetson_rails_fans_thermal_and_supplies() {
        let dir = tempdir("sysfs");
        write(
            &dir,
            &[
                ("hwmon/hwmon1/name", "ina3221"),
                ("hwmon/hwmon1/in1_label", "VDD_IN"),
                ("hwmon/hwmon1/in1_input", "5080"),
                ("hwmon/hwmon1/curr1_input", "1200"),
                ("hwmon/hwmon1/in2_input", "1000"),
                ("hwmon/hwmon2/name", "pwmfan"),
                ("hwmon/hwmon2/fan1_input", "2100"),
                ("hwmon/hwmon3/name", "rapl"),
                ("hwmon/hwmon3/power1_input", "15500000"),
                ("thermal/thermal_zone0/type", "cpu-thermal"),
                ("thermal/thermal_zone0/temp", "48312"),
                ("power_supply/BAT0/type", "Battery"),
                ("power_supply/BAT0/status", "Discharging"),
                ("power_supply/BAT0/capacity", "73"),
                ("power_supply/BAT0/voltage_now", "12000000"),
                ("power_supply/BAT0/current_now", "1500000"),
            ],
        );
        let (watts, fans) = hwmon(&dir.join("hwmon"));
        assert_eq!(
            Value::Object(watts),
            json!({ "ina3221/VDD_IN": 6.1, "rapl/power1": 15.5 })
        );
        assert_eq!(Value::Object(fans), json!({ "pwmfan/fan1": 2100 }));
        assert_eq!(
            Value::Object(thermal_zones(&dir.join("thermal"))),
            json!({ "thermal_zone0/cpu-thermal": 48.3 })
        );
        assert_eq!(
            Value::Object(power_supplies(&dir.join("power_supply"))),
            json!({ "BAT0": { "type": "Battery", "status": "Discharging", "capacity": 73, "watts": 18.0 } })
        );
        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn reads_pi_gpu_battery_and_disks() {
        assert_eq!(
            pi_throttled("throttled=0x50005"),
            Some(
                json!({ "raw": "0x50005", "now": ["undervoltage", "throttled"], "since_boot": ["undervoltage", "throttled"] })
            )
        );
        assert_eq!(
            pi_throttled("0"),
            Some(json!({ "raw": "0x0", "now": [], "since_boot": [] }))
        );
        assert_eq!(
            nvidia_smi("NVIDIA RTX 4090, 37, 2048, 24564, 51, 88.20\n"),
            vec![
                json!({ "name": "NVIDIA RTX 4090", "util_percent": 37.0, "mem_used_mb": 2048.0,
                "mem_total_mb": 24564.0, "temp_c": 51.0, "power_w": 88.2 })
            ]
        );
        let pmset = "Now drawing from 'Battery Power'\n -InternalBattery-0 (id=1)\t84%; discharging; 3:12 remaining present: true\n";
        assert_eq!(
            pmset_batt(pmset),
            Some(json!({ "percent": 84.0, "state": "discharging", "remaining": "3:12" }))
        );
        assert_eq!(pmset_batt("Now drawing from 'AC Power'\n"), None);
        let stats = "   8       0 sda 100 0 2000 0 50 0 4000 0 0 0 0\n   8       1 sda1 90 0 1800 0 40 0 3000 0 0 0 0\n";
        assert_eq!(diskstats(stats, |name| name == "sda"), (2000, 4000));
    }

    #[test]
    fn identity_readers() {
        let id = id("uid=501(jeff) gid=20(staff) groups=20(staff),80(admin)");
        assert_eq!(id["uid"], "501(jeff)");
        assert_eq!(id["groups"][1], "80(admin)");
        assert_eq!(
            id_name("uid=0(root) gid=0(root) groups=0(root)").as_deref(),
            Some("root")
        );
        assert_eq!(
            key_value("Model name:             Cortex-A78AE\n", "Model name").as_deref(),
            Some("Cortex-A78AE")
        );
        assert_eq!(
            redact_assignment("OPENAI_API_KEY=sk-abc"),
            "OPENAI_API_KEY=<redacted>"
        );
        assert_eq!(
            redact_assignment("--api-key=sk-abc"),
            "--api-key=<redacted>"
        );
        assert_eq!(redact_assignment("--sample_hz=4"), "--sample_hz=4");
        assert_eq!(redact_assignment("/usr/bin/python"), "/usr/bin/python");
        assert_eq!(redact_assignment("a==b"), "a==b");
        let dns = resolv_conf("# x\nsearch a.ts.net localdomain\nnameserver 100.100.100.100\n");
        assert_eq!(dns["nameservers"][0], "100.100.100.100");
        assert_eq!(dns["search"][1], "localdomain");
        let arp = arp("? (10.0.0.1) at aa:9c:6c:9c:56:f1 on en0 ifscope [ethernet]\n? (10.0.0.9) at (incomplete) on en0 ifscope [ethernet]\n? (192.168.1.2) at 2:9d:48:e4:91:35 [ether] on wlan0\n");
        assert_eq!(arp.len(), 2);
        assert_eq!(arp[0]["mac"], "aa:9c:6c:9c:56:f1");
        assert_eq!(arp[1]["iface"], "wlan0");
        let ioreg = "    \"manufacturer\" = <\"Apple Inc.\">\n    \"IOPlatformUUID\" = \"98-52\"\n";
        assert_eq!(
            key_value(ioreg, "manufacturer").as_deref(),
            Some("Apple Inc.")
        );
        assert_eq!(key_value(ioreg, "IOPlatformUUID").as_deref(), Some("98-52"));
        assert_eq!(
            key_value("  BSSID : <redacted>\n  SSID : home\n", "SSID").as_deref(),
            Some("home")
        );
        let ports = "Hardware Port: Ethernet\nDevice: en3\n\nHardware Port: Wi-Fi\nDevice: en0\nEthernet Address: x\n";
        assert_eq!(macos_wifi_devices(ports), vec!["en0"]);
    }
}
