// Copyright 2025-2026 Dimensional Inc.
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

//! Who and what this machine is, read once: host, user, OS, hardware ids, every interface's MAC and addresses,
//! routes, DNS, Wi-Fi and the LAN neighbours. Only what an unprivileged user can read, on Linux and macOS.

use std::path::Path;
use std::time::Duration;

use serde_json::{json, Map, Value};
use sysinfo::{Disks, Networks, System, Users};

use crate::machine::{on_path, run};
use crate::parse;

const ENV_ALLOWED: [&str; 6] = ["USER", "LOGNAME", "HOME", "SHELL", "TZ", "LANG"];

pub fn read(timeout: Duration) -> Value {
    let run = |program: &str, args: &[&str]| run(program, args, timeout);
    // the only environment variables read, so nothing else in the environment (API keys) reaches the log
    let env = |name: &str| {
        assert!(ENV_ALLOWED.contains(&name), "{name} is not in ENV_ALLOWED");
        std::env::var(name).ok()
    };
    let id = run("id", &[]);
    let mut identity = json!({
        "hostname": System::host_name(),
        "user": {
            "name": env("USER").or_else(|| env("LOGNAME")).or_else(|| parse::id_name(&id)),
            "home": env("HOME"),
            "shell": env("SHELL"),
            "id": parse::id(&id),
        },
        "os": {
            "name": System::name(),
            "version": System::long_os_version(),
            "kernel": System::kernel_version(),
            "distribution": System::distribution_id(),
            "arch": System::cpu_arch(),
            "boot_time": System::boot_time(),
            "timezone": std::fs::read_link("/etc/localtime").ok().map(|p| p.display().to_string()).or_else(|| env("TZ")),
            "locale": env("LANG"),
        },
        "hardware": hardware(&run),
        "interfaces": interfaces(),
        "dns": parse::resolv_conf(&parse::read(Path::new("/etc/resolv.conf")).unwrap_or_default()),
        "neighbours": parse::arp(&run("arp", &["-an"])),
        "accounts": Users::new_with_refreshed_list().iter().map(|u| json!({
            "name": u.name(),
            "uid": u.id().to_string(),
            "groups": u.groups().iter().map(|g| g.name().to_string()).collect::<Vec<_>>(),
        })).collect::<Vec<_>>(),
        "logged_in": run("who", &[]).lines().map(str::to_string).collect::<Vec<_>>(),
        "disks": Disks::new_with_refreshed_list().iter().map(|d| json!({
            "name": d.name().to_string_lossy(),
            "mount": d.mount_point().display().to_string(),
            "fs": d.file_system().to_string_lossy(),
            "kind": d.kind().to_string(),
            "removable": d.is_removable(),
            "total_gb": parse::round(d.total_space() as f64 / 1e9, 1),
        })).collect::<Vec<_>>(),
    });
    let map = identity.as_object_mut().expect("an object");
    if cfg!(target_os = "macos") {
        let (_, route) = parse::macos_route(&run("route", &["-n", "get", "default"])).unzip();
        map.insert(
            "routes".into(),
            json!(route.into_iter().collect::<Vec<_>>()),
        );
        map.insert("wifi".into(), macos_wifi(&run));
    } else {
        map.insert(
            "routes".into(),
            json!(parse::proc_net_route(
                &parse::read(Path::new("/proc/net/route")).unwrap_or_default()
            )),
        );
        let iw = if on_path("iw") {
            run("iw", &["dev"])
        } else {
            String::new()
        };
        map.insert(
            "wifi".into(),
            json!(Some(iw.trim()).filter(|w| !w.is_empty())),
        );
    }
    identity
}

fn hardware(run: &dyn Fn(&str, &[&str]) -> String) -> Value {
    let mut system = System::new();
    system.refresh_cpu_all();
    system.refresh_memory();
    let mut hardware = Map::new();
    // ARM Linux often has no brand string; lscpu names the core ("Cortex-A78AE") instead
    let cpu = system
        .cpus()
        .first()
        .map(|c| c.brand().to_string())
        .filter(|b| !b.is_empty());
    let cpu =
        cpu.or_else(|| parse::key_value(&run("lscpu", &[]), "Model name").filter(|m| m != "-"));
    hardware.insert("cpu".into(), json!(cpu));
    hardware.insert("cpus".into(), json!(system.cpus().len()));
    hardware.insert(
        "physical_cores".into(),
        json!(System::physical_core_count()),
    );
    hardware.insert(
        "memory_gb".into(),
        json!(parse::round(system.total_memory() as f64 / 1e9, 1)),
    );
    hardware.insert(
        "swap_gb".into(),
        json!(parse::round(system.total_swap() as f64 / 1e9, 1)),
    );
    if cfg!(target_os = "macos") {
        let platform = run("ioreg", &["-rd1", "-c", "IOPlatformExpertDevice"]);
        for (key, name) in [
            ("IOPlatformSerialNumber", "serial"),
            ("IOPlatformUUID", "uuid"),
            ("model", "model"),
            ("manufacturer", "manufacturer"),
        ] {
            hardware.insert(name.into(), json!(parse::key_value(&platform, key)));
        }
        hardware.insert(
            "computer_name".into(),
            json!(run("scutil", &["--get", "ComputerName"]).trim()),
        );
    } else {
        // /etc/machine-id, DMI and the device tree (Jetson, Pi) are world-readable; product_serial/uuid aren't
        let files = [
            ("machine_id", "/etc/machine-id"),
            ("vendor", "/sys/class/dmi/id/sys_vendor"),
            ("model", "/sys/class/dmi/id/product_name"),
            ("model_version", "/sys/class/dmi/id/product_version"),
            ("board", "/sys/class/dmi/id/board_name"),
            ("bios", "/sys/class/dmi/id/bios_version"),
            ("device_tree_model", "/proc/device-tree/model"),
            ("serial", "/proc/device-tree/serial-number"),
        ];
        for (name, path) in files {
            if let Some(text) = parse::read(Path::new(path)) {
                hardware.insert(name.into(), json!(text.trim_end_matches('\0')));
            }
        }
    }
    Value::Object(hardware)
}

fn interfaces() -> Value {
    let networks = Networks::new_with_refreshed_list();
    let mut interfaces = Map::new();
    for (name, data) in networks.list() {
        interfaces.insert(
            name.clone(),
            json!({
                "mac": data.mac_address().to_string(),
                "addrs": data.ip_networks().iter().map(|ip| format!("{}/{}", ip.addr, ip.prefix)).collect::<Vec<_>>(),
            }),
        );
    }
    Value::Object(interfaces)
}

fn macos_wifi(run: &dyn Fn(&str, &[&str]) -> String) -> Value {
    let mut wifi = Map::new();
    for iface in parse::macos_wifi_devices(&run("networksetup", &["-listallhardwareports"])) {
        // macOS 14+ hides these from processes without Location access: then they read "redacted"
        let summary = run("ipconfig", &["getsummary", &iface]);
        wifi.insert(
            iface.clone(),
            json!({
                "ssid": parse::key_value(&summary, "SSID"),
                "bssid": parse::key_value(&summary, "BSSID"),
            }),
        );
    }
    Value::Object(wifi)
}
