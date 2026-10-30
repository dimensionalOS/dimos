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

//! What the computer is doing. A sample is cheap (sysinfo plus a few /proc and /sys reads) so it can run at a few
//! Hz; what needs a subprocess (`iw`, `nvidia-smi`, `route`, `pmset`) is refreshed on its own thread by `read_slow`
//! and carried along in every sample.

use std::io::Read;
use std::path::Path;
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::{json, Map, Value};
use sysinfo::{
    Components, Disks, Networks, ProcessRefreshKind, ProcessesToUpdate, System, UpdateKind,
};

use crate::parse::{self, round};

const SYS_CLASS: &str = "/sys/class";
const ABSOLUTE_ZERO_C: f32 = -273.15;
const PI_THROTTLED: &str = "/sys/devices/platform/soc/soc:firmware/get_throttled";
const JETSON_GPU_LOAD: [&str; 2] = [
    "/sys/devices/gpu.0/load",
    "/sys/devices/platform/gpu.0/load",
];

/// The command's stdout, or nothing if it fails or is still running after `timeout` (then it's killed).
pub(crate) fn run(program: &str, args: &[&str], timeout: Duration) -> String {
    let Ok(mut child) = Command::new(program)
        .args(args)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
    else {
        return String::new();
    };
    let Some(mut stdout) = child.stdout.take() else {
        return String::new();
    };
    // read while it runs, so a full pipe can't stall it
    let reader = std::thread::spawn(move || {
        let mut out = Vec::new();
        let _ = stdout.read_to_end(&mut out);
        out
    });
    let deadline = Instant::now() + timeout;
    let succeeded = loop {
        match child.try_wait() {
            Ok(Some(status)) => break status.success(),
            Ok(None) if Instant::now() < deadline => std::thread::sleep(Duration::from_millis(10)),
            _ => {
                let _ = child.kill();
                let _ = child.wait();
                break false;
            }
        }
    };
    match (succeeded, reader.join()) {
        (true, Ok(out)) => String::from_utf8_lossy(&out).to_string(),
        _ => String::new(),
    }
}

pub(crate) fn on_path(program: &str) -> bool {
    std::env::var_os("PATH")
        .is_some_and(|paths| std::env::split_paths(&paths).any(|dir| dir.join(program).is_file()))
}

fn refresh_processes(system: &mut System) {
    let kind = ProcessRefreshKind::nothing()
        .with_cpu()
        .with_memory()
        .with_cmd(UpdateKind::OnlyIfNotSet);
    system.refresh_processes_specifics(ProcessesToUpdate::All, true, kind);
}

#[derive(Default)]
struct Slow {
    /// merged into every sample
    fields: Map<String, Value>,
    /// iface → `iw` link details, merged into that iface's /proc/net/wireless entry
    iw: Map<String, Value>,
    /// merged into every sample's temps_c
    temps: Map<String, Value>,
}

/// The hottest of the GPU's SMC sensors. M3 and later keep GPU temperatures in the SMC rather than in the IOHID
/// sensors sysinfo reads, and M5 left the IOHID channels that used to carry them (`PMU tdev*`) unpopulated.
#[cfg(target_os = "macos")]
fn smc_temps(smc: Option<&crate::smc::Smc>) -> Map<String, Value> {
    let hottest = smc
        .map(|smc| smc.gpu_temps())
        .unwrap_or_default()
        .into_iter()
        .map(|(_, celsius)| celsius)
        .reduce(f32::max);
    hottest
        .map(|celsius| ("GPU".to_string(), json!(round(celsius as f64, 1))))
        .into_iter()
        .collect()
}

/// The parts that need a subprocess or change slowly: Wi-Fi link details, routes, GPUs, battery, disk usage.
fn read_slow(has_iw: bool, has_nvidia_smi: bool, timeout: Duration) -> Slow {
    let mut slow = Map::new();
    let mut iw = Map::new();
    if cfg!(target_os = "linux") {
        let wireless = parse::proc_net_wireless(
            &parse::read(Path::new("/proc/net/wireless")).unwrap_or_default(),
        );
        iw = wireless
            .keys()
            .filter(|_| has_iw)
            .map(|iface| {
                (
                    iface.clone(),
                    Value::Object(parse::iw_link(&run("iw", &["dev", iface, "link"], timeout))),
                )
            })
            .collect();
        slow.insert(
            "routes".into(),
            json!(parse::proc_net_route(
                &parse::read(Path::new("/proc/net/route")).unwrap_or_default()
            )),
        );
    } else {
        let route = parse::macos_route(&run("route", &["-n", "get", "default"], timeout));
        slow.insert(
            "routes".into(),
            json!(route.map(|(_, route)| vec![route]).unwrap_or_default()),
        );
        slow.insert(
            "battery".into(),
            parse::pmset_batt(&run("pmset", &["-g", "batt"], timeout)).unwrap_or(Value::Null),
        );
    }
    if has_nvidia_smi {
        let query =
            "--query-gpu=name,utilization.gpu,memory.used,memory.total,temperature.gpu,power.draw";
        slow.insert(
            "gpus".into(),
            json!(parse::nvidia_smi(&run(
                "nvidia-smi",
                &[query, "--format=csv,noheader,nounits"],
                timeout
            ))),
        );
    }
    let disks = Disks::new_with_refreshed_list();
    let root = disks
        .list()
        .iter()
        .filter(|d| Path::new("/").starts_with(d.mount_point()))
        .max_by_key(|d| d.mount_point().as_os_str().len());
    if let Some(disk) = root {
        let used = disk.total_space().saturating_sub(disk.available_space());
        slow.insert(
            "disk_percent".into(),
            json!(round(
                100.0 * used as f64 / disk.total_space().max(1) as f64,
                1
            )),
        );
    }
    slow.insert("uptime_s".into(), json!(System::uptime()));
    Slow {
        fields: slow,
        iw,
        temps: Map::new(),
    }
}

pub struct Sampler {
    system: System,
    networks: Networks,
    components: Components,
    last_net: Instant,
    last_disk: Option<(Instant, (u64, u64))>,
    /// the latest `read_slow`, refreshed on its own thread so a stuck command never delays a sample
    slow: Arc<Mutex<Slow>>,
}

impl Sampler {
    pub fn new(command_timeout: Duration, slow_period: Duration) -> Self {
        let mut system = System::new();
        // CPU usage is since the previous refresh; prime it so the first sample means something
        system.refresh_cpu_usage();
        refresh_processes(&mut system);
        let slow = Arc::new(Mutex::new(Slow::default()));
        let (has_iw, has_nvidia_smi) = (on_path("iw"), on_path("nvidia-smi"));
        let shared = Arc::clone(&slow);
        std::thread::Builder::new()
            .name("system_log_slow".into())
            .spawn(move || {
                // ~14 ms for an M5 Max's 84 GPU sensors, too slow to read every sample
                #[cfg(target_os = "macos")]
                let smc = crate::smc::Smc::open();
                loop {
                    #[allow(unused_mut)]
                    let mut readings = read_slow(has_iw, has_nvidia_smi, command_timeout);
                    #[cfg(target_os = "macos")]
                    {
                        readings.temps = smc_temps(smc.as_ref());
                    }
                    *shared.lock().unwrap_or_else(|e| e.into_inner()) = readings;
                    std::thread::sleep(slow_period);
                }
            })
            .expect("spawn the system_log_slow thread");
        Self {
            system,
            networks: Networks::new_with_refreshed_list(),
            components: Components::new_with_refreshed_list(),
            last_net: Instant::now(),
            last_disk: None,
            slow,
        }
    }

    /// One sample; rates (CPU %, network and disk throughput) are since the previous call.
    pub fn sample(&mut self) -> Value {
        self.system.refresh_cpu_usage();
        self.system.refresh_memory();
        self.networks.refresh(true);
        self.components.refresh(true);
        let per_core: Vec<f64> = self
            .system
            .cpus()
            .iter()
            .map(|c| round(c.cpu_usage() as f64, 1))
            .collect();
        let mhz: Vec<u64> = self
            .system
            .cpus()
            .iter()
            .map(|c| c.frequency())
            .filter(|&f| f > 0)
            .collect();
        let load = System::load_average();
        let (mut temps, mut unreadable) = (Map::new(), Vec::new());
        for component in self.components.list() {
            let Some(celsius) = component.temperature() else {
                continue;
            };
            // below absolute zero is a channel with nothing on it (an M5's `PMU tdev*` read about -9200), not a temperature
            if celsius < ABSOLUTE_ZERO_C {
                unreadable.push(component.label().to_string());
            } else {
                temps.insert(
                    component.label().to_string(),
                    json!(round(celsius as f64, 1)),
                );
            }
        }
        unreadable.sort();
        let sys_class = Path::new(SYS_CLASS);
        temps.extend(parse::thermal_zones(&sys_class.join("thermal")));
        temps.extend(
            self.slow
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .temps
                .clone(),
        );
        let (rails, fans) = parse::hwmon(&sys_class.join("hwmon"));
        let mut sample = json!({
            "cpu_percent": round(self.system.global_cpu_usage() as f64, 1),
            "cpu_per_core": per_core,
            "cpu_mhz": (!mhz.is_empty()).then(|| mhz.iter().sum::<u64>() / mhz.len() as u64),
            "load": [round(load.one, 2), round(load.five, 2), round(load.fifteen, 2)],
            "memory": {
                "total_mb": self.system.total_memory() >> 20,
                "available_mb": self.system.available_memory() >> 20,
                "percent": round(100.0 * self.system.used_memory() as f64 / self.system.total_memory().max(1) as f64, 1),
                "swap_used_mb": self.system.used_swap() >> 20,
            },
            "temps_c": temps,
            "temps_unreadable": unreadable,
            "power": {
                "rails_w": rails,
                "supplies": parse::power_supplies(&sys_class.join("power_supply")),
            },
            "fans_rpm": fans,
            "wifi": self.wifi(),
            "net": self.net_rates(),
            "disk_io": self.disk_rates(),
        });
        if let Some(throttled) =
            parse::read(Path::new(PI_THROTTLED)).and_then(|t| parse::pi_throttled(&t))
        {
            sample["pi_throttled"] = throttled;
        }
        if let Some(load) = JETSON_GPU_LOAD
            .iter()
            .find_map(|p| parse::read(Path::new(p))?.parse::<f64>().ok())
        {
            sample["gpus"] = json!([{ "name": "jetson", "util_percent": load / 10.0 }]);
        }
        for (key, value) in &self.slow.lock().unwrap_or_else(|e| e.into_inner()).fields {
            sample[key] = value.clone();
        }
        sample
    }

    fn wifi(&self) -> Value {
        let slow = self.slow.lock().unwrap_or_else(|e| e.into_inner());
        let mut wifi = parse::proc_net_wireless(
            &parse::read(Path::new("/proc/net/wireless")).unwrap_or_default(),
        );
        for (iface, entry) in wifi.iter_mut() {
            if let (Some(entry), Some(Value::Object(link))) =
                (entry.as_object_mut(), slow.iw.get(iface))
            {
                entry.extend(link.clone());
            }
            entry["addr"] = json!(self.ipv4(iface));
        }
        Value::Object(wifi)
    }

    fn ipv4(&self, iface: &str) -> Option<String> {
        let data = self.networks.list().get(iface)?;
        data.ip_networks()
            .iter()
            .find(|ip| ip.addr.is_ipv4())
            .map(|ip| ip.addr.to_string())
    }

    fn net_rates(&mut self) -> Value {
        let elapsed = self.last_net.elapsed().as_secs_f64().max(1e-3);
        self.last_net = Instant::now();
        let rates: Map<String, Value> = self
            .networks
            .list()
            .iter()
            .filter(|(iface, data)| {
                !iface.starts_with("lo") && (data.received() > 0 || data.transmitted() > 0)
            })
            .map(|(iface, data)| {
                (
                    iface.clone(),
                    json!({
                        "rx_kbps": round(data.received() as f64 * 8.0 / 1000.0 / elapsed, 1),
                        "tx_kbps": round(data.transmitted() as f64 * 8.0 / 1000.0 / elapsed, 1),
                        "errors": data.errors_on_received() + data.errors_on_transmitted(),
                    }),
                )
            })
            .collect();
        Value::Object(rates)
    }

    fn disk_rates(&mut self) -> Value {
        let Some(stats) = parse::read(Path::new("/proc/diskstats")) else {
            return Value::Null;
        };
        let is_disk = |name: &str| {
            !name.starts_with("loop")
                && !name.starts_with("ram")
                && Path::new("/sys/block").join(name).exists()
        };
        let now = (Instant::now(), parse::diskstats(&stats, is_disk));
        let last = self.last_disk.replace(now);
        let Some((at, (read, written))) = last else {
            return Value::Null;
        };
        let elapsed = at.elapsed().as_secs_f64().max(1e-3);
        // diskstats counts 512-byte sectors
        let mbps = |sectors: u64| round(sectors as f64 * 512.0 / 1048576.0 / elapsed, 2);
        json!({ "read_mbps": mbps(now.1 .0.saturating_sub(read)), "write_mbps": mbps(now.1 .1.saturating_sub(written)) })
    }

    /// The busiest processes by CPU since the previous call.
    pub fn top_processes(&mut self, count: usize, cmdline_chars: usize) -> Value {
        refresh_processes(&mut self.system);
        let mut rows: Vec<_> = self
            .system
            .processes()
            .values()
            .filter(|p| p.thread_kind().is_none())
            .collect();
        rows.sort_by(|a, b| b.cpu_usage().total_cmp(&a.cpu_usage()));
        let rows: Vec<Value> = rows
            .into_iter()
            .take(count)
            .map(|p| {
                let cmdline: String = p
                    .cmd()
                    .iter()
                    .map(|a| parse::redact_assignment(&a.to_string_lossy()))
                    .collect::<Vec<_>>()
                    .join(" ");
                json!({
                    "pid": p.pid().as_u32(),
                    "name": p.name().to_string_lossy(),
                    "cpu_percent": round(p.cpu_usage() as f64, 1),
                    "rss_mb": p.memory() >> 20,
                    "cmdline": cmdline.chars().take(cmdline_chars).collect::<String>(),
                })
            })
            .collect();
        json!(rows)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_hung_command_is_killed_at_the_timeout() {
        let start = Instant::now();
        assert_eq!(run("sleep", &["10"], Duration::from_millis(200)), "");
        assert!(start.elapsed() < Duration::from_secs(2));
        assert_eq!(run("echo", &["hi"], Duration::from_secs(5)), "hi\n");
    }
}
