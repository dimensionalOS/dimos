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

//! Samples the machine at `sample_hz` into a ring of the last `buffer_s`, publishes one sample every
//! `publish_period_s` as a "health" record, and when something goes wrong (an alert starting, or `bad_event`)
//! publishes every sample since the last dump as a "health_dump" along with the busiest processes. It also
//! publishes the run's *.jsonl logs line by line, and passes `robot_health` through. Every record is a JSON
//! object with a "type" and "t" (unix seconds) on `system_log`.

use std::collections::{BTreeMap, VecDeque};
use std::path::PathBuf;
use std::sync::mpsc::{self, Receiver, Sender};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use dimos_module::{native_config, Input, Module, Output};
use lcm_msgs::std_msgs::{self, Bool};
use serde_json::{json, Value};
use tracing::warn;

use crate::log_tail::JsonlTail;
use crate::machine::Sampler;

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// How often the machine is sampled into the ring; what a dump has.
    #[validate(range(min = 0.1, max = 100.0))]
    sample_hz: f64,
    /// Seconds between the "health" records published in normal operation.
    #[validate(range(min = 0.01, max = 3600.0))]
    publish_period_s: f64,
    /// Seconds between refreshes of what needs a subprocess (iw, nvidia-smi, route, pmset) or changes slowly.
    #[validate(range(min = 0.1, max = 3600.0))]
    slow_period_s: f64,
    /// Seconds a slow refresh waits on each subprocess before killing it, so a hung one can't freeze those readings.
    #[validate(range(min = 0.1, max = 600.0))]
    command_timeout_s: f64,
    /// Seconds between "processes" records (they're also published with every dump).
    #[validate(range(min = 0.1, max = 3600.0))]
    process_period_s: f64,
    #[validate(range(min = 1, max = 1000))]
    top_processes: i64,
    /// Seconds of samples the ring holds.
    #[validate(range(min = 1.0, max = 3600.0))]
    buffer_s: f64,
    #[validate(range(min = 0.01, max = 60.0))]
    log_poll_s: f64,
    /// Directory whose *.jsonl files are published; empty for the run's (DIMOS_RUN_LOG_DIR).
    log_dir: String,
    /// Characters of each process's command line kept in "processes" records.
    #[validate(range(min = 0, max = 100000))]
    cmdline_chars: i64,
    /// Seconds after start that the one "identity" record (host, user, MACs, addresses, ...) is published.
    #[validate(range(min = 0.0, max = 86400.0))]
    identity_delay_s: f64,
    /// Alert (and dump) when total CPU goes above this.
    cpu_alert_percent: f64,
    /// Alert when memory use goes above this.
    memory_alert_percent: f64,
    /// Alert when a Wi-Fi interface's signal drops below this.
    weak_signal_dbm: f64,
    /// Alert when any temperature sensor goes above this.
    hot_celsius: f64,
}

#[derive(Module)]
#[module(name = "system_log", setup = start)]
pub struct SystemLog {
    #[input(decode = std_msgs::String::decode)]
    robot_health: Input<std_msgs::String>,

    #[input(decode = Bool::decode)]
    bad_event: Input<Bool>,

    #[output(encode = std_msgs::String::encode)]
    system_log: Output<std_msgs::String>,

    #[config]
    config: Config,

    dumps: Option<Sender<String>>,
}

impl SystemLog {
    async fn start(&mut self) {
        let (sender, receiver) = mpsc::channel();
        self.dumps = Some(sender);
        let config = self.config.clone();
        let output = self.system_log.clone();
        let runtime = tokio::runtime::Handle::current();
        std::thread::Builder::new()
            .name("system_log".into())
            .spawn(move || Worker::new(config, output, runtime, receiver).run())
            .expect("spawn the system_log thread");
        let (delay, timeout) = (
            Duration::from_secs_f64(self.config.identity_delay_s),
            Duration::from_secs_f64(self.config.command_timeout_s),
        );
        let output = self.system_log.clone();
        let runtime = tokio::runtime::Handle::current();
        std::thread::Builder::new()
            .name("system_log_identity".into())
            .spawn(move || {
                std::thread::sleep(delay);
                let msg = std_msgs::String {
                    data: record("identity", crate::identity::read(timeout)).to_string(),
                };
                if let Err(error) = runtime.block_on(output.publish(&msg)) {
                    warn!(%error, "publishing the identity record failed");
                }
            })
            .expect("spawn the system_log_identity thread");
    }

    async fn handle_robot_health(&mut self, msg: std_msgs::String) {
        self.system_log.publish(&msg).await.ok();
    }

    async fn handle_bad_event(&mut self, msg: Bool) {
        if let (true, Some(dumps)) = (msg.data, &self.dumps) {
            dumps.send("bad_event".into()).ok();
        }
    }
}

/// Whether a deadline has come, and if so the next one: a period on from it, so it doesn't drift by a tick's jitter.
fn due(next: &mut Instant, now: Instant, period_s: f64) -> bool {
    if now < *next {
        return false;
    }
    *next += Duration::from_secs_f64(period_s);
    if *next <= now {
        *next = now + Duration::from_secs_f64(period_s);
    }
    true
}

fn now_s() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
}

fn record(kind: &str, fields: Value) -> Value {
    let mut record = json!({ "type": kind, "t": now_s() });
    if let (Some(record), Value::Object(fields)) = (record.as_object_mut(), fields) {
        record.extend(fields);
    }
    record
}

/// Everything wrong in one sample, by condition → what to say about it.
pub fn alerts(sample: &Value, config: &Config) -> BTreeMap<String, String> {
    let mut wrong = BTreeMap::new();
    let number = |v: &Value| v.as_f64().unwrap_or(0.0);
    if number(&sample["cpu_percent"]) > config.cpu_alert_percent {
        wrong.insert("cpu".into(), format!("CPU {}%", sample["cpu_percent"]));
    }
    if number(&sample["memory"]["percent"]) > config.memory_alert_percent {
        wrong.insert(
            "memory".into(),
            format!("memory {}% used", sample["memory"]["percent"]),
        );
    }
    for (iface, wifi) in sample["wifi"].as_object().into_iter().flatten() {
        if wifi["connected"] == false {
            wrong.insert(format!("wifi:{iface}"), format!("{iface} not connected"));
        } else if wifi["level_dbm"]
            .as_f64()
            .is_some_and(|dbm| dbm < config.weak_signal_dbm)
        {
            wrong.insert(
                format!("signal:{iface}"),
                format!("{iface} weak signal {} dBm", wifi["level_dbm"]),
            );
        }
    }
    for (sensor, celsius) in sample["temps_c"].as_object().into_iter().flatten() {
        if number(celsius) > config.hot_celsius {
            wrong.insert(format!("hot:{sensor}"), format!("{sensor} at {celsius} C"));
        }
    }
    if let Some(now) = sample["pi_throttled"]["now"]
        .as_array()
        .filter(|now| !now.is_empty())
    {
        wrong.insert(
            "pi_throttled".into(),
            format!("Pi throttled: {}", json!(now)),
        );
    }
    wrong
}

/// The network the machine is on: a change (new SSID, address or route) is worth a dump; the signal wobbling isn't.
fn link_key(sample: &Value) -> String {
    let wifi: Vec<String> = sample["wifi"]
        .as_object()
        .into_iter()
        .flatten()
        .map(|(iface, w)| format!("{iface} {} {}", w["ssid"], w["addr"]))
        .collect();
    format!("{} | {}", wifi.join(", "), sample["routes"])
}

/// Ring of timestamped samples; `since_dump` is what hasn't been dumped yet.
struct Ring {
    samples: VecDeque<Value>,
    capacity: usize,
    dumped_to: f64,
}

impl Ring {
    fn push(&mut self, sample: Value) {
        if self.samples.len() == self.capacity {
            self.samples.pop_front();
        }
        self.samples.push_back(sample);
    }

    fn take_undumped(&mut self) -> Vec<Value> {
        let since = self.dumped_to;
        let samples: Vec<Value> = self
            .samples
            .iter()
            .filter(|s| s["t"].as_f64().unwrap_or(0.0) > since)
            .cloned()
            .collect();
        self.dumped_to = samples
            .last()
            .and_then(|s| s["t"].as_f64())
            .unwrap_or(since);
        samples
    }
}

struct Worker {
    config: Config,
    output: Output<std_msgs::String>,
    runtime: tokio::runtime::Handle,
    dumps: Receiver<String>,
    sampler: Sampler,
    ring: Ring,
    tail: Option<JsonlTail>,
}

impl Worker {
    fn new(
        config: Config,
        output: Output<std_msgs::String>,
        runtime: tokio::runtime::Handle,
        dumps: Receiver<String>,
    ) -> Self {
        let capacity = (config.buffer_s * config.sample_hz).ceil() as usize;
        let log_dir = match config.log_dir.as_str() {
            "" => std::env::var_os("DIMOS_RUN_LOG_DIR").map(PathBuf::from),
            dir => Some(PathBuf::from(dir)),
        };
        if log_dir.is_none() {
            warn!(
                "no run log directory (DIMOS_RUN_LOG_DIR), so dimos' logs won't be on system_log"
            );
        }
        let sampler = Sampler::new(
            Duration::from_secs_f64(config.command_timeout_s),
            Duration::from_secs_f64(config.slow_period_s),
        );
        Self {
            config,
            output,
            runtime,
            dumps,
            sampler,
            ring: Ring {
                samples: VecDeque::with_capacity(capacity),
                capacity: capacity.max(1),
                dumped_to: 0.0,
            },
            tail: log_dir.map(JsonlTail::new),
        }
    }

    fn publish(&self, record: &Value) {
        let msg = std_msgs::String {
            data: record.to_string(),
        };
        if let Err(error) = self.runtime.block_on(self.output.publish(&msg)) {
            warn!(%error, "publishing system_log failed");
        }
    }

    fn run(mut self) {
        let period = Duration::from_secs_f64(1.0 / self.config.sample_hz);
        let start = Instant::now();
        let (mut next_tick, mut next_publish, mut next_processes, mut next_log) =
            (start, start, start, start);
        let mut active: BTreeMap<String, String> = BTreeMap::new();
        let mut last_link = String::new();
        loop {
            let now = Instant::now();
            if due(&mut next_log, now, self.config.log_poll_s) {
                self.publish_logs();
            }
            let mut sample = self.sampler.sample();
            sample["t"] = json!(now_s());
            self.ring.push(sample.clone());

            let mut reasons: Vec<String> = self.dumps.try_iter().collect();
            let link = link_key(&sample);
            if !last_link.is_empty() && link != last_link {
                reasons.push(format!("network changed: {link}"));
            }
            last_link = link;
            let now_wrong = alerts(&sample, &self.config);
            for (condition, message) in &now_wrong {
                if !active.contains_key(condition) {
                    self.publish(&record(
                        "alert",
                        json!({ "condition": condition, "message": message }),
                    ));
                    reasons.push(message.clone());
                }
            }
            active = now_wrong;

            if !reasons.is_empty() {
                let samples = self.ring.take_undumped();
                self.publish(&record(
                    "health_dump",
                    json!({ "reason": reasons.join("; "), "samples": samples }),
                ));
            } else if due(&mut next_publish, now, self.config.publish_period_s) {
                sample["type"] = json!("health");
                self.publish(&sample);
            }
            if due(&mut next_processes, now, self.config.process_period_s) || !reasons.is_empty() {
                let top = self.sampler.top_processes(
                    self.config.top_processes as usize,
                    self.config.cmdline_chars as usize,
                );
                self.publish(&record("processes", json!({ "top": top })));
            }

            next_tick += period;
            let now = Instant::now();
            if next_tick > now {
                std::thread::sleep(next_tick - now);
            } else {
                next_tick = now; // fell behind: don't burst to catch up
            }
        }
    }

    fn publish_logs(&mut self) {
        let Some(tail) = self.tail.as_mut() else {
            return;
        };
        for (file, line) in tail.read() {
            let parsed = serde_json::from_str::<Value>(&line).unwrap_or(Value::String(line));
            self.publish(&record(
                "dimos_log",
                json!({ "file": file, "record": parsed }),
            ));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> Config {
        Config {
            sample_hz: 4.0,
            publish_period_s: 1.0,
            slow_period_s: 5.0,
            command_timeout_s: 2.0,
            process_period_s: 10.0,
            top_processes: 12,
            buffer_s: 120.0,
            log_poll_s: 0.5,
            log_dir: String::new(),
            cmdline_chars: 200,
            identity_delay_s: 30.0,
            cpu_alert_percent: 95.0,
            memory_alert_percent: 95.0,
            weak_signal_dbm: -80.0,
            hot_celsius: 90.0,
        }
    }

    #[test]
    fn flags_what_looks_wrong() {
        let calm = json!({ "cpu_percent": 30.0, "memory": { "percent": 40.0 },
            "wifi": { "wlan0": { "connected": true, "level_dbm": -50.0 } }, "temps_c": { "cpu": 60.0 } });
        assert!(alerts(&calm, &config()).is_empty());
        let bad = json!({ "cpu_percent": 99.0, "memory": { "percent": 97.0 },
            "wifi": { "wlan0": { "connected": true, "level_dbm": -86.0 }, "wlan1": { "connected": false } },
            "temps_c": { "cpu": 95.0 }, "pi_throttled": { "now": ["undervoltage"] } });
        let keys: Vec<String> = alerts(&bad, &config()).into_keys().collect();
        assert_eq!(
            keys,
            vec![
                "cpu",
                "hot:cpu",
                "memory",
                "pi_throttled",
                "signal:wlan0",
                "wifi:wlan1"
            ]
        );
    }

    #[test]
    fn ring_dumps_each_sample_once_and_drops_the_oldest() {
        let mut ring = Ring {
            samples: VecDeque::new(),
            capacity: 3,
            dumped_to: 0.0,
        };
        for t in 1..=4 {
            ring.push(json!({ "t": t as f64 }));
        }
        let ts = |samples: Vec<Value>| {
            samples
                .iter()
                .map(|s| s["t"].as_f64().unwrap())
                .collect::<Vec<_>>()
        };
        assert_eq!(ts(ring.take_undumped()), vec![2.0, 3.0, 4.0]);
        ring.push(json!({ "t": 5.0 }));
        assert_eq!(ts(ring.take_undumped()), vec![5.0]);
        assert_eq!(ts(ring.take_undumped()), Vec::<f64>::new());
    }

    #[test]
    fn link_key_ignores_signal() {
        let a = json!({ "wifi": { "wlan0": { "ssid": "dimos", "addr": "10.0.0.2", "level_dbm": -50.0 } }, "routes": ["wlan0 via 10.0.0.1"] });
        let mut b = a.clone();
        b["wifi"]["wlan0"]["level_dbm"] = json!(-70.0);
        assert_eq!(link_key(&a), link_key(&b));
        b["wifi"]["wlan0"]["ssid"] = json!("other");
        assert_ne!(link_key(&a), link_key(&b));
    }
}
