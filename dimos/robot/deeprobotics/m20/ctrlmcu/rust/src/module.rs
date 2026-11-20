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
//! The M20's body control MCU (LEDs, fill lights, fans, lidar / camera / external-load / FP / GPS power) on the AOS
//! board's UART, in place of the vendor `ctrlmcu_node`. Each input is one vendor DDS topic with the vendor's int32
//! meaning; each request waits for the MCU's reply and is reported on `status`. Like the vendor, nothing is sent
//! unless an input (or `startup_commands`) asks, and there is no heartbeat.

use std::collections::{BTreeMap, VecDeque};
use std::io::{self, Read, Write};
use std::sync::mpsc::{channel, Receiver, RecvTimeoutError, Sender};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use dimos_module::{native_config, Input, Module, Output};
use lcm_msgs::std_msgs::{self, Int32, UInt32, UInt32MultiArray, UInt8MultiArray};
use m20_ctrlmcu::serial::{Access, SerialPort};
use m20_ctrlmcu::{
    hex, Command, Decoder, Frame, Func, LedEntry, Policy, Received, Reply, Sequence,
};
use serde_json::{json, Value};
use tokio::runtime::Handle;
use tracing::{info, warn};

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// The MCU's UART and rate (the vendor's: /dev/ttyS3, 115200 8N1).
    device: String,
    #[validate(range(min = 1))]
    baud: i64,
    /// Bytes asked for per read.
    #[validate(range(min = 1))]
    read_chunk_bytes: i64,
    /// A declared payload longer than this is taken as a false header.
    #[validate(range(min = 0, max = 65535))]
    max_payload_bytes: i64,
    /// Bytes held waiting for a whole frame before the oldest are dropped.
    #[validate(range(min = 10))]
    max_buffered_bytes: i64,
    /// How long each request waits for the MCU's reply, by input name.
    reply_timeout_ms: BTreeMap<String, i64>,
    /// Which inputs may reach the MCU; queries always may. Power covers lidar, video, ext_load, fp and gps.
    allow_lights: bool,
    allow_fans: bool,
    allow_power: bool,
    /// (input name, vendor int32) sent once the port opens, through the same gate as inputs.
    startup_commands: Vec<(String, i64)>,
    /// Do not open the port while a process with one of these names runs (two readers would split the replies).
    exclusive_with: Vec<String>,
}

#[derive(Module)]
#[module(name = "m20_ctrlmcu", setup = start)]
pub struct M20CtrlMcu {
    #[input(decode = UInt8MultiArray::decode, handler = on_led_status)]
    led_status: Input<UInt8MultiArray>,
    #[input(decode = Int32::decode, handler = on_fill_light)]
    fill_light: Input<Int32>,
    #[input(decode = Int32::decode, handler = on_fan_speed)]
    fan_speed: Input<Int32>,
    #[input(decode = Int32::decode, handler = on_fan_speed_query)]
    fan_speed_query: Input<Int32>,
    #[input(decode = Int32::decode, handler = on_ext_load_current_query)]
    ext_load_current_query: Input<Int32>,
    #[input(decode = Int32::decode, handler = on_ext_load_power)]
    ext_load_power: Input<Int32>,
    #[input(decode = Int32::decode, handler = on_host_led)]
    host_led: Input<Int32>,
    #[input(decode = Int32::decode, handler = on_ext_led)]
    ext_led: Input<Int32>,
    #[input(decode = Int32::decode, handler = on_fp_power)]
    fp_power: Input<Int32>,
    #[input(decode = Int32::decode, handler = on_lidar_power)]
    lidar_power: Input<Int32>,
    #[input(decode = Int32::decode, handler = on_gps_power)]
    gps_power: Input<Int32>,
    #[input(decode = Int32::decode, handler = on_video_power)]
    video_power: Input<Int32>,

    /// One JSON object per event: a reply, a timeout, a refusal, an unsolicited frame, a port error.
    #[output(encode = std_msgs::String::encode)]
    status: Output<std_msgs::String>,
    /// [tail, mid, head] from each fan speed query.
    #[output(encode = UInt32MultiArray::encode)]
    fan_speeds: Output<UInt32MultiArray>,
    /// The raw reading from each external-load current query.
    #[output(encode = UInt32::encode)]
    ext_load_current: Output<UInt32>,

    #[config]
    config: Config,

    events: Option<Sender<Event>>,
}

/// One request: the command and the int32 it came from (for the report).
#[derive(Debug)]
struct Job {
    command: Command,
    value: i64,
}

enum Event {
    Job(Job),
    Frame(Received),
    ReadFailed(io::Error),
}

macro_rules! vendor_inputs {
    ($($handler:ident => $func:expr),* $(,)?) => {
        $(async fn $handler(&mut self, msg: Int32) {
            self.vendor($func, msg.data as i64);
        })*
    };
}

impl M20CtrlMcu {
    async fn start(&mut self) {
        let config = self.config.clone();
        let jobs =
            startup_jobs(&config).unwrap_or_else(|error| panic!("m20_ctrlmcu config: {error}"));
        timeouts(&config).unwrap_or_else(|error| panic!("m20_ctrlmcu config: {error}"));
        let (sender, receiver) = channel();
        for job in jobs {
            let _ = sender.send(Event::Job(job));
        }
        let worker = Worker {
            config,
            events_in: sender.clone(),
            status: self.status.clone(),
            fan_speeds: self.fan_speeds.clone(),
            ext_load_current: self.ext_load_current.clone(),
            handle: Handle::current(),
        };
        self.events = Some(sender);
        std::thread::spawn(move || worker.run(receiver));
    }

    fn submit(&self, job: Job) {
        if let Some(events) = &self.events {
            let _ = events.send(Event::Job(job));
        }
    }

    fn vendor(&self, func: Func, value: i64) {
        match Command::from_vendor(func, value as i32) {
            Some(command) => self.submit(Job { command, value }),
            None => warn!(%func, "not an int32 command"),
        }
    }

    async fn on_led_status(&mut self, msg: UInt8MultiArray) {
        match led_entries(&msg.data) {
            Some(entries) => self.submit(Job {
                command: Command::LedStatus(entries),
                value: 0,
            }),
            None => warn!(
                len = msg.data.len(),
                "led_status wants 6 bytes per entry (type, dir, num, cycle, color0, color1)"
            ),
        }
    }

    vendor_inputs! {
        on_fill_light => Func::FillLight,
        on_fan_speed => Func::FanSpeed,
        on_fan_speed_query => Func::FanSpeedQuery,
        on_ext_load_current_query => Func::ExtLoadCurrentQuery,
        on_ext_load_power => Func::ExtLoadPower,
        on_host_led => Func::HostLed,
        on_ext_led => Func::ExtLed,
        on_fp_power => Func::FpPower,
        on_lidar_power => Func::LidarPower,
        on_gps_power => Func::GpsPower,
        on_video_power => Func::VideoPower,
    }
}

/// Up to two LED entries, six bytes each: type, dir, num, cycle, color0, color1.
pub fn led_entries(data: &[u8]) -> Option<Vec<LedEntry>> {
    if data.is_empty() || !data.len().is_multiple_of(6) || data.len() > 12 {
        return None;
    }
    Some(
        data.chunks(6)
            .map(|e| LedEntry {
                kind: e[0],
                dir: e[1] != 0,
                num: e[2],
                cycle: e[3],
                color: [e[4], e[5]],
            })
            .collect(),
    )
}

fn startup_jobs(config: &Config) -> Result<Vec<Job>, String> {
    config
        .startup_commands
        .iter()
        .map(|(name, value)| {
            let func = Func::from_name(name)
                .ok_or_else(|| format!("startup_commands: unknown input {name:?}"))?;
            let command = Command::from_vendor(func, *value as i32)
                .ok_or_else(|| format!("startup_commands: {name} takes no int32"))?;
            Ok(Job {
                command,
                value: *value,
            })
        })
        .collect()
}

fn timeouts(config: &Config) -> Result<BTreeMap<u8, Duration>, String> {
    Func::ALL
        .into_iter()
        .map(|func| {
            let ms = config
                .reply_timeout_ms
                .get(func.name())
                .ok_or_else(|| format!("reply_timeout_ms has no {}", func.name()))?;
            if *ms < 1 {
                return Err(format!("reply_timeout_ms {} must be >= 1", func.name()));
            }
            Ok((func as u8, Duration::from_millis(*ms as u64)))
        })
        .collect()
}

/// Whether a process named one of `names` is running (Linux /proc).
fn running_process(names: &[String]) -> Option<String> {
    let entries = std::fs::read_dir("/proc").ok()?;
    entries.flatten().find_map(|entry| {
        let comm = std::fs::read_to_string(entry.path().join("comm")).ok()?;
        let comm = comm.trim();
        // The kernel keeps the first 15 bytes of a process name.
        names
            .iter()
            .any(|name| name.chars().take(15).collect::<String>() == comm)
            .then(|| comm.to_string())
    })
}

fn wall_s() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
}

fn time_s(time: SystemTime) -> f64 {
    time.duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
}

fn frame_json(frame: &Frame) -> Value {
    json!({
        "sequence": frame.sequence,
        "frame_type": frame.frame_type,
        "func": frame.func,
        "func_name": Func::from_u8(frame.func).map(Func::name),
        "crc_ok": frame.crc_ok(),
        "payload": hex(&frame.payload),
    })
}

struct Worker {
    config: Config,
    events_in: Sender<Event>,
    status: Output<std_msgs::String>,
    fan_speeds: Output<UInt32MultiArray>,
    ext_load_current: Output<UInt32>,
    handle: Handle,
}

impl Worker {
    fn run(self, events: Receiver<Event>) {
        let timeouts = timeouts(&self.config).expect("checked at start");
        let policy = Policy {
            allow_lights: self.config.allow_lights,
            allow_fans: self.config.allow_fans,
            allow_power: self.config.allow_power,
        };
        let mut sequence = Sequence::default();
        let mut port = self.open();
        let mut waiting: VecDeque<Job> = VecDeque::new();
        loop {
            let job = match waiting.pop_front() {
                Some(job) => job,
                None => match events.recv() {
                    Ok(Event::Job(job)) => job,
                    Ok(Event::Frame(received)) => {
                        self.report(
                            json!({"event": "unsolicited", "frame": frame_json(&received.frame),
                            "host_time": time_s(received.host_time)}),
                        );
                        continue;
                    }
                    Ok(Event::ReadFailed(error)) => {
                        port = None;
                        self.report(json!({"event": "read_failed", "error": error.to_string()}));
                        continue;
                    }
                    Err(_) => return,
                },
            };
            let func = job.command.func();
            let base = json!({"func": func.name(), "command": job.value, "effect": format!("{:?}", job.command.effect())});
            if !policy.allows(job.command.effect()) {
                warn!(%func, value = job.value, "refused: not allowed by config");
                self.report(merge(base, json!({"event": "refused", "result": -1})));
                continue;
            }
            if port.is_none() {
                port = self.open();
            }
            let Some(writer) = port.as_mut() else {
                self.report(merge(base, json!({"event": "port_closed", "result": 2})));
                continue;
            };
            let request = job.command.frame(sequence.take());
            let started = Instant::now();
            if let Err(error) = writer
                .write_all(&request.encode())
                .and_then(|()| writer.flush())
            {
                port = None;
                self.report(merge(
                    base,
                    json!({"event": "write_failed", "result": 1, "error": error.to_string()}),
                ));
                continue;
            }
            info!(%func, value = job.value, request = %hex(&request.encode()), "sent");
            let deadline = started + timeouts[&(func as u8)];
            let outcome = loop {
                match events.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
                    Ok(Event::Frame(received)) => break Ok(received),
                    Ok(Event::Job(next)) => waiting.push_back(next),
                    Ok(Event::ReadFailed(error)) => break Err((4, Some(error))),
                    Err(RecvTimeoutError::Timeout) => break Err((3, None)),
                    Err(RecvTimeoutError::Disconnected) => return,
                }
            };
            let base = merge(base, json!({"request": frame_json(&request)}));
            match outcome {
                Ok(received) => {
                    let round_trip_ms = started.elapsed().as_secs_f64() * 1e3;
                    let reply = Reply::read(func, &received.frame);
                    self.publish_reading(&reply);
                    self.report(merge(
                        base,
                        json!({"event": "reply", "result": 0, "round_trip_ms": round_trip_ms,
                        "reply": frame_json(&received.frame), "decoded": format!("{reply:?}")}),
                    ));
                }
                Err((code, error)) => {
                    if error.is_some() {
                        port = None;
                    }
                    let event = if code == 3 { "timeout" } else { "read_failed" };
                    warn!(%func, value = job.value, event, "no reply");
                    self.report(merge(
                        base,
                        json!({"event": event, "result": code,
                        "error": error.map(|e| e.to_string())}),
                    ));
                }
            }
        }
    }

    /// Open the port and start its reader, unless another owner runs; nothing is written.
    fn open(&self) -> Option<SerialPort> {
        if let Some(owner) = running_process(&self.config.exclusive_with) {
            self.report(
                json!({"event": "port_busy", "owner": owner, "device": self.config.device}),
            );
            return None;
        }
        let opened = SerialPort::open(
            &self.config.device,
            self.config.baud as u32,
            Access::ReadWrite,
        )
        .and_then(|port| Ok((port.try_clone()?, port)));
        match opened {
            Ok((reader, writer)) => {
                let decoder = Decoder::new(
                    self.config.max_payload_bytes as usize,
                    self.config.max_buffered_bytes as usize,
                );
                let chunk = self.config.read_chunk_bytes as usize;
                let events = self.events_in.clone();
                std::thread::spawn(move || read_forever(reader, decoder, chunk, events));
                info!(device = %self.config.device, baud = self.config.baud, "port open");
                self.report(json!({"event": "port_open", "device": self.config.device}));
                Some(writer)
            }
            Err(error) => {
                warn!(%error, device = %self.config.device, "cannot open the port");
                self.report(json!({"event": "port_error", "device": self.config.device, "error": error.to_string()}));
                None
            }
        }
    }

    fn publish_reading(&self, reply: &Reply) {
        let result = match reply {
            Reply::FanSpeeds(speeds) => {
                self.handle
                    .block_on(self.fan_speeds.publish(&UInt32MultiArray {
                        data: vec![speeds.tail as i32, speeds.mid as i32, speeds.head as i32],
                        ..UInt32MultiArray::default()
                    }))
            }
            Reply::ExtLoadCurrent(current) => {
                self.handle.block_on(self.ext_load_current.publish(&UInt32 {
                    data: *current as i32,
                }))
            }
            _ => return,
        };
        if let Err(error) = result {
            warn!(%error, "cannot publish the reading");
        }
    }

    fn report(&self, event: Value) {
        let event = merge(event, json!({"time": wall_s()}));
        if let Err(error) = self.handle.block_on(self.status.publish(&std_msgs::String {
            data: event.to_string(),
        })) {
            warn!(%error, "cannot publish status");
        }
    }
}

fn merge(mut base: Value, extra: Value) -> Value {
    if let (Some(base), Value::Object(extra)) = (base.as_object_mut(), extra) {
        base.extend(extra);
    }
    base
}

/// Decode the port onto `events` until a read fails (the worker reopens on its next request).
fn read_forever(
    mut port: impl Read,
    mut decoder: Decoder,
    chunk_bytes: usize,
    events: Sender<Event>,
) {
    let mut chunk = vec![0u8; chunk_bytes.max(1)];
    loop {
        let count = match port.read(&mut chunk) {
            Ok(0) => {
                let _ = events.send(Event::ReadFailed(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "port closed",
                )));
                return;
            }
            Ok(count) => count,
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error) => {
                let _ = events.send(Event::ReadFailed(error));
                return;
            }
        };
        let host_time = SystemTime::now();
        decoder.push(&chunk[..count]);
        while let Some(frame) = decoder.next_frame() {
            if events
                .send(Event::Frame(Received { frame, host_time }))
                .is_err()
            {
                return;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn led_entries_take_six_bytes_each() {
        let entries = led_entries(&[0, 0, 0, 1, 3, 0, 1, 1, 3, 1, 2, 0]).unwrap();
        assert_eq!(
            entries[0],
            LedEntry {
                kind: 0,
                dir: false,
                num: 0,
                cycle: 1,
                color: [3, 0]
            }
        );
        assert_eq!(
            entries[1],
            LedEntry {
                kind: 1,
                dir: true,
                num: 3,
                cycle: 1,
                color: [2, 0]
            }
        );
        assert_eq!(led_entries(&[1, 2, 3]), None);
        assert_eq!(led_entries(&[0; 18]), None);
        let frame = Command::LedStatus(entries).frame(1).encode();
        assert_eq!(
            frame[10..],
            [1, 0, 0, 0, 1, 3, 0, 0, 2, 1, 1, 3, 1, 2, 0, 0]
        );
    }

    #[test]
    fn merge_adds_fields() {
        assert_eq!(
            merge(json!({"a": 1}), json!({"b": 2})),
            json!({"a": 1, "b": 2})
        );
    }
}
