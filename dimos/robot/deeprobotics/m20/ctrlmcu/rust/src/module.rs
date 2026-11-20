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
use std::sync::mpsc::{channel, Receiver, RecvTimeoutError, Sender, TryRecvError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use dimos_module::{native_config, Input, Module, Output};
use lcm_msgs::std_msgs::{self, Int32, UInt32, UInt32MultiArray, UInt8MultiArray};
use m20_ctrlmcu::serial::{Access, ReadStopper, SerialPort};
use m20_ctrlmcu::{
    hex, Command, Decoder, FanSpeeds, Frame, Func, LedEntry, Policy, Received, Reply, Sequence,
};
use serde_json::{json, Value};
use tokio::runtime::Handle;
use tracing::{error, info, warn};
use validator::ValidationError;

#[native_config]
#[derive(Clone)]
#[validate(schema(function = "check_config"))]
pub struct Config {
    /// The MCU's UART and rate (the vendor's: /dev/ttyS3, 115200 8N1).
    device: String,
    #[validate(range(min = 1))]
    baud: i64,
    /// Bytes asked for per read.
    #[validate(range(min = 1, max = 1048576))]
    read_chunk_bytes: i64,
    /// A declared payload longer than this is taken as a false header.
    #[validate(range(min = 0, max = 65535))]
    max_payload_bytes: i64,
    /// Bytes held waiting for a whole frame before the oldest are dropped.
    #[validate(range(min = 10, max = 16777216))]
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

/// The cross-field checks, so a bad config fails validation at startup instead of panicking in setup.
fn check_config(config: &Config) -> Result<(), ValidationError> {
    let fail = |message: String| {
        let mut error = ValidationError::new("config");
        error.message = Some(message.into());
        error
    };
    if u32::try_from(config.baud).is_err() {
        return Err(fail(format!("baud {} does not fit a u32", config.baud)));
    }
    startup_jobs(config).map_err(fail)?;
    timeouts(config).map_err(fail)?;
    Ok(())
}

#[derive(Module)]
#[module(name = "m20_ctrlmcu", setup = start, teardown = stop)]
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
    worker: Option<JoinHandle<()>>,
}

/// One request: the command and the int32 it came from (for the report).
#[derive(Debug)]
struct Job {
    command: Command,
    value: i64,
}

/// What the worker waits on. Frames and read failures carry the number of the port opening they came from, so ones
/// from a port already closed are dropped.
enum Event {
    Job(Job),
    Frame(u64, Received),
    ReadFailed(u64, io::Error),
    Stop,
}

macro_rules! vendor_inputs {
    ($($handler:ident => $func:expr),* $(,)?) => {
        $(async fn $handler(&mut self, msg: Int32) {
            self.vendor($func, msg.data);
        })*
    };
}

impl M20CtrlMcu {
    async fn start(&mut self) {
        let config = self.config.clone();
        // Both were checked by check_config; this only keeps a failure from panicking.
        let (jobs, timeouts) = match (startup_jobs(&config), timeouts(&config)) {
            (Ok(jobs), Ok(timeouts)) => (jobs, timeouts),
            (Err(problem), _) | (_, Err(problem)) => {
                error!(%problem, "m20_ctrlmcu config: not starting");
                return;
            }
        };
        let (sender, receiver) = channel();
        for job in jobs {
            let _ = sender.send(Event::Job(job));
        }
        let worker = Worker {
            config,
            timeouts,
            events_in: sender.clone(),
            sink: Outputs {
                status: self.status.clone(),
                fan_speeds: self.fan_speeds.clone(),
                ext_load_current: self.ext_load_current.clone(),
                handle: Handle::current(),
            },
            openings: 0,
        };
        self.events = Some(sender);
        self.worker = Some(std::thread::spawn(move || worker.run(receiver)));
    }

    /// End the worker (it closes the port and joins its reader).
    async fn stop(&mut self) {
        if let Some(events) = self.events.take() {
            let _ = events.send(Event::Stop);
        }
        if let Some(worker) = self.worker.take() {
            if tokio::task::spawn_blocking(move || worker.join())
                .await
                .is_err()
            {
                warn!("the worker thread could not be joined");
            }
        }
    }

    fn submit(&self, job: Job) {
        if let Some(events) = &self.events {
            let _ = events.send(Event::Job(job));
        }
    }

    fn vendor(&self, func: Func, value: i32) {
        match Command::from_vendor(func, value) {
            Some(command) => self.submit(Job {
                command,
                value: i64::from(value),
            }),
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
            let int32 = i32::try_from(*value)
                .map_err(|_| format!("startup_commands: {name} value {value} is not an int32"))?;
            let command = Command::from_vendor(func, int32)
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
            let ms = u64::try_from(*ms)
                .ok()
                .filter(|ms| *ms >= 1)
                .ok_or_else(|| format!("reply_timeout_ms {} must be >= 1", func.name()))?;
            Ok((func as u8, Duration::from_millis(ms)))
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
    time_s(SystemTime::now())
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

/// An open port: its writer, and its reader thread, which is stopped and joined when this is dropped.
struct Port {
    writer: SerialPort,
    opening: u64,
    stopper: ReadStopper,
    reader: Option<JoinHandle<()>>,
}

impl Drop for Port {
    fn drop(&mut self) {
        self.stopper.stop();
        if let Some(reader) = self.reader.take() {
            let _ = reader.join();
        }
    }
}

/// Where the worker's reports and readings go.
trait Sink: Send + 'static {
    fn status(&self, event: String);
    fn fan_speeds(&self, speeds: FanSpeeds);
    fn ext_load_current(&self, current: u32);
}

/// The module's outputs.
struct Outputs {
    status: Output<std_msgs::String>,
    fan_speeds: Output<UInt32MultiArray>,
    ext_load_current: Output<UInt32>,
    handle: Handle,
}

impl Sink for Outputs {
    fn status(&self, event: String) {
        if let Err(error) = self
            .handle
            .block_on(self.status.publish(&std_msgs::String { data: event }))
        {
            warn!(%error, "cannot publish status");
        }
    }

    // LCM has no unsigned ints: the u32 readings go out bit for bit in int32 fields.
    fn fan_speeds(&self, speeds: FanSpeeds) {
        let message = UInt32MultiArray {
            data: vec![speeds.tail as i32, speeds.mid as i32, speeds.head as i32],
            ..UInt32MultiArray::default()
        };
        if let Err(error) = self.handle.block_on(self.fan_speeds.publish(&message)) {
            warn!(%error, "cannot publish the fan speeds");
        }
    }

    fn ext_load_current(&self, current: u32) {
        let message = UInt32 {
            data: current as i32,
        };
        if let Err(error) = self
            .handle
            .block_on(self.ext_load_current.publish(&message))
        {
            warn!(%error, "cannot publish the load current");
        }
    }
}

struct Worker<S: Sink> {
    config: Config,
    timeouts: BTreeMap<u8, Duration>,
    events_in: Sender<Event>,
    sink: S,
    /// How many times the port has been opened (the number of the current opening).
    openings: u64,
}

/// What the worker should do after an event that is not a reply.
enum Next {
    Continue,
    ClosePort,
    Stop,
}

impl<S: Sink> Worker<S> {
    fn run(mut self, events: Receiver<Event>) {
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
                    Ok(event) => match self.idle_event(event, &port, &mut waiting) {
                        Next::Continue => continue,
                        Next::ClosePort => {
                            port = None;
                            continue;
                        }
                        Next::Stop => return,
                    },
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
            // A frame already queued (a late reply to an earlier request) must not be taken as this one's reply.
            loop {
                let event = match events.try_recv() {
                    Ok(event) => event,
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => return,
                };
                match self.idle_event(event, &port, &mut waiting) {
                    Next::Continue => {}
                    Next::ClosePort => port = None,
                    Next::Stop => return,
                }
            }
            if port.is_none() {
                port = self.open();
            }
            let Some(open) = port.as_mut() else {
                self.report(merge(base, json!({"event": "port_closed", "result": 2})));
                continue;
            };
            let opening = open.opening;
            let request = job.command.frame(sequence.take());
            let encoded = request.encode();
            let started = Instant::now();
            if let Err(error) = open
                .writer
                .write_all(&encoded)
                .and_then(|()| open.writer.flush())
            {
                port = None;
                self.report(merge(
                    base,
                    json!({"event": "write_failed", "result": 1, "error": error.to_string()}),
                ));
                continue;
            }
            info!(%func, value = job.value, request = %hex(&encoded), "sent");
            let timeout = self
                .timeouts
                .get(&(func as u8))
                .copied()
                .unwrap_or_else(|| func.vendor_timeout());
            let deadline = started + timeout;
            let outcome = loop {
                match events.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
                    Ok(Event::Frame(from, received)) if from == opening => break Ok(received),
                    Ok(Event::ReadFailed(from, error)) if from == opening => {
                        break Err((4, Some(error)))
                    }
                    // From a port already closed.
                    Ok(Event::Frame(..) | Event::ReadFailed(..)) => {}
                    Ok(Event::Job(next)) => waiting.push_back(next),
                    Ok(Event::Stop) => return,
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

    /// An event while no request waits: frames are unsolicited, jobs queue, a read failure closes the port.
    fn idle_event(&self, event: Event, port: &Option<Port>, waiting: &mut VecDeque<Job>) -> Next {
        let current = port.as_ref().map(|port| port.opening);
        match event {
            Event::Job(job) => waiting.push_back(job),
            Event::Frame(from, received) if Some(from) == current => self.report(
                json!({"event": "unsolicited", "frame": frame_json(&received.frame),
                "host_time": time_s(received.host_time)}),
            ),
            Event::ReadFailed(from, error) if Some(from) == current => {
                self.report(json!({"event": "read_failed", "error": error.to_string()}));
                return Next::ClosePort;
            }
            // From a port already closed.
            Event::Frame(..) | Event::ReadFailed(..) => {}
            Event::Stop => return Next::Stop,
        }
        Next::Continue
    }

    /// Open the port and start its reader, unless another owner runs; nothing is written.
    fn open(&mut self) -> Option<Port> {
        if let Some(owner) = running_process(&self.config.exclusive_with) {
            self.report(
                json!({"event": "port_busy", "owner": owner, "device": self.config.device}),
            );
            return None;
        }
        // Validated to fit a u32.
        let baud = u32::try_from(self.config.baud).unwrap_or(u32::MAX);
        let opened = SerialPort::open(&self.config.device, baud, Access::ReadWrite)
            .and_then(|port| Ok((port.stoppable_reader()?, port)));
        match opened {
            Ok(((reader, stopper), writer)) => {
                // Validated non-negative and small, so these never truncate.
                let decoder = Decoder::new(
                    self.config.max_payload_bytes as usize,
                    self.config.max_buffered_bytes as usize,
                );
                let chunk = self.config.read_chunk_bytes as usize;
                self.openings += 1;
                let opening = self.openings;
                let events = self.events_in.clone();
                let reader = std::thread::spawn(move || {
                    read_forever(reader, decoder, chunk, opening, events)
                });
                info!(device = %self.config.device, baud = self.config.baud, "port open");
                self.report(json!({"event": "port_open", "device": self.config.device}));
                Some(Port {
                    writer,
                    opening,
                    stopper,
                    reader: Some(reader),
                })
            }
            Err(error) => {
                warn!(%error, device = %self.config.device, "cannot open the port");
                self.report(json!({"event": "port_error", "device": self.config.device, "error": error.to_string()}));
                None
            }
        }
    }

    fn publish_reading(&self, reply: &Reply) {
        match reply {
            Reply::FanSpeeds(speeds) => self.sink.fan_speeds(*speeds),
            Reply::ExtLoadCurrent(current) => self.sink.ext_load_current(*current),
            _ => {}
        }
    }

    fn report(&self, event: Value) {
        self.sink
            .status(merge(event, json!({"time": wall_s()})).to_string());
    }
}

fn merge(mut base: Value, extra: Value) -> Value {
    if let (Some(base), Value::Object(extra)) = (base.as_object_mut(), extra) {
        base.extend(extra);
    }
    base
}

/// Decode the port onto `events` until a read fails or the port is stopped (the worker reopens on its next request).
fn read_forever(
    mut port: impl Read,
    mut decoder: Decoder,
    chunk_bytes: usize,
    opening: u64,
    events: Sender<Event>,
) {
    let mut chunk = vec![0u8; chunk_bytes.max(1)];
    loop {
        let count = match port.read(&mut chunk) {
            Ok(0) => {
                let _ = events.send(Event::ReadFailed(
                    opening,
                    io::Error::new(io::ErrorKind::UnexpectedEof, "port closed"),
                ));
                return;
            }
            Ok(count) => count.min(chunk.len()),
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error) => {
                let _ = events.send(Event::ReadFailed(opening, error));
                return;
            }
        };
        let host_time = SystemTime::now();
        decoder.push(&chunk[..count]);
        while let Some(frame) = decoder.next_frame() {
            if events
                .send(Event::Frame(opening, Received { frame, host_time }))
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

    fn config(device: &str) -> Config {
        Config {
            device: device.to_string(),
            baud: 115_200,
            read_chunk_bytes: 128,
            max_payload_bytes: 256,
            max_buffered_bytes: 4096,
            reply_timeout_ms: Func::ALL
                .into_iter()
                .map(|func| (func.name().to_string(), 200))
                .collect(),
            allow_lights: true,
            allow_fans: false,
            allow_power: false,
            startup_commands: Vec::new(),
            exclusive_with: Vec::new(),
        }
    }

    #[test]
    fn bad_configs_fail_validation_instead_of_panicking() {
        use validator::Validate;
        assert!(config("/dev/null").validate().is_ok());
        let mut too_big = config("/dev/null");
        too_big.startup_commands = vec![("fan_speed".into(), i64::from(i32::MAX) + 1)];
        assert!(
            too_big.validate().is_err(),
            "an int32 out of range was truncated before"
        );
        let mut no_timeout = config("/dev/null");
        no_timeout.reply_timeout_ms.remove("lidar_power");
        assert!(no_timeout.validate().is_err());
        let mut huge_baud = config("/dev/null");
        huge_baud.baud = i64::from(u32::MAX) + 1;
        assert!(huge_baud.validate().is_err());
    }

    struct Recorder(Sender<String>);

    impl Sink for Recorder {
        fn status(&self, event: String) {
            let _ = self.0.send(event);
        }
        fn fan_speeds(&self, _speeds: FanSpeeds) {}
        fn ext_load_current(&self, _current: u32) {}
    }

    /// A pseudo-terminal's controller end and the path of its device end.
    fn pty() -> (std::fs::File, String) {
        use std::os::fd::FromRawFd;
        unsafe {
            let controller = libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY);
            assert!(controller >= 0);
            assert_eq!(libc::grantpt(controller), 0);
            assert_eq!(libc::unlockpt(controller), 0);
            let name = std::ffi::CStr::from_ptr(libc::ptsname(controller))
                .to_str()
                .unwrap()
                .to_string();
            (std::fs::File::from_raw_fd(controller), name)
        }
    }

    /// Run a worker on a pty with `queued` already waiting, then stop it; its status events, in order.
    fn run_worker(queued: Vec<Event>, until: &str) -> Vec<String> {
        let (mut controller, path) = pty();
        let config = config(&path);
        // The MCU end takes what is written, or the write's tcdrain would wait forever.
        std::thread::spawn(move || {
            let mut sink = [0u8; 64];
            while matches!(controller.read(&mut sink), Ok(count) if count > 0) {}
        });
        let (events_in, events) = channel();
        for event in queued {
            events_in.send(event).unwrap();
        }
        let (status_tx, status_rx) = channel();
        let worker = Worker {
            timeouts: timeouts(&config).unwrap(),
            config,
            events_in: events_in.clone(),
            sink: Recorder(status_tx),
            openings: 0,
        };
        let thread = std::thread::spawn(move || worker.run(events));
        let mut seen = Vec::new();
        while !seen.iter().any(|event: &String| event.contains(until)) {
            seen.push(status_rx.recv_timeout(Duration::from_secs(5)).unwrap());
        }
        events_in.send(Event::Stop).unwrap();
        // The worker returns only once the port's reader thread has been stopped and joined.
        let (joined_tx, joined_rx) = channel();
        std::thread::spawn(move || joined_tx.send(thread.join().is_ok()));
        assert!(joined_rx.recv_timeout(Duration::from_secs(5)).unwrap());
        seen
    }

    fn host_led_job() -> Event {
        Event::Job(Job {
            command: Command::HostLed(true),
            value: 1,
        })
    }

    #[test]
    fn a_frame_queued_before_a_request_is_not_taken_as_its_reply() {
        let stale = Received {
            frame: Frame::host(9, Func::ExtLed as u8, vec![0, 0]),
            host_time: SystemTime::now(),
        };
        // Opening 1 is the port the worker opens first.
        let seen = run_worker(vec![host_led_job(), Event::Frame(1, stale)], "\"timeout\"");
        assert!(seen[0].contains("port_open"), "{seen:?}");
        assert!(seen[1].contains("unsolicited"), "{seen:?}");
        assert!(seen[2].contains("\"timeout\""), "{seen:?}");
    }

    #[test]
    fn events_from_a_closed_port_are_dropped() {
        let failed = Event::ReadFailed(99, io::Error::other("an old port"));
        let late = Event::Frame(
            99,
            Received {
                frame: Frame::host(9, Func::HostLed as u8, vec![0, 0]),
                host_time: SystemTime::now(),
            },
        );
        let seen = run_worker(vec![failed, host_led_job(), late], "\"timeout\"");
        assert_eq!(seen.len(), 2, "{seen:?}");
        assert!(seen[0].contains("port_open"), "{seen:?}");
    }
}
