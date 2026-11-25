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

//! Optional bounded native-process stage tracing. No IO on the publishing task.
use crate::trace_ring::TraceRing;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::cell::RefCell;
use std::collections::HashMap;
use std::env;
use std::fs;
use std::future::Future;
use std::io::Write;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, RecvTimeoutError, SyncSender};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

static STATE: OnceLock<Option<State>> = OnceLock::new();
static SEQUENCE: AtomicU64 = AtomicU64::new(0);
static DROPS: AtomicU64 = AtomicU64::new(0);
struct State {
    trace: bool,
    worker: Mutex<Option<std::thread::JoinHandle<()>>>,
    stopped: Arc<AtomicBool>,
    sender: SyncSender<Event>,
    origin: Instant,
    run: String,
    process: String,
    clock_domain: String,
    shared_monotonic: bool,
    sequences: Mutex<HashMap<String, u64>>,
}
struct Event {
    context: Option<TraceContext>,
    stage: &'static str,
    channel: String,
    id: u64,
    at: u64,
    end: bool,
}

fn state() -> Option<&'static State> {
    let state = STATE.get_or_init(initialize).as_ref()?;
    (!state.stopped.load(Ordering::Relaxed)).then_some(state)
}

fn initialize() -> Option<State> {
    let mode = env::var("DIMOS_DIAGNOSTICS").ok()?;
    if mode != "trace" && mode != "metrics" {
        return None;
    }
    let root = PathBuf::from(env::var("DIMOS_DIAGNOSTICS_DIR").ok()?);
    let run = env::var("DIMOS_DIAGNOSTICS_RUN").ok()?;
    if run.is_empty()
        || run.len() > 128
        || !run
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
    {
        return None;
    }
    let reservation = Reservation::reserve(&root, 16 * 1024 * 1024).ok()?;
    if reservation.bytes < 4 * 1024 * 1024 {
        return None;
    }
    let process = format!("native-{}-{}", std::process::id(), wall_ns());
    let path = root.join(&run).join(&process);
    let shared_domain = shared_clock_domain();
    let shared_monotonic = shared_domain.is_some();
    let clock_domain = shared_domain.unwrap_or_else(|| format!("{process}/instant"));
    let host = if shared_monotonic {
        clock_domain.split('/').next().unwrap_or("unknown")
    } else {
        "unknown"
    };
    let origin = Instant::now();
    let mono = if shared_monotonic {
        system_monotonic_ns().unwrap_or(0)
    } else {
        0
    };
    let wall = wall_ns();
    let manifest = serde_json::json!({"kind":"manifest", "schema":"dimos.incident/v1",
        "wall_ns":wall as u64,"mono_ns":mono,"mode":mode, "process":process, "pid":std::process::id(), "run_id":run, "host":host,
        "clock_domain":clock_domain, "coverage":"native queues, handlers, encode, transport; custom cached inputs require explicit context"});
    let (sender, receiver) = sync_channel::<Event>(128);
    let stopped = Arc::new(AtomicBool::new(false));
    let worker_stopped = stopped.clone();
    let worker = std::thread::Builder::new()
        .name("dimos-diagnostics".into())
        .spawn(move || {
            let ring_limit = (reservation.bytes * 3 / 4).min(4 * 1024 * 1024);
            collect(receiver, path, root, manifest, worker_stopped, ring_limit);
            drop(reservation);
        })
        .ok()?;
    Some(State {
        trace: mode == "trace",
        worker: Mutex::new(Some(worker)),
        stopped,
        sender,
        origin,
        run,
        process,
        clock_domain,
        shared_monotonic,
        sequences: Mutex::new(HashMap::new()),
    })
}

// Linux Python and Rust can share CLOCK_MONOTONIC within one host boot. Other
// platforms retain a private domain until a measured mapping is supplied.
#[cfg(target_os = "linux")]
fn shared_clock_domain() -> Option<String> {
    let boot = fs::read_to_string("/proc/sys/kernel/random/boot_id").ok()?;
    let host = fs::read_to_string("/proc/sys/kernel/hostname").ok()?;
    system_monotonic_ns()?;
    Some(format!("{}/{}/monotonic", host.trim(), boot.trim()))
}

#[cfg(target_os = "macos")]
fn shared_clock_domain() -> Option<String> {
    let boot = std::process::Command::new("/usr/sbin/sysctl")
        .args(["-n", "kern.bootsessionuuid"])
        .output()
        .ok()?;
    let host = std::process::Command::new("/bin/hostname").output().ok()?;
    if !boot.status.success() || !host.status.success() {
        return None;
    }
    system_monotonic_ns()?;
    Some(format!(
        "{}/{}/monotonic",
        String::from_utf8(host.stdout).ok()?.trim(),
        String::from_utf8(boot.stdout).ok()?.trim()
    ))
}
#[cfg(not(any(target_os = "linux", target_os = "macos")))]
fn shared_clock_domain() -> Option<String> {
    None
}

#[cfg(any(target_os = "linux", target_os = "macos"))]
fn system_monotonic_ns() -> Option<u64> {
    let mut value = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    #[cfg(target_os = "linux")]
    let clock = libc::CLOCK_MONOTONIC;
    #[cfg(target_os = "macos")]
    let clock = libc::CLOCK_UPTIME_RAW; // Python time.monotonic uses mach_absolute_time, excluding suspend.
                                        // SAFETY: value is initialized, writable and correctly sized for timespec.
    if unsafe { libc::clock_gettime(clock, &mut value) } != 0 {
        return None;
    }
    Some(value.tv_sec as u64 * 1_000_000_000 + value.tv_nsec as u64)
}

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
fn system_monotonic_ns() -> Option<u64> {
    None
}

impl State {
    fn now_ns(&self) -> Option<u64> {
        if self.shared_monotonic {
            let value = system_monotonic_ns();
            if value.is_none() {
                DROPS.fetch_add(1, Ordering::Relaxed);
            }
            value
        } else {
            Some(self.origin.elapsed().as_nanos() as u64)
        }
    }
}

fn wall_ns() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos()
}

fn collect(
    receiver: Receiver<Event>,
    path: PathBuf,
    root: PathBuf,
    manifest: serde_json::Value,
    stopped: Arc<AtomicBool>,
    ring_limit: usize,
) {
    if fs::create_dir_all(&path).is_err() {
        stopped.store(true, Ordering::Relaxed);
        return;
    }
    let mut ring = TraceRing::new(ring_limit);
    let started = Instant::now();
    let mut errors = 0u64;
    let mut last_save = Instant::now();
    let mut last_status = Instant::now() - Duration::from_secs(1);
    let mut pending: Option<(Instant, String)> = None;
    let mut active = HashMap::new();
    let mut stages = serde_json::Map::new();
    let trace = manifest["mode"].as_str().unwrap_or("trace") == "trace";
    loop {
        let mut stopping = false;
        let mut flush = false;
        let now = Instant::now();
        match receiver.recv_timeout(Duration::from_millis(100)) {
            Ok(event) if event.stage == "collector.stop" => stopping = true,
            Ok(event) => {
                if event.end {
                    if let Some(start) = active.remove(&event.id) {
                        record_duration(&mut stages, event.stage, event.at.saturating_sub(start));
                        if event.at.saturating_sub(start) > 100_000_000 && pending.is_none() {
                            pending = Some((
                                now + Duration::from_secs(30),
                                format!("slow:{}", event.stage),
                            ));
                        }
                    }
                } else if active.len() < 4096 {
                    active.insert(event.id, event.at);
                }
                let mut context = serde_json::to_value(event.context).unwrap_or_default();
                if let Some(object) = context.as_object_mut() {
                    object.remove("schema");
                }
                let value = serde_json::json!({"schema":"dimos.trace/v1",
                    "event":format!("{}.{}", event.stage, if event.end {"end"} else {"start"}),
                    "channel":event.channel, "process":manifest["process"],
                    "clock_domain":manifest["clock_domain"], "span_id":event.id.to_string(),
                    "mono_ns":event.at, "context":context});
                if let Ok(mut row) = serde_json::to_vec(&value) {
                    row.push(b'\n');
                    if trace && row.len() <= 65536 {
                        if ring.push(&row, now).is_err() {
                            errors += 1;
                        }
                    } else if trace {
                        DROPS.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
            Err(RecvTimeoutError::Disconnected) => stopping = true,
            Err(RecvTimeoutError::Timeout) => {}
        }
        let cutoff = pending
            .as_ref()
            .map_or(now - Duration::from_secs(60), |(until, _)| {
                *until - Duration::from_secs(90)
            });
        if ring.trim(cutoff).is_err() {
            errors += 1;
        }
        if last_status.elapsed() >= Duration::from_secs(1) {
            let control = path.join("control.json");
            if let Ok(bytes) = fs::read(&control) {
                if bytes.len() <= 4096 {
                    if let Ok(value) = serde_json::from_slice::<serde_json::Value>(&bytes) {
                        stopping |= value["action"] == "stop";
                        flush = value["action"] == "flush";
                        if value["action"] == "mark" && pending.is_none() {
                            pending = Some((
                                now + Duration::from_secs(30),
                                value["reason"]
                                    .as_str()
                                    .unwrap_or("manual")
                                    .chars()
                                    .take(512)
                                    .collect(),
                            ));
                        }
                    }
                }
                let _ = fs::remove_file(control);
            }
        }
        let due = pending.as_ref().is_some_and(|(until, _)| now >= *until);
        let health = serde_json::json!({"observer_drops":DROPS.load(Ordering::Relaxed),"ring_evictions":ring.evictions,
            "io_errors":errors,"ring_bytes":ring.size(),"raw_ring_bytes":ring.raw_bytes,"ring_limit_bytes":ring_limit,"requested_pre_seconds":60,"requested_post_seconds":30,
            "stages":stages,"mode":if trace {"trace"}else{"metrics"},"retained_seconds":now.saturating_duration_since(started.max(cutoff).max(ring.capacity_lost_until.unwrap_or(started))).as_secs_f64(),"capture_window_complete":trace && started<=cutoff && ring.capacity_lost_until.is_none_or(|at|at<=cutoff) && (pending.is_none() || due),"capacity_truncated":ring.capacity_lost_until.is_some_and(|at|at>cutoff)});
        if stopping || flush || due || last_save.elapsed() >= Duration::from_secs(5) {
            let mut header = manifest.clone();
            if manifest["clock_domain"]
                .as_str()
                .is_some_and(|domain| domain.ends_with("/monotonic"))
            {
                if let Some(stamp) = system_monotonic_ns() {
                    header["captured_ns"] = stamp.into();
                }
            }
            header["health"] = health.clone();
            header["reason"] = pending
                .as_ref()
                .map_or("checkpoint", |(_, reason)| reason.as_str())
                .into();
            let destination = if due {
                path.join(format!("incident-native-{}.jsonl", wall_ns()))
            } else {
                path.join("incident-native.jsonl")
            };
            if persist_capture(&root, &destination, &header, &ring).is_err() {
                errors += 1;
            }
            last_save = Instant::now();
            if due {
                pending = None;
            }
        }
        if stopping || last_status.elapsed() >= Duration::from_secs(1) {
            let mut status = manifest.clone();
            status["state"] = if stopping { "stopped" } else { "collecting" }.into();
            status["mode"] = manifest["mode"].clone();
            status["at_ns"] = (wall_ns() as u64).into();
            if let (Some(target), Some(fields)) = (status.as_object_mut(), health.as_object()) {
                target.extend(fields.clone());
            }
            status["io_errors"] = errors.into();
            let temporary = path.join("status.tmp");
            if fs::write(&temporary, status.to_string())
                .and_then(|_| fs::rename(temporary, path.join("status.json")))
                .is_err()
            {
                errors += 1;
            }
            last_status = Instant::now();
        }
        if stopping {
            stopped.store(true, Ordering::Relaxed);
            break;
        }
    }
}

/// Measures a native stage on its own monotonic clock; never claims packet delivery.
pub struct Span {
    context: Option<TraceContext>,
    state: Option<&'static State>,
    stage: &'static str,
    channel: String,
    id: u64,
}
impl Span {
    pub fn new(stage: &'static str, channel: &str) -> Self {
        Self::with_context(stage, channel, current_context())
    }
    pub fn with_context(stage: &'static str, channel: &str, context: Option<TraceContext>) -> Self {
        let state = state();
        let id = if state.is_some() {
            SEQUENCE.fetch_add(1, Ordering::Relaxed)
        } else {
            0
        };
        let channel = if state.is_some() {
            channel.chars().take(256).collect()
        } else {
            String::new()
        };
        let span = Self {
            state,
            stage,
            channel,
            id,
            context,
        };
        span.emit(false);
        span
    }
    fn emit(&self, end: bool) {
        if let Some(state) = self.state {
            let Some(at) = state.now_ns() else {
                return;
            };
            let event = Event {
                context: if state.trace {
                    self.context.clone()
                } else {
                    None
                },
                stage: self.stage,
                channel: self.channel.clone(),
                id: self.id,
                at,
                end,
            };
            if state.sender.try_send(event).is_err() {
                DROPS.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}
impl Drop for Span {
    fn drop(&mut self) {
        self.emit(true);
    }
}

/// Flush and join the observer at process shutdown, outside control callbacks.
pub fn shutdown() {
    if let Some(state) = STATE.get().and_then(Option::as_ref) {
        if state.stopped.swap(true, Ordering::Relaxed) {
            return;
        }
        let _ = state.sender.send(Event {
            context: None,
            stage: "collector.stop",
            channel: String::new(),
            id: 0,
            at: 0,
            end: false,
        });
        if let Ok(mut worker) = state.worker.lock() {
            if let Some(worker) = worker.take() {
                let _ = worker.join();
            }
        }
    }
}

/// Wire-compatible Zenoh attachment. Origin is transport egress, not camera exposure.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TraceContext {
    pub schema: String,
    pub run_id: String,
    pub publisher: String,
    pub stream: String,
    pub sequence: u64,
    pub origin_ns: u64,
    pub clock_domain: String,
    pub parents: Vec<String>,
    pub lineage: String,
}

pub fn publication(stream: &str) -> Option<TraceContext> {
    let state = state()?;
    if !state.trace {
        return None;
    }
    let mut sequences = state.sequences.lock().ok()?;
    if sequences.len() >= 4096 && !sequences.contains_key(stream) {
        return None;
    }
    let sequence = sequences.entry(stream.to_string()).or_default();
    *sequence += 1;
    Some(TraceContext {
        schema: "dimos.trace/v1".into(),
        run_id: state.run.clone(),
        publisher: state.process.clone(),
        stream: stream.into(),
        sequence: *sequence,
        origin_ns: state.now_ns()?,
        clock_domain: state.clock_domain.clone(),
        parents: current_context()
            .iter()
            .map(TraceContext::sample_id)
            .collect(),
        lineage: current_context().map_or_else(|| "partial".into(), |ctx| ctx.lineage),
    })
}

impl TraceContext {
    /// Hash the same ASCII JSON identity tuple as the Python SDK.
    pub fn sample_id(&self) -> String {
        let value = serde_json::json!([self.run_id, self.publisher, self.stream, self.sequence]);
        let text = serde_json::to_string(&value).expect("identity is serializable");
        let mut ascii = String::new();
        for ch in text.chars() {
            if ch.is_ascii() {
                ascii.push(ch);
            } else {
                for word in ch.encode_utf16(&mut [0; 2]) {
                    ascii.push_str(&format!("\\u{word:04x}"));
                }
            }
        }
        format!("{:x}", Sha256::digest(ascii.as_bytes()))
    }
    pub fn attachment(&self) -> Option<Vec<u8>> {
        let mut data = b"DIMOS_TRACE_V1\0".to_vec();
        data.extend(serde_json::to_vec(self).ok()?);
        (data.len() <= 16399).then_some(data)
    }
    pub fn from_attachment(data: &[u8]) -> Option<Self> {
        if data.len() > 16399 {
            return None;
        }
        let value: Self = serde_json::from_slice(data.strip_prefix(b"DIMOS_TRACE_V1\0")?).ok()?;
        (value.schema == "dimos.trace/v1"
            && value.parents.len() <= 32
            && ["complete", "partial", "unknown"].contains(&value.lineage.as_str()))
        .then_some(value)
    }
}

thread_local! { static CALLBACK_CONTEXT: RefCell<Option<TraceContext>> = const { RefCell::new(None) }; }
tokio::task_local! { static TASK_CONTEXT: Option<TraceContext>; }

/// The actual input associated with this callback/task. Retain it with cached inputs.
pub fn current_context() -> Option<TraceContext> {
    TASK_CONTEXT
        .try_with(Clone::clone)
        .unwrap_or_else(|_| CALLBACK_CONTEXT.with(|value| value.borrow().clone()))
}

/// Scope context on an async task; it follows task migration and is restored on cancellation.
pub async fn scope<F: Future>(context: Option<TraceContext>, future: F) -> F::Output {
    TASK_CONTEXT.scope(context, future).await
}

pub fn with_context<R>(context: Option<TraceContext>, callback: impl FnOnce() -> R) -> R {
    struct Reset(Option<TraceContext>);
    impl Drop for Reset {
        fn drop(&mut self) {
            CALLBACK_CONTEXT.with(|value| {
                value.replace(self.0.take());
            });
        }
    }
    let _reset = Reset(CALLBACK_CONTEXT.with(|value| value.replace(context)));
    callback()
}

/// Metadata travels beside the original value across SDK queues, never in robot bytes.
pub struct Queued<T> {
    pub value: T,
    pub context: Option<TraceContext>,
    waiting: Option<Span>,
}
impl<T> Queued<T> {
    pub fn capture(value: T, channel: &str, context: Option<TraceContext>) -> Self {
        let waiting = Some(Span::with_context(
            "native.queue_wait",
            channel,
            context.clone(),
        ));
        Self {
            value,
            context,
            waiting,
        }
    }
    pub fn received(mut self) -> Self {
        self.waiting.take();
        self
    }
    pub async fn handle<F: Future>(self, handler: impl FnOnce(T) -> F) -> F::Output {
        let sample = self.received();
        scope(sample.context, async move {
            let _span = Span::new("native.handler", "input");
            handler(sample.value).await
        })
        .await
    }
}

#[cfg(test)]
mod tests {
    use super::TraceContext;
    #[test]
    fn attachment_contract_round_trip_and_legacy_rejection() {
        let context = TraceContext {
            schema: "dimos.trace/v1".into(),
            run_id: "run".into(),
            publisher: "native".into(),
            stream: "camera".into(),
            sequence: 7,
            origin_ns: 42,
            clock_domain: "native/instant".into(),
            parents: vec![],
            lineage: "partial".into(),
        };
        let bytes = context.attachment().unwrap();
        let decoded = TraceContext::from_attachment(&bytes).unwrap();
        assert_eq!(decoded.sequence, 7);
        assert_eq!(decoded.origin_ns, 42);
        assert!(TraceContext::from_attachment(b"old payload").is_none());
        assert!(TraceContext::from_attachment(&vec![0; 16400]).is_none());
    }

    #[test]
    fn collector_flushes_when_its_sender_closes() {
        struct Directory(std::path::PathBuf);
        impl Drop for Directory {
            fn drop(&mut self) {
                let _ = std::fs::remove_dir_all(&self.0);
            }
        }
        let directory = Directory(std::env::temp_dir().join(format!(
            "dimos-diagnostics-test-{}-{}",
            std::process::id(),
            super::wall_ns()
        )));
        let (sender, receiver) = std::sync::mpsc::sync_channel(8);
        for (at, end) in [(10, false), (20, true)] {
            sender
                .send(super::Event {
                    context: None,
                    stage: "work",
                    channel: "hand".into(),
                    id: 1,
                    at,
                    end,
                })
                .unwrap();
        }
        drop(sender);
        let stopped = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        super::collect(
            receiver,
            directory.0.clone(),
            directory.0.clone(),
            serde_json::json!({"kind":"manifest","process":"test","clock_domain":"test"}),
            stopped.clone(),
            4 * 1024 * 1024,
        );
        assert!(stopped.load(std::sync::atomic::Ordering::Relaxed));
        let rows: Vec<serde_json::Value> =
            std::fs::read_to_string(directory.0.join("incident-native.jsonl"))
                .unwrap()
                .lines()
                .map(|row| serde_json::from_str(row).unwrap())
                .collect();
        assert_eq!(rows.len(), 3);
        assert_eq!(rows[1]["event"], "work.start");
        assert_eq!(rows[2]["mono_ns"], 20);
        let status: serde_json::Value = serde_json::from_str(
            &std::fs::read_to_string(directory.0.join("status.json")).unwrap(),
        )
        .unwrap();
        assert_eq!(status["state"], "stopped");
    }
    #[tokio::test]
    async fn queued_handler_keeps_input_and_restores_task_context() {
        let input = TraceContext {
            schema: "dimos.trace/v1".into(),
            run_id: "run".into(),
            publisher: "native".into(),
            stream: "摄像📷".into(),
            sequence: 7,
            origin_ns: 42,
            clock_domain: "clock".into(),
            parents: vec![],
            lineage: "complete".into(),
        };
        assert_eq!(
            input.sample_id(),
            "f2f5937e482b16d1e02b3550b98efb46e5645ee4980abb98757a7d8439f866b9"
        );
        let sample = super::Queued::capture(vec![1, 2, 3], "input", Some(input.clone()));
        let id = sample
            .handle(|bytes| async move {
                tokio::task::yield_now().await;
                assert_eq!(bytes, vec![1, 2, 3]);
                super::current_context().unwrap().sample_id()
            })
            .await;
        assert_eq!(id, input.sample_id());
        assert!(super::current_context().is_none());
    }
}

// Reservations use the same host-level file/lock contract as the Python collector.
struct Reservation {
    root: PathBuf,
    key: String,
    bytes: usize,
}
impl Reservation {
    fn reserve(root: &std::path::Path, requested: usize) -> std::io::Result<Self> {
        use std::os::fd::AsRawFd;
        fs::create_dir_all(root)?;
        let lock = fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(root.join(".memory.lock"))?;
        // SAFETY: lock owns a valid file descriptor until the operation completes.
        if unsafe { libc::flock(lock.as_raw_fd(), libc::LOCK_EX) } != 0 {
            return Err(std::io::Error::last_os_error());
        }
        let path = root.join(".memory.json");
        let mut entries: serde_json::Map<String, serde_json::Value> = match fs::read(&path) {
            Ok(bytes) => serde_json::from_slice(&bytes).map_err(std::io::Error::other)?,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => Default::default(),
            Err(error) => return Err(error),
        };
        entries.retain(|_, value| {
            let Some(pid) = value["pid"].as_i64() else {
                return true;
            };
            // SAFETY: signal 0 only tests existence/permissions; sends no signal.
            unsafe {
                libc::kill(pid as libc::pid_t, 0) == 0
                    || std::io::Error::last_os_error().raw_os_error() != Some(libc::ESRCH)
            }
        });
        let used: usize = entries
            .values()
            .map(|v| v["bytes"].as_u64().unwrap_or(256 * 1024 * 1024) as usize)
            .sum();
        let process_used: usize = entries
            .values()
            .filter(|v| v["pid"].as_u64() == Some(std::process::id() as u64))
            .map(|v| v["bytes"].as_u64().unwrap_or(64 * 1024 * 1024) as usize)
            .sum();
        let bytes = requested
            .min((256usize * 1024 * 1024).saturating_sub(used))
            .min((64usize * 1024 * 1024).saturating_sub(process_used));
        let key = format!(
            "native-{}-{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        );
        entries.insert(
            key.clone(),
            serde_json::json!({"pid":std::process::id(),"bytes":bytes}),
        );
        fs::write(root.join(".memory.tmp"), serde_json::to_vec(&entries)?)?;
        fs::rename(root.join(".memory.tmp"), path)?;
        Ok(Self {
            root: root.to_path_buf(),
            key,
            bytes,
        })
    }
}
impl Drop for Reservation {
    fn drop(&mut self) {
        use std::os::fd::AsRawFd;
        let Ok(lock) = fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(self.root.join(".memory.lock"))
        else {
            return;
        };
        // SAFETY: lock owns the fd and flock does not transfer ownership.
        if unsafe { libc::flock(lock.as_raw_fd(), libc::LOCK_EX) } != 0 {
            return;
        }
        let path = self.root.join(".memory.json");
        if let Ok(raw) = fs::read(&path) {
            if let Ok(mut entries) =
                serde_json::from_slice::<serde_json::Map<String, serde_json::Value>>(&raw)
            {
                entries.remove(&self.key);
                if fs::write(
                    self.root.join(".memory.tmp"),
                    serde_json::to_vec(&entries).unwrap_or_default(),
                )
                .is_ok()
                {
                    let _ = fs::rename(self.root.join(".memory.tmp"), path);
                }
            }
        }
    }
}

// All collectors sharing a diagnostics root use this flock and 1 GiB disk budget.
fn persist_capture(
    root: &std::path::Path,
    destination: &std::path::Path,
    header: &serde_json::Value,
    ring: &TraceRing,
) -> std::io::Result<()> {
    use std::os::fd::AsRawFd;
    fs::create_dir_all(root)?;
    let lock = fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(root.join(".retention.lock"))?;
    // SAFETY: lock owns this valid fd; flock does not transfer ownership.
    if unsafe { libc::flock(lock.as_raw_fd(), libc::LOCK_EX) } != 0 {
        return Err(std::io::Error::last_os_error());
    }
    let header = serde_json::to_vec(header)?;
    let incoming = header.len() as u64 + 1 + ring.raw_bytes as u64;
    const BUDGET: u64 = 1024 * 1024 * 1024;
    if incoming > BUDGET {
        return Err(std::io::Error::other("capture exceeds host disk budget"));
    }
    let mut files = Vec::new();
    let mut directories = vec![root.to_path_buf()];
    let mut total = 0u64;
    while let Some(directory) = directories.pop() {
        for entry in fs::read_dir(directory)? {
            let entry = entry?;
            let kind = entry.file_type()?;
            if kind.is_dir() {
                directories.push(entry.path());
            } else if kind.is_file()
                && entry.file_name().to_string_lossy().starts_with("incident-")
                && entry.path().extension().is_some_and(|s| s == "jsonl")
            {
                let meta = entry.metadata()?;
                total += meta.len();
                files.push((meta.modified()?, entry.path(), meta.len()));
            }
            if files.len() + directories.len() > 32768 {
                return Err(std::io::Error::other(
                    "capture directory inventory exceeds bound",
                ));
            }
        }
    }
    files.sort_by_key(|(at, _, _)| *at);
    for (_, path, size) in files {
        if total + incoming <= BUDGET {
            break;
        }
        fs::remove_file(path)?;
        total = total.saturating_sub(size);
    }
    let temporary = destination.with_extension("tmp");
    let mut file = fs::File::create(&temporary)?;
    file.write_all(&header)?;
    file.write_all(b"\n")?;
    ring.write_to(&mut file)?;
    file.sync_data()?;
    fs::rename(temporary, destination)
}

fn record_duration(
    stages: &mut serde_json::Map<String, serde_json::Value>,
    stage: &str,
    duration: u64,
) {
    if stages.len() >= 512 && !stages.contains_key(stage) {
        return;
    }
    let metric = stages.entry(stage.to_string()).or_insert_with(
        || serde_json::json!({"n":0,"sum_ns":0,"max_ns":0,"buckets_log2_ns":vec![0u64;64]}),
    );
    metric["n"] = (metric["n"].as_u64().unwrap_or(0) + 1).into();
    metric["sum_ns"] = metric["sum_ns"]
        .as_u64()
        .unwrap_or(0)
        .saturating_add(duration)
        .into();
    metric["max_ns"] = metric["max_ns"].as_u64().unwrap_or(0).max(duration).into();
    let bucket = (64 - duration.leading_zeros()).min(63) as usize;
    metric["buckets_log2_ns"][bucket] =
        (metric["buckets_log2_ns"][bucket].as_u64().unwrap_or(0) + 1).into();
}
