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

//! Runtime for a baked host binary: several native modules in one process,
//! sharing one transport session, each on its own thread.
//!
//! `dimos bake` generates a `main.rs` that hands a static [`HostSpec`] to
//! [`host_main`]. Everything a host needs beyond that lives here, so the
//! generated crate stays a table of module entries and three `include_str!`s.

use std::collections::HashMap;
use std::future::Future;
use std::io;
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;

use serde_json::{json, Map, Value};
use tokio::sync::{mpsc, watch};
use tracing::{error, info, warn};

use crate::lcm::LcmTransport;
use crate::module::{
    init_tracing, log_wiring, parse_config_value, read_stdin_config, run_module_core,
    validate_config, Module,
};
use crate::transport::{SharedTransport, Transport};
use crate::zenoh::ZenohTransport;

/// How long the host waits for the remaining modules to unwind once one has
/// exited or ctrl_c arrived.
const SHUTDOWN_GRACE: Duration = Duration::from_secs(5);

/// Linux caps thread names at 15 characters plus the NUL.
const MAX_THREAD_NAME: usize = 15;

type ModuleFuture = Pin<Box<dyn Future<Output = io::Result<()>> + Send>>;
type RunFn = Box<dyn FnOnce(Arc<SharedTransport>, watch::Receiver<bool>) -> ModuleFuture + Send>;

/// One module's config, deserialized and validated, with a closure that runs it
/// once the transport is open. Produced before anything is spawned so a bad
/// config kills the host instead of half-starting it.
pub struct Prepared {
    topics: HashMap<String, String>,
    run: RunFn,
}

impl std::fmt::Debug for Prepared {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Prepared")
            .field("topics", &self.topics)
            .finish_non_exhaustive()
    }
}

/// A module baked into the host: its id, how much of a runtime it wants, and
/// the monomorphized entry point that parses its config and runs it.
pub struct ModuleEntry {
    pub name: &'static str,
    /// Worker threads in the module's own runtime. More than one is needed by a
    /// module that spawns tasks calling `block_in_place` alongside its dispatch
    /// loop. Always a multi-thread runtime: zenoh refuses to run on a
    /// current_thread scheduler.
    pub threads: usize,
    /// `setpriority` niceness for the module's threads. `None` leaves it alone.
    pub nice: Option<i32>,
    prepare: fn(&Value) -> io::Result<Prepared>,
}

impl ModuleEntry {
    pub const fn new<M: Module>(name: &'static str) -> Self {
        Self {
            name,
            threads: 1,
            nice: None,
            prepare: prepare_module::<M>,
        }
    }

    pub const fn threads(mut self, threads: usize) -> Self {
        self.threads = threads;
        self
    }

    pub const fn nice(mut self, nice: i32) -> Self {
        self.nice = Some(nice);
        self
    }
}

fn prepare_module<M: Module>(section: &Value) -> io::Result<Prepared> {
    let (topics, config) = parse_config_value::<M::Config>(section)?;
    validate_config(&config)?;
    let module_topics = topics.clone();
    Ok(Prepared {
        topics,
        run: Box::new(move |transport, shutdown| {
            Box::pin(run_module_core::<M, SharedTransport>(
                transport,
                module_topics,
                config,
                shutdown,
            ))
        }),
    })
}

/// Everything `dimos bake` bakes in: the module table plus the JSON blobs that
/// make the binary runnable with no arguments and inspectable without one.
pub struct HostSpec {
    pub name: &'static str,
    pub modules: &'static [ModuleEntry],
    /// `{"<module>": {"<port>": "<topic>"}}` — the wiring the graph was drawn
    /// from. Per-module `topics` on stdin override individual ports.
    pub default_topics: &'static str,
    /// Topics whose publishers stay inside this process. Replaced wholesale by
    /// a top-level `suppress` array on stdin.
    pub default_suppress: &'static [&'static str],
    /// `{"<topic>": {"reliability": ..., "congestion_control": ...}}`.
    pub default_qos: &'static str,
    /// The rendered connection graph, printed by `<host> graph`.
    pub graph_json: &'static str,
}

/// Entry point of a baked host binary. Never returns.
pub fn host_main(spec: &HostSpec) -> ! {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        None => run_host(spec),
        Some("graph") => {
            print_graph(spec, args.iter().any(|a| a == "--json"));
            std::process::exit(0)
        }
        Some(other) => {
            eprintln!(
                "{}: unknown argument {other:?}; usage: {0} [graph [--json]]",
                spec.name
            );
            std::process::exit(2)
        }
    }
}

fn print_graph(spec: &HostSpec, raw: bool) {
    if raw {
        println!("{}", spec.graph_json);
        return;
    }
    match serde_json::from_str::<Value>(spec.graph_json)
        .and_then(|v| serde_json::to_string_pretty(&v))
    {
        Ok(pretty) => println!("{pretty}"),
        Err(_) => println!("{}", spec.graph_json),
    }
}

fn run_host(spec: &HostSpec) -> ! {
    init_tracing();
    match run_host_fallible(spec) {
        Ok(()) => std::process::exit(0),
        Err(e) => {
            error!(host = spec.name, "{e}");
            std::process::exit(1)
        }
    }
}

fn invalid(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.into())
}

fn parse_baked(label: &str, src: &str) -> io::Result<Value> {
    serde_json::from_str(src).map_err(|e| invalid(format!("baked {label} is not valid JSON: {e}")))
}

fn object<'a>(value: &'a Value, label: &str) -> io::Result<&'a Map<String, Value>> {
    value
        .as_object()
        .ok_or_else(|| invalid(format!("`{label}` must be an object")))
}

/// Per-module topic map: the baked wiring with the stdin overrides applied.
fn merge_topics(defaults: Option<&Value>, overrides: Option<&Value>) -> Value {
    let mut merged = defaults
        .and_then(Value::as_object)
        .cloned()
        .unwrap_or_default();
    if let Some(over) = overrides.and_then(Value::as_object) {
        for (port, topic) in over {
            merged.insert(port.clone(), topic.clone());
        }
    }
    Value::Object(merged)
}

/// The `qos` object handed to the transport: baked defaults, then stdin
/// per-topic overrides, then `locality: session_local` on every suppressed
/// topic so its publisher never leaves this process.
fn merge_qos(defaults: &Value, overrides: Option<&Value>, suppress: &[String]) -> Value {
    let mut merged = defaults.as_object().cloned().unwrap_or_default();
    if let Some(over) = overrides.and_then(Value::as_object) {
        for (topic, entry) in over {
            merged.insert(topic.clone(), entry.clone());
        }
    }
    for topic in suppress {
        let entry = merged.entry(topic.clone()).or_insert_with(|| json!({}));
        if !entry.is_object() {
            *entry = json!({});
        }
        entry
            .as_object_mut()
            .expect("entry was just coerced to an object")
            .insert("locality".to_string(), json!("session_local"));
    }
    Value::Object(merged)
}

/// The effective suppression list: the baked one unless stdin replaces it.
fn resolve_suppress(spec: &HostSpec, stdin: &Value) -> io::Result<Vec<String>> {
    let Some(value) = stdin.get("suppress") else {
        return Ok(spec
            .default_suppress
            .iter()
            .map(|s| s.to_string())
            .collect());
    };
    let list = value
        .as_array()
        .ok_or_else(|| invalid("`suppress` must be an array of topic strings"))?;
    list.iter()
        .map(|v| {
            v.as_str()
                .map(str::to_string)
                .ok_or_else(|| invalid("`suppress` entries must be topic strings"))
        })
        .collect()
}

/// Deserialize and validate every module's config before anything runs, so a
/// typo in one module's section can't leave the others half-started.
fn prepare_all(spec: &HostSpec, stdin: &Value) -> io::Result<Vec<Prepared>> {
    let defaults = parse_baked("default_topics", spec.default_topics)?;
    let sections = object(
        stdin
            .get("modules")
            .ok_or_else(|| invalid("missing `modules` object in stdin JSON"))?,
        "modules",
    )?;

    for id in sections.keys() {
        if !spec.modules.iter().any(|m| m.name == id) {
            let known: Vec<&str> = spec.modules.iter().map(|m| m.name).collect();
            return Err(invalid(format!(
                "stdin configures unknown module `{id}`; this host bakes {known:?}"
            )));
        }
    }

    let mut prepared = Vec::with_capacity(spec.modules.len());
    for entry in spec.modules {
        let section = sections.get(entry.name).ok_or_else(|| {
            invalid(format!(
                "missing `modules.{}` section in stdin JSON",
                entry.name
            ))
        })?;
        let merged = json!({
            "topics": merge_topics(defaults.get(entry.name), section.get("topics")),
            "config": section.get("config").cloned().unwrap_or(Value::Null),
        });
        let one = (entry.prepare)(&merged)
            .map_err(|e| invalid(format!("module `{}`: {e}", entry.name)))?;
        log_wiring(
            entry.name,
            &one.topics,
            section.get("config").unwrap_or(&Value::Null),
        );
        prepared.push(one);
    }
    Ok(prepared)
}

async fn open_transport() -> io::Result<SharedTransport> {
    match std::env::var("DIMOS_TRANSPORT").as_deref() {
        Ok("lcm") => Ok(SharedTransport::new(LcmTransport::new().await?)),
        Ok("zenoh") => Ok(SharedTransport::new(ZenohTransport::new().await?)),
        other => Err(invalid(format!(
            "DIMOS_TRANSPORT must be 'lcm' or 'zenoh', got {other:?}"
        ))),
    }
}

fn thread_name(module: &str) -> String {
    let mut name = format!("dm-{module}");
    name.truncate(MAX_THREAD_NAME);
    name
}

#[cfg(unix)]
fn apply_nice(module: &str, nice: i32) {
    // PRIO_PROCESS with who=0 is "the calling thread" on Linux, which is what
    // we want: each module's priority, not the whole host's.
    let rc = unsafe { libc::setpriority(libc::PRIO_PROCESS, 0, nice) };
    if rc != 0 {
        warn!(module, nice, error = %io::Error::last_os_error(), "could not set niceness");
    }
}

#[cfg(not(unix))]
fn apply_nice(module: &str, nice: i32) {
    warn!(module, nice, "niceness is not supported on this platform");
}

/// The module's own runtime. Multi-thread even for a single worker: zenoh
/// panics on a current_thread scheduler, and every module shares its session.
fn build_runtime(entry: &ModuleEntry) -> io::Result<tokio::runtime::Runtime> {
    let name = entry.name;
    let nice = entry.nice;
    tokio::runtime::Builder::new_multi_thread()
        .worker_threads(entry.threads.max(1))
        .thread_name(thread_name(entry.name))
        .on_thread_start(move || {
            if let Some(nice) = nice {
                apply_nice(name, nice);
            }
        })
        .enable_all()
        .build()
}

/// What a module reports back when it stops.
enum Outcome {
    Finished(io::Result<()>),
    Panicked,
}

fn run_host_fallible(spec: &HostSpec) -> io::Result<()> {
    if spec.modules.is_empty() {
        return Err(invalid("host bakes no modules"));
    }

    // One worker for the host itself: it owns the transport, the ctrl_c watch
    // and (for LCM) the shared receive loop. Each module gets its own runtime.
    let main_rt = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(1)
        .thread_name("dm-host")
        .enable_all()
        .build()?;

    let stdin = main_rt.block_on(read_stdin_config())?;
    let suppress = resolve_suppress(spec, &stdin)?;
    let prepared = prepare_all(spec, &stdin)?;

    let transport = Arc::new(main_rt.block_on(open_transport())?);
    let qos = merge_qos(
        &parse_baked("default_qos", spec.default_qos)?,
        stdin.get("qos"),
        &suppress,
    );
    transport.set_publisher_qos(&qos);
    if !suppress.is_empty() {
        info!(host = spec.name, topics = ?suppress, "topics suppressed to this process");
    }

    supervise(spec, prepared, transport, &main_rt)
}

/// Run every prepared module on its own runtime and take the host down as soon
/// as one of them stops, whichever way it stops.
fn supervise(
    spec: &HostSpec,
    prepared: Vec<Prepared>,
    transport: Arc<SharedTransport>,
    main_rt: &tokio::runtime::Runtime,
) -> io::Result<()> {
    let (shutdown_tx, shutdown_rx) = watch::channel(false);
    let (done_tx, mut done_rx) = mpsc::unbounded_channel::<(&'static str, Outcome)>();

    let mut runtimes = Vec::with_capacity(prepared.len());
    for (entry, one) in spec.modules.iter().zip(prepared) {
        let runtime = build_runtime(entry)?;
        let joined = runtime.spawn((one.run)(Arc::clone(&transport), shutdown_rx.clone()));
        let done = done_tx.clone();
        let name = entry.name;
        main_rt.spawn(async move {
            let outcome = match joined.await {
                Ok(res) => Outcome::Finished(res),
                Err(e) if e.is_panic() => Outcome::Panicked,
                Err(_) => Outcome::Finished(Ok(())),
            };
            let _ = done.send((name, outcome));
        });
        runtimes.push(runtime);
        info!(
            host = spec.name,
            module = name,
            threads = entry.threads,
            "module started"
        );
    }
    drop(done_tx);

    // Fail fast: the first module to stop takes the host down with it. A module
    // that returns is as fatal as one that panics — nothing else can drive it.
    let first = main_rt.block_on(async {
        tokio::select! {
            first = done_rx.recv() => first,
            _ = tokio::signal::ctrl_c() => None,
        }
    });

    let _ = shutdown_tx.send(true);

    let failure = match &first {
        Some((name, Outcome::Panicked)) => {
            error!(
                host = spec.name,
                module = name,
                "module panicked, shutting down"
            );
            true
        }
        Some((name, Outcome::Finished(Err(e)))) => {
            error!(host = spec.name, module = name, error = %e, "module failed, shutting down");
            true
        }
        Some((name, Outcome::Finished(Ok(())))) => {
            error!(
                host = spec.name,
                module = name,
                "module exited, shutting down"
            );
            true
        }
        None => {
            info!(host = spec.name, "interrupted, shutting down");
            false
        }
    };

    // Bounded: a wedged module must not keep the host alive.
    main_rt.block_on(async {
        let drain = async { while done_rx.recv().await.is_some() {} };
        if tokio::time::timeout(SHUTDOWN_GRACE, drain).await.is_err() {
            warn!(
                host = spec.name,
                grace_s = SHUTDOWN_GRACE.as_secs_f32(),
                "modules did not stop within the grace period"
            );
        }
    });
    // Dropping a Runtime waits for its tasks; a wedged module would hang here.
    for runtime in runtimes {
        runtime.shutdown_background();
    }

    match first {
        Some((name, _)) if failure => Err(io::Error::other(format!("module `{name}` stopped"))),
        _ => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thread_name_is_prefixed_and_capped() {
        assert_eq!(thread_name("ray_tracing"), "dm-ray_tracing");
        assert_eq!(
            thread_name("a_very_long_module_name").len(),
            MAX_THREAD_NAME
        );
    }

    #[test]
    fn stdin_topics_override_baked_ports_and_keep_the_rest() {
        let defaults = json!({"lidar": "dimos/lidar", "odometry": "dimos/odom"});
        let overrides = json!({"odometry": "dimos/robot/odom"});
        let merged = merge_topics(Some(&defaults), Some(&overrides));
        assert_eq!(merged["lidar"], json!("dimos/lidar"));
        assert_eq!(merged["odometry"], json!("dimos/robot/odom"));
    }

    #[test]
    fn merge_topics_tolerates_missing_sides() {
        assert_eq!(merge_topics(None, None), json!({}));
        let over = json!({"a": "b"});
        assert_eq!(merge_topics(None, Some(&over)), json!({"a": "b"}));
    }

    #[test]
    fn suppressed_topics_become_session_local_publishers() {
        let defaults = json!({"dimos/global_map": {"reliability": "best_effort"}});
        let merged = merge_qos(&defaults, None, &["dimos/global_map".to_string()]);
        assert_eq!(
            merged["dimos/global_map"]["reliability"],
            json!("best_effort")
        );
        assert_eq!(
            merged["dimos/global_map"]["locality"],
            json!("session_local")
        );
    }

    #[test]
    fn suppressing_an_unqosed_topic_still_pins_locality() {
        let merged = merge_qos(&json!({}), None, &["dimos/local_map".to_string()]);
        assert_eq!(
            merged["dimos/local_map"]["locality"],
            json!("session_local")
        );
    }

    #[test]
    fn stdin_qos_overrides_the_baked_entry() {
        let defaults = json!({"dimos/path": {"congestion_control": "drop"}});
        let over = json!({"dimos/path": {"congestion_control": "block"}});
        let merged = merge_qos(&defaults, Some(&over), &[]);
        assert_eq!(merged["dimos/path"]["congestion_control"], json!("block"));
    }

    fn spec_with(default_topics: &'static str) -> HostSpec {
        HostSpec {
            name: "test-host",
            modules: &[],
            default_topics,
            default_suppress: &["dimos/global_map"],
            default_qos: "{}",
            graph_json: "{}",
        }
    }

    #[test]
    fn suppress_defaults_to_the_baked_list() {
        let spec = spec_with("{}");
        assert_eq!(
            resolve_suppress(&spec, &json!({})).unwrap(),
            vec!["dimos/global_map".to_string()]
        );
    }

    #[test]
    fn an_empty_stdin_suppress_list_unsuppresses_everything() {
        let spec = spec_with("{}");
        assert!(resolve_suppress(&spec, &json!({"suppress": []}))
            .unwrap()
            .is_empty());
    }

    #[test]
    fn a_non_array_suppress_is_rejected() {
        let spec = spec_with("{}");
        assert!(resolve_suppress(&spec, &json!({"suppress": "dimos/x"})).is_err());
    }

    #[test]
    fn missing_modules_object_is_rejected() {
        let spec = spec_with("{}");
        let err = prepare_all(&spec, &json!({})).expect_err("stdin must carry `modules`");
        assert!(err.to_string().contains("modules"), "{err}");
    }

    // Two mock modules on one host, over one shared transport.

    use crate::module::{Builder, Input, NoConfig, Output};
    use crate::transport::Dispatch;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Mutex;

    /// Loopback transport: a publish is delivered to this process's own
    /// subscribers, which is all a host needs to wire its modules together.
    #[derive(Default)]
    struct LoopbackTransport {
        routes: Mutex<HashMap<String, Vec<Dispatch>>>,
    }

    impl Transport for LoopbackTransport {
        async fn publish(&self, channel: &str, data: Vec<u8>) -> io::Result<()> {
            let routes = self.routes.lock().unwrap().get(channel).cloned();
            for route in routes.into_iter().flatten() {
                route(&data);
            }
            Ok(())
        }

        async fn subscribe(&self, channel: &str, on_msg: Dispatch) -> io::Result<()> {
            self.routes
                .lock()
                .unwrap()
                .entry(channel.to_string())
                .or_default()
                .push(on_msg);
            Ok(())
        }
    }

    static RECEIVED: AtomicBool = AtomicBool::new(false);
    static SENDER_THREAD: Mutex<Option<String>> = Mutex::new(None);

    /// Publishes on `ping` forever, so the receiver can't miss the message by
    /// subscribing late.
    struct Sender {
        ping: Output<Vec<u8>>,
    }

    impl Module for Sender {
        type Config = NoConfig;

        fn build(builder: &mut Builder, _config: NoConfig) -> Self {
            Self {
                ping: builder.output("ping", |b: &Vec<u8>| b.clone()),
            }
        }

        async fn handle(&mut self) {
            *SENDER_THREAD.lock().unwrap() = std::thread::current().name().map(str::to_string);
            loop {
                let _ = self.ping.publish(&vec![7u8]).await;
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }
    }

    /// Records the first message and returns, which is a fatal event for a host.
    struct Receiver {
        ping: Input<Vec<u8>>,
    }

    impl Module for Receiver {
        type Config = NoConfig;

        fn build(builder: &mut Builder, _config: NoConfig) -> Self {
            Self {
                ping: builder.input("ping", |b| Ok(b.to_vec())),
            }
        }

        async fn handle(&mut self) {
            if self.ping.recv().await.is_some() {
                RECEIVED.store(true, Ordering::SeqCst);
            }
        }
    }

    struct Exploder;

    impl Module for Exploder {
        type Config = NoConfig;

        fn build(_builder: &mut Builder, _config: NoConfig) -> Self {
            Self
        }

        async fn handle(&mut self) {
            panic!("module blew up");
        }
    }

    struct Idler;

    impl Module for Idler {
        type Config = NoConfig;

        fn build(_builder: &mut Builder, _config: NoConfig) -> Self {
            Self
        }

        async fn handle(&mut self) {
            std::future::pending::<()>().await
        }
    }

    fn host_spec(modules: &'static [ModuleEntry], topics: &'static str) -> HostSpec {
        HostSpec {
            name: "test-host",
            modules,
            default_topics: topics,
            default_suppress: &[],
            default_qos: "{}",
            graph_json: "{}",
        }
    }

    fn run_spec(spec: &HostSpec) -> io::Result<()> {
        let stdin = json!({
            "modules": spec
                .modules
                .iter()
                .map(|m| (m.name.to_string(), json!({"config": null})))
                .collect::<Map<String, Value>>(),
        });
        let prepared = prepare_all(spec, &stdin)?;
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(1)
            .enable_all()
            .build()?;
        let transport = Arc::new(SharedTransport::new(LoopbackTransport::default()));
        supervise(spec, prepared, transport, &rt)
    }

    #[test]
    fn two_modules_exchange_a_message_over_one_shared_transport() {
        static MODULES: &[ModuleEntry] = &[
            ModuleEntry::new::<Sender>("sender"),
            ModuleEntry::new::<Receiver>("receiver"),
        ];
        const TOPICS: &str =
            r#"{"sender": {"ping": "host/ping"}, "receiver": {"ping": "host/ping"}}"#;
        RECEIVED.store(false, Ordering::SeqCst);

        // The receiver stopping is what ends the host, so this also covers
        // fail-fast on a module that simply returns.
        let err =
            run_spec(&host_spec(MODULES, TOPICS)).expect_err("a stopped module fails the host");
        assert!(err.to_string().contains("receiver"), "{err}");
        assert!(
            RECEIVED.load(Ordering::SeqCst),
            "the receiver should have seen the sender's message"
        );
    }

    #[test]
    fn a_panicking_module_takes_the_host_down() {
        static MODULES: &[ModuleEntry] = &[
            ModuleEntry::new::<Exploder>("exploder"),
            ModuleEntry::new::<Idler>("idler"),
        ];
        let err = run_spec(&host_spec(MODULES, "{}")).expect_err("a panic must fail the host");
        assert!(err.to_string().contains("exploder"), "{err}");
    }

    #[test]
    fn each_module_runs_on_its_own_named_thread() {
        static MODULES: &[ModuleEntry] = &[
            ModuleEntry::new::<Sender>("sender"),
            ModuleEntry::new::<Receiver>("receiver").threads(2),
        ];
        const TOPICS: &str =
            r#"{"sender": {"ping": "host/ping"}, "receiver": {"ping": "host/ping"}}"#;
        *SENDER_THREAD.lock().unwrap() = None;

        let _ = run_spec(&host_spec(MODULES, TOPICS));
        let name = SENDER_THREAD.lock().unwrap().clone();
        assert_eq!(name.as_deref(), Some("dm-sender"));
    }

    #[test]
    fn a_module_missing_from_stdin_is_named_in_the_error() {
        static MODULES: &[ModuleEntry] = &[ModuleEntry::new::<Idler>("idler")];
        let spec = host_spec(MODULES, "{}");
        let err = prepare_all(&spec, &json!({"modules": {}}))
            .expect_err("every baked module needs a stdin section");
        assert!(err.to_string().contains("modules.idler"), "{err}");
    }

    #[test]
    fn an_unknown_module_section_is_rejected() {
        static MODULES: &[ModuleEntry] = &[ModuleEntry::new::<Idler>("idler")];
        let spec = host_spec(MODULES, "{}");
        let err = prepare_all(&spec, &json!({"modules": {"nope": {"config": null}}}))
            .expect_err("an unbaked module id must be rejected");
        assert!(err.to_string().contains("nope"), "{err}");
    }
}
