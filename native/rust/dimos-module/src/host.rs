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
    init_tracing, log_wiring, parse_config_value, read_launch_config, run_module_core,
    validate_config, Module, Topics,
};
use crate::transport::{SharedTransport, Transport};
use crate::zenoh::{ZenohTransport, SESSION_KEY};

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
struct Prepared {
    topics: Topics,
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
    prepare: fn(&Value) -> io::Result<Prepared>,
}

impl ModuleEntry {
    pub const fn new<M: Module>(name: &'static str) -> Self {
        Self {
            name,
            threads: 1,
            prepare: prepare_module::<M>,
        }
    }

    pub const fn threads(mut self, threads: usize) -> Self {
        self.threads = threads;
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
    /// `{"<module>": {"<port>": "<topic>"}}`: the wiring the graph was drawn
    /// from. Per-module `topics` on stdin override individual ports.
    pub default_topics: &'static str,
    /// Topics whose publishers stay inside this process. Replaced wholesale by
    /// a top-level `suppress` array on stdin.
    pub default_suppress: &'static [&'static str],
    /// `{"<topic>": {"reliability": ..., "congestion_control": ...}}`.
    pub default_qos: &'static str,
    /// The rendered connection graph, printed by `<host> graph`.
    pub graph_json: &'static str,
    /// Identity of the wiring this binary was baked with. A config emitted by
    /// `dimos bake --emit-config` carries the same stamp under `graph`.
    pub graph_hash: &'static str,
}

/// What the command line asks the binary to do. A baked host takes its whole
/// wiring on the launch line, so the only arguments it knows are `graph`.
#[derive(Debug, PartialEq)]
enum Action {
    Run,
    Graph { raw: bool },
    Unknown(String),
}

fn parse_args(args: impl IntoIterator<Item = String>) -> Action {
    let args: Vec<String> = args.into_iter().collect();
    match args.first().map(String::as_str) {
        None => Action::Run,
        Some("graph") => Action::Graph {
            raw: args.iter().any(|a| a == "--json"),
        },
        Some(other) => Action::Unknown(other.to_string()),
    }
}

/// Entry point of a baked host binary. Never returns.
pub fn host_main(spec: &HostSpec) -> ! {
    match parse_args(std::env::args().skip(1)) {
        Action::Run => run_host(spec),
        Action::Graph { raw } => {
            println!("{}", render_graph(spec, raw));
            std::process::exit(0)
        }
        Action::Unknown(other) => {
            eprintln!(
                "{}: unknown argument {other:?}; usage: {0} [graph [--json]]",
                spec.name
            );
            std::process::exit(2)
        }
    }
}

/// The baked graph as `<host> graph` prints it. Unparseable JSON falls back to
/// the raw string, so the operator sees what was baked either way.
fn render_graph(spec: &HostSpec, raw: bool) -> String {
    if raw {
        return spec.graph_json.to_string();
    }
    serde_json::from_str::<Value>(spec.graph_json)
        .and_then(|v| serde_json::to_string_pretty(&v))
        .unwrap_or_else(|_| spec.graph_json.to_string())
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

/// Whether a `suppress` entry names this topic, by full topic or channel name.
fn suppresses(topic: &str, entry: &str) -> bool {
    topic == entry || topic.split('/').nth(1) == Some(entry)
}

/// The baked list, kept to the topics this host actually touches.
///
/// A python driver can rewire a channel, which leaves a baked entry naming a
/// topic nothing here publishes. Suppressing that is a no-op, so drop it and
/// say so rather than reporting a suppression that does nothing.
fn baked_suppress(spec: &HostSpec, known: &[String]) -> Vec<String> {
    let mut resolved: Vec<String> = Vec::new();
    for entry in spec.default_suppress {
        let mut matched = false;
        for topic in known.iter().filter(|t| suppresses(t, entry)) {
            matched = true;
            if !resolved.contains(topic) {
                resolved.push(topic.clone());
            }
        }
        if !matched {
            warn!(
                host = spec.name,
                topic = entry,
                "baked suppression names a topic this host does not touch, ignoring it"
            );
        }
    }
    resolved
}

/// The effective suppression list: the baked one unless stdin replaces it.
///
/// A stdin entry names a channel (`local_map`) or a full topic, like the CLI's
/// `--suppress`. An entry matching nothing this host touches is an error: a
/// typo must not silently publish the topic it meant to keep in-process.
fn resolve_suppress(spec: &HostSpec, stdin: &Value, known: &[String]) -> io::Result<Vec<String>> {
    let Some(value) = stdin.get("suppress") else {
        return Ok(baked_suppress(spec, known));
    };
    let list = value
        .as_array()
        .ok_or_else(|| invalid("`suppress` must be an array of topic strings"))?;
    let mut resolved: Vec<String> = Vec::new();
    for value in list {
        let entry = value
            .as_str()
            .ok_or_else(|| invalid("`suppress` entries must be topic strings"))?;
        let matches: Vec<&String> = known.iter().filter(|t| suppresses(t, entry)).collect();
        if matches.is_empty() {
            return Err(invalid(format!(
                "`suppress` names `{entry}`, which no baked module touches"
            )));
        }
        for topic in matches {
            if !resolved.contains(topic) {
                resolved.push(topic.clone());
            }
        }
    }
    Ok(resolved)
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

/// Refuse a config baked for a different graph than this binary.
///
/// The file's topics override the baked ones, so a stale one wires the host to
/// keys nothing else uses: a clean wiring table and no messages.
fn check_graph_stamp(spec: &HostSpec, stdin: &Value) -> io::Result<()> {
    match stdin.get("graph").and_then(Value::as_str) {
        Some(stamp) if stamp == spec.graph_hash => Ok(()),
        Some(stamp) => Err(invalid(format!(
            "config was baked for graph `{stamp}`, this binary is `{}`: \
             re-run `dimos bake --emit-config`",
            spec.graph_hash
        ))),
        None => {
            warn!(
                host = spec.name,
                "config carries no `graph` stamp, so its wiring is taken on trust"
            );
            Ok(())
        }
    }
}

/// Refuse an LCM session over zenoh-shaped topics. The keys a bake emits
/// (`dimos/<name>/<type>`) name nothing on the LCM bus, so the host would run,
/// log a clean wiring table and exchange no messages at all.
fn check_topics_match_transport(transport: &str, known: &[String]) -> io::Result<()> {
    if transport != "lcm" {
        return Ok(());
    }
    if let Some(topic) = known.iter().find(|t| t.starts_with("dimos/")) {
        return Err(invalid(format!(
            "DIMOS_TRANSPORT=lcm but topic `{topic}` is zenoh-shaped; this config \
             was baked for zenoh"
        )));
    }
    Ok(())
}

/// The launch line owns the session. Bake cannot know the deployment's network,
/// so it emits no `session` block and zenoh's defaults stand until one is added.
fn note_default_session(launch: &Value) {
    if matches!(launch.get(SESSION_KEY), None | Some(Value::Null)) {
        warn!(
            "no `{SESSION_KEY}` block on the launch line, opening zenoh's defaults; \
             add one to pin the mode, interface and endpoints"
        );
    }
}

async fn open_transport(launch: &Value) -> io::Result<SharedTransport> {
    match std::env::var("DIMOS_TRANSPORT").as_deref() {
        Ok("lcm") => Ok(SharedTransport::new(LcmTransport::new().await?)),
        Ok("zenoh") => {
            note_default_session(launch);
            Ok(SharedTransport::new(
                ZenohTransport::from_launch(launch).await?,
            ))
        }
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

/// The module's own runtime. Multi-thread even for a single worker: zenoh
/// panics on a current_thread scheduler, and every module shares its session.
fn build_runtime(entry: &ModuleEntry) -> io::Result<tokio::runtime::Runtime> {
    tokio::runtime::Builder::new_multi_thread()
        .worker_threads(entry.threads.max(1))
        .thread_name(thread_name(entry.name))
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

    let stdin = main_rt.block_on(read_launch_config())?;
    check_graph_stamp(spec, &stdin)?;
    let prepared = prepare_all(spec, &stdin)?;
    let known: Vec<String> = prepared
        .iter()
        .flat_map(|p| p.topics.channels().cloned())
        .collect();
    let suppress = resolve_suppress(spec, &stdin, &known)?;
    if let Ok(transport) = std::env::var("DIMOS_TRANSPORT") {
        check_topics_match_transport(&transport, &known)?;
    }

    let transport = Arc::new(main_rt.block_on(open_transport(&stdin))?);
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

type DoneSender = mpsc::UnboundedSender<(&'static str, Outcome)>;
type DoneReceiver = mpsc::UnboundedReceiver<(&'static str, Outcome)>;

/// Spawn every prepared module on its own runtime, reporting each stop on
/// `done`.
fn spawn_modules(
    spec: &HostSpec,
    prepared: Vec<Prepared>,
    transport: &Arc<SharedTransport>,
    main_rt: &tokio::runtime::Runtime,
    shutdown_rx: &watch::Receiver<bool>,
    done_tx: &DoneSender,
) -> io::Result<Vec<tokio::runtime::Runtime>> {
    let mut runtimes = Vec::with_capacity(prepared.len());
    for (entry, one) in spec.modules.iter().zip(prepared) {
        let runtime = build_runtime(entry)?;
        let joined = runtime.spawn((one.run)(Arc::clone(transport), shutdown_rx.clone()));
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
    }
    Ok(runtimes)
}

/// Log why the host is going down. True when a module stopping caused it.
fn note_first_stop(spec: &HostSpec, first: &Option<(&'static str, Outcome)>) -> bool {
    match first {
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
    }
}

/// Wait for the remaining modules to stop. Bounded: a wedged module must not
/// keep the host alive.
fn drain_modules(spec: &HostSpec, main_rt: &tokio::runtime::Runtime, done_rx: &mut DoneReceiver) {
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
    let (done_tx, mut done_rx) = mpsc::unbounded_channel();
    let runtimes = spawn_modules(spec, prepared, &transport, main_rt, &shutdown_rx, &done_tx)?;
    drop(done_tx);

    // Fail fast: the first module to stop takes the host down with it. A module
    // that returns is as fatal as one that panics: nothing else can drive it.
    let first = main_rt.block_on(async {
        tokio::select! {
            first = done_rx.recv() => first,
            _ = tokio::signal::ctrl_c() => None,
        }
    });

    let _ = shutdown_tx.send(true);
    let failure = note_first_stop(spec, &first);
    drain_modules(spec, main_rt, &mut done_rx);
    // Dropping a Runtime waits for its tasks. A wedged module would hang here.
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
    use std::collections::HashMap;

    #[test]
    fn thread_name_is_prefixed_and_capped() {
        assert_eq!(thread_name("ray_tracing"), "dm-ray_tracing");
        assert_eq!(
            thread_name("a_very_long_module_name").len(),
            MAX_THREAD_NAME
        );
    }

    fn argv(args: &[&str]) -> Action {
        parse_args(args.iter().map(|a| a.to_string()))
    }

    #[test]
    fn bare_argv_runs_the_host() {
        assert_eq!(argv(&[]), Action::Run);
    }

    #[test]
    fn graph_argv_selects_pretty_or_raw() {
        assert_eq!(argv(&["graph"]), Action::Graph { raw: false });
        assert_eq!(argv(&["graph", "--json"]), Action::Graph { raw: true });
    }

    /// The wiring arrives on the launch line, so a topic flag is a caller that
    /// thinks it is driving a lone module. Naming it beats ignoring it.
    #[test]
    fn a_topic_flag_is_an_unknown_argument() {
        assert_eq!(
            argv(&["--lidar", "dimos/lidar/sensor_msgs.PointCloud2"]),
            Action::Unknown("--lidar".to_string())
        );
    }

    fn graph_spec(graph_json: &'static str) -> HostSpec {
        HostSpec {
            graph_json,
            ..spec_with("{}")
        }
    }

    #[test]
    fn render_graph_pretty_prints_unless_asked_for_raw() {
        let spec = graph_spec(r#"{"host":"go2-nav"}"#);
        assert_eq!(render_graph(&spec, true), r#"{"host":"go2-nav"}"#);
        assert_eq!(render_graph(&spec, false), "{\n  \"host\": \"go2-nav\"\n}");
    }

    #[test]
    fn render_graph_falls_back_to_the_raw_string() {
        let spec = graph_spec("not json");
        assert_eq!(render_graph(&spec, false), "not json");
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_launch_line_with_no_session_says_so() {
        note_default_session(&json!({}));
        assert!(logs_contain("opening zenoh's defaults"));
    }

    #[test]
    #[tracing_test::traced_test]
    fn a_launch_line_with_a_session_is_quiet() {
        note_default_session(&json!({"session": {"mode": "peer"}}));
        assert!(!logs_contain("opening zenoh's defaults"));
    }

    #[test]
    fn a_config_baked_for_another_graph_is_refused() {
        let spec = spec_with("{}");
        let err = check_graph_stamp(&spec, &json!({"graph": "0123456789abcdef"}))
            .expect_err("a stale stamp must fail");
        let message = err.to_string();
        assert!(message.contains("0123456789abcdef"), "{message}");
        assert!(message.contains("test-graph"), "{message}");
        assert!(message.contains("--emit-config"), "{message}");
    }

    #[test]
    fn a_matching_graph_stamp_is_accepted() {
        let spec = spec_with("{}");
        check_graph_stamp(&spec, &json!({"graph": "test-graph"})).expect("the stamp matches");
    }

    /// A python-driven host builds its wiring live, so it sends no stamp.
    #[test]
    fn a_config_with_no_graph_stamp_is_taken_on_trust() {
        let spec = spec_with("{}");
        check_graph_stamp(&spec, &json!({})).expect("an unstamped config still runs");
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
            graph_hash: "test-graph",
        }
    }

    #[test]
    fn suppress_defaults_to_the_baked_list() {
        let spec = spec_with("{}");
        let known = vec!["dimos/global_map".to_string()];
        assert_eq!(
            resolve_suppress(&spec, &json!({}), &known).unwrap(),
            vec!["dimos/global_map".to_string()]
        );
    }

    /// The stdin path errors on an entry nothing touches. The baked path cannot,
    /// because a python driver may legitimately have rewired the channel.
    #[test]
    #[tracing_test::traced_test]
    fn a_baked_suppression_the_host_does_not_touch_is_dropped() {
        let spec = spec_with("{}");
        let known = vec!["dimos/local_map/sensor_msgs.PointCloud2".to_string()];
        assert!(resolve_suppress(&spec, &json!({}), &known)
            .unwrap()
            .is_empty());
        assert!(logs_contain("does not touch"));
    }

    #[test]
    fn an_empty_stdin_suppress_list_unsuppresses_everything() {
        let spec = spec_with("{}");
        assert!(resolve_suppress(&spec, &json!({"suppress": []}), &[])
            .unwrap()
            .is_empty());
    }

    #[test]
    fn a_non_array_suppress_is_rejected() {
        let spec = spec_with("{}");
        assert!(resolve_suppress(&spec, &json!({"suppress": "dimos/x"}), &[]).is_err());
    }

    #[test]
    fn lcm_over_zenoh_shaped_topics_is_refused() {
        let known = vec!["dimos/local_map/sensor_msgs.PointCloud2".to_string()];
        assert!(check_topics_match_transport("lcm", &known).is_err());
        assert!(check_topics_match_transport("zenoh", &known).is_ok());
        assert!(check_topics_match_transport("lcm", &["/local_map".to_string()]).is_ok());
    }

    #[test]
    fn a_channel_name_suppress_entry_resolves_to_the_full_topic() {
        let spec = spec_with("{}");
        let known = vec!["dimos/local_map/sensor_msgs.PointCloud2".to_string()];
        assert_eq!(
            resolve_suppress(&spec, &json!({"suppress": ["local_map"]}), &known).unwrap(),
            known
        );
    }

    #[test]
    fn a_full_topic_suppress_entry_is_kept() {
        let spec = spec_with("{}");
        let known = vec!["dimos/local_map/sensor_msgs.PointCloud2".to_string()];
        let stdin = json!({"suppress": ["dimos/local_map/sensor_msgs.PointCloud2"]});
        assert_eq!(resolve_suppress(&spec, &stdin, &known).unwrap(), known);
    }

    #[test]
    fn a_suppress_entry_matching_no_topic_is_rejected() {
        let spec = spec_with("{}");
        let known = vec!["dimos/local_map/sensor_msgs.PointCloud2".to_string()];
        let err = resolve_suppress(&spec, &json!({"suppress": ["local_mpa"]}), &known).unwrap_err();
        assert!(err.to_string().contains("local_mpa"));
    }

    #[test]
    fn missing_modules_object_is_rejected() {
        let spec = spec_with("{}");
        let err = prepare_all(&spec, &json!({})).expect_err("stdin must carry `modules`");
        assert!(err.to_string().contains("modules"), "{err}");
    }

    // Two mock modules on one host, over one shared transport.

    use crate::module::{Builder, Input, NativeConfig, NoConfig, Output};
    use crate::transport::Dispatch;
    use std::sync::{LazyLock, Mutex};

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

    /// What the mock modules report, keyed per test so cargo's concurrent
    /// test threads cannot clobber each other.
    #[derive(Default, Clone)]
    struct Probe {
        received: bool,
        sender_thread: Option<String>,
    }

    static PROBES: LazyLock<Mutex<HashMap<String, Probe>>> = LazyLock::new(Default::default);

    fn probe(key: &str) -> Probe {
        PROBES.lock().unwrap().get(key).cloned().unwrap_or_default()
    }

    #[derive(Debug, serde::Deserialize, serde::Serialize)]
    #[serde(deny_unknown_fields)]
    struct ProbeConfig {
        key: String,
    }

    impl NativeConfig for ProbeConfig {}

    impl validator::Validate for ProbeConfig {
        fn validate(&self) -> Result<(), validator::ValidationErrors> {
            Ok(())
        }
    }

    /// Publishes on `ping` forever, so the receiver can't miss the message by
    /// subscribing late.
    struct Sender {
        key: String,
        ping: Output<Vec<u8>>,
    }

    impl Module for Sender {
        type Config = ProbeConfig;

        fn build(builder: &mut Builder, config: ProbeConfig) -> Self {
            Self {
                key: config.key,
                ping: builder.output("ping", |b: &Vec<u8>| b.clone()),
            }
        }

        async fn handle(&mut self) {
            let name = std::thread::current().name().map(str::to_string);
            PROBES
                .lock()
                .unwrap()
                .entry(self.key.clone())
                .or_default()
                .sender_thread = name;
            loop {
                let _ = self.ping.publish(&vec![7u8]).await;
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }
    }

    /// Records the first message and returns, which is a fatal event for a host.
    struct Receiver {
        key: String,
        ping: Input<Vec<u8>>,
    }

    impl Module for Receiver {
        type Config = ProbeConfig;

        fn build(builder: &mut Builder, config: ProbeConfig) -> Self {
            Self {
                key: config.key,
                ping: builder.input("ping", |b| Ok(b.to_vec())),
            }
        }

        async fn handle(&mut self) {
            if self.ping.recv().await.is_some() {
                PROBES
                    .lock()
                    .unwrap()
                    .entry(self.key.clone())
                    .or_default()
                    .received = true;
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
            graph_hash: "test-graph",
        }
    }

    fn run_spec(spec: &HostSpec, config: Value) -> io::Result<()> {
        let stdin = json!({
            "modules": spec
                .modules
                .iter()
                .map(|m| (m.name.to_string(), json!({"config": config.clone()})))
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

        // The receiver stopping is what ends the host, so this also covers
        // fail-fast on a module that simply returns.
        let err = run_spec(&host_spec(MODULES, TOPICS), json!({"key": "exchange"}))
            .expect_err("a stopped module fails the host");
        assert!(err.to_string().contains("receiver"), "{err}");
        assert!(
            probe("exchange").received,
            "the receiver should have seen the sender's message"
        );
    }

    #[test]
    fn a_panicking_module_takes_the_host_down() {
        static MODULES: &[ModuleEntry] = &[
            ModuleEntry::new::<Exploder>("exploder"),
            ModuleEntry::new::<Idler>("idler"),
        ];
        let err = run_spec(&host_spec(MODULES, "{}"), Value::Null)
            .expect_err("a panic must fail the host");
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

        let _ = run_spec(&host_spec(MODULES, TOPICS), json!({"key": "threads"}));
        assert_eq!(probe("threads").sender_thread.as_deref(), Some("dm-sender"));
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
