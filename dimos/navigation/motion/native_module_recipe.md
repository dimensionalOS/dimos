# Native module recipe: the four robot-side motion modules

Implementation spec for the rust native modules the deployment plan
([deployment_plan.md](deployment_plan.md) — read it first) calls for:
`motion_planner`, `trajectory_follower`, `cmd_vel_mux`, `odom_body_frame`.
Part 1 is the procedure every module follows, written once so the four do not
diverge. Part 2 is one self-contained brief per module.

**Brief 4 is dead.** `odom_body_frame` shipped, then was deleted: the mount is
a rotation AND a lever arm, and dimos grew real tf support (`navigation/
tf_pose.py`, `dimos-module/src/tf.rs`) that composes both. The planner and the
follower now resolve the base pose off tf themselves — see
`adapter/rust/src/tf_pose.rs`. The brief stays below because Part 1's procedure
was written against it and the second-module-id-in-one-crate argument is still
the live guidance.

Ground truth exemplars: `dimos/mapping/ray_tracing/rust/` (module id
`ray_tracing`, the cleanest) and `dimos/navigation/nav_3d/mls_planner/rust/`
(`mls_planner`). The machinery: `native/rust/dimos-module/`,
`native/rust/dimos-module-macros/`, `dimos/cli/bake/`,
`dimos/core/native_module.py`, `dimos/core/baked_host.py`. Commits `5f7de46a5`
(module-lib split, port registry, host runtime) and `b619a7b6f` (`dimos bake`,
`baked_host()`) carry the intent.

## Decisions (defaults picked rather than stalled on)

- **Q1 — one crate or two for planner+follower: ONE crate,
  `dimos/navigation/motion/adapter/rust/`, two module ids, two `[[bin]]`s.**
  The registry is keyed by module id, not by crate — `discovery.py`
  `parse_manifest` iterates every `[package.metadata.dimos.module.<id>]` table
  in one manifest (discovery.py:111), and the macro cross-check is per-id too,
  so multi-module crates are the supported shape (odom_body_frame proves it in
  the other direction, as a second id in the mls crate). Both adapters depend
  on the same two pure crates (`dimos-motion2-target` for the planner,
  `dimos-motion2-tc` for the follower AND for the planner's
  `stamps`/`clearance` facilities), so one crate means one dependency list,
  one Cargo.lock, one `.cargo/config.toml`, and it mirrors the python layout
  (`adapter/planner.py` + `adapter/follower.py` in one package). The
  deployment plan already leans this way ("Start with one and split if it
  chafes"). Cost: the two briefs share files, so they do not parallelize —
  see the matrix at the end.
- **Q2 — where the python wrappers live: alongside the modules they replace.**
  `adapter/` for the two adapter wrappers, `movement_manager/` for the mux,
  `mls_planner/` for odom_body_frame. This is the exemplar pattern
  (`mapping/ray_tracing/module.py`, `mls_planner/mls_planner_native.py`) and
  `--emit-config` imports the wrapper by the `python =` ref, so proximity to
  the config source of truth is the point.
- **Q3 — genuinely open items are marked OPEN inline.** The big one: how tuned
  config reaches the robot host. `--emit-config` builds the stdin blob from
  *python class defaults* (cli.py:34-44 `default_config` instantiates
  `config_type()`), not from blueprint values. Until bake learns to read a
  blueprint, the deployment artifact is a hand-written stdin JSON — the
  deployment plan's "config is the sharp edge" section, with
  `mount_rotation` as the test case. This spec sets wrapper defaults to the
  blueprint-tuned values wherever a single sane default exists, precisely to
  shrink that gap, and marks OPEN what it cannot.

---

# Part 1 — the recipe

## 1. Crate layout

```
<crate>/
  Cargo.toml            [lib] + one [[bin]] per module id + registry tables
  .cargo/config.toml    macOS: -undefined dynamic_lookup (copy from mls crate)
  src/
    lib.rs              pub mod module; #[cfg(feature = "python")] mod python;
    module.rs           the #[derive(Module)] struct(s) — linkable into a host
    main.rs             the standalone shim
    python.rs           optional pyo3 extension, behind the feature
```

The split exists because `dimos bake` links `module.rs` (the rlib) into a
host binary while `main.rs` keeps the crate independently runnable. From
`ray_tracing`:

```rust
// src/main.rs — the whole file (ray_tracing/rust/src/main.rs:15-20)
use dimos_voxel_ray_tracing::module::RayTracingVoxelMap;

#[tokio::main]
async fn main() {
    dimos_module::run_with_transport::<RayTracingVoxelMap>().await;
}
```

```toml
# Cargo.toml (ray_tracing/rust/Cargo.toml:8-16)
[lib]
# Must match the #[pymodule] fn name in python.rs.
name = "dimos_voxel_ray_tracing"
path = "src/lib.rs"
crate-type = ["cdylib", "rlib"]   # cdylib = python ext, rlib = what bake links

[[bin]]
name = "voxel_ray_tracing"
path = "src/main.rs"
```

`run_with_transport` (dimos-module/src/lib.rs:37-57) picks the transport from
the `DIMOS_TRANSPORT` env var (`lcm`|`zenoh`, always set by the python
wrapper; unset panics). A crate carrying two module ids has two `[[bin]]`
entries pointing at two `main.rs`-style shims (`src/bin/*.rs` or two files
named in `[[bin]] path`), each three lines like the above.

The module struct itself:

```rust
// ray_tracing/rust/src/module.rs:28-49 (abridged)
#[derive(Module)]
#[module(name = "ray_tracing")]           // must equal the registry table key
pub struct RayTracingVoxelMap {
    #[input(decode = PointCloud2::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,
    #[input(decode = Odometry::decode, handler = on_odometry)]
    odometry: Input<Odometry>,
    #[output(encode = PointCloud2::encode)]
    global_map: Output<PointCloud2>,
    #[config]
    config: Config,
    // any other field is plain state, initialized with Default::default()
    map: VoxelMap,
}
```

The derive generates `Module::build` (ports created via
`builder.input(<field name>, decode)` — the **field name is the port name**),
and a `handle()` that is a `tokio::select!` loop dispatching each input to its
handler (macros/src/lib.rs:301-323). Handlers are `async fn on_x(&mut self,
msg: T)` on the struct; `&mut self` means handlers are serialized — a slow
handler starves the others (the deployment plan's raycaster tail analysis is
about exactly this). Modules that need periodic ticks or heavy work off the
dispatch loop use `#[module(setup = ..., teardown = ...)]` to spawn a worker
task sharing state via `Arc<Mutex<...>>` + `Notify` — the `mls_planner`
worker pattern (mls_planner/rust/src/module.rs:48-115) is the template, and
it is the template for the planner's and follower's fixed-rate loops.

## 2. The registry table

```toml
# ray_tracing/rust/Cargo.toml:20-32
[package.metadata.dimos.module.ray_tracing]
path = "dimos_voxel_ray_tracing::module::RayTracingVoxelMap"
python = "dimos.mapping.ray_tracing.module:RayTracingVoxelMap"
threads = 1

[package.metadata.dimos.module.ray_tracing.inputs]
lidar = "sensor_msgs.PointCloud2"
odometry = "nav_msgs.Odometry"

[package.metadata.dimos.module.ray_tracing.outputs]
global_map = "sensor_msgs.PointCloud2"
...
```

Keys, per `discovery.py:parse_manifest` (discovery.py:98-137):

| key | required | meaning |
|---|---|---|
| `path` | yes (string) | full rust path of the struct, used verbatim in the generated host's `ModuleEntry::new::<{path}>(...)` (codegen.py:96-105) |
| `python` | yes (string) | `pkg.mod:Class` of the python `NativeModule` wrapper; imported by `--emit-config` (cli.py:34-44). This is what makes every bake module a standalone module too — the key is required, so a wrapper class must exist. |
| `threads` | no (int >= 1, default 1) | worker threads of the module's runtime **inside a baked host** (host.rs `ModuleEntry.threads`). Set 2 if the module uses `block_in_place` in a worker task, or the dispatch loop stalls — that is why `mls_planner` is `threads = 2` (its Cargo.toml:20-25). Standalone `#[tokio::main]` is multi-thread regardless. |
| `nice` | no (int) | `setpriority` for the module's threads in a host |
| `inputs` / `outputs` | tables | `port = "pkg.MsgType"`. Port names must equal struct field names; the **full** `pkg.MsgType` string matters because it is embedded in the zenoh key (see §7). |

Module ids are underscore-spelled; `-` and `_` are interchangeable on the CLI
only (discovery.py:75-77). An id declared twice anywhere under `dimos/` or
`native/` is a `BakeError` (discovery.py:146-151).

## 3. The compile-time cross-check

`#[module(name = "<id>")]` makes the derive read the crate's own Cargo.toml
at macro expansion (via `CARGO_MANIFEST_DIR`, macros/src/lib.rs:405-417) and
compare the struct's `#[input]`/`#[output]` fields against the
`inputs`/`outputs` tables: every declared port must exist in the table, every
table port must exist on the struct, and the **last dot-segment** of the
message type must match the `Input<T>`/`Output<T>` payload type name
(macros/src/lib.rs:441-475). Drift is a compile error on the struct:

```
error: #[module(name = "ray_tracing")] does not match Cargo.toml: port `odometry` is missing from `inputs`
error: #[module(name = "ray_tracing")] does not match Cargo.toml: port `lidar` is declared `sensor_msgs.PointCloud2` in `inputs` but the struct field carries `Odometry`
error: #[module(name = "ray_tracing")] does not match Cargo.toml: `inputs` declares port `lidar`, which the struct has no #[input] field for
error: #[module(name = "wrong_id")] does not match Cargo.toml: no [package.metadata.dimos.module.wrong_id] section in Cargo.toml
```

Only the last segment is compared (`sensor_msgs.PointCloud2` vs struct's
`PointCloud2`), so the *package* half of the type string is **not** checked by
the compiler — getting `nav_msgs.Path` vs `geometry_msgs.Path` wrong compiles
fine and lands on a dead zenoh key. Check package names by hand against the
python message's `msg_name`.

## 4. The `python` feature gate

```toml
# ray_tracing/rust/Cargo.toml:34-39
[features]
# pyo3 is only needed for the python extension module. `dimos bake` links this
# crate into a host binary with default-features off so no pyo3 objects (and
# their libpython symbols) reach a statically linked build.
default = ["python"]
python = ["dep:pyo3", "dep:numpy"]
```

`codegen.py:_dependencies` (codegen.py:85-93) generates the host's dependency
as `{ path = ..., default-features = false }`. pyo3 must therefore be
`optional = true` and gated, or the baked host drags libpython symbols into a
static/cross build and the link fails (or worse, succeeds dynamically and the
robot has no python). `lib.rs` gates the pyo3 module:

```rust
#[cfg(feature = "python")]
mod python;
```

The pure algorithm crates (`dimos-motion2-target`, `dimos-motion2-tc`)
already have this exact feature shape; the new adapter crates depend on them
with `default-features = false` so no pyo3 arrives transitively.

## 5. Config discipline: python owns every default

The rust config struct uses `#[native_config]` (macros/src/lib.rs:50-76):

```rust
#[native_config]
pub struct Config {
    #[validate(range(exclusive_min = 0.0))]
    pub voxel_size: f32,
    ...
}
```

It injects `Deserialize, Serialize, Validate` + `deny_unknown_fields` and
**rejects at compile time**: `Option<T>` fields, `#[serde(default)]` (field or
container), `skip`, `flatten` — because python owns defaults and must send
every field, always (macros/src/lib.rs:100-158; the error strings say so).
At startup, `parse_config_value` additionally enforces a one-to-one key match
between the JSON `config` object and the struct
(dimos-module/src/module.rs:180-198): a missing key OR an extra key is
`config keys do not match struct fields: missing [...], unexpected [...]` and
the process exits 1 before subscribing to anything. Validation
(`validator` attributes) runs right after, same fate.

Consequences that bite the four modules directly:

- Python wrapper config fields typed `X | None` cannot cross the boundary:
  `to_config_dict` drops `None` values (native_module.py:139-150), which then
  fails the one-to-one check as "missing". The follower's
  `half_width: float | None` and `controller: str | None` must be resolved to
  concrete values (or dropped) in the wrapper — see the brief.
- Nested config objects work (pydantic `model_dump()` nests; serde nests) but
  `#[native_config]`'s checks apply only to the top struct. A nested struct
  must hand-carry `#[serde(deny_unknown_fields)]`, avoid `Option`, and be
  wired with `#[validate(nested)]`. The top-level one-to-one check does not
  recurse; serde's required-field errors are the backstop.
- Fields inherited from `NativeModuleConfig` (executable, cwd, stdin_config,
  ...) never cross unless opted in via `base_fields`
  (native_module.py:129-137). Do not name a module config field after a base
  field — it silently vanishes from the blob.

## 6. The python `NativeModule` wrapper

The exemplar, whole (mapping/ray_tracing/module.py:28-71, abridged):

```python
class RayTracingVoxelMapConfig(NativeModuleConfig):
    cwd: str | None = "rust"                       # resolved rel. to the wrapper file
    executable: str = "result/bin/voxel_ray_tracing"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True                      # mandatory for this stack
    voxel_size: float = 0.1                        # module fields = the blob
    ...

class RayTracingVoxelMap(NativeModule, mapping.GlobalPointcloud):
    config: RayTracingVoxelMapConfig
    lidar: In[PointCloud2]
    odometry: In[Odometry]
    global_map: Out[PointCloud2]
```

Prefer the `mls_planner_native.py` build settings for new crates —
`executable: "target/release/<bin>"`, `build_command: "cargo build --release"`
— see gotcha #1.

What `start()` does (native_module.py:243-303): collects the connected
streams' transport channels per port (`_collect_topics`,
native_module.py:491-503), spawns
`<executable> --<port> <topic> ... <config cli args>` (argv is ignored by the
rust side; it reads stdin), then writes **one JSON line** to the child's
stdin and closes it:

```json
{"topics": {"lidar": "dimos/lidar/sensor_msgs.PointCloud2", "...": "..."},
 "config": {"voxel_size": 0.1},
 "qos": {"dimos/local_map/sensor_msgs.PointCloud2": {}}}
```

(`_stdin_blob`, native_module.py:480-489; read by `read_stdin_config`,
dimos-module/src/module.rs:458-464). The env gets `DIMOS_TRANSPORT`,
`DIMOS_ZENOH_CONNECT` (explicit dial endpoints — multicast-hostile LANs),
`DIMOS_ZENOH_INTERFACE` (the parent's resolved scouting interface, so parent and
child discover on the same one), `DIMOS_ZENOH_MODE` (the parent's session mode,
so a `client` deployment doesn't leave a peer meshing past the router),
`DIMOS_ZENOH_MULTICAST` / `DIMOS_ZENOH_GOSSIP` (`on`/`off`, the parent's
discovery knobs — a child left gossiping meshes past that router anyway) and
`RUST_LOG` mapped from the python log level
(native_module.py:177-211). The
child's JSON stderr lines are re-emitted through the python logger; a child
that dies unexpectedly takes the module down (`_watch_process`). `prctl`
PDEATHSIG kills the child if python dies.

In a baked host the same wrapper class is a *member*: `baked_host(...)`
(core/baked_host.py:122-156) creates a union-port NativeModule whose stdin
blob nests one `{topics, config}` section per member id.

## 7. Ports to zenoh keys, and agreeing with python producers

The full key for a logical channel is:

```
dimos/<channel name>/<pkg.MsgType>          e.g. dimos/local_map/sensor_msgs.PointCloud2
```

Built python-side by `transport_topic` (`"dimos/" + name`,
core/transport_factory.py:44-53) plus `ZenohTopic.key_expr` appending
`lcm_type.msg_name` (protocol/pubsub/impl/zenohpubsub.py:88-95). Built
bake-side by `graph.py:topic_for` (graph.py:36-44), whose docstring states the
contract: *"A baked host must agree, or a standalone run would sit on keys
nobody else uses."*

How a module agrees with a python producer:

- **Same channel name.** Autoconnect wires ports by name; blueprints remap
  (`.remappings([(MLSPlannerNative, "path", "planner_path")])`), and the
  bake equivalent is `--remap mls_planner.path=planner_path`. The
  rust module never hardcodes topics: `Builder.topic_for` reads the stdin
  `topics` map (dimos-module/src/module.rs:296-301).
- **Same `pkg.MsgType` string.** It is part of the key. The metadata table
  string must equal the dimos message's `msg_name` exactly.
- The `topic_for` fallback for an unmapped port is `"/{port}"` — an LCM-shaped
  default that under zenoh is a dead key. The python wrapper only sends topics
  for ports whose streams are actually connected, so an unwired In port
  silently subscribes to nothing meaningful. Fine for optional ports; a trap
  for required ones.

QoS: bake mirrors `default_zenoh_qos` — PointCloud2/Image channels get
latest-wins (graph.py:47-56, transport_factory.py:58-70). Twist/Bool/Path
channels ride the defaults; nothing to do.

## 8. Testing

What the exemplars do, all runnable with no hardware:

1. **Pure-logic unit tests in `module.rs`** under `#[cfg(test)]` — the
   ray_tracing crate tests `nearest_pose`, `emit_due`, cylinder filtering
   directly on the module's helper functions (module.rs:364-582); mls tests
   `stamps_paired`, `goal_position` (NaN-cancel) (module.rs:593-650). Write
   the module so the decision logic is free functions/structs over plain data
   and the async handlers are thin — that is what makes this possible.
2. **`cargo test` / `cargo test --no-default-features`** — the latter is what
   proves the python feature gate holds (the pure crates document this in
   their Cargo.toml comments).
3. **Registry/graph checks without compiling**: `dimos bake --list` shows the
   parsed table; `dimos bake <ids> -o /tmp/x --dry-run` prints the wiring and
   flags type conflicts. `dimos/cli/bake/test_bake_e2e.py` is the pattern for
   asserting a module's graph.
4. **Standalone process smoke test**: build the bin, run with
   `DIMOS_TRANSPORT=zenoh` and a hand-written stdin line, drive it from a
   python script publishing on the mapped keys (the
   `examples/native-modules/rust_ping_pong.py` pattern).
5. **Parity stays in the pure crates** (`control/test_rust_parity.py`,
   `planner/rust/tests/invariants.rs`). Module crates do NOT
   re-test law/planner numerics — they test wiring, gating, staleness and
   lifecycle. Existing python module tests (`adapter/test_planner.py`,
   `adapter/test_follower.py`, `movement_manager/test_movement_manager.py`,
   `mls_planner/test_odom_body_frame.py`) are the behavioral spec to port.

## 9. Gotchas a first-timer WILL hit

1. **The stale `result/bin` nix default.** `RayTracingVoxelMapConfig` builds
   with `nix build` into the `result` symlink (module.py:29-31). `cargo build`
   does not touch `result`, and `_maybe_build` skips the build when the exe
   exists (native_module.py:438), so you can edit rust, rebuild with cargo,
   and still run last week's binary. Use the mls pattern
   (`target/release/<bin>` + `cargo build --release`) for all four new
   modules. (`result` and `target` are also both pruned from registry
   discovery — discovery.py:35-43 — so manifests inside them never register.)
2. **`--emit-config` emits python class *defaults*, not blueprint values.**
   `default_config` instantiates `config_type()` (cli.py:34-44). Anything
   tuned in a blueprint — `mount_rotation` above all — is silently absent
   from an emitted host config. Treat the robot's stdin JSON as a reviewed
   deployment artifact. (OPEN: blueprint-as-arg, per the deployment plan.)
3. **Every config field, every time.** A missing field is a startup error even
   if it "has a default" — rust-side defaults are forbidden by design (§5).
   Corollary: adding a field to the rust struct without adding it to the
   python wrapper breaks startup with `missing ["x"]`; that error is the
   feature.
4. **`Option`/`None` never crosses.** `to_config_dict` drops `None`s;
   `native_config` rejects `Option`. Resolve unions python-side.
5. **`threads = 2` when a worker uses `block_in_place`**, or the baked host's
   dispatch loop for that module deadlocks — standalone runs hide this
   because `#[tokio::main]` defaults to a full multi-thread runtime
   (mls Cargo.toml:20-22 comment).
6. **`&mut self` handlers serialize.** Long work in a handler backs up *all*
   inputs of that module (the raycaster's odometry-behind-lidar failure in
   the deployment plan). Fixed-rate control/replan loops belong in a spawned
   worker (`setup =` hook), never in a handler.
7. **Staleness is measured from arrival, never `msg.ts`.** The producer's
   clock is not the robot's. Rust `Input`s carry no arrival time — record
   `std::time::Instant::now()` inside the handler.
8. **`[lib] name` must match the `#[pymodule]` fn name** (both exemplar
   Cargo.tomls say so at line 9-10), and the crate needs
   `crate-type = ["cdylib", "rlib"]` — cdylib for maturin/pyo3, rlib for the
   bake host link. Missing rlib fails only at bake time.
9. **Copy `.cargo/config.toml` from the mls crate** — plain `cargo build` of a
   pyo3 cdylib on macOS needs `-undefined dynamic_lookup` (maturin injects it,
   NativeModule's `cargo build` does not).
10. **stdin is one line, once.** A process started by hand without piping the
    blob blocks forever in `read_line`, looking hung.
    `printf '%s\n' "$(cat cfg.json)" | DIMOS_TRANSPORT=zenoh ./bin` is the
    incantation.
11. **The package half of the msg type is unchecked** (§3) — the macro
    compares only `PointCloud2`, the wire key uses `sensor_msgs.PointCloud2`.
    Verify against the python message's `msg_name`.
12. **One instance per host.** `select_modules` refuses duplicate ids
    (discovery.py:163-167). Known limit; not blocking for this stack.

---

# Part 2 — the four briefs

Shared context for all four: message types come from the `lcm-msgs` crate
(git dep, branch `rust-codegen` — `geometry_msgs::Twist`,
`geometry_msgs::PointStamped`, `std_msgs::Bool`, `nav_msgs::{Odometry, Path}`
all exist). Dependencies for the module crates mirror the mls crate's list
(dimos-module, lcm-msgs, tokio, serde, tracing, tracing-subscriber,
validator, + pyo3/numpy optional behind `python`). The two pure crates stay
pure: **do not** add dimos-module/lcm-msgs/tokio to `dimos-motion2-target` or
`dimos-motion2-tc`; the adapter crate depends on them
(`default-features = false`) instead. `stamps.rs` (wire dialect,
encode+decode) and `clearance.rs` (`path_clearance`) already exist in
`dimos-motion2-tc` and are the shared facilities both adapters call.

The reference wiring these modules must slot into is `go2_zenoh_motion`
(robot/unitree/go2/zenoh/blueprints.py): both planner and follower read
`odometry` straight and resolve it into `base_link` off tf; MLS's `path` is
remapped to `planner_path`; the follower track is `hinted`.

## Brief 1 — `motion_planner`

- **Crate**: `dimos/navigation/motion/adapter/rust/` (new; crate name
  `dimos-motion-adapter`, lib `dimos_motion_adapter`). Module id
  `motion_planner`, `[[bin]] name = "motion_planner"`. Depends on
  `dimos-motion2-target` and `dimos-motion2-tc` (both
  `default-features = false`, path deps `../../planner/rust` and
  `../../control/rust`).
- **Python reference**: `dimos/navigation/motion/adapter/planner.py`
  (`MotionPlanner` / `MotionPlannerConfig`). The wrapper class to write:
  `MotionPlannerNative(NativeModule)` in
  `dimos/navigation/motion/adapter/planner_native.py`, metadata
  `python = "dimos.navigation.motion.adapter.planner_native:MotionPlannerNative"`.
- **Ports** (field names are port names):

  | dir | port | type |
  |---|---|---|
  | in | `local_map` | `sensor_msgs.PointCloud2` |
  | in | `odometry` | `nav_msgs.Odometry` |
  | in | `planner_path` | `nav_msgs.Path` |
  | out | `path` | `nav_msgs.Path` |
  | out | `plan_body` | `nav_msgs.Path` |

- **Config** (source of truth `MotionPlannerConfig`, planner.py:101-121):

  | field | default | notes |
  |---|---|---|
  | `embodiment` | `"go2"` | rust validates against known tags and uses `Emb::go2()`; dims live in the pure crate, parity-locked to `scenarios.py` |
  | `resolution` | `0.1` | python takes it from `AvoidanceConfig().resolution` (geometry.py:135); crosses explicitly |
  | `replan_hz` | `5.0` | |
  | `goal_lookahead_m` | `5.0` | |
  | `world_frame` | `"odom"` | |
  | `base_frame` | `"base_link"` | tf resolves the sensor-stamped odometry into it |
  | `replan_on_change` | `True` | plan on a new local map or a moved global route, not on every tick of the clock |
  | `floor_anchor` | `True` | re-zero the cloud on the floor under the robot before planning (`adapter/floor.py`) |
  | `lidar_height` | `0.0` | lidar height above ground; with tf it gives the floor prior, without which anchoring stays off |
  | `ground_margin_m` | `0.16` | drop returns this close to the estimated floor, so the ground slab cannot wall the robot in |
  | `cloud_z_offset` | `0.0` | manual trim on the map's z origin, applied at extraction |
  | `max_map_age_s` | `5.0` | the staleness guard |
  | `viz_publish_hz` | `2.0` | rate-cap for `plan_body` |

  The python `planner: str` registry field does **not** cross — the deployed
  module *is* the rust target planner; a wrapper that wants a different
  planner is not this module.
- **What to port, from where**:
  - The worker-loop shape from `mls_planner/rust/src/module.rs`: handlers
    only store latest cloud (+ `Instant::now()` arrival), latest pose
    `(x, y, yaw)` (yaw = euler z of the odometry quaternion, matching
    planner.py:179), latest global path xy; a spawned worker ticks at
    `replan_hz` (`tokio::time::interval`, not input-driven).
  - `carrot_along` (planner.py:83-98) — arc-walk from nearest waypoint,
    clamped to path end. Empty MLS path means no carrot, so hold the last
    plan (planner.py:181-187).
  - **The staleness guard** (planner.py:210-239): if pose exists and
    `local_map` arrival age > `max_map_age_s`, publish the **single-pose hold
    stub** — a one-pose `Path` at the current pose, `world_frame` — with an
    edge-triggered warn on entry and an info on recovery. Arrival time, not
    `msg.ts`.
  - **The refusal path**: `dimos_motion2_target::planner::plan(points, pose,
    goal, &emb, resolution)` returning `None`/empty gives the same single-pose
    stub (target.py:106-108 `RustTargetEpisode.plan` is the spec — a refusal
    comes out *as the planner made it*, and the follower reads one pose as
    "hold").
  - **Annotation**: on success, compute
    `dimos_motion2_tc::clearance::path_clearance(&xy, &cloud_pts,
    emb.width / 2.0)` then `dimos_motion2_tc::stamps::encode_precision(&path,
    &clearance, t0 = now)` and write the stamps into the poses'
    `header.stamp` (sec/nsec from the f64). This mirrors `annotate`
    (planner.py:60-64); the encoder constants are fixed wire constants
    (stamps.rs:36-48 explains why they must not follow config).
  - `plan_body`: re-publish the same plan, rate-capped at `viz_publish_hz`
    (planner.py:261-270).
- **What to test** (rust unit tests in the crate): `carrot_along` cases from
  `adapter/test_planner.py`; stale-map gives a one-pose stub and an
  edge-triggered transition; empty `planner_path` publishes no plan; refusal
  gives a stub at the current pose; stamps present and monotone on a planned
  path.
- **What NOT to do**: no planner logic in the module crate (call `plan()`);
  no `StallReporter` port — `warn_throttled!` lines are enough; do not stamp
  with per-robot config; do not measure staleness from `msg.ts`; do not give
  the module a `goal_pose` input — the carrot comes from `planner_path` only.

## Brief 2 — `trajectory_follower`

- **Crate**: same crate as Brief 1. Module id `trajectory_follower`,
  `[[bin]] name = "trajectory_follower"`.
- **Python reference**: `dimos/navigation/motion/adapter/follower.py`
  (`TrajectoryFollower` / `TrajectoryFollowerConfig`). Wrapper:
  `TrajectoryFollowerNative` in
  `dimos/navigation/motion/adapter/follower_native.py`.
- **Ports**:

  | dir | port | type |
  |---|---|---|
  | in | `path` | `nav_msgs.Path` |
  | in | `odometry` | `nav_msgs.Odometry` |
  | in | `local_map` | `sensor_msgs.PointCloud2` |
  | in | `stop_movement` | `std_msgs.Bool` |
  | out | `nav_cmd_vel` | `geometry_msgs.Twist` |
  | out | `goal_reached` | `std_msgs.Bool` |

- **Config** (source of truth `TrajectoryFollowerConfig`, follower.py:109-131,
  and `ControllerConfig`, control/controller.py:44-124):

  | field | default | notes |
  |---|---|---|
  | `track` | `"hinted"` | **names a TRACK, never a law** (control/tracks.py:15-33). Rust maps track to law internally: `"hinted"` uses `laws::hinted::Law` (stateful; `reset()` on start and on stop_movement), `"blind"` uses `laws::blind::update` (+ walk params + stamp decode). Validation rejects anything else. |
  | `controller_config` | nested object | mirror of `ControllerConfig` minus `frame_id` (see cleanup below): the 11 `law_params`, the 3 `walk_params`, the 6 `hinted_params` — flat inside the nested struct, `deny_unknown_fields`, `#[validate(nested)]`. Converted into `geom::Params` / `HintedParams` / `BlindParams`. |
  | `control_frequency` | `10.0` | |
  | `goal_tolerance` | `0.20` | |
  | `embodiment` | `"go2"` | half-width = `Emb` width / 2 from the pure crate — same one-place-for-body-dims policy the python states (follower.py:117-124). The python `half_width: float \| None` override does not cross (Option ban); drop it from the native wrapper. |
  | `floor_anchor` | `True` | re-zero the local map on the floor under the robot before the room hint is measured off it (`adapter/floor.py`) — the same band the planner planned in |
  | `lidar_height` | `0.0` | lidar height above ground; with tf it gives the floor prior, without which anchoring stays off |
  | `ground_margin_m` | `0.16` | drop returns this close to the estimated floor, so the ground slab is not read as a wall |
  | `idle_speed` | `0.02` | for the stall log classification only |
  | `max_path_age_s` | `1.0` | **NEW — the guards-table todo.** Chosen default: 1.0 s = five missed 5 Hz replans; the planner is co-located so this guards planner death, not the link. Marked here as a picked default, tune on robot. |

  **Python-side cleanup that lands with this brief**: delete
  `ControllerConfig.frame_id` and the tf claim in the `controller.py`
  docstring — the deployment plan says port no lies. `law_params` stops
  skipping it.
- **What to port**:
  - Control loop at `control_frequency` in a spawned worker (same shape as
    Brief 1); handlers store pose, path (+arrival `Instant`), cloud.
  - `GoalLatch` (follower.py:81-106) verbatim: goal = last pose of any path
    with two or more poses (a single-pose stub is a refusal, **never** an
    arrival target — follower.py:189-195); fire `goal_reached` exactly once;
    zero twist while latched.
  - `stop_movement` true clears the path, publishes a zero twist, resets the
    law (follower.py:205-209).
  - **The staleness guard (todo in the deployment plan's table)**: at tick,
    if the held path's *arrival* age > `max_path_age_s`, treat as no-path:
    publish zero twist, edge-triggered warn. This is the follower's own
    deadman — the planner's hold stub covers a stale map, this covers a dead
    planner.
  - Tick: path to `[[x, y, yaw]]` rows (`path_xy_yaw`, controller.py:174-180),
    pose to `(x, y, yaw)`, `t` = monotonic seconds, then
    hinted: `Law::step(pose, &path, clearance, &cfg, t)`;
    blind: `update(pose, &path, None, Some(&ts), &cfg)` where `ts` are the
    poses' stamps as f64 — the blind law decodes the dialect itself
    (blind.rs:158-168).
  - **Clearance (hinted track only)**: recompute
    `dimos_motion2_tc::clearance::path_clearance(&wp, &cloud, half_width)`
    per `(path, cloud)` pair, cached by identity/arrival-counter like
    follower.py:261-277. With no cloud, pass `None` clearance and hand the law
    the stamps instead — the hinted law's governor falls back the same way
    the python fallback (`decode_ceilings` then `ceilings_to_clearance`) does;
    if the hinted rust law's signature turns out not to take `ts` (check
    `laws/hinted.rs:222` — it takes `clearance: Option<&[f64]>` only), port
    `ceilings_to_clearance` (profile.py:127) as a tiny helper *in the module
    crate* and convert. **OPEN**: whether that helper should instead live in
    `stamps.rs` next to `decode_ceilings`; do not block on it.
  - Stall classification logs (single-pose stub vs law-commands-zero,
    follower.py:243-259) as throttled logs.
- **What to test**: `GoalLatch` cases from `adapter/test_follower.py`
  (fires once, small goal moves don't re-arm); single-pose stub gives zero
  twist and no goal latch; `stop_movement` gives zero + law reset; stale path
  gives zero; track dispatch (hinted stateful reset semantics: fresh vs reset
  `Law` answer identically — hinted.rs:214-219 states the contract).
- **What NOT to do**: never name a law in config or code outside the
  track-to-law map; no law math in the module crate (the laws are
  parity-locked — any behavior change lands python-first); do not publish
  `stop_movement` (input only); do not re-add `frame_id`/tf; do not chase the
  stamps as a schedule (stamps.rs:138-147 says why).

## Brief 3 — `cmd_vel_mux`

- **Crate**: `dimos/navigation/movement_manager/rust/` (new; crate name
  `dimos-cmd-vel-mux`). Module id `cmd_vel_mux`,
  `[[bin]] name = "cmd_vel_mux"`. Depends only on dimos-module + lcm-msgs +
  the standard list — no pure crates.
- **Python reference**:
  `dimos/navigation/movement_manager/movement_manager.py`, and the deployment
  plan's "Splitting MovementManager" section. The seam runs through
  `_on_teleop` (movement_manager.py:122-142): it preempts nav (robot side)
  *and* cancels the goal (laptop side). Both halves subscribe `tele_cmd_vel`.
- **Ports (rust half)**:

  | dir | port | type |
  |---|---|---|
  | in | `nav_cmd_vel` | `geometry_msgs.Twist` |
  | in | `tele_cmd_vel` | `geometry_msgs.Twist` |
  | out | `cmd_vel` | `geometry_msgs.Twist` |
  | out | `stop_movement` | `std_msgs.Bool` |

  `stop_movement` stays here because its only consumer (the follower) is on
  the robot. The NaN goal cancel does **not** move: it stays in the python
  half, and nothing routes it back from rust.
- **Config** (source of truth `MovementManagerConfig`,
  movement_manager.py:46-48):

  | field | default | notes |
  |---|---|---|
  | `tele_cooldown_sec` | `1.0` | |
  | `tele_scale_linear` | `[1.0, 1.0, 1.0]` | the python `Twist`-typed scaling flattened to two arrays — a Twist is not a config scalar |
  | `tele_scale_angular` | `[1.0, 1.0, 1.0]` | |
  | `nav_stale_s` | `0.5` | **NEW — the guards-table todo.** Chosen default: 0.5 s = five missed 10 Hz follower ticks. Picked, tune on robot. |

- **What to port / behavior**:
  - `_on_teleop`: set teleop-active + timestamp, publish `stop_movement=true`,
    publish the scaled twist on `cmd_vel`.
  - `_on_nav`: if teleop active and cooldown not expired, drop; else clear the
    flag and forward unmodified. Record arrival `Instant`.
  - **The nav staleness watchdog**: a spawned ticker (10 Hz). Armed after the
    first `nav_cmd_vel` arrival; while nav arrival age > `nav_stale_s` and
    teleop is not within cooldown, publish a zero `cmd_vel` every tick.
    Continuous rather than edge-only, chosen deliberately: a single zero can
    be lost, and a stream of zeros is exactly what a deadman is — the link
    used to provide this for free and this watchdog is its replacement. Note
    the behavior change: `cmd_vel` now carries zeros while nav is idle.
  - **Python half (same brief, python work)**: strip `MovementManager` to the
    click relay — keep `clicked_point`/`tele_cmd_vel` in, `goal`/`way_point`
    out, `_on_click` bounds checks, and `_cancel_goal`'s NaN goal on teleop;
    delete `nav_cmd_vel`/`cmd_vel`/`stop_movement` from it. Write the
    `CmdVelMuxNative` wrapper beside it. Update
    `movement_manager/test_movement_manager.py` for the split.
- **What to test** (mux logic as a pure struct with injected clock):
  teleop preempts nav; cooldown expiry restores nav; scaling applied to teleop
  only; `stop_movement` fires on every teleop edge; watchdog zeros on stale
  nav, suppressed during teleop cooldown, disarmed before first nav; python
  half still emits the NaN cancel.
- **What NOT to do**: no NaN goal from rust; do not consume `stop_movement`;
  do not scale nav; do not "helpfully" forward teleop when nav is stale beyond
  what the cooldown logic already does.

## Brief 4 — `odom_body_frame` (SUPERSEDED, see the note at the top)

- **Crate**: the existing `dimos/navigation/nav_3d/mls_planner/rust/` — a
  **second module id** in its Cargo.toml (the table is keyed by id; discovery
  handles many per manifest), NOT a new crate. Add
  `src/odom_body_frame.rs`, `pub mod odom_body_frame;` in lib.rs, a second
  `[[bin]] name = "odom_body_frame"` shim, and:

  ```toml
  [package.metadata.dimos.module.odom_body_frame]
  path = "dimos_mls_planner::odom_body_frame::OdomBodyFrame"
  python = "dimos.navigation.nav_3d.mls_planner.odom_body_frame:OdomBodyFrameNative"
  threads = 1

  [package.metadata.dimos.module.odom_body_frame.inputs]
  odometry = "nav_msgs.Odometry"

  [package.metadata.dimos.module.odom_body_frame.outputs]
  body_odometry = "nav_msgs.Odometry"
  ```

- **Python reference**:
  `dimos/navigation/nav_3d/mls_planner/odom_body_frame.py` (~11 lines of math:
  `leveled = msg.orientation * mount_inv`, position and twist pass through,
  `child_frame_id` replaced). Wrapper `OdomBodyFrameNative` +
  `OdomBodyFrameNativeConfig` go in the same file.
- **Config**:

  | field | default | notes |
  |---|---|---|
  | `mount_rotation` | `[0.0, 0.0, 0.0, 1.0]` (xyzw) | identity default is the deployment plan's named `--emit-config` trap: the real value is `_mount_rotation()` in blueprints.py:64-73 (from `MID360_MOUNT_RPY_DEG = (-60, 0, -90)`). A robot host must get the blueprint value via the hand-written stdin JSON — this module is the test case. |
  | `body_frame_id` | `"base_link"` | |

- **What to port**: precompute the inverse quaternion once; per message,
  Hamilton-product it. **Match the dimos convention exactly**:
  `Quaternion.__mul__` is `q1 * q2` = rotate by q2 first, then q1
  (msgs/geometry_msgs/Quaternion.py:220-232), and the python does
  `msg.orientation * mount_inv` — i.e. the leveled quaternion is orientation
  composed with the *inverse mount on the right*. Copy the four product lines
  rather than reaching for a quaternion crate whose convention you would then
  have to verify. Pass pose covariance, twist (and its covariance),
  header/stamp through untouched; only `child_frame_id` and the orientation
  change.
- **What to test**: mirror `test_odom_body_frame.py`'s vectors; identity mount
  gives passthrough except `child_frame_id`; a known rpy mount gives a leveled
  yaw matching the python to 1e-12; twist/position untouched.
- **What NOT to do**: no tf, no subscriptions beyond `odometry`, no new crate,
  no worker task (a pure per-message map belongs in the handler), and do not
  normalize/renormalize beyond what the python does.

## Parallelism between the four

| pair | parallel? | why |
|---|---|---|
| motion_planner x trajectory_follower | **NO** | same crate: shared Cargo.toml, lib.rs, Cargo.lock. Sequence them or give both to one implementer. |
| motion_planner x cmd_vel_mux | yes | disjoint crates and python files |
| motion_planner x odom_body_frame | yes | disjoint (adapter crate vs mls crate) |
| trajectory_follower x cmd_vel_mux | yes | disjoint — note both touch the `stop_movement`/`nav_cmd_vel` *contract*, but no shared files |
| trajectory_follower x odom_body_frame | yes | disjoint |
| cmd_vel_mux x odom_body_frame | yes | disjoint |

The only shared-file risk outside the adapter crate is `blueprints.py` when
the wrappers eventually get wired in; keep blueprint edits out of these four
briefs (a fifth, later task).
