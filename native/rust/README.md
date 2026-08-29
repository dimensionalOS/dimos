# DimOS Rust module SDK

Two crates:

- **`dimos-module`**: runtime. `Module` trait, `Builder`, `Input`/`Output`, `Transport` (`LcmTransport`/`ZenohTransport`), `run_with_transport()`.
- **`dimos-module-macros`**: `#[derive(Module)]` and `#[native_config]` proc-macros.

## Writing a module

```rust
use dimos_module::{native_config, run_with_transport, Input, Module, Output};
use lcm_msgs::geometry_msgs::Twist;

#[native_config]
struct MyConfig {
    #[validate(range(min = 0.0))]
    threshold: f64,
}

#[derive(Module)]
#[module(setup = on_start, teardown = on_stop)]
struct MyModule {
    #[input(decode = Twist::decode)]
    cmd: Input<Twist>,

    #[output(encode = Twist::encode)]
    out: Output<Twist>,

    #[io(decode = Twist::decode, encode = Twist::encode)]
    shared: Io<Twist>,

    #[config]
    config: MyConfig,
}

impl MyModule {
    // initialization or publisher setup
    async fn on_start(&mut self) { /* ... */ }

    // processing function expected by cmd: Input
    async fn handle_cmd(&mut self, msg: Twist) { /* ... */ }

    // processing function expected by shared: Io
    async fn handle_shared(&mut self, msg: Twist) { /* ... */ }

    // teardown / clean up logic
    async fn on_stop(&mut self) { /* ... */ }
}

#[tokio::main]
async fn main() {
    run_with_transport::<MyModule>().await;
}
```

## Transport

Every transport is compiled into the binary. `run_with_transport` opens the one named by the `DIMOS_TRANSPORT` env var (`lcm` or `zenoh`), which the Python coordinator sets from `global_config.transport` (the `--transport` flag). The module never names a transport, so the same binary runs over either.

## Attributes

- `#[derive(Module)]`: on the struct. Required.
- `#[module(setup = fn, teardown = fn)]`: on the struct. Both optional. Names methods on `Self`. `setup` runs once before the input dispatch loop starts (use it to spawn background tasks or initialize resources); `teardown` runs once after the loop exits (use it for cleanup).
- `#[input(decode = fn, handler = fn)]`: on a field of type `Input<T>`. `decode` is required; `handler` defaults to `handle_<field_name>`.
- `#[input_group(decode = fn, handler = fn)]`: on a field of type `InputGroup<T>`, one port fed by several topics of the same message type (see [Stream groups](#stream-groups)). `decode` is required; `handler` defaults to `handle_<field_name>` and takes `(index, msg)`.
- `#[output(encode = fn)]`: on a field of type `Output<T>`. `encode` is required.
- `#[io(decode = fn, encode = fn, handler = fn)]`: on a field of type `Io<T>`, a port that publishes to and subscribes on one topic. `decode` and `encode` are required; `handler` defaults to `handle_<field_name>`. The transports deliver a message back to its own sender, so the handler also sees what the module publishes. Use `#[output]` instead when the module only publishes.
- `#[config]`: on one field. The type must be defined with `#[native_config]` (see [Config](#config)). At most one per struct. If absent, `Config` defaults to `dimos_module::NoConfig`.
- `#[tf]`: on a field of type `Tf`. Subscribes to the `tf` topic, answers transform queries, and publishes transforms (see [Transforms](#transforms)). No arguments.
- Unattributed fields are initialized via `Default::default()` and treated as module state.

## Config

A config struct is defined with `#[native_config]`. The attribute enforces a one-to-one mapping with the Python wrapper: every field is required and supplied by Python over stdin, with no Rust-side defaults.

It injects `#[derive(Debug, Deserialize, Serialize, Validate)]` and `#[serde(deny_unknown_fields)]`, emits the `NativeConfig` marker impl that `#[config]` requires, and rejects at compile time anything that would let a field be filled in by Rust:

- `Option<T>` fields
- `#[serde(default)]`, field or container
- `#[serde(skip)]`, `#[serde(skip_deserializing)]`, `#[serde(flatten)]`

A type alias to `Option` slips past the compile-time check, but the runtime check below still rejects it.

Field-level `#[validate(...)]` and a container `#[validate(schema(function = "..."))]` (from the [`validator`](https://docs.rs/validator) crate) pass through for value and cross-field validation. `run()` calls `config.validate()` after deserializing and bails with an `io::Error` on failure.

```rust
use dimos_module::native_config;
use validator::ValidationError;

#[native_config]
#[validate(schema(function = "validate_health_range"))]
struct Config {
    #[validate(range(exclusive_min = 0.0))]
    voxel_size: f32,
    #[validate(range(min = 1))]
    max_health: i32,
    min_health: i32,
}

fn validate_health_range(cfg: &Config) -> Result<(), ValidationError> {
    if cfg.min_health >= cfg.max_health {
        return Err(ValidationError::new("min_health_lt_max_health"));
    }
    Ok(())
}
```

At runtime `run()` enforces the mapping on the Python payload: deserialization rejects an unknown field, and a key-set check rejects any field whose JSON key is absent, even an `Option` or a type alias to `Option` that serde would otherwise accept as `None`.

Field name = port name. Ports map to topics via the stdin JSON; unmapped ports fall back to `/{port}`.

## Stream groups

A rig with N identical sensors would otherwise need N ports and N near-identical handlers. An `InputGroup<T>` is one port wired to a list of topics that all carry `T`, delivered to one handler in arrival order. Each message is tagged with the index of the topic it arrived on, so the handler can tell the sources apart.

```rust
#[derive(Module)]
struct MultiCam {
    #[input_group(decode = Image::decode)]
    cameras: InputGroup<Image>,
}

impl MultiCam {
    async fn handle_cameras(&mut self, index: usize, image: Image) {
        let topic = self.cameras.topic(index);
    }
}
```

The Python wrapper supplies the sources with `stream_groups`, keyed by port name:

```python
MultiCam.blueprint(
    stream_groups={"cameras": StreamGroup(names=["left_cam", "right_cam"], msg_type=Image)},
)
```

`names` are stream names, not backend topics — a leading `/` is rejected. Each entry becomes a synthetic `In` stream on the python side, so the blueprint machinery treats it like a declared port: autoconnect matches it against producers' `Out` streams, `.remappings()` and `.namespace()` rewrite it, and blueprint transport pins apply. The same blueprint runs unchanged over LCM or zenoh.

The group itself is not a port with a stream of its own: python hands the wired entries' channels to the native process, which subscribes them directly. On the launch line the port's value is an array rather than a string. A group configured with no names still claims its port but never yields.

`stream_groups` lives on `ModuleConfig`, so a plain Python `Module` takes the same field. There the module subscribes the group itself and dispatches to `async def handle_<port>(self, index, msg)`, matching the Rust handler signature:

```python
class MultiCam(Module):
    async def handle_cameras(self, index: int, image: Image) -> None: ...
```

The whole group shares one dispatcher, so the handler is never re-entered, and its mailbox holds the latest unprocessed message per stream rather than one slot for the group — a chatty camera cannot starve the others.

## Transforms

A `#[tf]` field gives a module a view of the transform graph, the Rust counterpart to Python's `tf.get()` and `tf.publish()`. It subscribes to the `tf` topic (mapped like any other port, default `/tf`), buffers each `parent -> child` edge it sees, and answers queries by composing transforms along the shortest path through the graph.

```rust
#[derive(Module)]
struct VoxelMap {
    #[input(decode = PointCloud2::decode)]
    lidar: Input<PointCloud2>,
    #[tf]
    tf: Tf,
}

impl VoxelMap {
    async fn handle_lidar(&mut self, cloud: PointCloud2) {
        // De-rotate a scan from the lidar's mount frame into the robot base frame.
        if let Some(t) = self.tf.get_latest("base_link", "mid360_link") {
            let point_in_base = t.rotation() * point_in_lidar + t.translation();
        }
    }
}
```

`Tf` is a cheap-to-clone handle; the graph fills in the background as `tf` messages arrive. `get_latest(parent, child)` is the common case. For a query against a particular stamp, `lookup(parent, child)` starts one that `.at(time)` points at the sample nearest that stamp and `.tolerance(secs)` bounds how far that sample may sit from it, finished with `.get()`:

```rust
let at_scan = self.tf.lookup("map", "base_link").at(scan_ts).tolerance(0.1).get();
```

A message and the transform it needs arrive on separate topics, so the transform for a given stamp is often merely late. `.within(duration)` replaces `.get()` to wait for one, returning as soon as the lookup succeeds or `None` at the deadline:

```rust
let at_scan = self.tf.lookup("odom", &cloud.header.frame_id)
    .at(scan_ts)
    .tolerance(0.02)
    .within(Duration::from_millis(200))
    .await;
```

`.tolerance()` and `.within()` are different clocks. Tolerance bounds how far the chosen sample may sit from `.at()` in message stamps — accuracy. `.within()` bounds how long to wait in wall time — patience. Always set a tolerance when waiting, or the lookup is satisfied by anything inside the buffer window and returns a stale transform immediately.

`.within()` suspends the caller, and awaiting it inside a `handle_*` method parks that module's whole dispatch loop, so every other topic it subscribes to stops being served until it returns. Prefer `.get()` there; move waiting onto its own task when the wait may be long.

Either way the result is `None` when no path connects the frames or no sample falls within the tolerance. It exposes its `nalgebra` parts via `translation()` (a `Vector3<f64>`) and `rotation()` (a `UnitQuaternion<f64>`). Lookups are nearest-in-time, not interpolated.

A result composed over several edges carries the stamp of the stalest edge on the path, so `ts` reads as the age of the whole answer rather than of one hop. A chain mixing a live edge with a static one is only as fresh as the live edge, in either direction.

`publish` sends transforms onto the same `tf` topic, the counterpart to Python's `tf.publish()`. Published transforms also feed the module's own graph, so a lookup right after the publish sees them. Build the isometry from `dimos_module::nalgebra`, re-exported so the version matches the SDK's types:

```rust
use dimos_module::nalgebra::Isometry3;
use dimos_module::Transform;

let iso = Isometry3::translation(0.5, 0.0, 0.0);
self.tf.publish(&[Transform::new("base_link", "gripper", ts, iso)]).await?;
```

## What `#[derive(Module)]` generates

Just for reference, in the example above the macro expands to:

```rust ignore
impl ::dimos_module::Module for MyModule {
    type Config = MyConfig;

    fn build(builder: &mut ::dimos_module::Builder, config: Self::Config) -> Self {
        Self {
            cmd: builder.input("cmd", Twist::decode),
            out: builder.output("out", Twist::encode),
            config,
        }
    }

    async fn setup(&mut self)    { self.on_start().await }
    async fn teardown(&mut self) { self.on_stop().await }

    async fn handle(&mut self) {
        loop {
            // run whichever input channel has available messages and run the handler function
            tokio::select! {
                Some(msg) = self.cmd.recv() => self.handle_cmd(msg).await,
                else => break,
            }
        }
    }
}
```

`builder.input` registers a route from the resolved topic into an mpsc channel that backs `Input<T>`. `builder.output` hands back an `Output<T>` carrying a sender into its own per-channel publish queue.

## Lifecycle inside `run()`

1. Read one JSON line from stdin, parse into topics, config, and per-channel publisher QoS.
2. `M::build(&mut builder, config)`: macro-generated, populates each field.
3. Subscribe each input channel on the transport (push callbacks into the input mpsc channels), and spawn one publish worker per output channel, each draining its queue into `transport.publish()`. Receive and publish run independently, and a stalled publish on one channel can't block the others.
4. `module.setup().await`.
5. `module.handle().await`, racing ctrl-c.
6. `module.teardown().await`.
