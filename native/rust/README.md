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

    #[config]
    config: MyConfig,
}

impl MyModule {
    // initialization or publisher setup
    async fn on_start(&mut self) { /* ... */ }

    // processing function expected by cmd: Input
    async fn handle_cmd(&mut self, msg: Twist) { /* ... */ }

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
- `#[output(encode = fn)]`: on a field of type `Output<T>`. `encode` is required.
- `#[config]`: on one field. The type must be defined with `#[native_config]` (see [Config](#config)). At most one per struct. If absent, `Config` defaults to `dimos_module::NoConfig`.
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
