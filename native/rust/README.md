# DimOS Rust module SDK

Two crates:

- **`dimos-module`**: runtime. `Module` trait, `Builder`, `Input`/`Output`, `Transport`/`LcmTransport`, `run()`.
- **`dimos-module-macros`**: `#[derive(Module)]` proc-macro.

## Writing a module

```rust
use dimos_module::{run, Input, LcmTransport, Module, Output};
use lcm_msgs::geometry_msgs::Twist;
use serde::Deserialize;

#[derive(Debug, Deserialize, Default)]
struct MyConfig { threshold: f64 }

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
    let transport = LcmTransport::new().await.unwrap();
    run::<MyModule, _>(transport).await.unwrap();
}
```

## Attributes

- `#[derive(Module)]`: on the struct. Required.
- `#[module(setup = fn, teardown = fn)]`: on the struct. Both optional. Names methods on `Self`. `setup` runs once before the input dispatch loop starts (use it to spawn background tasks or initialize resources); `teardown` runs once after the loop exits (use it for cleanup).
- `#[input(decode = fn, handler = fn)]`: on a field of type `Input<T>`. `decode` is required; `handler` defaults to `handle_<field_name>`.
- `#[output(encode = fn)]`: on a field of type `Output<T>`. `encode` is required.
- `#[config]`: on one field of any `Deserialize` type. At most one per struct. If absent, `Config = ()`.
- Unattributed fields are initialized via `Default::default()` and treated as module state.

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

`builder.input` registers a route from the resolved topic into an mpsc channel that backs `Input<T>`. `builder.output` hands back an `Output<T>` carrying a sender into the shared publish channel.

## Lifecycle inside `run()`

1. Read one JSON line from stdin, parse into `(topics, config)`.
2. `M::build(&mut builder, config)`: macro-generated, populates each field.
3. Spawn two tokio tasks: one drives `transport.recv()` and dispatches to input channels; one drains the publish channel into `transport.publish()`. The two run independently so a slow publish can't block recv.
4. `module.setup().await`.
5. `module.handle().await`, racing ctrl-c.
6. `module.teardown().await`.
