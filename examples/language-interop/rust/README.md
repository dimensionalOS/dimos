# Rust Robot Control Example

Subscribes to `/odom` and publishes velocity commands to `/cmd_vel` via LCM UDP multicast.

## Build & Run

```bash
cargo run
```

## Dependencies

- [Rust toolchain](https://rustup.rs/)
- Message types fetched automatically from [dimos-lcm](https://github.com/dimensionalOS/dimos-lcm) (`rust-codegen` branch)
