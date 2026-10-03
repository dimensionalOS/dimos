# DimOS raw LCM transport

This crate publishes and receives opaque bytes over LCM UDP multicast. It has no
message definitions, fingerprints, generated message crate, or CDR dependency.
Typed consumers choose their own codec above this transport.

The initial implementation was extracted from Dimensional's `dimos-lcm` repository,
`tools/rust/lcm`, commit `04d78e8622500244123ba9cefa4c51b4cb454549`, matching the
existing workspace lockfile. That crate declares Apache-2.0 in its Cargo manifest.
The source is maintained here so adding a message no longer depends on that
repository. Original transport behavior and its unit tests are retained; local
changes are reviewed in the DimOS history.

Source: https://github.com/dimensionalOS/dimos-lcm/tree/04d78e8622500244123ba9cefa4c51b4cb454549/tools/rust/lcm

Run the raw transport checks with:

```bash
cargo test -p dimos-lcm-transport
cargo build -p dimos-lcm-transport --example interop
.venv/bin/python examples/message-codegen/demo_transport.py \
  --executable target/debug/examples/interop
```

The demo chooses a local UDP port, starts the native receiver, waits for its ready
signal, and exchanges both short and fragmented binary payloads with Python's LCM
binding. It terminates and reaps the native process even on failure.
