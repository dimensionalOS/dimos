# Native RPC toy

Python starts a Rust `ToyPlanner` through `NativeModule` and calls `plan(start, goal)`.
It returns the two points, not a collision-checked path. No robot needed.

```sh
uv run pytest examples/native_rpc/test_native_rpc.py --no-cov
```

`@native_rpc` declares Rust-owned methods. The stdin launch carries their names;
`start()` waits for `_ready` to confirm this launch and its methods. Python RPC stays on pickle.

JSON-RPC 2.0 request/reply over Zenoh only: named JSON parameters, no retries,
notifications, batches or typed messages. The Rust runner handles one call at a time;
streaming modules, concurrent handlers and bake are left for review. No authentication is added;
use a trusted network. Tests use loopback with discovery off.

Non-finite floats are rejected. Numbers outside serde_json's range return a parse error.
