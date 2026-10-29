# Zenoh RPC and the JSON-RPC Proposal

This proposal changes the **RPC communication layer used to call `@rpc` methods between DimOS modules**: `RPCClient / RpcCall → RPCSpec → ZenohRPC`. Module references injected through Specs use this layer to call remote methods. Module lifecycle operations and Coordinator management interfaces also use it. The first phase covers Python methods suitable for JSON.

Worker management through **Actor IPC**, the **MCP protocol** used by agents, and `In/Out/IO` data streams remain outside this scope.

Zenoh continues to listen for requests, route them by key, and transmit responses. DimOS handles JSON encoding, decoding, and JSON-RPC message validation using Python's standard `json` library and protocol models. Existing call syntax stays the same; objects that cannot be represented in JSON continue to use the existing protocol.

This document describes the proposed transport changes; runtime integration is not yet implemented.

## 1. Existing Zenoh RPC Flow

Consider `self.calc.compute1(2, 3)`, which returns `5`.

At startup, a module registers its `@rpc` methods with Zenoh. The Blueprint resolves the target module through its Spec and injects an `RPCClient` proxy into `self.calc`.

Existing RPC keys follow `dimos/rpc/<module-instance>/<method>`. For example, Calculator's methods are registered at `dimos/rpc/Calculator/compute1` and `dimos/rpc/Calculator/compute2`. The client queries the matching key, and the server replies on that key within the same query. There is no separate response topic.

| Step | What happens |
| --- | --- |
| 1. Access the method | `self.calc.compute1` returns a `RpcCall` object |
| 2. Invoke the call | `RpcCall` calls `call_sync()` with target `Calculator/compute1` and arguments `((2, 3), {})` |
| 3. Send the request | `ZenohRPC` pickles the arguments and sends them to `dimos/rpc/Calculator/compute1` through `session.get()` |
| 4. Execute remotely | The Zenoh Queryable receives the request; a worker thread decodes the arguments and runs `compute1(2, 3)` |
| 5. Return the result | The server pickles `5` and sends it through `query.reply()` |
| 6. Complete the call | The client callback decodes the result and wakes `call_sync()`, returning `5` to the caller |

The current protocol has three key properties:

- The Zenoh key contains the method name; the request body contains only `(args, kwargs)`.
- `call_id` is local client bookkeeping for callbacks and is not sent with the request.
- Calls appear synchronous to the caller, while responses arrive through callbacks.

The existing protocol is **Zenoh + pickle RPC**, rather than JSON-RPC.

Related code: [client proxy](/dimos/core/rpc_client.py), [RPC interfaces](/dimos/protocol/rpc/spec.py), and [Zenoh backend](/dimos/protocol/rpc/zenohrpc.py).

## 2. JSON-RPC Proposal

Application code continues to call `self.calc.compute1(2, 3)`. JSON-RPC support is added at the RPC backend layer.

| Component | Responsibility |
| --- | --- |
| Existing `RPCClient` / `RpcCall` | Preserve the application call interface |
| Proposed `ZenohJsonRPC` | Implement `RPCSpec`, encode and decode JSON, validate messages and arguments, execute bound methods, and convert exceptions |
| Zenoh | Listen on method keys through Queryables, route requests to their callbacks, and transmit responses |

Methods are still registered with Zenoh through `session.declare_queryable()`. When a callback receives a request, it submits the work to a worker thread. DimOS then decodes and validates the request before executing the original module method.

Use a separate namespace, `dimos/jsonrpc/python-args/<module-instance>/<method>`, for the new protocol. Each method retains its own Zenoh key. The request body's `method` is checked against the key for consistency; it does not introduce another dispatch layer.

| Protocol | Calculator example key | Method selection |
| --- | --- | --- |
| Existing pickle RPC | `dimos/rpc/Calculator/compute1` | Zenoh routes by key to the method callback |
| Proposed JSON-RPC | `dimos/jsonrpc/python-args/Calculator/compute1` | Zenoh routes by key; the callback validates and executes the call |

The new request for the same calculation contains:

| Field | Value |
| --- | --- |
| `jsonrpc` | `"2.0"` |
| `id` | `"client1:1"` |
| `method` | `"Calculator/compute1"` |
| `params` | `{"args": [2, 3], "kwargs": {}}` |

The successful response is `{"jsonrpc": "2.0", "id": "client1:1", "result": 5}`.

The ID `"client1:1"` is illustrative. In production, generate a UUID string with `uuid.uuid4().hex` for each ordinary call to avoid reusing IDs across clients or process restarts. JSON-RPC does not require UUIDs: distinct pending requests need distinct IDs, the server echoes the request ID in its response, and the client uses it to locate the waiting call. Notifications omit the ID.

The `args` and `kwargs` structure is a DimOS convention. After validating the message and arguments, the DimOS adapter invokes the bound method with `f(*args, **kwargs)`, preserving positional, keyword, and mixed Python calls. This fits JSON-RPC's named parameter format; clients in other languages must also follow this convention. See the [JSON-RPC 2.0 specification](https://www.jsonrpc.org/specification).

The new flow is: **existing call interface → JSON-RPC encoding → Zenoh routing by key → DimOS validation and method execution → JSON-RPC response through Zenoh**. The client validates the response structure and request ID before returning the result or raising an exception.

## 3. Error Handling

When a method raises an exception, the existing `ZenohRPC` pickles its type, arguments, and traceback, then returns it through `query.reply_err()` with encoding `dimos/rpc-exception`. The client reconstructs the exception and passes it to `call_sync()` to raise. Built-in exception types are restored where possible; custom exceptions become `RemoteError`, preserving the remote traceback.

In the new protocol, the DimOS adapter converts parsing, validation, and execution failures into the JSON-RPC `error` field and returns them through an ordinary Zenoh `query.reply()`. A response contains either `result` or `error`, never both. For example:

`{"jsonrpc": "2.0", "id": "client1:1", "error": {"code": -32000, "message": "Arguments must be non-negative"}}`

| Error | Code | Meaning |
| --- | --- | --- |
| Parse error | `-32700` | The request is not valid JSON |
| Invalid request | `-32600` | The JSON-RPC request structure is invalid |
| Method not found | `-32601` | A handler received the request but determined that the target method is unavailable |
| Invalid params | `-32602` | `params` does not satisfy the interface requirements |
| Internal error | `-32603` | The RPC framework failed to process the request |
| Application exception | `-32000` (assigned by this proposal) | The original module method raised an exception |

See the [JSON-RPC error object](https://www.jsonrpc.org/specification#error_object) for standard definitions. To preserve Python behavior, application exceptions should also carry `type_name`, `type_module`, `args`, and `traceback` in `error.data`. The client converts these details back into the existing exception representation. Exceptions such as `TypeError` raised inside a method must not automatically become protocol parameter errors. Methods whose exceptions cannot be encoded losslessly continue to use the existing protocol.

With one Queryable per method key, a key with no Queryable has no handler to return `-32601`; the client instead times out. Timeouts still raise `TimeoutError`. They do not prove that the remote method did not execute and must not trigger an automatic resend. Calls that do not wait for a response, such as `stop()`, log exceptions on the server without returning an error to the client.

## 4. Compatibility

**Keep both protocols available and migrate methods individually.**

1. **Keep the existing protocol as the default.** Enable JSON-RPC only when both sides support it and the method's arguments, results, and exceptions can be converted losslessly. Initially, configure these methods explicitly instead of adding automatic negotiation.
2. **Keep complex objects on the existing path.** `set_module_ref()` passes module proxies, `set_transport()` passes Transport objects, and `get_skills()` returns `SkillInfo` instances. Define explicit conversions for these types later.
3. **Preserve call behavior.** Keep synchronous, callback, and asynchronous interfaces, timeout settings, and exception types compatible. `stop()` remains a call that does not wait for a response. Reuse existing async `@rpc` wrappers and preserve protocol selection when proxies cross worker boundaries.
4. **Choose the protocol before sending.** Once a JSON request has been sent, a timeout or response decoding failure must not cause a second invocation through the existing protocol. This prevents duplicate actions.
5. **Preserve existing Rust/C++ integration.** Keep `NativeModule` process startup, stdin configuration, typed stream topics, and message encoding intact. RPC interfaces following the same JSON contract can be added to Rust/C++ later.

Relevant code: [module interfaces](/dimos/core/module.py), [exception reconstruction](/dimos/protocol/rpc/rpc_utils.py), and [NativeModule](/dimos/core/native_module.py). Actor process management and attribute access continue to use the existing IPC path.

Validate JSON-RPC messages, Zenoh method routing, and DimOS argument binding and exception reconstruction separately.

Acceptance criteria: **existing code runs without changes, migrated methods preserve result types and call behavior, and existing Rust/C++ data streams continue to work.**
