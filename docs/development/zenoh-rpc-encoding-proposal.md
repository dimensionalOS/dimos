# CBOR RPC proposal

## Proposed approach

Keep Zenoh as the transport and encode each RPC message with CBOR, using JSON-RPC-style requests, responses, and errors:

- Encode ordinary values directly with CBOR.
- Wrap registered LCM messages in an LCM tag.
- Wrap other picklable Python objects in a pickle tag.
- Validate the message with CDDL before restoring tagged objects.

## Message contract

Every message has `rpc: "dimos-cbor/1"`. This identifies a DimOS CBOR protocol; standard JSON-RPC clients expect JSON and need an adapter.

| Message      | Other required fields    | Behavior                                            |
| ------------ | ------------------------ | --------------------------------------------------- |
| Request      | `id`, `method`, `params` | Execute a call and return one response              |
| Notification | `method`, `params`       | Execute without a response; omit `id`               |
| Success      | `id`, `result`           | Echo the request ID; result may be null             |
| Failure      | `id`, `error`            | Echo the ID, or use null if it cannot be determined |

`params` contains `args` (array) and `kwargs` (string-keyed map), even when empty. The server calls `f(*args, **kwargs)`. Request IDs are unique among pending calls. Responses contain exactly one of `result` and `error`; errors contain `code`, `message`, and optional `data`. The CDDL files define field limits and exclude extra envelope fields and batch messages.

Use `dimos/cbor-rpc/v1/<module-instance>/<method>` as the Zenoh key, with one Queryable per method and `application/cbor` payload encoding. The body method is `<module-instance>/<method>` and must match the key. Replies use the same Zenoh query.

## Value encoding

| Python value                                                                      | Wire representation                                             |
| --------------------------------------------------------------------------------- | --------------------------------------------------------------- |
| Exact `None`, `bool`, `float`, `str`, `bytes`, and `int` in `[-2**64, 2**64 - 1]` | Native CBOR                                                     |
| Exact `list`, or exact `dict` with string keys                                    | Native array/map; recursively encode members                    |
| Registered LCM type                                                               | Tag 60003 containing `[type_id, lcm_bytes]`                     |
| Other picklable objects                                                           | Tag 60004 containing pickle bytes (protocol 5, in-band buffers) |
| Unpicklable objects                                                               | Encoding error                                                  |

The tag numbers are provisional and need allocation before interoperable deployment. An LCM `type_id`, such as `geometry_msgs.Vector3`, selects an explicitly registered codec. Keep the complete LCM bytes, including the fingerprint. Unknown types and codec failures are errors; a failed LCM encoder must not silently fall back to pickle.

Use exact native types to avoid silently converting tuples or enums into lists or primitives. Tuples, enums, dataclasses, larger integers, and maps with non-string keys use pickle unless registered as LCM types. Pickle requires compatible Python classes and dependencies; only the native CBOR and portable LCM subset is suitable for non-Python peers.

The result is one encoding entry point and one CBOR payload. LCM and pickle still serialize their objects into bytes first. Preserving cycles or shared identity across separate values is outside v1.

## CDDL files and validation

| File                                                          | Responsibility                                    |
| ------------------------------------------------------------- | ------------------------------------------------- |
| [rpc.cddl](/docs/development/cbor-rpc/rpc.cddl)               | Requests, notifications, responses, and arguments |
| [values.cddl](/docs/development/cbor-rpc/values.cddl)         | Native values and allowed extensions              |
| [extensions.cddl](/docs/development/cbor-rpc/extensions.cddl) | LCM and pickle tag content                        |

Concatenate the files in this order to form one schema, starting at `rpc-message`. CDDL checks the envelope, value types, and tag shapes. It does not validate the contents of LCM/pickle bytes or Python method signatures.

Receive messages in this order:

1. Parse one CBOR message with size/depth limits, unique map keys, and definite-length containers; retain tags as data.
2. Validate CDDL, then check message direction, method/key consistency or pending response ID, and authorization.
3. Restore registered LCM values and explicitly permitted pickle values.
4. Bind and validate method arguments, execute the call, and encode the response using the same rules.

**Pickle is opt-in for trusted peers and methods only.** Unpickling can execute code, so do not call `pickle.loads()` in the initial CBOR tag hook. CDDL validation does not make pickle safe.

## Minimal example

Run [demo_cbor_rpc.py](/dimos/spec/demo_cbor_rpc.py) from the repository root:

```sh
uv run --no-sync --with cbor2==6.1.5 --with pycddl==0.6.4 python -m dimos.spec.demo_cbor_rpc
```

The example calls `calculate(Vector3(1, 2, 3), scale=2.0, label=Label("demo"))`: Vector3 uses LCM, scale uses CBOR, and Label uses pickle. Both request and response pass CDDL validation; the result is `Vector3(2, 4, 6)` with the original label and types restored.

It trusts its own local messages and supports one method and one LCM type. `pycddl` validates in-process; `cbor2` then decodes the message while retaining extension tags. These dependencies do not change the project lockfile.

## Errors and rollout

Return protocol and method errors in normal Zenoh replies, using JSON-RPC-style error codes. Keep argument-binding failures separate from exceptions raised inside the method. Preserve Python exception details in `error.data` using the existing reconstruction policy. Notifications only log errors; transport failures remain separate.

1. Extract a shared codec with LCM registration, pickle policy, and validation.
2. Check real management payloads and LCM/exception round trips, including codecs that embed pickle or change values during decoding.
3. Integrate into ZenohRPC and verify startup, wiring, calls, and shutdown before switching defaults.
