# DimOS web

The browser side of DimOS as one Deno workspace: the relay, the cockpit and the web SDK. The
documentation lives in [`docs/web/`](../docs/web/index.md):

- [Web](../docs/web/index.md): what the pieces are and how to run them.
- [Development](../docs/web/development.md): tasks, dev servers, builds and tests.
- [Wire protocol](../docs/web/protocol.md): framing, messages, and the WebTransport workarounds this
  code depends on.

| Directory   | What it is                                               |
| ----------- | -------------------------------------------------------- |
| `shared/`   | the wire protocol and the manifest, with golden fixtures |
| `relay/`    | the WebTransport relay (Deno)                            |
| `sdk/`      | `@dimos/sdk`, the viewer library the cockpit is built on |
| `cockpit/`  | the cockpit app (Vite + React)                           |
| `examples/` | zero-build SDK pages                                     |

The Python side (the bridge module, the protocol mirror, the authoring API) is under `dimos/web/`.

```bash
deno task dev            # relay on http://127.0.0.1:7780
deno task test           # relay + shared tests
deno task check          # type-check relay + shared
```
