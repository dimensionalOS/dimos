# @dimos/topics

DimOS robot topics in the browser — a transport-agnostic client for subscribing to topics, teleoperating,
and calling `@rpc` commands over the internet.

```ts
import { createDimosClient, ws } from "@dimos/topics";

const dimos = createDimosClient({ transport: ws() }); // or webtransport() — WS control + WT data
await dimos.connect("ws://localhost:8080");
dimos.subscribe("/nav/pose", (m) => use(m.data, m.ts)); // one { data, ts, meta } envelope everywhere
await dimos.modules.ScopeNav.navigate_to(goal);         // typed @rpc call
```

Pass the generated maps for full type-safety — topic names + message types autocomplete, and
`modules.<Target>.<method>()` gets real arg/return types:

```ts
import type { DimosCommands, DimosTopics } from "./dimos.topics.gen.ts";
const dimos = createDimosClient<DimosTopics, DimosCommands>();
```

## Typed codegen — `scripts/gen_types.py`

Generates the `DimosTopics` + `DimosCommands` TypeScript **statically** from a dimos blueprint (a `Module`
subclass) — no gateway, no robot, no running bus. The import is side-effect-free (class-definition
reflection only; the blueprint is never instantiated). It's the only way to type RPC args/returns — the
wire only advertises `{target, method}`, never signatures.

```bash
deno task gen-types scenarios/nav.py                              # print to stdout
deno task gen-types scenarios/nav.py --out app/src/dimos.topics.gen.ts   # write the app's map
```

`deno task gen-types` = `uv run python packages/topics/scripts/gen_types.py`. Regenerate whenever a
blueprint's topics or `@rpc` methods change.

**What it reads:**

- **Topics** — the blueprint's module-level `PORTS = [(attr, topic, MsgClass), …]` list (the same list its
  `__main__` wires to transports). Each → `"<topic>": <pkg>.<Name>` from `MsgClass.msg_name`.
- **Commands** — each `@rpc` method declared on the `Module` subclass → a typed descriptor
  `"<method>": { args: [...]; ret: ... }` from its signature.

**Type mapping (`@rpc` args + returns):**

| Python | TS |
|---|---|
| `bool` | `boolean` |
| `int` / `float` | `number` |
| `str` | `string` |
| a dimos message class | `pkg.Name` (imported from `@dimos/msgs`) |
| `list[X]` | `X[]` |
| `Optional[X]` / `X \| None` | `X \| null` |
| `-> None` / unannotated | `void` |
| anything else | `unknown` |

Unknown target/method or a wrong-typed arg is a compile error.
