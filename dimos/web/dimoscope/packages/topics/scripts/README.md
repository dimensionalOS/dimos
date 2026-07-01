# gen_types.py — typed maps from a dimos blueprint

Generates the browser SDK's `DimosTopics` + `DimosCommands` TypeScript **statically** from a dimos
blueprint (a `Module` subclass) — no gateway, no robot, no running bus. The import is side-effect-free
(class-definition reflection only; the blueprint is never instantiated).

Why static (not live discovery): the blueprint is the source of truth, it works offline/CI, and it's the
**only** way to type RPC args/returns — the wire only advertises `{target, method}`, never signatures.

## Usage

```bash
# print to stdout
deno task gen-types scenarios/nav.py

# write the app's typed map
deno task gen-types scenarios/nav.py --out app/src/dimos.topics.gen.ts
```

`deno task gen-types` = `uv run python packages/topics/scripts/gen_types.py`. Regenerate whenever a
blueprint's topics or `@rpc` methods change.

## What it reads

- **Topics** — the blueprint's module-level `PORTS = [(attr, topic, MsgClass), …]` list (the same list its
  `__main__` wires to transports). Each → `"<topic>": <pkg>.<Name>` from `MsgClass.msg_name`.
- **Commands** — each `@rpc` method declared on the `Module` subclass → a typed descriptor
  `"<method>": { args: [...]; ret: ... }` from its signature.

## Type mapping (`@rpc` args + returns)

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

## Consuming the output

```ts
import { createDimosClient } from "@dimos/topics";
import type { DimosCommands, DimosTopics } from "./dimos.topics.gen.ts";

const dimos = createDimosClient<DimosTopics, DimosCommands>();
dimos.subscribe("/nav/pose", (m) => m.data);      // m.data: geometry_msgs.PoseStamped
await dimos.modules.ScopeNav.navigate_to(goal);   // goal: geometry_msgs.PoseStamped → Promise<boolean>
```

Topic names + message types autocomplete; unknown target/method or a wrong-typed arg is a compile error.
