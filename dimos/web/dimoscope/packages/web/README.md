# @dimos/web — Dimos JS

DimOS robot topics in the browser — a transport-agnostic client for subscribing to topics, teleoperating,
and calling `@rpc` commands over the internet.

```ts
import { createDimosClient, ws } from "@dimos/web";

const dimos = createDimosClient({ transport: ws() }); // or webtransport() — full-duplex QUIC, auto-falls back to WS
await dimos.connect("ws://localhost:8080");
dimos.subscribe("/nav/pose", (m) => use(m.data, m.ts)); // one { data, ts, meta } envelope everywhere
```

Untyped, that works — but every topic is `unknown` and any module/method is accepted. The type-safety
below makes the browser match the robot.

## Type-safety, end to end

Types are generated from the dimos blueprint — the Python `Module` that defines the robot's topics and
commands — so the browser types can't drift from what the robot publishes and accepts.

```
scenarios/nav.py  ──►  deno task gen-types  ──►  dimos.topics.gen.ts  ──►  createDimosClient<DimosTopics, DimosCommands>()
  (Out[] + @rpc)          (static, offline)         (DimosTopics/Commands)        (typed subscribe + typed modules)
```

### 1. Generate the maps from the blueprint (static — no gateway, no robot)

```python
# scenarios/nav.py — Out[] topics + @rpc commands are the source of truth
class ScopeNav(Module):
    pose:  Out[PoseStamped]
    cloud: Out[PointCloud2]

    @rpc
    def navigate_to(self, goal: PoseStamped) -> bool: ...

PORTS = [("pose", "/nav/pose", PoseStamped), ("cloud", "/nav/cloud", PointCloud2), ...]
```

```bash
deno task gen-types scenarios/nav.py --out app/src/dimos.topics.gen.ts
```

emits `dimos.topics.gen.ts`:

```ts
export interface DimosTopics {
  "/nav/pose": geometry_msgs.PoseStamped;
  "/nav/cloud": sensor_msgs.PointCloud2;
}
export interface DimosCommands {
  "ScopeNav": {
    "navigate_to": { args: [geometry_msgs.PoseStamped]; ret: boolean };
    "start": { args: []; ret: void };
  };
}
```

### 2. Consume with a typed client

Pass the two maps as generics — topic names + message types and RPC target/method/args all become checked
and autocompleted:

```ts
import { createDimosClient } from "@dimos/web";
import type { DimosCommands, DimosTopics } from "./dimos.topics.gen.ts";
import type { geometry_msgs } from "@dimos/msgs";

const dimos = createDimosClient<DimosTopics, DimosCommands>();
await dimos.connect("ws://localhost:8080");

// ── topics: the name autocompletes, and `m.data` is inferred from the blueprint's Out[] type ──
dimos.subscribe("/nav/pose", (m) => {
  m.data.position;          // ✅ geometry_msgs.PoseStamped — fields autocomplete
});
const latest = dimos.latest("/nav/pose"); // latest?: geometry_msgs.PoseStamped
dimos.subscribe("/nav/psoe", () => {});   // ❌ compile error — not a topic on this robot

// ── commands: target, method, args and the return type are all typed ──
const goal = {} as geometry_msgs.PoseStamped;
const ok: boolean = await dimos.modules.ScopeNav.navigate_to(goal); // (goal: PoseStamped) => Promise<boolean>
await dimos.modules.ScopeNav.start();                               // () => Promise<void>

dimos.modules.ScopeNav.navigate_to(42);   // ❌ arg must be a PoseStamped
dimos.modules.ScopeNav.liftoff();         // ❌ no such @rpc method on ScopeNav
dimos.modules.Roomba.start();             // ❌ no such module
```

In React, bind the map once and every topic hook is typed (see `app/src/dimos.ts`):

```ts
export const { useTopicLatest } = createDimosHooks<DimosTopics>();   // from @dimos/react
const { data } = useTopicLatest("/nav/pose");                       // data?: geometry_msgs.PoseStamped
```

Escape hatch: a bare `createDimosClient()` (no generics) accepts any topic/module and returns `unknown`,
and `dimos.call(target, method, …)` is always available for dynamic, ungenerated commands. An `@rpc`
arg/return the mapper can't type stays `unknown`; regenerate when the blueprint's topics or `@rpc` change.

## Typed codegen reference — `scripts/gen_types.py`

Generates the `DimosTopics` + `DimosCommands` TypeScript **statically** from a blueprint — no gateway, no
robot, no running bus. The import is side-effect-free (class-definition reflection only; the blueprint is
never instantiated). It's the only way to type RPC args/returns — the wire only advertises
`{target, method}`, never signatures.

```bash
deno task gen-types scenarios/nav.py                                     # print to stdout
deno task gen-types scenarios/nav.py --out app/src/dimos.topics.gen.ts   # write the app's map
```

`deno task gen-types` = `uv run python packages/web/scripts/gen_types.py`.

**What it reads:**

- **Topics** — the blueprint's module-level `PORTS = [(attr, topic, MsgClass), …]` list. Each →
  `"<topic>": <pkg>.<Name>` from `MsgClass.msg_name`.
- **Commands** — each `@rpc` method declared on the `Module` subclass → `"<method>": { args: [...]; ret: ... }`
  from its signature.

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
