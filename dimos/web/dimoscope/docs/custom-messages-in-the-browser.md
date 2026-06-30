# Custom messages in the browser — a zero-rebuild design proposal

**Status:** proposal (not yet implemented). **Scope:** `dimos/web/dimoscope` (the `@dimos/topics` browser SDK + the gateways).

See also: [README](./README.md) · [findings](./findings.md) · [benchmarks](./benchmarks.md).

dimos is a framework where users — and their AI agents — author their own **Blueprints, Modules, and message types**, then publish their own **topics**. The browser SDK (`@dimos/topics`) is meant to show that data live. This doc explains why a *custom* message type doesn't render in the browser today, and proposes a path that needs **no codec regeneration and no package republish** from the user.

---

## 1. The problem

The browser-facing wire format is LCM. A channel is `"<topic>#<pkg>.<Type>"`, and every message body begins with an **8-byte fingerprint** that *identifies* the type. The browser decodes by looking that fingerprint up in `@dimos/msgs` (the JSR package `@jsr/dimos__msgs`), which holds a `hash → generated class` registry built at publish time.

The fingerprint is **one-way**: it confirms a type's identity but cannot reconstruct the field layout. So if a user invents a new message type, the browser has no class for it and **cannot decode it**. Worse, today the sample is *silently dropped* — `packages/topics/src/client.ts` `onSample` wraps `decodeBody` in a `try/catch` and discards anything that throws `Unknown message hash`:

```ts
// client.ts — onSample
try {
  data = decodeBody(s.payload);
} catch {
  return; // unknown type / fragment  ← custom messages die here
}
```

The only way to make the type known is to **regenerate and republish `@dimos/msgs`**. That generator (`lcm-ts`) lives in a *separate* repo (`dimensionalOS/dimos-lcm`) and the output is a published JSR package — there is no `.lcm` source or generator in this repo to run locally. Asking a user to fork/regen/publish a codec package to see their own topic is the "rebuild" we want to eliminate.

---

## 2. What already works (so the gap is narrow)

Three of the four pieces already exist for *any* type, including custom ones:

- **Discovery** — topics are surfaced from the channel name, no codec needed. A custom topic already shows up in `listTopics()` / the `SubscribeBar`. (`client.ts`, gateway `op:"topic"`.)
- **A generic render fallback** — `app/src/widgets/registry.ts` already maps message type → widget and **falls back to `JsonInspector` for any unknown type**:
  ```ts
  export function widgetForType(type: string): ComponentType<PanelProps> {
    return BY_TYPE[type] ?? JsonInspector; // generic fallback
  }
  ```
- **Custom messages are documented on the Python side** — `docs/usage/lcm.md` § "Creating Custom Message Types" (write a `.lcm` + a Python overlay subclassing `dimos_lcm`).

**The single missing link is DECODE for unknown types.** Because `decode()` throws and the sample is dropped, `JsonInspector` never receives any data. Close that one link and the existing discovery + render fallback light up. The TS/browser path for custom messages is the one piece nobody has documented or built — the Python side is documented; the Python→browser sync step is not.

---

## 3. The unlock: dimos messages self-describe at runtime

The generated `dimos_lcm` Python classes carry their full schema as class attributes. Example (`dimos_lcm/geometry_msgs/Vector3.py`):

```python
class Vector3(object):
    msg_name       = "geometry_msgs.Vector3"
    __slots__      = ["x", "y", "z"]                 # field names + order
    __typenames__  = ["double", "double", "double"]  # field types
    __dimensions__ = [None, None, None]              # array dims (None = scalar)
    @classmethod
    def _get_field_type(cls, field_name): ...        # resolves nested message types
```

Every dimos message (`dimos/msgs/*` subclasses `dimos_lcm/*`) exposes `__slots__` / `__typenames__` / `__dimensions__` / `_get_field_type`, plus the public `_decode_one(buf)`. **A Python gateway can therefore introspect any message class — including a user's — and either decode it to a dict or emit a complete recursive schema, with zero codegen, no `.lcm` IDL, and no JSR publish.**

That's the leverage: the *Python* side already has everything; only the *browser* lacks the type. So let the Python gateway, which imports the classes anyway, do the work for the long tail.

---

## 4. Recommended approach — gateway-side decode, tiered

The `@dimos/msgs` in-browser binary decode is an **optimization for the hot, known types**. Keep it. Add a fallback for everything else. Ship Tier 1 first.

### Tier 0 — known types (unchanged)
Browser decodes binary via `@dimos/msgs`. Fast, typed, zero gateway CPU.

### Tier 1 — server-side decode → self-describing payload (the no-rebuild answer)
For any topic whose type the browser can't decode, the **Python gateway** (`servers/gateway_zenoh.py`) calls `lcm_decode` (it has the class), walks the message into a plain dict recursively (using `__slots__`/`__typenames__`/`_get_field_type`; numpy arrays → lists), and ships it as a self-describing payload. The browser delivers it through the unchanged `Topic._deliver` path as a plain object, and `widgetForType` → **`JsonInspector`** renders it. **The user does nothing beyond defining their Python message.**

**Negotiation (lazy):** keep sending raw binary; when the browser's `decode()` throws `Unknown message hash`, the SDK sends a new control message `{op:"decode", topic}`; the gateway thereafter emits that topic decoded. Only the long tail of unknown types pays the decode cost; known topics stay on the binary fast path.

**Encoding — JSON first, measure before optimizing.** Start with **JSON**: it's the simplest, it's debuggable, and both gateways already speak JSON on the control channel (`subscribe`/`teleop`/`rpc`/`hello`) — so this is an additive data message, not a new channel. Move to **CBOR/msgpack** *only if* a real high-rate or array-heavy custom topic shows JSON size/CPU is a problem. Don't pre-optimize.

**Fit with the codebase:** net app change is small — route decoded-mode topics to a plain object (the SDK's `Topic<T>` is already `T = unknown`-tolerant) and let the existing `JsonInspector` fallback render it. A numeric auto-plot atop `JsonInspector` is a nice-to-have, not required.

### Tier 2 — schema advertisement (optional, future)
For high-rate custom topics where per-message gateway decode is wasteful, the gateway can emit the recursive **schema descriptor** once per type on discovery (`op:"schema"`), built from `__slots__`/`__typenames__`/`__dimensions__`. The browser then builds a **generic binary decoder driven by the schema** and keeps decoding raw bytes itself — preserving the zero-copy binary path for custom types too.

This is the shape of the **Foxglove WebSocket protocol** (advertise channel + schema → generic client decode). Note: **no Foxglove WS bridge or MCAP writer exists in this repo today** (only stray citation comments), so Tier 2 is built fresh, not adopted. Gate it on profiling — most custom topics are low-rate telemetry where Tier 1's JSON is plenty.

---

## 5. Gaps to close for true zero-rebuild

1. **Type/schema source for third-party packages.** `resolve_msg_type` (`dimos/msgs/helpers.py`) only searches `dimos.msgs.<m>.<C>`, `dimos.msgs.<m>`, and `dimos_lcm.<m>` — a user's `my_msgs.Foo` won't resolve by name. Cleanest fix: **don't resolve by name on the publish path at all.** A dimos `Out` port is constructed *with its `msg_type`* (`LCMTransport("/map", OccupancyGrid)`), so a gateway co-located with the blueprint can read the schema directly off the live transport binding. For the decoupled relay gateway, add a registration hook (a configurable namespace list, or an entry-point registry custom message packages opt into).
2. **Which gateway.** Decode requires Python, so this is a **Python/Zenoh-gateway feature**. The Deno↔LCM gateway can't import Python classes and stays a pure byte-relay for known types — which is fine, since the vibe-coding user's natural path is the Python gateway.
3. **Render polish.** `JsonInspector` covers correctness; a generic numeric auto-plot / array table keyed off "no bespoke widget for this type" makes custom telemetry pleasant. Small, optional, additive to the app.

---

## 6. Why not the alternatives

- **Local codegen (e.g. `dimos gen-msgs`) into a local `@dimos/msgs` overlay** — still a build step the user must run, and the `lcm-ts` generator isn't in this repo. Violates "they don't want to rebuild." Keep only as a power-user opt-in for those who *want* fully-typed TS classes.
- **Reuse the pickled transport (`pLCMTransport`/`pZenohTransport`)** — it serializes with Python `pickle`, which is not browser-safe. Making it browser-friendly would mean swapping the pickle path to CBOR/msgpack framework-wide — a much bigger blast radius than a gateway feature.
- **Make the LCM wire format self-describing** — would change the core encoding for every transport and robot. Far too invasive; the compact fingerprint is deliberate.

---

## 7. Verification plan (once Tier 1 is built)

1. **Regression (Tier 0):** `python examples/simplerobot/simplerobot.py --headless` + `python examples/fakesensors.py` publish `/odom` (PoseStamped) and `/map` (OccupancyGrid). Confirm both still decode in-browser via `@dimos/msgs` unchanged.
2. **The custom-type win:** add a throwaway custom message (e.g. `my_msgs.RobotState` with novel fields) to a tiny publisher, run the Python gateway, open the web app. Expect: the topic appears in `SubscribeBar` (already works via channel-name discovery) **and now renders decoded** through `JsonInspector` — with **no change to `@dimos/msgs`**.
3. **Encoding decision:** on a high-rate custom topic, compare JSON vs CBOR payload sizes and gateway CPU to decide whether Tier 2 (or a CBOR Tier-1 payload) is worth building.

---

## First implementation slice (when green-lit)

Tier 1 is one small PR:
- Gateway: handle `{op:"decode", topic}`; for decoded-mode topics, `lcm_decode` → dict → emit `{op:"sample", topic, json}` (JSON to start).
- SDK: on `Unknown message hash`, send `{op:"decode"}`; deliver the decoded object via the existing `Topic._deliver`.
- App: nothing new strictly required — `widgetForType` already falls back to `JsonInspector`.
