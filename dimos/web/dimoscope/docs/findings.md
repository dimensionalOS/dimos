# dimoscope — research findings

A distilled, team-facing summary of a research pass on **DimOS web/JS integration**: a transport-agnostic browser SDK (`@dimos/topics`), the gateways behind it, a media plane for the camera, and what it implies for multi-robot and the #2502 web-API direction.

See also: [README](./README.md) · [benchmarks](./benchmarks.md) · [custom messages in the browser](./custom-messages-in-the-browser.md).

---

## 1. Transports & the browser story
- **dimos is a bus of typed streams.** Modules have `In[T]`/`Out[T]` ports and pub/sub over a transport. The transport *is* the internal bus; macOS defaults **zenoh**, Linux **lcm** (`core/global_config.py`).
- **LCM has no browser path** — browsers can't do UDP multicast, so `@dimos/lcm` is Deno-only and browsers need a gateway.
- **Zenoh is the only transport with a real browser story** (`zenoh-ts` + the `remote-api` plugin over WS/WebTransport). It carries the *same* LCM-encoded payloads, so the codec is unchanged. → the main argument to **converge on Zenoh** long-term and treat the LCM gateway as transitional.
- **SHM** (`pSHMTransport`) is great for big local data (camera/pointcloud) but lives **off the network bus**, so anything on SHM is invisible to a bus-tapping gateway — the reason the Mac `color_image`→SHM optimization was disabled for the web viewer. A global SHM-vs-UDP toggle is the open TODO there.
- **The codec is transport-agnostic** (`@dimos/msgs`): a self-describing **8-byte type hash** lets any client decode any topic with zero config — what makes a *generic* viewer and SDK possible in the first place.

## 2. The `@dimos/topics` SDK + three gateways behind one contract
**Thesis: "DimOS topics in the browser", not "Zenoh in the browser."** One small `Transport` interface (`packages/topics/src/transport.ts`); the app/SDK code is identical across all three transports, switchable live from the topbar dropdown:
- **Bun↔LCM** — `servers/gateway.ts`. Reads the LCM UDP-multicast bus, relays raw packets; the browser decodes. Bun can't speak Zenoh natively.
- **Python↔Zenoh** — `servers/gateway_zenoh.py`. The only practical way to put Zenoh behind a gateway (Bun/Node have no native Zenoh).
- **zenoh-ts (direct)** — `packages/topics/src/adapters/zenohTs.ts`. Browser ↔ `zenoh-bridge-remote-api`, **no gateway in the read path**; the browser drives a real server-side session and gets real `declareSubscriber` → **true end-to-end on-demand**. Read-path only; teleop/goal still go through a gateway.

Key properties:
- **The gateway is a byte-relay, not a parser** — it forwards raw, self-describing payloads. → language/transport isn't the bottleneck (see [benchmarks](./benchmarks.md)).
- **On-demand subscribe** cuts WS bandwidth ~75% when viewing 1 of 4 topics.
- **Teleop is server-side safety-clamped**: the browser sends structured `{linearX, angularZ, ttlMs}` (never raw bus bytes); the gateway clamps velocity, runs a TTL deadman (~400 ms), and stops the robot on disconnect. zenoh-ts forwards teleop to a gateway to preserve that boundary.

## 3. Media plane (the camera is special)
- The camera is the one heavy stream (`sensor_msgs.Image` 1280×720@20fps ≈ **55 MB/s raw / 11 MB/s JPEG**) and the only stream that gains nothing from "decode in the browser" — you just want to *see* it. So it belongs on its **own plane beside Transport**: opaque video, no 8-byte hash, no decode.
- Built `MediaChannel` + `selectMediaChannel()` (`packages/topics/src/media.ts`) + `useVideo(topic)`; **pluggable + negotiated** like the transport dropdown — JPEG (universal floor) / WebRTC (aiortc; ~20–40× less bandwidth) / WebCodecs (deferred), chosen by capability + fallback.
- Pulled into a **standalone media node** (`servers/media_server.py`, :8092): its own Zenoh peer, **encode-once-per-topic, fan-out to N viewers**, publishes nothing to the bus. Two-server model = thin data relay + media node (CPU-encode vs I/O-relay, independent lifecycle, no safety coupling).
- **Scaling keystones for many-cameras**: encode once / fan out; encode at the source and carry encoded video on the bus; on-demand by visibility; HW-decode per tile. Delivery future = WebRTC-SFU (LiveKit/mediasoup) vs WebCodecs+WebTransport (more native to the on-demand model; negotiate + fall back).

## 4. Custom messages in the browser (zero-rebuild)
Users who invent their own message types can't see them in the browser today: the SDK only decodes types baked into the published `@dimos/msgs` registry, and an unknown type's sample is silently dropped. The fix needs **no codec regeneration** — the Python `dimos_lcm` classes self-describe at runtime, so the Python gateway can decode (or schema-emit) any type and the existing `JsonInspector` fallback renders it. Full design: **[custom-messages-in-the-browser.md](./custom-messages-in-the-browser.md)**.

## 5. Multi-robot / multi-camera
- **The reframe:** topic collisions only exist because sims hardcode the *same* names. **Distinct logical names never collide** (`make_transport("/robot1/cmd_vel", …)` → `dimos/robot1/cmd_vel/…`).
- → **multi-robot/multi-cam VIEW is zero-core** (distinct topic names; a 3-robots-in-one-browser demo needs no core change — synthetic robots + a small gateway tweak + a browser selector + camera grid).
- → **independent COMMAND is core-owned**: RPC and TF stay **global** (`dimos/rpc/{Module}/…`, `dimos/tf`), so two `GO2Connection`s collide there. Per-robot bus namespacing (a real `namespace` field through `transport_topic`/RPC/TF — *not* overloading `robot_id`) is the ROS-standard mechanism but is unbuilt and cross-cutting.
- `FleetConnection` today is a **swarm** (one twist broadcast to all; only the primary's sensors on the bus), not independent control.
- Multi-camera on one host = **blueprint remapping** (`/cam_front`, `/cam_arm`); no multi-camera blueprint exists yet → greenfield.

## 6. #2502 web-API direction
- #2502 ("TS API Spec") is a **proposal, not code**. Today modules each embed their own web server (FastAPI/Starlette/SocketIO/gRPC — 4 bespoke, non-topic-generic bridges). #2502 wants to **consolidate** into one shared `DimosWebsocket` (subscribe-all + static serve + per-conn whitelist/QoS) + `app.modules.X.y()` proxy RPC + `peek`/`subscribeAll`/`setQos`.
- **Our gateways + media node are the de-facto prototype** — they already do subscribe-all + self-describing relay + RPC bridge + on-demand + multi-transport, uniformly via the 8-byte hash. Video isn't even in the spec; the topic-keyed media node is a counterpart we could fold in.
- **The unifying lens:** a Module is just `In`(subscribe) / `Out`(publish) / `@rpc`(call) over one transport → **three primitives: subscribe · publish · call.** #2502's whole TS surface is those three over WS. "Nothing to invent, just consolidate." The browser is **just another module-consumer**, reached over WS only because browsers can't speak the bus.

## 7. QoS from the client (parked)
- Client QoS today = **rate only** (`topic.setRateLimit(hz)`): enforced server-side on Bun↔LCM, **ignored by the Zenoh gateway** (gap, ~15 lines to port), client-drop-only on zenoh-ts.
- Zenoh QoS is **publisher-side only** (`SubscriberOptions` carries none) → the browser is a subscriber, so reliability/durability have no subscriber-side home; the QoS fruit is **gateway-side**, not zenoh-ts. #2502's `setQos` is ~10% built.
- The compelling demo (build later): an in-app network selector (LAN/4G/3G/2G → gateway injects latency/bw-cap/loss) + per-topic rate sliders beside live stats → "pick 3G, rate-limit lidar, keep camera/teleop responsive." (Chrome DevTools throttling does **not** affect WebSocket/WebRTC — use gateway netsim or toxiproxy.)

---

## You're not running upstream (forks)
- **Rerun is forked** — `dimos-viewer` (pip `dimos-viewer==0.32.0a1`), a Dimensional fork; `RerunBridge` falls back to stock rerun if missing. When debugging Rerun, read the fork.
- **LCM is forked** — `lcm-dimos-fork` / `dimos-lcm`, which also hosts the `.lcm` defs + the `lcm-ts` codegen behind `@dimos/msgs`.
- **Rerun is not on the transport** — a `RerunBridge` *module* `subscribe_all`s the active transport and forwards to the Rerun viewer over Rerun's own gRPC SDK.

## Side fixes made (review together)
- **DimSim→Zenoh bridge** (`dimos/robot/unitree/dimsim_connection.py`, committed `df863e1b0`): on a non-LCM transport, `DimSimConnection` bridges DimSim's LCM sensors onto the active transport, so DimSim renders in Rerun on a zenoh run with no flag; teleop reverse path bridged; echo-guarded on LCM runs.
- **`docs/usage/cli.md`**: corrected the `--simulation` row — it's a **string** (`mujoco|dimsim`), there is no `--no-simulation`; bare `--simulation` → `mujoco`.

---

## Open questions for the team
**Transports**
- Long-term: converge on Zenoh and drop the LCM gateway (LCM has no browser path), or keep both with `zenoh-ts`-direct as the target and the gateway as a bridge?
- Should the Mac SHM `color_image` optimization return behind a config flag, or do we standardize on Zenoh (now the macOS default)?
- QoS: `SubscriberOptions` carries no QoS in Zenoh, so a browser `setQos` (#2502) can only map to publisher/bridge config or gateway emulation — is that the intended model? Rate-only now, or invest in gateway-side QoS emulation?

**Cameras / video**
- Standalone media server (encode-once, fan-out, transport-independent) vs per-viewer encode — right model?
- Many-camera/many-viewer future: WebRTC-SFU (LiveKit/mediasoup) vs WebCodecs+WebTransport?
- Encode at the source and carry encoded video on the bus (extend `jpeg_lcm`/gstreamer H.264) so every hop just forwards — desired?
- Multi-camera on one host = blueprint remapping (`/cam_front`, `/cam_arm`) — intended pattern? (none exists yet.)

**Multiple robots**
- Per-robot bus namespacing (`/robotN/*`) — wanted, or is "which robot" deliberately a broker/app concern (as `robot_id` is today, #2490)?
- Independent multi-robot control on the roadmap, or is `FleetConnection` (swarm) the model?
- With two instances of one module, `dimos/rpc/{Module}/{method}` collides — how do you envision **per-instance RPC/TF addressing**?

**Web API (#2502)**
- Is #2502 the official web direction, and is the intent to promote our gateway → the core `DimosWebsocket` (vs staying a dimoscope-local prototype)? If so, who owns building it?
- Is **video/media** in scope for the web API? (Our topic-keyed media node is a working counterpart.)
- Worth publishing `@dimos/topics` / `@dimos/react` to the public JSR `@dimos` scope?

**Tooling / repo**
- A **JS monorepo** inside this Python monorepo (top-level `./js`, Deno runtime, strict config for efficient AI coding) — is that the management approach we want for first-class web/JS? (Would also bring in the `dimos-lcm/tools/ts` codegen so everything's in one automatable place.)
