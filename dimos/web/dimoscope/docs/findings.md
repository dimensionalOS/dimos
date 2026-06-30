# dimoscope — research findings

A distilled, team-facing summary of a research pass on **DimOS web/JS integration**: a transport-agnostic browser SDK (`@dimos/topics`), the gateways behind it, a media plane for the camera, and what it implies for multi-robot and the #2502 web-API direction.

See also: [README](./README.md) · [benchmarks](./benchmarks.md) · [custom messages in the browser](./custom-messages-in-the-browser.md).

> **Architecture note:** the three gateways + standalone media node described below were since
> **consolidated into one `serve.py` service** (`servers/{bus,data,media,bench}.py`) that taps LCM
> **and** Zenoh at once and serves every transport on one origin — see [README](./README.md). The
> findings and tradeoffs below stand; only the deployment changed.

---

## 1. Transports & the browser story
- **dimos is a bus of typed streams.** Modules have `In[T]`/`Out[T]` ports and pub/sub over a transport. The transport *is* the internal bus; macOS defaults **zenoh**, Linux **lcm** (`core/global_config.py`).
- **LCM has no browser path** — browsers can't do UDP multicast, so `@dimos/lcm` is Deno-only and browsers need a gateway.
- **Zenoh is the only transport with a real browser story** (`zenoh-ts` + the `remote-api` plugin over WS/WebTransport). It carries the *same* LCM-encoded payloads, so the codec is unchanged. → the main argument to **converge on Zenoh** long-term and treat the LCM gateway as transitional.
- **SHM** (`pSHMTransport`) is great for big local data (camera/pointcloud) but lives **off the network bus**, so anything on SHM is invisible to a bus-tapping gateway — the reason the Mac `color_image`→SHM optimization was disabled for the web viewer. A global SHM-vs-UDP toggle is the open TODO there.
- **The codec is transport-agnostic** (`@dimos/msgs`): a self-describing **8-byte type hash** lets any client decode any topic with zero config — what makes a *generic* viewer and SDK possible in the first place.

## 2. The `@dimos/topics` SDK behind one contract
**Thesis: "DimOS topics in the browser", not "Zenoh in the browser."** One small `Transport` interface (`packages/topics/src/transport.ts`); the app/SDK code is identical across every delivery mechanism, switchable live from the topbar dropdown. The research explored three backend approaches before folding them into the single `serve.py` (which taps LCM **and** Zenoh):
- **LCM relay** — read the LCM UDP-multicast bus, relay raw packets; the browser decodes. (A Node/Deno relay can't speak Zenoh natively — one reason the consolidated service is Python.)
- **Zenoh relay** — subscribe Zenoh `**`, re-wrap as LC02; the practical way to put Zenoh behind a gateway.
- **zenoh-ts (direct)** — `packages/topics/src/adapters/zenohTs.ts`. Browser ↔ `zenoh-bridge-remote-api`, **no gateway in the read path** → **true end-to-end on-demand**. Still shipped in the SDK as an option, but not wired into the app after consolidation.

Key properties:
- **The gateway is a byte-relay, not a parser** — it forwards raw, self-describing payloads. → language/transport isn't the bottleneck (see [benchmarks](./benchmarks.md)).
- **On-demand subscribe** cuts WS bandwidth ~75% when viewing 1 of 4 topics.
- **Teleop is server-side safety-clamped**: the browser sends structured `{linearX, angularZ, ttlMs}` (never raw bus bytes); the gateway clamps velocity, runs a TTL deadman (~400 ms), and stops the robot on disconnect. zenoh-ts forwards teleop to a gateway to preserve that boundary.

## 3. Media plane (the camera is special)
- The camera is the one heavy stream (`sensor_msgs.Image` 1280×720@20fps ≈ **55 MB/s raw / 11 MB/s JPEG**) and the only stream that gains nothing from "decode in the browser" — you just want to *see* it. So it belongs on its **own plane beside Transport**: opaque video, no 8-byte hash, no decode.
- Built `MediaChannel` + `selectMediaChannel()` (`packages/topics/src/media.ts`) + `useVideo(topic)`; **pluggable + negotiated** like the transport dropdown — JPEG (universal floor) / WebRTC (aiortc; ~20–40× less bandwidth) / WebCodecs (deferred), chosen by capability + fallback.
- Runs as its **own plane** (`servers/media.py`, the `/media` path of the one service): taps the shared bus, **encode-once-per-topic, fan-out to N viewers**, publishes nothing. (It began as a standalone Zenoh-peer node; consolidation folded it onto the unified bus, so webrtc/webcodecs now work on an LCM *or* Zenoh camera source.)
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

## 7. QoS from the client (built)
- Beyond rate-limiting (`topic.setRateLimit(hz)`), the data plane now runs a **priority + conflation outbox** (`servers/qos_sched.py`): pose/teleop lanes drain ahead of bulk lanes (lidar/camera), and freshest-wins conflation drops stale frames under backpressure instead of queueing them — so important data survives a saturated link. Client knobs live in `packages/topics/src/qos.ts`; see **[qos-demo.md](./qos-demo.md)**.
- Zenoh QoS is **publisher-side only** (`SubscriberOptions` carries none) → the browser is a subscriber, so the QoS fruit is **service-side** (where the outbox lives), not in the browser-direct path.
- Network impairment for the demo rides the gateway `netsim` proxy (Chrome DevTools throttling does **not** affect WebSocket/WebRTC).

## 8. Rerun in the browser: the firehose ceiling
- The dimoscope **Rerun 3D panel** embeds the **stock** `@rerun-io/web-viewer-react@0.32.0-alpha.1` against the bridge's gRPC proxy (`:9877`). It connects and streams (`Streaming messages from gRPC endpoint :9877` ✓) but renders a **black canvas with no UI chrome** — never paints geometry.
- **Root cause is data volume, not view config.** Live console: **+1 s** `Channel byte budget (128 MiB) exceeded. Cannot block on web; sending anyway.` → **~1 min** `Reached memory limit of 2.3 GiB. Freeing up data…` with GC reclaiming **<1% per pass**. The `RerunBridge` `subscribe_all`s and `rr.log`s every stream **undecimated** — lidar + global_map + costmap `PointCloud2` + camera — so the browser WASM viewer blows its 128 MiB channel budget in ~1 s and its ~2.3 GiB heap ceiling in ~1 min, then thrashes GC and never reaches a paint. (The earlier `RuntimeError: unreachable` was the *same* cause: an arrow allocation failing under that pressure.)
- **The native `dimos-viewer` (`:9876`) eats the identical feed fine** — real process, no browser heap cap, faster arrow handling. This is *why* dimos defaults to and ships the native fork (see "forks" below); it isn't a nicety.
- **Implication:** in a browser tab the stock viewer can't survive this rate as-is, and **giving it the full page doesn't help** — the blank screen was never a sizing issue. **3D belongs in the native viewer**; a browser-viable 3D path would need **server-side decimation** of what the bridge logs to gRPC (rate/voxel caps, or a "lite" log set) — unbuilt.

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
