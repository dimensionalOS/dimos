# dimTELE feasibility for Microduck World

Reviewed 2026-09-06. This is a source review and implementation proposal, not a deployed dimTELE integration or a public-network benchmark.

## Decision

**Feasible, but not a configuration-only migration.** dimTELE supplies reusable Internet teleoperation transports. Its current broker and browser application do not supply our anonymous, three-duck multiplayer experience.

For the immediate goal of publishing `sim.tule.world`, I recommend an optional WebSocket browser transport in the existing DimOS web SDK/relay, carried through Cloudflare Tunnel. This retains the current cockpit, Three.js rendering, robot blueprints, admission rules and scene assets. It does not remove DimOS from the stack. dimTELE/WebRTC remains a useful subsequent option for camera video and physical-robot teleoperation.

If proving dimTELE integration is the higher priority, first build a bounded one-duck prototype, including an independent read-only spectator and a forced connection failure. Expand to three ducks only after those paths work. Do not begin by replacing all three working blueprints.

## What was inspected

- DimOS checkpoint `e0676c6`: `dimos/core/transport.py`, `dimos/protocol/pubsub/impl/webrtc/providers/broker.py`, `video_track.py`, and `dimos/teleop/hosted/`.
- Broker repository `dimensionalOS/dimensional-teleop`, commit `895356b`: session routes, tenant authorization, Cloudflare orchestration, LiveKit token grants and browser connection code. This repository is private; it has not been copied into the public DimOS PR.
- Hosted project checkpoint `579c6e7`: project relay/lobby, gateway, cockpit declarations, world snapshots and the public-access plan. The live DimOS vendor remains on its existing tested pin plus patch.
- Current Cloudflare documentation for SFU DataChannels, TURN and Tunnel WebSockets.

No broker credentials were created, no Cloudflare resources were provisioned, and no running duck was connected to dimTELE during this review. Permission to use the shared production broker for public visitor traffic, its deployed revision, account limits and billing remain unverified.

## What dimTELE already provides

DimOS has real `CloudflareTransport`, `CloudflareVideoTransport` and audio transport implementations. The robot makes outbound connections; browsers and robots exchange WebRTC media/data through Cloudflare Realtime SFU after HTTPS setup through the broker. Omarchy can continue hosting MuJoCo and every agent without exposing an inbound WebRTC port.

The transport binds ordinary typed DimOS streams. Therefore the boundary can remain portable: each duck's transport adapter receives that duck's image/odometry and sends typed movement or skill requests to its existing controller. A future physical connection module can sit behind the same boundary. This does not make the MuJoCo ONNX locomotion executor itself hardware-ready.

Three independently running duck processes fit three broker sessions. The inspected broker namespaces sessions by owner and a client-supplied robot ID, so explicit distinct robot IDs can coexist under an owner. The provider is a per-process singleton keyed by configuration; all broker-bound modules for one duck must share the appropriate worker/configuration. The shared world remains one simulation, not three simulations.

WebRTC can carry game state as well as cameras. Cloudflare supports named DataChannels with multiple subscribers and delivery settings suitable for disposable state updates. Its current API also supports optional single-subscriber reply access; the inspected dimTELE broker uses separate forward and reverse channels instead. Platform support does not mean the broker has already implemented our multiplayer policy. [Cloudflare DataChannels](https://developers.cloudflare.com/realtime/sfu/datachannels/)

TURN is needed for robust access from restrictive networks. Cloudflare provides UDP, TCP and TLS relay options; merely configuring STUN is insufficient for every visitor network. [Cloudflare TURN](https://developers.cloudflare.com/realtime/turn/)

## Gaps that matter for this project

| Requirement | Inspected dimTELE behavior | Required integration |
|---|---|---|
| Anyone can choose an available duck | Operators authenticate through Cognito; listing/joining checks robot owner or admin | A scoped visitor grant backed by our lobby; do not share an owner login or robot API key with visitors |
| Three exclusive duck slots | One operator claim per robot session; the same account may reclaim its own slot | Bind ownership to a unique participant and duck generation, including concurrent tabs, revocation and our reconnect grace |
| Read-only world viewer | Join schema accepts `viewer`, but Cloudflare channel bridging checks the bound operator and uses one stored operator session; shipped browser joins as operator | Complete spectator registration, independent subscriptions and cleanup; world-only permissions; spectator departure must not evict a controller |
| Three.js world, score and actors | Robot video/map/telemetry are built in; no world-state manifest contract | Feed the existing world snapshot to a public read-only channel, keep geometry/textures on HTTPS and retain client interpolation |
| Agent chat, tool activity, policies and annotated maps | Fixed command/state/map channels; no adapter for our web SDK's manifest, codecs and replay | Map those streams explicitly, or implement a general SDK transport adapter with bounded replay/backpressure |
| Microduck controls | Broker accepts only `go2` and `arm`; existing command handlers are robot-specific | Add a Microduck/generic robot capability path; call our controller rather than reusing Go2 action IDs |
| Persistent headless hosting | Provider closes after repeated terminal heartbeat responses; no automatic full redial in that path | Supervised reconnect, new SFU session/channel IDs, deadman stop and generation-safe restoration |
| Three independent robot minds | Transport sessions do not implement knowledge isolation | Preserve our per-duck blueprints and private stream wiring; never feed shared world geometry into agents |

The optional LiveKit backend does not remove these gaps. In the inspected broker, viewer tokens still receive data-publishing permission. That is not proof that a viewer can move a robot, but it is insufficient evidence for a read-only boundary; robot-side sender/role checks and tighter grants would need validation. The current DimOS checkout also lacks the corresponding LiveKit provider, so this is not the shortest integration path.

The DimOS provider warns above 32 KiB per data message but still sends it. That warning is not an enforced payload limit or fragmentation mechanism. Full maps, transcripts and JPEGs cannot be routed blindly through those channels. Use bounded frames/chunks, request a fresh state after reconnect, and use media tracks for video. No payload-limit or throughput benchmark has been performed here.

## Architecture if we adopt dimTELE

```text
Browser at sim.tule.world
  ├─ HTTPS through Cloudflare Tunnel → Omarchy lobby and scene assets
  ├─ scoped session authorization → dimTELE broker
  └─ WebRTC through Cloudflare Realtime SFU
       ├─ chosen duck: commands, private telemetry/chat/map, optional camera
       └─ public world: read-only poses, actors and score

Omarchy
  ├─ one MuJoCo physics world
  ├─ Duck 1 blueprint → its controller, sensors, map and agent → session 1
  ├─ Duck 2 blueprint → its controller, sensors, map and agent → session 2
  └─ Duck 3 blueprint → its controller, sensors, map and agent → session 3
```

The public world needs its own logical read-only stream. It could initially use WebSocket through the existing relay, or become a separate SFU publisher. Spectators must not subscribe to every duck's private camera/map/chat merely to render the world.

Our lobby should remain the authority for seat allocation and generation changes. Broker sessions should derive from that decision through short-lived scoped credentials. Maintaining two independent ownership systems would create release/reconnect races. A stale participant must lose both command access and private subscriptions.

The existing production broker would require a supported extension and deployment by its maintainers. Alternatively, a separate broker could run on Omarchy behind Tunnel, but its current setup adds Cognito, a database and Cloudflare SFU/TURN credentials. Source access alone does not establish authorization to operate its shared service or redistribute private broker code.

## Cost and performance implications

MuJoCo physics and the agents' own server-rendered images continue unchanged. Browser Three.js rendering does not become smoother merely because the transport changes; the existing renderer already interpolates incoming state locally.

Using WebRTC for camera video introduces encoding work on Omarchy and may reduce network use compared with individual JPEG frames. The inspected provider prefers H.264, but it does not establish an NVIDIA hardware-encoding configuration. Three camera tracks require a benchmark under physics and agent load. SFU fanout can reduce Omarchy's upload amplification as spectators grow, while introducing a managed service and its billing.

For our current primary display, small world-state messages drive browser rendering and no continuous server video is required. This makes a WebSocket option attractive for the first public release. Cloudflare Tunnel explicitly supports WebSockets. [Cloudflare Tunnel FAQ](https://developers.cloudflare.com/cloudflare-one/faq/cloudflare-tunnels-faq/#does-cloudflare-tunnel-support-websockets)

WebSocket's reliable TCP delivery can delay newer messages behind older traffic during loss or congestion. The implementation must cap queues, replace stale world/video updates, avoid accumulating old motion commands, prioritize control, preserve the deadman timeout, and reconnect without replaying movement. It is a smaller migration, not an automatic latency improvement.

## File ownership and proposed checkpoints

Generic SDK/relay transport code belongs in DimOS. A future generic broker capability or spectator correction belongs in dimTELE. Lobby grants, Microduck command adaptation, blueprints and deployment configuration belong in `microduck-world`. Football rules, scene geometry, colors and textures remain project assets. No scene-specific additions to DimOS demos are necessary.

1. Choose the first public transport. For WebSocket, add it behind the existing SDK/relay interface with bounded control/state handling. For dimTELE, first prove one isolated duck plus one read-only spectator, including a forced redial.
2. Validate all three duck slots, duplicate tabs, fresh visitor generations, delayed/revoked commands, private-stream denial and world-only spectators. Do not drop existing chat, policies, maps or JPEG/Three.js comparison features.
3. Configure the Omarchy tunnel and domain, settle Duck 1 persistence/access, and enforce anonymous-agent and spectator budgets. Test from outside the tailnet, including a restrictive network and a slow client.
4. Measure physics timing, client rendering, command latency, reconnects and resource use with three controllers and representative spectators before switching the default public transport.

One upgrade detail surfaced during the checkpoint: upstream `main` now recognizes acknowledged `pub` commands, while the hosted lobby currently gates its pinned protocol's command set (`tx`, twist, stop and teleop leases). Before upgrading that vendor pin, extend the admission policy and tests to every new write path, including `pub`. The working service was not upgraded during this checkpoint.

## Source pointers

- [DimOS hosted teleoperation](https://github.com/dimensionalOS/dimos/blob/e0676c6f3926e855c33dd397d4120ccf98073989/docs/capabilities/teleoperation/hosted.md)
- [DimOS broker provider](https://github.com/dimensionalOS/dimos/blob/e0676c6f3926e855c33dd397d4120ccf98073989/dimos/protocol/pubsub/impl/webrtc/providers/broker.py)
- Private broker revision `895356b`: `app/routers/sessions.py` (`create_session`, `_owns`, `_claim_operator_slot`, `join_session`, `bridge_datachannel`, `leave_session`); `app/services/auth.py`; `app/services/livekit.py`; `web/js/webrtc.js`.
- Project: `relay/lobby.ts`, `relay/main.ts`, `app/microduck_world/cockpit.py`, `app/microduck_world/world_sim.py`, and [public access](public-access.md).
