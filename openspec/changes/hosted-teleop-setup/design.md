# Design — Hosted Teleop Setup

## Context

DimOS robots previously supported LAN teleop only (quest module's local WebSocket server). Internet operation requires dial-out through an SFU broker — robots sit behind NAT/cellular with no inbound ports. An earlier iteration (`HostedTeleopModule`) put the whole WebRTC client inside one teleop module; it worked but coupled broker plumbing to teleop logic and could not share its session with transport-bound streams (each module owning a PeerConnection = a second session the operator can't see). The broker (Cloudflare Realtime or LiveKit, via the `dimensional-teleop` repo) bridges datachannels one-direction-only and assigns SCTP ids; the deployed operator HTML lives broker-side.

## Goals / Non-Goals

**Goals:**
- One broker session per robot carrying video + commands + state, all bound via blueprint transports (transport-swap pattern).
- Shared control plane across robot shapes; a new robot (G1) implements only hooks.
- Operator safety: E-STOP latch, deadman on link loss, stale/reorder command drops, allow-listed robot commands.
- Multi-camera support (operator-selectable) within the single video track.
- Both CF Realtime and LiveKit backends behind the same blueprint surface.

**Non-Goals:**
- Multi-operator arbitration (one operator per robot).
- Robot-side auto-redial and TURN (tracked as follow-up tasks).
- Multiple simultaneous video tracks (mux composites into one).

## Decisions

- **Transport-swap over module-owned WebRTC**: `BrokerProvider` is a per-process singleton owning the PeerConnection; blueprints bind `Cloudflare*`/`LiveKit*` transports to streams of ONE module per robot. Alternative (kept-legacy): module-owned PC — rejected because video and data planes must share one bundled session, and provider-level ownership lets any stream ride it.
- **Colocation rule**: all broker-bound streams live on a single module per robot (`Go2HostedConnection` subclasses the driver since it is `dedicated_worker=True`; `ArmHostedConnection` is standalone because arm actuation goes through the ControlCoordinator over LCM).
- **Shared plane as mixins** (`HostedConnectionMixin` extends `CameraMuxMixin`): hooks (`_handle_estop`, `_handle_robot_msg`, `_telemetry_state`, `_telemetry_tick`) rather than a base Module class, so robot connections keep their natural base class (driver vs teleop module). Safety hooks have **no defaults** — a hosted robot without an E-STOP path fails loudly at wiring time.
- **Arm teleop logic reused from the quest module**: `ArmHostedConnection` subclasses `ArmTeleopModule` (engage gating, delta poses, task routing, trigger packing); the quest module's web server became lazily created so hosted subclasses never start it. Alternative: port logic from deprecated `HostedTeleopModule` — rejected; it was itself a copy of the quest module.
- **Multiple same-model cameras via subclassing** (`FrontCamera`/`WristCamera(RealSenseCamera)`): module identity is the class throughout the stack (blueprint dedup, registries, remap keys, RPC topics, config namespace), so distinct classes give distinct instances with zero framework change. Serials pin devices (`-o frontcamera.serial_number=...`). Alternative: first-class instance identity in the blueprint layer — larger framework change, deferred.
- **One video track + mux**: operator selects cam1/cam2/both; "both" hstacks into the single track. Publish-side fps/width caps applied at the mux so they hold on both transports. Latency measured by a pixel-encoded capture-time strip (survives H.264; metadata does not), decoded and cropped operator-side.
- **Go2 command execution**: single-worker executor (strict ordering — rage toggles raced), bounded backlog (busy-reject past 4), urgent bypass for Damp/E-STOP, nonce dedup for browser re-sends. Arm commands are instant state flips; no queue needed.
- **Clock-sync answered by the provider** (`BrokerProvider._maybe_answer_ping`), inline on the loop thread — pub/sub dispatch would jitter RTT samples.

## Risks / Trade-offs

- [CF/aiortc quirks: MAX_BUNDLE, addTrack-before-createDataChannel, candidate propagation, SCTP id 0 reservation] → hard-won workarounds documented in `dimos/teleop/hosted/README.md`; do not regress.
- [CF does not auto-reap datachannel pushes; browser nonces restart per session] → broker closes stale state_back push before re-push; robot clears nonce cache on operator_lost.
- [Legacy modules still power `teleop-hosted-go2`/`teleop-hosted-xarm7`] → deprecation notes added; deletion gated on migrating those blueprints to the new connections.
- [Uplink congestion on cellular turns into latency, not just bitrate loss] → publish-side caps (`video_max_width`/`video_max_fps`); budget ~5 Mbps uplink per robot.
- [Both-cams hstack raises encode cost ~30-50%] → cap width to restore budget.
- [No TURN yet] → networks blocking UDP/STUN will fail to connect; follow-up task.

## Migration Plan

1. Land transport-swap connections + blueprints (done on this branch; legacy blueprints untouched).
2. Validate `teleop-hosted-xarm7-multicam` on hardware (dual RealSense) and ship the arm operator UI broker-side.
3. Migrate `teleop-hosted-xarm7` → `ArmHostedConnection`; drop `teleop-hosted-go2` (superseded by `-transport`).
4. Delete `hosted_teleop_module.py` / `hosted_extensions.py` + their tests.

## Open Questions

- Arm operator UI: reuse the Go2 cockpit shell with an arm panel, or a separate WebXR-first page? (camera_select/estop/telemetry protocol is shared either way)
- TURN: broker-issued credentials vs static config.
- Auto-redial backoff policy when the broker restarts mid-session.
