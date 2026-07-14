# Hosted Teleop

Robot dials out to the [dimensional-teleop](https://github.com/dimensionalOS/dimensional-teleop)
broker (Cloudflare Realtime SFU) — no inbound ports needed. The browser/VR
operator connects through the broker; commands arrive over WebRTC datachannels,
robot video goes out as a WebRTC track, and — opt-in (`audio_in`) — the
operator's mic comes back on a recvonly audio track.

## Files

The session (dial-out, datachannel lifecycle, video track) is owned by the
per-process `BrokerProvider` / `LiveKitBrokerProvider`
(`dimos/protocol/pubsub/impl/webrtc/providers/`); blueprints bind
`Cloudflare*`/`LiveKit*` transports to the streams of several small,
per-concern modules that all run in one process so everything shares that
single session:

- **`go2_command.py`** — `Go2CommandModule`: operator command / E-STOP dispatch
  and the manual-drive guard. Reaches the driver over `@rpc` (`GO2Connection`).
- **`camera_mux.py`** — `CameraMuxModule`: N cameras → one composited, capped
  video track (operator-selectable views).
- **`map_compress.py`** — `MapCompressModule`: costmap + odom → the minimap
  datachannel (coarsened, PNG-encoded, kept under the 16 KB datachannel limit).
- **`hosted_stats.py`** — `HostedStatsModule`: telemetry frame, command acks,
  and command-link latency/rate stats.
- **`command_executor.py`** — `SerializedCommandMixin`: serializes blocking
  driver commands with nonce dedup and a safety-epoch fence (E-STOP aborts).
- **`blueprints/cloudflare.py`** / **`blueprints/livekit.py`** — wire the above
  to the Go2 driver, cameras, planner, and transports (single + multicam).

The operator HTML lives in the [dimensional-teleop](https://github.com/dimensionalOS/dimensional-teleop)
broker repo (`web/`), not here.

## How a session connects

1. Robot creates an `RTCPeerConnection` (MAX_BUNDLE, **must** — see below),
   `addTrack(video)`, adds a recvonly audio transceiver if `audio_in` is set
   (the broker later pulls the operator's mic onto it and renegotiates via a
   heartbeat-ack offer), opens a throwaway negotiated DataChannel on SCTP id 0,
   creates an offer, gathers ICE non-trickle.
2. `POST /api/v1/sessions` to the broker with the offer. Broker creates a CF
   session, returns the answer + a `session_id` keyed off the robot.
3. SDP answer's candidates are propagated across bundled m-sections (aiortc
   workaround — see below) before `setRemoteDescription`.
4. Heartbeat thread polls `/sessions/{id}/heartbeat`; each ack carries the SCTP
   ids the broker has assigned for `cmd_unreliable`, `state_reliable`,
   `state_reliable_back`, and `map_unreliable`. We open / re-open / close
   negotiated channels to track the broker's view (when `state_reliable` drops
   to no id, the operator has left → the robot stops motion).
5. Once `pc.connectionState == "connected"`, `CameraVideoTrack.arm()` starts
   delivering frames (drops everything before the operator was actually able
   to receive).
6. Telemetry thread pushes command-plane stats (latency / jitter / rate
   from the inbound twist stream) on `state_reliable_back` at `telemetry_hz`,
   so the operator HUD can show what *arrived* — the operator only knows what
   it *sent*.

## Datachannels (this is the trickiest part)

CF Realtime bridges datachannels **publisher → subscriber, one direction
only**. That's why we need two reliable channels — one for each direction:

| Channel | Direction | Reliable? | What it carries |
|---|---|---|---|
| `cmd_unreliable` | operator → robot | no (unordered, 0 retransmits) | TwistStamped / Joy / PoseStamped LCM |
| `state_reliable` | operator → robot | yes | JSON: `ping`, `clock_report`, `video_stats`, `estop`, `camera_select`, `nav_goal` |
| `state_reliable_back` | robot → operator | yes | JSON: `pong`, `robot_telemetry`, `cmd_ack` |
| `map_unreliable` | robot → operator | no (lossy) | JSON: minimap occupancy grid + odom pose |

All four are **negotiated by SCTP id** (broker assigns; we never pick).

### SCTP id 0 reservation (the throwaway DC)

A plain `createDataChannel` auto-grabs SCTP id 1 at connect time — same id the
broker tends to assign `cmd_unreliable`. Collision → `createDataChannel(id=1)`
throws. So at offer time we pin a *throwaway* negotiated channel to id 0
(reserved, never handed out by the broker). It also forces an SCTP m-line into
the offer so the SFU has a transport to bind the real channels to.

**Do not close that channel.** Under MAX_BUNDLE the SCTP shares the one bundled
ICE/DTLS transport with the video track; closing the only datachannel risks
the transport.

## aiortc / Cloudflare quirks (do not regress)

These are **hard-won, not in any docs** — corresponding fixes are commented at
the call sites but the *why* lives here:

- **MAX_BUNDLE is mandatory.** aiortc 1.14's default (BALANCED) puts video and
  SCTP on separate ICE transports. CF Realtime publishes one bundled transport;
  the video one fails ICE and you get a black quad forever. Force
  `RTCBundlePolicy.MAX_BUNDLE` on `RTCConfiguration`.

- **`addTrack` BEFORE `createDataChannel`.** Otherwise the SCTP m-line is
  created without a transceiver and aiortc's bundle-collapse discards the
  shared transport. ICE never starts.

- **`_propagate_bundle_candidates`.** aiortc keys remote candidates by transport,
  and under one bundled transport the *last* m-section processed wins. CF puts
  `a=candidate` only on the video section; the empty SCTP section overwrites
  it → remote-candidates=0 → ICE stalls at "checking". The helper replicates
  the candidate block into every m-section that lacks one. **Do not remove.**

- **`makeXRCompatible()` on real hardware.** The operator side, not us, but
  worth knowing: `xrCompatible: true` at context creation is not enough on
  Quest — `await gl.makeXRCompatible()` is required before building the
  `XRWebGLLayer`.

## Reconnect

Operator-side reconnect is handled in the broker — it closes the stale
`state_reliable_back` push (CF `datachannels/close`, not in prose docs but in
the OpenAPI spec) before re-pushing. CF does **not** auto-reap datachannel
pushes (the 30s GC is media-only), so without that close, the long-lived robot
session accumulates half-dead pushes and the second bridge 502s with
`repeated_local_track_error`.

Robot-side auto-redial is not yet implemented and is gated behind TURN
landing first.
