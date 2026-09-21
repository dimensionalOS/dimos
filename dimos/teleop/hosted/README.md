# Remote Teleop

Robot dials out to the dimensional-teleop broker —
no inbound ports needed. The browser/WebXR operator connects through the broker;
commands arrive over WebRTC datachannels, robot video goes out as a WebRTC
track.

## Files

The session (dial-out, datachannel lifecycle, video track) is owned by the
per-process `BrokerProvider`
(`dimos/protocol/pubsub/impl/webrtc/providers/`); blueprints bind
`Cloudflare*` transports to the streams of several small, per-concern modules
that all run in one worker so everything shares that single session (the
`GO2Connection` driver runs in a second worker — `n_workers=2`):

- **`go2_command.py`** — `Go2CommandModule`: operator command / E-STOP dispatch
  and the manual-drive guard. Reaches the driver over `@rpc` (`GO2Connection`).
- **`arm_command.py`** — `ArmCommandModule`: WebXR controller poses and browser
  EE-twist → ControlCoordinator tasks (stale/reorder/future-stamp guarded).
- **`camera_mux.py`** — `CameraMuxModule`: N cameras → one composited, capped
  video track (operator-selectable views).
- **`map_compress.py`** — `MapCompressModule`: costmap + odom → the minimap
  datachannel (coarsened, PNG-encoded, kept under a 32 KB budget — CF drops
  messages above ~64 KB).
- **`hosted_stats.py`** — `HostedStatsModule`: telemetry frame, command acks,
  and command-link latency/rate stats.
- **`go2_audio_bridge.py`** — operator microphone PCM → the Go2 Pro speaker
  through the existing Unitree WebRTC audio-hub/megaphone APIs.
- **`command_executor.py`** — `SerializedCommandExecutor`: serializes blocking
  driver commands with nonce dedup and a safety-epoch fence (E-STOP aborts).
- **`robot_type.py`** — `RobotType`: carried in the broker session config so
  the operator page auto-selects the right view (drive vs arm).
- **`blueprints/cloudflare.py`** — wires the above into the four hosted
  blueprints: `teleop-hosted-go2-transport` / `-multicam` (`n_workers=2`) and
  `teleop-hosted-xarm6` / `-xarm7` (`n_workers=1`, coordinator + dual cams).

The operator HTML lives in the dimensional-teleop broker repo (`web/`).

## How a session connects

1. Robot creates an `RTCPeerConnection` (MAX_BUNDLE, **must**),
   `addTrack(video)`, adds a recvonly audio transceiver,
   forwards decoded operator PCM to the Go2 audio bridge, and opens a
   throwaway negotiated DataChannel on SCTP id 0, creates an offer, gathers
   ICE non-trickle.
2. `POST /api/v1/sessions` to the broker with the offer. Broker creates a CF
   session, returns the answer + a `session_id` keyed off the robot.
3. SDP answer's candidates are propagated across bundled m-sections (aiortc
   workaround) before `setRemoteDescription`.
4. A heartbeat task polls `/sessions/{id}/heartbeat`; each ack carries the SCTP
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

## Go2 speaker support

Hosted Go2 blueprints default the audio bridge to `speaker=auto`. The first
operator audio batch probes whether the Unitree audio-hub API responds. This is
not a physical speaker test: Go2 Air and a Pro with a failed or muted speaker may
still return success. Set `speaker=enabled` only for inventory known to have a
working speaker, and set `speaker=disabled` for Go2 Air. The routing decision is
local to the bridge and is not treated as hardware-health telemetry.

```bash
# Go2 Pro (skip probing)
dimos run teleop-hosted-go2-transport -o go2audiobridgemodule.speaker=enabled

# Go2 Air (never call the firmware audio hub)
dimos run teleop-hosted-go2-transport \
  -o go2audiobridgemodule.speaker=disabled
```

## Datachannels

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

This channel must stay open. Under MAX_BUNDLE the SCTP shares the one bundled
ICE/DTLS transport with the video track; closing the only datachannel risks
the transport.

## Reconnect

Operator-side reconnect is handled in the broker — it closes the stale
`state_reliable_back` push (CF `datachannels/close`) before re-pushing. CF does
**not** auto-reap datachannel
pushes (the 30s GC is media-only), so without that close, the long-lived robot
session accumulates half-dead pushes and the second bridge 502s with
`repeated_local_track_error`.

Robot-side auto-redial is not yet implemented (a follow-up). TURN has landed
(`_fetch_ice_servers` fetches broker-minted creds, STUN fallback), so it is no
longer blocked on that.
