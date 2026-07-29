# Hosted Teleop Setup

## Why

Operating robots over the internet (not LAN) requires a dial-out architecture — robots behind NAT/cellular connect out to a broker (Cloudflare Realtime / LiveKit SFU), and browser/VR operators reach them through it. This branch builds the complete robot-side stack for that: video, commands, safety, and telemetry over one WebRTC session per robot, for both the Go2 quadruped and coordinator-driven arms (xArm7 with dual RealSense cameras).

## What Changes

- Transport-swap pattern: `BrokerProvider` (per-process singleton) owns the broker session; blueprints bind `Cloudflare*`/`LiveKit*` transports to the streams of ONE module per robot. Replaces the legacy self-connecting `HostedTeleopModule` (deprecated, pending deletion).
- Shared hosted control plane (`HostedConnectionMixin` + `CameraMuxMixin`): state_json dispatch (estop / camera_select / video_stats / clock_report), cmd_ack, E-STOP latch, telemetry loop, multi-camera mux with fps/width caps and optional latency stamping.
- `Go2HostedConnection`: drive (cmd_vel with stale/reorder drop), allow-listed sport commands, rage mode, obstacle avoidance, head LED, serialized command executor with nonce dedup, map/odom overlay for the operator minimap.
- `ArmHostedConnection`: WebXR controller poses → engage/delta-pose loop → ControlCoordinator TeleopIK task routing; analog triggers → gripper; dual RealSense (front + wrist) as `FrontCamera`/`WristCamera` subclasses feeding the cam1/cam2 mux.
- Blueprints: `teleop-hosted-go2-transport`, `-livekit`, `-multicam`, `teleop-hosted-xarm7`, `teleop-hosted-xarm7-multicam`.
- Remaining (not yet implemented): broker-side arm operator UI (WebXR pose sender + camera-select), legacy module migration + deletion, TURN support, robot-side auto-redial.

## Capabilities

### New Capabilities
- `hosted-broker-session`: robot ⇄ broker WebRTC session — dial-out registration, negotiated datachannels (cmd_unreliable / state_reliable / state_reliable_back), single sendonly video track, heartbeats, clock-sync ping/pong, reconnect behavior.
- `hosted-control-plane`: operator ⇄ robot message protocol — E-STOP latch semantics, cmd_ack with nonces, camera_select, video_stats relay, robot-authoritative telemetry (cockpit state seeding).
- `hosted-camera-mux`: multi-camera composition into the single video track — selection, side-by-side compositing, publish-side fps/width caps, glass-to-glass latency stamping.
- `go2-hosted-teleop`: Go2-specific operator features — drive safety (stale/out-of-order drop, deadman), sport-command allow-list, rage mode, serialized command execution, map/odom overlay.
- `arm-hosted-teleop`: arm-specific operator features — engage/delta-pose control, coordinator task routing, gripper via analog triggers, dual-RealSense wiring, arm E-STOP (freeze-in-place) semantics.

### Modified Capabilities
<!-- none — openspec/specs/ is empty; all capabilities are new -->

## Impact

- Robot-side code: `dimos/teleop/hosted/` (command modules, mux, stats, blueprints), `dimos/protocol/pubsub/impl/webrtc/` (BrokerProvider, video transports), `dimos/robot/unitree/go2/`, `dimos/control/` (coordinator teleop tasks).
- Broker repo (`dimensional-teleop`): operator web UI — Go2 cockpit exists; arm cockpit is pending work.
- Deprecations: `hosted_teleop_module.py` / `hosted_extensions.py` deleted once `teleop-hosted-go2` / `teleop-hosted-xarm7` migrate to the transport-swap modules.
- Network requirements: ~2-5 Mbps uplink per robot; outbound-only (no inbound ports); TURN not yet supported.
