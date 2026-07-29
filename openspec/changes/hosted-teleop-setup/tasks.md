# Tasks — Hosted Teleop Setup

## 1. Robot-side foundation (landed on ruthwik/hostedteleop/2)

- [x] 1.1 Transport-swap plumbing: BrokerProvider session, CF/LiveKit transports, video track, inline ping responder
- [x] 1.2 Shared plane: `CameraMuxMixin` (mux, caps, latency stamp) + `HostedConnectionMixin` (state dispatch, acks, E-STOP latch, telemetry) with robot hooks
- [x] 1.3 `Go2HostedConnection` on the mixin: drive guards, sport allow-list, serialized executor + nonce dedup, rage/OA/LED, map/odom overlay
- [x] 1.4 Quest module reuse: `Hand` → quest_types, lazy web server, `ArmTeleopModule` as base for hosted arm
- [x] 1.5 `ArmHostedConnection`: engage/delta-pose over broker streams, arm E-STOP (freeze-in-place), operator-loss disengage
- [x] 1.6 Dual RealSense via `FrontCamera`/`WristCamera` subclasses + `teleop-hosted-xarm7-multicam` blueprint + registry
- [x] 1.7 Unit tests: Go2 command paths (45), arm station (17), broker ping (7), clock-sync legacy (9)
- [x] 1.8 Deprecation notes on `hosted_teleop_module.py` / `hosted_extensions.py`; READMEs updated

## 2. Hardware validation

- [ ] 2.1 Run `teleop-hosted-xarm7-multicam` with both RealSense units (serials pinned), verify cam1/cam2 select + both-view mux end-to-end
- [ ] 2.2 Measure uplink bitrate against the ~5 Mbps/robot budget (video_stats HUD); tune `video_max_width`/`video_max_fps` defaults if needed
- [ ] 2.3 Latency-stamp benchmark on the arm path (glass-to-glass), compare against Go2 numbers
- [ ] 2.4 E-STOP + operator-loss drills on the real arm (freeze-in-place, re-engage-from-zero)

## 3. Broker-side arm operator UI (dimensional-teleop repo)

- [ ] 3.1 Decide cockpit shape: extend Go2 cockpit shell vs WebXR-first arm page (design.md open question)
- [ ] 3.2 WebXR pose sender: controller poses + Joy on cmd_unreliable (LCM-encoded, left/right frame_ids)
- [ ] 3.3 Camera-select + E-STOP controls speaking the shared protocol; seed UI from robot telemetry (engaged/cams/estopped)
- [ ] 3.4 Command-plane HUD for the arm (pose rate/latency from robot_telemetry cmd stats)

## 4. Legacy migration and deletion

- [ ] 4.1 Migrate `teleop-hosted-xarm7` blueprint to `ArmHostedConnection` (validate single-cam path)
- [ ] 4.2 Drop `teleop-hosted-go2` (superseded by `teleop-hosted-go2-transport`)
- [ ] 4.3 Delete `hosted_teleop_module.py`, `hosted_extensions.py`, `test_clock_sync.py`; regenerate blueprint registry

## 5. Network robustness

- [ ] 5.1 TURN support (broker-issued credentials vs static config — design.md open question)
- [ ] 5.2 Robot-side auto-redial with backoff on broker/session loss
- [ ] 5.3 Multi-robot router validation (N dogs on one cellular uplink)
