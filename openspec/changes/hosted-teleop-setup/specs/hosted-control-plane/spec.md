# hosted-control-plane

## ADDED Requirements

### Requirement: JSON state-plane dispatch
Robot connection modules SHALL dispatch `state_reliable` JSON messages by `type`, handling the universal set (`estop`, `estop_clear`, `operator_lost`, `camera_select`, `video_stats`, `clock_report`) in the shared plane and delegating unknown types to a robot-specific hook. Malformed JSON and non-JSON frames MUST be dropped without crashing the transport callback.

#### Scenario: Malformed payload
- **WHEN** the operator channel delivers `{broken` or binary garbage
- **THEN** the message is logged and dropped; subsequent messages still process

### Requirement: E-STOP latch semantics
An `estop` message SHALL latch the robot into a no-motion state before any robot RPC is issued, and the latch SHALL persist until an explicit `estop_clear`. Clearing the latch MUST NOT resume motion by itself — the operator must issue a fresh motion command (Go2) or re-engage (arm). A hosted robot without an implemented E-STOP path MUST fail at wiring time, not silently no-op.

#### Scenario: E-STOP during drive
- **WHEN** `{"type":"estop"}` arrives while the robot is moving
- **THEN** motion commands are refused immediately and the robot is made safe (Go2: Damp; arm: freeze at last target)

#### Scenario: Clear does not move
- **WHEN** `{"type":"estop_clear"}` arrives
- **THEN** the latch clears, no motion occurs, and the ack reports ok=true

### Requirement: Command acknowledgment with nonces
Every nonce-carrying operator command SHALL receive exactly one `cmd_ack` (`{"type":"cmd_ack","nonce":...,"ok":...}`) on `state_reliable_back` reporting success or failure — including rejections (busy, E-STOP latched, malformed). Duplicate nonces within the dedup window MUST re-ack the prior result instead of re-executing; duplicates of in-flight commands are dropped (the original acks).

#### Scenario: Browser re-send after missed ack
- **WHEN** the operator re-sends a command with the same nonce
- **THEN** the robot re-acks the recorded result without executing twice

### Requirement: Robot-authoritative telemetry
The robot SHALL push `robot_telemetry` frames on `state_reliable_back` at a configurable rate (default 3 Hz) containing command-plane stats and a `state` object (current camera selection, E-STOP latch, plus robot-specific state) so a (re)connecting operator's cockpit seeds from reality rather than optimistic defaults.

#### Scenario: Operator reconnects
- **WHEN** an operator reconnects while the robot is E-STOP-latched with cam2 selected
- **THEN** the first telemetry frame shows `estopped: true` and `cams: ["cam2"]` and the cockpit renders that state

### Requirement: Operator video stats relay
`video_stats` messages (browser `getStats()` samples) SHALL be parsed defensively and republished on a typed robot-side stream for recorders; malformed payloads are dropped with a warning.

#### Scenario: Recorder tap
- **WHEN** the operator reports fps/kbps via video_stats
- **THEN** a typed `VideoStats` message is published robot-side
