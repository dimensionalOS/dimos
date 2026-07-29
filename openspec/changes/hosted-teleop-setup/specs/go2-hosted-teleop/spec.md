# go2-hosted-teleop

## ADDED Requirements

### Requirement: Drive safety on the unreliable wire
`cmd_vel` twists arriving on `cmd_unreliable` SHALL be dropped when stale (older than `cmd_stale_after_sec`, default 0.5s) or out-of-order (timestamp ≤ newest seen), and refused entirely while the E-STOP latch is set. The base's velocity deadman remains active underneath (no fresh twist → stop).

#### Scenario: Network hiccup reorders packets
- **WHEN** twists arrive out of timestamp order after a burst of loss
- **THEN** only monotonically newer twists reach the base; older ones are dropped

### Requirement: Sport command allow-list
Operator `sport_cmd` messages SHALL execute only names on the robot-side allow-list (StandDown, RecoveryStand, Sit, Hello, Stretch, Damp, FrontPounce, FrontJump, plus the StandReady combo); anything else is rejected with ok=false and no robot call. StandReady SHALL run the full drive-ready sequence (standup → RecoveryStand → BalanceStand → joystick on), never a raw single command.

#### Scenario: Disallowed command
- **WHEN** the operator sends `sport_cmd` with name "Backflip"
- **THEN** no robot RPC is issued and the ack reports ok=false

### Requirement: Serialized command execution with urgent bypass
Blocking operator commands SHALL execute off the transport thread on a single ordered worker with a bounded backlog (busy-reject past 4 pending). Damp/E-STOP commands MUST bypass the queue on a dedicated thread so a stop never waits behind a slow command (StandReady ≈ 3.6s).

#### Scenario: E-STOP behind a slow command
- **WHEN** StandReady is executing and the operator hits E-STOP
- **THEN** Damp executes immediately without waiting for StandReady to finish

### Requirement: Speed modes and rage boundary
`set_mode` SHALL accept normal/high/rage; normal and high differ only by operator-side scaling, and only crossing the rage on/off boundary toggles robot firmware. The rage-state check MUST run inside the serialized task to avoid races between rapid toggles.

#### Scenario: Rapid rage toggles
- **WHEN** the operator toggles rage on/off in quick succession
- **THEN** firmware toggles execute in order and tracked state matches firmware afterwards

### Requirement: Operator loss stops the base
On `operator_lost` the robot SHALL zero base motion (`stop_movement`) and clear the nonce cache (browser nonces restart per session); it SHALL additionally Damp only when `damp_on_operator_lost` is enabled. A link blip MUST NOT latch E-STOP.

#### Scenario: WiFi blip mid-patrol
- **WHEN** the operator link drops with default config
- **THEN** the base stops but does not go limp, and reconnecting allows immediate drive

### Requirement: Go2 cockpit state and overlays
Telemetry SHALL carry Go2 state (posture, rage, obstacle avoidance, head-LED brightness, battery soc) and, when configured, a map/odom overlay (compressed occupancy grid ≤16 KB per message at `map_hz`, robot pose at `odom_hz`) on `state_reliable_back` for the operator minimap.

#### Scenario: Minimap
- **WHEN** nav is running and map_hz > 0
- **THEN** the operator receives grid + pose updates within the datachannel message ceiling
