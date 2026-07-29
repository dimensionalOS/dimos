# hosted-broker-session

## ADDED Requirements

### Requirement: Robot dials out to the broker
The robot SHALL establish its teleop session by dialing out to the broker (Cloudflare Realtime or LiveKit) over HTTPS/WebRTC, requiring no inbound ports or port forwarding on the robot's network.

#### Scenario: Registration behind NAT
- **WHEN** the robot starts a hosted blueprint on a NAT'd/cellular network with a valid `transports.broker.api_key`
- **THEN** it POSTs an SDP offer to the broker, receives an answer, and reaches `connected` state without any inbound connectivity

### Requirement: One session per robot
All broker-bound streams of a robot SHALL share a single WebRTC session owned by the per-process `BrokerProvider` singleton; blueprints MUST bind all broker transports to streams of one module per robot.

#### Scenario: Transports share the provider
- **WHEN** a blueprint binds cmd, state, state_back and video transports to one module's streams
- **THEN** all four ride one PeerConnection/session and the operator sees a single robot entry

### Requirement: Negotiated datachannel set
The session SHALL carry three broker-negotiated datachannels: `cmd_unreliable` (operator→robot, unordered, 0 retransmits), `state_reliable` (operator→robot, ordered), and `state_reliable_back` (robot→operator, ordered). SCTP ids are assigned by the broker via heartbeat acks; the robot MUST track id changes by closing and re-opening channels.

#### Scenario: Operator joins after robot
- **WHEN** an operator connects and the broker's heartbeat ack carries new SCTP ids
- **THEN** the robot opens the negotiated channels on those ids and commands begin flowing

### Requirement: Single sendonly video track
The session SHALL include exactly one sendonly video track, armed only once the connection is established so pre-connection frames are dropped.

#### Scenario: Frames before connection
- **WHEN** cameras publish frames before the PeerConnection reaches `connected`
- **THEN** no frames are queued or sent; delivery starts from "now" at connection time

### Requirement: Clock-sync ping answered inline
The provider SHALL answer operator `ping` messages (`{"type":"ping","client_ts":...}`) with a `pong` echoing `client_ts` plus a fresh `robot_ts`, sent on `state_reliable_back`, inline on the loop thread (not via pub/sub dispatch) to keep RTT samples jitter-free. Malformed pings MUST be dropped silently.

#### Scenario: RTT measurement
- **WHEN** the operator sends a well-formed ping
- **THEN** a pong with the same `client_ts` and current `robot_ts` returns on state_reliable_back

#### Scenario: Back channel not open
- **WHEN** a ping arrives while `state_reliable_back` is closed or absent
- **THEN** the pong is dropped (never rerouted onto `state_reliable`)

### Requirement: Operator loss signal
The provider SHALL inject a synthetic `{"type":"operator_lost"}` message into the state plane when the operator's command plane goes away, so robot modules can neutralize motion.

#### Scenario: Operator tab closes
- **WHEN** the operator disconnects mid-drive
- **THEN** the robot module receives `operator_lost` and stops/disengages motion
