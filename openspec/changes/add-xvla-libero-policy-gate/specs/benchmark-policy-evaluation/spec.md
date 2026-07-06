## ADDED Requirements

### Requirement: X-VLA LIBERO synchronous benchmark gate
The system SHALL include an X-VLA LIBERO synchronous benchmark gate that validates `lerobot/xvla-libero` through the fast lockstep benchmark policy evaluation path before module-native live stream acceptance is considered complete.

#### Scenario: X-VLA benchmark gate runs the real policy
- **WHEN** a developer runs the X-VLA LIBERO synchronous benchmark gate with compatible LeRobot dependencies and prepared LIBERO assets
- **THEN** the gate loads the actual `lerobot/xvla-libero` policy rather than using fake, fixed, or VLA-JEPA policy actions as the acceptance path

#### Scenario: X-VLA benchmark gate preserves lockstep evaluation ownership
- **WHEN** the X-VLA synchronous benchmark gate evaluates an episode
- **THEN** benchmark evaluation owns runtime reset, stream snapshot collection, policy inference call, action adaptation, runtime step, scoring, artifacts, and cleanup without requiring ControlCoordinator policy chunk execution

#### Scenario: X-VLA benchmark gate enforces 10-episode success
- **WHEN** the 10-episode X-VLA synchronous benchmark gate over `libero_object` completes without setup or contract aborts
- **THEN** the gate passes only if the recorded success rate is greater than `0.50`

#### Scenario: X-VLA benchmark artifacts identify policy and stage
- **WHEN** the X-VLA synchronous benchmark gate runs
- **THEN** it writes rollout artifacts that identify the X-VLA policy family, checkpoint, benchmark stage, episode count, success count, success rate, threshold, and pass/fail result
