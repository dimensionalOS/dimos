## ADDED Requirements

### Requirement: Manual agent-facing grasp pipeline demo
The system SHALL provide a deterministic MuJoCo demo path that proves manual agent-facing tool calls can compose registered-object scanning, GPD grasp generation, and grasp execution without requiring an autonomous LLM loop.

#### Scenario: Manual sequence completes through tool surface
- **WHEN** the manual grasp demo is running with the configured sphere Grasp target visible
- **THEN** a caller MUST be able to invoke `scan_objects("sphere")`, `generate_grasps("sphere")`, and `execute_grasp(0)` through the agent-facing tool surface
- **AND** the sequence MUST route through the same facade methods intended for future agent/MCP callers

#### Scenario: Demo does not require autonomous LLM execution
- **WHEN** the required manual grasp demo or smoke test runs
- **THEN** it MUST NOT require an `McpClient`, model API key, or autonomous LLM decision loop

### Requirement: Gated sim smoke validation
The system SHALL include an opt-in MuJoCo/self-hosted smoke validation for the manual grasp pipeline.

#### Scenario: Smoke test validates pipeline completion
- **WHEN** the gated smoke test runs in an environment with the required MuJoCo and GPD runtime dependencies prepared
- **THEN** it MUST fail if the configured Grasp target cannot be registered
- **AND** it MUST fail if grasp generation returns no cached Grasp candidates
- **AND** it MUST fail if `execute_grasp(0)` does not complete its motion/gripper sequence successfully

#### Scenario: Smoke test does not require object lift success
- **WHEN** `execute_grasp(0)` completes in the smoke test
- **THEN** the test MUST NOT require a MuJoCo contact/object-lift assertion as its pass/fail condition

### Requirement: Manual demo documentation
The system SHALL document how to prepare and run the manual agent-facing grasp demo and how to assess visual demo quality.

#### Scenario: User follows manual command sequence
- **WHEN** a user follows the manual demo documentation after preparing the required runtime
- **THEN** the documentation MUST provide the command sequence for starting the demo and invoking the manual tool calls

#### Scenario: User checks Rerun visual outputs
- **WHEN** a user runs the manual demo with visualization enabled
- **THEN** the documentation MUST describe the expected Rerun checklist: registered Grasp target, GPD Grasp candidates, robot approach, gripper close, and lift/retract motion
