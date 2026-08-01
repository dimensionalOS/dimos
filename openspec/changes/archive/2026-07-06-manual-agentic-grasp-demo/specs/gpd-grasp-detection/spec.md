## ADDED Requirements

### Requirement: GPD candidates can feed manual grasp execution demo
The GPD grasp detection workflow SHALL support an opt-in manual execution demo that consumes generated Grasp candidates through the agent-facing grasp-capable manipulation facade while preserving the existing candidate-generation-only demo behavior.

#### Scenario: Existing GPD visualization demo remains non-executing
- **WHEN** the existing candidate-generation-only GPD MuJoCo demo runs
- **THEN** it MUST continue to stop before robot motion, gripper actuation, pick/place motion, or trajectory execution

#### Scenario: Manual execution demo consumes generated candidates
- **WHEN** the manual execution demo runs and GPD returns one or more Grasp candidates for the configured Grasp target
- **THEN** the agent-facing grasp-capable facade MUST be able to cache those candidates for a subsequent `execute_grasp(candidate_index)` call

#### Scenario: Empty GPD result is reported clearly
- **WHEN** GPD returns no usable Grasp candidates during the manual execution demo
- **THEN** the agent-facing sequence MUST report a clear empty-result failure before attempting `execute_grasp(...)`
