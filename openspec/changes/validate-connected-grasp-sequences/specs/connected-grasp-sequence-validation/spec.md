## ADDED Requirements

### Requirement: Explicit-start RoboPlan planning
RoboPlan planning SHALL accept a valid explicit selected-joint start that differs from the authoritative live robot state. The requested start SHALL seed native planning, while unselected joints and other robots SHALL retain their latest live scene state. Planning MUST NOT mutate authoritative live state.

#### Scenario: Hypothetical start differs from live state
- **WHEN** a caller requests a path from a valid selected-joint start that differs from the live selected-joint state
- **THEN** RoboPlan attempts the path from the requested start and leaves the live state unchanged

#### Scenario: Unselected scene state is retained
- **WHEN** a selected planning group is planned from a hypothetical start
- **THEN** unselected joints, other robots, and non-target scene geometry retain their latest live values during collision checking

### Requirement: Shared safety-lift validation
The pick pipeline SHALL determine whether its existing safety lift is required before candidate evaluation. It SHALL dry-run a required lift exactly once and SHALL use the successful lift endpoint as the common start for candidate validation.

#### Scenario: Required safety lift is feasible
- **WHEN** the end effector requires a safety lift and a connected lift plan succeeds
- **THEN** every grasp candidate is evaluated from the planned lift endpoint without executing the lift during validation

#### Scenario: Required safety lift is infeasible
- **WHEN** the end effector requires a safety lift and no lift plan succeeds
- **THEN** the pick aborts in `PREPARE` with `PLANNING_FAILED` and does not attribute the failure to any candidate

### Requirement: Connected candidate sequence
The pipeline SHALL accept a grasp candidate only when connected plans succeed in order from the shared start to pre-grasp, from the pre-grasp path endpoint to grasp, and from the grasp path endpoint to retreat. Each IK solve SHALL be seeded from the preceding endpoint, and each plan SHALL begin at that same endpoint.

#### Scenario: All candidate segments connect
- **WHEN** pre-grasp, grasp, and retreat each have a collision-free path beginning at the preceding path endpoint
- **THEN** the candidate passes motion-feasibility validation

#### Scenario: Individually reachable poses are disconnected
- **WHEN** all three target poses have collision-free IK solutions but any required connecting path fails
- **THEN** the candidate is rejected without robot motion

#### Scenario: Higher-ranked candidate has a blocked segment
- **WHEN** a higher-ranked candidate has a failed connected segment and a lower-ranked candidate has a complete connected sequence
- **THEN** the pipeline selects the lower-ranked candidate without moving for the rejected candidate

### Requirement: Motion-free validation and fresh execution planning
Candidate validation SHALL NOT store an execution plan, dispatch robot motion, or issue gripper commands. After selection, every physical motion segment SHALL be replanned from freshly measured state before execution.

#### Scenario: Candidate is selected
- **WHEN** a candidate passes connected dry-run validation
- **THEN** no validation path is executed and approach planning begins from fresh measured state

#### Scenario: Execution replan fails after motion begins
- **WHEN** any execution-time replan fails after the robot has moved
- **THEN** the transaction stops at that phase and does not attempt a different grasp candidate

### Requirement: Live-scene MVP collision contract
Each validation segment SHALL use the latest available planning scene with the selected target suppressed and non-target obstacles active. The MVP SHALL NOT claim atomic scene validation, attached-object clearance, or separate open-versus-closed finger-sweep validation.

#### Scenario: Non-target scene changes during validation
- **WHEN** a non-target obstacle update arrives between two segment planning calls
- **THEN** the later segment uses the updated scene without restarting or freezing the complete validation sequence

#### Scenario: Retreat validation succeeds
- **WHEN** the grasp-to-retreat path is collision-free for the robot and gripper state represented in the planning scene
- **THEN** retreat passes the MVP gate without asserting clearance for attached target geometry

### Requirement: Stage-level rejection reporting
The pipeline SHALL report candidate rejection by the first failed sequence stage using `pre_grasp_infeasible`, `grasp_infeasible`, or `retreat_infeasible`. Backend-specific IK and planner outcomes SHALL remain diagnostic details and SHALL NOT become new public rejection categories in this change.

#### Scenario: Grasp segment planning fails
- **WHEN** pre-grasp succeeds and the pre-grasp-to-grasp segment fails through IK, timeout, collision, or no path
- **THEN** the pipeline increments `grasp_infeasible` and records the detailed cause in diagnostic logs
