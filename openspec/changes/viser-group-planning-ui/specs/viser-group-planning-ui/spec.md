## ADDED Requirements

### Requirement: Viser panel must select planning groups explicitly
The Viser planning panel MUST present and use planning-group selections for group-aware joint and pose planning controls.

#### Scenario: User selects a manipulator group
- **WHEN** a user selects a planning group in the panel
- **THEN** joint controls, pose target controls, and preview state apply to that group

### Requirement: Viser preview must reflect group feasibility and current state
The Viser UI MUST show target feasibility and preview validity for the selected planning group.

#### Scenario: Target is infeasible
- **WHEN** target evaluation reports infeasible for the selected group
- **THEN** the target ghost and panel state indicate that the target cannot be executed safely

### Requirement: Viser execution must require a fresh matching plan
The Viser panel MUST prevent execution when no fresh plan matches the current selected robot/group state.

#### Scenario: Current state no longer matches preview
- **WHEN** the robot state changes after planning
- **THEN** the execute action is rejected until a fresh matching plan is generated
