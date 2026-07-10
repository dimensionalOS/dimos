## ADDED Requirements

### Requirement: Viser panel must select planning groups explicitly
The Viser planning panel MUST present and use planning-group selections for group-aware joint and pose planning controls.

#### Scenario: User selects a manipulator group
- **WHEN** a user selects a planning group in the panel
- **THEN** joint controls, pose target controls, and preview state apply to that group

### Requirement: Viser panel must preserve the normative reference interaction contract
The Viser panel MUST match the controls, labels, ordering, defaults, selection behavior, presets, joint sliders, pose gizmos, target ghosts, feasibility colors, preview behavior, execute gate, and clear behavior from `cc/spec/movegroup@0edb8d3dd`.

#### Scenario: Extracted panel is reviewed
- **WHEN** the extracted panel is compared with the normative reference
- **THEN** no user-visible deviation is present unless separately approved

### Requirement: Viser must preserve the cleaned planning-group API boundary
The Viser implementation MUST consume PR 4's explicit planning-group APIs without adding planner APIs. The only manipulation/visualization API change permitted is the group-native preview transaction required below.

#### Scenario: Backend adaptation is required
- **WHEN** reference UI code expects a different backend shape
- **THEN** the visualization backend translates the call without changing planner algorithms or control production code

### Requirement: Preview must be group-native and synchronized
The visualization protocol MUST expose `show_preview` and `hide_preview` over ordered planning-group IDs and `animate_plan` over one generated plan plus a total display duration. It MUST preview the plan as one validated transaction so all affected robots advance on a shared animation clock.

#### Scenario: Selected groups span multiple robots
- **WHEN** the user previews a fresh multi-robot plan
- **THEN** all affected preview ghosts advance together from the same generated plan

#### Scenario: Preview duration is omitted
- **WHEN** `ManipulationModule.preview_plan` is called without a duration
- **THEN** it sends the full generated plan to `animate_plan` with a total display duration of `1.0` second

#### Scenario: Visualization is called directly
- **WHEN** `animate_plan` is called without an explicit duration
- **THEN** its total display duration defaults to `3.0` seconds

#### Scenario: A plan controls a subset of robot joints
- **WHEN** a generated plan contains selected global joints
- **THEN** each affected robot's unselected joints come from one fixed pre-playback snapshot for every animation frame

#### Scenario: Preview input is invalid
- **WHEN** a plan contains unknown groups, malformed joint states, or cannot form complete affected-robot frames
- **THEN** preview rejects the transaction before showing any ghost

#### Scenario: Preview is replaced or stopped
- **WHEN** cancel, clear, close, or a new preview invalidates an active preview generation
- **THEN** old animation frames no longer mutate scene handles

### Requirement: Panel freshness must cover the full selected target set
The panel MUST associate asynchronous results with a selection epoch and MUST snapshot ordered joints for every affected robot when a plan becomes executable.

#### Scenario: Selection changes during evaluation
- **WHEN** a planning group is toggled while target evaluation is in flight
- **THEN** the stale result does not update target state or visuals

#### Scenario: A newer operation supersedes an older operation
- **WHEN** callbacks arrive out of order for the same selection epoch
- **THEN** only the callback whose operation sequence is current may update panel or scene state

#### Scenario: Execute spans multiple robots
- **WHEN** every selected group's ordered joints still match the stored all-robot snapshot
- **THEN** execute delegates the full stored plan without applying a robot filter

#### Scenario: Another caller replaces the stored module plan
- **WHEN** replacement occurs between the panel freshness check and execution
- **THEN** the external-plan replacement race is unavoidable

### Requirement: Joint-target feasibility must use existing PR 4 world APIs
The visualization backend MUST compose full per-robot target states and call `WorldMonitor.is_state_valid`; it MUST NOT add or copy manipulation-module FK or collision APIs.

#### Scenario: Multiple robots have joint targets
- **WHEN** each composed per-robot state is valid
- **THEN** the panel reports the conjunction of those checks without claiming simultaneous target-target collision proof

#### Scenario: Pose targets are evaluated
- **WHEN** selected pose groups and auxiliary groups form a target set
- **THEN** the backend uses PR 4 `inverse_kinematics` with coordinated collision checking, then validates each affected robot's composed full target state through `WorldMonitor.is_state_valid`

### Requirement: Viser preview must reflect group feasibility and current state
The Viser UI MUST show target feasibility and preview validity for the selected planning group.

#### Scenario: Target is infeasible
- **WHEN** target evaluation reports infeasible for the selected group
- **THEN** the target ghost and panel state indicate that the target cannot be executed safely

### Requirement: Viser execution must require a fresh matching plan
The Viser panel MUST prevent execution when no fresh plan matches the current selected robot/group state.

#### Scenario: Current state no longer matches preview
- **WHEN** the robot state changes after planning
- **THEN** the panel rejects execute until all selected-group snapshots match again
