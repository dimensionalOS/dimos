## ADDED Requirements

### Requirement: Deliver dependency-ordered working PR stages

The change SHALL be delivered as a linear stack of working PRs covering generation, packaging, standalone viewer recordings, runtime integration, actual recording/replay, and final dependency removal in that order. Each PR SHALL depend only on its predecessors. The generation stage SHALL resolve backend conformance and the Rust raw transport removal feasibility before the application cutover proceeds. A stage subdivided for reviewability SHALL retain all acceptance requirements for every resulting PR.

#### Scenario: Review an intermediate branch
- **WHEN** a reviewer checks out any PR branch and its predecessors
- **THEN** that stage's demonstrated workflow runs without requiring a later PR
- **AND** all preceding stage demos remain runnable

### Requirement: Every PR includes a human-facing demo and automated tests

Each PR SHALL include a documented, runnable, hardware-free human demo in addition to automated tests. The demo SHALL show real observable data or behavior, not merely a test-suite pass result. Documentation SHALL include setup, exact commands, expected visible results, teardown, and known limitations. PR review evidence SHALL include a terminal capture or screen recording and the automated test results.

#### Scenario: An infrastructure stage is submitted
- **WHEN** the packaging PR is ready for review
- **THEN** a reviewer can run a separate consumer application, add a field, rebuild all three languages, and observe that field in exchanged messages
- **AND** the PR includes captured evidence and independent automated package/codec checks

#### Scenario: Automated tests pass without a demo
- **WHEN** a stage passes its tests but lacks a runnable human-facing demo or its review evidence
- **THEN** that stage is not complete

### Requirement: Use a cumulative example to demonstrate the user workflow

Stages SHALL evolve one small example project using a custom message with a standard dependency, synthetic image data, and a moving pose. The demos SHALL progress from a visible three-language file relay to an external consumer, a viewer-ready MCAP, a live blueprint on both transports, actual recording/replay, and a clean final installation without the old repository.

#### Scenario: Final acceptance
- **WHEN** a reviewer follows the final example from a clean environment
- **THEN** they can define a message, generate and build all three consumers, run them on LCM and Zenoh, record and replay them, and inspect the recording in both viewers
- **AND** generation and ordinary runtime use require neither ROS installation nor a `dimos-lcm` PR or dependency
