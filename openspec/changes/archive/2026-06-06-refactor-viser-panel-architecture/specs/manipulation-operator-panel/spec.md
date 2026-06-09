## ADDED Requirements

### Requirement: Stable panel compatibility boundary during internal refactors
The system SHALL preserve the optional manipulation operator panel's public launch, import, configuration, and safety behavior while reorganizing internal implementation details.

#### Scenario: Existing panel launch surfaces continue to work
- **GIVEN** a user has installed the optional Viser manipulation panel dependencies
- **WHEN** the user launches the panel through the existing companion entrypoint or an existing blueprint that includes the panel module
- **THEN** the launch surface accepts the same configuration fields as before
- **AND** the panel connects to the manipulation stack through the same public manipulation control surface

#### Scenario: Existing developer imports remain stable
- **GIVEN** developer code imports the documented Viser panel module, config, or blueprint symbols
- **WHEN** the panel implementation is reorganized into internal collaborators
- **THEN** those documented imports continue to resolve
- **AND** callers do not need to import the internal backend, GUI, scene, controller, or animation helpers

#### Scenario: Safety gates remain unchanged
- **GIVEN** the panel implementation has been reorganized internally
- **WHEN** an operator plans, previews, executes, cancels, or clears a plan
- **THEN** execution remains gated by the same operator opt-in and fresh-plan checks
- **AND** no new robot-facing action is enabled solely because of the internal refactor
