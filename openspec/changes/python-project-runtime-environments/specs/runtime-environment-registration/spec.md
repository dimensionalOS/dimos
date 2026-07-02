## ADDED Requirements

### Requirement: Register named runtime environments on blueprints
Blueprint authors SHALL be able to register named runtime environments on a blueprint before deployment resolves runtime placements.

#### Scenario: Blueprint registers one runtime environment
- **GIVEN** a blueprint that declares a runtime environment named `detector-runtime`
- **WHEN** the blueprint is built
- **THEN** deployment can resolve runtime placements that reference `detector-runtime`
- **AND** unplaced modules continue to deploy through the default Python runtime.

#### Scenario: Placement references unknown runtime
- **GIVEN** a blueprint with a runtime placement referencing `missing-runtime`
- **WHEN** deployment validates runtime placement
- **THEN** deployment fails before launching module workers
- **AND** the error identifies the unknown runtime name.

### Requirement: Reject duplicate runtime project paths
Runtime environment registration SHALL reject distinct runtime names that refer to the same canonical Runtime Project path.

#### Scenario: Two runtime names use one project path
- **GIVEN** runtime environments named `detector-a` and `detector-b`
- **AND** both refer to the same canonical Runtime Project path
- **WHEN** the environments are registered or merged through blueprint composition
- **THEN** registration fails
- **AND** the error identifies both conflicting runtime names and the duplicate project path.

### Requirement: Preserve runtime registrations through blueprint composition
Blueprint composition SHALL merge runtime environment registrations while preserving validation rules.

#### Scenario: Composed blueprints contribute runtime environments
- **GIVEN** two blueprints with distinct runtime environment registrations
- **WHEN** they are composed
- **THEN** the composed blueprint exposes both runtime environments for deployment
- **AND** duplicate runtime names or duplicate Runtime Project paths are rejected deterministically.
