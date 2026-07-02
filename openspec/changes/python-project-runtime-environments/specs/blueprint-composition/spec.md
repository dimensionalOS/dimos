## ADDED Requirements

### Requirement: Blueprint composition preserves runtime metadata
Blueprint composition SHALL preserve runtime environment registrations and runtime placements from composed blueprints.

#### Scenario: Composed blueprint includes runtime metadata
- **GIVEN** one blueprint registers a runtime environment
- **AND** another blueprint contributes a module placement that references that runtime
- **WHEN** the blueprints are composed
- **THEN** the resulting blueprint contains the runtime registration and placement
- **AND** deployment can resolve the placement if all validation rules pass.

### Requirement: Disabled modules do not create active runtime demand
Blueprint deployment SHALL consider disabled modules when selecting active runtime placements.

#### Scenario: Disabled placed module in composed blueprint
- **GIVEN** a composed blueprint with a placed module
- **AND** that module is disabled before deployment
- **WHEN** deployment determines active runtime demand
- **THEN** the disabled module's placement does not require reconciliation or worker launch
- **AND** other active placed modules are unaffected.

### Requirement: Existing blueprint behavior remains compatible
Blueprints without runtime registrations or runtime placements SHALL behave as they did before this change.

#### Scenario: Blueprint has no runtime metadata
- **GIVEN** a blueprint with only regular Python modules
- **WHEN** the blueprint is built
- **THEN** modules deploy through the default Python worker path
- **AND** no runtime reconciliation is required.
