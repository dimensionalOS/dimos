## MODIFIED Requirements

### Requirement: Versioned typed scene oracle view
The DimOS integration SHALL assemble a private, read-only, versioned `SceneOracleView` for benchmark generation from one synchronous DimSim runtime snapshot and one compatible semantic profile. The view SHALL carry explicit scene, upstream, profile, and semantic-schema revision fields and SHALL use strict typed records that reject unknown or malformed fields.

#### Scenario: Export a supported scene
- **WHEN** benchmark generation requests the oracle view for a coherently reset supported scene
- **THEN** the integration joins the live snapshot to the compatible profile and returns a typed view containing the scene identity, explicit revisions, frame contract, entities, regions, and provenance

#### Scenario: Reject an unsupported semantic schema
- **WHEN** the DimSim integration receives an oracle view with an unsupported schema version or malformed required record
- **THEN** it fails with an actionable compatibility error instead of silently omitting or coercing semantic facts
