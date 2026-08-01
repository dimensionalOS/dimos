## ADDED Requirements

### Requirement: Registered object metadata contract
The system SHALL provide a typed `RegisteredObject` contract for cross-module object metadata used to identify a Grasp target.

#### Scenario: Registered object includes target bounds metadata
- **WHEN** object scene registration returns a registered object
- **THEN** the record includes object id, display name if available, center, size, frame id, and timestamp

#### Scenario: Registered object is typed
- **WHEN** a module consumes registered object metadata
- **THEN** it receives a typed `RegisteredObject` value rather than an unstructured dictionary

### Requirement: Object lookup by object id
The object scene registration capability SHALL provide a typed lookup by stable object id.

#### Scenario: Object id resolves
- **WHEN** a caller requests a registered object by an object id that is known to object scene registration
- **THEN** the system returns the matching `RegisteredObject` metadata

#### Scenario: Object id is unknown
- **WHEN** a caller requests a registered object by an unknown object id
- **THEN** the system returns no object and reports a clear no-target outcome to the caller

### Requirement: User-facing object grasp orchestration
The grasp orchestration layer SHALL expose a user-facing operation that generates grasps for a non-ambiguous object id.

#### Scenario: Generate grasps for object id
- **WHEN** a caller requests grasp generation for an object id and that id resolves to a registered object
- **THEN** the grasp orchestration layer passes the registered object target bounds to the TSDF grasp generator

#### Scenario: Preserve pointcloud grasping compatibility
- **WHEN** existing pointcloud grasp APIs are called by object name or object id
- **THEN** their existing behavior remains available and is not replaced by target-conditioned TSDF grasping
