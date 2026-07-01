## ADDED Requirements

### Requirement: Orthogonal changes must be reviewed separately
Changes that are not necessary for planning-group semantics MUST be extracted into a separate review or explicitly excluded from the stack.

#### Scenario: Candidate file is unrelated to planning groups
- **WHEN** a file change is docs-only, generated-only, control-only, or repo-housekeeping-only
- **THEN** it is reviewed in this housekeeping change or reported as intentionally excluded

### Requirement: Generated files must match their source changes
Generated registry files MUST be included only with the source blueprint/config change that requires regeneration, or in a final generated-files PR.

#### Scenario: Blueprint registry changes are generated
- **WHEN** a generated registry file differs from `main`
- **THEN** the extraction task identifies the source change and validates regeneration
