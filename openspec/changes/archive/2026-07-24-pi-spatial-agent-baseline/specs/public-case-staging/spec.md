## ADDED Requirements

### Requirement: Canonical public-only case projection
The staging process SHALL materialize a canonical versioned `case.v1.json` projection containing only the selected Scene, Trajectory, Question, Snapshot, and Instance records, plus release identity, map-artifact identity, and staging-manifest/schema references. The staged bundle SHALL include the referenced map artifact and staging manifest/schema, but SHALL NOT include shared full JSONL records, oracle data, authoritative answers, private geometry, or other private answer material.

#### Scenario: Stage a public case
- **WHEN** a corpus case is selected for evaluation
- **THEN** the staged bundle contains the exact referenced public artifacts through `case.v1.json` and its referenced map/manifest/schema files, and an inspection finds no shared full JSONL, oracle, or private answer material

### Requirement: Stable staging provenance
The staged bundle SHALL identify the corpus release, projection version, and exact artifact identities or hashes used for the case, without adding answer-bearing fields to the public input.

#### Scenario: Verify staged inputs
- **WHEN** a reviewer compares a staged case with its corpus manifest
- **THEN** every selected artifact resolves to the manifest identity or hash and the staged question and instance remain answer-neutral
