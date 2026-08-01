## 1. Models and Policy Configuration

- [x] 1.1 Create the DimSim benchmark-generation package and define versioned generator, predicate, template, frame, clearance, and comparison-margin configuration constants.
- [x] 1.2 Add strict frozen `SceneOracleView` models for scene/reset revisions, frame contract, embodiment and spawn, navigation geometry, entities, semantic properties, regions, and grouped field provenance.
- [x] 1.3 Add canonical oracle-view serialization and digest utilities with tests proving stable bytes for equivalent content and changed digests for truth-bearing changes.
- [x] 1.4 Add strict public manifest/task models and private contract, expected-outcome, source-provenance, diagnostic, and generation-report models using discriminated unions.
- [x] 1.5 Generate or expose JSON Schemas for every persisted corpus record and test rejection of unknown fields, malformed references, unsupported schema versions, and invalid answer types.

## 2. DimSim Oracle Provider

- [x] 2.1 Define a private `SceneOracleProvider` protocol that returns one validated in-memory `SceneOracleView`, plus an in-memory fixture implementation for unit tests.
- [x] 2.2 Add a private `SceneClient` oracle command that negotiates the supported semantic-schema version and returns one coherent reset payload without exposing it as an agent skill or normal observation.
- [x] 2.3 Add a versioned DimOS apartment semantic profile with exact stable-ID
  bindings, aliases, power-state mapping, navigation policy, canonical spawn,
  and provenance, and combine it with live DimSim transforms and footprints.
- [x] 2.4 Pin the compatible unmodified upstream DimSim revision, validate the
  cached checkout, and record the upstream and profile revisions in
  oracle-view provenance.
- [x] 2.5 Add contract tests that run the live provider against the compatible
  apartment scene and fail clearly for missing IDs, unsupported states,
  incoherent snapshot geometry, or a revision/profile mismatch.

## 3. Deterministic Candidate Generation

- [x] 3.1 Implement exact semantic entity resolution and deterministic ordering without display-title substring matching, including cardinality and missing-provenance diagnostics.
- [x] 3.2 Implement validated 2-D footprint geometry, outer-edge distance, polygon surface distance, stopping-band construction, collision clearance, and canonical-spawn reachability utilities with boundary tests.
- [x] 3.3 Implement the bathtub `navigate-within-outer-footprint` generator with the one-metre threshold and destination feasibility gates.
- [x] 3.4 Implement the television `entity-state` generator with authoritative `power` vocabulary validation and typed `ON`/`OFF` outcomes.
- [x] 3.5 Implement the scene-scoped `count-semantic-class` generator for exact class `dining-chair`, including exclusion of `work-chair`.
- [x] 3.6 Implement the sofa-anchored `argmin-surface-distance` generator for bathtub versus television with a versioned non-zero stability margin.
- [x] 3.7 Implement controlled public templates, semantic task identity payloads that exclude expected answers, stable opaque IDs, and fixed four-category smoke ordering.
- [x] 3.8 Add rejection tests for duplicate entities, missing semantic classes, visual-only state, invalid footprints, unreachable destinations, center-distance disagreement, and comparison ties or near-ties.

## 4. Corpus Writing and Validation

- [x] 4.1 Implement canonical writing of `manifest.json`, `public/tasks.jsonl`, `oracle/task_contracts.jsonl`, `oracle/expected_outcomes.jsonl`, and `oracle/generation_report.json`.
- [x] 4.2 Implement one-to-one public/contract/outcome reference validation and reconstruct every stable ID from its declared semantic identity payload.
- [x] 4.3 Implement release-blocking validation for schema validity, exact category cardinality, answer typing, geometry and reachability gates, comparison stability, source digest/provenance, and deterministic regeneration.
- [x] 4.4 Implement recursive public-package leakage scanning for expected values, private entity bindings, executable contracts, oracle-view digests, semantic provenance, and private paths.
- [x] 4.5 Ensure failed generation writes deterministic private diagnostics but never marks the manifest complete or leaves a partially publishable public release.
- [x] 4.6 Add round-trip loader tests proving the public root can be distributed independently and the full corpus joins only through opaque task IDs.

## 5. Apartment Smoke Corpus

- [x] 5.1 Add a minimal typed apartment oracle fixture sourced from the authoritative DimSim contract, including one bathtub, one television, four dining chairs, one work chair, one sofa, navigable geometry, and canonical reset state.
- [x] 5.2 Add a smoke-generation entrypoint that produces exactly the four approved public tasks and no additional retained tasks.
- [x] 5.3 Add golden assertions for the terminal bathtub predicate, television enum answer, dining-chair integer answer, and sofa-relative entity-choice answer.
- [x] 5.4 Verify byte-equivalent regeneration from the same fixture and verify that state or predicate-policy changes update only the intended identities, outcomes, digests, and provenance.
- [x] 5.5 Run the smoke generator against the pinned live DimSim apartment
  snapshot plus semantic profile and require the same schema, cardinality,
  objectivity, and leakage gates as the fixture path.

## 6. Documentation and Quality Gates

- [x] 6.1 Document the private `SceneOracleView` contract, generator entrypoint,
  corpus layout, policy versioning, rejection diagnostics, and distinction
  between fixture data, the live simulator snapshot, and the DimOS semantic
  profile.
- [x] 6.2 Document that scene generation, evaluated-agent execution, Pi-baseline integration, submission, scoring, and result publication remain outside this change.
- [x] 6.3 Run focused unit and integration tests for the new package,
  `git diff --check`, and the repository's required formatting, linting, typing,
  and pre-commit checks.
