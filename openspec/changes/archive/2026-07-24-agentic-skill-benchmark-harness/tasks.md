## 1. Contracts and Configuration

- [x] 1.1 Confirm Structured3D access and derivative-artifact distribution terms and document whether generated public artifacts may be redistributed or must remain gated.
- [x] 1.2 Define versioned JSON/JSONL schemas for manifests, scenes, trajectories, questions, snapshots, instances, source provenance, geometry, topology, answers, and review overrides.
- [x] 1.3 Define stable opaque ID generation, relative-path rules, artifact hashing, public/oracle separation, and immutable release versioning.
- [x] 1.4 Calibrate and freeze the v1 square footprint, safety margin, coordinate conventions, mapper configuration, clean profile, two noisy profiles, and deterministic seed policy.

## 2. Structured3D Oracle Import

- [x] 2.1 Implement a Structured3D scene loader that normalizes source geometry into the benchmark's metric right-handed frame and records private source provenance.
- [x] 2.2 Extract walkable floor regions, blocked geometry, explicit openings, and candidate room regions while rejecting unsupported floors, stairs, closets, inaccessible regions, and open-plan subdivisions.
- [x] 2.3 Build the private room-opening graph and validate unique room membership, symmetric adjacency, direct-opening semantics, and node degrees.
- [x] 2.4 Implement continuous square-footprint pose, fixed-yaw translation, and in-place rotation collision oracles with safety margin and boundary-contact collision semantics.
- [x] 2.5 Add tolerance sweeps and uncertainty-band rejection for collision, opening, and room-boundary candidates.

## 3. Offline Map Generation

- [x] 3.1 Implement deterministic full-coverage trajectory generation with one retained trajectory per scene.
- [x] 3.2 Implement lidar raycasting with finite range, angular resolution, and physical occlusion from Structured3D geometry.
- [x] 3.3 Implement seeded range noise, beam dropout, finite-coverage behavior, and mild temporally correlated pose drift as pre-mapping noise profiles.
- [x] 3.4 Feed clean and noisy world-frame observations through the real DimOS `VoxelGridMapper` and capture its final `PointCloud2` output.
- [x] 3.5 Serialize each canonical map as `global_map.pc2.lcm` and emit `snapshot.json` with variant-specific terminal pose, mapper provenance, profile, seed, frame contract, and SHA-256 hash.
- [x] 3.6 Verify deterministic regeneration, PointCloud2 decoding, frame alignment, units, handedness, yaw, and transform round trips on asymmetric calibration scenes.

## 4. Question and Bundle Generation

- [x] 4.1 Implement versioned executable definitions and controlled text templates for all seven predicates.
- [x] 4.2 Implement balanced candidate sampling for positive, negative, and count answers while rejecting ambiguous room assignments, malformed openings, and boundary-sensitive geometry.
- [x] 4.3 Generate approximately two physical questions per predicate per retained scene with stable question IDs and one private oracle answer per question.
- [x] 4.4 Generate one public instance per clean or noisy map variant, including variant-specific neutral marker and pose coordinates without exposing room or answer metadata.
- [x] 4.5 Write the agreed hierarchical public/oracle corpus bundles and verify all internal references use valid relative paths and stable IDs.

## 5. Validation and Tests

- [x] 5.1 Add schema, foreign-reference, required-cardinality, hash, PointCloud2 decode, and public/oracle package-separation tests.
- [x] 5.2 Add independent or tolerance-perturbed tests for continuous collision predicates, including collision-free endpoints with colliding intermediate sweeps.
- [x] 5.3 Add room-graph tests for unique membership, adjacency symmetry, distinct-room direct connections, and neighbor degree consistency.
- [x] 5.4 Add paired-variant tests enforcing one shared physical answer, exactly three instances, complete variant-specific coordinates, and one scene-level split.
- [x] 5.5 Add determinism, scene-disjoint split, public leakage, balanced-label, and nuisance-correlation validation checks with actionable failure reports.
- [x] 5.6 Add release validation that refuses to mark a corpus complete when any mandatory invariant fails.

## 6. Read-Only Viser Inspection

- [x] 6.1 Implement a corpus loader that selects scenes, trajectories, questions, variants, and instances and optionally joins the private oracle root.
- [x] 6.2 Build a read-only Viser view for the PointCloud2 map, terminal pose, square footprint, neutral markers, predicate-specific translation or rotation overlays, and exact question text.
- [x] 6.3 Add optional oracle geometry and topology toggles plus previous/next instance and variant navigation without record-editing or review-state controls.
- [x] 6.4 Add automated loader checks proving the viewer can open one valid instance of every predicate and all three variants without modifying corpus files.

## 7. Blocking Smoke-Generation Gate

- [x] 7.1 Add a dedicated smoke-generation command that uses a minimal development-scene subset and writes a disposable public/oracle smoke corpus.
- [x] 7.2 Make smoke generation retain at least one valid physical question for each of the seven predicates and emit its clean, noisy-01, and noisy-02 instances.
- [x] 7.3 Run schema, artifact decode/hash, coordinate, oracle, pairing, topology, collision, leakage, and Viser-load validations against the smoke corpus.
- [x] 7.4 Add an explicit gate that blocks the 30-scene pilot-generation command unless every predicate has smoke coverage and every smoke validation passes; report missing predicates and failed checks.
- [x] 7.5 Execute the smoke gate and record a passing smoke validation report before starting pilot generation.

## 8. Pilot Corpus Generation

- [x] 8.1 Generate the 10-scene development split only after the smoke gate passes, inspect representative instances, and correct recurring generator-policy defects.
- [x] 8.2 Generate the 20-scene held-out split with the frozen schemas, predicate contracts, mapper configuration, noise profiles, and seed policy.
- [x] 8.3 Apply isolated manual exclusions or corrections only through `review_overrides.jsonl` and regenerate the release if review exposes a recurring defect.
- [x] 8.4 Run the complete release validator, confirm 30 scene-disjoint scenes and approximately 1,170 map-question instances, and write the immutable release manifest and validation report.
- [x] 8.5 Document corpus generation, validation, Viser inspection, review-override usage, known limitations, and the deferred agent-input and scoring work.
