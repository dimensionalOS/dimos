## ADDED Requirements

### Requirement: Gate publication on native-session verification
Prediction and score SHALL remain provisional until the retained native session and its receipt pass verification. An evidence failure MUST make the attempt terminal `failed`, and no canonical prediction or score SHALL be published; any valid partial session and failure evidence SHALL remain privately retained.

#### Scenario: Publish after a complete valid session
- **WHEN** the session, receipt, prompt bindings, and required sidecars pass verification
- **THEN** the attempt may publish its canonical prediction and score and mark the evidence complete

#### Scenario: Fail the evidence gate
- **WHEN** session verification fails after a provisional prediction or score exists
- **THEN** the attempt becomes terminal `failed`, the provisional values remain unpublished canonically, and valid partial-session and failure evidence remain private

### Requirement: Reject invalid or unsafe session evidence
Verification SHALL reject a wrong Pi package or session version, malformed JSONL, invalid Pi header/tree/message integrity, a receipt mismatch, a path escape, a symlink, or any other unsafe session file.

#### Scenario: Reject an identity mismatch
- **WHEN** the receipt or session identifies a package or session version other than the pinned expected versions
- **THEN** verification fails and the attempt cannot publish canonical prediction or score

#### Scenario: Reject malformed or mismatched content
- **WHEN** JSONL parsing, Pi structural validation, or receipt size/hash/entry-count comparison fails
- **THEN** verification fails and the attempt is terminal `failed` without canonical result publication

#### Scenario: Reject unsafe paths
- **WHEN** the receipt names a path outside the private controlled location, a symlink, or an otherwise unsafe file
- **THEN** verification fails before the file is accepted as session evidence

### Requirement: Verify on-demand Pi export as disposable inspection
The verifier SHALL support on-demand inspection with the pinned command `pi --export INPUT OUTPUT` using the retained native session as input. Exported HTML SHALL be disposable, noncanonical evidence. Verification SHALL confirm that the native source remains unchanged and that export succeeds.

#### Scenario: Export a retained session
- **WHEN** an operator requests inspection of a valid retained session
- **THEN** the pinned Pi CLI export succeeds and produces disposable HTML without changing the source JSONL or its recorded hash

#### Scenario: Reject export failure
- **WHEN** the pinned Pi export command fails or changes the source session
- **THEN** verification fails and no canonical prediction or score is published

### Requirement: Require the visualization-forbidden precursor gate
Before the existing paired gate, one visualization-forbidden precursor run SHALL require an accepted submission, normal scoring with correctness not required, a complete matching receipt and hash for a valid native session, and successful pinned Pi export. This precursor gate MUST remain separate and MUST NOT replace the existing paired-mode gate.

#### Scenario: Accept a qualifying precursor run
- **WHEN** the visualization-forbidden precursor has an accepted submission, normal scoring, a complete verified session receipt and hash, and a successful export
- **THEN** the precursor gate passes while leaving the existing paired-mode gate to run separately

#### Scenario: Reject an incomplete precursor run
- **WHEN** any required submission, scoring, complete-session, receipt/hash, or export condition is absent or invalid
- **THEN** the precursor gate fails and no canonical prediction or score is published
