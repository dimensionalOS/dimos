## ADDED Requirements

### Requirement: Retain a private native session for every attempt
Each evaluated attempt SHALL create a fresh file-backed native Pi v3 session in a private, controlled location, using the pinned Pi package version `0.80.10`. The evaluated path MUST NOT use `SessionManager.inMemory`.

#### Scenario: Start an evaluated attempt
- **WHEN** an attempt starts
- **THEN** it receives a new private native Pi v3 JSONL session associated only with that attempt and produced by Pi `0.80.10`

#### Scenario: Prevent in-memory-only evidence
- **WHEN** the evaluated runner creates or retains its session
- **THEN** the session is file-backed in the controlled private location and is not created through `SessionManager.inMemory`

### Requirement: Preserve and receipt the native JSONL
The retained native session SHALL be preserved verbatim and SHALL remain the canonical conversation evidence. A private receipt SHALL record its relative basename, producer and package version, session version, terminal state (`complete`, `partial`, or `unavailable`), byte size, content hash, and JSONL entry count. The retained content SHALL be sufficiently validated for Pi session inspection, including its header, version, tree, and message integrity.

#### Scenario: Receipt a valid complete session
- **WHEN** an attempt settles successfully with a valid native session
- **THEN** the original JSONL bytes remain unchanged and the receipt records the required identity, integrity, count, and `complete` state fields

#### Scenario: Retain a valid failed prefix
- **WHEN** an attempt fails or is interrupted after producing a valid JSONL prefix
- **THEN** that prefix is retained byte-for-byte and its receipt records `partial` with matching size, hash, and entry count

#### Scenario: No valid native session exists
- **WHEN** an attempt produces no valid inspectable session
- **THEN** no unsafe or fabricated transcript is retained, and a private receipt records `unavailable` with only safe metadata

### Requirement: Bind private context evidence without changing the session
The exact private system prompt and exact initial prompt SHALL be retained as sidecars, each bound to the native session by a digest. Binding or retaining these sidecars MUST NOT modify the native session JSONL.

#### Scenario: Verify prompt sidecars
- **WHEN** the retained evidence is inspected
- **THEN** the system and initial prompt sidecars match their recorded digests and the native session bytes still match their original hash

#### Scenario: Keep context private
- **WHEN** an attempt is exposed through public result or operational outputs
- **THEN** neither prompt sidecar nor its private contents are leaked

### Requirement: Retain benchmark sidecars separately
Benchmark execution records, tool-policy audit records, prediction records, and score records SHALL remain separate private sidecars from the canonical native session.

#### Scenario: Reconstruct benchmark evidence
- **WHEN** a retained attempt is reviewed
- **THEN** its execution and tool-audit evidence and its prediction and score records are available as sidecars without requiring a parallel conversation transcript

#### Scenario: Keep all retained evidence private
- **WHEN** an attempt is incomplete or failed
- **THEN** its native session, receipt, prompts, retained execution sidecars, and failure evidence remain in the private attempt evidence location, no canonical prediction or score is published, and no public result is emitted from them
