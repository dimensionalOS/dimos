## ADDED Requirements

### Requirement: H.264 image storage is opt-in per memory2 stream
memory2 SHALL allow image streams to opt into H.264-backed storage while preserving the default image-storage behavior for streams that do not opt in.

#### Scenario: Stream opts into H.264 storage
- **GIVEN** a memory2 image stream is configured for H.264-backed storage
- **WHEN** the stream appends `Image` values
- **THEN** memory2 MUST store those image observations using H.264-backed payloads
- **AND** queries for the stream must continue to return image observations associated with the original frame timestamps.

#### Scenario: Stream uses default image storage
- **GIVEN** a memory2 image stream is created without H.264 image-storage configuration
- **WHEN** the stream appends `Image` values
- **THEN** memory2 MUST preserve the existing default image-storage behavior
- **AND** existing JPEG-backed recordings remain readable through the normal memory2 APIs.

### Requirement: H.264 storage preserves one observation per source frame
memory2 SHALL store H.264-backed image streams with one observation corresponding to each source image frame.

#### Scenario: Recording a sequence of image frames
- **GIVEN** a recorder receives a sequence of `Image` frames on an H.264-backed memory2 stream
- **WHEN** memory2 stores the sequence
- **THEN** memory2 MUST create one queryable observation per source frame
- **AND** each observation must retain its timestamp, frame identifier, pose metadata when available, and tags independently of pixel decode.

### Requirement: Stored H.264 packets are complete frame access units
memory2 SHALL store each H.264-backed image observation with an encoded payload that contains the complete Annex B access unit for that source frame.

#### Scenario: Stored packet is inspected or exported
- **GIVEN** an H.264-backed image observation has an encoded payload
- **WHEN** the payload is inspected by storage tooling or exported to a compatible video-message format
- **THEN** the payload MUST represent all NAL units emitted for that source frame in Annex B form
- **AND** memory2 MUST avoid exposing individual RTP fragments as the stored observation payload.

### Requirement: GOP metadata supports random access and replay
memory2 SHALL persist enough GOP and keyframe metadata for H.264-backed image streams to decode requested observations and replay streams deterministically.

#### Scenario: Query decodes a mid-GOP observation
- **GIVEN** a user queries an H.264-backed image observation whose encoded payload is a delta frame
- **WHEN** the user accesses the observation pixel data
- **THEN** memory2 MUST decode from the nearest prior usable keyframe through the requested observation
- **AND** the returned value must be a decoded `Image` for the requested observation.

#### Scenario: Required GOP data is missing
- **GIVEN** an H.264-backed image observation requires prior GOP data to decode
- **WHEN** memory2 cannot load a usable keyframe or required delta-frame sequence
- **THEN** memory2 MUST fail the pixel decode with a clear storage/decode error
- **AND** memory2 MUST avoid returning corrupted pixels as a valid `Image`.

### Requirement: Metadata queries do not force pixel decode
memory2 SHALL allow metadata access for H.264-backed image observations without decoding image pixels.

#### Scenario: Query reads observation metadata only
- **GIVEN** a memory2 store contains H.264-backed image observations
- **WHEN** a user queries observations and reads timestamps, frame identifiers, pose metadata, tags, width, height, or image format metadata
- **THEN** memory2 MUST provide that metadata without requiring H.264 pixel decode
- **AND** pixel decode should occur only when the user accesses image data.

### Requirement: Lazy pixel access reconstructs Image values
memory2 SHALL lazily reconstruct `Image` values for H.264-backed observations when pixel data is requested.

#### Scenario: User accesses observation data
- **GIVEN** a queried H.264-backed image observation has not decoded its pixels yet
- **WHEN** the user accesses `obs.data`
- **THEN** memory2 MUST return a decoded `Image` value for that observation
- **AND** subsequent compatible accesses may reuse decoded state without changing observable image contents.

### Requirement: H.264-backed replay emits normal Image frames
memory2 SHALL replay H.264-backed image streams as normal decoded `Image` frames on the existing replay schedule.

#### Scenario: Replaying a stored H.264 image stream
- **GIVEN** a memory2 store contains an H.264-backed image stream
- **WHEN** replay is started for that stream
- **THEN** replay MUST emit decoded `Image` values in observation timestamp order
- **AND** consumers of replayed streams must not need to consume encoded video packet values.

### Requirement: H.264 storage survives store reopen
memory2 SHALL persist H.264 storage configuration and frame-index metadata so a reopened store can query, decode, and replay H.264-backed image streams.

#### Scenario: Reopen and decode
- **GIVEN** a memory2 store was written with an H.264-backed image stream
- **WHEN** the process closes and a later process reopens the store
- **THEN** memory2 MUST recognize the stream as H.264-backed
- **AND** the reopened store must support metadata query, lazy pixel decode, and replay for the stored observations.
