## ADDED Requirements

### Requirement: Opt-in H.264 image streams preserve the Image contract
DimOS SHALL allow an image stream to opt into H.264 encoding while preserving `Image` as the public stream payload type for publishers and subscribers.

#### Scenario: Publisher and subscriber use normal Image objects
- **GIVEN** a blueprint configures an image stream for H.264 live transmission
- **AND** the source module publishes `Image` values on an `Out[Image]` stream
- **WHEN** a downstream module subscribes through an `In[Image]` stream
- **THEN** the downstream callback receives decoded `Image` values
- **AND** the module author does not need to publish or subscribe to encoded video packet values.

#### Scenario: Existing image streams remain unchanged by default
- **GIVEN** a blueprint does not opt an image stream into H.264 transmission
- **WHEN** the blueprint runs with its existing image transport configuration
- **THEN** DimOS MUST preserve the existing image transport behavior
- **AND** H.264 dependencies or settings are not required for that stream.

### Requirement: H.264 packets are complete per-frame Annex B access units
DimOS SHALL represent each H.264-transmitted source image frame as one complete encoded-frame packet containing the Annex B NAL units emitted for that encoder input frame.

#### Scenario: One encoded packet corresponds to one source frame
- **GIVEN** an H.264-enabled image stream publishes one source `Image` frame
- **WHEN** DimOS encodes that frame for a non-WebRTC carrier or for packet inspection
- **THEN** the encoded packet data MUST contain all NAL units emitted for that source frame in Annex B form
- **AND** the encoded packet represents exactly one source frame.

#### Scenario: Delta-frame packets require GOP state
- **GIVEN** an encoded packet contains a delta frame
- **WHEN** a decoder processes that packet without the prior GOP state required by H.264
- **THEN** DimOS MUST treat the packet as requiring recovery from a keyframe
- **AND** DimOS MUST avoid presenting corrupted image pixels as a valid decoded `Image`.

### Requirement: Keyframes bootstrap late join and recovery
DimOS SHALL provide periodic keyframes for H.264 image streams so subscribers can start or recover decoding at bounded intervals.

#### Scenario: Late subscriber waits for a keyframe
- **GIVEN** an H.264 image stream is already publishing
- **WHEN** a subscriber joins after the stream has started
- **THEN** DimOS MUST begin delivering decoded images only after the subscriber has valid keyframe-based decoder state
- **AND** the subscriber must not receive corrupted decoded images from incomplete GOP state.

#### Scenario: Keyframes include decoder parameter data
- **GIVEN** an H.264 image stream emits an IDR keyframe
- **WHEN** the keyframe packet is used to bootstrap a new decoder
- **THEN** the keyframe packet MUST include the decoder parameter information needed for that bootstrap, such as SPS/PPS for H.264 Annex B streams
- **AND** later delta frames in the same GOP may depend on that decoded keyframe state.

### Requirement: Sequence gaps recover safely
DimOS SHALL detect missing or out-of-order H.264 live-stream packets and resume decoded image delivery from a valid keyframe state.

#### Scenario: Packet loss occurs mid-GOP
- **GIVEN** a subscriber is decoding an H.264 image stream
- **WHEN** DimOS detects a sequence gap before the next keyframe
- **THEN** DimOS MUST stop delivering decoded images from the invalid GOP state
- **AND** DimOS SHALL resume delivery after a subsequent keyframe establishes valid decoder state.

### Requirement: Unsupported image formats fail explicitly
DimOS SHALL accept only image formats supported by the configured H.264 image-stream mode and provide a clear failure for unsupported formats.

#### Scenario: Supported color image is transmitted
- **GIVEN** an H.264-enabled image stream receives a supported 8-bit color `Image` format
- **WHEN** DimOS encodes and transmits the image
- **THEN** subscribers MUST receive a decoded `Image` with the expected dimensions, timestamp, frame identifier, and color format semantics.

#### Scenario: Unsupported image format is rejected
- **GIVEN** an H.264-enabled image stream receives an unsupported image format such as depth, 16-bit, or alpha data
- **WHEN** DimOS attempts to encode or publish the image through the H.264 stream mode
- **THEN** DimOS MUST fail with a clear unsupported-format error
- **AND** DimOS MUST preserve safety by avoiding silent lossy conversion or corrupted output.

### Requirement: H.264 stream configuration is observable and bounded
DimOS SHALL expose user-configurable H.264 stream settings for bitrate, keyframe cadence, frame-rate assumptions, and low-latency profile behavior.

#### Scenario: Blueprint opts into H.264 settings
- **GIVEN** a blueprint configures an image stream for H.264 live transmission with bitrate and keyframe cadence settings
- **WHEN** the blueprint runs
- **THEN** DimOS MUST apply those settings to the H.264 stream behavior
- **AND** subscribers must continue to observe normal `Image` payloads rather than codec-specific internals.

#### Scenario: H.264 dependencies are unavailable
- **GIVEN** a user selects H.264 image-stream mode in an environment without the required video codec dependencies
- **WHEN** DimOS starts or initializes the H.264 stream
- **THEN** DimOS MUST fail with an actionable dependency error
- **AND** DimOS MUST preserve non-H.264 image-stream behavior for configurations that do not select H.264.
