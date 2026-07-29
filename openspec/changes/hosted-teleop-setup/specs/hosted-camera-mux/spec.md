# hosted-camera-mux

## ADDED Requirements

### Requirement: Operator-selectable camera mux
The robot SHALL composite its named cameras (e.g. cam1/cam2) into the single video track according to the operator's selection: one camera passes through; multiple cameras are hstacked side-by-side, tiles scaled to the smallest height. Selection changes via `{"type":"camera_select","cams":[...]}` MUST take effect immediately (republish from cached frames, not waiting for the next frame). Unknown camera names fall back to the first camera.

#### Scenario: Switch to wrist cam
- **WHEN** the operator sends `camera_select` with `["cam2"]`
- **THEN** the very next published frame shows cam2 alone and telemetry `state.cams` reports `["cam2"]`

#### Scenario: Both cameras
- **WHEN** the operator selects `["cam1","cam2"]`
- **THEN** one frame containing both views side-by-side is published on the single track

### Requirement: Publish-side video caps
The mux SHALL enforce configurable fps (`video_max_fps`) and width (`video_max_width`) caps before encoding, so uplink congestion is prevented at the source rather than surfacing as encoder drops/freezes. Zero disables a cap.

#### Scenario: Constrained cellular uplink
- **WHEN** `video_max_width=960` and `video_max_fps=15` are set
- **THEN** published frames never exceed 960px width or 15 fps regardless of camera source rate

### Requirement: Glass-to-glass latency stamping
When `latency_stamp` is enabled, the mux SHALL append a strip of black/white cells below the frame encoding capture time in milliseconds (sync pattern + 44 time bits, 16px cells), without overwriting video content. The operator decodes the strip for glass-to-glass latency and crops it before display. The strip is skipped when the frame is narrower than the cell budget (768px). Cell constants MUST stay in sync with the operator-side decoder.

#### Scenario: Latency benchmark run
- **WHEN** `latency_stamp=true` and frames are ≥768px wide
- **THEN** each published frame carries a decodable capture-time strip and the operator HUD reports glass-to-glass latency
