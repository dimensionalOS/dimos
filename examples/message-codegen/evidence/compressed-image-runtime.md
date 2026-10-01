# Standard compressed-image transport and raw adapters

JPEG LCM and JPEG SHM now serialize generated `sensor_msgs/CompressedImage`
CDR values, retaining the original image header and standard ROS compressed
format. The application-facing transport remains generated `Image`; its wire
topic declares `CompressedImage`, including after pickling. WebRTC video uses
explicit generated ROS encodings and array views without changing frame timing.
Raw robot bridge JSON/JPEG/XYZ contracts and deadman behavior remain unchanged;
M20 ports/scalar commands now use generated messages without changing guards.
Tests use synthetic values, capture-only M20 construction, and local loopback.

**23 focused tests passed** in the final combined image/codec/WebRTC/raw-bridge/
M20 run. The earlier codec/video/registry run passed 33 tests, and separate raw
bridge/M20 runs passed 7 and 1 respectively (overlapping coverage, not summed).
Strict scoped mypy passed for all seven changed production files; repository
hooks passed after normal license-header insertion. The image fixture expanded
the existing 133 KiB cafe archive; this check does not prove legacy Image API
retirement. The exact Python 3.11 CI failure in the collection E2E fixture was
repaired by importing generated Image and viewing its pixel buffer with
`image_view`; local E2E collection remains blocked by missing h5py.

A fresh synthetic MCAP contains 150 messages across five channels (30 each),
3,123,201 bytes. Native Rerun 0.32.0 imported it with Python provider imports
absent and raw fallback disabled. Its RRD contains typed image/encoded-image,
pose/TF, and nested telemetry fields; `check_rerun_mcap.py` passed. Evidence is
under ignored `build/message-codegen/viewers-current/evidence`. This is offline
conversion evidence, not GUI/Foxglove acceptance. The viewer script prioritizes
the demo binding over the root binding and now disables Rerun raw fallback.

Exact prior published HEAD 2334dd05d3e62b1fbc05f3e773b1ff1696f345c1 passed
message-codegen, strict lint, Native C++/Rust, Rust, Web, and ARM CI. Python 3.11
had 6292 passed, 82 skipped, and two fixture errors fixed above; 3.10/3.12 were
cancelled by fail-fast. md-babel still fails on old memory codecs. The eight
paused files remain excluded. WebXR control edits and model-dependent memory
document execution have not been retried. Full migration/retirement remains open.
