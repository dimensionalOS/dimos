# Hosted arm, robot and simulation consumer continuation

The hosted arm uses generated PoseStamped, TwistStamped and Float32 with the
base WebXR explicit channel/type/CDR framing. Its existing 24 tests pass,
including E-stop, stale/future/out-of-order rejection, per-hand watermarks,
disconnect and scaling. Blueprint transport type keys match generated ports.
No physical device was connected or controlled.

Drone odometry and camera callbacks use generated headers and nested poses,
explicit image views and stamped TF edges. Mock numerical processing, PID and
camera tracking checks pass; this does not validate historical drone pickle
recordings, a camera pipeline or a live MAVLink connection.

The old Unitree Odometry subclass is removed. Device dictionaries convert to
standard generated PoseStamped. The mock robot's CDR port preserves the source
header; a separate TimestampedData Python-object port carries publication time
for the existing latency benchmark. The original 179-entry raw odometry fixture
and rotation checks pass. Its 15,685-byte LFS archive was fetched; large legacy
recordings were not fetched for this validation.

Go2 standard DDS payloads decode through generated codecs. Six independent
checks cover little/big-endian Odometry against rosbags, exact timestamp and
covariance fields, arbitrary padded PointCloud2 layouts, the device's IMU
quaternion ordering, compressed JPEG payloads and mocked H264 frame emission.
The device-specific HeightMap layout remains separate. The optional ROS fallback
uses generated standard values; an installed ROS runtime is not required.

Blueprints, manipulator configurations and simulation interfaces use generated
message type keys. Rerun optical-frame and path-height overrides are explicit
helper parameters without modifying source messages. Existing mapping and
manipulation safety assertions remain enabled.

MuJoCo publishes generated PoseStamped, Imu and TFMessage. All 12 existing
module tests pass with temporary synthetic MJCFs and mocks, including camera
relative transforms, source frame timestamps and SHM ready ordering. Added
assertions decode the actual published pose and IMU CDR bytes. These are bounded
offline tests, not a robot or viewer demonstration. MuJoCo 3.10.0 from the lockfile
was installed only in the project virtualenv (19.9 MiB wheel).

The combined hosted arm, DDS, Rerun, manipulation, MuJoCo, odometry and timestamp
regression passed 119 tests on Python 3.12.14. Strict scoped mypy passed all 40 changed production files. Required repository
hooks passed.
Large historical perception/DDS/replay fixture tests, default-interface LCM
multicast, remaining consumer and dependency retirement, the clean build matrix
and human viewer/cumulative demo gates remain open.

## Previous exact-commit CI

At 36a0fd2c35848d26b1daf19b4b744e6d49afd9fd, message-codegen run 36744961105
succeeded. Main run 36744956207 terminated cancelled: native C++/Rust, Rust,
lint, web and complete md-babel succeeded. ARM failed seven hosted-arm tests
that still dispatched through legacy fingerprints; this batch fixes that path.
Other Python lanes were cancelled by fail-fast, so they do not supply a terminal
RPC result. The unchanged 100 ms RPC assertion has no pre-warming or relaxed
limit. A subsequent published HEAD requires its own terminal CI results.
