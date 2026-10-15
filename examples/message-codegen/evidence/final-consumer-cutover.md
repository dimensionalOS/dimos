# Remaining consumer cutover

The previous published batch `a36cc9565e92b4acda71d5e883b3480362b6b9e7`
passed message-codegen run 36755062352. Main run 36755058007 failed three
Python 3.10 assertions: two simulated settling fixtures still constructed the
old PoseStamped, and image resizing had a module-level OpenCV import. Other
Python lanes were cancelled by fail-fast. Web, documentation, lint, Rust and
native builds passed. This is not a full green CI result.

The follow-up migrates eval dataset/prompt/coverage fixtures and the actual
coverage scorer to generated poses. Pose text remains numeric and includes
frame, position and quaternion through an external agent formatter; the
original selection, downsampling, timing, scoring and stopping assertions
remain. OpenCV resizing now imports its native extension on demand.

Remaining mock-only consumers use generated messages: Alfred wheel odometry
preserves coordinate inversion, source time and local velocity; existing
Cartesian/trajectory controllers use generated JointCommand/RobotState and
retain PID limits, joint count and interpolation. No driver/controller start
or physical robot operation was performed. Passive lidar conversion and
recorder scripts were type checked, without executing device/network setup.

TemporalMemory publishes the existing generated dimos_msgs/EntityMarkers
schema, with nested geometry_msgs/Point positions. Its obsolete private JSON
message wrapper is removed. The external Rerun helper preserves colors,
labels, coordinates and marker radius. An independent rosbags decoder verifies
Unicode metadata, dependency closure, positions and exact sec/nanosec source
time. No rendering method is attached to a generated message.

The alternate pgo_auto implementation uses generated TransformStamped/clouds
and external geometry/cloud algorithms. Solver parameters and the existing
nearest-neighbor correction lookup remain unchanged. Tests exercise time zero,
clamped lookup endpoints, frame composition, unchanged payloads, empty input,
and a synthetic cloud through real GTSAM; no recording download is required.

Scoped strict mypy passed all 15 changed production files on Python 3.12.14.
The generation/build succeeded with the existing authoritative nested Point
schema. The full targeted offline suite passed 147 tests, excluding one
explicitly networked TemporalMemory integration test. No acceptance gate was
weakened or marked complete based only on these tests.

Legacy convenience modules, their tests and fixtures, manual tools and the
external dimos-lcm package are still present and require final retirement.
Default-interface multicast and human viewer/physical demonstrations remain
separate unresolved gates; these results do not establish whole-plan completion.
