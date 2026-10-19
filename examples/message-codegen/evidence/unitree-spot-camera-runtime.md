# Unitree, Spot and camera metadata boundaries

B1 command/joystick/transport types are generated TwistStamped and Int32.
The handler reads nested twist fields; mode changes, thresholds, saturation and
200 ms watchdog remain unchanged. Existing MockB1 watchdog tests passed (10
tests together with the prior odometry case). Three additional CDR cases passed
for WALK/STAND mapping and captured zero publication; neither a robot socket
nor a pygame control window was opened.

G1 high-level RPC/skill contracts and keyboard operator gates/slider values use
generated Twist, Int8 and Float32. G1's lossless depth storage codec is lz4+cdr.
Two tests captured real constructor/callback outputs with mocked pygame and RPC
collaborators; operator-only mode emitted no movement. The G1 recorder metadata
case awaits CI because local Recorder imports torch.

Lazy named calibration YAML loading is an external helper returning ordinary
generated CameraInfo. ZED calibration access is tested with its SDK absent.
Seven calibration tests passed. The camera module and G1 camera/navigation
blueprint use generated stamped transforms/calibration/rendering helpers; 19
offline/mocked camera tests passed. Explicit identity rotations and matrix
assertions preserve existing behavior: generated Quaternion already defaults
to w=1. The earlier evidence's all-zero-default claim has been corrected.

Spot image/calibration publishers, recorder and replay ports use generated
messages, retaining robot-to-local clock conversion and camera rotation rules.
Lossless raw/depth storage uses lz4+cdr, with no legacy payload decoder. Seventeen
offline tests passed, including five raw pixel layouts, JPEG decoding, five
quarter-turn cases with exact headers/nonmutation, calibration and the existing
odometry/TF cases. SDK image constants/responses were mocked; these tests do not
establish native SDK interoperability or hardware behavior. No Spot SDK startup
or device commands ran. Scoped strict mypy passed for the changed production
files. Full repository/optional-device integration remains a CI/manual gate.

Paused WebXR/control/imitation/memory-index files remain excluded, and neither
denied script was retried. Legacy convenience consumers/dependency retirement,
model-dependent memory documentation and final human viewer evidence remain
open; these local results do not mark the OpenSpec change complete.

CI run 36729267784 at 2c0ce030f passed Python 3.10, 3.11 and ARM
3.12, lint, Web, Rust and native builds; codegen run 36729275058 also
passed. The x86 Python 3.12 job exhausted its 20-minute whole-job budget
after approximately nine minutes of cold setup and reached 99% of tests
without a final result. The Python job budget is raised to 30 minutes;
no tests, fail-fast behavior or assertions are disabled. md-babel still fails
on deliberately unsupported legacy JPEG storage and is not claimed passing.
