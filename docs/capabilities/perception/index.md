# Perception

## Detections

On the CDR proposal branch, detector inputs are generated `Image` values from
`dimos.messages.sensor_msgs.msg`. Detection collections remain local Python
application objects; `to_ros_detection2d_array()` produces a generated
`Detection2DArray` for typed ports, recording and CDR transport.

This complete CPU example uses the project's sample image and model assets
(downloaded on first use). It performs detection without a robot or API key:

```python
from dimos.msgs.image import image_from_file
from dimos.msgs.time import header_now
from dimos.perception.detection.detectors.yolo import Yolo2DDetector
from dimos.utils.data import get_data

image = image_from_file(get_data("cafe.jpg"), header=header_now("camera_optical"))
detector = Yolo2DDetector(device="cpu")
try:
    detections = detector.process_image(image)
    message = detections.to_ros_detection2d_array()
    assert message.header == image.header
    assert len(message.detections) > 0
finally:
    detector.stop()
```

`Detection2DModule` and `Detection3DModule` keep their existing processing and
port APIs. Their image, calibration, TF, point-cloud and detection ports now
use generated values. Until the proposal's default transport cutover is
complete, select the explicit CDR transport when wiring these ports; see
[message packages](/docs/development/messages-external-project.md).

3D projection takes a generated calibration and a `TransformStamped` mapping
the cloud frame into the optical frame. Cloud filters preserve all selected
point records, including extra fields and source nanoseconds. Generated
messages do not acquire `.ts` or NumPy/Open3D methods: use the helpers in
`dimos.msgs.image`, `dimos.msgs.pointcloud` and `dimos.msgs.geometry`.

The real Go2 regression fixture is a small canonical CDR selection from
`unitree_go2_lidar_corrected`. Its manifest records the original archive OID,
source entry hashes, recorder times, exact source stamps and CDR hashes.
Recorder-time pairing does not make sensor source clocks identical: the
fixture retains their measured offset, and TF lookups must select the right
time domain. Historical LCM/pickle recordings require explicit conversion.

## Experimental WorldBelief

The experimental xArm6 WorldBelief stack records RGB-D observations and processes
them on demand to maintain object identities across scans and process restarts.
Its implementation lives in
`dimos/experimental/world_belief/` while the shared detector,
embedding, and 3D object primitives remain in their standard packages.

Run the hardware blueprint:

```bash
dimos --xarm6-ip <ROBOT_IP> run xarm6-worldbelief
```

Then request a scan or recall from another terminal:

```bash
dimos mcp call scan -a prompt='["mug", "coke can"]'
dimos mcp call recall -a text="mug"
```

This stack is experimental and may change without compatibility guarantees.
