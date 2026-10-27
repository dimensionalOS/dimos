# Perception

## Detections

Detectors accept `Image` messages from
`dimos.messages.sensor_msgs.msg`. Detection collections remain local Python
application objects; `to_ros_detection2d_array()` produces a generated
`Detection2DArray` for typed ports, recording and CDR transport.

This complete CPU example uses the project's sample image and model assets
(downloaded on first use). It performs detection without a robot or API key:

```python
from dimos.message_utils import image as image_utils
from dimos.message_utils import time as message_time
from dimos.perception.detection.detectors.yolo import Yolo2DDetector
from dimos.utils.data import get_data

image = image_utils.from_file(get_data("cafe.jpg"), header=message_time.header_now("camera_optical"))
detector = Yolo2DDetector(device="cpu")
try:
    detections = detector.process_image(image)
    message = detections.to_ros_detection2d_array()
    assert message.header == image.header
    assert len(message.detections) > 0
finally:
    detector.stop()
```

`Detection2DModule` and `Detection3DModule` expose typed image, calibration,
TF, point-cloud, and detection ports. LCM and Zenoh transports select CDR
encoding from the message type; see
[message packages](/docs/development/messages-external-project.md).

3D projection takes a `CameraInfo` calibration and a `TransformStamped` mapping
the cloud frame into the optical frame. Cloud filters preserve all selected
point records, including extra fields and source nanoseconds. For pixel arrays, point-cloud operations, and geometry, use the helpers in
`dimos.message_utils.image`, `dimos.message_utils.pointcloud` and `dimos.message_utils.geometry`.

### Tracking and embeddings

Person tracking and re-ID align images and detections by timestamp within a
two-second buffer. A published target retains the image source stamp even
when its TF sample is older.

With the ML dependencies installed, embed an image:

```python
from dimos.models.embedding.clip import CLIPModel
from dimos.message_utils import image as image_utils
from dimos.message_utils import time as message_time
from dimos.utils.data import get_data

image = image_utils.from_file(get_data("cafe.jpg"), header=message_time.header_now("camera_optical"))
model = CLIPModel(device="cpu")
try:
    embedding = model.embed(image)
    assert tuple(embedding.vector.shape) == (512,)
finally:
    model.stop()
```

CLIP, DINO, SigLIP, SigLIP2, and TorchReID accept `Image` messages.
Embedding timestamps are in seconds. Models download weights on first use
and reuse their cached weights.

## Fiducial markers

Fiducial detection uses generated images, calibration and stamped transforms.
This example renders and detects a marker entirely on CPU; it needs the existing
OpenCV contrib dependency, but no camera, model download, network or API key:

```python
import cv2
import numpy as np

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos.messages.std_msgs.msg import Header
from dimos.message_utils import camera_info
from dimos.message_utils import image as image_utils
from dimos.perception.fiducial.marker_detect import detect_markers_in_image

pixels = np.full((400, 400), 255, dtype=np.uint8)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
pixels[100:300, 100:300] = cv2.aruco.generateImageMarker(dictionary, 7, 200)
header = Header(frame_id="camera_optical", stamp=Time(sec=0, nanosec=123456789))
image = image_utils.from_array(pixels, encoding="mono8", header=header)
camera = camera_info.from_intrinsics(400, 400, 200, 200, 400, 400, header=header)
mount = TransformStamped(
    header=Header(frame_id="world", stamp=header.stamp),
    child_frame_id="camera_optical",
    transform=Transform(translation=Vector3(x=2, y=0, z=0), rotation=Quaternion(x=0, y=0, z=0, w=1)),
)
markers = detect_markers_in_image(
    image, camera_info=camera, world_T_optical=mount,
    marker_length_m=0.2, aruco_dictionary="DICT_4X4_50",
)
message = markers[0].to_detection3d_msg()
assert message.id == "7"
assert message.header == Header(frame_id="world", stamp=header.stamp)
assert abs(message.bbox.center.position.z - 0.4) < 0.01
```

`MarkerDetectionStreamModule` publishes marker detections. `MarkerTfModule`
publishes marker transforms. Both outputs retain the image source timestamp.

## Query-time localization geometry

Localization discovers generated calibration, color, depth and lidar streams by
message type and source frame. Depth classification uses the decoded pixel type.

This standalone geometry example composes two mounts without loading models:

```python
from dimos.perception.localize.rig import Rig
from dimos.protocol.tf.static_tf_publisher import frames_to_edge_transforms
from dimos.protocol.tf.tf import MultiTBuffer

tf = MultiTBuffer()
tf.receive_transform(*frames_to_edge_transforms([
    ("base_link", "world", (2, 0, 0), (0, 0, 0)),
    ("front", "base_link", (0.5, 0, 0), (0, 0, 0)),
]))
rig = Rig(cameras={}, tf=tf)
pose = rig.camera_pose(0, "front")
assert pose is not None
assert pose.header.frame_id == "world"
assert pose.pose.position.x == 2.5
```

The color cache stores JPEG-compressed images with their source headers;
geometry uses lossless compression. `Localization.last_seen_timestamp` is in
seconds, and `as_detection_array()` returns a `Detection3DArray` message.

## Experimental object memory

Object memory combines detections across scene scans. Objects retain the color
capture timestamp on their pose and detection, and the depth timestamp on their
projected cloud. Pending objects expire according to wall-clock time.

Scene registration, OpenCV tracking, EdgeTAM, and YOLO-E consume image and
calibration messages. Temporal memory and PerceiveLoop also accept images;
PerceiveLoop selects sharp frames from a rolling window. Temporal memory
publishes `dimos_msgs/EntityMarkers` for visualization in Rerun.

## Experimental WorldBelief

The experimental xArm6 WorldBelief stack records RGB-D observations and processes
them on demand to maintain object identities across scans and process restarts.
Its implementation lives in
`dimos/experimental/world_belief/` while the shared detector,
embedding, and 3D object primitives remain in their standard packages.

Run the hardware blueprint:

```bash
dimos run xarm6-worldbelief --controlcoordinator.addresses '{"arm": "<ROBOT_IP>"}'
```

Then request a scan or recall from another terminal:

```bash
dimos mcp call scan -a prompt='["mug", "coke can"]'
dimos mcp call recall -a text="mug"
```

This stack is experimental and may change without compatibility guarantees.

The arm's address is configured per hardware id, here `arm`; see [Hardware configuration](/docs/usage/configuration.md#hardware-configuration).
