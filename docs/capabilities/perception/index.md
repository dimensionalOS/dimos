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

### Tracking and embeddings

Person tracking and re-ID keep their existing module methods and ports. Their
zero-tolerance image/detection join compares exact source nanoseconds inside
the existing two-second window. A published target retains the image source
stamp even when its TF sample is older.

With the existing ML dependencies, the same generated image can be embedded:

```python
from dimos.models.embedding.clip import CLIPModel
from dimos.msgs.image import image_from_file
from dimos.msgs.time import header_now
from dimos.utils.data import get_data

image = image_from_file(get_data("cafe.jpg"), header=header_now("camera_optical"))
model = CLIPModel(device="cpu")
try:
    embedding = model.embed(image)
    assert tuple(embedding.vector.shape) == (512,)
finally:
    model.stop()
```

CLIP, DINO, SigLIP, SigLIP2 and TorchReID use generated images and shared pixel
helpers. Their embedding API keeps seconds-based timestamps, including a
valid zero stamp. Model weights follow each existing backend's cache policy;
message import and message installation do not trigger model downloads.

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
from dimos.msgs.camera_info import camera_info_from_intrinsics
from dimos.msgs.image import image_from_array
from dimos.perception.fiducial.marker_detect import detect_markers_in_image

pixels = np.full((400, 400), 255, dtype=np.uint8)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
pixels[100:300, 100:300] = cv2.aruco.generateImageMarker(dictionary, 7, 200)
header = Header(frame_id="camera_optical", stamp=Time(sec=0, nanosec=123456789))
image = image_from_array(pixels, encoding="mono8", header=header)
camera = camera_info_from_intrinsics(400, 400, 200, 200, 400, 400, header=header)
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

`MarkerDetectionStreamModule` and `MarkerTfModule` keep their existing stream
names. Marker arrays and resulting TF messages preserve the image source
nanoseconds, including zero. Frame aggregation also distinguishes adjacent
nanoseconds when the local Observation API represents their times as the same
floating-point second. Select explicit CDR transports until the default runtime
cutover is complete; the desk blueprint retains its LCM backend with CDR encoding.

## Query-time localization geometry

Localization discovers generated calibration, color, depth and lidar streams by
message identity and source frame. Depth classification inspects decoded pixels,
including both byte orders; wire image data is a flat byte array. Existing model
selection and localization thresholds remain unchanged.

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

The local color cache keeps JPEG compression in a generated CompressedImage
CDR envelope (`jpegcdr` storage codec); geometry uses lossless `lz4+cdr`.
JPEG decoding normalizes color to BGR8 while retaining the exact source Header.
These are local storage codecs, not new transport or MCAP encodings.

Merged lidar maps and grouped object clouds retain the newest contributing
source stamp in integer nanoseconds. Query results still expose seconds through
the existing `Localization.last_seen_timestamp` API; `as_detection_array()`
converts that value to a generated Detection3DArray. Live module/blueprint
execution requires the remaining default transport cutover. Historical recordings
still require explicit conversion; this slice does not add a legacy decoder.

## Experimental object memory

The experimental object, scene-registration and tracker modules now use generated
image, calibration, detection and TF values on their typed ports. EdgeTAM and
YOLO-E segmentation retain their existing model APIs and use the same image
helpers. OpenCV tracking keeps its existing geometry and processing-time policy.

Object geometry keeps the color source stamp on its pose/detection and the depth
source stamp on its projected cloud. Scene scans preserve their capture Header;
spatial deduplication compares source frame and integer nanoseconds rather than
rounded application seconds. Wall-clock pending-object TTL remains separate, and
source timestamp zero is not replaced by wall time. Until default encoding
cutover, these generated ports require the explicit CDR transports described
above. Offline boundary tests do not constitute full model or robot acceptance.

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
