# LCM transport and generated messages

LCM carries bytes between processes. dimOS typed streams encode generated
messages as standalone XCDR1 payloads, including when the selected transport is
LCM. Zenoh and WebRTC typed streams use the same generated message model.

The LCM network protocol and LCM message encoding are separate choices. Selecting
`--transport=lcm` selects the carrier. It does not select the old LCM message
format or the deprecated Python message overlays.

## Generated message values

Import message classes from `dimos.messages`. Use the registry to encode and
decode them with an agreed schema:

```python session=lcm_demo ansi=false
from dimos.messages.geometry_msgs.msg import Vector3
from dimos_cdrgen.registry import encode, decode

message = Vector3(x=1.0, y=2.0, z=3.0)
payload = encode(message)
restored = decode(payload, Vector3)
assert type(restored) is Vector3
assert (restored.x, restored.y, restored.z) == (1.0, 2.0, 3.0)
```

Generated classes hold schema fields. Numerical operations and image or point
cloud conversions belong to `dimos.message_utils`, rather than message methods.
For example, create a generated point cloud from coordinates and an explicit
source Header:

```python session=lcm_demo ansi=false
import numpy as np
from dimos.message_utils import pointcloud
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.std_msgs.msg import Header

points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
cloud = pointcloud.from_xyz(
	points,
	header=Header(stamp=Time(sec=12, nanosec=345), frame_id="camera"),
)
np.testing.assert_allclose(pointcloud.xyz(cloud), points)
assert cloud.header.stamp.nanosec == 345
```

Use NumPy arrays for calculations. Use `pointcloud.to_open3d(cloud)` when an
Open3D algorithm needs an Open3D value. The generated `PointCloud2` remains the
public stream and recording type.

## Typed channels

Declare the same generated type at both ends of each channel:

```python skip
from dimos.core.transport import LCMTransport
from dimos.messages.geometry_msgs.msg import Vector3

velocity = LCMTransport("/velocity", Vector3)
velocity.publish(Vector3(1.0, 2.0, 3.0))
velocity.stop()
```

The channel name is `/velocity`. The payload does not contain a type name or the
old LCM fingerprint. Different message types need different channels. A remote
publisher must use the schema expected by its subscriber.

Wildcard readers pass a `message_types` map from canonical channel names to
registered generated classes. Known channels decode with those schemas. Unknown
channels remain bytes. See the [transport contract](/docs/usage/transports/index.md#typed-payloads-and-wildcard-subscriptions).

## Third-party messages

A library user can define `.msg` files in an external project, generate a message
package with cdrgen, and register its provider through the `dimos.messages`
entry-point group. External messages use the same typed transports and CDR
recording boundary as built-in messages. Shared dependencies reuse the registry's
canonical classes.

Follow the [external message project guide](/docs/development/messages-external-project.md).
No dimOS message subclass or handwritten codec is required.

## Historical recordings

Historical recording readers accept the old LCM envelopes and JPEG-in-LCM image
records and return generated values. These readers are compatibility boundaries
for existing artifacts. New native recordings use `cdr`, `lz4+cdr`, `cdrjpeg`, or
the explicit JSON codec.

`dimos-lcm` remains a dependency for LCM network transport and supported foreign
LCM boundaries. Its generated message classes are not the public dimOS message
API. Old language examples that encode LCM messages need a CDR codec and matching
schema before they can consume current typed streams.
