# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from collections.abc import Callable, Generator
from contextlib import ExitStack
from copy import deepcopy
import functools
import hashlib
from importlib import resources
import json
import math
from pathlib import Path
from typing import TypedDict

from dimos_cdrgen.registry import decode as cdr_decode
import pytest

from dimos.core.transport import LCMTransport
from dimos.message_utils.camera_info import camera_info_from_yaml
from dimos.message_utils.geometry import transform_from_pose
from dimos.message_utils.time import to_nanoseconds
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import PoseStamped, TransformStamped
from dimos.messages.sensor_msgs.msg import CameraInfo, Image, PointCloud2
from dimos.messages.std_msgs.msg import Header
from dimos.messages.vision_msgs.msg import Detection2DArray
from dimos.perception.detection.detectors.yolo import Yolo2DDetector
from dimos.perception.detection.module2D import Detection2DModule
from dimos.perception.detection.module3D import Detection3DModule
from dimos.perception.detection.type.detection2d.base import Detection2D
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection3d.imageDetections3DPC import ImageDetections3DPC
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.perception.experimental.moduleDB import ObjectDBModule
from dimos.protocol.tf.static_tf_publisher import frames_to_edge_transforms
from dimos.protocol.tf.tf import TF
from dimos.utils.data import get_data


def _test_data(name):
    local = Path(__file__).resolve().parents[3] / "data" / name
    return local if local.exists() else get_data(name)


@pytest.fixture(scope="session")
def detection_assets():
    return _test_data


def front_camera_calibration() -> CameraInfo:
    source = resources.files("dimos.robot.unitree.go2").joinpath("front_camera_720.yaml")
    with resources.as_file(source) as path:
        return camera_info_from_yaml(
            path, header=Header(frame_id="camera_optical", stamp=Time(sec=0, nanosec=0))
        )


def _dispose_module(module):
    try:
        module.detector.stop()
    finally:
        module._close_module()


class Moment(TypedDict, total=False):
    odom_frame: PoseStamped
    lidar_frame: PointCloud2
    image_frame: Image
    camera_info: CameraInfo
    transforms: list[TransformStamped]
    tf: TF
    detections: ImageDetections3DPC | None


class Moment2D(Moment):
    detections2d: ImageDetections2D


class Moment3D(Moment):
    detections3dpc: ImageDetections3DPC


@pytest.fixture(scope="session")
def tf():
    t = TF()
    yield t
    t.dispose()


@pytest.fixture(scope="session")
def get_moment(tf):
    @functools.lru_cache(maxsize=1)
    def moment_provider(**kwargs) -> Moment:
        seek = kwargs.get("seek", 10.0)
        data_dir = _test_data("unitree_go2_detection_cdr")
        manifest = json.loads((data_dir / "manifest.json").read_text())
        entry = next(item for item in manifest["moments"] if item["seek"] == seek)

        def payload(name):
            stream = entry["streams"][name]
            data = (data_dir / stream["file"]).read_bytes()
            assert hashlib.sha256(data).hexdigest() == stream["cdr_sha256"]
            value = cdr_decode(
                data, {"lidar": PointCloud2, "video": Image, "odom": PoseStamped}[name]
            )
            assert (
                to_nanoseconds(value.header.stamp)
                == stream["source_sec"] * 1_000_000_000 + stream["source_nanosec"]
            )
            return data

        lidar_frame = cdr_decode(payload("lidar"), PointCloud2)
        image_frame = cdr_decode(payload("video"), Image)
        image_frame.header.frame_id = "camera_optical"
        odom_frame = cdr_decode(payload("odom"), PoseStamped)
        # Match the Go2 connection's configured global parent for this recording.
        odom_frame.header.frame_id = "world"

        transforms = [transform_from_pose(odom_frame, child_frame_id="base_link")]
        # Recorded Go2 mounting geometry; no robot connection is constructed.
        transforms.extend(
            frames_to_edge_transforms(
                [
                    ("camera_link", "base_link", (0.3, 0, 0), (0, 0, 0)),
                    (
                        "camera_optical",
                        "camera_link",
                        (0, 0, 0),
                        (-math.pi / 2, 0, -math.pi / 2),
                    ),
                ]
            )
        )
        for transform in transforms:
            transform.header.stamp = deepcopy(odom_frame.header.stamp)

        tf.receive_transform(*transforms)

        return {
            "odom_frame": odom_frame,
            "lidar_frame": lidar_frame,
            "image_frame": image_frame,
            "camera_info": front_camera_calibration(),
            "transforms": transforms,
            "tf": tf,
        }

    yield moment_provider
    moment_provider.cache_clear()


@pytest.fixture(scope="session")
def publish_moment():
    def publisher(moment: Moment | Moment2D | Moment3D) -> None:
        with ExitStack() as cleanup:

            def publish(topic, msg_type, value):
                transport = LCMTransport(topic, msg_type)
                cleanup.callback(transport.stop)
                transport.publish(value)

            detections = moment.get("detections2d")
            if detections is not None:
                publish("/detections", Detection2DArray, detections.to_ros_detection2d_array())
            for key, topic, msg_type in [
                ("lidar_frame", "/lidar", PointCloud2),
                ("image_frame", "/image", Image),
                ("camera_info", "/camera_info", CameraInfo),
            ]:
                value = moment.get(key)
                if value is not None:
                    publish(topic, msg_type, value)
            buffer = moment.get("tf")
            transforms = moment.get("transforms")
            if buffer is not None and transforms is not None:
                buffer.publish(*transforms)

    return publisher


@pytest.fixture(scope="session")
def imageDetections2d(get_moment_2d) -> ImageDetections2D:
    moment = get_moment_2d()
    assert len(moment["detections2d"]) > 0, "No detections found in the moment"
    return moment["detections2d"]


@pytest.fixture(scope="session")
def detection2d(get_moment_2d) -> Detection2D:
    moment = get_moment_2d()
    assert len(moment["detections2d"]) > 0, "No detections found in the moment"
    return moment["detections2d"][0]


@pytest.fixture(scope="session")
def detections3dpc(get_moment_3dpc) -> Detection3DPC:
    moment = get_moment_3dpc(seek=10.0)
    assert len(moment["detections3dpc"]) > 0, "No detections found in the moment"
    return moment["detections3dpc"]


@pytest.fixture(scope="session")
def detection3dpc(detections3dpc) -> Detection3DPC:
    return detections3dpc[0]


@pytest.fixture(scope="session")
def get_moment_2d(get_moment, request) -> Generator[Callable[[], Moment2D], None, None]:
    c = front_camera_calibration()
    module = Detection2DModule(
        detector=lambda: Yolo2DDetector(model_path=str(_test_data("models_yolo")), device="cpu"),
        camera_info=c,
    )
    request.addfinalizer(lambda: _dispose_module(module))

    @functools.lru_cache(maxsize=1)
    def moment_provider(**kwargs) -> Moment2D:
        moment = get_moment(**kwargs)
        detections = module.process_image_frame(moment.get("image_frame"))

        return {
            **moment,
            "detections2d": detections,
        }

    yield moment_provider

    moment_provider.cache_clear()


@pytest.fixture(scope="session")
def get_moment_3dpc(get_moment_2d, request) -> Generator[Callable[[], Moment3D], None, None]:
    module: Detection3DModule | None = None

    @functools.lru_cache(maxsize=1)
    def moment_provider(**kwargs) -> Moment3D:
        nonlocal module
        moment = get_moment_2d(**kwargs)

        if not module:
            module = Detection3DModule(
                camera_info=moment["camera_info"],
                detector=lambda: Yolo2DDetector(
                    model_path=str(_test_data("models_yolo")), device="cpu"
                ),
            )
            request.addfinalizer(lambda: _dispose_module(module))

        lidar_frame = moment.get("lidar_frame")
        if lidar_frame is None:
            raise ValueError("No lidar frame found")

        camera_transform = moment["tf"].get("camera_optical", lidar_frame.header.frame_id)
        if camera_transform is None:
            raise ValueError("No camera_optical transform in tf")

        detections3dpc = module.process_frame(
            moment["detections2d"], moment["lidar_frame"], camera_transform
        )

        return {
            **moment,
            "detections3dpc": detections3dpc,
        }

    yield moment_provider
    moment_provider.cache_clear()


@pytest.fixture(scope="session")
def object_db_module(get_moment, request):
    """Create and populate an ObjectDBModule with detections from multiple frames."""
    c = front_camera_calibration()
    module2d = Detection2DModule(
        detector=lambda: Yolo2DDetector(model_path=str(_test_data("models_yolo")), device="cpu"),
        camera_info=c,
    )
    request.addfinalizer(lambda: _dispose_module(module2d))
    module3d = Detection3DModule(
        camera_info=front_camera_calibration(),
        detector=lambda: Yolo2DDetector(model_path=str(_test_data("models_yolo")), device="cpu"),
    )
    request.addfinalizer(lambda: _dispose_module(module3d))
    moduleDB = ObjectDBModule(
        camera_info=front_camera_calibration(),
        detector=lambda: Yolo2DDetector(model_path=str(_test_data("models_yolo")), device="cpu"),
    )
    request.addfinalizer(lambda: _dispose_module(moduleDB))

    # Process 5 frames to build up object history
    for i in range(5):
        seek_value = 10.0 + (i * 2)
        moment = get_moment(seek=seek_value)

        # Process 2D detections
        imageDetections2d = module2d.process_image_frame(moment["image_frame"])

        # Get camera transform
        camera_transform = moment["tf"].get("camera_optical", moment["lidar_frame"].header.frame_id)

        # Process 3D detections
        imageDetections3d = module3d.process_frame(
            imageDetections2d, moment["lidar_frame"], camera_transform
        )

        # Add to database
        moduleDB.add_detections(imageDetections3d)

    yield moduleDB


@pytest.fixture(scope="session")
def first_object(object_db_module):
    """Get the first object from the database."""
    objects = list(object_db_module.objects.values())
    assert len(objects) > 0, "No objects found in database"
    return objects[0]


@pytest.fixture(scope="session")
def all_objects(object_db_module):
    """Get all objects from the database."""
    return list(object_db_module.objects.values())
