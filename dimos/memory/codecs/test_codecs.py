# Copyright 2026 Dimensional Inc.
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

"""Storage uses generated CDR without altering message fields or image pixels."""

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped
from dimos_generated.sensor_msgs.msg import CompressedImage, Image
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.memory.codecs.base import codec_for, codec_from_id, codec_id
from dimos.memory.codecs.cdr import CdrCodec
from dimos.memory.codecs.lz4 import Lz4Codec
from dimos.memory.codecs.pickle import PickleCodec
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry import transform_from_pose
from dimos.msgs.image import image_from_array, image_to_jpeg, image_view


@pytest.fixture(params=["pose", "image", "depth", "compressed"])
def message(request):
    header = Header(frame_id="sensor", stamp=Time(sec=1700000000, nanosec=123456789))
    if request.param == "pose":
        return PoseStamped(header=header, pose=Pose(position=Point(x=1.25, y=-2.5, z=3.75)))
    if request.param == "depth":
        return image_from_array(
            np.array([[0, 1, 65535]], dtype=np.uint16), encoding="16UC1", header=header
        )
    image = image_from_array(
        np.full((8, 8, 3), [20, 80, 140], dtype=np.uint8), encoding="rgb8", header=header
    )
    if request.param == "compressed":
        return CompressedImage(
            header=header, format="rgb8; jpeg compressed bgr8", data=image_to_jpeg(image)
        )
    return image


@pytest.mark.parametrize("wrapped", [False, True])
def test_codec_roundtrip_and_reconstruction_preserve_generated_value(message, wrapped):
    codec = Lz4Codec(CdrCodec(type(message))) if wrapped else codec_for(type(message))
    name = codec_id(codec)
    assert name == ("lz4+cdr" if wrapped else "cdr")
    restored = codec_from_id(name, f"{type(message).__module__}.{type(message).__qualname__}")
    encoded = codec.encode(message)
    decoded = restored.decode(encoded)
    assert decoded == message
    assert decoded.header.stamp.nanosec == 123456789
    if not wrapped:
        assert encoded == message.encode()
    if isinstance(message, Image):
        np.testing.assert_array_equal(image_view(decoded), image_view(message))


@pytest.mark.parametrize("wrapped", [False, True])
def test_sqlite_reopen_uses_cdr_and_preserves_exact_message(message, wrapped, tmp_path):
    path = tmp_path / "messages.db"
    with SqliteStore(path=str(path)) as store:
        stream = store.stream("samples", type(message), codec="lz4+cdr" if wrapped else "cdr")
        stream.append(message, ts=12.5)
    with SqliteStore(path=str(path), must_exist=True) as store:
        observation = store.stream("samples").first()
        assert observation.ts == 12.5
        assert observation.data == message
        assert observation.data.encode() == message.encode()


@pytest.mark.parametrize("identifier", ["lcm", "lz4+lcm", "jpeg"])
def test_obsolete_storage_codecs_are_rejected(identifier):
    with pytest.raises(ValueError, match="Unknown codec"):
        codec_from_id(identifier, "builtins.dict")


def test_cdr_requires_a_generated_message_type():
    with pytest.raises(TypeError, match="not a generated CDR"):
        codec_from_id("cdr", "builtins.dict")


def test_cdr_malformed_payload_raises():
    with pytest.raises((ValueError, RuntimeError)):
        CdrCodec(PoseStamped).decode(b"bad")


@pytest.mark.parametrize("value", [42, "hello", b"raw bytes", {"key": "value"}])
def test_python_objects_keep_the_explicit_python_storage_path(value):
    codec = codec_for(type(value))
    assert isinstance(codec, PickleCodec)
    assert codec.decode(codec.encode(value)) == value


@pytest.mark.parametrize("stamped_transform", [False, True])
def test_observation_pose_metadata_accepts_nested_generated_values(stamped_transform):
    pose = PoseStamped(header=Header(frame_id="world"), pose=Pose(position=Point(x=1, y=2, z=3)))
    value = transform_from_pose(pose, child_frame_id="base") if stamped_transform else pose
    observation = Observation(id=0, ts=-0.5, pose=value, _data="payload")
    assert observation.pose_tuple == (1, 2, 3, 0, 0, 0, 1)
    assert observation.pose == pose.pose
    assert observation.pose_stamped.pose == pose.pose
    assert observation.pose_stamped.header.stamp == Time(sec=-1, nanosec=500000000)


def test_cdr_rejects_a_message_of_the_wrong_type():
    with pytest.raises(TypeError, match="Expected geometry_msgs/msg/PoseStamped"):
        CdrCodec(PoseStamped).encode(Image())
