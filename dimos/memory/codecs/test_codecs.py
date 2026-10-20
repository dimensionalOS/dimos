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

"""Grid tests for Codec implementations.

Runs roundtrip encode→decode tests across every codec, verifying data preservation.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from dimos_cdrgen.registry import decode
import numpy as np
import pytest

from dimos.memory.codecs.base import Codec, codec_for, codec_from_id, codec_id
from dimos.memory.codecs.cdr import CdrCodec
from dimos.memory.codecs.jpeg import JpegCdrCodec, JpegCodec
from dimos.memory.codecs.lcm import LcmCodec
from dimos.memory.codecs.pickle import PickleCodec
from dimos.memory.store.sqlite import SqliteStore
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.sensor_msgs.msg import CompressedImage, Image as GeneratedImage, PointCloud2
from dimos.messages.std_msgs.msg import Header, Int32
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.image import image_from_array, image_to_rgb
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.msgs.protocol import DimosMsg


@dataclass
class Case:
    name: str
    codec: Codec[Any]
    values: list[Any]
    eq: Callable[[Any, Any], bool] | None = None  # custom equality: (original, decoded) -> bool


def _lcm_values() -> list[DimosMsg]:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

    return [
        PoseStamped(
            ts=1.0,
            frame_id="map",
            position=Vector3(1.0, 2.0, 3.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        PoseStamped(ts=0.5, frame_id="odom"),
    ]


def _pickle_case() -> Case:
    from dimos.memory.codecs.pickle import PickleCodec

    return Case(
        name="pickle",
        codec=PickleCodec(),
        values=[42, "hello", b"raw bytes", {"key": "value"}],
    )


def _lcm_case() -> Case:
    from dimos.memory.codecs.lcm import LcmCodec
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

    return Case(
        name="lcm",
        codec=LcmCodec(PoseStamped),
        values=_lcm_values(),
    )


def _lz4_pickle_case() -> Case:
    from dimos.memory.codecs.lz4 import Lz4Codec
    from dimos.memory.codecs.pickle import PickleCodec

    return Case(
        name="lz4+pickle",
        codec=Lz4Codec(PickleCodec()),
        values=[42, "hello", b"raw bytes", {"key": "value"}, list(range(1000))],
    )


def _lz4_lcm_case() -> Case:
    from dimos.memory.codecs.lcm import LcmCodec
    from dimos.memory.codecs.lz4 import Lz4Codec
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

    return Case(
        name="lz4+lcm",
        codec=Lz4Codec(LcmCodec(PoseStamped)),
        values=_lcm_values(),
    )


def _jpeg_eq(original: Any, decoded: Any) -> bool:
    """JPEG is lossy and normalizes to RGB — check shape, frame_id, RGB tag, and color closeness.

    Compares against ``original.to_rgb()`` because the codec normalizes everything to RGB on
    the wire (so a BGR-tagged input comes back RGB-tagged with channels swapped accordingly).
    """
    import numpy as np

    if decoded.data.shape != original.data.shape:
        return False
    if decoded.frame_id != original.frame_id:
        return False
    if decoded.format != ImageFormat.RGB:
        return False
    expected = original.to_rgb().data
    return bool(np.mean(np.abs(decoded.data.astype(float) - expected.astype(float))) < 5)


def _turbojpeg_available() -> bool:
    try:
        from turbojpeg import TurboJPEG

        TurboJPEG()  # fail fast if native lib is missing
    except (ImportError, RuntimeError):
        return False
    return True


def _jpeg_case() -> Case | None:
    if not _turbojpeg_available():
        return None

    import numpy as np

    # smooth gradients survive lossy jpeg within the eq tolerance
    frames = []
    for shift in (0, 90, 180):
        arr = np.zeros((48, 64, 3), np.uint8)
        arr[..., 0] = np.linspace(0, 255, 64, dtype=np.uint8)
        arr[..., 1] = np.linspace(0, 255, 48, dtype=np.uint8)[:, None]
        arr[..., 2] = shift
        frames.append(Image(data=arr, format=ImageFormat.RGB, frame_id="cam", ts=1.0))

    return Case(
        name="jpeg",
        codec=JpegCodec(quality=95),
        values=frames,
        eq=_jpeg_eq,
    )


_case_factories = {
    "pickle": _pickle_case,
    "lcm": _lcm_case,
    "lz4+pickle": _lz4_pickle_case,
    "lz4+lcm": _lz4_lcm_case,
    "jpeg": _jpeg_case,
}

case_params: list[Any] = ["pickle", "lcm", "lz4+pickle", "lz4+lcm"]
if _turbojpeg_available():
    case_params.append("jpeg")


@pytest.fixture
def case(request: pytest.FixtureRequest) -> Case:
    resolved = _case_factories[request.param]()
    if resolved is None:
        pytest.skip(f"no usable data for the {request.param} case")
    return resolved


@pytest.mark.parametrize("case", case_params, indirect=True)
class TestCodecRoundtrip:
    """Every codec must perfectly roundtrip its values."""

    def test_roundtrip_preserves_value(self, case: Case) -> None:
        eq = case.eq or (lambda a, b: a == b)
        for value in case.values:
            encoded = case.codec.encode(value)
            assert isinstance(encoded, bytes)
            decoded = case.codec.decode(encoded)
            assert eq(value, decoded), f"Roundtrip failed for {value!r}: got {decoded!r}"

    def test_encode_returns_nonempty_bytes(self, case: Case) -> None:
        for value in case.values:
            encoded = case.codec.encode(value)
            assert len(encoded) > 0, f"Empty encoding for {value!r}"

    def test_different_values_produce_different_bytes(self, case: Case) -> None:
        encodings = [case.codec.encode(v) for v in case.values]
        assert len(set(encodings)) > 1, "All values encoded to identical bytes"


class TestCodecFor:
    """codec_for() auto-selects the right codec."""

    def test_none_returns_pickle(self) -> None:
        assert isinstance(codec_for(None), PickleCodec)

    def test_unknown_type_returns_pickle(self) -> None:
        assert isinstance(codec_for(dict), PickleCodec)

    def test_lcm_type_returns_lcm(self) -> None:
        assert isinstance(codec_for(PoseStamped), LcmCodec)

    def test_image_type_returns_jpeg(self) -> None:
        pytest.importorskip("turbojpeg")
        from dimos.memory.codecs.jpeg import JpegCodec

        assert isinstance(codec_for(Image), JpegCodec)

    def test_colour_image_sample_returns_jpeg(self) -> None:
        pytest.importorskip("turbojpeg")
        rgb = Image(data=np.zeros((48, 64, 3), dtype=np.uint8), format=ImageFormat.RGB)

        assert isinstance(codec_for(Image, sample=rgb), JpegCodec)

    @pytest.mark.parametrize("fmt", [ImageFormat.DEPTH, ImageFormat.DEPTH16, ImageFormat.GRAY16])
    def test_depth_image_sample_returns_lossless(self, fmt: ImageFormat) -> None:
        """JPEG is 8-bit colour: uint16 depth comes back as wrong metres, float32 does not encode."""
        metres = np.linspace(0.5, 6.0, 48 * 64, dtype=np.float32).reshape(48, 64)
        data = metres if fmt is ImageFormat.DEPTH else (metres * 1000).astype(np.uint16)
        depth = Image(data=data, format=fmt, frame_id="depth_optical", ts=1.0)

        codec = codec_for(Image, sample=depth)
        decoded = codec.decode(codec.encode(depth))

        assert codec_id(codec) == "lz4+lcm"
        assert decoded.format == fmt
        assert decoded.frame_id == "depth_optical"
        np.testing.assert_array_equal(decoded.data, depth.data)


@pytest.mark.parametrize("encoding", ["rgb8", "bgr8", "mono8"])
def test_color_cache_reopens_jpeg_cdr_with_exact_source_header(tmp_path, encoding):
    shape = (8, 8) if encoding == "mono8" else (8, 8, 3)
    pixels = np.full(shape, 100, dtype=np.uint8)
    if encoding != "mono8":
        pixels[:, :, 0] = 20
        pixels[:, :, 2] = 180
    header = Header(frame_id="camera", stamp=Time(sec=1700000000, nanosec=123456789))
    image = image_from_array(pixels, encoding=encoding, header=header)
    codec = JpegCdrCodec()
    # Stored bytes are a real generated CompressedImage, rather than an LCM envelope.
    compressed = decode(codec.encode(image), CompressedImage)
    assert compressed.header == header
    assert "jpeg" in compressed.format
    database = tmp_path / "color.db"
    with SqliteStore(path=str(database)) as store:
        store.stream("color", GeneratedImage, codec="jpegcdr").append(image, ts=2)
    with SqliteStore(path=str(database), must_exist=True) as store:
        restored = store.stream("color").first().data
        assert type(restored) is GeneratedImage
        assert restored.header == header
        np.testing.assert_allclose(
            image_to_rgb(restored).astype(float), image_to_rgb(image), atol=5
        )
    with pytest.raises(TypeError, match="generated Image"):
        codec_from_id("jpegcdr", f"{PointCloud2.__module__}.{PointCloud2.__qualname__}")


@pytest.mark.parametrize("payload", [b"malformed", b"\x00\x07\x00\x00" + bytes(4)])
def test_cdr_storage_rejects_wrong_encapsulation_before_decoding_fields(payload):
    codec = CdrCodec(Int32)
    with pytest.raises(ValueError, match="XCDR1 encapsulation"):
        codec.decode(payload)
    assert codec.decode(codec.encode(Int32(29))) == Int32(29)
