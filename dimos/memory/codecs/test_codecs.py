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

from pydantic import BaseModel, ConfigDict
import pytest

from dimos.memory.codecs.base import Codec, codec_for, codec_from_id, codec_id
from dimos.memory.codecs.jpeg import JpegCodec
from dimos.memory.codecs.lcm import LcmCodec
from dimos.memory.codecs.pickle import PickleCodec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
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


@dataclass(frozen=True)
class _JsonInner:
    __pydantic_config__ = ConfigDict(extra="forbid")

    label: str
    weight: float


@dataclass(frozen=True)
class _JsonRecord:
    """Stand-in for a derived record: versioned, nested, strict."""

    __pydantic_config__ = ConfigDict(extra="forbid")

    schema_version: str
    name: str
    count: int
    parts: tuple[_JsonInner, ...] = ()
    note: str | None = None


class _JsonModel(BaseModel):
    model_config = ConfigDict(extra="forbid")

    schema_version: str
    value: int


def _json_values() -> list[_JsonRecord]:
    return [
        _JsonRecord(schema_version="1", name="empty", count=0),
        _JsonRecord(
            schema_version="1",
            name="nested",
            count=2,
            parts=(_JsonInner(label="a", weight=1.5), _JsonInner(label="b", weight=-0.25)),
            note="with a note",
        ),
    ]


def _json_case() -> Case:
    from dimos.memory.codecs.json import JsonCodec

    return Case(name="json", codec=JsonCodec(_JsonRecord), values=_json_values())


def _lz4_json_case() -> Case:
    from dimos.memory.codecs.json import JsonCodec
    from dimos.memory.codecs.lz4 import Lz4Codec

    return Case(name="lz4+json", codec=Lz4Codec(JsonCodec(_JsonRecord)), values=_json_values())


_case_factories = {
    "pickle": _pickle_case,
    "lcm": _lcm_case,
    "lz4+pickle": _lz4_pickle_case,
    "lz4+lcm": _lz4_lcm_case,
    "jpeg": _jpeg_case,
    "json": _json_case,
    "lz4+json": _lz4_json_case,
}

case_params: list[Any] = ["pickle", "lcm", "lz4+pickle", "lz4+lcm", "json", "lz4+json"]
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

    def test_json_is_never_auto_selected(self) -> None:
        """Opt-in only: auto-selecting it would silently change existing streams."""
        from dimos.memory.codecs.json import JsonCodec

        assert not isinstance(codec_for(_JsonRecord), JsonCodec)


class TestJsonCodecContracts:
    """The two construction-time contracts, and why each exists."""

    def test_rejects_type_that_allows_unknown_fields(self) -> None:
        from dimos.memory.codecs.json import JsonCodec

        @dataclass(frozen=True)
        class Lax:
            schema_version: str

        with pytest.raises(TypeError, match="extra='forbid'"):
            JsonCodec(Lax)

    def test_rejects_type_without_a_version_field(self) -> None:
        from dimos.memory.codecs.json import JsonCodec

        @dataclass(frozen=True)
        class Unversioned:
            __pydantic_config__ = ConfigDict(extra="forbid")

            name: str

        with pytest.raises(TypeError, match="schema_version"):
            JsonCodec(Unversioned)

    def test_version_requirement_can_be_waived(self) -> None:
        from dimos.memory.codecs.json import JsonCodec

        @dataclass(frozen=True)
        class Unversioned:
            __pydantic_config__ = ConfigDict(extra="forbid")

            name: str

        codec = JsonCodec(Unversioned, version_field=None)
        assert codec.decode(codec.encode(Unversioned(name="x"))) == Unversioned(name="x")

    def test_unknown_field_on_decode_raises(self) -> None:
        """A field a newer writer added must surface, not vanish."""
        from dimos.memory.codecs.json import JsonCodec

        codec = JsonCodec(_JsonRecord)
        payload = b'{"schema_version":"1","name":"x","count":1,"parts":[],"from_the_future":7}'
        with pytest.raises(Exception, match="from_the_future"):
            codec.decode(payload)

    def test_missing_required_field_on_decode_raises(self) -> None:
        from dimos.memory.codecs.json import JsonCodec

        codec = JsonCodec(_JsonRecord)
        with pytest.raises(Exception, match="count"):
            codec.decode(b'{"schema_version":"1","name":"x"}')

    def test_pydantic_model_is_supported(self) -> None:
        from dimos.memory.codecs.json import JsonCodec

        codec = JsonCodec(_JsonModel)
        value = _JsonModel(schema_version="1", value=7)
        assert codec.decode(codec.encode(value)) == value

    def test_output_is_human_readable_json(self) -> None:
        """The whole point over pickle: another tool can read it."""
        import json as json_module

        from dimos.memory.codecs.json import JsonCodec

        encoded = JsonCodec(_JsonRecord).encode(_json_values()[1])
        assert json_module.loads(encoded)["parts"][0]["label"] == "a"


class TestJsonCodecRegistry:
    """A stream stored as json must be reopenable from its recorded codec id."""

    def test_codec_id_is_json(self) -> None:
        from dimos.memory.codecs.json import JsonCodec

        assert codec_id(JsonCodec(_JsonRecord)) == "json"

    def test_lz4_wrapper_id_roundtrips(self) -> None:
        from dimos.memory.codecs.json import JsonCodec
        from dimos.memory.codecs.lz4 import Lz4Codec

        assert codec_id(Lz4Codec(JsonCodec(_JsonRecord))) == "lz4+json"

    @pytest.mark.parametrize("cid", ["json", "lz4+json"])
    def test_rebuilt_from_id_still_roundtrips(self, cid: str) -> None:
        module = f"{_JsonRecord.__module__}.{_JsonRecord.__qualname__}"
        codec = codec_from_id(cid, module)
        value = _json_values()[1]
        assert codec.decode(codec.encode(value)) == value
