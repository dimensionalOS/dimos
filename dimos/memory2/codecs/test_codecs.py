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

import pytest

from dimos.memory2.codecs.base import Codec, codec_for
from dimos.memory2.codecs.jpeg import JpegCodec
from dimos.memory2.codecs.lcm import LcmCodec
from dimos.memory2.codecs.pickle import PickleCodec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.testing.replay import TimedSensorReplay

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
    from dimos.memory2.codecs.pickle import PickleCodec

    return Case(
        name="pickle",
        codec=PickleCodec(),
        values=[42, "hello", b"raw bytes", {"key": "value"}],
    )


def _lcm_case() -> Case:
    from dimos.memory2.codecs.lcm import LcmCodec
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

    return Case(
        name="lcm",
        codec=LcmCodec(PoseStamped),
        values=_lcm_values(),
    )


def _lz4_pickle_case() -> Case:
    from dimos.memory2.codecs.lz4 import Lz4Codec
    from dimos.memory2.codecs.pickle import PickleCodec

    return Case(
        name="lz4+pickle",
        codec=Lz4Codec(PickleCodec()),
        values=[42, "hello", b"raw bytes", {"key": "value"}, list(range(1000))],
    )


def _lz4_lcm_case() -> Case:
    from dimos.memory2.codecs.lcm import LcmCodec
    from dimos.memory2.codecs.lz4 import Lz4Codec
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

    return Case(
        name="lz4+lcm",
        codec=Lz4Codec(LcmCodec(PoseStamped)),
        values=_lcm_values(),
    )


def _jpeg_eq(original: Any, decoded: Any) -> bool:
    """JPEG is lossy — check shape, frame_id, and pixel closeness."""
    import numpy as np

    if decoded.data.shape != original.data.shape:
        return False
    if decoded.frame_id != original.frame_id:
        return False
    return bool(np.mean(np.abs(decoded.data.astype(float) - original.data.astype(float))) < 5)


def _jpeg_case() -> Case | None:
    try:
        from turbojpeg import TurboJPEG

        TurboJPEG()  # fail fast if native lib is missing

        replay = TimedSensorReplay("go2_bigoffice/color_image")
        frames = [replay.find_closest_seek(float(i)) for i in range(1, 4)]
        codec = JpegCodec(quality=95)
    except (ImportError, RuntimeError):
        return None

    return Case(
        name="jpeg",
        codec=codec,
        values=frames,
        eq=_jpeg_eq,
    )


testcases = [
    c
    for c in [_pickle_case(), _lcm_case(), _lz4_pickle_case(), _lz4_lcm_case(), _jpeg_case()]
    if c is not None
]


@pytest.mark.parametrize("case", testcases, ids=lambda c: c.name)
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
        from dimos.memory2.codecs.jpeg import JpegCodec

        assert isinstance(codec_for(Image), JpegCodec)
