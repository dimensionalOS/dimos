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

from __future__ import annotations

import imagecodecs
import numpy as np
import pytest

from dimos.memory.codecs.base import codec_from_id, codec_id
from dimos.memory.codecs.lerc import MAX_ERROR_METERS, LercCodec
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def _float_depth() -> Image:
    pixels = np.array(
        [
            [0.25, 0.5031, np.nan, np.inf],
            [1.0079, 2.501, -np.inf, -1.0],
            [4.999, 7.1234, 0.0, 9.8765],
        ],
        dtype=np.float32,
    )
    return Image(data=pixels, format=ImageFormat.DEPTH, frame_id="depth_optical", ts=42.125)


def _uint16_depth() -> Image:
    pixels = np.array(
        [
            [250, 503, 0, 65535],
            [1008, 2501, 4200, 9999],
            [5000, 7123, 8100, 15000],
        ],
        dtype=np.uint16,
    )
    return Image(data=pixels, format=ImageFormat.DEPTH16, frame_id="depth_optical", ts=42.125)


@pytest.mark.parametrize("source", [_float_depth(), _uint16_depth()])
def test_roundtrip_preserves_depth_contract(source: Image) -> None:
    decoded = LercCodec().decode(LercCodec().encode(source))

    expected_valid = np.isfinite(source.data) & (source.data > 0)
    actual_valid = np.isfinite(decoded.data) & (decoded.data > 0)
    scale = 1.0 if source.format is ImageFormat.DEPTH else 0.001
    max_error_m = float(
        np.max(
            np.abs(
                decoded.data[expected_valid].astype(np.float64)
                - source.data[expected_valid].astype(np.float64)
            )
        )
        * scale
    )

    assert decoded.format is source.format
    assert decoded.dtype == source.dtype
    assert decoded.shape == source.shape
    assert decoded.frame_id == source.frame_id
    assert decoded.ts == pytest.approx(source.ts)
    assert np.array_equal(actual_valid, expected_valid)
    assert max_error_m <= MAX_ERROR_METERS


def test_float_invalid_samples_decode_as_nan() -> None:
    decoded = LercCodec().decode(LercCodec().encode(_float_depth()))

    expected_invalid = ~(np.isfinite(_float_depth().data) & (_float_depth().data > 0))

    assert np.all(np.isnan(decoded.data[expected_invalid]))


def test_uint16_invalid_samples_decode_as_zero() -> None:
    decoded = LercCodec().decode(LercCodec().encode(_uint16_depth()))

    assert decoded.data[0, 2] == 0


@pytest.mark.parametrize(
    ("pixels", "image_format"),
    [
        (np.array([[0.001]], dtype=np.float32), ImageFormat.DEPTH),
        (np.array([[1]], dtype=np.uint16), ImageFormat.DEPTH16),
    ],
)
def test_small_positive_depth_remains_valid(pixels: np.ndarray, image_format: ImageFormat) -> None:
    source = Image(data=pixels, format=image_format)

    decoded = LercCodec().decode(LercCodec().encode(source))

    assert decoded.data[0, 0] > 0


@pytest.mark.parametrize(
    ("image_format", "dtype"),
    [
        (ImageFormat.RGB, np.uint8),
        (ImageFormat.GRAY16, np.uint16),
        (ImageFormat.DEPTH, np.float64),
        (ImageFormat.DEPTH16, np.int16),
    ],
)
def test_encode_rejects_unsupported_image(image_format: ImageFormat, dtype: np.dtype) -> None:
    image = Image(data=np.ones((2, 3), dtype=dtype), format=image_format)

    with pytest.raises(ValueError, match="supports only DEPTH/float32"):
        LercCodec().encode(image)


def test_decode_rejects_wrong_wire_format() -> None:
    payload = CompressedImage(data=b"not lerc", format="png").lcm_encode()

    with pytest.raises(ValueError, match="expected 'lerc' payload"):
        LercCodec().decode(payload)


def test_decode_rejects_corrupt_lerc_payload() -> None:
    payload = CompressedImage(data=b"not lerc", format="lerc").lcm_encode()

    with pytest.raises(imagecodecs.LercError):
        LercCodec().decode(payload)


def test_codec_id_roundtrip() -> None:
    codec = codec_from_id("lerc", "dimos.msgs.sensor_msgs.Image.Image")

    assert isinstance(codec, LercCodec)
    assert codec_id(codec) == "lerc"


def test_sqlite_reopen_restores_lerc_codec(tmp_path) -> None:
    db = str(tmp_path / "depth.db")
    source = _float_depth()
    with SqliteStore(path=db) as store:
        store.stream("depth", Image, codec="lerc").append(source, ts=source.ts)

    with SqliteStore(path=db) as reopened:
        decoded = reopened.stream("depth", Image).first().data

    valid = np.isfinite(source.data) & (source.data > 0)
    assert decoded.frame_id == source.frame_id
    assert float(np.max(np.abs(decoded.data[valid] - source.data[valid]))) <= MAX_ERROR_METERS
