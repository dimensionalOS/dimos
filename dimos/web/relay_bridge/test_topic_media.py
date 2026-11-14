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

from io import BytesIO
import json
from pathlib import Path
import zlib

from dimos_lcm.sensor_msgs import CompressedImage, Image, PointCloud2
import numpy as np
from PIL import Image as PILImage
import pytest

from dimos.web.relay_bridge.topic_media import decode_color, encode_media
from dimos.web.relay_bridge.topic_profiles import (
    MEDIA_PROFILES,
    PROFILE_OPTIONS,
    normalize_settings,
)

VECTORS = json.loads(
    (Path(__file__).parents[3] / "web/shared/fixtures/topic_media.json").read_text()
)


@pytest.mark.parametrize("vector", VECTORS, ids=lambda vector: vector["name"])
def test_media_golden_vectors(vector):
    result = encode_media(vector["profile"], bytes.fromhex(vector["lcm"]), vector["settings"])
    assert result.metadata == vector["metadata"]
    assert result.encoding == vector["encoding"]
    assert result.payload == bytes.fromhex(vector["payload"])


@pytest.mark.parametrize("profile", sorted(PROFILE_OPTIONS))
def test_normalize_settings_defaults_to_the_profile_and_clamps_to_it(profile):
    defaults = normalize_settings(profile, None)
    options = PROFILE_OPTIONS[profile]
    assert defaults == {name: option["default"] for name, option in options.items()}
    assert normalize_settings(profile, defaults) == defaults
    assert normalize_settings(profile, {name: None for name in options}) == defaults
    media = MEDIA_PROFILES.get(profile)
    assert defaults["maxHz"] == (media.max_hz if media else 1000)
    assert normalize_settings(profile, {"maxHz": 1000})["maxHz"] == defaults["maxHz"]
    if "quality" in options:
        assert normalize_settings(profile, {"quality": 100})["quality"] == media.quality
        assert normalize_settings(profile, {"quality": 1})["quality"] == 1
        boxed = normalize_settings(profile, {"maxResolution": [4096, 16]})
        assert boxed["maxResolution"] == [media.width, 16]
    if "keepEvery" in options:
        assert normalize_settings(profile, {"keepEvery": 1})["keepEvery"] == media.keep_every
        assert normalize_settings(profile, {"keepEvery": 64})["keepEvery"] == 64
        assert normalize_settings(profile, {"fields": []})["fields"] == []
        assert normalize_settings(profile, {"fields": ["color"]})["fields"] == ["color"]


@pytest.mark.parametrize(
    ("profile", "requested", "code"),
    [
        ("image.jpeg.low.v1", {"keepEvery": 2}, "unsupported_option"),
        ("image.jpeg.low.v1", {"maxBitrate": 1}, "unsupported_option"),
        ("raw", {"quality": 50}, "unsupported_option"),
        ("image.jpeg.low.v1", {"quality": 0}, "option_out_of_range"),
        ("image.jpeg.low.v1", {"quality": 50.5}, "option_out_of_range"),
        ("image.jpeg.low.v1", {"maxResolution": [8, 8]}, "option_out_of_range"),
        ("image.jpeg.low.v1", {"maxResolution": [640]}, "option_out_of_range"),
        ("image.jpeg.low.v1", {"maxHz": 0}, "option_out_of_range"),
        ("cloud.q16.full.v1", {"keepEvery": 65}, "option_out_of_range"),
        ("cloud.q16.full.v1", {"fields": ["normals"]}, "option_out_of_range"),
        ("cloud.q16.full.v1", {"fields": ["color", "color"]}, "option_out_of_range"),
    ],
)
def test_normalize_settings_rejects_unknown_names_and_values_outside_the_ranges(
    profile, requested, code
):
    with pytest.raises(ValueError, match=code):
        normalize_settings(profile, requested)


def test_settings_shape_the_jpeg_box_and_quality_and_report_the_values_in_force():
    vector = next(v for v in VECTORS if v["name"] == "color_rgb_boxed")
    full = encode_media(vector["profile"], bytes.fromhex(vector["lcm"]))
    boxed = encode_media(vector["profile"], bytes.fromhex(vector["lcm"]), vector["settings"])
    with PILImage.open(BytesIO(boxed.payload)) as image:
        assert image.size == (16, 8)
    assert (boxed.metadata["w"], boxed.metadata["h"], boxed.metadata["quality"]) == (16, 8, 40)
    assert (full.metadata["w"], full.metadata["h"], full.metadata["quality"]) == (32, 16, 75)
    # Above the profile the request is a no-op reported as the profile values.
    ceiling = encode_media(
        vector["profile"], bytes.fromhex(vector["lcm"]), {"maxResolution": [4096, 4096]}
    )
    assert ceiling.payload == full.payload


def test_cloud_settings_stride_the_finite_points_and_select_the_optional_buffers():
    vector = next(v for v in VECTORS if v["name"] == "cloud")
    data = bytes.fromhex(vector["lcm"])
    strided = encode_media(vector["profile"], data, {"keepEvery": 2, "fields": ["color"]})
    decoded = zlib.decompress(strided.payload)
    assert (strided.metadata["count"], strided.metadata["keepEvery"]) == (2, 2)
    assert (strided.metadata["intensity"], strided.metadata["color"]) == (False, True)
    assert len(decoded) == 2 * 6 + 2 * 3
    assert decoded[12:] == bytes.fromhex("112233123456")
    bare = encode_media(vector["profile"], data, {"fields": []})
    assert len(zlib.decompress(bare.payload)) == 3 * 6
    assert (bare.metadata["intensity"], bare.metadata["color"]) == (False, False)
    # The low profile's stride is a floor: asking for every point keeps one in four.
    low = encode_media("cloud.q16.low.v1", data, {"keepEvery": 1})
    assert low.metadata["keepEvery"] == 4


@pytest.mark.parametrize(
    "vector", [v for v in VECTORS if v["expectedDepth"]], ids=lambda v: v["name"]
)
def test_depth_is_bit_exact_including_nan_payloads_and_signed_zero(vector):
    result = encode_media(vector["profile"], bytes.fromhex(vector["lcm"]))
    assert zlib.decompress(result.payload).hex() == vector["expectedDepth"]


@pytest.mark.parametrize("name", ["color_rgb", "color_bgr", "color_compressed"])
def test_color_layouts_encode_the_same_visible_red(name):
    vector = next(v for v in VECTORS if v["name"] == name)
    result = encode_media(vector["profile"], bytes.fromhex(vector["lcm"]))
    with PILImage.open(BytesIO(result.payload)) as image:
        pixels = np.asarray(image)
        assert image.size == (16, 8)
        assert pixels[..., 0].min() >= 250
        assert pixels[..., 1:].max() <= 5


@pytest.mark.parametrize("encoding", ["mono8", "8UC1"])
def test_grayscale_layouts_decode_to_rgb_for_every_shared_encoder(encoding):
    message = Image.lcm_decode(
        bytes.fromhex(next(v for v in VECTORS if v["name"] == "color_rgb")["lcm"])
    )
    message.height, message.width, message.encoding = 2, 2, encoding
    message.step = 2
    message.data = bytes([0, 80, 160, 240])
    message.data_length = 4
    payload = message.lcm_encode()
    image, _ = decode_color(payload)
    assert image.mode == "RGB"
    assert np.asarray(image).tolist() == [[[0] * 3, [80] * 3], [[160] * 3, [240] * 3]]
    result = encode_media("image.jpeg.balanced.v1", payload)
    assert result.encoding == "image/jpeg"


def test_camera_profile_resizes_before_jpeg_encoding():
    vector = next(v for v in VECTORS if v["name"] == "color_rgb")
    message = Image.lcm_decode(bytes.fromhex(vector["lcm"]))
    message.width, message.height, message.step = 1280, 720, 1280 * 3
    message.data = bytes(message.step * message.height)
    message.data_length = len(message.data)
    result = encode_media("image.jpeg.low.v1", message.lcm_encode())
    with PILImage.open(BytesIO(result.payload)) as image:
        assert image.size == (640, 360)


def test_compressed_camera_rejects_oversized_shape_before_decoding():
    vector = next(v for v in VECTORS if v["name"] == "color_compressed")
    message = CompressedImage.lcm_decode(bytes.fromhex(vector["lcm"]))
    with PILImage.new("RGB", (4097, 1024)) as image:
        encoded = BytesIO()
        image.save(encoded, "JPEG")
    message.data = encoded.getvalue()
    message.data_length = len(message.data)
    with pytest.raises(ValueError, match="dimensions"):
        encode_media("image.jpeg.low.v1", message.lcm_encode())


def test_compressed_color_cannot_be_used_as_numeric_depth():
    vector = next(v for v in VECTORS if v["name"] == "color_compressed")
    with pytest.raises(ValueError, match="depth profile"):
        encode_media("depth.lossless.v1", bytes.fromhex(vector["lcm"]))


@pytest.mark.parametrize("name", ["cloud", "cloud_big"])
def test_cloud_quantization_filters_nan_and_preserves_color_and_intensity(name):
    vector = next(v for v in VECTORS if v["name"] == name)
    result = encode_media(vector["profile"], bytes.fromhex(vector["lcm"]))
    meta = result.metadata
    decoded = zlib.decompress(result.payload)
    positions = np.frombuffer(decoded, dtype="<i2", count=9).reshape(3, 3)
    positions = positions * meta["scale"] + np.asarray(meta["origin"])
    np.testing.assert_allclose(
        positions, [[1, 2, 3], [10, -20, 30], [-5, 6, 7]], atol=meta["maxError"] + 1e-10
    )
    np.testing.assert_array_equal(
        np.frombuffer(decoded, dtype="<f4", count=3, offset=18), [4, 40, 80]
    )
    assert decoded[30:] == bytes.fromhex("112233ffeedd123456")
    assert (meta["sourceCount"], meta["count"]) == (4, 3)


@pytest.mark.parametrize("profile", ["image.jpeg.balanced.v1", "depth.lossless.v1"])
def test_image_rejects_impossible_dimensions_before_allocating(profile):
    vector = next(v for v in VECTORS if v["name"] == "depth16")
    message = Image.lcm_decode(bytes.fromhex(vector["lcm"]))
    message.width = 2**31 - 1
    with pytest.raises(ValueError, match="dimensions"):
        encode_media(profile, message.lcm_encode())


@pytest.mark.parametrize("change", ["offset", "stride", "dimensions", "duplicate"])
def test_cloud_rejects_invalid_layout_before_expanding(change):
    vector = next(v for v in VECTORS if v["name"] == "cloud")
    message = PointCloud2.lcm_decode(bytes.fromhex(vector["lcm"]))
    if change == "offset":
        message.fields[0].offset = 2**31 - 1
    elif change == "stride":
        message.row_step = 1
    elif change == "dimensions":
        message.width = 2**31 - 1
    else:
        message.fields[1].name = "x"
    with pytest.raises(ValueError):
        encode_media(vector["profile"], message.lcm_encode())


@pytest.mark.parametrize("vector", VECTORS, ids=lambda vector: vector["name"])
def test_media_rejects_truncation_wrong_fingerprint_and_trailing_bytes(vector):
    data = bytes.fromhex(vector["lcm"])
    for corrupt in (data[:-1], b"wrong-fp" + data[8:], data + b"\0"):
        with pytest.raises(ValueError):
            encode_media(vector["profile"], corrupt)
