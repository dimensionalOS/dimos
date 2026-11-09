import numpy as np

from dimos.memory.codecs.base import codec_from_id, codec_id
from dimos.memory.codecs.cdrjpeg import CdrjpegCodec
from dimos.message_utils import image
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.std_msgs.msg import Header


def test_generated_jpeg_storage_preserves_header_and_pixels():
    pixels = np.full((8, 8, 3), [20, 80, 140], dtype=np.uint8)
    original = image.from_array(
        pixels, encoding="rgb8", header=Header(stamp=Time(12, 500_000_000), frame_id="camera")
    )
    codec = CdrjpegCodec()
    wire = codec.encode(original)
    assert wire[:2] == b"\0\1"
    assert codec_id(codec) == "cdrjpeg"
    result = codec_from_id("cdrjpeg", "sensor_msgs/msg/Image").decode(wire)
    assert result.header == original.header
    assert result.encoding == "rgb8"
    assert np.mean(np.abs(image.view(result).astype(float) - pixels.astype(float))) < 5


def test_big_endian_depth_jpeg_retains_historical_high_byte_normalization():
    original = image.from_array(
        np.full((8, 8), 128 * 256, dtype=">u2"),
        encoding="mono16",
        header=Header(stamp=Time(7, 9), frame_id="depth"),
    )
    codec = CdrjpegCodec()
    result = codec.decode(codec.encode(original))
    assert result.header == original.header
    assert result.encoding == "rgb8"
    assert np.max(np.abs(image.view(result).astype(int) - 128)) <= 1
