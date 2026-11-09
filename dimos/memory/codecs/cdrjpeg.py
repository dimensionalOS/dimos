"""JPEG-compressed generated Image storage with a CDR envelope."""

from dimos_cdrgen.registry import decode, encode
import numpy as np
from turbojpeg import TJPF_RGB, TurboJPEG

from dimos.message_utils import image
from dimos.messages.sensor_msgs.msg import CompressedImage, Image


class CdrjpegCodec:
    payload_type = Image

    def __init__(self, quality: int = 50) -> None:
        self.quality = quality
        self._jpeg = TurboJPEG()

    def encode(self, value: Image) -> bytes:
        if value.encoding in ("mono16", "16UC1", "16SC1"):
            pixels = (image.view(value).astype(np.uint16) >> 8).astype(np.uint8)
            value = image.from_array(pixels, encoding="mono8", header=value.header)
        return encode(image.to_compressed(value, format="jpeg", quality=self.quality))

    def decode(self, data: bytes) -> Image:
        compressed = decode(data, CompressedImage)
        pixels = self._jpeg.decode(bytes(compressed.data), pixel_format=TJPF_RGB)
        return image.from_array(pixels, encoding="rgb8", header=compressed.header)
