import threading

import numpy as np

from dimos.message_utils import image
from dimos.messages.sensor_msgs.msg import Image
from dimos.protocol.pubsub.impl.jpeg_lcm import JpegLCM
from dimos.protocol.pubsub.impl.lcmpubsub import Topic


def test_generated_jpeg_lcm_keeps_image_values_at_public_boundary():
    bus = JpegLCM(url="memq://")
    bus.start()
    arrived = threading.Event()
    values = []

    def receive(value, topic):
        values.append(value)
        arrived.set()

    unsubscribe = bus.subscribe(Topic("image", Image), receive)
    try:
        original = image.from_array(np.full((8, 8, 3), [20, 80, 140], np.uint8), encoding="rgb8")
        bus.publish("image", original)
        assert arrived.wait(2)
        result = values[0]
        assert type(result) is Image
        assert result.header == original.header
        assert (
            np.mean(np.abs(image.to_rgb(result).astype(float) - image.view(original).astype(float)))
            < 5
        )
    finally:
        unsubscribe()
        bus.stop()
