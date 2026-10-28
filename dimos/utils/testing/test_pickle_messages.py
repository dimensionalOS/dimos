# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
"""Historical byte fixtures remain readable after retiring their original classes."""

import base64
import builtins
import io

import numpy as np

from dimos.message_utils import image, pointcloud
from dimos.utils.testing.pickle_messages import load_message_pickle

FIXTURES = {
    "image": "gASVEgEAAAAAAACMHGRpbW9zLm1zZ3Muc2Vuc29yX21zZ3MuSW1hZ2WUjAVJbWFnZZSTlCmBlH2UKIwEZGF0YZSMFm51bXB5Ll9jb3JlLm11bHRpYXJyYXmUjAxfcmVjb25zdHJ1Y3SUk5SMBW51bXB5lIwHbmRhcnJheZSTlEsAhZRDAWKUh5RSlChLAUsBSwJLA4eUaAmMBWR0eXBllJOUjAJ1MZSJiIeUUpQoSwOMAXyUTk5OSv////9K/////0sAdJRiiUMGAQIDBAUGlHSUYowGZm9ybWF0lGgAjAtJbWFnZUZvcm1hdJSTlIwDQkdSlIWUUpSMCGZyYW1lX2lklIwGY2FtZXJhlIwCdHOURz/0AAAAAAAAdWIu",
    "pose": "gASVoAEAAAAAAACMJGRpbW9zLm1zZ3MuZ2VvbWV0cnlfbXNncy5Qb3NlU3RhbXBlZJSMC1Bvc2VTdGFtcGVklJOUKYGUfZQojAhmcmFtZV9pZJSMA21hcJSMAnRzlEc/9AAAAAAAAHV9lCiMCHBvc2l0aW9ulIwgZGltb3MubXNncy5nZW9tZXRyeV9tc2dzLlZlY3RvcjOUjAdWZWN0b3IzlJOUKYGUfZQojAF4lEc/8AAAAAAAAIwBeZRHQAAAAAAAAACMAXqUR0AIAAAAAAAAdX2UKGgPRz/wAAAAAAAAaBBHQAAAAAAAAABoEUdACAAAAAAAAHWGlGKMC29yaWVudGF0aW9ulIwjZGltb3MubXNncy5nZW9tZXRyeV9tc2dzLlF1YXRlcm5pb26UjApRdWF0ZXJuaW9ulJOUKYGUfZQoaA9HAAAAAAAAAABoEEcAAAAAAAAAAGgRRwAAAAAAAAAAjAF3lEc/8AAAAAAAAHV9lChoD0cAAAAAAAAAAGgQRwAAAAAAAAAAaBFHAAAAAAAAAABoGkc/8AAAAAAAAHWGlGJ1hpRiLg==",
    "cloud": "gASVHwEAAAAAAACMImRpbW9zLm1zZ3Muc2Vuc29yX21zZ3MuUG9pbnRDbG91ZDKUjAtQb2ludENsb3VkMpSTlCmBlH2UKIwCdHOURz/0AAAAAAAAjAhmcmFtZV9pZJSMBWxpZGFylIwDc2VxlEsAjBFfcGNkX2xlZ2FjeV9jYWNoZZROjApfcGNkX251bXB5lIwWbnVtcHkuX2NvcmUubXVsdGlhcnJheZSMDF9yZWNvbnN0cnVjdJSTlIwFbnVtcHmUjAduZGFycmF5lJOUSwCFlEMBYpSHlFKUKEsBSwFLA4aUaA6MBWR0eXBllJOUjAJmNJSJiIeUUpQoSwOMATyUTk5OSv////9K/////0sAdJRiiUMMAACAPwAAAEAAAEBAlHSUYnViLg==",
}


def test_historical_messages_decode_without_importing_retired_classes(monkeypatch):
    original_import = builtins.__import__

    def guarded_import(name, *args, **kwargs):
        if name.startswith("dimos.msgs."):
            raise AssertionError("Historical message classes must not be imported")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    decoded = {
        name: load_message_pickle(io.BytesIO(base64.b64decode(payload)))
        for name, payload in FIXTURES.items()
    }
    assert decoded["image"].__msgtype__ == "sensor_msgs/msg/Image"
    assert decoded["image"].encoding == "bgr8"
    np.testing.assert_array_equal(image.view(decoded["image"]), [[[1, 2, 3], [4, 5, 6]]])
    assert decoded["pose"].__msgtype__ == "geometry_msgs/msg/PoseStamped"
    assert (
        decoded["pose"].pose.position.x,
        decoded["pose"].pose.position.y,
        decoded["pose"].pose.position.z,
    ) == (1, 2, 3)
    np.testing.assert_array_equal(pointcloud.xyz(decoded["cloud"]), [[1, 2, 3]])
    for name, frame in [("image", "camera"), ("pose", "map"), ("cloud", "lidar")]:
        assert decoded[name].header.frame_id == frame
        assert (decoded[name].header.stamp.sec, decoded[name].header.stamp.nanosec) == (
            1,
            250000000,
        )
