# Copyright 2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

import numpy as np

from dimos.models.openpi.camera_bridge import OpenPIYAMCameraBridge


def test_cameras_stay_open_across_batches_and_release_on_stop(mocker):
    module = OpenPIYAMCameraBridge()
    captures = [mocker.Mock() for _ in range(3)]
    for capture in captures:
        capture.isOpened.return_value = True
        capture.grab.return_value = True
        capture.retrieve.return_value = (True, np.array([[[10, 20, 30]]], dtype=np.uint8))
    factory = mocker.patch(
        "dimos.models.openpi.camera_bridge.cv2.VideoCapture", side_effect=captures
    )
    top = mocker.patch.object(module.top_image, "publish")
    left = mocker.patch.object(module.left_wrist_image, "publish")
    right = mocker.patch.object(module.right_wrist_image, "publish")
    mocker.patch.object(module._stop, "wait", return_value=False)

    def stop_after_two_batches(image):
        if right.call_count == 2:
            module._stop.set()

    right.side_effect = stop_after_two_batches
    try:
        module._capture_loop()
        assert factory.call_count == 3
        assert top.call_count == left.call_count == right.call_count == 2
        np.testing.assert_array_equal(top.call_args.args[0].data, [[[30, 20, 10]]])
        for capture in captures:
            assert capture.grab.call_count == 2
            capture.release.assert_called_once()
    finally:
        module.stop()
