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

import cv2
import numpy as np
import pytest

from dimos.cli.hand_eye_preview import hand_eye_preview
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


@pytest.fixture
def view(mocker, monkeypatch):
    monkeypatch.setenv("DISPLAY", ":test")
    mocker.patch.object(cv2, "namedWindow")
    mocker.patch.object(cv2, "setWindowTitle")
    mocker.patch.object(cv2, "getWindowProperty", return_value=1.0)
    shown = mocker.patch.object(cv2, "imshow")
    keys = mocker.patch.object(cv2, "waitKey")
    closed = mocker.patch.object(cv2, "destroyWindow")
    return keys, shown, closed


@pytest.mark.parametrize(
    ("key", "expected"),
    [
        (13, "accept"),
        (10, "accept"),
        (ord("r"), "retry"),
        (ord("R"), "retry"),
        (ord("q"), "quit"),
        (ord("Q"), "quit"),
        (27, "quit"),
    ],
)
def test_reviews_display_copy_and_ignores_prior_enter(view, key, expected):
    keys, shown, closed = view
    keys.side_effect = [13, -1, key, -1]
    pixels = np.full((35, 47, 3), 41, dtype=np.uint8)
    image = Image(data=pixels, format=ImageFormat.BGR)

    with hand_eye_preview() as review:
        assert review(image, "Training 2/8") == expected

    _, displayed = shown.call_args.args
    np.testing.assert_array_equal(displayed[0, 0], (41, 41, 41))
    np.testing.assert_array_equal(displayed[17, 23], (255, 255, 255))
    np.testing.assert_array_equal(image.data, np.full_like(pixels, 41))
    closed.assert_called_once()


def test_closing_window_discards_even_if_accept_was_queued(view, mocker):
    keys, _, _ = view
    keys.side_effect = [-1, 13, -1]
    mocker.patch.object(cv2, "getWindowProperty", side_effect=[1, 1, cv2.error("window closed")])
    image = Image(data=np.zeros((32, 32, 3), dtype=np.uint8), format=ImageFormat.BGR)

    with hand_eye_preview() as review:
        assert review(image, "Validation 1/4") == "quit"


def test_unavailable_window_visibility_fails_before_capture_and_closes_window(view, mocker):
    _, _, closed = view
    mocker.patch.object(cv2, "getWindowProperty", return_value=-1.0)

    with pytest.raises(RuntimeError, match="reports window visibility"), hand_eye_preview():
        pytest.fail("Capture started without a usable review window")

    closed.assert_called_once()
