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

from collections.abc import Callable, Iterator
from contextlib import contextmanager, suppress
import os
import sys
import threading
from typing import TYPE_CHECKING, Literal, TypeVar

import cv2
import numpy as np

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.Image import Image

ReviewChoice = Literal["accept", "retry", "quit"]
RejectedChoice = Literal["retry", "halfway", "quit"]
_Choice = TypeVar("_Choice", bound=str)
_WINDOW = "DimOS hand-eye calibration"
_QUIT_KEYS = (27, ord("q"), ord("Q"))


def _window_visibility() -> float:
    try:
        return cv2.getWindowProperty(_WINDOW, cv2.WND_PROP_VISIBLE)
    except cv2.error:
        # Some backends discard the named window immediately when the operator closes it.
        return -1.0


def _prompt(
    image: Image, label: str, controls: str, actions: dict[int, _Choice]
) -> _Choice | Literal["quit"]:
    # Cocoa processes one event per 10 ms. Allow time for mouse and keyboard
    # events queued during motion before showing the next image.
    while (key := cv2.waitKey(120)) != -1:
        if key & 0xFF in _QUIT_KEYS:
            return "quit"
    if _window_visibility() < 1:
        return "quit"

    preview = cv2.copyMakeBorder(image.to_opencv(), 0, 32, 0, 0, cv2.BORDER_CONSTANT)
    center = (image.width // 2, image.height // 2)
    cv2.drawMarker(preview, center, (0, 0, 0), cv2.MARKER_CROSS, 15, 3)
    cv2.drawMarker(preview, center, (255, 255, 255), cv2.MARKER_CROSS, 15, 1)
    cv2.putText(
        preview,
        controls,
        (8, image.height + 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.imshow(_WINDOW, preview)
    cv2.setWindowTitle(_WINDOW, label)
    while True:
        key = cv2.waitKey(50) & 0xFF
        if _window_visibility() < 1 or key in _QUIT_KEYS:
            return "quit"
        if key in actions:
            break

    # The main thread blocks while the arm moves and waits for a stationary image.
    # Paint the waiting state before handing control back to the runner.
    waiting = np.zeros_like(preview)
    choice = actions[key]
    status = {"halfway": "Moving halfway toward home", "retry": "Retaking at this pose"}.get(
        choice, "Waiting for next capture"
    )
    cv2.putText(
        waiting, status, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA
    )
    cv2.imshow(_WINDOW, waiting)
    cv2.setWindowTitle(_WINDOW, status)
    if cv2.waitKey(20) & 0xFF in _QUIT_KEYS or _window_visibility() < 1:
        return "quit"
    return choice


def _review(image: Image, label: str) -> ReviewChoice:
    accept_key = "Return" if sys.platform == "darwin" else "Enter"
    controls = f"{accept_key}: accept   R: retake   Q/Esc: quit"
    actions: dict[int, ReviewChoice] = {
        10: "accept",
        13: "accept",
        ord("r"): "retry",
        ord("R"): "retry",
    }
    return _prompt(image, label, controls, actions)


def review_rejected(image: Image, label: str, can_move_halfway: bool) -> RejectedChoice:
    """Review a rejected capture without letting Enter accept it or move the arm."""
    controls = "R: retake here"
    actions: dict[int, RejectedChoice] = {ord("r"): "retry", ord("R"): "retry"}
    if can_move_halfway:
        controls += "   H: move halfway toward home"
        actions.update({ord("h"): "halfway", ord("H"): "halfway"})
    return _prompt(image, f"Rejected - {label}", controls + "   Q/Esc: quit", actions)


@contextmanager
def hand_eye_preview() -> Iterator[Callable[[Image, str], ReviewChoice]]:
    """Open the CLI review window before motion and close it when capture ends."""
    if threading.current_thread() is not threading.main_thread():
        raise RuntimeError("Open the hand-eye preview on the main thread.")
    if sys.platform.startswith("linux") and not (
        os.getenv("DISPLAY") or os.getenv("WAYLAND_DISPLAY")
    ):
        raise RuntimeError("Hand-eye image review requires a graphical display.")
    try:
        cv2.namedWindow(_WINDOW, cv2.WINDOW_AUTOSIZE)
    except cv2.error as exc:
        raise RuntimeError("Could not open the hand-eye image review window.") from exc
    try:
        # Some HighGUI backends cannot report when the operator closes the window.
        if _window_visibility() < 0:
            raise RuntimeError(
                "Hand-eye review needs an OpenCV backend that reports window visibility."
            )
        yield _review
    finally:
        # Clicking the system close button may already have destroyed the window.
        with suppress(cv2.error):
            cv2.destroyWindow(_WINDOW)
            cv2.waitKey(1)
