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

"""Camera mux compositing — even-dimension guard for the H.264 encoder.

Regression cover for the camera-switch crash: an odd composite width/height
made aiortc's libx264 reopen fail with avcodec_open2 on the next selection
change. _composite must always return even w x h.
"""

from __future__ import annotations

import json
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.teleop.hosted.camera_mux import CameraMuxModule


class _Mux(CameraMuxModule):
    """Bare host: just the config the module reads + a captured publish.

    Bypasses ``Module.__init__`` (no ports/RPC plumbing) — build the mux state
    directly via ``_mux_init`` like the module does after ``super().__init__``.
    """

    def __init__(self, cameras: list[str], **cfg: object) -> None:
        self.config = SimpleNamespace(
            video_max_width=cfg.get("video_max_width", 0),
            video_max_fps=cfg.get("video_max_fps", 0.0),
            latency_stamp=cfg.get("latency_stamp", False),
        )
        self.published: list[Image] = []
        self.mux_image = SimpleNamespace(publish=self.published.append)
        self._mux_init(cameras)


def _make(cameras: list[str], **cfg: object) -> _Mux:
    """Construct without triggering CameraMuxModule.__init__ / Module.__init__."""
    return _Mux(cameras, **cfg)


def _img(w: int, h: int) -> Image:
    return Image(data=np.zeros((h, w, 3), dtype=np.uint8), format=ImageFormat.BGR)


def _feed(mux: _Mux, cam: str, img: Image) -> None:
    with mux._cam_lock:
        mux._cam_frames[cam] = img


def _is_even(img: Image) -> bool:
    h, w = img.data.shape[:2]
    return h % 2 == 0 and w % 2 == 0


# ─── _even_dims unit ──────────────────────────────────────────────────


def test_even_dims_crops_odd_width_and_height() -> None:
    out = CameraMuxModule._even_dims(_img(641, 481))
    assert out.data.shape[:2] == (480, 640)
    assert out.data.flags["C_CONTIGUOUS"]  # from_ndarray needs contiguous


def test_even_dims_passes_even_through_unchanged() -> None:
    src = _img(640, 480)
    assert CameraMuxModule._even_dims(src) is src  # no copy when already even


# ─── _composite always even (the actual crash path) ───────────────────


def test_single_camera_downscale_to_odd_is_evened() -> None:
    # 1280→641 cap yields an odd width, and 641*720/1280 = 360 (even h) —
    # width alone would crash the encoder; the guard fixes it.
    mux = _make(["cam1"], video_max_width=641)
    _feed(mux, "cam1", _img(1280, 720))
    out = mux._composite()
    assert out is not None and _is_even(out)


def test_hstack_odd_tile_width_is_evened() -> None:
    # Two cams of different aspect → per-tile int() scaling can sum to an odd
    # composite width. Guard must even it regardless of the arithmetic.
    mux = _make(["cam1", "cam2"])
    with mux._cam_lock:
        mux._cam_selected = ["cam1", "cam2"]
    _feed(mux, "cam1", _img(853, 480))  # 16:9-ish, odd width
    _feed(mux, "cam2", _img(641, 481))  # deliberately odd both ways
    out = mux._composite()
    assert out is not None and _is_even(out)


def test_hstack_degenerate_tile_width_does_not_crash() -> None:
    # A very short reference tile (sets target_h) beside a tall-narrow one makes
    # int(w * target_h / h) round to 0; cv2.resize raises on a 0 dimension.
    # target_h = min(4, 1000) = 4; cam2 tile width = int(100*4/1000) = 0 → must
    # be clamped to 1, not crash.
    mux = _make(["cam1", "cam2"])
    with mux._cam_lock:
        mux._cam_selected = ["cam1", "cam2"]
    _feed(mux, "cam1", _img(200, 4))  # short → target_h = 4
    _feed(mux, "cam2", _img(100, 1000))  # tall-narrow → scaled width rounds to 0
    out = mux._composite()  # must not raise
    assert out is not None and _is_even(out)


def test_composite_returns_none_on_error_does_not_raise() -> None:
    # Mismatched channel counts make np.hstack raise; _composite must return
    # None (drop the frame), not kill its RxPY subscription.
    mux = _make(["cam1", "cam2"])
    with mux._cam_lock:
        mux._cam_selected = ["cam1", "cam2"]
    _feed(mux, "cam1", Image(data=np.zeros((480, 640, 3), np.uint8), format=ImageFormat.BGR))
    _feed(mux, "cam2", Image(data=np.zeros((480, 640, 4), np.uint8), format=ImageFormat.BGRA))
    assert mux._composite() is None


def test_switch_between_selections_stays_even() -> None:
    # Reproduces the report: flipping selection changes frame size (encoder
    # reopen). Every produced frame must be even so libx264 never fails.
    mux = _make(["cam1", "cam2"], video_max_width=641, latency_stamp=True)
    _feed(mux, "cam1", _img(1280, 721))
    _feed(mux, "cam2", _img(647, 483))
    for selection in (["cam1", "cam2"], ["cam1"], ["cam2"], ["cam1", "cam2"]):
        with mux._cam_lock:
            mux._cam_selected = list(selection)
        out = mux._composite()
        assert out is not None and _is_even(out), f"odd dims for {selection}"


@pytest.mark.parametrize("cams", [None, "cam1", 5, {"cam1": 1}])
def test_set_cam_selection_ignores_non_list(cams: object) -> None:
    # Untrusted wire payload (e.g. cams:null) must not raise into the state
    # handler; a bad value falls back to the first camera.
    mux = _make(["cam1", "cam2"])
    payload = json.dumps({"type": "camera_select", "cams": cams}).encode()
    mux._set_cam_selection(payload)
    assert mux._cam_selected == ["cam1"]


def test_set_cam_selection_switches_camera() -> None:
    # The happy path: a well-formed camera_select flips the active camera.
    mux = _make(["cam1", "cam2"])
    mux._set_cam_selection(json.dumps({"type": "camera_select", "cams": ["cam2"]}).encode())
    assert mux._cam_selected == ["cam2"]


@pytest.mark.parametrize(
    "payload",
    [
        json.dumps({"type": "estop", "nonce": "abc"}).encode(),  # other state kind
        json.dumps({"type": "sport_cmd", "name": "Sit"}).encode(),
        b"\x00\x01\x02",  # non-JSON binary frame (e.g. a Twist on the plane)
        b'{"malformed',  # truncated JSON
    ],
)
def test_set_cam_selection_ignores_foreign_frames(payload: bytes) -> None:
    # camera_select shares the state_reliable plane with estop/sport/etc.; a
    # frame that isn't ours must leave the selection untouched (no exception,
    # no spurious switch, no log spam).
    mux = _make(["cam1", "cam2"])
    with mux._cam_lock:
        mux._cam_selected = ["cam2"]
    mux._set_cam_selection(payload)
    assert mux._cam_selected == ["cam2"]  # unchanged
