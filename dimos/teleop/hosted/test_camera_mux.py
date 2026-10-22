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

import numpy as np
import pytest

from dimos.message_utils.image import image_from_array, image_to_rgb, image_view
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.sensor_msgs.msg import Image
from dimos.messages.std_msgs.msg import Header
from dimos.teleop.hosted.blueprints.cloudflare import (
    teleop_hosted_go2_multicam,
    teleop_hosted_go2_transport,
    teleop_hosted_xarm6,
    teleop_hosted_xarm7,
)
from dimos.teleop.hosted.camera_mux import CameraMuxModule


@pytest.fixture
def make_mux(mocker):
    instances = []

    def make(cameras, **config):
        module = CameraMuxModule(cameras=cameras, **config)
        mocker.patch.object(module.mux_image, "publish")
        instances.append(module)
        return module

    yield make
    for module in instances:
        module.stop()


def _img(w: int, h: int) -> Image:
    return image_from_array(np.zeros((h, w, 3), dtype=np.uint8), encoding="bgr8")


def _feed(mux: CameraMuxModule, cam: str, img: Image) -> None:
    with mux._cam_lock:
        mux._cam_frames[cam] = img


def _is_even(img: Image) -> bool:
    h, w = image_view(img).shape[:2]
    return h % 2 == 0 and w % 2 == 0


# ─── _even_dims unit ──────────────────────────────────────────────────


def test_even_dims_crops_odd_width_and_height() -> None:
    out = CameraMuxModule._even_dims(_img(641, 481))
    assert image_view(out).shape[:2] == (480, 640)
    assert image_view(out).flags["C_CONTIGUOUS"]  # from_ndarray needs contiguous


def test_even_dims_passes_even_through_unchanged() -> None:
    src = _img(640, 480)
    assert CameraMuxModule._even_dims(src) is src  # no copy when already even


# ─── _composite always even (the actual crash path) ───────────────────


def test_single_camera_downscale_to_odd_is_evened(
    make_mux,
) -> None:
    # 1280→641 cap yields an odd width, and 641*720/1280 = 360 (even h) —
    # width alone would crash the encoder; the guard fixes it.
    mux = make_mux(["cam1"], video_max_width=641)
    _feed(mux, "cam1", _img(1280, 720))
    out = mux._composite()
    assert out is not None and _is_even(out)


def test_hstack_odd_tile_width_is_evened(
    make_mux,
) -> None:
    # Two cams of different aspect → per-tile int() scaling can sum to an odd
    # composite width. Guard must even it regardless of the arithmetic.
    mux = make_mux(["cam1", "cam2"])
    with mux._cam_lock:
        mux._cam_selected = ["cam1", "cam2"]
    _feed(mux, "cam1", _img(853, 480))  # 16:9-ish, odd width
    _feed(mux, "cam2", _img(641, 481))  # deliberately odd both ways
    out = mux._composite()
    assert out is not None and _is_even(out)


def test_hstack_degenerate_tile_width_does_not_crash(
    make_mux,
) -> None:
    # A very short reference tile (sets target_h) beside a tall-narrow one makes
    # int(w * target_h / h) round to 0; cv2.resize raises on a 0 dimension.
    # target_h = min(4, 1000) = 4; cam2 tile width = int(100*4/1000) = 0 → must
    # be clamped to 1, not crash.
    mux = make_mux(["cam1", "cam2"])
    with mux._cam_lock:
        mux._cam_selected = ["cam1", "cam2"]
    _feed(mux, "cam1", _img(200, 4))  # short → target_h = 4
    _feed(mux, "cam2", _img(100, 1000))  # tall-narrow → scaled width rounds to 0
    out = mux._composite()  # must not raise
    assert out is not None and _is_even(out)


def test_composite_returns_none_on_error_does_not_raise(
    make_mux,
) -> None:
    # Mismatched channel counts make np.hstack raise; _composite must return
    # None (drop the frame), not kill its RxPY subscription.
    mux = make_mux(["cam1", "cam2"])
    with mux._cam_lock:
        mux._cam_selected = ["cam1", "cam2"]
    _feed(mux, "cam1", image_from_array(np.zeros((480, 640, 3), np.uint8), encoding="bgr8"))
    _feed(mux, "cam2", image_from_array(np.zeros((480, 640, 4), np.uint8), encoding="bgra8"))
    assert mux._composite() is None


def test_switch_between_selections_stays_even(
    make_mux,
) -> None:
    # Reproduces the report: flipping selection changes frame size (encoder
    # reopen). Every produced frame must be even so libx264 never fails.
    mux = make_mux(["cam1", "cam2"], video_max_width=641, latency_stamp=True)
    _feed(mux, "cam1", _img(1280, 721))
    _feed(mux, "cam2", _img(647, 483))
    for selection in (["cam1", "cam2"], ["cam1"], ["cam2"], ["cam1", "cam2"]):
        with mux._cam_lock:
            mux._cam_selected = list(selection)
        out = mux._composite()
        assert out is not None and _is_even(out), f"odd dims for {selection}"


@pytest.mark.parametrize("cams", [None, "cam1", 5, {"cam1": 1}])
def test_set_cam_selection_ignores_non_list(make_mux, cams: object) -> None:
    # Untrusted wire payload (e.g. cams:null) must not raise into the state
    # handler; a bad value falls back to the first camera.
    mux = make_mux(["cam1", "cam2"])
    payload = json.dumps({"type": "camera_select", "cams": cams}).encode()
    mux._set_cam_selection(payload)
    assert mux._cam_selected == ["cam1"]


def test_set_cam_selection_switches_camera(
    make_mux,
) -> None:
    # The happy path: a well-formed camera_select flips the active camera.
    mux = make_mux(["cam1", "cam2"])
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
def test_set_cam_selection_ignores_foreign_frames(make_mux, payload: bytes) -> None:
    # camera_select shares the state_reliable plane with estop/sport/etc.; a
    # frame that isn't ours must leave the selection untouched (no exception,
    # no spurious switch, no log spam).
    mux = make_mux(["cam1", "cam2"])
    with mux._cam_lock:
        mux._cam_selected = ["cam2"]
    mux._set_cam_selection(payload)
    assert mux._cam_selected == ["cam2"]  # unchanged


def test_latency_stamp_fits_640_width(
    make_mux,
) -> None:
    mux = make_mux(["cam1"], latency_stamp=True)
    _feed(mux, "cam1", _img(640, 480))
    out = mux._composite()
    assert out is not None and image_view(out).shape[:2] == (480 + 16, 640)


def test_fps_cap_drops_frames_within_window(make_mux, mocker) -> None:
    mocker.patch("dimos.teleop.hosted.camera_mux.time.monotonic", return_value=100.0)
    mux = make_mux(["cam1"], video_max_fps=30.0)
    mux._on_cam("cam1", _img(640, 480))
    mux._on_cam("cam1", _img(640, 480))  # immediately again — inside 1/30 s
    assert mux.mux_image.publish.call_count == 1


def test_fps_cap_window_released_on_failed_composite(
    make_mux,
) -> None:
    mux = make_mux(["cam1", "cam2"], video_max_fps=30.0)
    with mux._cam_lock:
        mux._cam_selected = ["cam1", "cam2"]
    _feed(mux, "cam2", image_from_array(np.zeros((480, 640, 4), np.uint8), encoding="bgra8"))
    mux._on_cam("cam1", _img(640, 480))  # BGR + BGRA hstack fails → None
    mux.mux_image.publish.assert_not_called()
    _feed(mux, "cam2", _img(640, 480))
    mux._on_cam("cam1", _img(640, 480))  # still inside the window — must publish
    assert mux.mux_image.publish.call_count == 1


def test_generated_header_survives_crop_resize_and_latency_strip(make_mux, mocker):
    mocker.patch("dimos.teleop.hosted.camera_mux.time.time", return_value=17.125)
    source = image_from_array(
        np.full((481, 853, 3), [255, 0, 0], np.uint8),
        encoding="rgb8",
        header=Header(Time(1_789_654_321, 1), "camera"),
    )
    original = source.data.copy()
    mux = make_mux(["cam1"], video_max_width=641, latency_stamp=True)
    _feed(mux, "cam1", source)
    out = mux._composite()
    assert out is not None
    assert out.header == source.header
    assert out.header is not source.header
    assert out.width == 640
    assert out.height % 2 == 0
    assert image_to_rgb(out)[0, 0].tolist() == [255, 0, 0]
    np.testing.assert_array_equal(source.data, original)
    out.header.frame_id = "changed"
    assert source.header.frame_id == "camera"


def test_mixed_rgb_bgr_tiles_keep_the_same_visible_color(make_mux):
    mux = make_mux(["cam1", "cam2"])
    with mux._cam_lock:
        mux._cam_selected = ["cam1", "cam2"]
    _feed(mux, "cam1", image_from_array(np.full((4, 4, 3), [0, 0, 255], np.uint8), encoding="bgr8"))
    _feed(mux, "cam2", image_from_array(np.full((4, 4, 3), [255, 0, 0], np.uint8), encoding="rgb8"))
    out = mux._composite()
    assert out is not None
    assert out.encoding == "bgr8"
    assert out.header.frame_id == "camera_mux"
    assert image_to_rgb(out)[0, 0].tolist() == [255, 0, 0]
    assert image_to_rgb(out)[0, 6].tolist() == [255, 0, 0]


def test_hosted_blueprints_bind_canonical_mux_image_to_video_transport():
    for blueprint in (
        teleop_hosted_go2_transport,
        teleop_hosted_go2_multicam,
        teleop_hosted_xarm6,
        teleop_hosted_xarm7,
    ):
        assert ("mux_image", Image) in blueprint.transport_map
