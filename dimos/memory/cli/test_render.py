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

from pathlib import Path
from types import SimpleNamespace

import numpy as np

from dimos.memory.cli.render import _frame_first_seen, _pair_camera_infos, render_store
from dimos.memory.store.memory import MemoryStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


def _entry(name: str, data: object) -> tuple:  # type: ignore[type-arg]
    return (name, SimpleNamespace(), SimpleNamespace(data=data))


def _info(frame_id: str) -> CameraInfo:
    return CameraInfo.from_intrinsics(
        fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480, frame_id=frame_id
    )


def _image(frame_id: str) -> Image:
    return Image(data=np.zeros((4, 4, 3), dtype=np.uint8), frame_id=frame_id)


def test_pairs_by_frame_id() -> None:
    renderable = [
        _entry("camera_info", _info("camera_optical")),
        _entry("color_image", _image("camera_optical")),
        _entry("lidar", SimpleNamespace()),
    ]
    pinholes, paired = _pair_camera_infos(renderable)
    assert set(pinholes) == {"color_image"}
    info, frame = pinholes["color_image"]
    assert info.frame_id == "camera_optical"
    assert frame == "camera_optical"
    assert paired == {"camera_info"}


def test_falls_back_to_single_pair_on_frame_mismatch() -> None:
    renderable = [
        _entry("camera_info", _info("camera_optical")),
        _entry("color_image", _image("other_frame")),
    ]
    pinholes, paired = _pair_camera_infos(renderable)
    assert set(pinholes) == {"color_image"}
    _, frame = pinholes["color_image"]
    assert frame == "other_frame"
    assert paired == {"camera_info"}


def test_unmatched_info_stays_unpaired() -> None:
    renderable = [
        _entry("front_info", _info("front_optical")),
        _entry("rear_info", _info("rear_optical")),
        _entry("front_image", _image("front_optical")),
    ]
    pinholes, paired = _pair_camera_infos(renderable)
    assert set(pinholes) == {"front_image"}
    assert paired == {"front_info"}


def test_one_info_many_images_matches_all_same_frame() -> None:
    renderable = [
        _entry("camera_info", _info("camera_optical")),
        _entry("color_image", _image("camera_optical")),
        _entry("depth_image", _image("camera_optical")),
    ]
    pinholes, paired = _pair_camera_infos(renderable)
    assert set(pinholes) == {"color_image", "depth_image"}
    assert paired == {"camera_info"}


def test_no_infos() -> None:
    renderable = [_entry("color_image", _image("camera_optical"))]
    pinholes, paired = _pair_camera_infos(renderable)
    assert pinholes == {}
    assert paired == set()


# --- render_store end to end -------------------------------------------------


def _tf(child: str, ts: float) -> TFMessage:
    return TFMessage(
        Transform(
            translation=Vector3(1.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="world",
            child_frame_id=child,
            ts=ts,
        )
    )


def _written(rrd: str) -> bool:
    return Path(rrd).is_file() and Path(rrd).stat().st_size > 0


def test_render_store_logs_pinhole_on_image_entity(tmp_path, capsys) -> None:  # type: ignore[no-untyped-def]
    with MemoryStore() as store:
        store.stream("camera_info", CameraInfo).append(_info("camera_optical"), ts=90.0)
        images = store.stream("color_image", Image)
        images.append(_image("camera_optical"), ts=100.0)
        images.append(_image("camera_optical"), ts=100.5)
        store.stream("tf", TFMessage).append(_tf("camera_optical", 101.0), ts=101.0)
        out = render_store(store, out=str(tmp_path / "a.rrd"), no_gui=True)

    text = capsys.readouterr().out
    # Logged at the frame's first tf (+1.0 s from t0=100), not at the stale info ts (90).
    assert "color_image: pinhole from camera_info (frame 'camera_optical' at +1.00s)" in text
    assert not any(ln.strip().startswith("camera_info") for ln in text.splitlines())
    assert _written(out)


def test_render_store_pinhole_waits_for_its_own_frame(tmp_path, capsys) -> None:  # type: ignore[no-untyped-def]
    with MemoryStore() as store:
        store.stream("camera_info", CameraInfo).append(_info("camera_optical"), ts=100.0)
        store.stream("color_image", Image).append(_image("camera_optical"), ts=100.0)
        tf = store.stream("tf", TFMessage)
        tf.append(_tf("mid360_link", 101.0), ts=101.0)  # unrelated static edge first
        tf.append(_tf("base_link", 102.0), ts=102.0)
        tf.append(_tf("camera_optical", 103.0), ts=103.0)
        render_store(store, out=str(tmp_path / "b.rrd"), no_gui=True)
    assert "frame 'camera_optical' at +3.00s" in capsys.readouterr().out


def test_render_store_static_pinhole_without_tf(tmp_path, capsys) -> None:  # type: ignore[no-untyped-def]
    with MemoryStore() as store:
        store.stream("camera_info", CameraInfo).append(_info("camera_optical"), ts=100.0)
        store.stream("color_image", Image).append(_image("camera_optical"), ts=100.0)
        out = render_store(store, out=str(tmp_path / "c.rrd"), no_gui=True)
    assert "static: no tf" in capsys.readouterr().out
    assert _written(out)


def test_render_store_skips_pinhole_when_frame_never_in_tf(tmp_path, capsys) -> None:  # type: ignore[no-untyped-def]
    with MemoryStore() as store:
        store.stream("camera_info", CameraInfo).append(_info("camera_optical"), ts=100.0)
        store.stream("color_image", Image).append(_image("camera_optical"), ts=100.0)
        store.stream("tf", TFMessage).append(_tf("base_link", 101.0), ts=101.0)
        render_store(store, out=str(tmp_path / "d.rrd"), no_gui=True)
    assert "never appears in tf" in capsys.readouterr().out


def test_render_store_skips_pinhole_outside_window(tmp_path, capsys) -> None:  # type: ignore[no-untyped-def]
    with MemoryStore() as store:
        store.stream("camera_info", CameraInfo).append(_info("camera_optical"), ts=100.0)
        store.stream("color_image", Image).append(_image("camera_optical"), ts=100.0)
        store.stream("tf", TFMessage).append(_tf("camera_optical", 120.0), ts=120.0)
        render_store(store, out=str(tmp_path / "e.rrd"), no_gui=True, seconds=5.0)
    assert "first appears after the window" in capsys.readouterr().out


def test_render_store_nothing_renderable(tmp_path, capsys) -> None:  # type: ignore[no-untyped-def]
    with MemoryStore() as store:
        store.stream("raw", bytes).append(b"x", ts=1.0)
        render_store(store, out=str(tmp_path / "f.rrd"), no_gui=True)
    assert "nothing renderable" in capsys.readouterr().out


def test_frame_first_seen_takes_first_ts_per_frame() -> None:
    with MemoryStore() as store:
        tf = store.stream("tf", TFMessage)
        tf.append(_tf("a", 5.0), ts=5.0)
        tf.append(_tf("b", 6.0), ts=6.0)
        tf.append(_tf("a", 7.0), ts=7.0)
        seen = _frame_first_seen([("tf", tf, tf.first())], {"a", "b"})
    assert seen == {"a": 5.0, "b": 6.0}
