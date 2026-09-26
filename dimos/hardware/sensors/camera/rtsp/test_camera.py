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

"""RtspCamera against a synthetic H.265 clip. No camera, no network."""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
import socket
import threading
import time
from types import SimpleNamespace
from typing import Any

import av
import numpy as np
from pydantic import ValidationError
import pytest

from dimos.hardware.sensors.camera.rtsp import camera as camera_module
from dimos.hardware.sensors.camera.rtsp.camera import RtspCamera
from dimos.hardware.sensors.camera.rtsp.synthetic import SQUARE, square_origin, write_synthetic_h265

pytestmark = pytest.mark.skipif_no_turbojpeg


@pytest.fixture(scope="module")
def clip(tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, int]:
    path = tmp_path_factory.mktemp("rtsp") / "synthetic.mp4"
    n = write_synthetic_h265(path, width=320, height=180, fps=25, seconds=2.0)
    return path, n


@pytest.fixture
def camera(clip: tuple[Path, int]) -> Iterator[tuple[RtspCamera, dict[str, list[Any]]]]:
    cam = RtspCamera(
        url=str(clip[0]),
        replay_realtime=False,
        replay_loop=False,
        color_hz=1e9,
        jpeg_hz=1e9,
        capture_latency_s=0.08,
        sensor_stats_interval_s=0.0,
    )
    published: dict[str, list[Any]] = {"video": [], "color_image": [], "color_jpeg": []}
    for name, sink in published.items():
        getattr(cam, name).publish = sink.append
    yield cam, published
    cam.stop()


def test_url_is_required() -> None:
    with pytest.raises(ValidationError, match="url"):
        RtspCamera()


def test_ports(camera: tuple[RtspCamera, dict[str, list[Any]]]) -> None:
    cam, _ = camera
    assert set(cam.inputs) == set()
    assert set(cam.outputs) == {"video", "color_image", "color_jpeg"}
    assert {"set_video_enabled", "set_jpeg_rate", "sensor_stats"} <= set(cam.rpcs)


def test_every_access_unit_is_passed_through(
    camera: tuple[RtspCamera, dict[str, list[Any]]], clip: tuple[Path, int]
) -> None:
    cam, published = camera
    t0 = time.time()
    packets = cam.relay_once()
    assert packets == clip[1]
    assert len(published["video"]) == clip[1]
    first = published["video"][0]
    assert first.format == "h265" and first.frame_id == "camera_optical"
    assert first.data.size > 0
    # Stamped at the read minus the configured capture latency, on the wall clock.
    assert t0 - 0.08 - 0.5 <= first.ts <= time.time() - 0.08
    stamps = [v.ts for v in published["video"]]
    assert stamps == sorted(stamps)


def test_decoded_frames_carry_the_moving_square(
    camera: tuple[RtspCamera, dict[str, list[Any]]], clip: tuple[Path, int]
) -> None:
    cam, published = camera
    cam.relay_once()
    frames = published["color_image"]
    assert len(frames) == clip[1]
    img = frames[10]
    assert img.data.shape == (180, 320, 3)
    x, y = square_origin(10, 320, 180)
    assert np.mean(img.data[y : y + SQUARE, x : x + SQUARE]) > 200
    assert np.mean(img.data[:20, :20]) < 130
    # No B-frames, as from a live camera: every frame carries its own packet's stamp, in order.
    assert [f.ts for f in frames] == [v.ts for v in published["video"]]


def test_jpeg_is_small_and_capped(clip: tuple[Path, int]) -> None:
    cam = RtspCamera(
        url=str(clip[0]), replay_realtime=False, replay_loop=False, color_hz=0.0, jpeg_hz=1e9
    )
    jpegs: list[Any] = []
    cam.video.publish = lambda _m: None
    cam.color_image.publish = lambda _m: None
    cam.color_jpeg.publish = jpegs.append
    try:
        cam.relay_once()
    finally:
        cam.stop()
    assert len(jpegs) == clip[1]
    assert jpegs[0].format == "jpeg" and 0 < len(jpegs[0].data) < 20_000
    assert cam.sensor_stats()["color_image"]["published"] == 0


def test_missing_libturbojpeg_only_stops_the_jpeg(
    camera: tuple[RtspCamera, dict[str, list[Any]]],
    clip: tuple[Path, int],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def no_library() -> Any:
        raise RuntimeError("Unable to locate turbojpeg library automatically.")

    monkeypatch.setattr("dimos.msgs.sensor_msgs.Image.get_turbojpeg", no_library)
    cam, published = camera
    cam.relay_once()
    assert len(published["video"]) == clip[1]
    assert len(published["color_image"]) == clip[1]
    assert published["color_jpeg"] == []
    assert cam.sensor_stats()["color_jpeg"]["errors"] == 1  # switched off, not retried per frame
    assert cam._jpeg_hz == 0.0


def _nal_types(unit: bytes) -> list[int]:
    """HEVC NAL unit types found after each Annex-B start code."""
    types, i = [], unit.find(b"\x00\x00\x01")
    while i >= 0:
        types.append((unit[i + 3] >> 1) & 0x3F)
        i = unit.find(b"\x00\x00\x01", i + 3)
    return types


def test_replayed_video_is_annexb_with_parameter_sets(
    camera: tuple[RtspCamera, dict[str, list[Any]]],
) -> None:
    # An mp4 stores length-prefixed units with the parameter sets out of band; a viewer
    # needs what RTSP delivers: start codes, and VPS/SPS/PPS ahead of the first IDR.
    cam, published = camera
    cam.relay_once()
    units = [v.data.tobytes() for v in published["video"]]
    assert all(u.startswith(b"\x00\x00\x00\x01") for u in units)
    assert _nal_types(units[0])[:3] == [32, 33, 34]  # VPS, SPS, PPS
    assert {19, 20} & set(_nal_types(units[0]))  # an IDR follows the parameter sets


def test_live_path_forwards_the_units_untouched(tmp_path: Path) -> None:
    # "://" selects the live branch: no bitstream filter, the demuxed units byte for byte.
    path = tmp_path / "clip.ts"  # MPEG-TS carries Annex-B, as RTSP does
    n = write_synthetic_h265(path, seconds=1.0)
    with av.open(str(path)) as container:
        expected = [bytes(p) for p in container.demux(video=0) if p.size]
    cam = RtspCamera(url=f"file://{path}", color_hz=1e9, jpeg_hz=0.0, sensor_stats_interval_s=0.0)
    video: list[Any] = []
    images: list[Any] = []
    cam.video.publish = video.append
    cam.color_image.publish = images.append
    try:
        assert cam.relay_once() == n
    finally:
        cam.stop()
    assert [v.data.tobytes() for v in video] == expected
    assert len(images) == n


class _DamagedUnitStream:
    """The real stream, except that one access unit fails to decode."""

    def __init__(self, real: Any, bad_index: int) -> None:
        self._real, self._bad, self._seen = real, bad_index, 0

    def __getattr__(self, name: str) -> Any:
        return getattr(self._real, name)

    def decode(self, packet: Any = None) -> Any:
        self._seen += 1
        if self._seen == self._bad:
            raise av.InvalidDataError(1, "Invalid data found when processing input")
        return self._real.decode(packet)


class _DamagedUnitContainer:
    def __init__(self, real: Any, bad_index: int) -> None:
        self._real = real
        self.streams = SimpleNamespace(video=[_DamagedUnitStream(real.streams.video[0], bad_index)])

    def __enter__(self) -> _DamagedUnitContainer:
        return self

    def __exit__(self, *_exc: Any) -> None:
        self._real.close()

    def demux(self, _stream: Any) -> Any:
        return self._real.demux(self._real.streams.video[0])


def test_a_damaged_access_unit_does_not_end_the_passthrough(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / "clip.ts"
    n = write_synthetic_h265(path, seconds=1.0)
    real_open = av.open
    monkeypatch.setattr(
        camera_module.av, "open", lambda *a, **kw: _DamagedUnitContainer(real_open(*a, **kw), 5)
    )
    cam = RtspCamera(url=f"file://{path}", color_hz=1e9, jpeg_hz=0.0, sensor_stats_interval_s=0.0)
    video: list[Any] = []
    images: list[Any] = []
    cam.video.publish = video.append
    cam.color_image.publish = images.append
    try:
        assert cam.relay_once() == n
    finally:
        cam.stop()
    assert len(video) == n
    assert 0 < len(images) < n
    assert cam.sensor_stats()["color_image"]["errors"] == 1


def test_a_live_stream_that_ends_is_reopened_after_the_wait(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    cam = RtspCamera(url="rtsp://unused/main", sensor_stats_interval_s=0.0)
    opens: list[float] = []
    monkeypatch.setattr(cam, "relay_once", lambda: opens.append(time.monotonic()) or 0)
    monkeypatch.setattr(camera_module, "_RECONNECT_WAIT_S", 0.2)
    relay = threading.Thread(target=cam._relay_loop)
    relay.start()
    time.sleep(0.5)
    cam._stop_event.set()
    relay.join(timeout=2.0)
    cam.stop()
    assert not relay.is_alive()
    assert 2 <= len(opens) <= 4  # not a busy loop
    assert opens[1] - opens[0] >= 0.2


def test_synthetic_source_starts_and_stop_removes_the_clip() -> None:
    cam = RtspCamera(url="synthetic", color_hz=1e9, jpeg_hz=1e9, sensor_stats_interval_s=0.0)
    published: dict[str, list[Any]] = {"video": [], "color_image": [], "color_jpeg": []}
    for name, sink in published.items():
        getattr(cam, name).publish = sink.append
    try:
        cam.start()
        clip_dir = Path(cam._source).parent
        assert clip_dir.name.startswith("rtsp-synthetic-") and clip_dir.is_dir()
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not all(published.values()):
            time.sleep(0.02)
    finally:
        cam.stop()
    assert all(published.values())
    x, y = square_origin(0, 320, 180, centered=True)
    assert np.mean(published["color_image"][0].data[y : y + SQUARE, x : x + SQUARE]) > 200
    assert not clip_dir.exists()
    assert not [t for t in threading.enumerate() if t.name.startswith("rtsp-")]


_SDP = (
    "v=0\r\no=- 0 0 IN IP4 127.0.0.1\r\ns=x\r\nt=0 0\r\n"
    "m=video 0 RTP/AVP 96\r\na=rtpmap:96 H265/90000\r\na=control:track1\r\n"
)


def _handshake_then_silence(server: socket.socket) -> None:
    """An RTSP server that answers every request and never sends a packet of video."""
    conn, _ = server.accept()
    buf = b""
    while data := conn.recv(4096):
        buf += data
        while b"\r\n\r\n" in buf:
            req, buf = buf.split(b"\r\n\r\n", 1)
            lines = req.decode().split("\r\n")
            cseq = next(
                ln.split(":", 1)[1].strip() for ln in lines if ln.lower().startswith("cseq")
            )
            head = f"RTSP/1.0 200 OK\r\nCSeq: {cseq}\r\nSession: 1\r\n"
            if lines[0].startswith("DESCRIBE"):
                head += (
                    f"Content-Type: application/sdp\r\nContent-Length: {len(_SDP)}\r\n\r\n{_SDP}"
                )
            elif lines[0].startswith("SETUP"):
                head += "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n\r\n"
            else:
                head += "Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN\r\n\r\n"
            conn.sendall(head.encode())


@pytest.mark.parametrize("handshake", [False, True])
def test_stop_outlasts_a_source_that_never_answers(
    monkeypatch: pytest.MonkeyPatch, handshake: bool
) -> None:
    # The relay thread sits in av.open for the whole av timeout, twice when the camera
    # answers the handshake and then sends nothing; stop() must not return (and let a
    # restart clear the stop flag) while it is still in there.
    monkeypatch.setattr(camera_module, "_AV_TIMEOUT_S", 1.0)
    monkeypatch.setattr(camera_module, "DEFAULT_THREAD_JOIN_TIMEOUT", 0.5)
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    if handshake:
        threading.Thread(target=_handshake_then_silence, args=(server,), daemon=True).start()
    cam = RtspCamera(
        url=f"rtsp://127.0.0.1:{server.getsockname()[1]}/main", sensor_stats_interval_s=0.0
    )
    try:
        cam.start()
        time.sleep(0.1)
        cam.stop()
        assert not [t for t in threading.enumerate() if t.name == "rtsp-relay"]
    finally:
        server.close()


def test_rate_gate_delivers_the_configured_rate(monkeypatch: pytest.MonkeyPatch) -> None:
    # 25 fps with jitter against a 25 Hz cap: a gate that restarts its period at each publish
    # loses every frame that arrives a hair early. A faster source is still cut to 25 Hz.
    clock = [1000.0]
    monkeypatch.setattr(
        camera_module, "time", SimpleNamespace(monotonic=lambda: clock[0], time=time.time)
    )
    frame = av.VideoFrame.from_ndarray(np.zeros((18, 32, 3), np.uint8), format="rgb24")
    counts = []
    for fps in (25, 90):
        cam = RtspCamera(url="unused", color_hz=25.0, jpeg_hz=0.0)
        images: list[Any] = []
        cam.color_image.publish = images.append
        try:
            for i in range(2 * fps):
                clock[0] = 1000.0 + i / fps + (0.003 if i % 2 else 0.0)
                cam._publish_decoded(frame, ts=0.0)
        finally:
            cam.stop()
        counts.append(len(images))
    assert counts[0] == 50
    assert 50 <= counts[1] <= 51


def test_video_can_be_disabled_and_jpeg_paced_by_rpc(
    camera: tuple[RtspCamera, dict[str, list[Any]]], clip: tuple[Path, int]
) -> None:
    cam, published = camera
    assert cam.set_video_enabled(False) is False
    assert cam.set_jpeg_rate(0.5) == 0.5
    cam.relay_once()
    assert published["video"] == []
    assert cam.sensor_stats()["video"]["dropped"] >= clip[1]
    assert len(published["color_image"]) == clip[1]  # decoded frames are unaffected
    assert len(published["color_jpeg"]) == 1  # 0.5 Hz over a clip replayed as fast as possible

    assert cam.set_video_enabled(True) is True
    assert cam.set_jpeg_rate(0) == 0.0
    cam.relay_once()
    assert len(published["video"]) == clip[1]
    assert len(published["color_jpeg"]) == 1  # rate 0: no more JPEGs
    assert cam.set_jpeg_rate(-3) == 0.0
