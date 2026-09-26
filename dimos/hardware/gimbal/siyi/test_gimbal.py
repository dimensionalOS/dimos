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

"""SIYI A8 mini: frame maths, the SDK packet format, and the gimbal Module on a synthetic
attitude. No A8, no MAVLink, no transports."""

from __future__ import annotations

from collections.abc import Iterator
import math
import select
import socket
import threading
import time
from typing import Any

from pydantic import ValidationError
import pytest

from dimos.hardware.gimbal.siyi import gimbal as gimbal_module
from dimos.hardware.gimbal.siyi.frame import (
    BENCH_MOUNT,
    FLIGHT_MOUNT,
    GimbalDeviceFlags,
    decode_flags,
    normalize_attitude,
)
from dimos.hardware.gimbal.siyi.gimbal import SiyiA8Gimbal
from dimos.hardware.gimbal.siyi.replay import AttitudeSample
from dimos.hardware.gimbal.siyi.sdk import (
    CMD_ZOOM_MULTIPLE,
    SiyiSdk,
    build_packet,
    parse_packet,
    parse_zoom,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def _quat_wxyz(roll: float, pitch: float, yaw: float) -> list[float]:
    """ZYX euler (degrees) -> MAVLink [w, x, y, z]."""
    q = Quaternion.from_euler(Vector3(*(math.radians(a) for a in (roll, pitch, yaw))))
    return [q.w, q.x, q.y, q.z]


def test_mavlink_wxyz_quaternion_is_zyx_roll_pitch_yaw() -> None:
    # Pins what frame.py relies on: MAVLink sends [w, x, y, z], dimOS Quaternion is (x, y, z, w),
    # and quaternion_to_euler returns ZYX (roll, pitch, yaw).
    c, s = math.cos(math.radians(45.0)), math.sin(math.radians(45.0))
    assert normalize_attitude([c, s, 0.0, 0.0], FLIGHT_MOUNT) == pytest.approx((90.0, 0.0, 0.0))
    assert normalize_attitude([c, 0.0, 0.0, s], FLIGHT_MOUNT) == pytest.approx((0.0, 0.0, 90.0))
    # Rz(45) Ry(-30) Rx(10): yaw first, then pitch, then roll.
    q = [0.880371, 0.176447, -0.205991, 0.389078]
    assert normalize_attitude(q, FLIGHT_MOUNT) == pytest.approx((10.0, -30.0, 45.0), abs=1e-3)


@pytest.mark.parametrize("w", [0.0, math.nan, math.inf])
def test_zero_or_non_finite_quaternion_is_rejected(w: float) -> None:
    with pytest.raises(ValueError):
        normalize_attitude([w, 0.0, 0.0, 0.0], FLIGHT_MOUNT)


def test_flight_mount_uses_raw_angles() -> None:
    # Flight mount: raw angles are used as reported.
    _, pitch, yaw = normalize_attitude(_quat_wxyz(0.0, 20.0, 45.0), FLIGHT_MOUNT)
    assert (pitch, yaw) == pytest.approx((20.0, 45.0))


def test_bench_mount_negates_pitch_and_shifts_yaw() -> None:
    # Base-down on the bench the A8 reports roll 180 and yaw +180.
    _, pitch, yaw = normalize_attitude(_quat_wxyz(180.0, -20.0, -135.0), BENCH_MOUNT)
    assert (pitch, yaw) == pytest.approx((20.0, 45.0))


def test_decode_flags() -> None:
    assert decode_flags(0) == "none"
    flags = GimbalDeviceFlags.YAW_LOCK | GimbalDeviceFlags.YAW_IN_VEHICLE_FRAME
    assert decode_flags(flags) == "YAW_LOCK|YAW_IN_VEHICLE_FRAME"


def test_packet_roundtrip_and_crc() -> None:
    pkt = build_packet(CMD_ZOOM_MULTIPLE, seq=7)
    assert pkt[:3] == b"\x55\x66\x01"
    assert parse_packet(pkt) == (CMD_ZOOM_MULTIPLE, b"")
    corrupted = pkt[:-1] + bytes([pkt[-1] ^ 0xFF])
    assert parse_packet(corrupted) is None
    # SIYI SDK manual: "request gimbal attitude" (0x0D) and the heartbeat (one payload byte).
    assert build_packet(0x0D).hex() == "556601000000000de805"
    assert build_packet(0x00, b"\x00").hex() == "556601010000000000598b"


def test_zoom_parsing() -> None:
    assert parse_zoom(bytes([2, 5])) == 2.5


class _FakeSdkCamera:
    """SIYI SDK responder on localhost: every request is answered with zoom 2.5x."""

    def __init__(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", 0))
        self.port: int = self._sock.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self) -> None:
        while not self._stop.is_set():
            if not select.select([self._sock], [], [], 0.05)[0]:
                continue
            data, addr = self._sock.recvfrom(1024)
            cmd, _ = parse_packet(data) or (0, b"")
            self._sock.sendto(build_packet(cmd, bytes([2, 5])), addr)

    def close(self) -> None:
        self._stop.set()
        self._thread.join()
        self._sock.close()


def test_sdk_query_survives_a_closed_port() -> None:
    # Camera down or booting: the ICMP port-unreachable surfaces as ConnectionRefusedError on
    # the connected socket. A raise would kill the gimbal's zoom poll thread.
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    sdk = SiyiSdk("127.0.0.1", port)
    sdk.open()
    try:
        assert sdk.query_zoom() is None
    finally:
        sdk.close()


@pytest.fixture
def gimbal() -> Iterator[tuple[SiyiA8Gimbal, dict[str, list[Any]]]]:
    g = SiyiA8Gimbal(aim_enabled=True)
    published: dict[str, list[Any]] = {"tf": [], "camera_info": [], "gimbal_target": []}
    for name, sink in published.items():
        getattr(g, name).publish = sink.append
    yield g, published
    g.stop()


def test_ports(gimbal: tuple[SiyiA8Gimbal, dict[str, list[Any]]]) -> None:
    g, _ = gimbal
    assert set(g.inputs) == {"gimbal_attitude", "target_los"}
    assert set(g.outputs) == {"tf", "camera_info", "gimbal_target"}
    assert {"aim", "state"} <= set(g.rpcs)


@pytest.mark.parametrize("field", ["tf_hz", "aim_hz", "zoom_poll_s"])
def test_rates_and_periods_must_be_positive(field: str) -> None:
    with pytest.raises(ValidationError, match=field):
        SiyiA8Gimbal(**{field: 0.0})


def test_start_publishes_tf_and_polls_the_zoom_then_stop_ends_the_threads(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    camera = _FakeSdkCamera()
    monkeypatch.setattr(gimbal_module, "SiyiSdk", lambda ip: SiyiSdk(ip, camera.port))
    g = SiyiA8Gimbal(ip="127.0.0.1", tf_hz=50.0, zoom_poll_s=0.05)
    published: dict[str, list[Any]] = {"tf": [], "camera_info": []}
    for name, sink in published.items():
        getattr(g, name).publish = sink.append
    for port in (g.gimbal_attitude, g.target_los):
        monkeypatch.setattr(port, "subscribe", lambda _cb: lambda: None)
    try:
        g.start()
        g._on_attitude(AttitudeSample(time.time(), 0.0, -20.0, 30.0).joint_state())
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not (published["tf"] and g.state()["zoom"]):
            time.sleep(0.02)
    finally:
        g.stop()
        camera.close()
    assert published["tf"]
    assert g.state()["zoom"] == 2.5
    assert published["camera_info"] == []  # never while the zoom is unknown or 2.5x
    assert not [t for t in threading.enumerate() if t.name.startswith("siyi-")]


def test_tf_chain_follows_the_attitude(
    gimbal: tuple[SiyiA8Gimbal, dict[str, list[Any]]],
) -> None:
    g, published = gimbal
    sample = AttitudeSample(1.0, 0.0, -20.0, 30.0)
    g._on_attitude(sample.joint_state())
    assert g.publish_transforms()
    (msg,) = published["tf"]
    edges = {(t.frame_id, t.child_frame_id): t for t in msg.transforms}
    assert set(edges) == {
        ("base_link", "gimbal_base"),
        ("gimbal_base", "gimbal_link"),
        ("gimbal_link", "a8_optical"),
    }
    link = edges[("gimbal_base", "gimbal_link")]
    euler = link.rotation.to_euler()
    # MAVLink yaw clockwise positive and pitch up positive become FLU's opposite signs.
    assert math.degrees(euler.z) == pytest.approx(-sample.yaw_deg, abs=0.05)
    assert math.degrees(euler.y) == pytest.approx(-sample.pitch_deg, abs=0.05)
    optical = edges[("gimbal_link", "a8_optical")]
    assert (optical.rotation.x, optical.rotation.y, optical.rotation.z, optical.rotation.w) == (
        -0.5,
        0.5,
        -0.5,
        0.5,
    )
    assert g.state()["flags"] == "YAW_LOCK|YAW_IN_VEHICLE_FRAME"


def test_stale_attitude_publishes_nothing(
    gimbal: tuple[SiyiA8Gimbal, dict[str, list[Any]]],
) -> None:
    g, published = gimbal
    assert not g.publish_transforms()
    g._on_attitude(AttitudeSample(1.0, 0.0, -20.0, 30.0).joint_state())
    assert g._attitude is not None
    g._attitude.rx_mono -= 5.0  # older than attitude_max_age_s
    assert not g.publish_transforms()
    assert published["tf"] == []


def test_camera_info_is_the_a8_main_stream_and_gated_by_zoom(
    gimbal: tuple[SiyiA8Gimbal, dict[str, list[Any]]],
) -> None:
    g, published = gimbal
    assert g.publish_camera_info()
    (info,) = published["camera_info"]
    assert (info.width, info.height, info.frame_id) == (1280, 720, "a8_optical")
    assert info.K[0] == pytest.approx(749.3, abs=0.1) and info.K[2] == pytest.approx(640.0)
    g._zoom = 2.0
    assert not g.publish_camera_info()
    assert len(published["camera_info"]) == 1


class _ScriptedSdk:
    def __init__(self, zooms: list[float | None]) -> None:
        self._zooms = zooms

    def query_zoom(self) -> float | None:
        return self._zooms.pop(0)

    def close(self) -> None:
        pass


def test_zoom_gate_fails_closed() -> None:
    g = SiyiA8Gimbal(ip="127.0.0.1")
    infos: list[Any] = []
    g.camera_info.publish = infos.append
    g._sdk = _ScriptedSdk([None, 1.0, 2.5, None])  # type: ignore[assignment]
    g._stop_event.set()  # one poll per _zoom_loop call
    try:
        assert not g.publish_camera_info()  # polled, not read yet
        g._zoom_loop()
        assert not g.publish_camera_info()  # the first poll went unanswered
        g._zoom_loop()
        assert g.publish_camera_info()
        g._zoom_loop()
        assert not g.publish_camera_info()
        g._zoom_loop()  # a lost reply does not reopen the gate
        assert g.state()["zoom"] == 2.5 and not g.publish_camera_info()
    finally:
        g.stop()
    assert len(infos) == 1


def test_aim_from_line_of_sight_is_clamped_and_rate_limited(
    gimbal: tuple[SiyiA8Gimbal, dict[str, list[Any]]],
) -> None:
    g, published = gimbal
    # Target 150 deg to the left (FLU counter-clockwise) and 10 deg below the horizon.
    los = PoseStamped(
        ts=1.0,
        frame_id="base_link",
        position=Vector3(),
        orientation=Quaternion.from_euler(Vector3(0.0, math.radians(10.0), math.radians(150.0))),
    )
    g._on_target_los(los)
    (target,) = published["gimbal_target"]
    pitch, yaw = (math.degrees(p) for p in target.position)
    assert target.name == ["gimbal_pitch", "gimbal_yaw"]
    assert pitch == pytest.approx(-10.0, abs=0.05)  # nose-down LOS -> gimbal pitch down
    assert yaw == pytest.approx(-120.0, abs=0.05)  # left of the nose, clamped at the yaw limit
    g._on_target_los(los)  # inside the 10 Hz window: not sent again
    assert len(published["gimbal_target"]) == 1


def test_non_finite_aim_is_dropped(gimbal: tuple[SiyiA8Gimbal, dict[str, list[Any]]]) -> None:
    g, published = gimbal
    assert not g.aim(math.nan, 0.0)
    assert not g.aim(0.0, math.inf)
    for q in (Quaternion(0.0, 0.0, 0.0, 0.0), Quaternion(math.nan, 0.0, 0.0, 1.0)):
        g._on_target_los(PoseStamped(ts=1.0, frame_id="base_link", orientation=q))
    assert published["gimbal_target"] == []
    assert g.aim(-10.0, 20.0)  # the rate window was not spent on the rejected requests


def test_aim_disabled_publishes_no_target() -> None:
    g = SiyiA8Gimbal(aim_enabled=False)
    sent: list[Any] = []
    g.gimbal_target.publish = sent.append
    try:
        g._on_target_los(PoseStamped(ts=1.0, frame_id="base_link"))
        assert sent == []
        assert g.aim(-10.0, 20.0)  # the manual RPC still publishes a request
        assert len(sent) == 1
    finally:
        g.stop()
