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

"""The OAK module against a fake DepthAI: no camera, no SDK needed."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from datetime import timedelta
import os
from pathlib import Path
import queue
import sys
import threading
import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest
from reactivex.scheduler import ThreadPoolScheduler

from dimos.hardware.sensors.camera.oak import camera as oak
from dimos.hardware.sensors.camera.oak.camera import (
    OAK_UNBOOTED_USB_PRODUCT,
    OakCamera,
    call_within,
    oak_usb_devices,
    usb_permission_problem,
    wait_for_unbooted_oak,
)
from dimos.hardware.sensors.camera.spec import OPTICAL_ROTATION
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

WIDTH, HEIGHT = 8, 4
K = [[500.0, 0.0, 4.0], [0.0, 501.0, 2.0], [0.0, 0.0, 1.0]]
OTHER_K = [[600.0, 0.0, 4.0], [0.0, 601.0, 2.0], [0.0, 0.0, 1.0]]
DEVICE_CLOCK_S = 100.0
FRAME_AGE_S = 0.5
# Solid BGR colour of the fake camera's picture.
BGR = (10, 120, 230)
DEPTH_MM = 1000


class FakeTransformation:
    def __init__(self, k: list[list[float]]) -> None:
        self._k = k

    def getSize(self) -> tuple[int, int]:
        return WIDTH, HEIGHT

    def getIntrinsicMatrix(self) -> list[list[float]]:
        return self._k


class FakeFrame:
    def __init__(self, data: np.ndarray, k: list[list[float]] = K) -> None:
        self._data = data
        self._k = k

    def getTimestamp(self) -> timedelta:
        return timedelta(seconds=DEVICE_CLOCK_S - FRAME_AGE_S)

    def getTransformation(self) -> FakeTransformation:
        return FakeTransformation(self._k)

    def getData(self) -> np.ndarray:
        return self._data

    def getFrame(self) -> np.ndarray:
        return self._data


def jpeg_frame(k: list[list[float]] = K) -> FakeFrame:
    picture = np.full((HEIGHT, WIDTH, 3), BGR, dtype=np.uint8)
    ok, encoded = cv2.imencode(".jpg", picture, [cv2.IMWRITE_JPEG_QUALITY, 100])
    assert ok
    return FakeFrame(encoded.reshape(-1), k)


def depth_frame(k: list[list[float]] = K) -> FakeFrame:
    return FakeFrame(np.full((HEIGHT, WIDTH), DEPTH_MM, dtype=np.uint16), k)


class FakeQueue:
    """A DepthAI output queue that serves the frames a test puts in."""

    def __init__(self) -> None:
        self._frames: queue.Queue[FakeFrame] = queue.Queue()

    def put(self, frame: FakeFrame) -> None:
        self._frames.put(frame)

    def get(self, timeout: timedelta) -> FakeFrame | None:
        try:
            return self._frames.get(timeout=timeout.total_seconds())
        except queue.Empty:
            return None

    def tryGetAll(self) -> list[FakeFrame]:
        frames = []
        while not self._frames.empty():
            frames.append(self._frames.get_nowait())
        return frames


class FakeOak:
    """A fake ``depthai`` module. Each bring-up follows the next entry of ``attempts``:
    "ok", "not-found" (Device() raises), "hang-open" (Device() does not return),
    "no-pipeline" (the device opens, Pipeline() raises), "odd-error" (Pipeline() raises
    something that is not a DepthAI error), "hang-start" (pipeline.start() does not return),
    or a camera that cannot run the pipeline: "no-stereo", "rvc4", "small-sensor".

    As documented for DepthAI 3.12 with reconnection off (``setMaxReconnectionAttempts(0)``)
    and ``setStopOnDeviceLoss(True)``, an unplugged camera's pipeline stops at once."""

    def __init__(self, attempts: list[str]) -> None:
        self.attempts = attempts
        self.release = threading.Event()
        self.devices: list[MagicMock] = []
        self.pipelines: list[MagicMock] = []
        self.nodes: dict[Any, MagicMock] = {}
        self.color = FakeQueue()
        self.depth = FakeQueue()
        dai = MagicMock(name="depthai")
        dai.__version__ = "3.12.1"
        dai.MessageQueue.QueueException = type("QueueException", (RuntimeError,), {})
        dai.Clock.now.return_value = timedelta(seconds=DEVICE_CLOCK_S)
        dai.Device.side_effect = self._open
        dai.Pipeline.side_effect = self._pipeline
        dai.Device.getAllAvailableDevices.return_value = []
        self.dai = dai

    def _open(self, *device_info: Any) -> MagicMock:
        behaviour = self.attempts.pop(0) if self.attempts else "ok"
        if behaviour == "not-found":
            raise RuntimeError("X_LINK_DEVICE_NOT_FOUND")
        if behaviour == "hang-open":
            self.release.wait()
        device = MagicMock(name=f"device{len(self.devices)}")
        device.getDeviceName.return_value = "OAK-D-S2"
        device.getUsbSpeed.return_value = "UsbSpeed.HIGH"
        device.behaviour = behaviour
        socket = self.dai.CameraBoardSocket
        sensors = {socket.CAM_A: (4056, 3040), socket.CAM_B: (1280, 800), socket.CAM_C: (1280, 800)}
        if behaviour == "no-stereo":
            del sensors[socket.CAM_B], sensors[socket.CAM_C]
        if behaviour == "small-sensor":
            sensors[socket.CAM_A] = (640, 480)
        device.getConnectedCameraFeatures.return_value = [
            SimpleNamespace(socket=key, width=w, height=h) for key, (w, h) in sensors.items()
        ]
        device.getPlatform.return_value = (
            self.dai.Platform.RVC4 if behaviour == "rvc4" else self.dai.Platform.RVC2
        )
        self.devices.append(device)
        return device

    def _pipeline(self, device: MagicMock) -> MagicMock:
        if device.behaviour == "no-pipeline":
            raise RuntimeError("X_LINK_ERROR")
        if device.behaviour == "odd-error":
            raise ValueError("not a DepthAI error")
        node = self.dai.node
        self.nodes = {
            node.Camera: MagicMock(),
            node.StereoDepth: MagicMock(),
            node.VideoEncoder: MagicMock(),
        }
        self.nodes[
            node.VideoEncoder
        ].build.return_value.out.createOutputQueue.return_value = self.color
        self.nodes[node.StereoDepth].depth.createOutputQueue.return_value = self.depth
        pipeline = MagicMock(name=f"pipeline{len(self.pipelines)}")
        pipeline.create.side_effect = self.nodes.__getitem__
        if device.behaviour == "hang-start":
            pipeline.start.side_effect = self.release.wait
        unplugged = threading.Event()
        pipeline.unplug = unplugged.set
        pipeline.isRunning.side_effect = lambda: not (unplugged.is_set() or pipeline.stop.called)
        self.pipelines.append(pipeline)
        return pipeline


class Received:
    """What a camera published, with an event per stream."""

    def __init__(self, camera: OakCamera) -> None:
        self.messages: dict[str, list[Any]] = {}
        self.events: dict[str, threading.Event] = {}
        for name in (
            "color_image",
            "depth_image",
            "camera_info",
            "depth_camera_info",
            "tf",
            "pointcloud",
        ):
            self.messages[name] = []
            self.events[name] = threading.Event()
            getattr(camera, name).subscribe(self._keep(name))

    def _keep(self, name: str) -> Callable[[Any], None]:
        def keep(message: Any) -> None:
            self.messages[name].append(message)
            self.events[name].set()

        return keep

    def wait(self, *names: str, timeout: float = 10.0) -> None:
        for name in names:
            assert self.events[name].wait(timeout), f"no {name} within {timeout} s"


@pytest.fixture
def fake_oak(
    request: pytest.FixtureRequest, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> Iterator[FakeOak]:
    fake = FakeOak(list(getattr(request, "param", [])))
    monkeypatch.setitem(sys.modules, "depthai", fake.dai)
    # No sysfs: the module does not wait for the USB bus.
    monkeypatch.setattr(oak, "SYS_USB_DEVICES", tmp_path / "no-sysfs")
    yield fake
    fake.release.set()


@pytest.fixture
def pool(monkeypatch: pytest.MonkeyPatch) -> Iterator[ThreadPoolScheduler]:
    # The pointcloud timer runs through backpressure(), whose shared pool lives for the whole
    # process; a pool of its own keeps the thread-leak check clean.
    scheduler = ThreadPoolScheduler(max_workers=1)
    monkeypatch.setattr("dimos.utils.reactive.get_scheduler", lambda: scheduler)
    yield scheduler
    scheduler.executor.shutdown(wait=True)


@pytest.fixture
def start_camera(fake_oak: FakeOak) -> Iterator[Callable[..., tuple[OakCamera, Received]]]:
    cameras: list[OakCamera] = []

    def start(**config: Any) -> tuple[OakCamera, Received]:
        camera = OakCamera(
            **{"open_timeout": 1.0, "retry_interval": 0.0, "camera_info_fps": 50.0, **config}
        )
        cameras.append(camera)
        received = Received(camera)
        camera.start()
        return camera, received

    yield start
    for camera in cameras:
        camera.stop()


def closed(fake_oak: FakeOak, index: int) -> bool:
    return len(fake_oak.devices) > index and fake_oak.devices[index].close.called


def feed(fake_oak: FakeOak, k: list[list[float]] = K) -> None:
    fake_oak.color.put(jpeg_frame(k))
    fake_oak.depth.put(depth_frame(k))


def fake_sysfs(tmp_path: Path, mode: int = 0o666) -> tuple[Path, Path]:
    """A sysfs USB tree with two OAKs and an Intel device, and their device nodes."""
    sysfs, dev = tmp_path / "sys", tmp_path / "dev"
    for name, vendor, product, bus, number in (
        ("1-1", "03e7", "2485", 1, 5),
        ("1-2", "8086", "0b3a", 1, 6),
        ("3-1", "03e7", "f63b", 3, 2),
    ):
        folder = sysfs / name
        folder.mkdir(parents=True)
        for key, value in (
            ("idVendor", vendor),
            ("idProduct", product),
            ("busnum", bus),
            ("devnum", number),
        ):
            (folder / key).write_text(f"{value}\n")
        node = dev / f"{bus:03d}" / f"{number:03d}"
        node.parent.mkdir(parents=True, exist_ok=True)
        node.touch(mode=mode)
    return sysfs, dev


def test_usb_devices_are_read_from_sysfs(tmp_path: Path) -> None:
    sysfs, dev = fake_sysfs(tmp_path)

    devices = sorted(oak_usb_devices(sysfs, dev), key=lambda d: d.product)

    assert [(d.product, d.node) for d in devices] == [
        ("2485", dev / "001" / "005"),
        ("f63b", dev / "003" / "002"),
    ]


@pytest.mark.skipif(os.getuid() == 0, reason="root opens any device node")
def test_a_device_node_this_user_cannot_open_names_the_udev_rule(tmp_path: Path) -> None:
    sysfs, dev = fake_sysfs(tmp_path, mode=0o444)
    open_sysfs, open_dev = fake_sysfs(tmp_path / "open")

    problem = usb_permission_problem(oak_usb_devices(sysfs, dev))

    assert problem is not None and "udev rule" in problem and str(dev / "001" / "005") in problem
    assert usb_permission_problem(oak_usb_devices(open_sysfs, open_dev)) is None


class FakeClock:
    """A clock that a wait advances, so a wait takes no real time."""

    def __init__(self) -> None:
        self.t = 0.0

    def now(self) -> float:
        return self.t

    def is_set(self) -> bool:
        return False

    def wait(self, seconds: float) -> bool:
        self.t += seconds
        return False


def test_waits_until_the_unbooted_oak_has_been_quiet_on_usb() -> None:
    clock = FakeClock()

    def products() -> list[str]:
        # Gone, back for a moment, booted by someone, then back unbooted for good.
        if clock.t < 1.0:
            return []
        if clock.t < 1.5:
            return [OAK_UNBOOTED_USB_PRODUCT]
        if clock.t < 3.0:
            return ["f63b"]
        return [OAK_UNBOOTED_USB_PRODUCT]

    settled = wait_for_unbooted_oak(
        clock, settle_s=2.0, timeout_s=20.0, products=products, now=clock.now
    )

    assert settled
    assert clock.t == 5.0


def test_gives_up_waiting_when_no_unbooted_oak_shows() -> None:
    clock = FakeClock()

    settled = wait_for_unbooted_oak(
        clock, settle_s=2.0, timeout_s=20.0, products=lambda: ["f63b"], now=clock.now
    )

    assert not settled
    assert clock.t == 20.0


def test_call_within_returns_reraises_and_gives_up() -> None:
    stop, hang = threading.Event(), threading.Event()
    try:
        assert call_within(1.0, lambda: "device", stop) == "device"
        with pytest.raises(RuntimeError, match="X_LINK_DEVICE_NOT_FOUND"):
            call_within(
                1.0, lambda: (_ for _ in ()).throw(RuntimeError("X_LINK_DEVICE_NOT_FOUND")), stop
            )
        with pytest.raises(TimeoutError, match="not done within 0.3 s"):
            call_within(0.3, hang.wait, stop)
        stop.set()
        with pytest.raises(TimeoutError, match="stopped"):
            call_within(30.0, hang.wait, stop)
    finally:
        hang.set()


def test_publishes_color_depth_intrinsics_and_tf(
    fake_oak: FakeOak, start_camera: Callable[..., tuple[OakCamera, Received]]
) -> None:
    feed(fake_oak)
    before = time.time()

    _, received = start_camera(frame_id="wrist_link", frame_id_prefix="arm")
    received.wait("color_image", "depth_image", "camera_info", "depth_camera_info", "tf")

    optical = "arm/wrist_color_optical_frame"
    color: Image = received.messages["color_image"][0]
    assert (color.format, color.frame_id, color.data.shape) == (
        ImageFormat.RGB,
        optical,
        (HEIGHT, WIDTH, 3),
    )
    assert np.abs(color.data.astype(int) - BGR[::-1]).max() <= 3
    assert before - FRAME_AGE_S - 0.1 <= color.ts <= time.time() - FRAME_AGE_S + 0.1
    depth: Image = received.messages["depth_image"][0]
    assert (depth.format, depth.frame_id) == (ImageFormat.DEPTH16, optical)
    assert depth.data.tolist() == [[DEPTH_MM] * WIDTH] * HEIGHT
    for name in ("camera_info", "depth_camera_info"):
        info: CameraInfo = received.messages[name][0]
        assert (info.width, info.height, info.frame_id) == (WIDTH, HEIGHT, optical)
        assert list(info.K) == [500.0, 0.0, 4.0, 0.0, 501.0, 2.0, 0.0, 0.0, 1.0]
        assert list(info.D) == [0.0] * 5
    tf: TFMessage = received.messages["tf"][0]
    (mount, optical_edge) = tf.transforms
    assert (mount.frame_id, mount.child_frame_id) == ("arm/wrist_link", "arm/wrist_color_frame")
    assert (optical_edge.frame_id, optical_edge.child_frame_id) == (
        "arm/wrist_color_frame",
        optical,
    )
    assert optical_edge.rotation == OPTICAL_ROTATION
    assert mount.ts == color.ts


def test_pipeline_fits_usb2(
    fake_oak: FakeOak, start_camera: Callable[..., tuple[OakCamera, Received]]
) -> None:
    feed(fake_oak)

    _, received = start_camera()
    received.wait("color_image")

    node, dai = fake_oak.nodes, fake_oak.dai
    requests = node[dai.node.Camera].build.return_value.requestOutput.call_args_list
    color, left, right = (request for request in requests)
    assert color.args == ((1280, 720), dai.ImgFrame.Type.NV12)
    assert color.kwargs == {"fps": 15.0, "enableUndistortion": True}
    assert left.args == right.args == ((640, 400),)
    assert left.kwargs == right.kwargs == {"fps": 5.0}
    encoder = node[dai.node.VideoEncoder].build.call_args
    assert encoder.kwargs == {
        "frameRate": 15.0,
        "profile": dai.VideoEncoderProperties.Profile.MJPEG,
        "quality": 90,
    }


@pytest.mark.parametrize("fake_oak", [["not-found"]], indirect=True)
def test_an_open_that_fails_is_tried_again(
    fake_oak: FakeOak, start_camera: Callable[..., tuple[OakCamera, Received]]
) -> None:
    feed(fake_oak)

    _, received = start_camera()

    received.wait("color_image")
    assert fake_oak.dai.Device.call_count == 2


@pytest.mark.parametrize("fake_oak", [["hang-open"]], indirect=True)
def test_an_open_that_never_returns_is_given_up_and_tried_again(
    fake_oak: FakeOak,
    start_camera: Callable[..., tuple[OakCamera, Received]],
    wait_until: Callable[..., None],
) -> None:
    feed(fake_oak)

    _, received = start_camera()
    received.wait("color_image")
    streaming = fake_oak.devices[0]
    fake_oak.release.set()

    # The first open comes back late with its own device: it is closed, not kept.
    wait_until(
        lambda: len(fake_oak.devices) == 2 and fake_oak.devices[1].close.called, timeout=10.0
    )
    streaming.close.assert_not_called()


@pytest.mark.parametrize("fake_oak", [["hang-start"]], indirect=True)
def test_a_bring_up_that_hangs_after_opening_releases_the_device(
    fake_oak: FakeOak,
    start_camera: Callable[..., tuple[OakCamera, Received]],
    wait_until: Callable[..., None],
) -> None:
    feed(fake_oak)

    _, received = start_camera()

    received.wait("color_image")
    hung, streaming = fake_oak.devices
    wait_until(lambda: hung.close.called, timeout=10.0)
    streaming.close.assert_not_called()


def test_a_camera_that_stops_streaming_is_brought_up_again(
    fake_oak: FakeOak,
    start_camera: Callable[..., tuple[OakCamera, Received]],
    wait_until: Callable[..., None],
) -> None:
    feed(fake_oak)
    _, received = start_camera()
    received.wait("color_image")
    received.events["color_image"].clear()

    # Reconnecting is the module's: DepthAI's own is off, so the lost device stops the pipeline.
    fake_oak.devices[0].setMaxReconnectionAttempts.assert_called_once_with(0)
    fake_oak.pipelines[0].setStopOnDeviceLoss.assert_called_once_with(True)
    fake_oak.pipelines[0].unplug()
    wait_until(lambda: closed(fake_oak, 0), timeout=10.0)
    # Another OAK, another calibration: its own intrinsics are published, not the first one's.
    feed(fake_oak, OTHER_K)

    received.wait("color_image")
    assert len(fake_oak.devices) == 2
    wait_until(lambda: received.messages["camera_info"][-1].K[0] == OTHER_K[0][0], timeout=10.0)


@pytest.mark.parametrize("fake_oak", [["no-pipeline"]], indirect=True)
def test_a_device_whose_pipeline_fails_is_released_and_tried_again(
    fake_oak: FakeOak,
    start_camera: Callable[..., tuple[OakCamera, Received]],
    wait_until: Callable[..., None],
) -> None:
    feed(fake_oak)

    _, received = start_camera()

    received.wait("color_image")
    failed, streaming = fake_oak.devices
    wait_until(lambda: failed.close.called, timeout=10.0)
    streaming.close.assert_not_called()


def test_a_poe_oak_is_opened_without_waiting_for_usb(
    fake_oak: FakeOak,
    start_camera: Callable[..., tuple[OakCamera, Received]],
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    # A USB bus with no OAK on it: a USB OAK would be waited for here for 20 s.
    monkeypatch.setattr(oak, "SYS_USB_DEVICES", tmp_path)
    feed(fake_oak)

    _, received = start_camera(device_id="192.168.1.20")

    received.wait("color_image", timeout=5.0)
    fake_oak.dai.DeviceInfo.assert_called_once_with("192.168.1.20")


def test_stop_closes_the_pipeline_and_the_device(
    fake_oak: FakeOak, start_camera: Callable[..., tuple[OakCamera, Received]]
) -> None:
    feed(fake_oak)
    camera, received = start_camera()
    received.wait("color_image")

    camera.stop()

    fake_oak.pipelines[0].stop.assert_called_once_with()
    fake_oak.devices[0].close.assert_called_once_with()


def test_pointcloud_from_depth_aligned_to_color(
    pool: ThreadPoolScheduler,
    fake_oak: FakeOak,
    start_camera: Callable[..., tuple[OakCamera, Received]],
    wait_until: Callable[..., None],
) -> None:
    feed(fake_oak)

    _, received = start_camera(enable_pointcloud=True, pointcloud_fps=50.0)
    received.wait("pointcloud")

    infos = len(received.messages["camera_info"])
    wait_until(lambda: len(received.messages["camera_info"]) >= infos + 10, timeout=10.0)

    # One depth frame, one cloud: the timer does not publish it again.
    assert len(received.messages["pointcloud"]) == 1
    cloud: PointCloud2 = received.messages["pointcloud"][0]
    points, _ = cloud.as_numpy()
    assert cloud.frame_id == "camera_color_optical_frame"
    assert len(points) > 0
    assert np.allclose(points[:, 2], DEPTH_MM / 1000.0)


def test_missing_sdk_names_the_extra(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setitem(sys.modules, "depthai", None)

    with pytest.raises(ImportError, match="uv sync --extra oak"):
        OakCamera.check_dependencies()


def capture_ended() -> bool:
    return not any(t.name == "oak-camera" and t.is_alive() for t in threading.enumerate())


def test_a_camera_that_sends_nothing_is_brought_up_again(
    fake_oak: FakeOak,
    start_camera: Callable[..., tuple[OakCamera, Received]],
    wait_until: Callable[..., None],
) -> None:
    # The pipeline runs, but no picture comes: what the prototype once saw after a relaunch.
    _, received = start_camera(stall_timeout=0.5)

    wait_until(lambda: closed(fake_oak, 0), timeout=10.0)
    feed(fake_oak)

    received.wait("color_image")
    assert len(fake_oak.devices) == 2


@pytest.mark.parametrize(
    "fake_oak, reason",
    [
        (["no-stereo"], "no stereo pair"),
        (["rvc4"], "not an RVC2"),
        (["small-sensor"], "larger than"),
    ],
    indirect=["fake_oak"],
)
def test_a_camera_that_cannot_run_the_pipeline_is_reported_and_not_retried(
    fake_oak: FakeOak,
    reason: str,
    start_camera: Callable[..., tuple[OakCamera, Received]],
    wait_until: Callable[..., None],
    mocker: Any,
) -> None:
    error = mocker.patch.object(oak.logger, "error")

    start_camera()

    wait_until(capture_ended, timeout=10.0)
    wait_until(lambda: closed(fake_oak, 0), timeout=10.0)
    assert fake_oak.dai.Device.call_count == 1
    assert reason in error.call_args.kwargs["reason"]


def test_a_device_id_that_is_not_there_is_reported_after_three_tries(
    fake_oak: FakeOak,
    start_camera: Callable[..., tuple[OakCamera, Received]],
    wait_until: Callable[..., None],
    mocker: Any,
) -> None:
    fake_oak.dai.Device.getAllAvailableDevices.return_value = [
        SimpleNamespace(deviceId="18443010C1", name="1.3")
    ]
    error = mocker.patch.object(oak.logger, "error")

    start_camera(device_id="WRONG")

    wait_until(capture_ended, timeout=10.0)
    assert fake_oak.dai.Device.getAllAvailableDevices.call_count == 3
    fake_oak.dai.Device.assert_not_called()
    assert "18443010C1" in error.call_args.kwargs["reason"]


@pytest.mark.parametrize("fake_oak", [["odd-error"]], indirect=True)
def test_any_other_bring_up_error_is_tried_again(
    fake_oak: FakeOak, start_camera: Callable[..., tuple[OakCamera, Received]]
) -> None:
    feed(fake_oak)

    _, received = start_camera()

    received.wait("color_image")
    assert len(fake_oak.devices) == 2


class BrokenFrame(FakeFrame):
    def getTransformation(self) -> FakeTransformation:
        raise ValueError("a frame without a transformation")


def test_an_error_while_streaming_brings_the_camera_up_again(
    fake_oak: FakeOak,
    start_camera: Callable[..., tuple[OakCamera, Received]],
    wait_until: Callable[..., None],
) -> None:
    fake_oak.color.put(BrokenFrame(jpeg_frame().getData()))

    _, received = start_camera()
    wait_until(lambda: closed(fake_oak, 0), timeout=10.0)
    feed(fake_oak)

    received.wait("color_image")
    assert len(fake_oak.devices) == 2


def test_color_only_without_depth(
    fake_oak: FakeOak,
    start_camera: Callable[..., tuple[OakCamera, Received]],
    wait_until: Callable[..., None],
) -> None:
    feed(fake_oak)

    _, received = start_camera(enable_depth=False)
    received.wait("color_image", "camera_info")
    wait_until(lambda: len(received.messages["camera_info"]) >= 10, timeout=10.0)

    created = [call.args[0] for call in fake_oak.pipelines[0].create.call_args_list]
    assert fake_oak.dai.node.StereoDepth not in created
    assert created.count(fake_oak.dai.node.Camera) == 1
    assert received.messages["depth_image"] == []
    assert received.messages["depth_camera_info"] == []
