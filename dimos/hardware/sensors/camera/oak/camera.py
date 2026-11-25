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

"""Luxonis OAK cameras on DepthAI v3: colour, depth aligned to colour, intrinsics and tf.

RVC2 cameras only (the OAK-D family). Colour is undistorted and MJPEG-encoded on the camera, and
depth is aligned to it there, so both images share one pinhole model and the stream fits a USB 2
link. ``dimos run oak-camera-vis`` shows the camera in Rerun; README.md says what to install.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import timedelta
from functools import partial
from importlib import import_module
import ipaddress
import os
from pathlib import Path
import queue
import threading
import time
from typing import Any, Literal, TypeVar, cast

from pydantic import Field
import reactivex as rx

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.camera.spec import OPTICAL_ROTATION, DepthCameraConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.spec import perception
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()

T = TypeVar("T")

LUXONIS_USB_VENDOR = "03e7"
# An OAK enumerates with this product id until DepthAI boots it, and with another while it runs.
OAK_UNBOOTED_USB_PRODUCT = "2485"
SYS_USB_DEVICES = Path("/sys/bus/usb/devices")
DEV_USB = Path("/dev/bus/usb")
# How long an unbooted OAK must stay on the bus before it is opened, and the longest wait for that.
USB_SETTLE_S = 2.0
USB_SETTLE_TIMEOUT_S = 20.0
_USB_POLL_S = 0.25
# The stereo pair's resolution (the OAK-D mono sensors binned); depth is resampled to colour.
MONO_SIZE = (640, 400)
# StereoDepth outputs uint16 millimetres.
DEPTH_SCALE = 0.001
_FRAME_WAIT = timedelta(seconds=0.2)
_STOP_POLL_S = 0.2
# A gap in colour this long is logged; `stall_timeout` brings the camera up again.
_SILENCE_WARN_S = 1.0
# Attempts in a row that find other OAKs but not `device_id` before giving up.
_DEVICE_ID_MISSES = 3
# Closing the device takes a few seconds; a stop waits this long for it.
_CLOSE_WAIT_S = 8.0
_POINTCLOUD_VOXEL_M = 0.005
_UDEV_HINT = "Linux needs the Luxonis udev rule (dimos/hardware/sensors/camera/oak/README.md)"


class OakCameraConfig(ModuleConfig, DepthCameraConfig):
    """Configuration of an OAK camera.

    The defaults (1280x720 colour at 15 Hz, depth at 5 Hz) fit a USB 2 link, which is what
    an OAK on a long or passive cable often gets.

    Attributes:
        frame_id: The camera's tf link, at its colour imager. The colour frames hang off it
            with ``_link`` dropped: ``camera_link`` -> ``camera_color_optical_frame``.
        depth_fps: Rate of the stereo pair, and so of depth.
        align_depth_to_color: Depth is always aligned to colour on the camera.
        device_id: The DepthAI device id (MxId), USB port name or IP address; None opens the
            first OAK.
        mjpeg_quality: JPEG quality of the colour stream (1 to 100).
        open_timeout: Seconds that opening the device, building the pipeline and starting it
            may take together before the attempt is abandoned and tried again.
        retry_interval: Seconds between attempts to bring the camera up.
        stall_timeout: Seconds without a colour picture after which the camera is closed and
            brought up again.
    """

    width: int = Field(default=1280, gt=0)
    height: int = Field(default=720, gt=0)
    fps: float = Field(default=15.0, gt=0.0)
    depth_fps: float = Field(default=5.0, gt=0.0)
    frame_id: str | None = "camera_link"
    align_depth_to_color: Literal[True] = True
    enable_depth: bool = True
    enable_pointcloud: bool = False
    pointcloud_fps: float = Field(default=5.0, gt=0.0)
    camera_info_fps: float = Field(default=1.0, gt=0.0)
    device_id: str | None = None
    mjpeg_quality: int = Field(default=90, ge=1, le=100)
    open_timeout: float = Field(default=40.0, gt=0.0)
    retry_interval: float = Field(default=4.0, ge=0.0)
    stall_timeout: float = Field(default=10.0, gt=0.0)


class OakSetupError(Exception):
    """The camera or this machine cannot run the configured pipeline: trying again cannot help."""


class DeviceIdNotFoundError(OakSetupError):
    """Other OAKs are there, but not the configured one."""


@dataclass(frozen=True)
class OakUsbDevice:
    product: str
    node: Path


def oak_usb_devices(devices: Path = SYS_USB_DEVICES, dev_usb: Path = DEV_USB) -> list[OakUsbDevice]:
    """The Luxonis devices on the USB bus and their device nodes, read from Linux sysfs."""
    found = []
    for vendor in devices.glob("*/idVendor"):
        folder = vendor.parent
        try:
            if vendor.read_text().strip() != LUXONIS_USB_VENDOR:
                continue
            product = (folder / "idProduct").read_text().strip()
            bus = int((folder / "busnum").read_text())
            number = int((folder / "devnum").read_text())
        except (OSError, ValueError):
            # The device left the bus between the glob and the read.
            continue
        found.append(OakUsbDevice(product, dev_usb / f"{bus:03d}" / f"{number:03d}"))
    return found


def oak_usb_products(devices: Path = SYS_USB_DEVICES) -> list[str]:
    """The USB product ids of the Luxonis devices on the bus."""
    return [device.product for device in oak_usb_devices(devices)]


def usb_permission_problem(devices: list[OakUsbDevice]) -> str | None:
    """Why this user cannot open the OAKs on the bus, or None when it can."""
    blocked = [
        str(device.node)
        for device in devices
        if device.node.exists() and not os.access(device.node, os.R_OK | os.W_OK)
    ]
    if not blocked:
        return None
    return f"no permission to open the OAK at {', '.join(blocked)}: {_UDEV_HINT}, then replug it"


def wait_for_unbooted_oak(
    stop: threading.Event,
    settle_s: float = USB_SETTLE_S,
    timeout_s: float = USB_SETTLE_TIMEOUT_S,
    products: Callable[[], list[str]] = oak_usb_products,
    now: Callable[[], float] = time.monotonic,
) -> bool:
    """Wait until an unbooted OAK has stayed on the USB bus for ``settle_s``.

    A closed OAK drops off the bus and comes back unbooted seconds later. A DepthAI open in
    between could hang without an error. False when that did not happen within ``timeout_s``
    (no OAK, or one held by another process): the caller opens anyway and DepthAI says why
    it cannot.
    """
    deadline = now() + timeout_s
    since: float | None = None
    while not stop.is_set() and now() < deadline:
        if OAK_UNBOOTED_USB_PRODUCT in products():
            since = now() if since is None else since
            if now() - since >= settle_s:
                return True
        else:
            since = None
        stop.wait(_USB_POLL_S)
    return False


def call_within(seconds: float, work: Callable[[], T], stop: threading.Event) -> T:
    """Run ``work`` on a daemon thread and return its result.

    Raises TimeoutError when it is not done within ``seconds`` or ``stop`` is set first.
    DepthAI calls cannot be interrupted, so one that does not return is left on its thread.
    """
    results: queue.Queue[tuple[bool, Any]] = queue.Queue(maxsize=1)

    def run() -> None:
        try:
            results.put((True, work()))
        except Exception as error:
            # Re-raised on the caller's thread, which decides what is retried.
            results.put((False, error))

    threading.Thread(target=run, name="oak-bring-up", daemon=True).start()
    deadline = time.monotonic() + seconds
    while True:
        try:
            ok, value = results.get(timeout=_STOP_POLL_S)
            break
        except queue.Empty:
            if stop.is_set():
                raise TimeoutError("stopped") from None
            if time.monotonic() >= deadline:
                raise TimeoutError(f"not done within {seconds:g} s") from None
    if not ok:
        raise value
    return cast("T", value)


def _close_part(name: str, close: Callable[[], None]) -> None:
    try:
        close()
    except RuntimeError as error:
        # Closing a device that went away fails; there is nothing left to close.
        logger.warning("OAK did not close cleanly", part=name, error=str(error))


class AbandonedBringUpError(RuntimeError):
    """A bring-up that came back after it was given up; what it opened is closed."""


@dataclass
class _Session:
    """What one bring-up has opened so far; it streams once ``color`` is set."""

    stage: str = "opening the device"
    name: str = ""
    usb_speed: str = ""
    device: Any = None
    pipeline: Any = None
    color: Any = None
    depth: Any = None
    abandoned: bool = False
    lock: threading.Lock = field(default_factory=threading.Lock)

    def advance(self, stage: str, **opened: Any) -> None:
        """Record what the bring-up opened. A bring-up given up meanwhile closes it and ends."""
        with self.lock:
            for name, value in opened.items():
                setattr(self, name, value)
            self.stage = stage
            abandoned = self.abandoned
        if abandoned:
            self.close()
            raise AbandonedBringUpError(stage)

    def close(self) -> None:
        """Close what is open now; a bring-up still running closes what it opens later."""
        with self.lock:
            pipeline, device = self.pipeline, self.device
            self.pipeline = self.device = None
        if pipeline is not None:
            _close_part("pipeline", pipeline.stop)
        if device is not None:
            _close_part("device", device.close)

    def abandon(self) -> None:
        """Give the session up: close it on a daemon thread, as closing a device that hung or
        vanished can hang too, and have a bring-up still running close what it opens later."""
        with self.lock:
            self.abandoned = True
        threading.Thread(target=self.close, name="oak-close", daemon=True).start()


def _is_ip_address(device_id: str | None) -> bool:
    try:
        ipaddress.ip_address(device_id or "")
    except ValueError:
        return False
    return True


def _check_device_id(dai: Any, device_id: str) -> None:
    available = dai.Device.getAllAvailableDevices()
    if available and not any(device_id in (info.deviceId, info.name) for info in available):
        found = ", ".join(f"{info.deviceId} ({info.name})" for info in available)
        raise DeviceIdNotFoundError(
            f"no OAK with device_id {device_id!r}; found {found} "
            "(an OAK in use by another process is not listed)"
        )


def _check_device(dai: Any, device: Any, config: OakCameraConfig) -> None:
    """Refuse a camera that cannot run the configured pipeline."""
    if device.getPlatform() != dai.Platform.RVC2:
        raise OakSetupError(
            f"{device.getDeviceName()} is not an RVC2 OAK; only the RVC2 family (OAK-D) is supported"
        )
    sensors = {feature.socket: feature for feature in device.getConnectedCameraFeatures()}
    color = sensors.get(dai.CameraBoardSocket.CAM_A)
    if color is None:
        raise OakSetupError("this OAK has no colour camera on CAM_A")
    if config.width > color.width or config.height > color.height:
        raise OakSetupError(
            f"{config.width}x{config.height} is larger than the colour sensor's "
            f"{color.width}x{color.height}"
        )
    stereo = {dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C}
    if config.enable_depth and not stereo <= sensors.keys():
        raise OakSetupError(
            "this OAK has no stereo pair on CAM_B and CAM_C: set enable_depth to false"
        )


def _wall_time(dai: Any, frame: Any) -> float:
    # Device timestamps are on the host's steady clock.
    age: float = (dai.Clock.now() - frame.getTimestamp()).total_seconds()
    return time.time() - age


def _camera_info(transformation: Any, frame_id: str) -> CameraInfo:
    """The pinhole model DepthAI reports for an undistorted (or aligned) frame."""
    width, height = transformation.getSize()
    k = transformation.getIntrinsicMatrix()
    fx, fy, cx, cy = k[0][0], k[1][1], k[0][2], k[1][2]
    return CameraInfo(
        height=height,
        width=width,
        distortion_model="plumb_bob",
        D=[0.0] * 5,
        K=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        frame_id=frame_id,
    )


class OakCamera(Module, perception.DepthCamera):
    """A Luxonis OAK-D camera.

    Brings the camera up on a thread of its own and keeps trying until it streams: after a
    relaunch the OAK is still re-enumerating on USB, and a DepthAI open at that moment fails
    or hangs. A camera that is lost or stops sending pictures is brought up again. A camera
    or machine that cannot run the pipeline (no stereo pair, no permission, ...) is reported
    once and not retried.
    """

    config: OakCameraConfig

    color_image: Out[Image]
    depth_image: Out[Image]
    pointcloud: Out[PointCloud2]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    @classmethod
    def check_dependencies(cls, **kwargs: Any) -> None:
        try:
            import_module("depthai")
        except ImportError as exc:
            raise ImportError(
                f"Luxonis DepthAI SDK unavailable: {exc}. "
                "Install it with the oak extra: `uv sync --extra oak`."
            ) from exc

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        # Each capture thread has its own stop event, so a thread still closing after a
        # stop never runs on alongside the next one.
        self._stop_event = threading.Event()
        self._capture_thread: threading.Thread | None = None
        self._color_info: CameraInfo | None = None
        self._depth_info: CameraInfo | None = None
        self._latest_color: Image | None = None
        self._latest_depth: Image | None = None

    @property
    def _color_frame(self) -> str:
        return f"{self.frame_id.removesuffix('_link')}_color_frame"

    @property
    def _color_optical_frame(self) -> str:
        return f"{self.frame_id.removesuffix('_link')}_color_optical_frame"

    @rpc
    def start(self) -> None:
        super().start()
        stop = threading.Event()
        thread = threading.Thread(
            target=self._capture, args=(stop,), name="oak-camera", daemon=True
        )
        with self._lock:
            self._stop_event, self._capture_thread = stop, thread
        thread.start()
        self.register_disposable(
            rx.interval(1.0 / self.config.camera_info_fps).subscribe(
                lambda _: self._publish_camera_info()
            )
        )
        if self.config.enable_pointcloud and self.config.enable_depth:
            self.register_disposable(
                backpressure(rx.interval(1.0 / self.config.pointcloud_fps)).subscribe(
                    lambda _: self._publish_pointcloud()
                )
            )

    @rpc
    def stop(self) -> None:
        with self._lock:
            stop, thread, self._capture_thread = self._stop_event, self._capture_thread, None
        stop.set()
        if thread is not None:
            thread.join(timeout=_CLOSE_WAIT_S)
            if thread.is_alive():
                logger.warning("OAK still closing after the stop", waited_s=_CLOSE_WAIT_S)
        super().stop()

    def _capture(self, stop: threading.Event) -> None:
        # The vendor SDK is the optional `oak` extra; check_dependencies names it.
        import depthai as dai

        # A PoE OAK (device_id an IP address) is not on the USB bus.
        usb = SYS_USB_DEVICES.is_dir() and not _is_ip_address(self.config.device_id)
        attempt = misses = 0
        while not stop.is_set():
            attempt += 1
            if usb and not wait_for_unbooted_oak(stop) and not stop.is_set():
                logger.warning(
                    "No idle OAK on the USB bus (unplugged, or held by another process?); "
                    "opening anyway",
                    waited_s=USB_SETTLE_TIMEOUT_S,
                )
            if stop.is_set():
                return
            session = _Session()
            try:
                call_within(
                    self.config.open_timeout, partial(self._bring_up, dai, session, usb), stop
                )
            except OakSetupError as error:
                session.abandon()
                misses = (
                    misses + 1 if isinstance(error, DeviceIdNotFoundError) else _DEVICE_ID_MISSES
                )
                if misses >= _DEVICE_ID_MISSES:
                    logger.error("OAK camera not started", reason=str(error))
                    return
                logger.warning("OAK not found, trying again", error=str(error), attempt=attempt)
                stop.wait(self.config.retry_interval)
                continue
            except Exception as error:
                # DepthAI reports a camera that is not ready as RuntimeError; anything else
                # is retried as well, with its traceback.
                session.abandon()
                if stop.is_set():
                    return
                fields = {"attempt": attempt, "stage": session.stage, "error": str(error)}
                if isinstance(error, (RuntimeError, TimeoutError)):
                    logger.warning("OAK not ready, trying again", **fields)
                else:
                    logger.exception("OAK bring-up failed, trying again", **fields)
                stop.wait(self.config.retry_interval)
                continue
            misses = 0
            try:
                ended = self._stream(dai, session, stop, attempt)
            except Exception:
                logger.exception("OAK capture failed, bringing it up again")
                ended = "failed"
            finally:
                self._forget_frames()
            if stop.is_set():
                session.close()
                return
            if ended != "failed":
                logger.error("OAK stopped streaming, bringing it up again", reason=ended)
            session.abandon()

    def _bring_up(self, dai: Any, session: _Session, usb: bool) -> None:
        """Open the OAK, check it, build the pipeline and start it, recording each step in
        ``session``."""
        config = self.config
        if usb and (problem := usb_permission_problem(oak_usb_devices())):
            raise OakSetupError(problem)
        if config.device_id:
            _check_device_id(dai, config.device_id)
            device = dai.Device(dai.DeviceInfo(config.device_id))
        else:
            device = dai.Device()
        session.advance("checking the camera", device=device)
        # Reconnecting is ours: DepthAI's own would keep the pipeline running with idle
        # streams while it retries, without the USB wait and the time limit used here.
        device.setMaxReconnectionAttempts(0)
        _check_device(dai, device, config)
        session.advance(
            "building the pipeline",
            name=device.getDeviceName(),
            usb_speed=str(device.getUsbSpeed()).rpartition(".")[2],
        )
        pipeline = dai.Pipeline(device)
        session.advance("building the pipeline", pipeline=pipeline)
        # With reconnection off, a lost device stops the pipeline at once.
        pipeline.setStopOnDeviceLoss(True)
        color = (
            pipeline.create(dai.node.Camera)
            .build(dai.CameraBoardSocket.CAM_A)
            .requestOutput(
                (config.width, config.height),
                dai.ImgFrame.Type.NV12,
                fps=config.fps,
                enableUndistortion=True,
            )
        )
        depth_queue = None
        if config.enable_depth:
            stereo = pipeline.create(dai.node.StereoDepth)
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
            stereo.setLeftRightCheck(True)
            for socket, stereo_input in (
                (dai.CameraBoardSocket.CAM_B, stereo.left),
                (dai.CameraBoardSocket.CAM_C, stereo.right),
            ):
                mono = pipeline.create(dai.node.Camera).build(socket)
                mono.requestOutput(MONO_SIZE, fps=config.depth_fps).link(stereo_input)
            color.link(stereo.inputAlignTo)
            depth_queue = stereo.depth.createOutputQueue(maxSize=2, blocking=False)
        # MJPEG on the camera: raw NV12 at 720p would not fit a USB 2 link.
        encoder = pipeline.create(dai.node.VideoEncoder).build(
            color,
            frameRate=config.fps,
            profile=dai.VideoEncoderProperties.Profile.MJPEG,
            quality=config.mjpeg_quality,
        )
        color_queue = encoder.out.createOutputQueue(maxSize=4, blocking=False)
        session.advance("starting the pipeline")
        pipeline.start()
        session.advance("streaming", color=color_queue, depth=depth_queue)

    def _stream(self, dai: Any, session: _Session, stop: threading.Event, attempt: int) -> str:
        """Publish frames until the module stops or the camera does; returns why it ended."""
        started = last_color = time.monotonic()
        streaming = warned = False
        while not stop.is_set():
            try:
                if not session.pipeline.isRunning():
                    return "pipeline stopped (device lost)"
                color = session.color.get(_FRAME_WAIT)
                depths = session.depth.tryGetAll() if session.depth is not None else []
            except (dai.MessageQueue.QueueException, RuntimeError) as error:
                # The pipeline and its queues fail once the device goes away.
                return f"lost the device ({error})"
            now = time.monotonic()
            if color is None:
                silent = now - last_color
                if silent >= self.config.stall_timeout:
                    return f"sent no colour picture for {silent:.0f} s"
                if silent >= _SILENCE_WARN_S and not warned:
                    logger.warning(
                        "OAK sends no colour pictures",
                        silent_s=round(silent, 1),
                        restart_after_s=self.config.stall_timeout,
                    )
                    warned = True
            else:
                if not streaming:
                    streaming = True
                    logger.info(
                        "OAK streaming",
                        device=session.name,
                        usb=session.usb_speed,
                        depthai=dai.__version__,
                        width=self.config.width,
                        height=self.config.height,
                        fps=self.config.fps,
                        depth_fps=self.config.depth_fps if self.config.enable_depth else 0.0,
                        first_picture_s=round(now - started, 2),
                        attempt=attempt,
                    )
                last_color, warned = now, False
                self._on_color(color, _wall_time(dai, color))
            for depth in depths:
                self._on_depth(depth, _wall_time(dai, depth))
        return "stopped"

    def _forget_frames(self) -> None:
        """Drop what the last session saw: a camera brought up again may be another OAK."""
        with self._lock:
            self._color_info = self._depth_info = None
            self._latest_color = self._latest_depth = None

    def _on_color(self, frame: Any, ts: float) -> None:
        # A heavy native library: imported where it is used, as DimOS keeps cv2 off module level.
        import cv2

        try:
            bgr = cv2.imdecode(frame.getData(), cv2.IMREAD_COLOR)
        except cv2.error:
            bgr = None
        if bgr is None:
            logger.warning("OAK sent a colour frame that does not decode; dropped")
            return
        image = Image.from_numpy(
            cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB),
            format=ImageFormat.RGB,
            frame_id=self._color_optical_frame,
            ts=ts,
        )
        with self._lock:
            if self._color_info is None:
                self._color_info = _camera_info(
                    frame.getTransformation(), self._color_optical_frame
                )
            if self.config.enable_pointcloud:
                self._latest_color = image
        self.color_image.publish(image)
        self._publish_tf(ts)

    def _on_depth(self, frame: Any, ts: float) -> None:
        # Aligned to colour, so depth is in the colour optical frame.
        image = Image.from_numpy(
            frame.getFrame(),
            format=ImageFormat.DEPTH16,
            frame_id=self._color_optical_frame,
            ts=ts,
        )
        with self._lock:
            if self._depth_info is None:
                self._depth_info = _camera_info(
                    frame.getTransformation(), self._color_optical_frame
                )
            if self.config.enable_pointcloud:
                self._latest_depth = image
        self.depth_image.publish(image)

    def _publish_tf(self, ts: float) -> None:
        self.tf.publish(
            TFMessage(
                Transform(
                    translation=Vector3(0.0, 0.0, 0.0),
                    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                    frame_id=self.frame_id,
                    child_frame_id=self._color_frame,
                    ts=ts,
                ),
                Transform(
                    translation=Vector3(0.0, 0.0, 0.0),
                    rotation=OPTICAL_ROTATION,
                    frame_id=self._color_frame,
                    child_frame_id=self._color_optical_frame,
                    ts=ts,
                ),
            )
        )

    def _publish_camera_info(self) -> None:
        ts = time.time()
        with self._lock:
            color_info, depth_info = self._color_info, self._depth_info
        if color_info is not None:
            self.camera_info.publish(color_info.with_ts(ts))
        if depth_info is not None:
            self.depth_camera_info.publish(depth_info.with_ts(ts))

    def _publish_pointcloud(self) -> None:
        # Each new depth frame with the newest colour (at most one colour period apart); none
        # once the camera stops.
        with self._lock:
            color, depth, info = self._latest_color, self._latest_depth, self._color_info
            self._latest_depth = None
        if color is None or depth is None or info is None:
            return
        cloud = PointCloud2.from_rgbd(
            color_image=color, depth_image=depth, camera_info=info, depth_scale=DEPTH_SCALE
        )
        self.pointcloud.publish(cloud.voxel_downsample(_POINTCLOUD_VOXEL_M))
