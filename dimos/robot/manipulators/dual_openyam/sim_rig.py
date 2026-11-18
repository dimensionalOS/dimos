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

"""The simulated rig: the workcell model stepped by one MujocoEngine.

``DamiaoSimRig`` stands where :class:`~dimos.robot.manipulators.damiao.arm.DamiaoArm`
stands for the real robot. It takes plain robot data and exposes the same few
operations: connect, command, hold, read and disconnect, plus the rig's
cameras. The DimOS connection module only translates ports and time into
those calls. ``DualOpenYamSimRig`` is the Dual OpenYAM's.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
import queue
import threading
import time
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.robot.manipulators.damiao.robot import DamiaoRobot
from dimos.robot.manipulators.dual_openyam.sim_scene import (
    DUAL_OPENYAM_WORKCELL,
    OVERHEAD_CAMERA,
    SimWorkcell,
    scene_spec,
)
from dimos.simulation.engines.mujoco_engine import MujocoEngine

# How long the first physics step may take to arrive, and how long the physics
# may then stand still before a reading counts as lost, seconds.
_START_TIMEOUT_S = 30.0
_STALLED_AFTER_S = 1.0
# An optical frame (x right, y down, z forward) in MuJoCo's camera frame (x
# right, y up, looking along -z).
_OPTICAL_IN_MUJOCO_CAMERA = np.diag([1.0, -1.0, -1.0])


@dataclass(frozen=True)
class SimReading:
    """One sample of every commanded joint, in the robot's joint order and units.

    Attributes:
        position: Arm joints in rad, each jaw in its gripper joint's unit.
        velocity: Arm joints in rad/s; jaws report 0, as on the robot.
        effort: Arm joint torques in Nm, gravity compensation included; jaws
            report 0, as on the robot.
    """

    position: list[float]
    velocity: list[float]
    effort: list[float]


@dataclass(frozen=True, eq=False)
class SimCameraFrame:
    """One camera's view and where the camera was, in the workcell frame.

    Attributes:
        color: RGB image, height x width x 3.
        depth: Metres along the optical axis per pixel, or ``None`` for a
            colour-only camera.
        position: The optical centre, metres.
        rotation: The optical frame (x right, y down, z forward) as a 3x3
            rotation matrix.
        stamp: When the pictured state was simulated, Unix seconds.
    """

    color: NDArray[np.uint8]
    depth: NDArray[np.float32] | None
    position: NDArray[np.float64]
    rotation: NDArray[np.float64]
    stamp: float


@dataclass(frozen=True, eq=False)
class _Snapshot:
    """The pose of everything after one physics step, and when that step was.

    ``sim_time`` is in simulated seconds, ``stamp`` in Unix seconds and
    ``stepped_at`` on the monotonic clock.
    """

    qpos: NDArray[np.float64]
    sim_time: float
    stamp: float
    stepped_at: float


FramesCallback = Callable[[Mapping[str, SimCameraFrame]], None]


class DamiaoSimRig:
    """A robot's arms, their jaws, the table and its objects in MuJoCo, with the rig's cameras.

    Positions are commanded and read by joint name. Each jaw is its gripper
    joint, in that joint's unit; its closed-to-open range maps linearly onto
    the fingers' travel.

    Rendering runs on its own thread from a snapshot of the last step, so the
    cameras do not slow the physics below real time.

    Args:
        robot: The motors, gains, gripper joints and models to simulate.
        workcell: Where the robot's arms are in the URDF it is built from.
        headless: Run without MuJoCo's viewer window.
        camera_size: Image (width, height) in pixels, or ``None`` to render no
            cameras; rendering needs an OpenGL context.
        camera_fps: How often the cameras are rendered, in frames per second.
        on_frames: Called from the rendering thread with every camera's new
            frame, keyed by camera name.
    """

    def __init__(
        self,
        robot: DamiaoRobot,
        workcell: SimWorkcell,
        *,
        headless: bool = True,
        camera_size: tuple[int, int] | None = (640, 480),
        camera_fps: float = 30.0,
        on_frames: FramesCallback | None = None,
    ) -> None:
        self._robot = robot
        self._workcell = workcell
        self._headless = headless
        self._camera_size = camera_size
        self._camera_fps = camera_fps
        self._on_frames = on_frames
        self._index: dict[str, int] = {}
        # Jaw joint -> ((closed, open) as commanded, (closed, open) in finger travel).
        self._jaws: dict[str, tuple[tuple[float, float], tuple[float, float]]] = {}
        # Guards the engine, the snapshot and the frames: callers, the physics
        # thread and the rendering thread all touch them.
        self._lock = threading.Lock()
        self._engine: MujocoEngine | None = None
        self._snapshot: _Snapshot | None = None
        self._frames: dict[str, SimCameraFrame] = {}
        self._stepped = threading.Event()
        self._stop = threading.Event()
        self._renderer: threading.Thread | None = None
        # The rendering thread reports here once: None when it can render,
        # otherwise why it cannot.
        self._renderer_ready: queue.Queue[Exception | None] = queue.Queue()

    @property
    def joint_names(self) -> list[str]:
        return list(self._robot.joint_names)

    def connect(self) -> None:
        """Build the workcell, start its physics at the home posture and start the cameras.

        Raises:
            RuntimeError: If the rig is already connected, the physics does not
                start stepping, or cameras are wanted and MuJoCo cannot open an
                OpenGL context.
        """
        with self._lock:
            if self._engine is not None:
                raise RuntimeError("the simulated rig is already connected")
        robot = self._robot
        model = scene_spec(robot, self._workcell).compile()
        if self._camera_size is not None:
            width, height = self._camera_size
            model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
            model.vis.global_.offheight = max(model.vis.global_.offheight, height)
        source = self._workcell.urdf.load().source_path
        assert source is not None, "the workcell's URDF is a file"
        engine = MujocoEngine(
            config_path=source,
            headless=self._headless,
            model=model,
            # The jaws keep the model's zero, which is closed.
            reset_joint_positions=robot.home_joints,
        )
        if engine.joint_names != robot.joint_names:
            raise ValueError("the workcell's actuators do not match the robot's joints")
        self._index = {name: index for index, name in enumerate(engine.joint_names)}
        travel = robot.gripper_travel()
        for bus, arm in zip(robot.buses, self._workcell.arms, strict=True):
            if bus.gripper is not None:
                self._jaws[bus.gripper.joint] = (travel[bus.gripper.joint], arm.finger_travel)
        engine.set_step_hooks(after=self._after_step)
        self._stepped.clear()
        self._stop.clear()
        with self._lock:
            self._engine = engine
        try:
            if not engine.connect() or not self._stepped.wait(_START_TIMEOUT_S):
                raise RuntimeError(
                    "the MuJoCo simulation did not start stepping"
                    + ("" if self._headless else "; its viewer window needs a display")
                )
            if self._camera_size is not None:
                self._start_rendering(model)
        except Exception:
            self.disconnect()
            raise

    def disconnect(self) -> None:
        """Stop the cameras and the physics. Safe before ``connect`` and to repeat."""
        self._stop.set()
        renderer, self._renderer = self._renderer, None
        if renderer is not None and renderer is not threading.current_thread():
            renderer.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        with self._lock:
            engine, self._engine = self._engine, None
            self._snapshot = None
            self._frames = {}
        if engine is not None:
            engine.disconnect()

    def command(self, targets: Mapping[str, float]) -> None:
        """Set position targets by joint name; joints left out keep theirs.

        Raises:
            ValueError: If a name is not one of the robot's joints.
            RuntimeError: If the rig is not connected.
        """
        unknown = set(targets) - set(self._index)
        if unknown:
            raise ValueError(f"unknown {self._workcell.name} joints: {sorted(unknown)}")
        engine = self._connected_engine()
        for name, value in targets.items():
            if name in self._jaws:
                commanded, fingers = self._jaws[name]
                value = _rescale(value, commanded, fingers)
            engine.set_position_target(self._index[name], value)

    def hold(self) -> None:
        """Hold every joint where it is now, e.g. when commands stop arriving."""
        with self._lock:
            engine = self._engine
        if engine is not None:
            engine.hold_current_position()

    def read(self) -> SimReading:
        """Sample every joint.

        Raises:
            RuntimeError: If the rig is not connected or its physics stopped
                stepping, e.g. because the viewer window was closed.
        """
        with self._lock:
            engine, snapshot = self._engine, self._snapshot
        if engine is None:
            raise RuntimeError("the simulated rig is not connected")
        if snapshot is None or time.monotonic() - snapshot.stepped_at > _STALLED_AFTER_S:
            raise RuntimeError("the MuJoCo simulation stopped stepping")
        position = engine.joint_positions
        velocity = engine.joint_velocities
        effort = engine.joint_efforts
        for name, (commanded, fingers) in self._jaws.items():
            index = self._index[name]
            # A finger pressed against its end stop reads a hair past it.
            opening = _rescale(position[index], fingers, commanded)
            position[index] = min(max(opening, min(commanded)), max(commanded))
            velocity[index] = 0.0
            effort[index] = 0.0
        return SimReading(position=position, velocity=velocity, effort=effort)

    @property
    def sim_time(self) -> float:
        """Seconds of physics simulated since ``connect``."""
        with self._lock:
            return 0.0 if self._snapshot is None else self._snapshot.sim_time

    def read_camera(self, name: str) -> SimCameraFrame | None:
        """The named camera's newest frame, or ``None`` before its first."""
        with self._lock:
            return self._frames.get(name)

    def body_position(self, name: str) -> NDArray[np.float64]:
        """Where a body of the workcell is now, e.g. "rubiks_cube", metres in the workcell frame.

        Raises:
            ValueError: If the workcell has no such body.
            RuntimeError: If the rig is not connected.
        """
        pose = self._connected_engine().get_body_pose(name)
        if pose is None:
            raise ValueError(f"the workcell has no body named {name!r}")
        return pose[0]

    def _connected_engine(self) -> MujocoEngine:
        with self._lock:
            engine = self._engine
        if engine is None:
            raise RuntimeError("the simulated rig is not connected")
        return engine

    def _after_step(self, engine: MujocoEngine) -> None:
        """Runs on the physics thread after every step."""
        data = engine.data
        snapshot = _Snapshot(data.qpos.copy(), float(data.time), time.time(), time.monotonic())
        with self._lock:
            self._snapshot = snapshot
        self._stepped.set()

    def _start_rendering(self, model: mujoco.MjModel) -> None:
        self._renderer = threading.Thread(
            target=self._render_loop, args=(model,), daemon=True, name="damiao-sim-render"
        )
        self._renderer.start()
        try:
            error = self._renderer_ready.get(timeout=_START_TIMEOUT_S)
        except queue.Empty:
            raise RuntimeError("the cameras did not start rendering") from None
        if error is not None:
            raise RuntimeError(
                "MuJoCo cannot open an OpenGL context for the cameras; on a machine "
                "without a display set MUJOCO_GL=egl"
            ) from error

    def _render_loop(self, model: mujoco.MjModel) -> None:
        """Render every camera from the newest snapshot, at the camera rate."""
        assert self._camera_size is not None
        width, height = self._camera_size
        try:
            color = mujoco.Renderer(model, height=height, width=width)
            depth = mujoco.Renderer(model, height=height, width=width)
        except mujoco.FatalError as error:  # type: ignore[attr-defined]
            self._renderer_ready.put(error)
            return
        depth.enable_depth_rendering()
        self._renderer_ready.put(None)
        cameras = [OVERHEAD_CAMERA, *(arm.wrist_camera for arm in self._workcell.arms)]
        view = mujoco.MjData(model)
        period = 1.0 / self._camera_fps
        rendered = 0.0
        due = time.monotonic()
        with color, depth:
            while not self._stop.is_set():
                with self._lock:
                    snapshot = self._snapshot
                if snapshot is not None and snapshot.stamp > rendered:
                    rendered = snapshot.stamp
                    view.qpos[:] = snapshot.qpos
                    mujoco.mj_forward(model, view)
                    frames = {}
                    for name in cameras:
                        color.update_scene(view, camera=name)
                        image = color.render()
                        distance = None
                        # Only the overhead camera is RGB-D, as on the rig.
                        if name == OVERHEAD_CAMERA:
                            depth.update_scene(view, camera=name)
                            distance = np.asarray(depth.render(), dtype=np.float32)
                        camera = model.camera(name).id
                        frames[name] = SimCameraFrame(
                            color=image,
                            depth=distance,
                            position=view.cam_xpos[camera].copy(),
                            rotation=view.cam_xmat[camera].reshape(3, 3)
                            @ _OPTICAL_IN_MUJOCO_CAMERA,
                            stamp=snapshot.stamp,
                        )
                    with self._lock:
                        self._frames = frames
                    if self._on_frames is not None:
                        self._on_frames(frames)
                # Keep to the schedule; a whole frame behind, start it afresh.
                due += period
                now = time.monotonic()
                if now - due > period:
                    due = now
                self._stop.wait(max(0.0, due - now))


class DualOpenYamSimRig(DamiaoSimRig):
    """Both Dual OpenYAM arms; ``options`` as for ``DamiaoSimRig``."""

    def __init__(self, robot: DamiaoRobot, **options: Any) -> None:
        super().__init__(robot, DUAL_OPENYAM_WORKCELL, **options)


def _rescale(value: float, source: tuple[float, float], target: tuple[float, float]) -> float:
    """Map ``value`` from one (closed, open) range onto another, linearly."""
    fraction = (value - source[0]) / (source[1] - source[0])
    return target[0] + fraction * (target[1] - target[0])
