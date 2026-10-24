# Copyright 2025-2026 Dimensional Inc.
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

from pathlib import Path
import threading
import time
from typing import Any
from unittest.mock import MagicMock

import mujoco
import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.simulation.engines.mujoco_engine import CameraConfig, CameraFrame, MujocoEngine
from dimos.simulation.engines.mujoco_shm import CMD_MODE_PD_TAU
from dimos.simulation.engines.mujoco_sim_module import (
    MujocoSimModule,
    MujocoSimModuleConfig,
    _WholeBodySimHooks,
)


class _FakeData:
    qpos = np.array([0.0, 0.0, 0.75, 1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    sensordata = np.array([0.1, 0.2, 0.3, 1.0, 2.0, 3.0], dtype=np.float64)


class _FakeEngine:
    data = _FakeData()
    joint_names = ["joint_a", "joint_b"]


class _FakeRespawnEngine:
    def __init__(self, *, ground_z: float = 0.08) -> None:
        self.ground_z = ground_z
        self.reset_requested = False
        self.reset_to_kwargs: dict[str, Any] | None = None

    def request_reset(self, *, wait: bool) -> bool:
        self.reset_requested = wait
        return True

    def ground_height_at(self, x: float, y: float) -> float:
        assert x == pytest.approx(2.6)
        assert y == pytest.approx(0.0)
        return self.ground_z

    def request_reset_to(
        self,
        *,
        spawn_xy: tuple[float, float],
        spawn_z: float | None,
        spawn_yaw: float | None,
        wait: bool,
    ) -> bool:
        self.reset_to_kwargs = {
            "spawn_xy": spawn_xy,
            "spawn_z": spawn_z,
            "spawn_yaw": spawn_yaw,
            "wait": wait,
        }
        return True

    def disconnect(self) -> None:
        pass


class _FakeSimHooks:
    def __init__(self) -> None:
        self.cleared = False

    def clear_latched_commands(self) -> None:
        self.cleared = True


def test_whole_body_physics_waits_for_complete_pd_command(mocker: Any) -> None:
    shm = mocker.MagicMock()
    shm.read_command_mode.return_value = CMD_MODE_PD_TAU
    shm.read_position_command.return_value = None
    shm.read_velocity_command.return_value = None
    shm.read_kp_command.return_value = None
    shm.read_kd_command.return_value = None
    shm.read_tau_command.return_value = None
    engine = mocker.MagicMock()
    engine.joint_positions = [0.0, 0.0]
    engine.joint_velocities = [0.0, 0.0]
    hooks = _WholeBodySimHooks(shm, dof=2)

    hooks.pre_step(engine)
    assert hooks.ready_for_physics() is False

    shm.read_position_command.return_value = np.array([0.1, 0.2])
    shm.read_kp_command.return_value = np.array([10.0, 10.0])
    shm.read_kd_command.return_value = np.array([1.0, 1.0])
    hooks.pre_step(engine)

    assert hooks.ready_for_physics() is True


def test_ready_signal_happens_after_joint_state_and_imu_write() -> None:
    events: list[str] = []
    module = MujocoSimModule()
    module.config = MujocoSimModuleConfig(dof=2)
    module._shm_ready_signaled = False
    module._root_base_qpos_adr = 0
    module._imu_quat_slice = None
    module._imu_base_qpos_slice = slice(3, 7)
    module._imu_gyro_slice = slice(0, 3)
    module._imu_accel_slice = slice(3, 6)
    module.odom = MagicMock()
    module.imu = MagicMock()

    class _FakeHooks:
        def post_step(self, engine: Any) -> None:
            assert engine is _FakeEngine
            events.append("joint_state")

    class _FakeShm:
        def write_imu(self, **_: Any) -> None:
            events.append("imu")

        def signal_ready(self, *, num_joints: int, arm_joints: int) -> None:
            assert num_joints == 2
            assert arm_joints == 2
            events.append("ready")

        def signal_stop(self) -> None:
            pass

        def cleanup(self) -> None:
            pass

    try:
        module._root_base_qpos_adr = 0
        module._imu_base_qpos_slice = slice(3, 7)
        module._imu_gyro_slice = slice(0, 3)
        module._imu_accel_slice = slice(3, 6)
        module._sim_hooks = _FakeHooks()
        module._shm = _FakeShm()

        module._publish_shm_and_lcm(_FakeEngine)

        assert events == ["joint_state", "imu", "ready"]
    finally:
        module.stop()


def test_camera_tf_is_published_relative_to_configured_base_frame() -> None:
    module = MujocoSimModule(base_frame_id="link7")
    try:
        module.config = MujocoSimModuleConfig(base_frame_id="link7")

        class _FakeEngine:
            def get_body_pose(self, body_name: str) -> tuple[np.ndarray, np.ndarray]:
                raise AssertionError("TF publication must use the frame snapshot")

            def disconnect(self) -> None:
                pass

        messages: list[Any] = []
        module._engine = _FakeEngine()
        module.tf.subscribe(messages.append)
        frame = CameraFrame(
            rgb=np.zeros((1, 1, 3), dtype=np.uint8),
            depth=np.ones((1, 1), dtype=np.float32),
            cam_pos=np.array([1.0, 2.0, 3.0]),
            cam_mat=np.eye(3),
            fovy=60.0,
            timestamp=1.0,
            base_pos=np.array([1.0, 2.0, 2.0]),
            base_mat=np.eye(3),
        )

        module._publish_tf(10.0, frame)

        color_tf, depth_tf, camera_link_tf = messages[-1].transforms
        assert color_tf.frame_id == "link7"
        assert color_tf.child_frame_id == "wrist_camera_color_optical_frame"
        assert np.allclose(color_tf.translation.to_numpy(), [0.0, 0.0, 1.0])
        assert depth_tf.frame_id == "link7"
        assert depth_tf.child_frame_id == "wrist_camera_depth_optical_frame"
        assert camera_link_tf.frame_id == "link7"
        assert camera_link_tf.child_frame_id == "wrist_camera_link"
        assert np.allclose(camera_link_tf.translation.to_numpy(), [0.0, 0.0, 1.0])
    finally:
        module.stop()


def test_tracked_bodies_are_published_on_tf_in_world() -> None:
    module = MujocoSimModule(tracked_bodies=["apple", "ghost"])
    try:
        module.config = MujocoSimModuleConfig(tracked_bodies=["apple", "ghost"])

        class _FakeEngine:
            def get_body_pose(self, body_name: str) -> tuple[np.ndarray, np.ndarray] | None:
                if body_name == "apple":
                    return np.array([0.4, 0.08, 0.17]), np.array([0.0, 0.0, 0.0, 1.0])
                return None

            def disconnect(self) -> None:
                pass

        messages: list[Any] = []
        module._engine = _FakeEngine()
        module.tf.subscribe(messages.append)
        frame = CameraFrame(
            rgb=np.zeros((1, 1, 3), dtype=np.uint8),
            depth=np.ones((1, 1), dtype=np.float32),
            cam_pos=np.array([1.0, 2.0, 3.0]),
            cam_mat=np.eye(3),
            fovy=60.0,
            timestamp=1.0,
        )
        module._publish_tf(10.0, frame)
        module._publish_tf(11.0, frame)
        children = [t.child_frame_id for t in messages[-1].transforms]
        assert children[:3] == [
            "wrist_camera_color_optical_frame",
            "wrist_camera_depth_optical_frame",
            "wrist_camera_link",
        ]
        assert children[3:] == ["apple"]
        apple = messages[-1].transforms[3]
        assert apple.frame_id == "world"
        assert apple.ts == pytest.approx(time.time(), abs=5.0)
        assert np.allclose(apple.translation.to_numpy(), [0.4, 0.08, 0.17])
    finally:
        module.stop()


def test_reset_requests_engine_reset_and_clears_latched_commands() -> None:
    engine = _FakeRespawnEngine()
    hooks = _FakeSimHooks()
    module = MujocoSimModule()
    try:
        module._engine = engine
        module._sim_hooks = hooks

        assert module.reset() is True

        assert engine.reset_requested is True
        assert hooks.cleared is True
    finally:
        module.stop()


def test_respawn_at_uses_ground_height_plus_initial_root_clearance() -> None:
    engine = _FakeRespawnEngine(ground_z=0.08)
    hooks = _FakeSimHooks()
    module = MujocoSimModule()
    try:
        module._engine = engine
        module._sim_hooks = hooks
        module._root_spawn_clearance_z = 0.793

        assert module.respawn_at(2.6, 0.0, yaw=0.25) is True

        assert engine.reset_to_kwargs == {
            "spawn_xy": (2.6, 0.0),
            "spawn_z": pytest.approx(0.873),
            "spawn_yaw": 0.25,
            "wait": True,
        }
        assert hooks.cleared is True
    finally:
        module.stop()


def test_reset_waiters_are_released_when_reset_requests_are_coalesced(tmp_path: Path) -> None:
    robot_xml = tmp_path / "freejoint.xml"
    _write_freejoint_xml(robot_xml)
    engine = MujocoEngine(config_path=robot_xml, headless=True)
    results: list[bool] = []

    def _wait_until_waiters_ready() -> None:
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            with engine._lock:
                if len(engine._reset_done_events) == 2:
                    return
            time.sleep(0.001)
        raise TimeoutError("reset waiters were not registered")

    def _request_reset() -> None:
        results.append(engine.request_reset(wait=True, timeout=1.0))

    def _request_reset_to() -> None:
        results.append(
            engine.request_reset_to(
                spawn_xy=(1.0, 2.0),
                spawn_z=0.5,
                spawn_yaw=0.25,
                wait=True,
                timeout=1.0,
            )
        )

    threads = [
        threading.Thread(target=_request_reset),
        threading.Thread(target=_request_reset_to),
    ]
    for thread in threads:
        thread.start()

    try:
        _wait_until_waiters_ready()
        with engine._lock:
            assert engine._reset_requested
            assert engine._spawn_xy == (1.0, 2.0)
            assert engine._spawn_z == 0.5
            assert engine._spawn_yaw == 0.25
            done_events = engine._reset_done_events
            engine._reset_done_events = []
            engine._reset_requested = False
        for done_event in done_events:
            done_event.set()

        for thread in threads:
            thread.join(timeout=1.0)
            assert not thread.is_alive()
        assert results == [True, True]
    finally:
        engine.disconnect()


def _write_freejoint_xml(path: Path) -> None:
    path.write_text(
        """
<mujoco model="freejoint">
  <option gravity="0 0 0" timestep="0.01"/>
  <worldbody>
    <body name="base" pos="0 0 0.5">
      <freejoint name="floating_base_joint"/>
      <geom name="base_geom" type="sphere" size="0.05" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
""".strip()
    )


def test_observer_follows_base_and_renders_at_its_own_rate(
    tmp_path: Path, mocker: MockerFixture
) -> None:
    robot_xml = tmp_path / "robot.xml"
    _write_freejoint_xml(robot_xml)
    engine = MujocoEngine(
        config_path=robot_xml,
        headless=True,
        observer_camera=CameraConfig(name="observer", fps=10),
    )
    renderer = mocker.Mock(spec=mujoco.Renderer)
    pixels = np.full((2, 3, 3), 127, dtype=np.uint8)
    renderer.render.return_value = pixels
    camera = mujoco.MjvCamera()
    camera.azimuth = 45
    try:
        assert engine.read_observer() is None
        engine.data.qpos[:3] = [1.0, 2.0, 0.8]
        engine._render_observer(10.0, renderer, camera, engine.data)
        np.testing.assert_array_equal(camera.lookat, [1.0, 2.0, 0.8])
        engine.data.qpos[:3] = [3.0, 4.0, 0.6]
        engine._render_observer(10.05, renderer, camera, engine.data)
        assert renderer.render.call_count == 1
        engine._render_observer(10.11, renderer, camera, engine.data)
        assert renderer.render.call_count == 2
        assert camera.azimuth == 45
        np.testing.assert_array_equal(camera.lookat, [3.0, 4.0, 0.6])
        frame = engine.read_observer()
        assert frame is not None
        assert frame[1] == 10.11
        pixels[:] = 0
        np.testing.assert_array_equal(frame[0], np.full((2, 3, 3), 127, dtype=np.uint8))
    finally:
        engine.disconnect()


def test_blocked_render_does_not_block_physics_and_uses_private_snapshot(tmp_path, mocker):
    robot_xml = tmp_path / "robot.xml"
    _write_freejoint_xml(robot_xml)
    rendering = threading.Event()
    release = threading.Event()
    advanced = threading.Event()
    captured = []

    def render(now, renderers, data):
        captured.append((data, data.time))
        rendering.set()
        assert release.wait(5)
        assert data.time == captured[-1][1]

    def after_step(engine):
        if rendering.is_set() and engine.data.time > captured[0][1] + 0.05:
            advanced.set()

    engine = MujocoEngine(
        config_path=robot_xml,
        headless=True,
        cameras=[CameraConfig(name="camera", fps=10)],
        on_after_step=after_step,
    )
    mocker.patch.object(engine, "_init_cameras", return_value={})
    mocker.patch.object(engine, "_render_cameras", side_effect=render)
    try:
        assert engine.connect()
        assert rendering.wait(2)
        assert advanced.wait(2), "physics waited for the renderer"
        assert captured[0][0] is not engine.data
        assert engine.timing()["actual_hz"] > 0
    finally:
        release.set()
        assert engine.disconnect()


def test_sensor_failure_is_reported_without_killing_physics(tmp_path, mocker):
    robot_xml = tmp_path / "robot.xml"
    _write_freejoint_xml(robot_xml)
    advanced = threading.Event()
    engine = MujocoEngine(
        config_path=robot_xml,
        headless=True,
        cameras=[CameraConfig(name="camera")],
        on_after_step=lambda engine: advanced.set(),
    )
    mocker.patch.object(engine, "_init_cameras", side_effect=RuntimeError("render failed"))
    try:
        assert engine.connect()
        assert advanced.wait(2)
        engine._sensing_thread.join(timeout=2)
        assert engine.timing()["sensor_error"] == "render failed"
        assert engine._sim_thread.is_alive()
    finally:
        assert engine.disconnect()


def test_timing_reports_slow_physics_and_stale_feedback(tmp_path, mocker):
    robot_xml = tmp_path / "robot.xml"
    _write_freejoint_xml(robot_xml)
    engine = MujocoEngine(config_path=robot_xml, headless=True)
    mocker.patch("dimos.simulation.engines.mujoco_engine.time.perf_counter", return_value=10.1)
    try:
        assert engine.timing()["actual_hz"] is None
        engine._tick_samples.extend([(10.0, 1.0, 0.001), (10.02, 1.005, 0.01)])
        timing = engine.timing()
        assert timing["actual_hz"] == pytest.approx(50)
        assert timing["real_time_factor"] == pytest.approx(0.25)
        assert timing["physics_age_ms"] == pytest.approx(80)
        assert timing["interval_max_ms"] == pytest.approx(20)
    finally:
        engine.disconnect()


def test_observer_publishes_separately_without_primary_camera_and_skips_duplicate_frames(
    mocker: MockerFixture,
) -> None:
    module = MujocoSimModule(enable_observer=True, enable_color=False, enable_depth=False)
    engine = mocker.Mock(spec=MujocoEngine)
    pixels = np.full((2, 3, 3), 127, dtype=np.uint8)
    engine.read_observer.side_effect = [None, (pixels, 5.0), (pixels, 5.0), (pixels, 6.0)]
    mocker.patch.object(module, "_engine", engine)
    images: list[Image] = []
    primary: list[Image] = []
    try:
        module.observer_image.subscribe(images.append)
        module.color_image.subscribe(primary.append)
        for _ in range(4):
            module._publish_observer()
        assert [image.ts for image in images] == [5.0, 6.0]
        assert [image.frame_id for image in images] == ["", ""]
        np.testing.assert_array_equal(images[0].data, pixels)
        assert primary == []
    finally:
        module.stop()


class _FakeCameraEngine:
    """Serves a fixed list of frame timestamps, then stops the publish loop."""

    connected = True

    def __init__(self, timestamps: list[float], stop_event: threading.Event) -> None:
        self._timestamps = list(timestamps)
        self._stop_event = stop_event

    def read_camera(self, camera_name: str) -> CameraFrame | None:
        if not self._timestamps:
            self._stop_event.set()
            return None
        return CameraFrame(
            rgb=np.zeros((1, 1, 3), dtype=np.uint8),
            depth=np.ones((1, 1), dtype=np.float32),
            cam_pos=np.array([1.0, 2.0, 3.0]),
            cam_mat=np.eye(3),
            fovy=60.0,
            timestamp=self._timestamps.pop(0),
            base_pos=np.array([1.0, 2.0, 2.0]),
            base_mat=np.eye(3),
        )

    def disconnect(self) -> None:
        pass


def _run_publish_loop(module: MujocoSimModule, timestamps: list[float]) -> None:
    module._engine = _FakeCameraEngine(timestamps, module._stop_event)
    module._publish_loop()


def test_publish_loop_stamps_messages_with_frame_timestamp() -> None:
    # Sim time far from wall clock, so a wall-clock stamp cannot pass by accident.
    frame_ts = [1_000_000.0, 1_000_000.5]
    module = MujocoSimModule()
    try:
        module.config = MujocoSimModuleConfig(fps=1000)
        module._camera_info_base = CameraInfo.from_intrinsics(
            width=1, height=1, fx=1.0, fy=1.0, cx=0.5, cy=0.5, frame_id="wrist_camera_color_frame"
        )
        color: list[Any] = []
        depth: list[Any] = []
        tf: list[Any] = []
        info: list[Any] = []
        module.color_image.subscribe(color.append)
        module.depth_image.subscribe(depth.append)
        module.tf.subscribe(tf.append)
        module.camera_info.subscribe(info.append)

        _run_publish_loop(module, frame_ts)
        module._publish_camera_info()

        assert [img.ts for img in color] == frame_ts
        assert [img.ts for img in depth] == frame_ts
        assert [msg.transforms[0].ts for msg in tf] == frame_ts
        # camera_info rides the latest frame's sim clock, not wall time.
        assert info[-1].ts == frame_ts[-1]
        assert module.get_color_camera_info().ts == frame_ts[-1]
        assert module.get_depth_camera_info().ts == frame_ts[-1]
    finally:
        module.stop()


def test_camera_info_falls_back_to_wall_clock_before_first_frame() -> None:
    module = MujocoSimModule()
    try:
        module.config = MujocoSimModuleConfig()
        module._camera_info_base = CameraInfo.from_intrinsics(
            width=1, height=1, fx=1.0, fy=1.0, cx=0.5, cy=0.5, frame_id="wrist_camera_color_frame"
        )
        assert module._latest_frame_ts is None
        assert module._camera_info_ts() == pytest.approx(time.time(), abs=5.0)
    finally:
        module.stop()


@pytest.mark.parametrize("base_ts", [0.05, 1_000_000.0])
def test_publish_loop_pacing_is_independent_of_frame_timestamp_magnitude(base_ts: float) -> None:
    # Pacing must come from the wall clock, not from (wall - sim), which is a huge
    # positive number for any realistic sim clock and drops the loop to render rate.
    fps = 100
    frame_ts = [base_ts + i * 0.05 for i in range(5)]
    module = MujocoSimModule()
    try:
        module.config = MujocoSimModuleConfig(fps=fps)
        start = time.monotonic()
        _run_publish_loop(module, frame_ts)
        elapsed = time.monotonic() - start
        assert elapsed >= (len(frame_ts) - 1) / fps
    finally:
        module.stop()
