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

from collections.abc import Iterator
from pathlib import Path
import threading
import time
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.simulation.engines.mujoco_engine import CameraConfig, CameraFrame, MujocoEngine
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule, MujocoSimModuleConfig


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


def _write_scene_xml(path: Path) -> None:
    path.write_text(
        """
<mujoco model="scene">
  <option timestep="0.02"/>
  <worldbody>
    <geom name="static_scene_box" type="box" pos="2 0 0.5" size="0.2 0.2 0.5"/>
  </worldbody>
</mujoco>
""".strip()
    )


def _write_robot_xml(path: Path) -> None:
    path.write_text(
        """
<mujoco model="robot">
  <option timestep="0.005"/>
  <worldbody>
    <body name="base" pos="0 0 0">
      <freejoint name="floating_base_joint"/>
      <geom name="base_geom" type="sphere" size="0.05" mass="1.0"/>
      <site name="imu_site" pos="0 0 0"/>
      <body name="link" pos="0 0 0.1">
        <joint name="hinge" type="hinge" axis="0 0 1"/>
        <geom name="link_geom" type="sphere" size="0.04" mass="1.0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="hinge_motor" joint="hinge"/>
  </actuator>
</mujoco>
""".strip()
    )


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


def _scene_entity(entity_id: str) -> dict[str, object]:
    return {
        "id": entity_id,
        "spawn": "initial",
        "descriptor": {
            "entity_id": entity_id,
            "kind": "dynamic",
            "mass": 1.0,
            "shape_hint": "box",
            "extents": [0.2, 0.2, 0.2],
        },
        "initial_pose": {
            "x": 1.0,
            "y": 0.0,
            "z": 0.1,
            "qw": 1.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
        },
    }


def _write_hull_obj(path: Path) -> None:
    path.write_text(
        """
v 0 0 0
v 0.1 0 0
v 0 0.1 0
v 0 0 0.1
f 1 2 3
f 1 2 4
f 1 3 4
f 2 3 4
""".strip()
    )


def _mesh_scene_entity(entity_id: str, hull_path: Path) -> dict[str, object]:
    entity = _scene_entity(entity_id)
    descriptor = dict(entity["descriptor"])  # type: ignore[arg-type]
    descriptor["shape_hint"] = "mesh"
    descriptor["extents"] = []
    entity["descriptor"] = descriptor
    entity["collision_paths"] = [str(hull_path)]
    return entity


@pytest.mark.mujoco
def test_compose_model_attaches_robot_before_scene_entities(tmp_path: Path) -> None:
    import mujoco

    scene_xml = tmp_path / "scene.xml"
    robot_xml = tmp_path / "robot.xml"
    _write_scene_xml(scene_xml)
    _write_robot_xml(robot_xml)

    module = MujocoSimModule(
        scene_xml=scene_xml,
        robot_mjcf=robot_xml,
        scene_entities=[_scene_entity("chair_000")],
        spawn_xy=(0.25, -0.5),
        spawn_z=0.8,
    )
    try:
        model = module._compose_model()

        assert model.opt.timestep == pytest.approx(0.005)
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "static_scene_box") >= 0
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "entity:chair_000") >= 0

        free_joints: list[tuple[str, int]] = []
        for joint_id in range(model.njnt):
            if int(model.jnt_type[joint_id]) != int(mujoco.mjtJoint.mjJNT_FREE):
                continue
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or ""
            free_joints.append((name, int(model.jnt_qposadr[joint_id])))

        robot_free = next(item for item in free_joints if item[0].endswith("floating_base_joint"))
        entity_free = next(item for item in free_joints if item[0] == "entity:chair_000:free")
        assert robot_free[1] == 0
        assert entity_free[1] > robot_free[1]

        engine = MujocoEngine(config_path=robot_xml, headless=True, model=model)
        assert engine.model is model
        assert engine.root_qpos_adr == 0
        assert any(name.endswith("hinge") for name in engine.joint_names)
    finally:
        module.stop()


@pytest.mark.mujoco
def test_compose_model_reuses_entity_mesh_assets(tmp_path: Path) -> None:
    import mujoco

    scene_xml = tmp_path / "scene.xml"
    robot_xml = tmp_path / "robot.xml"
    hull_obj = tmp_path / "shared_hull.obj"
    _write_scene_xml(scene_xml)
    _write_robot_xml(robot_xml)
    _write_hull_obj(hull_obj)

    module = MujocoSimModule(
        scene_xml=scene_xml,
        robot_mjcf=robot_xml,
        scene_entities=[
            _mesh_scene_entity("box_000", hull_obj),
            _mesh_scene_entity("box_001", hull_obj),
        ],
        spawn_xy=(0.0, 0.0),
    )
    try:
        model = module._compose_model()

        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "entity:box_000") >= 0
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "entity:box_001") >= 0
        assert model.nmesh == 1
    finally:
        module.stop()


@pytest.fixture
def freejoint_engine(tmp_path: Path) -> Iterator[MujocoEngine]:
    robot_xml = tmp_path / "freejoint.xml"
    _write_freejoint_xml(robot_xml)
    engine = MujocoEngine(config_path=robot_xml, headless=True)
    assert engine.connect() is True
    try:
        yield engine
    finally:
        engine.disconnect()


@pytest.mark.mujoco
def test_sim_loop_holds_real_time_with_no_cameras(tmp_path: Path) -> None:
    """The loop must pace against an absolute deadline, not per iteration.

    time.sleep() overshoots - asking for 5 ms typically returns after ~6 - so a
    pacer that sleeps `dt - elapsed` and starts the next iteration from
    scratch throws that overshoot away every step and tops out near RTF 0.83
    with nothing rendering at all. Only the accumulated `next_step_at`
    schedule repays it.

    Note this asserts against engine.data.time, MuJoCo's own clock. It cannot
    be written against the odom message timestamp: that is stamped with
    time.time(), so odom-vs-wall is wall-vs-wall and reads 1.000 no matter how
    far behind the simulation actually is.
    """
    robot_xml = tmp_path / "freejoint.xml"
    _write_freejoint_xml(robot_xml)
    engine = MujocoEngine(config_path=robot_xml, headless=True)
    assert engine.connect() is True
    try:
        time.sleep(0.5)
        t0, sim0 = time.monotonic(), float(engine.data.time)
        time.sleep(3.0)
        rtf = (float(engine.data.time) - sim0) / (time.monotonic() - t0)
    finally:
        engine.disconnect()
    assert rtf > 0.95, f"sim loop cannot hold real time even when idle (RTF={rtf:.3f})"


@pytest.mark.mujoco
def test_slow_renders_cost_frame_rate_not_simulated_time(tmp_path: Path) -> None:
    """Cameras render inline on the sim thread, so a slow renderer (GPU
    contention: a screen recorder, another encoder) must be allowed to cost
    frames, never simulated time. Before the lag governor, a render that
    overran the step budget silently pushed the simulation clock behind the
    wall clock - the user-visible symptom being that time ran in slow motion."""
    robot_xml = tmp_path / "camera.xml"
    robot_xml.write_text(
        """
<mujoco model="camera">
  <option gravity="0 0 0" timestep="0.005"/>
  <worldbody>
    <camera name="cam" pos="0 -1 0.5" xyaxes="1 0 0 0 0 1"/>
    <body name="base" pos="0 0 0.5">
      <freejoint name="floating_base_joint"/>
      <geom name="base_geom" type="sphere" size="0.05" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
""".strip()
    )
    engine = MujocoEngine(
        config_path=robot_xml,
        headless=True,
        cameras=[CameraConfig(name="cam", width=32, height=32, fps=60.0, render_depth=False)],
    )
    assert engine.connect() is True
    try:
        # Each render sleeps far longer than the 5 ms step budget: a renderer
        # roughly 8x slower than real, which is past anything the loop could
        # absorb by rendering every frame.
        renders = 0
        real_render = engine._render_cameras

        def _slow_render(now: float, states: dict[str, Any]) -> None:
            nonlocal renders
            renders += 1
            time.sleep(0.040)
            real_render(now, states)

        engine._render_cameras = _slow_render  # type: ignore[method-assign]

        time.sleep(0.5)  # let the loop settle before sampling
        t0, sim0 = time.monotonic(), float(engine.data.time)
        time.sleep(4.0)
        wall = time.monotonic() - t0
        sim = float(engine.data.time) - sim0
    finally:
        engine.disconnect()

    rtf = sim / wall
    # The clock is protected: simulated time keeps up with the wall clock.
    assert rtf > 0.9, f"simulated time fell behind the wall clock (RTF={rtf:.3f})"
    # And the cost was paid in frames instead - the governor skipped renders
    # rather than letting them run every cycle (60 fps over ~4 s would be
    # ~240; a renderer this slow cannot sustain that and must be throttled).
    assert 0 < renders < 200, f"expected renders to be throttled, got {renders}"


@pytest.mark.mujoco
@pytest.mark.parametrize("render_depth", [True, False])
def test_camera_render_depth_controls_the_second_render(tmp_path: Path, render_depth: bool) -> None:
    """Depth is a whole second render of the scene, so a camera whose depth
    nobody reads (a video feed) must be able to skip it and still deliver rgb."""
    robot_xml = tmp_path / "camera.xml"
    robot_xml.write_text(
        """
<mujoco model="camera">
  <option gravity="0 0 0" timestep="0.01"/>
  <worldbody>
    <camera name="cam" pos="0 -1 0.5" xyaxes="1 0 0 0 0 1"/>
    <body name="base" pos="0 0 0.5">
      <freejoint name="floating_base_joint"/>
      <geom name="base_geom" type="sphere" size="0.05" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
""".strip()
    )
    engine = MujocoEngine(
        config_path=robot_xml,
        headless=True,
        cameras=[
            CameraConfig(name="cam", width=16, height=16, fps=1000.0, render_depth=render_depth)
        ],
    )
    assert engine.connect() is True
    try:
        deadline = time.monotonic() + 5.0
        frame = None
        while frame is None and time.monotonic() < deadline:
            frame = engine.read_camera("cam")
            if frame is None:
                time.sleep(0.01)
        assert frame is not None, "camera never produced a frame"
        assert frame.rgb.shape == (16, 16, 3)  # rgb is unaffected either way
        if render_depth:
            assert frame.depth is not None
            assert frame.depth.shape == (16, 16)
        else:
            assert frame.depth is None
    finally:
        engine.disconnect()


@pytest.mark.mujoco
def test_engine_request_reset_to_applies_pose_in_sim_loop(freejoint_engine: MujocoEngine) -> None:
    assert freejoint_engine.request_reset_to(
        spawn_xy=(1.25, -0.5),
        spawn_z=0.9,
        spawn_yaw=0.3,
        wait=True,
    )
    pose = freejoint_engine.get_root_pose()
    assert pose is not None
    position, quat_xyzw = pose
    np.testing.assert_allclose(position, [1.25, -0.5, 0.9], atol=1e-8)
    np.testing.assert_allclose(
        quat_xyzw,
        [0.0, 0.0, np.sin(0.15), np.cos(0.15)],
        atol=1e-8,
    )


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
