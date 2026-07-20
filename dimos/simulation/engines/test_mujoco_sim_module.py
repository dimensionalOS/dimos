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

import numpy as np
import pytest

from dimos.simulation.engines.mujoco_engine import MujocoEngine
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule


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

    class _FakeHooks:
        def post_step(self, engine: Any) -> None:
            assert engine is _FakeEngine
            events.append("joint_state")

    class _FakeShm:
        def write_imu(self, **_: Any) -> None:
            events.append("imu")

        def signal_ready(self, *, num_joints: int) -> None:
            assert num_joints == 2
            events.append("ready")

        def signal_stop(self) -> None:
            pass

        def cleanup(self) -> None:
            pass

    module = MujocoSimModule()
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


def test_reset_waiters_are_released_when_reset_requests_are_coalesced() -> None:
    engine = object.__new__(MujocoEngine)
    engine._lock = threading.Lock()
    engine._reset_requested = False
    engine._reset_done_events = []
    engine._spawn_xy = None
    engine._spawn_z = None
    engine._spawn_yaw = None
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
