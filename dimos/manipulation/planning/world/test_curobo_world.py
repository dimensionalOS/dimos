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

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import replace
from importlib.metadata import PackageNotFoundError
from math import sqrt
from pathlib import Path
import sys
from threading import Event
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any

import numpy as np
import pytest

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.manipulation.planning.world import curobo_world as adapter
from dimos.manipulation.planning.world.curobo_world import CuRoboWorld
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec.validation import PreparedRobotModel


class _Native:
    joint_names = ["elbow", "shoulder"]  # Deliberately not canonical order.

    def __init__(self, robot: dict, scene: dict, cache: dict) -> None:
        self.robot = robot
        self.scene = deepcopy(scene)
        self.cache = cache
        self.update_calls = 0
        self.destroyed = False
        self.fail_update = False

    def update_world(self, scene: dict) -> None:
        self.update_calls += 1
        if self.fail_update:
            self.scene = {}
            raise RuntimeError("update failed after partially clearing scene")
        self.scene = scene

    def destroy(self) -> None:
        self.destroyed = True


@pytest.fixture
def prepared() -> PreparedRobotModel:
    model = RobotModel.from_urdf_string(
        '<robot name="test"><link name="base"/><link name="arm"/><link name="tcp"/>'
        '<joint name="shoulder" type="revolute"><parent link="base"/><child link="arm"/>'
        '<limit lower="-1" upper="1" effort="1" velocity="1"/></joint>'
        '<joint name="elbow" type="revolute"><parent link="arm"/><child link="tcp"/>'
        '<limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>'
    ).with_default_joint_acceleration_limit(2.0)
    return prepare_robot_model(
        RobotModelConfig(
            model=model,
            joint_names=["shoulder", "elbow"],
            base_link="base",
            planning_groups=[
                PlanningGroupDefinition(
                    name="arm", joint_names=("elbow", "shoulder"), base_link="base", tip_link="tcp"
                )
            ],
        )
    )


@pytest.fixture
def scene_world(prepared: PreparedRobotModel, monkeypatch: pytest.MonkeyPatch):
    builds: list[PreparedRobotModel] = []
    runtime: list[_Native] = []
    robot = {"kinematics": {"cspace": {"default_joint_position": [0.2, 0.3]}}}

    def build(model: PreparedRobotModel) -> dict:
        builds.append(model)
        return robot

    def create(robot: dict, scene: dict, cache: dict, device: Any) -> _Native:
        native = _Native(robot, scene, cache)
        runtime.append(native)
        return native

    monkeypatch.setattr(adapter, "_load_device", lambda _: SimpleNamespace(to_device=np.asarray))
    monkeypatch.setattr(adapter, "build_curobo_model", build)
    monkeypatch.setattr(adapter, "_create_native_planner", create)
    monkeypatch.setitem(
        sys.modules, "curobo.scene", SimpleNamespace(Scene=SimpleNamespace(create=deepcopy))
    )
    monkeypatch.setitem(
        sys.modules,
        "curobo.types",
        SimpleNamespace(
            JointState=SimpleNamespace(
                from_position=lambda q, **kw: SimpleNamespace(position=q, **kw)
            )
        ),
    )
    world = CuRoboWorld()
    assert isinstance(world, WorldSpec)
    world.load_model(prepared)
    world.finalize()
    yield world, runtime, builds
    world.close()


def _box(name: str) -> Obstacle:
    return Obstacle(name, ObstacleType.BOX, PoseStamped(), (0.4, 0.5, 0.6))


def test_scene_conversion_keeps_all_cells_and_applies_pose(tmp_path: Path) -> None:
    box = _box("box")
    sky = Obstacle("sphere", ObstacleType.SPHERE, PoseStamped(), (0.12,))
    rod = Obstacle("cylinder", ObstacleType.CYLINDER, PoseStamped(), (0.1, 0.4))
    path = tmp_path / "mesh.obj"
    path.touch()
    mesh = Obstacle("mesh", ObstacleType.MESH, PoseStamped(), mesh_path=str(path))
    rotation = sqrt(0.5)
    occupied = Obstacle(
        "map",
        ObstacleType.OCTREE,
        PoseStamped(position=[1, 2, 3], orientation=[0, 0, rotation, rotation]),
        points=np.array([[0.1, 0, 0], [0, 0.2, 0], [0, 0, 0.3]]),
        octree_resolution=0.05,
    )
    native = adapter._scene_model([box, sky, rod, mesh, occupied])
    assert native["cuboid"]["box"]["dims"] == [0.4, 0.5, 0.6]
    assert native["capsule"]["sphere"]["radius"] == 0.12
    assert (native["cylinder"]["cylinder"]["radius"], native["cylinder"]["cylinder"]["height"]) == (
        0.1,
        0.4,
    )
    assert native["mesh"]["mesh"]["file_path"] == str(path)
    cells = [value for name, value in native["cuboid"].items() if name != "box"]
    np.testing.assert_allclose(
        [cell["pose"][:3] for cell in cells], [[1, 2.1, 3], [0.8, 2, 3], [1, 2, 3.3]]
    )
    for cell in cells:
        np.testing.assert_allclose(cell["pose"][3:], [rotation, 0, 0, rotation])
        assert cell["dims"] == [0.05] * 3


def test_native_planner_only_removes_trajopt_external_constraint(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    stock = {
        "rollout": {
            "constraint_cfg": {
                "cspace_cfg": {"weight": [1]},
                "self_collision_cfg": {},
                "scene_collision_cfg": {},
            },
            "convergence_cfg": {"tool_pose_cfg": {}},
        },
        "scene_collision_checker_cfg": {"cache": None},
    }
    original = deepcopy(stock)
    calls: list[dict[str, Any]] = []

    def create(**kwargs: Any) -> dict:
        calls.append(kwargs)
        return kwargs

    monkeypatch.setitem(
        sys.modules, "curobo.config_io", SimpleNamespace(load_yaml=lambda path: stock)
    )
    monkeypatch.setitem(
        sys.modules, "curobo.content", SimpleNamespace(get_task_configs_path=lambda: Path("stock"))
    )
    monkeypatch.setitem(
        sys.modules,
        "curobo.motion_planner",
        SimpleNamespace(MotionPlanner=dict, MotionPlannerCfg=SimpleNamespace(create=create)),
    )
    scene = adapter._scene_model([_box("block")])
    planner = adapter._create_native_planner({}, scene, {"cuboid": 1}, "fake device")
    assert planner == calls[0]
    assert planner["scene_model"] == scene and planner["self_collision_check"] is True
    assert "trajopt_optimizer_configs" not in planner and "ik_metrics_rollout" not in planner
    expected = deepcopy(stock)
    del expected["rollout"]["constraint_cfg"]["scene_collision_cfg"]
    assert planner["trajopt_metrics_rollout"] == expected
    assert stock == original


def test_pinned_curobo_cpu_scene_and_stock_rollouts() -> None:
    try:
        installed = adapter.version("nvidia-curobo")
    except PackageNotFoundError:
        pytest.skip("optional nvidia-curobo distribution is not installed")
        return
    if installed != adapter.CUROBO_VERSION:
        pytest.skip(f"requires pinned CuRobo V2, found {installed}")

    from curobo.config_io import load_yaml
    from curobo.content import get_task_configs_path
    from curobo.scene import Scene

    stock_path = get_task_configs_path()
    stock = load_yaml(str(stock_path / "metrics_base.yml"))
    modified = adapter._trajopt_metrics(stock)["rollout"]["constraint_cfg"]
    assert "scene_collision_cfg" in stock["rollout"]["constraint_cfg"]
    assert "scene_collision_cfg" not in modified
    assert {"cspace_cfg", "self_collision_cfg"} <= modified.keys()
    optimizer = load_yaml(str(stock_path / "trajopt/lbfgs_bspline_trajopt.yml"))
    assert "scene_collision_cfg" in optimizer["rollout"]["constraint_cfg"]

    sphere = Obstacle("sphere", ObstacleType.SPHERE, PoseStamped(position=[1, 2, 3]), (0.2,))
    scene = Scene.create(adapter._scene_model([_box("box"), sphere]))
    assert len(scene) == 2
    assert scene.cuboid[0].dims == [0.4, 0.5, 0.6]
    capsule = scene.capsule[0]
    assert capsule.base == capsule.tip == [0, 0, 0]
    assert capsule.radius == 0.2 and capsule.pose == [1, 2, 3, 1, 0, 0, 0]


def test_obstacle_lifecycle_snapshots_and_cache_reuse(scene_world) -> None:
    world, runtime, builds = scene_world
    obstacle = _box("block")
    assert world.add_obstacle(obstacle) == "block"
    assert world.add_obstacle(obstacle) is None
    assert world.add_obstacle(_box("")) is None
    obstacle.pose.position.x = 99.0
    world.get_obstacles()[0].pose.position.x = 98.0
    with world.native_context() as native:
        assert native is runtime[1] and runtime[0].destroyed
        assert native.scene["cuboid"]["block"]["pose"][0] == 0.0
        assert native.cache == {"cuboid": 1}
    assert world.update_obstacle_pose("block", PoseStamped(position=[2, 0, 0]))
    with world.native_context() as native:
        assert native is runtime[1] and native.update_calls == 1
        assert native.scene["cuboid"]["block"]["pose"][0] == 2.0
    assert world.update_obstacle(replace(_box("block"), dimensions=(1.0, 2.0, 3.0)))
    with world.native_context() as native:
        assert native.scene["cuboid"]["block"]["dims"] == [1.0, 2.0, 3.0]
    assert world.remove_obstacle("block")
    assert not world.remove_obstacle("block")
    assert not world.update_obstacle(_box("block"))
    assert not world.update_obstacle_pose("block", PoseStamped())
    with world.native_context() as native:
        assert native.scene == {} and native.update_calls == 3
    world.add_obstacle(_box("next"))
    world.clear_obstacles()
    with world.native_context() as native:
        assert native.scene == {}
    assert world.get_obstacles() == [] and len(builds) == 1


def test_invalid_replacement_keeps_current_scene(
    scene_world, monkeypatch: pytest.MonkeyPatch
) -> None:
    world, _, _ = scene_world
    world.add_obstacle(_box("block"))
    with pytest.raises(ValueError, match="finite and positive"):
        world.update_obstacle(replace(_box("block"), dimensions=(1.0, np.inf, 3.0)))
    with pytest.raises(ValueError, match="touch_links"):
        world.update_obstacle(replace(_box("block"), touch_links=("tcp",)))
    assert world.get_obstacles() == [_box("block")]
    monkeypatch.setattr(adapter, "MAX_VOXEL_CUBOIDS", 2)
    cells = Obstacle(
        "map", ObstacleType.OCTREE, PoseStamped(), points=np.ones((2, 3)), octree_resolution=0.04
    )
    assert world.add_obstacle(cells) == "map"
    with pytest.raises(ValueError, match="at most 2 occupied cells"):
        world.add_obstacle(replace(cells, name="overflow"))
    assert [obs.name for obs in world.get_obstacles()] == ["block", "map"]
    assert world.update_obstacle(replace(cells, points=np.zeros((1, 3))))
    with world.native_context() as native:
        assert len(native.scene["cuboid"]) == 2
        assert native.scene["cuboid"]["__dimos_voxel:map:0"]["pose"][:3] == [0, 0, 0]


def test_native_update_failure_discards_partial_cache(scene_world) -> None:
    world, runtime, _ = scene_world
    world.add_obstacle(_box("block"))
    with world.native_context() as native:
        native.fail_update = True
    world.update_obstacle_pose("block", PoseStamped(position=[1, 0, 0]))
    with pytest.raises(RuntimeError, match="partially clearing"):
        with world.native_context():
            pytest.fail("The partial native scene escaped")
    assert runtime[1].destroyed
    with world.native_context() as native:
        assert native is runtime[2]
        assert native.scene["cuboid"]["block"]["pose"][0] == 1.0


def test_mesh_replacement_rebuilds_native_name_cache(scene_world, tmp_path: Path) -> None:
    world, runtime, builds = scene_world
    first, second = tmp_path / "first.obj", tmp_path / "second.obj"
    first.touch()
    second.touch()
    mesh = Obstacle("mesh", ObstacleType.MESH, PoseStamped(), mesh_path=str(first))
    world.add_obstacle(mesh)
    with world.native_context() as native:
        assert native.scene["mesh"]["mesh"]["file_path"] == str(first)
    world.update_obstacle(replace(mesh, mesh_path=str(second)))
    with world.native_context() as native:
        assert runtime[1].destroyed and native is runtime[2]
        assert native.scene["mesh"]["mesh"]["file_path"] == str(second)
    assert len(builds) == 1


def test_hard_checks_use_canonical_limits_and_self_collision_only(
    scene_world, monkeypatch: pytest.MonkeyPatch
) -> None:
    world, runtime, _ = scene_world
    assert not world.is_ready()
    assert world.get_joint_state(world.get_live_context()).position == [0.2, 0.3]
    world.sync_from_joint_state(JointState(name=["elbow", "shoulder"], position=[-0.4, 0.4]))
    assert world.is_ready()
    with world.scratch_context() as scratch:
        assert world.get_joint_state(scratch).position == [0.4, -0.4]
        world.set_joint_state(scratch, JointState(position=[-0.4, 0.0]))
        assert world.get_joint_state(world.get_live_context()).position == [0.4, -0.4]
    calls: list[np.ndarray] = []

    def collision(q: np.ndarray) -> np.ndarray:
        calls.append(q.copy())
        # Native order is elbow, shoulder. Model one interior collision.
        return np.where(np.abs(q[..., 1]) < 0.01, 1.0, 0.0)

    checker = SimpleNamespace(
        kinematics=SimpleNamespace(joint_names=["elbow", "shoulder"]),
        get_kinematics=lambda q: SimpleNamespace(robot_spheres=q),
        get_self_collision=collision,
    )
    monkeypatch.setattr(adapter, "_create_self_checker", lambda robot, device: checker)
    world.add_obstacle(_box("external"))
    assert world.check_config_collision_free(JointState(position=[0.4, -0.2]))
    assert calls[0].tolist() == [[[-0.2, 0.4]]]
    assert not world.check_config_collision_free(JointState(position=[1.2, 0.0]))
    assert len(calls) == 1
    assert not world.check_edge_collision_free(
        JointState(position=[-0.2, 0.0]), JointState(position=[0.2, 0.0]), 0.1
    )
    np.testing.assert_allclose(calls[1][:, 0, 1], [-0.2, -0.1, 0.0, 0.1, 0.2], atol=1e-14)
    assert len(runtime) == 1  # The hard checks did not even install the queued scene.


def test_native_context_serializes_obstacle_mutations(scene_world) -> None:
    world, _, _ = scene_world
    started, entered = Event(), Event()

    def update() -> None:
        started.set()
        world.clear_obstacles()
        entered.set()

    world.add_obstacle(_box("block"))
    with ThreadPoolExecutor(max_workers=1) as executor:
        with world.native_context() as native:
            future = executor.submit(update)
            assert started.wait(timeout=2)
            assert not entered.wait(timeout=0.1)
            assert "block" in native.scene["cuboid"]
        future.result(timeout=2)
    with world.native_context() as native:
        assert native.scene == {}


def test_optional_dependency_and_version_are_checked_at_finalize(
    prepared: PreparedRobotModel, monkeypatch: pytest.MonkeyPatch
) -> None:
    def missing(name: str) -> str:
        raise PackageNotFoundError(name)

    monkeypatch.setattr(adapter, "version", missing)
    world = CuRoboWorld()
    world.load_model(prepared)
    with pytest.raises(ImportError, match="requires nvidia-curobo==0.9.0"):
        world.finalize()
    monkeypatch.setattr(adapter, "version", lambda _: "0.8.0")
    with pytest.raises(ImportError, match="got 0.8.0"):
        world.finalize()
    assert not world.is_finalized
