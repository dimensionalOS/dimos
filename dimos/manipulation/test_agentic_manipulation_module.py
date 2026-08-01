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

from collections.abc import Callable, Iterator
import inspect
import json
import math
from typing import TypeAlias, cast, get_type_hints

import pytest

from dimos.agents.skill_result import SkillResult
from dimos.manipulation.agentic_manipulation_module import (
    DEFAULT_LIFT_DISTANCE_M,
    DEFAULT_PREGRASP_OFFSET_M,
    AgenticGraspManipulationModule,
    AgenticManipulationModule,
)
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.perception_msgs.RegisteredObject import RegisteredObject
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header

Call: TypeAlias = tuple[str, tuple[object, ...]]
GetStateMethod: TypeAlias = Callable[
    [AgenticManipulationModule, str | None], SkillResult[ManipulationSkillError]
]
MoveToJointsMethod: TypeAlias = Callable[
    [AgenticManipulationModule, str, str | None], SkillResult[ManipulationSkillError]
]
SetMotionSpeedMethod: TypeAlias = Callable[
    [AgenticManipulationModule, float], SkillResult[ManipulationSkillError]
]
GetMotionSpeedMethod: TypeAlias = Callable[
    [AgenticManipulationModule], SkillResult[ManipulationSkillError]
]


def make_pose(x: float, y: float, z: float) -> Pose:
    return Pose(([x, y, z], Quaternion([0.0, 0.0, 0.0, 1.0])))


def make_oriented_pose(x: float, y: float, z: float, orientation: Quaternion) -> Pose:
    return Pose(([x, y, z], orientation))


class FakeManipulationProvider:
    def __init__(self, result: SkillResult[ManipulationSkillError]) -> None:
        self.result = result
        self.calls: list[Call] = []
        self.plan_results: list[bool] = []

    def get_robot_state(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        self.calls.append(("get_robot_state", (robot_name,)))
        return self.result

    def move_to_joints(
        self, joints: str, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]:
        self.calls.append(("move_to_joints", (joints, robot_name)))
        return self.result

    def set_motion_speed(self, speed_scale: float) -> bool:
        self.calls.append(("set_motion_speed", (speed_scale,)))
        return True

    def get_motion_speed(self) -> float:
        self.calls.append(("get_motion_speed", ()))
        return 0.5

    def open_gripper(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        self.calls.append(("open_gripper", (robot_name,)))
        return self.result

    def close_gripper(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        self.calls.append(("close_gripper", (robot_name,)))
        return self.result

    def reset(self) -> SkillResult[ManipulationSkillError]:
        self.calls.append(("reset", ()))
        return self.result

    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        robot_name: str | None = None,
    ) -> SkillResult[ManipulationSkillError]:
        self.calls.append(("move_to_pose", (x, y, z, roll, pitch, yaw, robot_name)))
        return self.result

    def get_ee_pose(self, robot_name: str | None = None) -> Pose | None:
        self.calls.append(("get_ee_pose", (robot_name,)))
        return make_pose(1.0, 2.0, 3.0)

    def plan_to_pose(self, pose: Pose, robot_name: str | None = None) -> bool:
        self.calls.append(("plan_to_pose", (pose, robot_name)))
        if self.plan_results:
            return self.plan_results.pop(0)
        return True

    def go_home(self, robot_name: str | None = None) -> SkillResult[ManipulationSkillError]:
        self.calls.append(("go_home", (robot_name,)))
        return self.result

    def set_gripper(
        self, position: float, robot_name: str | None = None
    ) -> SkillResult[ManipulationSkillError]:
        self.calls.append(("set_gripper", (position, robot_name)))
        return self.result


class FakeSceneRegistration:
    def __init__(self) -> None:
        self.calls: list[Call] = []
        self.objects = [
            RegisteredObject(
                object_id="obj-1",
                name="sphere",
                center=Vector3(0.1, 0.2, 0.3),
                size=Vector3(0.4, 0.5, 0.6),
                frame_id="world",
                ts=1.0,
            )
        ]
        self.name_pointcloud: PointCloud2 | None = PointCloud2(frame_id="world", ts=1.0)
        self.id_pointcloud: PointCloud2 | None = PointCloud2(frame_id="world", ts=2.0)
        self.scene_pointcloud: PointCloud2 | None = PointCloud2(frame_id="world", ts=3.0)

    def set_prompts(self, text: list[str] | None = None, bboxes: object | None = None) -> None:
        del bboxes
        prompt_text = [] if text is None else text
        self.calls.append(("set_prompts", (tuple(prompt_text),)))

    def get_registered_objects(self) -> list[RegisteredObject]:
        self.calls.append(("get_registered_objects", ()))
        return self.objects

    def get_object_pointcloud_by_name(self, name: str) -> PointCloud2 | None:
        self.calls.append(("get_object_pointcloud_by_name", (name,)))
        return self.name_pointcloud

    def get_object_pointcloud_by_object_id(self, object_id: str) -> PointCloud2 | None:
        self.calls.append(("get_object_pointcloud_by_object_id", (object_id,)))
        return self.id_pointcloud

    def get_object_by_object_id(self, object_id: str) -> RegisteredObject | None:
        self.calls.append(("get_object_by_object_id", (object_id,)))
        return next((obj for obj in self.objects if obj.object_id == object_id), None)

    def get_full_scene_pointcloud(
        self,
        exclude_object_id: str | None = None,
        depth_trunc: float = 2,
        voxel_size: float = 0.01,
    ) -> PointCloud2 | None:
        del depth_trunc, voxel_size
        self.calls.append(("get_full_scene_pointcloud", (exclude_object_id,)))
        return self.scene_pointcloud


class FakeGraspGen:
    def __init__(self, grasps: PoseArray) -> None:
        self.calls: list[tuple[str, tuple[PointCloud2 | None, PointCloud2 | None]]] = []
        self.grasps = grasps

    def generate_grasps(
        self, pointcloud: PointCloud2, scene_pointcloud: PointCloud2 | None = None
    ) -> PoseArray:
        self.calls.append(("generate_grasps", (pointcloud, scene_pointcloud)))
        return self.grasps


@pytest.fixture
def skill_result() -> SkillResult[ManipulationSkillError]:
    return SkillResult[ManipulationSkillError].ok("provider result", state="ready")


@pytest.fixture
def provider(skill_result: SkillResult[ManipulationSkillError]) -> FakeManipulationProvider:
    return FakeManipulationProvider(skill_result)


@pytest.fixture
def module(provider: FakeManipulationProvider) -> Iterator[AgenticManipulationModule]:
    agentic_module = AgenticManipulationModule()
    agentic_module._manipulation = provider
    try:
        yield agentic_module
    finally:
        agentic_module.stop()


def test_get_robot_state_delegates_exact_arguments_and_result(
    module: AgenticManipulationModule,
    provider: FakeManipulationProvider,
    skill_result: SkillResult[ManipulationSkillError],
) -> None:
    get_robot_state = cast("GetStateMethod", AgenticManipulationModule.get_robot_state.__wrapped__)

    result = get_robot_state(module, "left_arm")

    assert result is skill_result
    assert provider.calls == [("get_robot_state", ("left_arm",))]


def test_move_to_joints_delegates_exact_arguments_and_result(
    module: AgenticManipulationModule,
    provider: FakeManipulationProvider,
    skill_result: SkillResult[ManipulationSkillError],
) -> None:
    joints = "0.1, -0.5, 1.2, 0.0, 0.3, -0.1"
    move_to_joints = cast(
        "MoveToJointsMethod", AgenticManipulationModule.move_to_joints.__wrapped__
    )

    result = move_to_joints(module, joints, "right_arm")

    assert result is skill_result
    assert provider.calls == [("move_to_joints", (joints, "right_arm"))]


def test_open_gripper_delegates_exact_arguments_and_result(
    module: AgenticManipulationModule,
    provider: FakeManipulationProvider,
    skill_result: SkillResult[ManipulationSkillError],
) -> None:
    open_gripper = cast("GetStateMethod", AgenticManipulationModule.open_gripper.__wrapped__)

    result = open_gripper(module, None)

    assert result is skill_result
    assert provider.calls == [("open_gripper", (None,))]


def test_set_motion_speed_delegates_exact_arguments_and_result(
    module: AgenticManipulationModule,
    provider: FakeManipulationProvider,
) -> None:
    set_motion_speed = cast(
        "SetMotionSpeedMethod", AgenticManipulationModule.set_motion_speed.__wrapped__
    )

    result = set_motion_speed(module, 0.5)

    assert result.success is True
    assert result.message == "Motion speed scale set to 0.50x. Re-plan to apply it."
    assert provider.calls == [("set_motion_speed", (0.5,))]


def test_get_motion_speed_delegates_exact_arguments_and_result(
    module: AgenticManipulationModule,
    provider: FakeManipulationProvider,
) -> None:
    get_motion_speed = cast(
        "GetMotionSpeedMethod", AgenticManipulationModule.get_motion_speed.__wrapped__
    )

    result = get_motion_speed(module)

    assert result.success is True
    assert result.message == "Current motion speed scale is 0.50x."
    assert result.metadata["speed_scale"] == pytest.approx(0.5)
    assert provider.calls == [("get_motion_speed", ())]


def test_close_gripper_delegates_exact_arguments_and_result(
    module: AgenticManipulationModule,
    provider: FakeManipulationProvider,
    skill_result: SkillResult[ManipulationSkillError],
) -> None:
    close_gripper = cast("GetStateMethod", AgenticManipulationModule.close_gripper.__wrapped__)

    result = close_gripper(module, "left_arm")

    assert result is skill_result
    assert provider.calls == [("close_gripper", ("left_arm",))]


def test_decorated_skill_call_preserves_provider_result_semantics(
    module: AgenticManipulationModule,
    provider: FakeManipulationProvider,
    skill_result: SkillResult[ManipulationSkillError],
) -> None:
    result = module.get_robot_state("left_arm")

    assert result is not skill_result
    assert result.success == skill_result.success
    assert result.message == skill_result.message
    assert result.error_code == skill_result.error_code
    assert result.metadata == skill_result.metadata
    assert result.duration_ms >= 0.0
    assert provider.calls == [("get_robot_state", ("left_arm",))]


@pytest.mark.parametrize(
    ("method_name", "expected_params"),
    [
        ("get_robot_state", {"robot_name": str | None}),
        ("move_to_joints", {"joints": str, "robot_name": str | None}),
        ("set_motion_speed", {"speed_scale": float}),
        ("get_motion_speed", {}),
        ("open_gripper", {"robot_name": str | None}),
        ("close_gripper", {"robot_name": str | None}),
    ],
)
def test_skill_methods_have_schema_safe_metadata(
    method_name: str, expected_params: dict[str, type | object]
) -> None:
    method = getattr(AgenticManipulationModule, method_name)
    signature = inspect.signature(method)
    type_hints = get_type_hints(method)

    assert getattr(method, "__skill__", False) is True
    assert inspect.getdoc(method)
    for name, annotation in expected_params.items():
        assert name in signature.parameters
        assert type_hints[name] == annotation
    for name, parameter in signature.parameters.items():
        if name == "self":
            continue
        assert parameter.annotation is not inspect.Parameter.empty
        assert name in expected_params


def test_skill_methods_generate_get_skills_schemas(module: AgenticManipulationModule) -> None:
    skills = {skill.func_name: skill for skill in module.get_skills()}

    assert set(skills) == {
        "close_gripper",
        "get_motion_speed",
        "get_robot_state",
        "move_to_joints",
        "open_gripper",
        "set_motion_speed",
    }
    for method_name in skills:
        schema = json.loads(skills[method_name].args_schema)
        assert schema["type"] == "object"
        assert "properties" in schema
    move_schema = json.loads(skills["move_to_joints"].args_schema)
    assert "joints" in move_schema["properties"]
    assert "robot_name" in move_schema["properties"]
    assert move_schema["required"] == ["joints"]
    speed_schema = json.loads(skills["set_motion_speed"].args_schema)
    assert "speed_scale" in speed_schema["properties"]
    assert speed_schema["required"] == ["speed_scale"]


@pytest.fixture
def grasp_candidates() -> PoseArray:
    return PoseArray(
        header=Header("world"),
        poses=[make_pose(0.4, 0.5, 0.6), make_pose(0.7, 0.8, 0.9)],
    )


@pytest.fixture
def scene_registration() -> FakeSceneRegistration:
    return FakeSceneRegistration()


@pytest.fixture
def grasp_gen(grasp_candidates: PoseArray) -> FakeGraspGen:
    return FakeGraspGen(grasp_candidates)


@pytest.fixture
def grasp_module(
    provider: FakeManipulationProvider,
    scene_registration: FakeSceneRegistration,
    grasp_gen: FakeGraspGen,
) -> Iterator[AgenticGraspManipulationModule]:
    agentic_module = AgenticGraspManipulationModule()
    agentic_module._manipulation = provider
    agentic_module._scene_registration = scene_registration
    agentic_module._grasp_gen = grasp_gen
    try:
        yield agentic_module
    finally:
        agentic_module.stop()


def test_grasp_skill_methods_generate_expected_schemas(
    grasp_module: AgenticGraspManipulationModule,
) -> None:
    skills = {skill.func_name: skill for skill in grasp_module.get_skills()}

    assert set(skills) == {
        "close_gripper",
        "execute_grasp",
        "generate_grasps",
        "get_motion_speed",
        "get_robot_state",
        "go_home",
        "move_along_axis",
        "move_relative",
        "move_to_joints",
        "move_to_pose",
        "open_gripper",
        "scan_objects",
        "set_gripper",
        "set_motion_speed",
    }
    expected_required = {
        "execute_grasp": [],
        "generate_grasps": [],
        "go_home": [],
        "move_along_axis": ["axis", "distance"],
        "move_relative": ["dx", "dy", "dz"],
        "move_to_pose": ["x", "y", "z"],
        "scan_objects": [],
        "set_gripper": ["position"],
    }
    for method_name, required in expected_required.items():
        schema = json.loads(skills[method_name].args_schema)
        assert schema["type"] == "object"
        assert schema.get("required", []) == required


def test_grasp_facade_delegates_scan_generate_motion_and_gripper(
    grasp_module: AgenticGraspManipulationModule,
    provider: FakeManipulationProvider,
    scene_registration: FakeSceneRegistration,
    grasp_gen: FakeGraspGen,
    skill_result: SkillResult[ManipulationSkillError],
) -> None:
    scan_result = grasp_module.scan_objects("sphere")
    generate_result = grasp_module.generate_grasps("sphere", filter_collisions=True)
    move_result = grasp_module.move_to_pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, "arm")
    home_result = grasp_module.go_home("arm")
    gripper_result = grasp_module.set_gripper(0.04, "arm")

    assert scan_result.success is True
    assert generate_result.success is True
    assert move_result.message == skill_result.message
    assert home_result.message == skill_result.message
    assert gripper_result.message == skill_result.message
    assert scene_registration.calls == [
        ("set_prompts", (("sphere",),)),
        ("get_registered_objects", ()),
        ("get_registered_objects", ()),
        ("get_object_pointcloud_by_object_id", ("obj-1",)),
        ("get_full_scene_pointcloud", ("obj-1",)),
    ]
    assert grasp_gen.calls == [
        (
            "generate_grasps",
            (scene_registration.id_pointcloud, scene_registration.scene_pointcloud),
        )
    ]
    assert provider.calls == [
        ("move_to_pose", (1.0, 2.0, 3.0, 0.1, 0.2, 0.3, "arm")),
        ("go_home", ("arm",)),
        ("set_gripper", (0.04, "arm")),
    ]


def test_execute_grasp_uses_cached_candidate_without_regenerate(
    grasp_module: AgenticGraspManipulationModule,
    provider: FakeManipulationProvider,
    scene_registration: FakeSceneRegistration,
    grasp_gen: FakeGraspGen,
) -> None:
    generate_result = grasp_module.generate_grasps("sphere")
    scene_registration.calls.clear()
    grasp_gen.calls.clear()

    result = grasp_module.execute_grasp(1, "arm")

    assert generate_result.success is True
    assert result.success is True
    assert scene_registration.calls == []
    assert grasp_gen.calls == []
    assert provider.calls == [
        (
            "plan_to_pose",
            (make_pose(0.7 - DEFAULT_PREGRASP_OFFSET_M, 0.8, 0.9), "arm"),
        ),
        ("plan_to_pose", (make_pose(0.7, 0.8, 0.9), "arm")),
        ("open_gripper", ("arm",)),
        (
            "move_to_pose",
            (
                0.7 - DEFAULT_PREGRASP_OFFSET_M,
                0.8,
                0.9,
                0.0,
                0.0,
                0.0,
                "arm",
            ),
        ),
        (
            "move_to_pose",
            (0.7, 0.8, 0.9, 0.0, 0.0, 0.0, "arm"),
        ),
        ("get_ee_pose", ("arm",)),
        ("close_gripper", ("arm",)),
        (
            "move_to_pose",
            (
                0.7 - DEFAULT_PREGRASP_OFFSET_M,
                0.8,
                0.9,
                0.0,
                0.0,
                0.0,
                "arm",
            ),
        ),
        ("get_ee_pose", ("arm",)),
        ("move_to_pose", (1.0, 2.0, 3.0 + DEFAULT_LIFT_DISTANCE_M, None, None, None, "arm")),
    ]


def test_execute_grasp_approaches_rotated_candidate_along_local_axis(
    grasp_module: AgenticGraspManipulationModule,
    grasp_gen: FakeGraspGen,
    provider: FakeManipulationProvider,
) -> None:
    grasp_gen.grasps = PoseArray(
        header=Header("world"),
        poses=[
            make_oriented_pose(
                0.4,
                0.5,
                0.6,
                Quaternion.from_euler(Vector3(0.0, 0.0, math.pi / 2)),
            )
        ],
    )
    generate_result = grasp_module.generate_grasps("sphere")

    result = grasp_module.execute_grasp(0, "arm")

    assert generate_result.success is True
    assert result.success is True
    pregrasp_args = provider.calls[3][1]
    final_args = provider.calls[4][1]
    assert pregrasp_args[:3] == pytest.approx((0.4, 0.5 - DEFAULT_PREGRASP_OFFSET_M, 0.6))
    assert final_args[:3] == pytest.approx((0.4, 0.5, 0.6))


def test_execute_grasp_selects_first_plan_feasible_cached_candidate(
    grasp_module: AgenticGraspManipulationModule,
    provider: FakeManipulationProvider,
) -> None:
    grasp_module._cached_grasps = PoseArray(
        header=Header("world"),
        poses=[make_pose(0.1, 0.2, 0.3), make_pose(0.7, 0.8, 0.9)],
    )
    provider.plan_results = [False, True, True]

    result = grasp_module.execute_grasp(0, "arm")

    assert result.success is True
    assert result.metadata["selected_candidate_index"] == 1
    call_names = [call[0] for call in provider.calls]
    assert call_names[:4] == ["plan_to_pose", "reset", "plan_to_pose", "plan_to_pose"]
    assert provider.calls[0] == (
        "plan_to_pose",
        (make_pose(0.1 - DEFAULT_PREGRASP_OFFSET_M, 0.2, 0.3), "arm"),
    )
    assert provider.calls[1] == ("reset", ())
    assert provider.calls[2] == (
        "plan_to_pose",
        (make_pose(0.7 - DEFAULT_PREGRASP_OFFSET_M, 0.8, 0.9), "arm"),
    )
    assert provider.calls[3] == ("plan_to_pose", (make_pose(0.7, 0.8, 0.9), "arm"))
    assert ("move_to_pose", (0.7, 0.8, 0.9, 0.0, 0.0, 0.0, "arm")) in provider.calls


def test_execute_grasp_fails_without_motion_when_no_cached_candidate_is_feasible(
    grasp_module: AgenticGraspManipulationModule,
    provider: FakeManipulationProvider,
) -> None:
    grasp_module._cached_grasps = PoseArray(
        header=Header("world"),
        poses=[make_pose(0.1, 0.2, 0.3), make_pose(0.7, 0.8, 0.9)],
    )
    provider.plan_results = [False, False]

    result = grasp_module.execute_grasp(0, "arm")

    assert result.success is False
    assert result.error_code == "PLANNING_FAILED"
    assert "No cached Grasp candidates" in result.message
    assert provider.calls == [
        (
            "plan_to_pose",
            (make_pose(0.1 - DEFAULT_PREGRASP_OFFSET_M, 0.2, 0.3), "arm"),
        ),
        ("reset", ()),
        (
            "plan_to_pose",
            (make_pose(0.7 - DEFAULT_PREGRASP_OFFSET_M, 0.8, 0.9), "arm"),
        ),
        ("reset", ()),
    ]


def test_generate_grasps_rejects_non_world_candidates_before_caching(
    grasp_module: AgenticGraspManipulationModule,
    grasp_gen: FakeGraspGen,
    provider: FakeManipulationProvider,
) -> None:
    grasp_gen.grasps = PoseArray(
        header=Header("wrist_camera_color_optical_frame"),
        poses=[make_pose(0.1, 0.2, 0.3)],
    )

    generate_result = grasp_module.generate_grasps("sphere")
    execute_result = grasp_module.execute_grasp()

    assert generate_result.success is False
    assert generate_result.error_code == "INVALID_INPUT"
    assert "requires 'world' candidates" in generate_result.message
    assert execute_result.error_code == "INVALID_STATE"
    assert provider.calls == []


def test_execute_grasp_rejects_cached_non_world_candidates_before_motion(
    grasp_module: AgenticGraspManipulationModule,
    provider: FakeManipulationProvider,
) -> None:
    grasp_module._cached_grasps = PoseArray(
        header=Header("wrist_camera_color_optical_frame"),
        poses=[make_pose(0.1, 0.2, 0.3)],
    )

    result = grasp_module.execute_grasp()

    assert result.success is False
    assert result.error_code == "INVALID_INPUT"
    assert "requires 'world' candidates" in result.message
    assert provider.calls == []


def test_execute_grasp_missing_cache_fails_without_scan_or_generate(
    grasp_module: AgenticGraspManipulationModule,
    provider: FakeManipulationProvider,
    scene_registration: FakeSceneRegistration,
    grasp_gen: FakeGraspGen,
) -> None:
    result = grasp_module.execute_grasp()

    assert result.success is False
    assert result.error_code == "INVALID_STATE"
    assert provider.calls == []
    assert scene_registration.calls == []
    assert grasp_gen.calls == []


def test_execute_grasp_invalid_index_fails_before_motion(
    grasp_module: AgenticGraspManipulationModule,
    provider: FakeManipulationProvider,
) -> None:
    grasp_module._cached_grasps = PoseArray(poses=[make_pose(0.1, 0.2, 0.3)])

    result = grasp_module.execute_grasp(3)

    assert result.success is False
    assert result.error_code == "INVALID_INPUT"
    assert provider.calls == []


def test_move_relative_defaults_to_world_frame_and_rejects_unsupported_frame(
    grasp_module: AgenticGraspManipulationModule,
    provider: FakeManipulationProvider,
) -> None:
    world_result = grasp_module.move_relative(0.1, -0.2, 0.3, robot_name="arm")
    bad_frame_result = grasp_module.move_relative(0.1, 0.0, 0.0, frame="tool")

    assert world_result.success is True
    assert bad_frame_result.success is False
    assert bad_frame_result.error_code == "INVALID_INPUT"
    assert provider.calls == [
        ("get_ee_pose", ("arm",)),
        ("move_to_pose", (1.1, 1.8, 3.3, None, None, None, "arm")),
    ]
