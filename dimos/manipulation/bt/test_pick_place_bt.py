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

"""Unit tests for pick-and-place behavior-tree orchestration."""

from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any

import py_trees
from py_trees.common import Status

from dimos.manipulation.bt import actions
from dimos.manipulation.bt.conditions import GripperHasObject
from dimos.manipulation.bt.trees import RetryOnFailure, build_pick_tree, build_place_tree
from dimos.msgs.geometry_msgs import Vector3


@dataclass
class _Config:
    bt_tick_rate_hz: float = 1000.0
    bt_max_pick_attempts: int = 3
    bt_max_rescan_attempts: int = 2
    bt_enable_graspgen: bool = True
    bt_scan_duration: float = 0.0
    bt_execution_timeout: float | None = None
    bt_gripper_open_position: float = 0.85
    bt_gripper_close_position: float = 0.0
    bt_gripper_grasp_threshold: float = 0.005
    bt_gripper_grasp_max_open_fraction: float = 0.5
    bt_gripper_grasp_max_open_position: float | None = None
    bt_gripper_grasp_open_reference: float | None = None
    bt_gripper_grasp_min_closure: float | None = None
    bt_gripper_settle_time: float = 0.0
    bt_lift_height: float = 0.10
    bt_min_grasp_z: float = 0.05
    bt_max_grasp_distance: float = 0.9
    bt_max_approach_angle: float = 1.05
    pre_grasp_offset: float = 0.10


class _FakeModule:
    def __init__(self) -> None:
        self.config = _Config()
        self._detection_snapshot = [
            SimpleNamespace(name="cup", object_id="obj-1", center=Vector3(0.3, 0.1, 0.2))
        ]
        self._last_pick_position = None
        self.gripper_position: float | None = 0.03

    def _find_object_in_detections(self, object_name: str, object_id: str | None = None) -> Any:
        for det in self._detection_snapshot:
            if object_id == det.object_id or object_name in det.name:
                return det
        return None

    def get_rpc_calls(self, _name: str) -> Any:
        raise RuntimeError("RPC unavailable in unit test")

    def generate_grasps(self, *_args: Any, **_kwargs: Any) -> Any:
        return None

    def get_gripper(self, _robot_name: str | None = None) -> float | None:
        return self.gripper_position


class _FailOnce(py_trees.behaviour.Behaviour):
    def __init__(self) -> None:
        super().__init__(name="FailOnce")
        self.calls = 0

    def update(self) -> Status:
        self.calls += 1
        return Status.FAILURE if self.calls == 1 else Status.SUCCESS


def test_retry_on_failure_retries_child() -> None:
    child = _FailOnce()
    root = RetryOnFailure("Retry", child=child, max_attempts=2)
    tree = py_trees.trees.BehaviourTree(root=root)
    tree.setup()

    tree.tick()
    assert root.status == Status.RUNNING

    tree.tick()
    assert root.status == Status.SUCCESS
    assert child.calls == 2


def test_pick_tree_builds() -> None:
    root = build_pick_tree(
        module=_FakeModule(),  # type: ignore[arg-type]
        object_name="cup",
        object_id=None,
        robot_name=None,
    )

    assert root.name == "Pick"
    assert [child.name for child in root.children] == [
        "EnsureReady",
        "PickWithRescan",
        "StorePickPosition",
        "SetResult",
    ]


def test_pick_tree_can_disable_graspgen() -> None:
    module = _FakeModule()
    module.config.bt_enable_graspgen = False

    root = build_pick_tree(
        module=module,  # type: ignore[arg-type]
        object_name="cup",
        object_id=None,
        robot_name=None,
    )

    names = [node.name for node in root.iterate()]
    assert "GraspGenCandidates" not in names
    assert "GetObjectPointcloud" not in names
    assert "GenerateGrasps" not in names
    assert "GenerateGraspCandidates" in names


def test_place_tree_builds() -> None:
    root = build_place_tree(
        module=_FakeModule(),  # type: ignore[arg-type]
        x=0.4,
        y=0.0,
        z=0.2,
        robot_name=None,
    )

    assert root.name == "Place"
    assert root.children[0].name == "ComputePlacePose"
    assert root.children[-1].name == "SetResult"


def test_graspgen_failure_falls_back_to_heuristic() -> None:
    module = _FakeModule()
    bb = py_trees.blackboard.Client(name="FallbackTestSetup")
    for key in ("target_object", "grasp_candidates", "grasp_index", "grasp_source"):
        bb.register_key(key=key, access=py_trees.common.Access.WRITE)
    bb.target_object = module._detection_snapshot[0]
    bb.grasp_candidates = []
    bb.grasp_index = 0
    bb.grasp_source = ""

    selector = py_trees.composites.Selector(
        "GenerateGraspCandidates",
        memory=True,
        children=[
            py_trees.composites.Sequence(
                "GraspGenCandidates",
                memory=True,
                children=[
                    actions.GetObjectPointcloud(
                        "GetObjectPointcloud", module  # type: ignore[arg-type]
                    ),
                    actions.GenerateGrasps("GenerateGrasps", module),  # type: ignore[arg-type]
                ],
            ),
            actions.GenerateHeuristicGrasps(
                "HeuristicGrasps", module  # type: ignore[arg-type]
            ),
        ],
    )
    tree = py_trees.trees.BehaviourTree(root=selector)
    tree.setup()
    tree.tick()

    assert selector.status == Status.SUCCESS
    assert bb.grasp_source == "heuristic"
    assert len(bb.grasp_candidates) >= 1


def test_gripper_check_accepts_partial_closure() -> None:
    module = _FakeModule()
    module.gripper_position = 0.03
    condition = GripperHasObject("VerifyGrasp", module)  # type: ignore[arg-type]
    tree = py_trees.trees.BehaviourTree(root=condition)
    tree.setup()
    tree.tick()

    assert condition.status == Status.SUCCESS


def test_gripper_check_rejects_fully_open_gripper() -> None:
    module = _FakeModule()
    module.gripper_position = module.config.bt_gripper_open_position
    condition = GripperHasObject("VerifyGrasp", module)  # type: ignore[arg-type]
    tree = py_trees.trees.BehaviourTree(root=condition)
    tree.setup()
    tree.tick()

    assert condition.status == Status.FAILURE


def test_gripper_check_rejects_empty_closed_gripper() -> None:
    module = _FakeModule()
    module.gripper_position = 0.0
    condition = GripperHasObject("VerifyGrasp", module)  # type: ignore[arg-type]
    tree = py_trees.trees.BehaviourTree(root=condition)
    tree.setup()
    tree.tick()

    assert condition.status == Status.FAILURE


def test_gripper_check_uses_min_closure_from_open_reference() -> None:
    module = _FakeModule()
    module.config.bt_gripper_open_position = 0.85
    module.config.bt_gripper_grasp_max_open_fraction = 1.0
    module.config.bt_gripper_grasp_open_reference = 0.85
    module.config.bt_gripper_grasp_min_closure = 0.10

    module.gripper_position = 0.80
    condition = GripperHasObject("VerifyGrasp", module)  # type: ignore[arg-type]
    tree = py_trees.trees.BehaviourTree(root=condition)
    tree.setup()
    tree.tick()
    assert condition.status == Status.FAILURE

    module.gripper_position = 0.74
    condition = GripperHasObject("VerifyGrasp", module)  # type: ignore[arg-type]
    tree = py_trees.trees.BehaviourTree(root=condition)
    tree.setup()
    tree.tick()
    assert condition.status == Status.SUCCESS
