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

"""Headless acceptance tests for the six-object xArm room scene."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import dataclass
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any, TypeVar, cast

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from dimos.core.global_config import global_config
from dimos.core.rpc_client import RPCClient
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.perception.experimental.object_scene_registration import (
    ObjectSceneRegistrationModule,
)
from dimos.robot.manipulators.xarm.blueprints.simulation import (
    XARM_ROOM_PROMPTS,
    XARM_ROOM_SCENE_PATH,
)

pytestmark = [pytest.mark.mujoco, pytest.mark.timeout(1800)]

POSITION_TOLERANCE_M = 0.02
EXPECTED_OBJECT_COUNT = 6
DESIGNATED_TARGET = "can"
PLACE_POSITION = (0.47, 0.0, 0.19)
T = TypeVar("T")


@dataclass(frozen=True)
class DetectedObject:
    object_id: str
    label: str
    position: np.ndarray


@dataclass(frozen=True)
class SceneTruth:
    origin: np.ndarray
    aabb_center: np.ndarray
    half_extents: np.ndarray


@pytest.fixture(autouse=True)
def _pin_to_lcm(monkeypatch: pytest.MonkeyPatch) -> None:
    """Override the package fixture: this acceptance path uses the Zenoh default."""
    monkeypatch.delenv("DIMOS_TRANSPORT", raising=False)
    monkeypatch.setattr(global_config, "transport", "zenoh")


def _scene_truth() -> dict[str, SceneTruth]:
    """Read target origins and world-space bounds from the collision proxies."""
    model = mujoco.MjModel.from_xml_path(str(XARM_ROOM_SCENE_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    truth: dict[str, SceneTruth] = {}
    for body_id in range(1, model.nbody):
        joint_start = model.body_jntadr[body_id]
        joint_count = model.body_jntnum[body_id]
        is_free = any(
            model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE
            for joint_id in range(joint_start, joint_start + joint_count)
        )
        if not is_free:
            continue
        collision_geoms = np.flatnonzero((model.geom_bodyid == body_id) & (model.geom_group == 3))
        assert collision_geoms.size == 1, (
            f"Expected one canonical collision proxy for body {body_id}, got {collision_geoms}"
        )
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        assert name is not None
        geom_id = int(collision_geoms[0])
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        local_aabb = model.geom_aabb[geom_id]
        truth[name] = SceneTruth(
            origin=data.geom_xpos[geom_id].copy(),
            aabb_center=data.geom_xpos[geom_id] + rotation @ local_aabb[:3],
            half_extents=np.abs(rotation) @ local_aabb[3:],
        )
    assert len(truth) == EXPECTED_OBJECT_COUNT, (
        f"Expected {EXPECTED_OBJECT_COUNT} free room targets in {XARM_ROOM_SCENE_PATH}, "
        f"got {sorted(truth)}"
    )
    return truth


def _outside_geometry_error(position: np.ndarray, truth: SceneTruth) -> np.ndarray:
    """Return axis error beyond the truth geometry, not within its visible volume."""
    return np.maximum(np.abs(position - truth.aabb_center) - truth.half_extents, 0.0)


def _eventually(
    operation: Callable[[], T],
    predicate: Callable[[T], bool],
    *,
    timeout_s: float,
    description: str,
) -> T:
    deadline = time.monotonic() + timeout_s
    wake = threading.Event()
    last_value: T | None = None
    last_error: BaseException | None = None
    while time.monotonic() < deadline:
        try:
            last_value = operation()
            if predicate(last_value):
                return last_value
        except Exception as exc:
            last_error = exc
        wake.wait(0.25)
    raise TimeoutError(f"Timed out waiting for {description}; last={last_value!r}") from last_error


@pytest.fixture
def room_clients() -> Iterator[tuple[RPCClient, RPCClient, RPCClient]]:
    call = DimosCliCall()
    call.demo_args = ["xarm-room-sim"]
    call.global_args = ["--viewer", "none", "--n-workers", "3"]
    call.extra_env = {
        "DIMOS_TRANSPORT": "zenoh",
        "LIBGL_ALWAYS_SOFTWARE": "true",
        "MESA_LOADER_DRIVER_OVERRIDE": "llvmpipe",
        "MUJOCO_GL": "egl",
        # DimosCliCall resolves `dimos` through PATH; pin it to the same
        # environment running pytest instead of a stale user installation.
        "PATH": f"{Path(sys.executable).parent}:{os.environ.get('PATH', os.defpath)}",
    }
    call.start()

    pick = RPCClient(None, PickAndPlaceModule)
    scene = RPCClient(None, ObjectSceneRegistrationModule)
    manipulation = RPCClient(None, ManipulationModule)
    try:

        def planning_groups() -> tuple[Any, ...]:
            process = call.process
            assert process is not None and process.poll() is None, (
                f"xarm-room-sim exited during startup with {process and process.returncode}"
            )
            return cast("tuple[Any, ...]", manipulation.list_planning_groups())

        _eventually(
            planning_groups,
            lambda value: len(value) == 1,
            timeout_s=120.0,
            description="xArm room planning group",
        )
        _eventually(
            lambda: pick.scan_objects(XARM_ROOM_PROMPTS),
            lambda value: bool(value.success and (value.metadata or {}).get("objects")),
            timeout_s=180.0,
            description="first synchronized RGB-D room scan",
        )
        yield pick, scene, manipulation
    finally:
        manipulation.stop_rpc_client()
        scene.stop_rpc_client()
        pick.stop_rpc_client()
        call.stop()


def _detected_objects(pick: RPCClient, scene: RPCClient) -> list[DetectedObject]:
    result = pick.scan_objects(XARM_ROOM_PROMPTS)
    assert result.success, result
    metadata = result.metadata or {}
    registered = metadata.get("objects", [])
    assert registered, f"Room scan registered no objects: {result}"
    objects: list[DetectedObject] = []
    for item in registered:
        object_id = str(item["object_id"])
        pointcloud = scene.get_object_pointcloud_by_object_id(object_id)
        assert pointcloud is not None, f"Missing pointcloud for {object_id}"
        center = pointcloud.center
        objects.append(
            DetectedObject(
                object_id=object_id,
                label=str(item["name"]),
                position=np.array([center.x, center.y, center.z], dtype=np.float64),
            )
        )
    return objects


def _match_truth(
    detected: list[DetectedObject], truth: dict[str, SceneTruth]
) -> dict[str, DetectedObject]:
    remaining = list(detected)
    matches: dict[str, DetectedObject] = {}
    for name, expected in truth.items():
        assert remaining, f"No detection left to match {name}"
        match = min(
            remaining,
            key=lambda item: np.linalg.norm(_outside_geometry_error(item.position, expected)),
        )
        error = _outside_geometry_error(match.position, expected)
        assert np.all(error <= POSITION_TOLERANCE_M), (
            f"{name}: truth_origin={expected.origin.tolist()}, "
            f"truth_half_extents={expected.half_extents.tolist()}, "
            f"detected_centroid={match.position.tolist()}, outside_error={error.tolist()}"
        )
        matches[name] = match
        remaining.remove(match)
    assert len({item.object_id for item in matches.values()}) == len(truth)
    return matches


def test_three_scans_find_six_distinct_objects_at_scene_truth(
    room_clients: tuple[RPCClient, RPCClient, RPCClient],
) -> None:
    pick, scene, _ = room_clients
    truth = _scene_truth()
    for _ in range(3):
        matches = _match_truth(_detected_objects(pick, scene), truth)
        assert len(matches) == EXPECTED_OBJECT_COUNT


def test_scan_objects_enter_the_planning_obstacles(
    room_clients: tuple[RPCClient, RPCClient, RPCClient],
) -> None:
    pick, scene, manipulation = room_clients
    truth = _scene_truth()
    _match_truth(_detected_objects(pick, scene), truth)
    refreshed = manipulation.refresh_obstacles()
    assert refreshed >= len(truth)
    obstacles = manipulation.get_obstacles()
    obstacle_positions = [
        np.array([pose.position.x, pose.position.y, pose.position.z]) for pose in obstacles.values()
    ]
    for name, expected in truth.items():
        assert any(
            np.all(_outside_geometry_error(position, expected) <= POSITION_TOLERANCE_M)
            for position in obstacle_positions
        ), f"Planner obstacle missing near {name} at {expected.origin.tolist()}: {obstacles}"


def test_scripted_pick_and_place_completes(
    room_clients: tuple[RPCClient, RPCClient, RPCClient],
) -> None:
    pick, scene, _ = room_clients
    target = _match_truth(_detected_objects(pick, scene), _scene_truth())[DESIGNATED_TARGET]
    picked = pick.pick_object(target.object_id)
    assert picked.success, picked
    placed = pick.place_at(*PLACE_POSITION)
    assert placed.success, placed


def test_stale_observation_empty_grasp_fails_loudly(
    room_clients: tuple[RPCClient, RPCClient, RPCClient],
) -> None:
    pick, scene, _ = room_clients
    target = _match_truth(_detected_objects(pick, scene), _scene_truth())[DESIGNATED_TARGET]
    first_pick = pick.pick_object(target.object_id)
    assert first_pick.success, first_pick
    placed = pick.place_at(*PLACE_POSITION)
    assert placed.success, placed

    empty_pick = pick.pick_object(target.object_id)
    assert not empty_pick.success, empty_pick
    assert empty_pick.error_code == "GRASP_VERIFICATION_FAILED", empty_pick
    assert "nothing in the jaws" in empty_pick.message.lower(), empty_pick
