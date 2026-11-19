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

"""Fixed-base CuRobo V2 world: hard robot checks, soft scene guidance."""

from __future__ import annotations

from contextlib import contextmanager
from copy import deepcopy
from dataclasses import dataclass, replace
from importlib.metadata import PackageNotFoundError, version
import math
from pathlib import Path
from threading import RLock
from typing import TYPE_CHECKING, Any
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.validation import validate_obstacle
from dimos.manipulation.planning.world.curobo_model import build_curobo_model
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix

if TYPE_CHECKING:
    from collections.abc import Generator, Iterable

    from numpy.typing import NDArray

    from dimos.manipulation.planning.groups.models import PlanningGroup
    from dimos.manipulation.planning.spec.models import Obstacle, PlanningGroupID
    from dimos.manipulation.planning.spec.validation import PreparedRobotModel

CUROBO_VERSION = "0.9.0"
# Each occupied cell is one exact cuboid. Bound the per-query scene work; never
# subsample a map silently. This limit applies to all occupied cells combined.
MAX_VOXEL_CUBOIDS = 8192
_VOXEL_PREFIX = "__dimos_voxel:"
SceneModel = dict[str, dict[str, dict[str, Any]]]


def _obstacle_geometry(obstacle: Obstacle) -> tuple[str, dict[str, dict[str, Any]]]:
    matrix = pose_to_matrix(obstacle.pose)
    validate_obstacle(obstacle, matrix)
    if obstacle.name.startswith(_VOXEL_PREFIX):
        raise ValueError(f"Obstacle names starting with {_VOXEL_PREFIX!r} are reserved")
    if obstacle.touch_links:
        raise ValueError("CuRoboWorld does not support Obstacle.touch_links")
    xyz = matrix[:3, 3]
    x, y, z, w = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    wxyz = [float(v) for v in (w, x, y, z)]
    data = {"pose": xyz.tolist() + wxyz, "color": list(obstacle.color)}
    dims = [float(value) for value in obstacle.dimensions]
    match obstacle.obstacle_type:
        case ObstacleType.BOX:
            return "cuboid", {obstacle.name: {**data, "dims": dims}}
        case ObstacleType.SPHERE:
            # V2 SceneCfg accepts spheres, but SceneData does not load them.
            # A zero-length capsule is the same sphere in its collision kernel.
            return "capsule", {obstacle.name: {**data, "radius": dims[0]}}
        case ObstacleType.CYLINDER:
            return "cylinder", {obstacle.name: {**data, "radius": dims[0], "height": dims[1]}}
        case ObstacleType.MESH:
            assert obstacle.mesh_path is not None
            path = Path(obstacle.mesh_path).resolve()
            if not path.is_file():
                raise ValueError(f"CuRobo obstacle mesh does not exist: {path}")
            return "mesh", {obstacle.name: {**data, "file_path": str(path)}}
        case ObstacleType.OCTREE:
            resolution = obstacle.octree_resolution
            assert resolution is not None
            if not math.isfinite(resolution):
                raise ValueError("OCTREE resolution must be finite")
            if len(obstacle.points) > MAX_VOXEL_CUBOIDS:
                raise ValueError(
                    f"CuRobo scene supports at most {MAX_VOXEL_CUBOIDS} occupied cells"
                )
            centers = obstacle.points @ matrix[:3, :3].T + xyz
            if not np.isfinite(centers).all():
                raise ValueError("OCTREE transformed cell centers must be finite")
            return "cuboid", {
                f"{_VOXEL_PREFIX}{obstacle.name}:{i}": {
                    **data,
                    "pose": center.tolist() + wxyz,
                    "dims": [float(resolution)] * 3,
                }
                for i, center in enumerate(centers)
            }
    raise ValueError(f"Unsupported obstacle type: {obstacle.obstacle_type}")


def _scene_model(obstacles: Iterable[Obstacle]) -> SceneModel:
    scene: SceneModel = {}
    cells = 0
    for obstacle in obstacles:
        if obstacle.obstacle_type == ObstacleType.OCTREE:
            cells += len(obstacle.points)
            if cells > MAX_VOXEL_CUBOIDS:
                raise ValueError(
                    f"CuRobo scene supports at most {MAX_VOXEL_CUBOIDS} occupied cells"
                )
        kind, entries = _obstacle_geometry(obstacle)
        scene.setdefault(kind, {}).update(entries)
    return scene


def _trajopt_metrics(stock: dict[str, Any]) -> dict[str, Any]:
    metrics = deepcopy(stock)
    del metrics["rollout"]["constraint_cfg"]["scene_collision_cfg"]
    return metrics


def _load_device(device: str) -> Any:
    try:
        installed = version("nvidia-curobo")
    except PackageNotFoundError as exc:
        raise ImportError(f"CuRoboWorld requires nvidia-curobo=={CUROBO_VERSION}") from exc
    if installed != CUROBO_VERSION:
        raise ImportError(f"CuRoboWorld requires nvidia-curobo=={CUROBO_VERSION}; got {installed}")

    from curobo.types import DeviceCfg
    import torch

    selected = torch.device(device)
    if (
        selected.type != "cuda"
        or not torch.cuda.is_available()
        or (selected.index is not None and selected.index >= torch.cuda.device_count())
    ):
        raise RuntimeError(f"CuRoboWorld requires an available CUDA device, got {device!r}")
    return DeviceCfg(device=selected)


def _create_native_planner(
    robot: dict[str, Any], scene: SceneModel, cache: dict[str, int], device_cfg: Any
) -> Any:
    from curobo.config_io import load_yaml
    from curobo.content import get_task_configs_path
    from curobo.motion_planner import MotionPlanner, MotionPlannerCfg

    stock = load_yaml(str(get_task_configs_path() / "metrics_base.yml"))
    cfg = MotionPlannerCfg.create(
        robot=deepcopy(robot),
        scene_model=deepcopy(scene),
        collision_cache=dict(cache),
        device_cfg=device_cfg,
        self_collision_check=True,
        trajopt_metrics_rollout=_trajopt_metrics(stock),
    )
    return MotionPlanner(cfg)


def _create_self_checker(robot: dict[str, Any], device_cfg: Any) -> Any:
    from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg

    return RobotCollisionChecker(
        RobotCollisionCheckerCfg.load_from_config(
            robot_config=deepcopy(robot), scene_model=None, device_cfg=device_cfg
        )
    )


@dataclass
class CuRoboContext:
    q: NDArray[np.float64]


class CuRoboWorld:
    """Own the native planner and its scene; serialize native work with scene changes.

    External obstacles guide trajectory optimization but are not hard acceptance
    constraints in this backend. Hard configuration/edge checks use the prepared
    joint limits and native self-collision only. Use this backend only when that
    distinction is acceptable to the caller.
    """

    def __init__(self, enable_viz: bool = False, *, device: str = "cuda:0") -> None:
        if enable_viz:
            raise ValueError("CuRoboWorld does not provide native visualization")
        self._device = device
        self._device_cfg: Any = None
        self._lock = RLock()
        self._prepared: PreparedRobotModel | None = None
        self._robot: dict[str, Any] | None = None
        self._obstacles: dict[str, Obstacle] = {}
        self._scene: SceneModel = {}
        self._scene_dirty = False
        self._mesh_dirty = False
        self._cache: dict[str, int] = {}
        self._native: Any = None
        self._fk: Any = None
        self._self_checker: Any = None
        self._live: CuRoboContext | None = None
        self._has_authoritative_state = False

    def load_model(self, prepared: PreparedRobotModel) -> None:
        with self._lock:
            if self._prepared is not None:
                raise ValueError("A model is already loaded")
            self._prepared = prepared

    def get_prepared_model(self) -> PreparedRobotModel:
        if self._prepared is None:
            raise RuntimeError("No CuRobo model loaded")
        return self._prepared

    def ordered_joint_positions(self, joint_state: JointState) -> NDArray[np.float64]:
        return self.get_prepared_model().joint_space.from_joint_state(joint_state)

    def finalize(self) -> None:
        with self._lock:
            if self.is_finalized:
                return
            prepared = self.get_prepared_model()
            self._device_cfg = _load_device(self._device)
            if self._robot is None:
                self._robot = build_curobo_model(prepared)
            home = self._robot["kinematics"]["cspace"]["default_joint_position"]
            q = prepared.joint_space.normalize_positions(home)
            self._install_scene()
            self._live = CuRoboContext(q)

    @property
    def is_finalized(self) -> bool:
        return self._live is not None

    def is_ready(self) -> bool:
        with self._lock:
            return self.is_finalized and self._has_authoritative_state

    def _install_scene(self) -> None:
        assert self._robot is not None
        capacity = {
            kind: max(self._cache.get(kind, 0), 1 << (len(entries) - 1).bit_length())
            for kind, entries in self._scene.items()
        }
        capacity = {**self._cache, **capacity}
        if self._native is None or capacity != self._cache or self._mesh_dirty:
            self._discard_native()
            planner = _create_native_planner(self._robot, self._scene, capacity, self._device_cfg)
            try:
                self._check_native_joint_names(planner.joint_names)
            except ValueError:
                planner.destroy()
                raise
            self._native, self._cache = planner, capacity
        elif self._scene_dirty:
            from curobo.scene import Scene

            try:
                self._native.update_world(Scene.create(deepcopy(self._scene)))
            except Exception:
                # Native storage may have been partially modified. No later
                # solve may see it: retry by rebuilding from the full snapshot.
                self._discard_native()
                raise
        self._scene_dirty = False
        self._mesh_dirty = False

    def _discard_native(self) -> None:
        native, self._native = self._native, None
        if native is not None:
            native.destroy()

    def _check_native_joint_names(self, names: Iterable[str]) -> None:
        canonical = self.get_prepared_model().joint_space.names
        native = tuple(names)
        if len(native) != len(canonical) or set(native) != set(canonical):
            raise ValueError(
                f"CuRobo joint names {native} do not match canonical joints {canonical}"
            )

    @contextmanager
    def native_context(self) -> Generator[Any, None, None]:
        """Yield the current MotionPlanner exclusively; do not retain it after exit."""
        with self._lock:
            self.get_live_context()
            self._install_scene()
            yield self._native

    def _replace_scene(self, obstacles: dict[str, Obstacle], *, reload_mesh: bool = False) -> None:
        scene = _scene_model(obstacles.values())
        # V2 keeps Warp meshes cached by name even after update_world clears
        # its slots. Rebuild when mesh geometry may have been replaced.
        if reload_mesh or scene.get("mesh", {}).keys() != self._scene.get("mesh", {}).keys():
            self._mesh_dirty = True
        self._obstacles, self._scene = obstacles, scene
        self._scene_dirty = True

    def add_obstacle(self, obstacle: Obstacle) -> str | None:
        with self._lock:
            self.get_live_context()
            if not obstacle.name or obstacle.name in self._obstacles:
                return None
            self._replace_scene({**self._obstacles, obstacle.name: deepcopy(obstacle)})
            return obstacle.name

    def update_obstacle(self, obstacle: Obstacle) -> bool:
        with self._lock:
            self.get_live_context()
            if obstacle.name not in self._obstacles:
                return False
            self._replace_scene(
                {**self._obstacles, obstacle.name: deepcopy(obstacle)},
                reload_mesh=obstacle.obstacle_type == ObstacleType.MESH,
            )
            return True

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> bool:
        with self._lock:
            self.get_live_context()
            obstacle = self._obstacles.get(obstacle_id)
            if obstacle is None:
                return False
            self._replace_scene(
                {**self._obstacles, obstacle_id: replace(obstacle, pose=deepcopy(pose))}
            )
            return True

    def remove_obstacle(self, obstacle_id: str) -> bool:
        with self._lock:
            self.get_live_context()
            if obstacle_id not in self._obstacles:
                return False
            obstacles = dict(self._obstacles)
            del obstacles[obstacle_id]
            self._replace_scene(obstacles)
            return True

    def clear_obstacles(self) -> None:
        with self._lock:
            self.get_live_context()
            if self._obstacles:
                self._replace_scene({})

    def get_obstacles(self) -> list[Obstacle]:
        with self._lock:
            self.get_live_context()
            return deepcopy(list(self._obstacles.values()))

    def get_live_context(self) -> CuRoboContext:
        if self._live is None:
            raise RuntimeError("CuRoboWorld is not finalized")
        return self._live

    @contextmanager
    def scratch_context(self) -> Generator[CuRoboContext, None, None]:
        with self._lock:
            context = CuRoboContext(self.get_live_context().q.copy())
        yield context

    def sync_from_joint_state(self, joint_state: JointState) -> None:
        with self._lock:
            if self._live is not None:
                self.set_joint_state(self._live, joint_state)
                self._has_authoritative_state = True

    def set_joint_state(self, ctx: CuRoboContext, joint_state: JointState) -> None:
        with self._lock:
            ctx.q = self.ordered_joint_positions(joint_state)

    def get_joint_state(self, ctx: CuRoboContext) -> JointState:
        with self._lock:
            return JointState(
                name=list(self.get_prepared_model().joint_space.names), position=ctx.q.tolist()
            )

    def _native_state(self, q: NDArray[np.float64], names: Iterable[str]) -> Any:
        from curobo.types import JointState as NativeState

        native_names = list(names)
        canonical = self.get_prepared_model().joint_space.names
        values = np.ascontiguousarray(q[..., [canonical.index(name) for name in native_names]])
        return NativeState.from_position(
            self._device_cfg.to_device(values), joint_names=native_names
        )

    def _self_collision_free(self, configurations: NDArray[np.float64]) -> bool:
        with self._lock:
            self.get_live_context()
            assert self._robot is not None
            if self._self_checker is None:
                # No external scene or solver buffers participate in hard checks.
                checker = _create_self_checker(self._robot, self._device_cfg)
                self._check_native_joint_names(checker.kinematics.joint_names)
                self._self_checker = checker
            checker = self._self_checker
            state = self._native_state(configurations[:, None, :], checker.kinematics.joint_names)
            spheres = checker.get_kinematics(state.position).robot_spheres
            return bool((checker.get_self_collision(spheres) == 0).all().item())

    def is_collision_free(self, ctx: CuRoboContext) -> bool:
        return self.check_config_collision_free(self.get_joint_state(ctx))

    def get_min_distance(self, ctx: CuRoboContext) -> float:
        raise NotImplementedError("CuRoboWorld does not expose a signed minimum distance")

    def check_config_collision_free(self, joint_state: JointState) -> bool:
        self.get_live_context()
        try:
            q = self.ordered_joint_positions(joint_state)
        except ValueError:
            return False
        return self._self_collision_free(q[None, :])

    def check_edge_collision_free(
        self, start: JointState, end: JointState, step_size: float = 0.05
    ) -> bool:
        self.get_live_context()
        if not math.isfinite(step_size) or step_size <= 0.0:
            raise ValueError("Collision edge step_size must be finite and positive")
        try:
            a = self.ordered_joint_positions(start)
            b = self.ordered_joint_positions(end)
        except ValueError:
            return False
        length = float(np.linalg.norm(b - a))
        steps = math.ceil(length / step_size)
        with self._lock:
            for first in range(0, steps + 1, 128):
                t = np.arange(first, min(first + 128, steps + 1)) / max(1, steps)
                if not self._self_collision_free(a + t[:, None] * (b - a)):
                    return False
        return True

    def _group(self, group_id: PlanningGroupID | None = None) -> PlanningGroup:
        (group,) = self.get_prepared_model().planning_groups
        if group_id is not None and group_id != group.id:
            raise ValueError(f"Unknown CuRobo planning group: {group_id}")
        return group

    def _compute_fk(self, ctx: CuRoboContext) -> Any:
        from curobo.kinematics import Kinematics, KinematicsCfg

        self.get_live_context()
        assert self._robot is not None
        if self._fk is None:
            # Do not make every link a planner goal frame. Build FK separately
            # from the same materialized URDF/locked joints, requesting all links.
            robot = deepcopy(self._robot)
            kinematics = robot["kinematics"]
            (tip,) = kinematics["tool_frames"]
            links = [tip] + [
                link.attrib["name"]
                for link in ET.parse(kinematics["urdf_path"]).findall("link")
                if link.attrib["name"] != tip
            ]
            cfg = KinematicsCfg.from_robot_yaml_file(
                robot, tool_frames=links, device_cfg=self._device_cfg
            )
            fk = Kinematics(cfg, compute_jacobian=True, compute_spheres=False)
            self._check_native_joint_names(fk.joint_names)
            self._fk = fk
        return self._fk.compute_kinematics(self._native_state(ctx.q, self._fk.joint_names))

    def get_link_pose(self, ctx: CuRoboContext, link_name: str) -> NDArray[np.float64]:
        with self._lock:
            if (
                link_name == self.get_prepared_model().description.root_link
                or link_name == self.get_prepared_model().config.base_link
            ):
                self.get_live_context()
                return np.eye(4)
            state = self._compute_fk(ctx)
            if link_name not in state.tool_frames:
                raise ValueError(f"Unknown CuRobo link: {link_name}")
            pose = state.tool_poses.get_link_pose(link_name)
            matrix = np.eye(4)
            matrix[:3, 3] = pose.position.reshape(3).detach().cpu().numpy()
            w, x, y, z = pose.quaternion.reshape(4).detach().cpu().numpy()
            matrix[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()
            return matrix

    def get_ee_pose(self, ctx: CuRoboContext) -> PoseStamped:
        return self.get_group_ee_pose(ctx, self._group().id)

    def get_group_ee_pose(self, ctx: CuRoboContext, group_id: PlanningGroupID) -> PoseStamped:
        tip = self._group(group_id).tip_link
        assert tip is not None
        pose = matrix_to_pose(self.get_link_pose(ctx, tip))
        return PoseStamped(frame_id="world", position=pose.position, orientation=pose.orientation)

    def get_jacobian(self, ctx: CuRoboContext) -> NDArray[np.float64]:
        group = self._group()
        return self._jacobian(ctx, group, self.get_prepared_model().joint_space.names)

    def get_group_jacobian(
        self, ctx: CuRoboContext, group_id: PlanningGroupID
    ) -> NDArray[np.float64]:
        group = self._group(group_id)
        return self._jacobian(ctx, group, group.joint_names)

    def _jacobian(
        self, ctx: CuRoboContext, group: PlanningGroup, names: tuple[str, ...]
    ) -> NDArray[np.float64]:
        with self._lock:
            state = self._compute_fk(ctx)
            tool = state.tool_frames.index(group.tip_link)
            cols = [self._fk.joint_names.index(name) for name in names]
            return (
                state.tool_jacobians[0, 0, tool].detach().cpu().numpy()[:, cols].astype(np.float64)
            )

    def close(self) -> None:
        with self._lock:
            self._live = None
            self._has_authoritative_state = False
            self._fk = None
            self._self_checker = None
            self._discard_native()
