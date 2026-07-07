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

"""MuJoCo ``WorldSpec`` implementation for manipulation planning.

This backend is intentionally small: it exposes MuJoCo FK, Jacobians, joint
limits, and collision checking through the planning ``WorldSpec`` protocol.
Reachability construction uses only that protocol surface, so the same sampler
can run against this backend or DrakeWorld for comparison.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from threading import RLock
from typing import TYPE_CHECKING, Any
import xml.etree.ElementTree as ET

import numpy as np

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import Obstacle, WorldRobotID
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Generator

    from numpy.typing import NDArray

logger = setup_logger()

try:
    import mujoco

    MUJOCO_AVAILABLE = True
except ImportError:
    mujoco = None
    MUJOCO_AVAILABLE = False


@dataclass(frozen=True)
class _MujocoRobotData:
    robot_id: WorldRobotID
    config: RobotModelConfig
    model: Any
    live_data: Any
    q_base: NDArray[np.float64]
    joint_ids: NDArray[np.intp]
    qpos_adr: NDArray[np.intp]
    dof_adr: NDArray[np.intp]
    lower: NDArray[np.float64]
    upper: NDArray[np.float64]
    ee_body_id: int
    geom_bodyid: NDArray[np.intp]
    collision_geom_mask: NDArray[np.bool_]
    excluded_body_pairs: NDArray[np.bool_]


@dataclass
class _MujocoContext:
    data_by_robot: dict[WorldRobotID, Any]


class MujocoWorld(WorldSpec):
    """MuJoCo implementation of the manipulation planning ``WorldSpec``."""

    def __init__(self) -> None:
        if not MUJOCO_AVAILABLE or mujoco is None:
            raise ImportError("MuJoCo is not installed. Install with: uv sync --extra sim")
        self._lock = RLock()
        self._robots: dict[WorldRobotID, _MujocoRobotData] = {}
        self._obstacles: dict[str, Obstacle] = {}
        self._robot_counter = 0
        self._finalized = False

    def add_robot(self, config: RobotModelConfig) -> WorldRobotID:
        """Add a robot model to the MuJoCo world."""
        if self._finalized:
            raise RuntimeError("Cannot add robot after world is finalized")

        with self._lock:
            self._robot_counter += 1
            robot_id = f"robot_{self._robot_counter}"
            model = compile_mujoco_model_from_config(config)
            live_data = mujoco.MjData(model)
            q_base = _base_qpos(model, config.base_pose)

            joint_ids: list[int] = []
            for joint_name in config.joint_names:
                joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                if joint_id < 0:
                    raise ValueError(f"Joint '{joint_name}' not found in model {config.model_path}")
                joint_ids.append(joint_id)

            ee_body_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, config.end_effector_link
            )
            if ee_body_id < 0:
                raise ValueError(
                    f"End-effector body '{config.end_effector_link}' not found in model"
                )

            qpos_adr = np.array([model.jnt_qposadr[j] for j in joint_ids], dtype=np.intp)
            dof_adr = np.array([model.jnt_dofadr[j] for j in joint_ids], dtype=np.intp)
            lower, upper = _joint_limits(model, joint_ids, config)
            geom_bodyid = np.asarray(model.geom_bodyid, dtype=np.intp)
            excluded = _excluded_body_pairs(model, config.collision_exclusion_pairs)
            collision_mask = _moving_subtree_geom_mask(model, joint_ids)

            live_data.qpos[:] = q_base
            mujoco.mj_forward(model, live_data)

            self._robots[robot_id] = _MujocoRobotData(
                robot_id=robot_id,
                config=config,
                model=model,
                live_data=live_data,
                q_base=q_base,
                joint_ids=np.asarray(joint_ids, dtype=np.intp),
                qpos_adr=qpos_adr,
                dof_adr=dof_adr,
                lower=lower,
                upper=upper,
                ee_body_id=ee_body_id,
                geom_bodyid=geom_bodyid,
                collision_geom_mask=collision_mask,
                excluded_body_pairs=excluded,
            )
            logger.info("Added MuJoCo robot '%s' (%s)", robot_id, config.name)
            return robot_id

    def get_robot_ids(self) -> list[WorldRobotID]:
        """Get all robot IDs."""
        return list(self._robots)

    def get_robot_config(self, robot_id: WorldRobotID) -> RobotModelConfig:
        """Get a robot's model config."""
        return self._robot(robot_id).config

    def get_joint_limits(
        self, robot_id: WorldRobotID
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Get lower and upper joint limits."""
        robot = self._robot(robot_id)
        return robot.lower.copy(), robot.upper.copy()

    def add_obstacle(self, obstacle: Obstacle) -> str:
        """Store obstacle metadata.

        Dynamic obstacle collision geometry is not compiled into this MuJoCo
        backend yet. Callers that need obstacle collision should use DrakeWorld.
        """
        self._obstacles[obstacle.name] = obstacle
        logger.warning("MuJoCoWorld stores obstacle '%s' but does not collide it", obstacle.name)
        return obstacle.name

    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove a stored obstacle."""
        return self._obstacles.pop(obstacle_id, None) is not None

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> bool:
        """Update stored obstacle pose."""
        obstacle = self._obstacles.get(obstacle_id)
        if obstacle is None:
            return False
        obstacle.pose = pose
        return True

    def clear_obstacles(self) -> None:
        """Remove all stored obstacles."""
        self._obstacles.clear()

    def get_obstacles(self) -> list[Obstacle]:
        """Get stored obstacles."""
        return list(self._obstacles.values())

    def finalize(self) -> None:
        """Finalize the world."""
        self._finalized = True

    @property
    def is_finalized(self) -> bool:
        """Check whether the world is finalized."""
        return self._finalized

    def get_live_context(self) -> _MujocoContext:
        """Get the live MuJoCo context."""
        self._require_finalized()
        return _MujocoContext(
            {robot_id: robot.live_data for robot_id, robot in self._robots.items()}
        )

    @contextmanager
    def scratch_context(self) -> Generator[_MujocoContext, None, None]:
        """Create a scratch context with copied live qpos/qvel state."""
        self._require_finalized()
        with self._lock:
            data_by_robot = {}
            for robot_id, robot in self._robots.items():
                data = mujoco.MjData(robot.model)
                data.qpos[:] = robot.live_data.qpos
                data.qvel[:] = robot.live_data.qvel
                mujoco.mj_forward(robot.model, data)
                data_by_robot[robot_id] = data
        yield _MujocoContext(data_by_robot)

    def sync_from_joint_state(self, robot_id: WorldRobotID, joint_state: JointState) -> None:
        """Sync live context from a joint state."""
        self._require_finalized()
        with self._lock:
            self._set_joint_state_on_data(
                self._robot(robot_id), self._robot(robot_id).live_data, joint_state
            )

    def set_joint_state(
        self, ctx: _MujocoContext, robot_id: WorldRobotID, joint_state: JointState
    ) -> None:
        """Set robot joint state in a context."""
        self._set_joint_state_on_data(
            self._robot(robot_id), ctx.data_by_robot[robot_id], joint_state
        )

    def get_joint_state(self, ctx: _MujocoContext, robot_id: WorldRobotID) -> JointState:
        """Get robot joint state from a context."""
        robot = self._robot(robot_id)
        data = ctx.data_by_robot[robot_id]
        return JointState(
            name=list(robot.config.joint_names),
            position=[float(data.qpos[adr]) for adr in robot.qpos_adr],
        )

    def is_collision_free(self, ctx: _MujocoContext, robot_id: WorldRobotID) -> bool:
        """Check whether the robot has penetrating self-collisions."""
        robot = self._robot(robot_id)
        data = ctx.data_by_robot[robot_id]
        mujoco.mj_forward(robot.model, data)
        mujoco.mj_collision(robot.model, data)
        return not _has_relevant_collision(robot, data)

    def get_min_distance(self, ctx: _MujocoContext, robot_id: WorldRobotID) -> float:
        """Return minimum relevant contact distance."""
        robot = self._robot(robot_id)
        data = ctx.data_by_robot[robot_id]
        mujoco.mj_forward(robot.model, data)
        mujoco.mj_collision(robot.model, data)
        if not data.ncon:
            return float("inf")
        geom = data.contact.geom[: data.ncon]
        dist = data.contact.dist[: data.ncon]
        relevant = _relevant_contact_mask(robot, geom)
        if not np.any(relevant):
            return float("inf")
        return float(np.min(dist[relevant]))

    def check_config_collision_free(self, robot_id: WorldRobotID, joint_state: JointState) -> bool:
        """Check a joint state for collision."""
        with self.scratch_context() as ctx:
            self.set_joint_state(ctx, robot_id, joint_state)
            return self.is_collision_free(ctx, robot_id)

    def check_edge_collision_free(
        self,
        robot_id: WorldRobotID,
        start: JointState,
        end: JointState,
        step_size: float = 0.05,
    ) -> bool:
        """Check interpolated joint-space edge for collision."""
        start_q = np.asarray(start.position, dtype=np.float64)
        end_q = np.asarray(end.position, dtype=np.float64)
        distance = float(np.linalg.norm(end_q - start_q))
        if distance < 1e-9:
            return self.check_config_collision_free(robot_id, start)
        n_steps = max(2, int(np.ceil(distance / step_size)) + 1)
        with self.scratch_context() as ctx:
            for i in range(n_steps):
                alpha = i / (n_steps - 1)
                q = (1.0 - alpha) * start_q + alpha * end_q
                self.set_joint_state(
                    ctx, robot_id, JointState(name=list(start.name), position=q.tolist())
                )
                if not self.is_collision_free(ctx, robot_id):
                    return False
        return True

    def get_ee_pose(self, ctx: _MujocoContext, robot_id: WorldRobotID) -> PoseStamped:
        """Get end-effector pose as a PoseStamped."""
        matrix = self.get_link_pose(ctx, robot_id, self._robot(robot_id).config.end_effector_link)
        quat = Quaternion.from_rotation_matrix(matrix[:3, :3])
        return PoseStamped(
            frame_id="world",
            position=Vector3(matrix[0, 3], matrix[1, 3], matrix[2, 3]),
            orientation=quat,
        )

    def get_link_pose(
        self, ctx: _MujocoContext, robot_id: WorldRobotID, link_name: str
    ) -> NDArray[np.float64]:
        """Get link pose as a 4x4 homogeneous transform."""
        robot = self._robot(robot_id)
        body_id = mujoco.mj_name2id(robot.model, mujoco.mjtObj.mjOBJ_BODY, link_name)
        if body_id < 0:
            raise KeyError(f"Link/body '{link_name}' not found in robot '{robot_id}'")
        data = ctx.data_by_robot[robot_id]
        mujoco.mj_forward(robot.model, data)
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = data.xmat[body_id].reshape(3, 3)
        matrix[:3, 3] = data.xpos[body_id]
        return matrix

    def get_jacobian(self, ctx: _MujocoContext, robot_id: WorldRobotID) -> NDArray[np.float64]:
        """Get geometric Jacobian rows ordered as [linear, angular]."""
        robot = self._robot(robot_id)
        data = ctx.data_by_robot[robot_id]
        mujoco.mj_forward(robot.model, data)
        jacp = np.zeros((3, robot.model.nv), dtype=np.float64)
        jacr = np.zeros((3, robot.model.nv), dtype=np.float64)
        mujoco.mj_jacBody(robot.model, data, jacp, jacr, robot.ee_body_id)
        cols = robot.dof_adr
        return np.vstack([jacp[:, cols], jacr[:, cols]])

    def _robot(self, robot_id: WorldRobotID) -> _MujocoRobotData:
        try:
            return self._robots[robot_id]
        except KeyError:
            raise KeyError(f"Robot '{robot_id}' not found") from None

    def _require_finalized(self) -> None:
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

    def _set_joint_state_on_data(
        self, robot: _MujocoRobotData, data: Any, joint_state: JointState
    ) -> None:
        values_by_name = dict(zip(joint_state.name, joint_state.position, strict=False))
        if values_by_name:
            positions = [values_by_name[name] for name in robot.config.joint_names]
        else:
            positions = joint_state.position
        if len(positions) != len(robot.qpos_adr):
            raise ValueError(
                f"Expected {len(robot.qpos_adr)} positions for {robot.robot_id}, got {len(positions)}"
            )
        data.qpos[:] = robot.q_base
        data.qpos[robot.qpos_adr] = np.asarray(positions, dtype=np.float64)
        mujoco.mj_forward(robot.model, data)


def compile_mujoco_model_from_config(config: RobotModelConfig) -> Any:
    """Compile a planning robot config into a MuJoCo model."""
    model_path = Path(str(config.model_path)).resolve()
    if not model_path.exists():
        raise FileNotFoundError(f"Robot model not found: {model_path}")

    if model_path.suffix == ".xml":
        spec = mujoco.MjSpec.from_file(str(model_path))
        spec.meshdir = str(_mjcf_meshdir(config, spec, model_path))
    else:
        prepared_path = prepare_urdf_for_mujoco(config)
        spec = mujoco.MjSpec.from_string(_mujoco_urdf_text(prepared_path))
    return spec.compile()


def prepare_urdf_for_mujoco(config: RobotModelConfig) -> Path:
    """Return a processed URDF path for MuJoCo-backed helpers."""
    model_path = Path(str(config.model_path)).resolve()
    if model_path.suffix == ".xml":
        raise ValueError("MJCF models do not have a prepared URDF path")
    return Path(
        prepare_urdf_for_drake(
            urdf_path=model_path,
            package_paths=config.package_paths,
            xacro_args=config.xacro_args,
            convert_meshes=bool(config.auto_convert_meshes),
        )
    )


def _mjcf_meshdir(config: RobotModelConfig, spec: Any, model_path: Path) -> Path:
    if spec.meshdir:
        return (model_path.parent / spec.meshdir).resolve()
    for root in config.package_paths.values():
        mesh_dir = Path(root) / "meshes"
        if mesh_dir.exists():
            return mesh_dir.resolve()
    return model_path.parent.resolve()


def _mujoco_urdf_text(urdf_path: Path) -> str:
    root = ET.parse(urdf_path).getroot()
    invalid_links = _remove_invalid_inertials(root)
    _ensure_mujoco_compiler(root)
    if invalid_links:
        logger.warning(
            "Removed invalid inertial tags for MuJoCo compile: %s",
            ", ".join(invalid_links),
        )
    return ET.tostring(root, encoding="unicode")


def _ensure_mujoco_compiler(root: ET.Element) -> None:
    mujoco_node = root.find("mujoco")
    if mujoco_node is None:
        mujoco_node = ET.SubElement(root, "mujoco")
    compiler = mujoco_node.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(mujoco_node, "compiler")
    defaults = {
        "strippath": "false",
        "discardvisual": "true",
        "inertiafromgeom": "auto",
    }
    for name, value in defaults.items():
        if compiler.get(name) is None:
            compiler.set(name, value)


def _remove_invalid_inertials(root: ET.Element) -> list[str]:
    invalid_links: list[str] = []
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        if _valid_inertial(inertial):
            continue
        link.remove(inertial)
        invalid_links.append(link.get("name") or "<unnamed>")
    return invalid_links


def _valid_inertial(inertial: ET.Element) -> bool:
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    if mass is None or inertia is None:
        return False
    try:
        mass_value = float(mass.get("value", "nan"))
        matrix = np.array(
            [
                [
                    float(inertia.get("ixx", "nan")),
                    float(inertia.get("ixy", "nan")),
                    float(inertia.get("ixz", "nan")),
                ],
                [
                    float(inertia.get("ixy", "nan")),
                    float(inertia.get("iyy", "nan")),
                    float(inertia.get("iyz", "nan")),
                ],
                [
                    float(inertia.get("ixz", "nan")),
                    float(inertia.get("iyz", "nan")),
                    float(inertia.get("izz", "nan")),
                ],
            ],
            dtype=np.float64,
        )
    except ValueError:
        return False
    if not np.isfinite(mass_value) or mass_value <= 0.0:
        return False
    if not np.all(np.isfinite(matrix)):
        return False
    principal = np.linalg.eigvalsh(matrix)
    if np.any(principal <= 0.0):
        return False
    largest = float(principal[-1])
    other_sum = float(principal[0] + principal[1])
    return largest <= other_sum + 1e-12


def _base_qpos(model: Any, pose: PoseStamped) -> NDArray[np.float64]:
    q = model.qpos0.copy()
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_FREE:
            continue
        adr = int(model.jnt_qposadr[joint_id])
        q[adr : adr + 3] = (pose.position.x, pose.position.y, pose.position.z)
        q[adr + 3 : adr + 7] = (
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        )
    return q


def _joint_limits(
    model: Any, joint_ids: list[int], config: RobotModelConfig
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    if config.joint_limits_lower is not None and config.joint_limits_upper is not None:
        return (
            np.asarray(config.joint_limits_lower, dtype=np.float64),
            np.asarray(config.joint_limits_upper, dtype=np.float64),
        )
    lower = np.array([model.jnt_range[j][0] for j in joint_ids], dtype=np.float64)
    upper = np.array([model.jnt_range[j][1] for j in joint_ids], dtype=np.float64)
    missing = ~np.isfinite(lower) | ~np.isfinite(upper) | (lower == upper)
    lower[missing] = -np.pi
    upper[missing] = np.pi
    return lower, upper


def _moving_subtree_geom_mask(model: Any, joint_ids: list[int]) -> NDArray[np.bool_]:
    chain_bodies = {int(model.jnt_bodyid[joint_id]) for joint_id in joint_ids}
    mask = np.zeros(model.ngeom, dtype=bool)
    for body_id in range(model.nbody):
        current = body_id
        while current != 0:
            if current in chain_bodies:
                adr = int(model.body_geomadr[body_id])
                num = int(model.body_geomnum[body_id])
                mask[adr : adr + num] = True
                break
            current = int(model.body_parentid[current])
    return mask


def _excluded_body_pairs(model: Any, excluded_pairs: list[tuple[str, str]]) -> NDArray[np.bool_]:
    excluded = np.zeros((model.nbody, model.nbody), dtype=bool)
    for name_a, name_b in excluded_pairs:
        body_a = _body_id(model, name_a)
        body_b = _body_id(model, name_b)
        if body_a >= 0 and body_b >= 0:
            excluded[body_a, body_b] = True
            excluded[body_b, body_a] = True
    return excluded


def _body_id(model: Any, name: str) -> int:
    if name == "world":
        return 0
    return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))


def _has_relevant_collision(robot: _MujocoRobotData, data: Any) -> bool:
    if not data.ncon:
        return False
    geom = data.contact.geom[: data.ncon]
    dist = data.contact.dist[: data.ncon]
    return bool(np.any(_relevant_contact_mask(robot, geom) & (dist < 0.0)))


def _relevant_contact_mask(robot: _MujocoRobotData, geom: NDArray[np.int32]) -> NDArray[np.bool_]:
    involved = robot.collision_geom_mask[geom[:, 0]] | robot.collision_geom_mask[geom[:, 1]]
    excluded = robot.excluded_body_pairs[
        robot.geom_bodyid[geom[:, 0]], robot.geom_bodyid[geom[:, 1]]
    ]
    return involved & ~excluded


__all__ = ["MujocoWorld", "compile_mujoco_model_from_config", "prepare_urdf_for_mujoco"]
