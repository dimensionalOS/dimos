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

"""Flat-ground MuJoCo Go2, and the Unitree↔MuJoCo motor index mapping."""

from __future__ import annotations

import os
from pathlib import Path

import mujoco
import numpy as np

# Unitree SDK LowCmd.motor_cmd / LowState.motor_state order for the 12 leg
# motors (indices 12-19 are unused on a Go2).
UNITREE_MOTOR_NAMES: tuple[str, ...] = (
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
)

# menagerie unitree_go2 actuator order.
MUJOCO_ACTUATOR_NAMES: tuple[str, ...] = (
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
)

DEFAULT_MENAGERIE = Path("/home/lesh/coding/deep/mujoco/menagerie")


def scene_path(menagerie: Path | None = None) -> Path:
    """Path to the flat-ground go2 scene (menagerie ``unitree_go2/scene.xml``)."""
    root = menagerie or Path(os.environ.get("MUJOCO_MENAGERIE", DEFAULT_MENAGERIE))
    scene = root / "unitree_go2" / "scene.xml"
    if not scene.is_file():
        raise FileNotFoundError(
            f"go2 scene not found at {scene}. Point MUJOCO_MENAGERIE at a "
            "mujoco_menagerie checkout."
        )
    return scene


def load(menagerie: Path | None = None) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(scene_path(menagerie)))
    return model, mujoco.MjData(model)


def unitree_to_mujoco(model: mujoco.MjModel) -> np.ndarray:
    """Index array ``p`` such that ``mujoco_vec = unitree_vec[p]``.

    Built by name, so it survives an actuator reorder in the MJCF; it does NOT
    validate that :data:`UNITREE_MOTOR_NAMES` matches the firmware that wrote a
    given recording. See ``replay.tracking_error`` for the empirical check.
    """
    actuators = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
    if actuators != list(MUJOCO_ACTUATOR_NAMES):
        raise ValueError(f"unexpected actuator order in model: {actuators}")
    return np.array([UNITREE_MOTOR_NAMES.index(name) for name in actuators], dtype=int)


def joint_qpos_adr(model: mujoco.MjModel) -> np.ndarray:
    """qpos addresses of the 12 leg joints, in MuJoCo actuator order."""
    return np.array(
        [
            model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{n}_joint")]
            for n in MUJOCO_ACTUATOR_NAMES
        ],
        dtype=int,
    )


def joint_dof_adr(model: mujoco.MjModel) -> np.ndarray:
    """qvel addresses of the 12 leg joints, in MuJoCo actuator order."""
    return np.array(
        [
            model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{n}_joint")]
            for n in MUJOCO_ACTUATOR_NAMES
        ],
        dtype=int,
    )


# Trunk collision box from the official URDF (unitree_ros go2_description):
# full size 0.3762 x 0.0935 x 0.114 at origin 0 0 0, i.e. the `base` frame sits
# at the geometric centre of the trunk. menagerie inherits this unchanged.
TRUNK_HALF_SIZE = (0.1881, 0.04675, 0.057)


def load_with_ghost(
    menagerie: Path | None = None,
    rgba: tuple[float, float, float, float] = (0.2, 1.0, 0.2, 0.35),
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Scene plus a translucent non-colliding mocap box for the recorded pose.

    Drive it through ``data.mocap_pos[0]`` / ``data.mocap_quat[0]``.
    """
    spec = mujoco.MjSpec.from_file(str(scene_path(menagerie)))
    body = spec.worldbody.add_body(name="ghost", mocap=True)
    geom = body.add_geom()
    geom.type = mujoco.mjtGeom.mjGEOM_BOX
    geom.size = TRUNK_HALF_SIZE
    geom.rgba = rgba
    geom.contype = 0
    geom.conaffinity = 0
    model = spec.compile()
    return model, mujoco.MjData(model)
