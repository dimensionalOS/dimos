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

"""Adapt the existing G1 MJCF to robosuite and retain its GR00T startup settings."""

from __future__ import annotations

from functools import partial
from pathlib import Path
from typing import TYPE_CHECKING
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import NDArray
from robosuite.models.robots.robot_model import RobotModel

from dimos.control.components import make_humanoid_joints
from dimos.robot.unitree.g1.groot_config import G1_GROOT_HOME, G1_GROOT_KD, G1_GROOT_KP
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.world.scene import scene_spawn
from dimos.utils.data import LfsPath

if TYPE_CHECKING:
    from dimos.robot.unitree.g1.wholebody_connection import G1WholeBodyConnectionConfig


class G1Model(RobotModel):
    """Use the 29-DOF G1 asset in its hardware motor order."""

    public_motor_names = tuple(make_humanoid_joints("g1"))
    control_imu = "control_imu"
    startup_kp = tuple(G1_GROOT_KP)
    startup_kd = tuple(G1_GROOT_KD)
    wait_for_first_command = True

    def __init__(self, idn: str | int = "g1") -> None:
        path = Path(__file__).parent / "assets" / "g1_29dof.xml"
        source = ET.parse(path).getroot()
        defaults = source.find("./default/joint")
        attributes = {
            joint.attrib["name"]: {
                **({} if defaults is None else defaults.attrib),
                **{key: value for key, value in joint.attrib.items() if key != "name"},
            }
            for joint in source.findall(".//worldbody//joint")
        }
        super().__init__(str(path), idn=idn)
        # RobotModel uses diagnostic colors on contact geoms and removes their
        # materials. Restore the same imported elements, including unnamed ones.
        for geom, appearance in self._authored_appearance.items():
            for attribute, value in appearance.items():
                if value is None:
                    geom.attrib.pop(attribute, None)
                else:
                    geom.set(
                        attribute, self.correct_naming(value) if attribute == "material" else value
                    )
        # robosuite removes anonymous defaults and supplies its own joint
        # dynamics. Restore the source values instead of changing the G1 plant.
        named_attributes = {self.correct_naming(name): attrs for name, attrs in attributes.items()}
        for joint in self.worldbody.iter("joint"):
            joint.attrib.update(named_attributes[joint.attrib["name"]])
        meshdir = LfsPath("g1_urdf/meshes")
        for mesh in self.asset.findall("mesh"):
            mesh.set("file", str(meshdir / Path(mesh.attrib["file"]).name))
        expected = self.correct_naming(
            [name.split("/", 1)[1] + "_joint" for name in self.public_motor_names]
        )
        if self.actuators != expected:
            raise ValueError("G1 model actuator order must match the hardware motor order")
        site = self.correct_naming(self.control_imu)
        for kind, suffix in (("gyro", "gyro"), ("accelerometer", "accel")):
            ET.SubElement(
                self.sensor,
                kind,
                name=self.correct_naming(f"{self.control_imu}_{suffix}"),
                site=site,
            )
        ET.SubElement(
            self.sensor,
            "framequat",
            name=self.correct_naming(f"{self.control_imu}_quat"),
            objtype="site",
            objname=site,
        )

    def _replace_defaults_inline(
        self, default_dic: dict[str | None, dict[str, ET.Element]], root: ET.Element | None = None
    ) -> None:
        super()._replace_defaults_inline(default_dic=default_dic, root=root)
        if root is None:
            self._authored_appearance = {
                geom: {attribute: geom.get(attribute) for attribute in ("rgba", "material")}
                for geom in self.worldbody.iter("geom")
            }

    @property
    def sensors(self) -> list[str]:
        return [sensor.attrib["name"] for sensor in self.sensor]

    @property
    def init_qpos(self) -> NDArray[np.float64]:
        return np.asarray(G1_GROOT_HOME, dtype=np.float64).copy()

    @property
    def bottom_offset(self) -> NDArray[np.float64]:
        # set_base_xpos takes a support position; retain the asset's root height.
        return -np.asarray(self.base_offset, dtype=np.float64)


def g1_model(scene: Path, spawn: str = "default", *, idn: str | int = "g1") -> G1Model:
    """Build a G1 above a scene's support pose on the simulation worker."""
    model = G1Model(idn=idn)
    support = scene_spawn(scene, spawn)
    model.set_base_xpos(np.asarray(support.position.to_tuple()))
    model.set_base_ori(np.asarray(support.orientation.to_euler().to_tuple()))
    return model


def simulated_robot(config: G1WholeBodyConnectionConfig, scene: Path) -> SimulatedRobot:
    """Bind the existing G1 model to the retained connection's public motors."""
    return SimulatedRobot(
        robot_id="g1",
        model=partial(g1_model, scene),
        joint_names=G1Model.public_motor_names,
        frame_id=config.frame_id,
        control_imu=G1Model.control_imu,
        startup_kp=G1Model.startup_kp,
        startup_kd=G1Model.startup_kd,
        wait_for_first_command=True,
    )
