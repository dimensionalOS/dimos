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

"""Physics-backed, parked-base ACT deployment check for R1Pro.

This is a simulation approximation, not a calibrated hardware dynamics model:
position servos use gravity compensation, steering/wheels and grippers are
fixed, and the camera is an external overview. Mesh contacts remain enabled.
"""

from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import mujoco

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.spec import JointLimits
from dimos.imitation.policy.lerobot.module import R1ProLeRobotPolicy
from dimos.imitation.policy.module import POLICY_ROLLOUT_INSTANCE_NAME, POLICY_ROLLOUT_TASK_NAME
from dimos.imitation.policy.skills import PolicySkills
from dimos.robot.galaxea.r1pro.config import R1PRO_COLLISION_EXCLUSIONS, R1PRO_MODEL
from dimos.robot.galaxea.r1pro.learning import R1PRO_SIM_ACT_JOINTS
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.simulation.engines.robot_sim_binding import RobotSimSpec
from dimos.simulation.mujoco.scene_package_entity_composer import add_scene_package_entities_to_spec
from dimos.simulation.scene_assets.spec import load_scene_package


def prepare_r1pro_act_scene(output: Path, *, scene_package: Path | None = None) -> Path:
    """Materialize the pinned vendor model and optional house into a local MJCF.

    Call explicitly before starting the blueprint. The output path also uniquely
    identifies this simulation's shared-memory bridge; use one path per run.
    """
    urdf = ET.fromstring(R1PRO_MODEL.load().xml)
    ET.SubElement(
        ET.SubElement(urdf, "mujoco"), "compiler", discardvisual="false", fusestatic="false"
    )
    model = mujoco.MjModel.from_xml_string(ET.tostring(urdf, encoding="unicode"))
    with TemporaryDirectory(prefix="r1pro-mjcf-") as directory:
        raw_path = Path(directory) / "robot.xml"
        mujoco.mj_saveLastXML(str(raw_path), model)  # type: ignore[attr-defined]
        root = ET.parse(raw_path).getroot()
    ET.SubElement(root, "option", timestep="0.002", integrator="implicitfast")
    actuators = ET.SubElement(root, "actuator")
    joints = {joint.attrib["name"]: joint for joint in root.findall(".//joint")}
    if set(joints) != set(R1PRO_SIM_ACT_JOINTS):
        raise ValueError("R1Pro MuJoCo joint set differs from the upper-body ACT contract")
    for name in R1PRO_SIM_ACT_JOINTS:
        joint = joints[name]
        joint.set("damping", "2")
        joint.set("armature", "0.02")
        ET.SubElement(
            actuators,
            "position",
            name=name,
            joint=name,
            kp="250",
            kv="20",
            ctrlrange=joint.attrib["range"],
        )
    for body in root.findall(".//body"):
        body.set("gravcomp", "1")
    contact = ET.SubElement(root, "contact")
    # The imported convex base hull overlaps the adjacent torso pivot by 12 cm
    # at home. Exclude that structural pair, as well as planning's known pairs.
    for first, second in [*R1PRO_COLLISION_EXCLUSIONS, ("base_link", "torso_link1")]:
        ET.SubElement(contact, "exclude", body1=first, body2=second)
    world = root.find("worldbody")
    assert world is not None
    ET.SubElement(world, "light", pos="0 0 3", diffuse=".8 .8 .8")
    ET.SubElement(
        world, "camera", name="overview", pos="2.5 -2 2", xyaxes=".625 .781 0 -.268 .214 .939"
    )
    if scene_package is None:
        ET.SubElement(world, "geom", name="floor", type="plane", size="5 5 .1", rgba=".2 .25 .3 1")
    robot = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
    robot.default.name = "r1pro_sim"
    if scene_package is not None:
        metadata = scene_package / "scene.meta.json" if scene_package.is_dir() else scene_package
        package = load_scene_package(metadata)
        if package.mujoco_scene_path is None:
            raise ValueError("Scene package has no MuJoCo scene")
        scene = mujoco.MjSpec.from_file(str(package.mujoco_scene_path))
        scene.meshdir = str(package.mujoco_scene_path.parent / scene.meshdir)
        scene.texturedir = str(package.mujoco_scene_path.parent / scene.texturedir)  # type: ignore[attr-defined]
        scene.option.timestep = robot.option.timestep
        scene.option.integrator = robot.option.integrator
        scene.attach(robot, prefix="", frame=scene.worldbody.add_frame())
        add_scene_package_entities_to_spec(scene, package.entities)
    else:
        scene = robot
    # Compilation catches incompatible assets and missing actuator targets before
    # any coordinator or policy process starts. No scene bodies are welded here.
    scene.compile()
    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(scene.to_xml())
    mujoco.MjModel.from_xml_path(str(output))
    return output


def build_r1pro_act_sim(
    *, scene_path: Path, artifact: str, device: str = "cuda", headless: bool = False
) -> Blueprint:
    """Wire real ACT inference to simulated position servos through the coordinator.

    Native MuJoCo display is the default. A supplied checkpoint must match
    R1PRO_SIM_ACT_IO; deployment is idle until the policy is explicitly started.
    """
    scene_path = scene_path.expanduser().resolve()
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    ranges = [model.jnt_range[model.joint(name).id] for name in R1PRO_SIM_ACT_JOINTS]
    hardware = HardwareComponent(
        hardware_id="r1pro",
        hardware_type=HardwareType.WHOLE_BODY,
        joints=list(R1PRO_SIM_ACT_JOINTS),
        adapter_type="sim_mujoco_whole_body",
        address=scene_path,
        auto_enable=True,
        adapter_kwargs={"num_motors": len(R1PRO_SIM_ACT_JOINTS), "command_mode": "position"},
        limits=JointLimits(
            position_lower=[float(bounds[0]) for bounds in ranges],
            position_upper=[float(bounds[1]) for bounds in ranges],
            velocity_max=[0.5] * len(ranges),
        ),
    )
    return autoconnect(
        MujocoSimModule.blueprint(
            address=scene_path,
            dof=len(R1PRO_SIM_ACT_JOINTS),
            headless=headless,
            camera_name="overview",
            width=320,
            height=240,
            fps=30,
            base_frame_id="world",
            enable_depth=False,
            enable_pointcloud=False,
            robot_sim_spec=RobotSimSpec(
                robot_id="r1pro",
                hardware_joints=R1PRO_SIM_ACT_JOINTS,
                model_joint_names=R1PRO_SIM_ACT_JOINTS,
                model_actuator_names=R1PRO_SIM_ACT_JOINTS,
                root_body_names=("base_link",),
            ),
        ),
        ControlCoordinator.blueprint(
            hardware=[hardware],
            tasks=[
                TaskConfig(
                    name=POLICY_ROLLOUT_TASK_NAME,
                    type="trajectory",
                    joint_names=list(R1PRO_SIM_ACT_JOINTS),
                    priority=30,
                    params={"start_position_tolerance": 0.05},
                )
            ],
        ),
        R1ProLeRobotPolicy.blueprint(
            instance_name=POLICY_ROLLOUT_INSTANCE_NAME,
            artifact=artifact,
            task="R1Pro upper-body deployment diagnostic",
            device=device,
            startup_timeout=120.0,
        ),
        PolicySkills.blueprint(),
    )
