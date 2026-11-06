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

"""Import shared physical geometry; no sensor profiles or sensor algorithms."""

import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.sim2.description import SceneConfig


def build_scene(
    world: SceneConfig, dt_ns: int = 2_000_000, *, kinematic: bool = False
) -> mujoco.MjSpec:
    if not np.isfinite(dt_ns / 1e9) or not 0 < (dt_ns / 1e9) <= 0.02:
        raise ValueError("dt must be in (0, .02] seconds")
    spec = mujoco.MjSpec()
    spec.option.timestep = dt_ns / 1e9
    spec.option.gravity = world.gravity_world
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.compiler.fusestatic = False
    if world.ground_plane:
        spec.worldbody.add_geom(
            name="floor",
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            size=[4, 4, 0.05],
            rgba=[0.25, 0.3, 0.35, 1],
        )
    spec.worldbody.add_light(
        pos=[0, -3, 5],
        dir=[0, 0, -1],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        diffuse=[0.8, 0.8, 0.8],
    )
    spec.visual.headlight.ambient = [0.35, 0.35, 0.35]
    spec.visual.headlight.diffuse = [0.45, 0.45, 0.45]
    spec.add_texture(
        name="sky",
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        width=256,
        height=1536,
        rgb1=[0.65, 0.73, 0.8],
        rgb2=[0.85, 0.88, 0.9],
    )
    ids: set[str] = set()
    for instance in world.instances:
        if instance.id in ids:
            raise ValueError(f"duplicate instance {instance.id}")
        ids.add(instance.id)
        model = instance.description
        if any(
            j.type not in {"fixed", "prismatic", "revolute", "continuous"} or j.mimic
            for j in model.joints
        ):
            raise ValueError("prototype supports fixed and independent scalar URDF joints")
        # Import flags must be present before parsing (setting discardvisual afterwards is too late).
        xml = ET.fromstring(model.xml)
        for extension in xml.findall("mujoco"):
            xml.remove(extension)
        extension = ET.SubElement(xml, "mujoco")
        ET.SubElement(extension, "compiler", fusestatic="false", discardvisual="false")
        for link in xml.findall("link"):
            # Optical queries use visuals where available. Avoid compiling a
            # second, unused copy of a large environment's collision geometry.
            if kinematic and link.findall("visual"):
                for collision in link.findall("collision"):
                    link.remove(collision)
            for shape_kind in ("visual", "collision"):
                for index, shape in enumerate(link.findall(shape_kind)):
                    shape.set("name", f"{instance.id}/{link.attrib['name']}/{shape_kind}/{index}")
        child = mujoco.MjSpec.from_string(ET.tostring(xml, encoding="unicode"))
        if instance.floating:
            child.body(model.root_link).add_freejoint(name=model.root_link + "/free")
        for drive in instance.drives:
            joint = model.get_joint(drive.joint)
            if joint is None or joint.type == "fixed" or joint.effort is None:
                raise ValueError(f"drive requires a scalar joint with effort limit: {drive.joint}")
            if (
                not np.isfinite(
                    [drive.kp, drive.kd, drive.initial, drive.armature, drive.friction]
                ).all()
                or drive.kp <= 0
                or min(drive.kd, drive.armature, drive.friction) < 0
            ):
                raise ValueError("invalid servo gains")
            if (joint.lower is not None and drive.initial < joint.lower) or (
                joint.upper is not None and drive.initial > joint.upper
            ):
                raise ValueError(f"initial position outside joint limits: {drive.joint}")
            child.joint(drive.joint).armature = drive.armature
            child.joint(drive.joint).frictionloss = drive.friction
            child.add_actuator(
                name=drive.joint,
                target=drive.joint,
                trntype=mujoco.mjtTrn.mjTRN_JOINT,
                gainprm=[drive.kp] + [0] * 9,
                biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                biasprm=[0, -drive.kp, -drive.kd] + [0] * 7,
                forcelimited=True,
                forcerange=[-joint.effort, joint.effort],
            )
        # MuJoCo's URDF importer retains colors but drops diffuse texture references.
        # Restore those from the same shared URDF; do not author a second visual asset.
        for visual in xml.findall(".//visual"):
            texture = visual.find("material/texture")
            if texture is None:
                continue
            name = visual.get("name")
            if not name:
                raise ValueError("textured URDF visuals must have names")
            child.add_texture(
                name=name + "/texture",
                type=mujoco.mjtTexture.mjTEXTURE_2D,
                file=texture.attrib["filename"],
            )
            textures = [""] * int(mujoco.mjtTextureRole.mjNTEXROLE)
            textures[int(mujoco.mjtTextureRole.mjTEXROLE_RGB)] = name + "/texture"
            child.add_material(name=name + "/material", textures=textures)
            child.geom(name).material = name + "/material"
        spec.attach(
            child,
            prefix="",
            frame=spec.worldbody.add_frame(
                pos=instance.xyz,
                quat=Rotation.from_euler("xyz", instance.rpy).as_quat(scalar_first=True),
            ),
        )
    if kinematic:
        # A query scene: every described body is independently positioned from
        # the observation stream. There are no joints, motors or physics state
        # coordinates to reconstruct. Geometry/materials retain their body-local
        # transforms; camera mounts retain their local optical-frame transform.
        xml = ET.fromstring(spec.to_xml())
        # Attached URDF specs emit empty unnamed child defaults, which the
        # XML parser rejects even though the in-memory spec compiles them.
        for defaults in xml.iter("default"):
            for child_default in list(defaults.findall("default")):
                if not child_default.attrib and not len(child_default):
                    defaults.remove(child_default)
        root = xml.find("worldbody")
        assert root is not None
        bodies = list(root.iter("body"))
        for parent in list(root.iter()):
            for element in list(parent):
                if element.tag in {"body", "joint", "freejoint"}:
                    parent.remove(element)
        for body in bodies:
            body.set("mocap", "true")
            body.set("pos", "0 0 0")
            body.set("quat", "1 0 0 0")
            root.append(body)
        for tag in ("actuator", "sensor", "tendon", "equality", "contact"):
            for element in xml.findall(tag):
                xml.remove(element)
        return mujoco.MjSpec.from_string(ET.tostring(xml, encoding="unicode"))
    return spec


def compile_scene(world: SceneConfig, dt_ns: int = 2_000_000) -> mujoco.MjModel:
    return build_scene(world, dt_ns).compile()
