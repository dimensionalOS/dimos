"""Generate the repository-local A1Z MuJoCo scene from the vendor URDF.

The generator deliberately uses only the Python standard library and native
MuJoCo XML.  It can therefore be rerun after hydrating the LFS description:

    python -m dimos.robot.manipulators.a1z.simulation.generate

The generated XML keeps mesh paths relative to the XML file, rather than
depending on ROS package lookup or the current working directory.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import shutil
import tarfile
import tempfile
import xml.etree.ElementTree as ET

PACKAGE_DIR = Path(__file__).resolve().parent
REPO_ROOT = PACKAGE_DIR.parents[4]
DESCRIPTION_TAR = REPO_ROOT / "data/.lfs/a1z_description.tar.gz"
A1Z_SCENE_PATH = PACKAGE_DIR / "a1z_tabletop.xml"
ASSET_DIR = PACKAGE_DIR / "assets"
MAX_GRIPPER_TRAVEL = 0.015
A1Z_SIM_HOME = (0.0, 0.7, -1.2, 0.5, 0.0, 0.0)
ARM_POSITION_KP = 500.0
ARM_POSITION_KV = 45.0
GRIPPER_POSITION_KP = 200.0
GRIPPER_POSITION_KV = 8.0
LEFT_GRIPPER_AXIS = "0 1 0"
RIGHT_GRIPPER_AXIS = "0 -1 0"
PAD_JAW_AXIS = (-0.164, 0.9865, 0.0)
PAD_HALF_THICKNESS = 0.00075
PAD_CENTER_OFFSET = 0.00025
GRASP_ARM_TARGET = (0.08004403, 0.98357235, -0.27377253, 0.08290665, -0.10518632, 0.06622248)
ROBOT_BASE_XY = (0.42, 0.0)
NATIVE_TABLE_CENTER = (0.0, 0.0, 0.38462332)
NATIVE_PILLAR_POSITION = (0.139000448, -0.001852509, 0.44462332)
PRESENTATION_YAW = math.pi
Vec = tuple[float, float, float]


def _add(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return tuple(x + y for x, y in zip(a, b, strict=True))


def _sub(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return tuple(x - y for x, y in zip(a, b, strict=True))


def _scale(a: tuple[float, float, float], s: float) -> tuple[float, float, float]:
    return tuple(s * x for x in a)


def _rotate_about_robot_base(position: Vec) -> Vec:
    """Rotate a world position by the presentation yaw about the robot base."""
    relative = (position[0] - ROBOT_BASE_XY[0], position[1] - ROBOT_BASE_XY[1], position[2])
    rotated = _mat_vec(_axis_angle("0 0 1", PRESENTATION_YAW), relative)
    return (rotated[0] + ROBOT_BASE_XY[0], rotated[1] + ROBOT_BASE_XY[1], rotated[2])


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sum(x * y for x, y in zip(a, b, strict=True))


def _cross(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(a: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(_dot(a, a))
    return _scale(a, 1.0 / length)


def _mat_vec(
    matrix: tuple[tuple[float, float, float], ...], vector: tuple[float, float, float]
) -> tuple[float, float, float]:
    return tuple(_dot(row, vector) for row in matrix)


def _mat_mul(
    a: tuple[tuple[float, float, float], ...], b: tuple[tuple[float, float, float], ...]
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3)
    )


def _transpose(a: tuple[tuple[float, float, float], ...]) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def _rpy(value: str = "0 0 0") -> tuple[tuple[float, float, float], ...]:
    roll, pitch, yaw = (float(item) for item in value.split())
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    return (
        (cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx),
        (sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx),
        (-sy, cy * sx, cy * cx),
    )


def _axis_angle(axis: str, angle: float) -> tuple[tuple[float, float, float], ...]:
    x, y, z = (float(item) for item in axis.split())
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return (
        (t * x * x + c, t * x * y - s * z, t * x * z + s * y),
        (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
        (t * x * z - s * y, t * y * z + s * x, t * z * z + c),
    )


def _quat(matrix: tuple[tuple[float, float, float], ...]) -> str:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w, x, y, z = (
            0.25 * s,
            (matrix[2][1] - matrix[1][2]) / s,
            (matrix[0][2] - matrix[2][0]) / s,
            (matrix[1][0] - matrix[0][1]) / s,
        )
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        s = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2
        w, x, y, z = (
            (matrix[2][1] - matrix[1][2]) / s,
            0.25 * s,
            (matrix[0][1] + matrix[1][0]) / s,
            (matrix[0][2] + matrix[2][0]) / s,
        )
    elif matrix[1][1] > matrix[2][2]:
        s = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2
        w, x, y, z = (
            (matrix[0][2] - matrix[2][0]) / s,
            (matrix[0][1] + matrix[1][0]) / s,
            0.25 * s,
            (matrix[1][2] + matrix[2][1]) / s,
        )
    else:
        s = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2
        w, x, y, z = (
            (matrix[1][0] - matrix[0][1]) / s,
            (matrix[0][2] + matrix[2][0]) / s,
            (matrix[1][2] + matrix[2][1]) / s,
            0.25 * s,
        )
    return f"{w:.9f} {x:.9f} {y:.9f} {z:.9f}"


def _quat_matrix(
    quaternion: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], ...]:
    w, x, y, z = quaternion
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def _compiled_pad_specs(
    scene_path: Path,
) -> tuple[dict[str, tuple[str, str]], tuple[float, float, float]]:
    """Fit the explicitly modeled simulation pad inserts to compiled visuals."""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    data.qpos[:6] = GRASP_ARM_TARGET
    data.qpos[6:8] = MAX_GRIPPER_TRAVEL
    mujoco.mj_forward(model, data)
    fits: dict[str, tuple[str, str]] = {}
    face_centers: list[tuple[float, float, float]] = []
    fit_records: list[
        tuple[
            str,
            tuple[float, float, float],
            tuple[float, float, float],
            tuple[tuple[float, float, float], ...],
            tuple[float, float, float],
            tuple[tuple[float, float, float], ...],
        ]
    ] = []
    for finger, shoe in (
        ("gripper_finger_left_link", "left_pad"),
        ("gripper_finger_rIght_link", "right_pad"),
    ):
        visual_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{finger}_visual")
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, finger)
        mesh_id = int(model.geom_dataid[visual_id])
        start = int(model.mesh_vertadr[mesh_id])
        vertices = [
            tuple(float(value) for value in model.mesh_vert[start + i])
            for i in range(int(model.mesh_vertnum[mesh_id]))
        ]
        rotation = _quat_matrix(tuple(float(value) for value in model.geom_quat[visual_id]))
        geom_pos = tuple(float(value) for value in model.geom_pos[visual_id])
        body_vertices = [_add(_mat_vec(rotation, vertex), geom_pos) for vertex in vertices]
        faces_start = int(model.mesh_faceadr[mesh_id])
        face_count = int(model.mesh_facenum[mesh_id])
        inward_local = (0.0, 1.0, 0.0) if finger == "gripper_finger_left_link" else (0.0, -1.0, 0.0)
        body_rotation = tuple(
            tuple(float(value) for value in row) for row in data.xmat[body_id].reshape(3, 3)
        )
        body_position = tuple(float(value) for value in data.xpos[body_id])
        inward = _unit(_mat_vec(body_rotation, inward_local))
        vertical = _unit(_sub((0.0, 0.0, 1.0), _scale(inward, inward[2])))
        tangent = _unit(_cross(inward, vertical))
        pillar_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pillar_geom")
        pillar = tuple(float(value) for value in data.geom_xpos[pillar_id])
        support: list[tuple[float, tuple[float, float, float]]] = []
        for face in range(faces_start, faces_start + face_count):
            indices = [int(value) for value in model.mesh_face[face]]
            triangle = tuple(body_vertices[index] for index in indices)
            normal = _unit(_cross(_sub(triangle[1], triangle[0]), _sub(triangle[2], triangle[0])))
            world_triangle = [
                _add(body_position, _mat_vec(body_rotation, point)) for point in triangle
            ]
            if abs(_dot(normal, _mat_vec(_transpose(rotation), inward))) < 0.65:
                continue
            center = _scale(
                _add(_add(world_triangle[0], world_triangle[1]), world_triangle[2]), 1.0 / 3.0
            )
            support.append((_dot(center, inward), center))
        if len(support) < 3:
            raise ValueError(
                f"compiled visual has insufficient distal support for {finger}: {len(support)} points"
            )
        # The selected support is the first aperture-facing surface, independent
        # of vendor facet winding.  It is an explicit simulation fixture, not
        # vendor CAD: the insert is rendered and colliding with this same mesh.
        support.sort(key=lambda item: sum((item[1][axis] - pillar[axis]) ** 2 for axis in range(3)))
        selected = [point for _, point in support[: max(12, min(len(support), 64))]]
        face_center = tuple(
            sum(point[axis] for point in selected) / len(selected) for axis in range(3)
        )
        face_centers.append(face_center)
        inward = _unit(_sub(pillar, face_center))
        vertical = _unit(_sub((0.0, 0.0, 1.0), _scale(inward, inward[2])))
        tangent = _unit(_cross(inward, vertical))
        # MuJoCo geom matrices store the local axes as columns.  Build the
        # world basis explicitly in that convention: local X is tangential,
        # local Y is aperture-facing, and local Z is vertical.
        world_rotation = _transpose((tangent, inward, vertical))
        fit_records.append(
            (shoe, face_center, inward, body_rotation, body_position, world_rotation)
        )
    common_face_z = sum(center[2] for center in face_centers) / len(face_centers)
    jaw_axis = _unit(PAD_JAW_AXIS)
    for shoe, face_center, inward, body_rotation, body_position, world_rotation in fit_records:
        inward = jaw_axis if shoe == "left_pad" else _scale(jaw_axis, -1.0)
        vertical = _unit(_sub((0.0, 0.0, 1.0), _scale(inward, inward[2])))
        tangent = _unit(_cross(inward, vertical))
        world_rotation = _transpose((tangent, inward, vertical))
        symmetric_face_center = (face_center[0], face_center[1], common_face_z)
        # The insert's inner face is 0.5 mm inside the compiled visual while
        # its 1.5 mm thickness remains a shared rendered/colliding fixture.
        world_center = _add(symmetric_face_center, _scale(inward, PAD_CENTER_OFFSET))
        local_center = _mat_vec(_transpose(body_rotation), _sub(world_center, body_position))
        # Mesh geoms carry MuJoCo's fixed STL frame (local Y = mesh -X and
        # local X = mesh Y).  Compensate that frame when converting the
        # desired world basis to the geom quaternion.
        mesh_frame_transpose = ((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        local_rotation = _mat_mul(
            _mat_mul(_transpose(body_rotation), world_rotation), mesh_frame_transpose
        )
        fits[shoe] = (" ".join(f"{value:.9f}" for value in local_center), _quat(local_rotation))
    # The native tabletop/object arrangement is presented around the fixed
    # robot base, rather than by rotating the robot root frame.
    pillar_position = _rotate_about_robot_base(NATIVE_PILLAR_POSITION)
    return fits, pillar_position


def _pad_mesh_attributes() -> tuple[str, str]:
    """Return one rendered/colliding convex chamfered insert mesh."""
    half_x, half_z, half_y, chamfer = 0.005, 0.007, PAD_HALF_THICKNESS, 0.0008
    outline = (
        (-half_x + chamfer, -half_z),
        (half_x - chamfer, -half_z),
        (half_x, -half_z + chamfer),
        (half_x, half_z - chamfer),
        (half_x - chamfer, half_z),
        (-half_x + chamfer, half_z),
        (-half_x, half_z - chamfer),
        (-half_x, -half_z + chamfer),
    )
    vertices = [(x, -half_y, z) for x, z in outline] + [(x, half_y, z) for x, z in outline]
    faces = [(7, index, index + 1) for index in range(1, 7)]
    faces.extend((8, index + 1, index) for index in range(1, 7))
    faces.extend((index, (index + 1) % 8, (index + 1) % 8 + 8) for index in range(8))
    faces.extend((index, (index + 1) % 8 + 8, index + 8) for index in range(8))
    return (
        " ".join(f"{value:.7g}" for vertex in vertices for value in vertex),
        " ".join(f"{face[0]} {face[1]} {face[2]}" for face in faces),
    )


def _patch_urdf(path: Path) -> None:
    root = ET.parse(path).getroot()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    left = joints["gripper_finger_left_joint"]
    right = joints["gripper_finger_rIght_joint"]
    left.set("type", "prismatic")
    right.set("type", "prismatic")
    for joint, axis in ((left, LEFT_GRIPPER_AXIS), (right, RIGHT_GRIPPER_AXIS)):
        axis_element = joint.find("axis")
        if axis_element is None:
            axis_element = ET.SubElement(joint, "axis")
        axis_element.set("xyz", axis)
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")
        limit.attrib.update(lower="0", upper=str(MAX_GRIPPER_TRAVEL), effort="20", velocity="0.2")
    mimic = right.find("mimic")
    if mimic is None:
        mimic = ET.SubElement(right, "mimic")
    mimic.attrib.update(joint="gripper_finger_left_joint", multiplier="1", offset="0")
    ET.indent(root, space="  ")
    path.write_text(ET.tostring(root, encoding="unicode") + "\n")


def patch_description(tar_path: Path = DESCRIPTION_TAR) -> None:
    """Patch and deterministically repack the local vendor description."""
    with tempfile.TemporaryDirectory() as temporary:
        unpacked = Path(temporary)
        with tarfile.open(tar_path, "r:gz") as archive:
            archive.extractall(unpacked)
        _patch_urdf(unpacked / "a1z_description/A1Z_G1Z/urdf/A1Z_G1Z.urdf")
        temporary_tar = tar_path.with_suffix(".tmp.tar.gz")
        try:
            with tarfile.open(temporary_tar, "w:gz", compresslevel=9) as archive:
                for path in sorted(unpacked.rglob("*")):
                    archive.add(path, arcname=path.relative_to(unpacked))
            temporary_tar.replace(tar_path)
        finally:
            temporary_tar.unlink(missing_ok=True)


def _mesh_name(filename: str) -> str:
    return Path(filename).name


def _copy_meshes(unpacked: Path, asset_dir: Path) -> None:
    source = unpacked / "a1z_description/A1Z_G1Z/meshes"
    asset_dir.mkdir(parents=True, exist_ok=True)
    for mesh in source.glob("*.STL"):
        shutil.copyfile(mesh, asset_dir / mesh.name)


def _body(
    link_name: str,
    joint: ET.Element | None,
    link: ET.Element,
    children: dict[str, list[tuple[ET.Element, ET.Element]]],
) -> ET.Element:
    body = ET.Element("body", name=link_name)
    inertial = link.find("inertial")
    if inertial is not None:
        origin = inertial.find("origin")
        mass = inertial.find("mass")
        inertia = inertial.find("inertia")
        if mass is not None and inertia is not None:
            attributes = {"mass": mass.attrib["value"]}
            if origin is not None:
                attributes["pos"] = origin.attrib.get("xyz", "0 0 0")
            attributes["fullinertia"] = " ".join(
                inertia.attrib[key] for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
            )
            ET.SubElement(body, "inertial", attrib=attributes)
    if joint is not None:
        origin = joint.find("origin")
        if origin is not None:
            body.set("pos", origin.attrib.get("xyz", "0 0 0"))
            if "rpy" in origin.attrib:
                body.set("euler", origin.attrib["rpy"])
        if joint.attrib["type"] != "fixed":
            mj_type = "slide" if joint.attrib["type"] == "prismatic" else "hinge"
            joint_attributes = {"name": joint.attrib["name"], "type": mj_type, "damping": "1"}
            if joint.attrib["name"] == "arm_joint6":
                joint_attributes["damping"] = "10"
            if joint.attrib["name"] in ("gripper_finger_left_joint", "gripper_finger_rIght_joint"):
                joint_attributes.update(damping="20", armature="0.05")
            joint_element = ET.SubElement(body, "joint", attrib=joint_attributes)
            axis = joint.find("axis")
            if axis is not None:
                joint_element.set("axis", axis.attrib.get("xyz", "0 0 1"))
            limit = joint.find("limit")
            if limit is not None:
                joint_element.set(
                    "range",
                    f"{limit.attrib.get('lower', '-3.14')} {limit.attrib.get('upper', '3.14')}",
                )
            if joint.attrib["name"] in ("gripper_finger_left_joint", "gripper_finger_rIght_joint"):
                joint_element.set("margin", "0.0001")
    for visual in link.findall("visual"):
        mesh = visual.find("geometry/mesh")
        if mesh is not None:
            filename = _mesh_name(mesh.attrib["filename"])
            ET.SubElement(
                body,
                "geom",
                name=f"{link_name}_visual",
                type="mesh",
                mesh=filename,
                contype="0",
                conaffinity="0",
                mass="0",
            )
    for child_joint, child_link in children.get(link_name, []):
        body.append(_body(child_link.attrib["name"], child_joint, child_link, children))
    return body


def _scene_xml(
    urdf_path: Path,
    pad_fits: dict[str, tuple[str, str]] | None = None,
    pillar_position: tuple[float, float, float] | None = None,
    meshdir: str = "assets",
) -> ET.Element:
    urdf = ET.parse(urdf_path).getroot()
    links = {link.attrib["name"]: link for link in urdf.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in urdf.findall("joint")}
    children: dict[str, list[tuple[ET.Element, ET.Element]]] = {}
    child_names: set[str] = set()
    for joint in urdf.findall("joint"):
        parent_element = joint.find("parent")
        child_element = joint.find("child")
        if parent_element is None or child_element is None:
            raise ValueError(f"joint {joint.attrib.get('name', '<unnamed>')} has no parent/child")
        parent = parent_element.attrib["link"]
        child = child_element.attrib["link"]
        children.setdefault(parent, []).append((joint, links[child]))
        child_names.add(child)
    base = next(name for name in links if name not in child_names)
    root = ET.Element("mujoco", model="a1z_tabletop")
    ET.SubElement(root, "compiler", angle="radian", meshdir=meshdir, balanceinertia="true")
    ET.SubElement(
        root,
        "option",
        timestep="0.002",
        gravity="0 0 -9.81",
        integrator="implicitfast",
        cone="elliptic",
        impratio="10",
        noslip_iterations="20",
    )
    asset = ET.SubElement(root, "asset")
    for mesh in sorted(ASSET_DIR.glob("*.STL")):
        ET.SubElement(
            asset, "mesh", name=mesh.name, file=mesh.name, refpos="0 0 0", refquat="1 0 0 0"
        )
    pad_vertices, pad_faces = _pad_mesh_attributes()
    ET.SubElement(asset, "mesh", name="a1z_simulation_pad", vertex=pad_vertices, face=pad_faces)
    ET.SubElement(
        asset,
        "material",
        name="a1z_simulation_pad_material",
        rgba="0.08 0.55 0.95 1",
    )
    ET.SubElement(
        asset,
        "texture",
        name="pillar_texture",
        type="2d",
        builtin="flat",
        width="32",
        height="32",
        rgb1="0.9 0.2 0.05",
        rgb2="0.9 0.2 0.05",
    )
    ET.SubElement(asset, "material", name="pillar_material", texture="pillar_texture")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(
        world, "light", name="key_light", pos="1 -1 2.5", dir="-0.3 0.3 -1", directional="true"
    )
    table_center = _rotate_about_robot_base(NATIVE_TABLE_CENTER)
    table = ET.SubElement(
        world,
        "body",
        name="table",
        pos=" ".join(f"{value:.9f}" for value in table_center),
        euler="0 0 3.141592653589793",
    )
    ET.SubElement(
        table,
        "geom",
        name="tabletop",
        type="box",
        size="0.65 0.5 0.03",
        pos="0 0 0",
        friction="1.2 0.1 0.1",
        contype="1",
        conaffinity="7",
    )
    ET.SubElement(
        table, "geom", name="table_leg", type="box", size="0.05 0.05 0.38", pos="0.5 0  -0.38"
    )
    arm = ET.SubElement(world, "body", name="a1z", pos="0.42 0 0.41")
    arm.append(_body(base, None, links[base], children))
    # Derived from the midpoint of the compiled distal inner-face vertices at
    # GRASP_ARM_TARGET/open gripper pose.
    pillar_pos = pillar_position or _rotate_about_robot_base(NATIVE_PILLAR_POSITION)
    pillar = ET.SubElement(
        world, "body", name="pillar", pos=" ".join(f"{value:.9f}" for value in pillar_pos)
    )
    ET.SubElement(pillar, "freejoint", name="pillar_free")
    ET.SubElement(
        pillar,
        "geom",
        name="pillar_geom",
        type="cylinder",
        size="0.01 0.03",
        mass="0.05",
        material="pillar_material",
        contype="4",
        conaffinity="7",
        condim="4",
        friction="0.8 0.005 0.0001",
        margin="0.0001",
    )
    contact = ET.SubElement(root, "contact")
    ET.SubElement(
        contact,
        "pair",
        geom1="pillar_geom",
        geom2="tabletop",
        condim="6",
        friction="0.8 0.8 0.005 0.005 0.005",
        margin="0.0001",
        solref="0.005 1",
        solimp="0.99 0.999 0.0001",
    )
    # Shoes are fitted from the compiler mesh triangles, in each finger's
    # local frame.  Their aperture-facing face is 0.25mm outside that fit.
    for finger_name, pad_name in (
        ("gripper_finger_left_link", "left_pad"),
        ("gripper_finger_rIght_link", "right_pad"),
    ):
        finger_body = next(
            element for element in arm.iter("body") if element.attrib.get("name") == finger_name
        )
        visual = next(iter(links[finger_name].findall("visual")))
        mesh = visual.find("geometry/mesh")
        if mesh is None:
            raise ValueError(f"finger visual has no mesh: {finger_name}")
        if pad_fits is not None:
            pad_pos, pad_quat = pad_fits[pad_name]
        else:
            continue
        attributes = dict(
            name=pad_name,
            type="mesh",
            mesh="a1z_simulation_pad",
            quat=pad_quat,
            pos=pad_pos,
            contype="2",
            conaffinity="5",
            condim="4",
            friction="2 0.01 0.001",
            margin="0.001",
            priority="1",
            solimp="0.95 0.99 0.001",
            solref="0.02 1",
            mass="0",
            material="a1z_simulation_pad_material",
        )
        ET.SubElement(finger_body, "geom", attrib=attributes)
    sensor_body = next(
        element for element in arm.iter("body") if element.attrib.get("name") == "arm_link6"
    )
    ET.SubElement(
        sensor_body,
        "camera",
        name="wrist_camera",
        pos="0.11 0 -0.02",
        euler="0 1.57 1.57",
        mode="fixed",
        fovy="65",
    )
    actuator = ET.SubElement(root, "actuator")
    for joint_name in [f"arm_joint{i}" for i in range(1, 7)]:
        limit = joints[joint_name].find("limit")
        if limit is None or "effort" not in limit.attrib:
            raise ValueError(f"joint {joint_name} has no authored limit/effort")
        lower = limit.attrib.get("lower", "-3.14")
        upper = limit.attrib.get("upper", "3.14")
        effort = float(limit.attrib["effort"])
        ET.SubElement(
            actuator,
            "position",
            name=f"{joint_name}_motor",
            joint=joint_name,
            kp=str(ARM_POSITION_KP),
            kv=str(ARM_POSITION_KV),
            ctrllimited="true",
            ctrlrange=f"{lower} {upper}",
            forcelimited="true",
            forcerange=f"{-effort:g} {effort:g}",
        )
    tendon = ET.SubElement(root, "tendon")
    split = ET.SubElement(tendon, "fixed", name="finger_split")
    ET.SubElement(split, "joint", joint="gripper_finger_left_joint", coef="0.5")
    ET.SubElement(split, "joint", joint="gripper_finger_rIght_joint", coef="0.5")
    ET.SubElement(
        actuator,
        "position",
        name="gripper_motor",
        tendon="finger_split",
        kp=str(GRIPPER_POSITION_KP),
        kv=str(GRIPPER_POSITION_KV),
        ctrlrange="0 0.015",
        forcelimited="true",
        forcerange="-20 20",
    )
    equality = ET.SubElement(root, "equality")
    ET.SubElement(
        equality,
        "joint",
        name="gripper_mimic",
        joint1="gripper_finger_rIght_joint",
        joint2="gripper_finger_left_joint",
        polycoef="0 1 0 0 0",
        solref="0.005 1",
        solimp="0.95 0.99 0.001",
    )
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="640", offheight="480")
    custom = ET.SubElement(root, "custom")
    ET.SubElement(custom, "numeric", name="wrist_camera_fps", data="30")
    keyframe = ET.SubElement(root, "keyframe")
    home_qpos = (*A1Z_SIM_HOME, 0.0, 0.0, *pillar_pos, 1.0, 0.0, 0.0, 0.0)
    ET.SubElement(keyframe, "key", name="home", qpos=" ".join(map(str, home_qpos)))
    ET.indent(root, space="  ")
    return root


def generate_scene(output: Path = A1Z_SCENE_PATH, description_tar: Path = DESCRIPTION_TAR) -> Path:
    """Generate the final scene and its relative mesh assets."""
    with tempfile.TemporaryDirectory() as temporary:
        unpacked = Path(temporary)
        with tarfile.open(description_tar, "r:gz") as archive:
            archive.extractall(unpacked)
        urdf_path = unpacked / "a1z_description/A1Z_G1Z/urdf/A1Z_G1Z.urdf"
        _patch_urdf(urdf_path)
        _copy_meshes(unpacked, ASSET_DIR)
        runtime = Path.home() / ".cache/opencode/runtime/a1z_collision_fit"
        runtime.mkdir(parents=True, exist_ok=True)
        base_path = runtime / "a1z_base.xml"
        base_path.write_text(
            ET.tostring(_scene_xml(urdf_path, meshdir=str(ASSET_DIR)), encoding="unicode") + "\n"
        )
        pad_fits, pillar_position = _compiled_pad_specs(base_path)
        # The checked-in scene is portable; only the temporary compiler input
        # needs an absolute mesh directory.
        root = _scene_xml(urdf_path, pad_fits, pillar_position, meshdir="assets")
        candidate = runtime / "a1z_final.xml"
        compiled_root = _scene_xml(urdf_path, pad_fits, pillar_position, meshdir=str(ASSET_DIR))
        candidate.write_text(ET.tostring(compiled_root, encoding="unicode") + "\n")
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(candidate))
        pad_ids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            for name in ("left_pad", "right_pad")
        ]
        if (
            any(pad_id < 0 for pad_id in pad_ids)
            or len({int(model.geom_dataid[pad_id]) for pad_id in pad_ids}) != 1
        ):
            raise ValueError("final A1Z pad scene did not compile with one shared pad mesh")
    temporary_output = output.with_suffix(output.suffix + ".tmp")
    try:
        temporary_output.write_text(ET.tostring(root, encoding="unicode") + "\n")
        os.replace(temporary_output, output)
    finally:
        temporary_output.unlink(missing_ok=True)
    return output


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repack-description", action="store_true")
    parser.add_argument("--output", type=Path, default=A1Z_SCENE_PATH)
    args = parser.parse_args()
    if args.repack_description:
        patch_description()
    generate_scene(args.output)


if __name__ == "__main__":
    main()
