"""Generate the TARS MJCF (and optionally URDF + STL meshes) from tars_sdk.model.params.

Usage:
    python -m tars_sdk.model.generate                  # rewrite the packaged tars.xml
    python -m tars_sdk.model.generate --urdf out/      # also export URDF + STL meshes (needs trimesh)
"""

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
from xml.dom import minidom
from xml.etree import ElementTree as ET

import numpy as np

from tars_sdk.model import MJCF_PATH
from tars_sdk.model.params import Params, poses

MATERIALS = {
    "aluminum": (0.52, 0.53, 0.55, 1.0),
    "sleeve": (0.45, 0.46, 0.48, 1.0),
    "seam": (0.08, 0.08, 0.09, 1.0),
    "rubber": (0.05, 0.05, 0.05, 1.0),
    "display": (0.02, 0.05, 0.06, 1.0),
    "axle": (0.25, 0.25, 0.27, 1.0),
}


@dataclass
class Box:
    name: str
    size: tuple[float, float, float]  # full extents
    pos: tuple[float, float, float]
    material: str
    collide: bool = False


@dataclass
class Link:
    name: str
    boxes: list[Box]
    mass: float
    com: tuple[float, float, float]
    inertia: tuple[float, float, float]  # diagonal, about com


def box_inertia(m, sx, sy, sz):
    return (m * (sy**2 + sz**2) / 12, m * (sx**2 + sz**2) / 12, m * (sx**2 + sy**2) / 12)


def slab_links(p: Params, i: int) -> tuple[Link, Link]:
    d, w = p.slab_depth, p.slab_width
    top, bot = p.pivot_from_top, p.pivot_from_top - p.upper_len
    # upper segment, frame at hinge axis
    zc = (top + bot) / 2
    ub = [Box(f"slab_{i}_upper_body", (d, w, p.upper_len), (0, 0, zc), "aluminum", collide=True)]
    for k, s in enumerate(p.seams_from_top):
        for side, x in (("front", d / 2), ("back", -d / 2)):
            ub.append(
                Box(
                    f"slab_{i}_seam_{side}_{k}",
                    (0.004, w - 0.02, p.seam_height),
                    (x, 0, top - s),
                    "seam",
                )
            )
    if i == p.display_slab:
        ub.append(
            Box(
                f"slab_{i}_display",
                (0.006, p.display_w, p.display_h),
                (d / 2, 0, top - p.display_from_top),
                "display",
            )
        )
    upper = Link(
        f"slab_{i}_upper",
        ub,
        p.upper_mass,
        (0, 0, zc),
        box_inertia(p.upper_mass, d, w, p.upper_len),
    )

    # lower segment, frame at the bottom of the upper segment (slide origin)
    L, si = p.lower_len, p.sleeve_inset
    lb = [
        Box(
            f"slab_{i}_lower_body",
            (d, w, L - p.foot_pad),
            (0, 0, -(L - p.foot_pad) / 2),
            "aluminum",
            collide=True,
        ),
        Box(
            f"slab_{i}_foot_pad",
            (d, w, p.foot_pad),
            (0, 0, -L + p.foot_pad / 2),
            "rubber",
            collide=True,
        ),
        Box(
            f"slab_{i}_sleeve",
            (d - 2 * si, w - 2 * si, p.sleeve_len),
            (0, 0, p.sleeve_len / 2),
            "sleeve",
        ),
    ]
    lower = Link(
        f"slab_{i}_lower", lb, p.lower_mass, (0, 0, -L / 2), box_inertia(p.lower_mass, d, w, L)
    )
    return upper, lower


def hub_link(p: Params) -> Link:
    r, L, m = p.hub_radius, p.span, p.hub_mass
    ixx = m * (3 * r**2 + L**2) / 12
    return Link("base_link", [], m, (0, 0, 0), (ixx, m * r**2 / 2, ixx))


def fmt(v) -> str:
    return " ".join(f"{x:.6g}" for x in v)


# ---------------------------------------------------------------- STL
def link_mesh(link: Link):  # -> trimesh.Trimesh
    import trimesh

    parts = []
    for b in link.boxes:
        t = trimesh.creation.box(extents=b.size)
        t.apply_translation(b.pos)
        parts.append(t)
    return trimesh.util.concatenate(parts)


def hub_mesh(p: Params):  # -> trimesh.Trimesh
    import trimesh

    t = trimesh.creation.cylinder(radius=p.hub_radius, height=p.span, sections=48)
    t.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    return t


# ---------------------------------------------------------------- URDF
def build_urdf(p: Params, links: list[tuple[Link, Link]]) -> ET.Element:
    robot = ET.Element("robot", name="tars")
    for name, rgba in MATERIALS.items():
        ET.SubElement(ET.SubElement(robot, "material", name=name), "color", rgba=fmt(rgba))

    def add_link(link: Link, mesh: str, extra_visuals: list[Box]):
        el = ET.SubElement(robot, "link", name=link.name)
        inertial = ET.SubElement(el, "inertial")
        ET.SubElement(inertial, "origin", xyz=fmt(link.com), rpy="0 0 0")
        ET.SubElement(inertial, "mass", value=f"{link.mass:.6g}")
        ixx, iyy, izz = link.inertia
        ET.SubElement(
            inertial,
            "inertia",
            ixx=f"{ixx:.6g}",
            iyy=f"{iyy:.6g}",
            izz=f"{izz:.6g}",
            ixy="0",
            ixz="0",
            iyz="0",
        )
        vis = ET.SubElement(el, "visual", name=f"{link.name}_mesh")
        ET.SubElement(vis, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(ET.SubElement(vis, "geometry"), "mesh", filename=f"meshes/{mesh}")
        ET.SubElement(vis, "material", name="aluminum")
        for b in extra_visuals:
            v = ET.SubElement(el, "visual", name=f"{b.name}_vis")
            ET.SubElement(v, "origin", xyz=fmt(b.pos), rpy="0 0 0")
            ET.SubElement(ET.SubElement(v, "geometry"), "box", size=fmt(b.size))
            ET.SubElement(v, "material", name=b.material)
        for b in link.boxes:
            if b.collide:
                c = ET.SubElement(el, "collision", name=b.name)
                ET.SubElement(c, "origin", xyz=fmt(b.pos), rpy="0 0 0")
                ET.SubElement(ET.SubElement(c, "geometry"), "box", size=fmt(b.size))
        return el

    def add_joint(name, jtype, parent, child, xyz, axis=None, limit=None, damping=None):
        j = ET.SubElement(robot, "joint", name=name, type=jtype)
        ET.SubElement(j, "parent", link=parent)
        ET.SubElement(j, "child", link=child)
        ET.SubElement(j, "origin", xyz=fmt(xyz), rpy="0 0 0")
        if axis:
            ET.SubElement(j, "axis", xyz=fmt(axis))
        if limit:
            ET.SubElement(j, "limit", **{k: f"{v:.6g}" for k, v in limit.items()})
        if damping is not None:
            ET.SubElement(j, "dynamics", damping=f"{damping:.6g}", friction="0")

    add_link(hub_link(p), "base_link.stl", [])
    ET.SubElement(robot, "link", name="imu_link")
    add_joint("imu_joint", "fixed", "base_link", "imu_link", (0, 0, 0))

    for i, (upper, lower) in enumerate(links, start=1):
        details = [b for b in upper.boxes if b.material in ("seam", "display")]
        add_link(upper, f"{upper.name}.stl", details)
        add_link(lower, f"{lower.name}.stl", [b for b in lower.boxes if b.material == "rubber"])
        add_joint(
            f"slab_{i}_hinge",
            "revolute",
            "base_link",
            upper.name,
            (0, p.slab_y(i), 0),
            (0, 1, 0),
            dict(
                lower=-p.hinge_limit,
                upper=p.hinge_limit,
                effort=p.hinge_effort,
                velocity=p.hinge_velocity,
            ),
            p.hinge_damping,
        )
        add_joint(
            f"slab_{i}_slide",
            "prismatic",
            upper.name,
            lower.name,
            (0, 0, p.pivot_from_top - p.upper_len),
            (0, 0, -1),
            dict(lower=0, upper=p.slide_travel, effort=p.slide_effort, velocity=p.slide_velocity),
            p.slide_damping,
        )
        if i == p.display_slab:
            ET.SubElement(robot, "link", name="camera_link")
            add_joint(
                "camera_joint",
                "fixed",
                upper.name,
                "camera_link",
                (p.slab_depth / 2 + 0.005, 0, p.pivot_from_top - p.camera_from_top),
            )
    return robot


# ---------------------------------------------------------------- MJCF
def build_mjcf(p: Params, links: list[tuple[Link, Link]], standalone: bool = True) -> ET.Element:
    """standalone=False emits only the robot (no floor, lights, sky, keyframes) for attaching
    into another scene."""
    m = ET.Element("mujoco", model="tars")
    ET.SubElement(m, "compiler", angle="radian", autolimits="true")
    ET.SubElement(m, "option", timestep=f"{p.timestep:.6g}", integrator="implicitfast")

    if standalone:
        vis = ET.SubElement(m, "visual")
        ET.SubElement(
            vis, "global", offwidth="1920", offheight="1440", azimuth="150", elevation="-15"
        )
        ET.SubElement(vis, "quality", shadowsize="8192", offsamples="8")
        ET.SubElement(
            vis,
            "headlight",
            ambient="0.15 0.15 0.15",
            diffuse="0.3 0.3 0.3",
            specular="0.1 0.1 0.1",
        )
        ET.SubElement(vis, "map", znear="0.01", haze="0.05")

    asset = ET.SubElement(m, "asset")
    if standalone:
        ET.SubElement(
            asset,
            "texture",
            type="skybox",
            builtin="gradient",
            rgb1="0.55 0.62 0.72",
            rgb2="0.08 0.09 0.12",
            width="512",
            height="512",
        )
        ET.SubElement(
            asset,
            "texture",
            name="floor",
            type="2d",
            builtin="checker",
            rgb1="0.78 0.72 0.6",
            rgb2="0.7 0.64 0.53",
            width="512",
            height="512",
            mark="edge",
            markrgb="0.6 0.55 0.45",
        )
        ET.SubElement(
            asset, "material", name="floor", texture="floor", texrepeat="8 8", reflectance="0.05"
        )
    spec = {
        "aluminum": ("0.6", "0.7", "0.15"),
        "sleeve": ("0.5", "0.6", "0.1"),
        "display": ("0.9", "0.9", "0.3"),
    }
    for name, rgba in MATERIALS.items():
        s, sh, r = spec.get(name, ("0.2", "0.2", "0"))
        attrs = dict(name=name, rgba=fmt(rgba), specular=s, shininess=sh, reflectance=r)
        if name == "display":
            attrs["emission"] = "0.25"
            attrs["rgba"] = "0.05 0.35 0.4 1"
        ET.SubElement(asset, "material", **attrs)

    default = ET.SubElement(m, "default")
    ET.SubElement(
        default,
        "geom",
        friction=f"{p.foot_friction:.6g} {p.torsional_friction:.6g} {p.rolling_friction:.6g}",
        solref=f"{p.contact_timeconst:.6g} 1",
    )
    ET.SubElement(
        ET.SubElement(default, "default", **{"class": "visual"}),
        "geom",
        contype="0",
        conaffinity="0",
        group="1",
    )
    ET.SubElement(ET.SubElement(default, "default", **{"class": "collision"}), "geom", group="0")
    h = ET.SubElement(default, "default", **{"class": "hinge"})
    ET.SubElement(
        h,
        "joint",
        type="hinge",
        axis="0 1 0",
        range=f"{-p.hinge_limit:.6g} {p.hinge_limit:.6g}",
        damping=f"{p.hinge_damping:.6g}",
        armature=f"{p.hinge_armature:.6g}",
    )
    ET.SubElement(h, "motor", ctrlrange=f"{-p.hinge_effort:.6g} {p.hinge_effort:.6g}")
    s = ET.SubElement(default, "default", **{"class": "slide"})
    ET.SubElement(
        s,
        "joint",
        type="slide",
        axis="0 0 -1",
        range=f"0 {p.slide_travel:.6g}",
        damping=f"{p.slide_damping:.6g}",
        armature=f"{p.slide_armature:.6g}",
    )
    ET.SubElement(s, "motor", ctrlrange=f"{-p.slide_effort:.6g} {p.slide_effort:.6g}")

    world = ET.SubElement(m, "worldbody")
    if standalone:
        ET.SubElement(
            world,
            "light",
            name="key",
            pos="3 -2 5",
            dir="-0.5 0.35 -1",
            diffuse="0.7 0.7 0.7",
            castshadow="true",
        )
        ET.SubElement(
            world,
            "light",
            name="fill",
            pos="-3 3 4",
            dir="0.5 -0.5 -1",
            diffuse="0.25 0.25 0.3",
            castshadow="false",
        )
        ET.SubElement(world, "geom", name="floor", type="plane", size="0 0 0.05", material="floor")

    hub = hub_link(p)
    base = ET.SubElement(world, "body", name="base_link", pos=f"0 0 {p.pivot_height:.6g}")
    ET.SubElement(base, "freejoint", name="root")
    ET.SubElement(
        base, "inertial", pos="0 0 0", mass=f"{hub.mass:.6g}", diaginertia=fmt(hub.inertia)
    )
    ET.SubElement(
        base,
        "geom",
        name="axle",
        type="cylinder",
        size=f"{p.hub_radius:.6g} {p.span / 2:.6g}",
        zaxis="0 1 0",
        material="axle",
        **{"class": "visual"},
    )
    ET.SubElement(base, "site", name="imu", size="0.01")

    def add_geoms(body, link: Link):
        for b in link.boxes:
            ET.SubElement(
                body,
                "geom",
                name=b.name,
                type="box",
                size=fmt(np.array(b.size) / 2),
                pos=fmt(b.pos),
                material=b.material,
                **{"class": "collision" if b.collide else "visual"},
            )

    act = ET.Element("actuator")
    for i, (upper, lower) in enumerate(links, start=1):
        ub = ET.SubElement(base, "body", name=upper.name, pos=f"0 {p.slab_y(i):.6g} 0")
        ET.SubElement(ub, "joint", name=f"slab_{i}_hinge", **{"class": "hinge"})
        ET.SubElement(
            ub,
            "inertial",
            pos=fmt(upper.com),
            mass=f"{upper.mass:.6g}",
            diaginertia=fmt(upper.inertia),
        )
        add_geoms(ub, upper)
        if i == p.display_slab:
            ET.SubElement(
                ub,
                "camera",
                name="front_camera",
                pos=f"{p.slab_depth / 2 + 0.005:.6g} 0 {p.pivot_from_top - p.camera_from_top:.6g}",
                xyaxes="0 -1 0 0 0 1",
                fovy="70",
            )
            ET.SubElement(
                ub,
                "geom",
                name="camera_lens",
                type="cylinder",
                size="0.012 0.004",
                pos=f"{p.slab_depth / 2 + 0.003:.6g} 0 {p.pivot_from_top - p.camera_from_top:.6g}",
                zaxis="1 0 0",
                material="rubber",
                **{"class": "visual"},
            )
        lb = ET.SubElement(
            ub, "body", name=lower.name, pos=f"0 0 {p.pivot_from_top - p.upper_len:.6g}"
        )
        ET.SubElement(lb, "joint", name=f"slab_{i}_slide", **{"class": "slide"})
        ET.SubElement(
            lb,
            "inertial",
            pos=fmt(lower.com),
            mass=f"{lower.mass:.6g}",
            diaginertia=fmt(lower.inertia),
        )
        add_geoms(lb, lower)
        pad = p.foot_pad
        ET.SubElement(
            lb,
            "site",
            name=f"slab_{i}_foot",
            type="box",
            pos=f"0 0 {-p.lower_len + pad / 2:.6g}",
            size=fmt((p.slab_depth / 2 + 0.005, p.slab_width / 2 + 0.005, pad / 2 + 0.01)),
            rgba="0 0 0 0",
        )
        ET.SubElement(
            act, "motor", name=f"slab_{i}_hinge", joint=f"slab_{i}_hinge", **{"class": "hinge"}
        )
        ET.SubElement(
            act, "motor", name=f"slab_{i}_slide", joint=f"slab_{i}_slide", **{"class": "slide"}
        )
    m.append(act)

    sensor = ET.SubElement(m, "sensor")
    ET.SubElement(sensor, "framequat", name="imu_quat", objtype="site", objname="imu")
    ET.SubElement(sensor, "gyro", name="imu_gyro", site="imu")
    ET.SubElement(sensor, "accelerometer", name="imu_acc", site="imu")
    for i in range(1, p.n_slabs + 1):
        ET.SubElement(sensor, "touch", name=f"slab_{i}_foot_touch", site=f"slab_{i}_foot")

    if standalone:
        key = ET.SubElement(m, "keyframe")
        for name, pose in poses(p).items():
            a = pose["hub_pitch"] / 2
            qpos = [0, 0, pose["hub_z"], math.cos(a), 0, math.sin(a), 0]
            for hinge, slide in pose["slabs"]:
                qpos += [hinge, slide]
            ET.SubElement(key, "key", name=name, qpos=fmt(qpos))
    return m


def mjcf_xml(p: Params, standalone: bool = True) -> str:
    """MJCF for robot `p` as a string (built on the fly, so any Params/scale works)."""
    links = [slab_links(p, i) for i in range(1, p.n_slabs + 1)]
    xml = minidom.parseString(ET.tostring(build_mjcf(p, links, standalone))).toprettyxml(
        indent="  "
    )
    return "\n".join(line for line in xml.splitlines() if line.strip()) + "\n"


def write_xml(el: ET.Element, path: Path):
    xml = minidom.parseString(ET.tostring(el)).toprettyxml(indent="  ")
    path.write_text("\n".join(line for line in xml.splitlines() if line.strip()) + "\n")


def generate(p: Params, mjcf_path: Path = MJCF_PATH, urdf_dir: Path | None = None) -> None:
    links = [slab_links(p, i) for i in range(1, p.n_slabs + 1)]
    write_xml(build_mjcf(p, links), mjcf_path)
    print(f"wrote {mjcf_path}  (total mass {p.total_mass:.1f} kg)")
    if urdf_dir is None:
        return
    (urdf_dir / "meshes").mkdir(parents=True, exist_ok=True)
    hub_mesh(p).export(urdf_dir / "meshes" / "base_link.stl")
    for upper, lower in links:
        link_mesh(upper).export(urdf_dir / "meshes" / f"{upper.name}.stl")
        link_mesh(lower).export(urdf_dir / "meshes" / f"{lower.name}.stl")
    write_xml(build_urdf(p, links), urdf_dir / "tars.urdf")
    print(f"wrote {urdf_dir}/tars.urdf, {urdf_dir}/meshes/*.stl")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--urdf", type=Path, default=None, help="also export URDF + STL meshes to this dir"
    )
    generate(Params().resolved(), urdf_dir=ap.parse_args().urdf)
