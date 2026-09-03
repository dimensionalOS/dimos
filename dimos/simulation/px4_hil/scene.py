#!/usr/bin/env python3
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

"""Builds a mixed MuJoCo scene: N PX4 quadrotors plus M legged robots.

The whole point of moving off Gazebo is that aircraft and ground robots end up
in *one* physics world, so they can see and avoid each other for real rather
than through a co-simulation bridge that only exchanges positions.

Everything here is generated from primitives -- boxes, capsules, cylinders --
with no mesh dependencies. DimOS's usual legged assets (Go1/G1) live in a
git-lfs blob and need `mujoco_playground`, neither of which is available on
every machine; a procedural quadruped keeps the scene runnable anywhere. The
real meshes can be swapped in later without touching the bridge, because the
bridge only ever refers to bodies and sensors by name.

Naming is the contract between this file and the bridge. Every vehicle gets a
unique prefix, and the bridge resolves handles from it:

    drone0_imu, drone0_thrust0..3, drone0_acc, drone0_gyro   (per quadrotor)
    dog0_torso, dog0_imu                                     (per quadruped)

That prefix is deliberately the same string used for the DimOS namespace, so a
vehicle's identity is consistent from MuJoCo through MAVLink to the agent's
tool names.

Frames: world is NWU (x-North, y-West, z-Up) to match hil_bridge.py, which
needs a body at identity to point North the way PX4 expects.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Rotor arm positions for a real Holybro X500: a symmetric 0.174 m square,
# taken from the x500_base SDF the visual mesh comes from.
# PX4 states them in FRD (y right); MuJoCo is FLU, so y is negated here.
# Order matters: index i is PX4 motor i+1, i.e. HIL_ACTUATOR_CONTROLS[i].
#
# These MUST stay in lockstep with the CA_ROTOR*_P* values written by
# sim_params.py. PX4 builds its control-allocation matrix from those; if
# MuJoCo applies the thrust anywhere else the controller mis-allocates torque.
# (PX4's own 4001_gz_x500 ships (0.13, 0.22), which matches neither its own mesh
# nor the real airframe, so we override it on both sides instead of adopting it.)
X500_ARM_M = 0.174

# Rotor plane height above OUR body origin.
#
# The SDF puts the rotor links at z=0.06 above base_link, with the frame visual
# at z=+0.025. But base_link is not our body origin: we centre the frame mesh on
# its bounding box so the model rests on its landing gear, and that box centre
# sits 0.1131 m below the mesh origin (the gear hangs a long way down). So
# everything measured in the SDF frame shifts up by (0.1131 - 0.025) here.
#
# Getting this wrong is what left the props hanging 8.8 cm under the motors.
# Verified against the mesh: structure under each rotor position tops out at
# z=+0.025 in the SDF frame, i.e. 0.1131 here, and the props sit just above it.
X500_FRAME_BBOX_CZ = -0.1131   # bbox centre of the raw frame mesh
X500_SDF_VISUAL_DZ = 0.025     # SDF pose of the frame visual in base_link
_SDF_TO_BODY_Z = -(X500_FRAME_BBOX_CZ + X500_SDF_VISUAL_DZ)   # = +0.0881
X500_ROTOR_Z = 0.06 + _SDF_TO_BODY_Z          # 0.1481
X500_MOTOR_Z = X500_ROTOR_Z - 0.032           # SDF motor-base offset in the rotor link
ROTORS = [
    (X500_ARM_M, -X500_ARM_M, "0.8 0.2 0.2 1"),    # M1 front-right
    (-X500_ARM_M, X500_ARM_M, "0.2 0.2 0.8 1"),    # M2 rear-left
    (X500_ARM_M, X500_ARM_M, "0.2 0.8 0.2 1"),     # M3 front-left
    (-X500_ARM_M, -X500_ARM_M, "0.8 0.8 0.2 1"),   # M4 rear-right
]
# Rotor i spins CCW when its PX4 KM is negative. M1/M2 share one sense, M3/M4
# the other, so the CW/CCW prop meshes are assigned to match.
ROTOR_PROP_MESH = ("prop_cw", "prop_cw", "prop_ccw", "prop_ccw")
# +KM rotors spin one way, -KM the other. PX4 builds its effectiveness matrix as
#     moment = ct * position.cross(axis) - ct * km * axis     (axis = (0,0,-1) FRD)
# so torque_z(FRD) = +km * thrust. FRD z points down, so a positive FRD yaw
# torque is NEGATIVE about MuJoCo's z. Getting this backwards makes yaw a
# positive-feedback loop and the vehicle flips within half a second of arming.
ROTOR_YAW_TORQUE = [-0.368, -0.368, 0.368, 0.368]

# Per-vehicle accent colours. Purely cosmetic, but load-bearing for a human
# watching the viewer: with every airframe the same grey you cannot tell which
# drone is executing which lane of a sweep. Index = vehicle number.
ACCENTS = [
    "0.95 0.26 0.21 1",   # red
    "0.16 0.71 0.96 1",   # cyan
    "1.00 0.76 0.03 1",   # amber
    "0.30 0.89 0.44 1",   # green
    "0.85 0.36 0.95 1",   # violet
    "1.00 0.50 0.15 1",   # orange
]


def _accent(i: int) -> str:
    return ACCENTS[i % len(ACCENTS)]
MAX_THRUST_PER_ROTOR_N = 7.36

# Quadruped standing pose. Slightly bent so the legs are not in the singular
# fully-extended configuration, which makes the stance solver ill-conditioned.
DOG_HIP_ANGLE = 0.3
DOG_KNEE_ANGLE = -0.6
DOG_TORSO_Z = 0.40


def _quadrotor(
    prefix: str, x: float, y: float, z: float = 0.15, accent: str = "0.95 0.26 0.21 1"
) -> tuple[str, str, str]:
    """One PX4-compatible quadrotor body."""
    rotors, thrusts, motors, props = [], [], [], []
    have_mesh = x500_meshes_available()
    for i, (rx, ry, rgba) in enumerate(ROTORS):
        # The disc is the collision proxy for a spinning prop. It stays in the
        # model when the meshes are on, just hidden (group 3).
        rotors.append(
            f'      <geom name="{prefix}_rotor{i}" type="cylinder" size="0.06 0.005" '
            f'pos="{rx} {ry} {X500_ROTOR_Z}" rgba="{rgba}" mass="0" '
            f'group="{"3" if have_mesh else "0"}"/>'
        )
        if have_mesh:
            props.append(
                f'      <geom name="{prefix}_motor{i}" type="mesh" mesh="x500_motor_base" '
                f'pos="{rx} {ry} {X500_MOTOR_Z}" material="carbon" '
                f'mass="0" contype="0" conaffinity="0" group="2"/>\n'
                f'      <geom name="{prefix}_prop{i}" type="mesh" '
                f'mesh="x500_{ROTOR_PROP_MESH[i]}" pos="{rx} {ry} {X500_ROTOR_Z}" '
                f'rgba="{rgba}" mass="0" contype="0" conaffinity="0" group="2"/>\n'
                # Spinning props read as a translucent disc, not as blades. The
                # bridge cross-fades prop mesh -> this disc with throttle, the
                # same trick Gazebo's iris uses. Alpha 0 here: invisible until
                # the motors actually spin.
                f'      <geom name="{prefix}_disc{i}" type="cylinder" size="0.174 0.0035" '
                f'pos="{rx} {ry} {X500_ROTOR_Z}" rgba="0.25 0.25 0.28 0" '
                f'mass="0" contype="0" conaffinity="0" group="2"/>'
            )
        thrusts.append(
            f'      <site name="{prefix}_thrust{i}" pos="{rx} {ry} {X500_ROTOR_Z}" '
            f'size="0.01" rgba="0 0 0 0"/>'
        )
        motors.append(
            f'    <general name="{prefix}_motor{i}" site="{prefix}_thrust{i}" '
            f'ctrlrange="0 1" gear="0 0 {MAX_THRUST_PER_ROTOR_N} 0 0 '
            f'{ROTOR_YAW_TORQUE[i]}"/>'
        )
    legs = "\n".join(
        f'      <geom name="{prefix}_leg{i}" type="capsule" '
        f'fromto="{sx * 0.08} {sy * 0.08} -0.02 {sx * 0.11} {sy * 0.11} -0.14" '
        f'size="0.008" material="carbon" mass="0" '
        f'group="{"3" if x500_meshes_available() else "0"}"/>'
        for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)])
    )
    # With the X500 meshes present the primitives stay in the model as the
    # COLLISION shapes but move to group 3, which the viewer hides by default
    # (press 3 in the viewer to see them). The meshes are pure decoration:
    # contype/conaffinity 0 and mass 0, so mass, inertia and every contact are
    # byte-identical either way. Toggling the meshes cannot change how it flies.
    if x500_meshes_available():
        # The recognisable X500 details the frame scan does not carry: the GPS
        # mast with its white puck, the battery slab, and a rear status LED the
        # bridge drives (green = disarmed, red = armed). All visual-only.
        dressing = f"""
      <geom name="{prefix}_gps_mast" type="cylinder" size="0.006 0.075" pos="-0.06 0 0.21"
            material="carbon" mass="0" contype="0" conaffinity="0" group="2"/>
      <geom name="{prefix}_gps_puck" type="cylinder" size="0.036 0.009" pos="-0.06 0 0.292"
            material="plastic_white" mass="0" contype="0" conaffinity="0" group="2"/>
      <geom name="{prefix}_battery" type="box" size="0.055 0.035 0.022" pos="0.015 0 0.055"
            rgba="0.10 0.12 0.30 1" mass="0" contype="0" conaffinity="0" group="2"/>
      <geom name="{prefix}_led" type="sphere" size="0.012" pos="-0.105 0 0.075"
            rgba="0.1 0.9 0.2 1" mass="0" contype="0" conaffinity="0" group="2"/>"""
        vis_group, mesh_visual = "3", f"""
      <geom name="{prefix}_shell" type="mesh" mesh="x500_frame" pos="0 0 0"
            euler="0 0 3.14159265" material="carbon"
            mass="0" contype="0" conaffinity="0" group="2"/>
{chr(10).join(props)}"""
    else:
        dressing = ""
        vis_group, mesh_visual = "0", ""

    body = f"""    <body name="{prefix}_base" pos="{x} {y} {z}">
      <freejoint name="{prefix}_root"/>
      <inertial pos="0 0 0" mass="1.5" diaginertia="0.029125 0.029125 0.055225"/>
      <geom name="{prefix}_core" type="box" size="0.09 0.09 0.03" material="carbon"
            mass="0" group="{vis_group}"/>{mesh_visual}
      <geom name="{prefix}_beacon" type="sphere" size="0.03" pos="0 0 0.105" rgba="{accent}"
            mass="0" contype="0" conaffinity="0" group="2"/>{dressing}
      <camera name="{prefix}_chase" pos="-1.6 0 0.65" xyaxes="0 -1 0 0.385 0 0.923" mode="track"/>
{chr(10).join(rotors)}
{legs}
      <site name="{prefix}_imu" pos="0 0 0" size="0.01" rgba="1 0 0 0.3"/>
{chr(10).join(thrusts)}
    </body>"""
    sensors = f"""    <accelerometer name="{prefix}_acc" site="{prefix}_imu"/>
    <gyro name="{prefix}_gyro" site="{prefix}_imu"/>"""
    return body, "\n".join(motors), sensors


def _quadruped(
    prefix: str, x: float, y: float, accent: str = "1.00 0.76 0.03 1"
) -> tuple[str, str, str]:
    """One mesh-free quadruped: box torso, four 2-DOF legs, position servos."""
    legs, actuators = [], []
    corners = [("fl", 0.15, 0.11), ("fr", 0.15, -0.11), ("rl", -0.15, 0.11), ("rr", -0.15, -0.11)]
    for name, lx, ly in corners:
        legs.append(f"""        <body name="{prefix}_{name}_thigh" pos="{lx} {ly} 0">
          <joint name="{prefix}_{name}_hip" type="hinge" axis="0 1 0" range="-1.6 1.6" damping="2.0" armature="0.01"/>
          <geom type="capsule" fromto="0 0 0 0 0 -0.20" size="0.022" material="metal" mass="0.7"/>
          <body name="{prefix}_{name}_calf" pos="0 0 -0.20">
            <joint name="{prefix}_{name}_knee" type="hinge" axis="0 1 0" range="-2.4 0.0" damping="2.0" armature="0.01"/>
            <geom type="capsule" fromto="0 0 0 0 0 -0.20" size="0.018" material="carbon" mass="0.4"/>
            <geom name="{prefix}_{name}_foot" type="sphere" pos="0 0 -0.20" size="0.028"
                  rgba="0.1 0.1 0.1 1" mass="0.1" friction="1.2 0.05 0.05"/>
          </body>
        </body>""")
        # Position servos hold the stance. `armature` is the important term:
        # it models geared-motor rotor inertia, and without it a stiff actuator
        # on a light link is only marginally stable at a 4 ms step -- the stance
        # height then depends non-monotonically on kp and MuJoCo emits NaN QACC
        # warnings. With armature the result is identical across a 10x gain
        # range, so the gain can stay low and gentle.
        actuators.append(
            f'    <position name="{prefix}_{name}_hip" joint="{prefix}_{name}_hip" '
            f'kp="120" ctrlrange="-1.6 1.6"/>'
        )
        actuators.append(
            f'    <position name="{prefix}_{name}_knee" joint="{prefix}_{name}_knee" '
            f'kp="120" ctrlrange="-2.4 0.0"/>'
        )
    body = f"""    <body name="{prefix}_torso" pos="{x} {y} {DOG_TORSO_Z}">
      <freejoint name="{prefix}_root"/>
      <inertial pos="0 0 0" mass="6.0" diaginertia="0.06 0.16 0.18"/>
      <geom name="{prefix}_body" type="box" size="0.19 0.075 0.05" rgba="{accent}" mass="0"/>
      <geom name="{prefix}_head" type="box" size="0.05 0.045 0.035" pos="0.23 0 0.02"
            material="carbon" mass="0.3"/>
      <geom name="{prefix}_visor" type="box" size="0.012 0.032 0.016" pos="0.281 0 0.024"
            rgba="0.15 0.9 1.0 1" mass="0" contype="0" conaffinity="0" group="2"/>
      <camera name="{prefix}_chase" pos="-1.5 0 0.55" xyaxes="0 -1 0 0.34 0 0.94" mode="track"/>
      <site name="{prefix}_imu" pos="0 0 0" size="0.01" rgba="0 1 0 0.3"/>
{chr(10).join(legs)}
    </body>"""
    return body, "\n".join(actuators), ""


def build_scene(n_drones: int = 3, n_dogs: int = 1, timestep: float = 0.004) -> str:
    """Return MJCF for a world holding *n_drones* quadrotors and *n_dogs* quadrupeds.

    Drone spawn poses mirror the Gazebo swarm layout (a 3 m triangle) so the
    demo geometry, and therefore the separation guardrail's expectations, carry
    over from the Gazebo runs unchanged.
    """
    drone_poses = [(0.0, 2.0), (-2.0, -1.0), (2.0, -1.0)]
    bodies, actuators, sensors = [], [], []
    spawned: list[tuple[float, float]] = []

    for i in range(n_drones):
        if i < len(drone_poses):
            px, py = drone_poses[i]
        else:
            # Drones beyond the demo triangle go on a 3 m grid north of it,
            # four per row. The previous "ring" formula (px + 4*(1 + i//3))
            # collided with itself: drones 6 and 8 both landed on (10, -1),
            # spawning inside each other. The pairwise assertion below now
            # proves whatever this produces, at every fleet size.
            k = i - len(drone_poses)
            px = -4.5 + 3.0 * (k % 4)
            py = 4.5 + 3.0 * (k // 4)
        spawned.append((px, py))
        b, a, s = _quadrotor(f"drone{i}", px, py, accent=_accent(i))
        bodies.append(b)
        actuators.append(a)
        sensors.append(s)

    # Landing pads mark each drone's home. Visual only (contype/conaffinity 0)
    # so they can never perturb a touchdown, but they give the viewer a fixed
    # ground reference and make an RTL legible at a glance.
    pads: list[str] = []
    for i, (px, py) in enumerate(spawned):
        pads.append(
            f'    <geom name="pad{i}" type="cylinder" size="0.75 0.004" pos="{px} {py} 0.004"'
            f' rgba="0.10 0.11 0.13 1" mass="0" contype="0" conaffinity="0" group="2"/>\n'
            f'    <geom name="pad{i}_ring" type="cylinder" size="0.62 0.005" pos="{px} {py} 0.005"'
            f' rgba="{_accent(i)}" mass="0" contype="0" conaffinity="0" group="2"/>\n'
            f'    <geom name="pad{i}_hole" type="cylinder" size="0.50 0.006" pos="{px} {py} 0.006"'
            f' rgba="0.10 0.11 0.13 1" mass="0" contype="0" conaffinity="0" group="2"/>'
        )

    dog_poses = [(-4.0 - 1.5 * i, 0.0) for i in range(n_dogs)]
    for i, (dx, dy) in enumerate(dog_poses):
        b, a, _ = _quadruped(f"dog{i}", dx, dy, accent=_accent(i + 2))
        bodies.append(b)
        actuators.append(a)

    # Spawn sanity: robots that spawn inside each other produce contact forces
    # on the very first step that read as an inexplicable launch-and-tumble.
    # The ring formula for >3 drones is ad hoc, so prove the poses rather than
    # trust them -- at every fleet size, not just the ones tried so far.
    all_poses = spawned + dog_poses
    for i_a in range(len(all_poses)):
        for i_b in range(i_a + 1, len(all_poses)):
            ax, ay = all_poses[i_a]
            bx, by = all_poses[i_b]
            d = ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5
            if d < 1.2:
                raise ValueError(
                    f"spawn poses {i_a} ({ax:.1f},{ay:.1f}) and {i_b} "
                    f"({bx:.1f},{by:.1f}) are {d:.2f} m apart -- robots would "
                    "spawn overlapping. Fix the pose table in build_scene()."
                )

    return f"""<!-- Generated by dimos/simulation/px4_hil/scene.py. Do not edit by hand. -->
<mujoco model="dimos_mixed_fleet">
  <compiler inertiafromgeom="auto" angle="radian"/>
  <option timestep="{timestep}" gravity="0 0 -9.81" integrator="implicitfast" density="1.225"/>
  <default>
    <geom contype="1" conaffinity="1" condim="3" friction="1 0.05 0.05"/>
  </default>
  <visual>
    <!-- Anti-aliasing and a large shadow map: without offsamples every strut is
         a staircase, and the default 1024 shadow map over a 600 m plane gives
         each drone a shadow made of visible blocks. -->
    <quality shadowsize="8192" offsamples="8"/>
    <!-- haze blends distant ground into the skybox, which is what makes the
         plane read as ground receding to a horizon rather than a cut-off slab. -->
    <map haze="0.20" stiffness="100" fogstart="40" fogend="900" zfar="3000"/>
    <rgba haze="0.62 0.72 0.84 1"/>
    <!-- Opening camera: three-quarter view looking slightly down, so the fleet
         is framed on load instead of edge-on at ground level. -->
    <global azimuth="140" elevation="-22" offwidth="1920" offheight="1080"/>
    <headlight ambient="0.32 0.33 0.36" diffuse="0.28 0.28 0.30" specular="0.1 0.1 0.1"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient"
             rgb1="0.28 0.45 0.72" rgb2="0.74 0.83 0.92" width="512" height="512"/>
    <!-- 2 m checker (600 m plane / 300 repeats). The old 20 m checker gave a
         drone at 8 m almost no parallax, so motion was nearly invisible. -->
    <texture name="grid" type="2d" builtin="checker" rgb1="0.24 0.28 0.33"
             rgb2="0.33 0.38 0.44" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="300 300"
              reflectance="0.12" specular="0.25" shininess="0.15"/>
    <material name="carbon" rgba="0.13 0.135 0.15 1" specular="0.75" shininess="0.65"/>
    <material name="metal" rgba="0.52 0.55 0.60 1" specular="0.80" shininess="0.75"/>
    <material name="plastic_white" rgba="0.88 0.89 0.92 1" specular="0.35" shininess="0.4"/>
{_x500_asset_block()}
  </asset>
  <worldbody>
    <!-- One directional key light casts the shadows that give the scene depth;
         a dimmer fill from the opposite side keeps the shadowed faces readable.
         The old single overhead light lit every surface equally, which is why
         everything looked flat. -->
    <light name="sun" directional="true" castshadow="true" pos="12 14 30" dir="-0.4 -0.45 -1"
           diffuse="0.85 0.83 0.78" specular="0.35 0.35 0.35"/>
    <light name="fill" directional="true" castshadow="false" pos="-20 -18 22" dir="0.5 0.45 -1"
           diffuse="0.22 0.24 0.30" specular="0 0 0"/>
    <camera name="field" pos="-16 -14 11" xyaxes="0.66 -0.75 0 0.30 0.26 0.92"/>
    <geom name="ground" type="plane" size="300 300 0.1" material="grid"/>
{_environment()}
{chr(10).join(pads)}
{chr(10).join(bodies)}
  </worldbody>
  <actuator>
{chr(10).join(actuators)}
  </actuator>
  <sensor>
{chr(10).join(s for s in sensors if s)}
  </sensor>
</mujoco>
"""


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(description="Emit a mixed drone/legged MuJoCo scene")
    p.add_argument("--drones", type=int, default=3)
    p.add_argument("--dogs", type=int, default=1)
    p.add_argument("-o", "--out", default="-")
    a = p.parse_args()
    xml = build_scene(a.drones, a.dogs)
    if a.out == "-":
        print(xml)
    else:
        with open(a.out, "w") as fh:
            fh.write(xml)
        print(f"wrote {a.out}")


# ---------------------------------------------------------------------------
# Real Unitree Go1 composition
# ---------------------------------------------------------------------------
# The primitive quadruped above runs anywhere with no assets. When MuJoCo
# Menagerie is present we can do much better: attach the real Go1 that DimOS's
# trained locomotion policy was trained against, one per dog, each under its own
# `dogN-` prefix. Same `MjSpec.attach(prefix=...)` call DimOS's MujocoSimModule
# uses -- just in a loop, because this world holds a whole fleet rather than one
# robot.
#
# Note the Go1 collides on FEET ONLY (4 geoms). That is not a shortcut: it is the
# model the policy was trained on, and it is cheaper than the primitive dog,
# which collides on feet, calves and thighs.

GO1_XML_REL = "mujoco_playground/_src/locomotion/go1/xmls/go1_mjx_feetonly.xml"
GO1_SPAWN_Z = 0.30          # menagerie "home" keyframe stands at z=0.27
GO1_HOME_ANGLES = [0.0, 0.9, -1.8] * 4      # from unitree_go1/go1.xml keyframe


X500_MESH_DIR = Path(__file__).parent / "assets" / "x500"
X500_MESHES = ("frame", "motor_base", "motor_bell", "prop_cw", "prop_ccw")
X500_MESH_EXT = ".stl"


def x500_meshes_available() -> bool:
    """True when the decimated X500 meshes are on disk.

    Fetched and converted by ``tools/fetch_x500_meshes.py``. They are optional:
    without them the drones fall back to the box-and-cylinder airframe, which
    is dimensionally identical and flies exactly the same.
    """
    return all((X500_MESH_DIR / f"{m}{X500_MESH_EXT}").exists() for m in X500_MESHES)


def _x500_asset_block() -> str:
    if not x500_meshes_available():
        return ""
    return "\n".join(
        f'    <mesh name="x500_{m}" '
        f'file="{(X500_MESH_DIR / (m + X500_MESH_EXT)).as_posix()}"/>'
        for m in X500_MESHES
    )


def _environment() -> str:
    """Static dressing for the demo field. Frames: MuJoCo NWU, so y = -East.

    Three kinds of object, with different physics rules on purpose:
      * field lines -- visual only, mark the default 40 x 30 m sweep area
        (NED corners (0,0)..(40,30)) so a grid_sweep is legible in the viewer;
      * trees -- visual only, parked well outside the field. Purely for depth
        perception; a visual-only trunk the dog could walk through must never
        sit anywhere a mission path goes;
      * crates -- the one COLLIDABLE cluster, at NED (12, -14), i.e. west of
        the field and away from spawns and mission routes. They exist so there
        is at least one honest obstacle in the world; nothing routes through
        them by default.
    """
    out: list[str] = []
    # Field lines: NED (0,0)-(40,30) -> MuJoCo x 0..40, y -30..0.
    lw, lh = 0.12, 0.012
    for name, x, y, sx, sy in (
        ("field_s", 0.0, -15.0, lw, 15.0),
        ("field_n", 40.0, -15.0, lw, 15.0),
        ("field_w", 20.0, 0.0, 20.0, lw),
        ("field_e", 20.0, -30.0, 20.0, lw),
    ):
        out.append(
            f'    <geom name="{name}" type="box" size="{sx} {sy} {lh}" pos="{x} {y} {lh}"'
            f' rgba="0.92 0.94 0.97 1" mass="0" contype="0" conaffinity="0" group="2"/>'
        )
    trees = [(-25, 18), (-32, -14), (15, 22), (52, 9), (-14, -26), (34, 20), (55, -18)]
    for i, (x, y) in enumerate(trees):
        out.append(
            f'    <geom name="tree{i}_trunk" type="cylinder" size="0.14 1.1" pos="{x} {y} 1.1"'
            f' rgba="0.42 0.30 0.18 1" mass="0" contype="0" conaffinity="0" group="2"/>'
        )
        out.append(
            f'    <geom name="tree{i}_top" type="sphere" size="1.5" pos="{x} {y} 2.9"'
            f' rgba="0.18 0.46 0.20 1" mass="0" contype="0" conaffinity="0" group="2"/>'
        )
    crates = [(12.0, 14.0, 0.30), (13.1, 14.7, 0.24), (12.4, 15.5, 0.20)]
    for i, (x, y, h) in enumerate(crates):
        out.append(
            f'    <geom name="crate{i}" type="box" size="{h} {h} {h}" pos="{x} {y} {h}"'
            f' rgba="0.55 0.42 0.26 1"/>'
        )
    return "\n".join(out)


def go1_unavailable_reason() -> str | None:
    """Why the real Go1 cannot be used, or None when it can.

    Falling back to the primitive quadruped silently is worse than failing: the
    robot still walks, just badly, and there is nothing on screen or in the log
    to say a much better one was available. Every caller reports this string.
    """
    try:
        from mujoco_playground._src import mjx_env
    except Exception as exc:
        return f"mujoco_playground not importable ({exc}); pip install playground"

    import sysconfig

    candidate = Path(sysconfig.get_paths()["purelib"]) / GO1_XML_REL
    if not candidate.exists():
        return f"Go1 MJCF missing at {candidate}"
    # The MJCF references menagerie by a relative path; check the assets are
    # really there rather than failing deep inside the MuJoCo compiler.
    assets = mjx_env.MENAGERIE_PATH / "unitree_go1" / "assets"
    if not assets.exists():
        return (
            f"MuJoCo Menagerie not downloaded ({assets}); run: python -c "
            '"from mujoco_playground._src import mjx_env; mjx_env.ensure_menagerie_exists()"'
        )
    return None


def go1_xml_path() -> str | None:
    """Absolute path to the Go1 MJCF the policy was trained on, or None."""
    if go1_unavailable_reason() is not None:
        return None
    import sysconfig

    return str(Path(sysconfig.get_paths()["purelib"]) / GO1_XML_REL)


def build_model(n_drones: int = 3, n_dogs: int = 1, timestep: float = 0.004,
                real_go1: bool = True) -> tuple[Any, bool]:
    """Compile the fleet world. Returns (MjModel, used_real_go1).

    With Menagerie available every dog is a real Go1 attached under ``dogN-``.
    Without it we fall back to the primitive quadruped so the simulator still
    runs on a machine that has no assets.
    """
    import mujoco

    go1 = go1_xml_path() if (real_go1 and n_dogs > 0) else None
    if go1 is None:
        return mujoco.MjModel.from_xml_string(build_scene(n_drones, n_dogs, timestep)), False

    # Drones and world only; dogs get attached as real Go1 bodies below.
    spec = mujoco.MjSpec.from_string(build_scene(n_drones, 0, timestep))
    for i in range(n_dogs):
        child = mujoco.MjSpec.from_file(go1)
        # The Go1 MJCF asks for Euler + 1 solver iteration (what the policy was
        # trained under). This world keeps implicitfast + 100 iterations, which
        # the drones need for stability. Adopt the parent's settings explicitly
        # instead of letting attach() resolve the conflict and warn on every
        # startup -- the policy was measured to transfer cleanly across the
        # difference (1.0 m/s commanded -> 0.94 m/s achieved, no falls).
        child.option.integrator = spec.option.integrator
        child.option.iterations = spec.option.iterations
        child.option.ls_iterations = spec.option.ls_iterations
        child.option.timestep = spec.option.timestep
        frame = spec.worldbody.add_frame(pos=[-4.0 - 1.5 * i, 0.0, GO1_SPAWN_Z])
        spec.attach(child, prefix=f"dog{i}-", frame=frame)
    model = spec.compile()
    model.opt.timestep = timestep
    return model, True


def go1_policy_path() -> str | None:
    """Path to DimOS's trained Go1 locomotion policy, or None if unavailable.

    Ships in the Git-LFS archive ``data/.lfs/mujoco_sim.tar.gz``; ``get_data``
    unpacks it on first use. Menagerie supplies the mesh but NOT the weights, so
    both have to be present before the real robot can walk.
    """
    try:
        from dimos.utils.data import get_data

        path = Path(str(get_data("mujoco_sim"))) / "unitree_go1_policy.onnx"
    except Exception as exc:
        logger.warning(
            f"trained Go1 policy unavailable ({exc}). "
            "Needs git-lfs: sudo apt install git-lfs && git lfs pull "
            '--include "data/.lfs/mujoco_sim.tar.gz"'
        )
        return None
    if not path.exists():
        logger.warning(f"trained Go1 policy not found at {path}")
        return None
    return str(path)
