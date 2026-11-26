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


from __future__ import annotations

import subprocess
import sys
import textwrap
from typing import Any

import numpy as np
import pytest

from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.assets import hash_of_file

pytest.importorskip("pxr")
mujoco = pytest.importorskip("mujoco")

_ROBOTS = ("g1", "go2")


def _module(robot: str) -> Any:
    if robot == "g1":
        from dimos.robot.unitree.g1.sim2 import G1Sim

        return G1Sim
    from dimos.robot.unitree.go2.sim2 import Go2Sim

    return Go2Sim


def _usd(robot: str) -> UsdModel:
    if robot == "g1":
        from dimos.robot.unitree.g1.sim2 import G1_USD

        return G1_USD
    from dimos.robot.unitree.go2.sim2 import GO2_USD

    return GO2_USD


@pytest.mark.parametrize("robot", _ROBOTS)
def test_the_layer_names_every_frame_a_sensor_mounts_at(robot: str) -> None:
    frames = _usd(robot).frame_names()

    mounts = list(_module(robot).mounts)
    assert mounts, f"{robot} advertises no mount frames, so this test proves nothing"
    assert set(mounts) <= frames, f"{robot}: {sorted(set(mounts) - frames)} is not in its layer"


@pytest.mark.parametrize("robot", _ROBOTS)
def test_every_file_the_layer_names_is_beside_it_and_hashes(robot: str) -> None:
    source = _usd(robot).source_path
    assert source is not None
    layer = UsdModel.from_file(source)

    named = layer.referenced_files()
    assert named, f"{robot} names no files at all, which a mesh-built robot should not"
    missing = sorted(name for name in named if not (source.parent / name).is_file())
    assert not missing, f"{robot}: {missing}"
    unhashable = sorted(name for name in named if not hash_of_file(str(source.parent / name)))
    assert not unhashable, f"{robot}: {unhashable} (a git-lfs pointer, most likely)"


@pytest.mark.self_hosted  # MuJoCo compiles the robot from its Git LFS meshes
@pytest.mark.parametrize("robot", _ROBOTS)
def test_mujoco_simulates_the_robot_off_the_layer_alone(robot: str) -> None:
    import mujoco._usd

    from dimos.sim2.usd_layer import usd_layer

    model = mujoco.MjSpec.from_file(str(usd_layer(_usd(robot)))).compile()
    data = mujoco.MjData(model)
    for _ in range(100):
        mujoco.mj_step(model, data)

    assert model.nu > 0, f"{robot} has no actuators, so no servo of its can be driven"
    assert model.nmesh > 0, f"{robot} drew nothing: its meshes did not compose"
    assert bool(np.isfinite(data.qpos).all()), f"{robot} left the real numbers within 100 steps"


@pytest.mark.self_hosted  # usd_layer fetches the robot's Git LFS meshes
def test_mujoco_keeps_its_actuators_when_usd_core_went_first() -> None:
    source = textwrap.dedent("""
        from dimos.robot.unitree.g1.sim2 import G1_USD
        from dimos.sim2.usd_layer import usd_layer
        # usd-core first: composing the mounted layer is what a blueprint does before the
        # world is built.
        path = usd_layer(G1_USD)
        import mujoco
        print(len(list(mujoco.MjSpec.from_file(str(path)).actuators)))
    """)

    finished = subprocess.run(
        [sys.executable, "-c", source], capture_output=True, text=True, timeout=600
    )

    assert finished.returncode == 0, finished.stderr[-2000:]
    assert int(finished.stdout.strip().splitlines()[-1]) == 29, "the G1 lost its actuators"


def _mujoco_frames(model: Any, data: Any) -> dict[str, tuple[Any, Any]]:
    """Every body's world pose, keyed by the frame name its prim path ends in."""
    frames = {}
    for body in range(model.nbody):
        path = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
        if path:
            frames[path.rsplit("/", 1)[-1]] = (data.xpos[body], data.xmat[body].reshape(3, 3))
    return frames


@pytest.mark.self_hosted  # MuJoCo compiles the robot from its Git LFS meshes
@pytest.mark.parametrize("robot", _ROBOTS)
def test_the_physics_and_the_tf_chain_place_a_frame_the_same(robot: str) -> None:
    """One layer, two readers, and they have to agree.

    MuJoCo builds the world from the prims' own transforms; tf is rebuilt by ``usd_to_urdf``
    from the joints -- their local frames and their axis. A field one reads and the other
    ignores diverges in silence, which is how a lidar ended up pointing down in physics and up
    on tf, and how every joint came to turn about z. Posing both at the same angles and
    comparing is the check that catches it.
    """
    import mujoco._usd

    from dimos.sim2.usd_layer import usd_layer

    usd = _usd(robot)
    model = mujoco.MjSpec.from_file(str(usd_layer(usd))).compile()
    data = mujoco.MjData(model)
    angles = {}
    for joint_id in range(model.njnt):
        if int(model.jnt_type[joint_id]) != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        angle = 0.1 + 0.01 * joint_id  # distinct per joint, so a swapped pair cannot pass
        data.qpos[model.jnt_qposadr[joint_id]] = angle
        angles[(name or "").rsplit("/", 1)[-1]] = angle
    mujoco.mj_forward(model, data)
    physics = _mujoco_frames(model, data)

    placed: dict[str, Any] = {usd.root_link: np.eye(4)}
    joints = list(usd.joints)
    while True:  # the joints come in no particular order, so place what can be placed
        grew = False
        for joint in joints:
            if joint.parent_link in placed and joint.child_link not in placed:
                placed[joint.child_link] = placed[joint.parent_link] @ joint.parent_from_child(
                    angles.get(joint.name, 0.0)
                )
                grew = True
        if not grew:
            break
    # A frame is its prim path, and MuJoCo names a body by the path's last segment.
    root_position = physics[usd.root_link.rpartition("/")[2]][0]
    compared = 0
    for frame, pose in placed.items():
        if frame.rpartition("/")[2] not in physics:
            continue
        position, rotation = physics[frame.rpartition("/")[2]]
        compared += 1
        assert np.allclose(pose[:3, 3], position - root_position, atol=2e-3), (
            f"{robot}: {frame} sits somewhere else in physics than on tf"
        )
        assert np.allclose(pose[:3, :3], rotation, atol=2e-3), (
            f"{robot}: {frame} faces somewhere else in physics than on tf"
        )
    assert compared > 10, f"{robot}: only {compared} frames compared, so this proves little"


@pytest.mark.self_hosted  # usd_layer fetches (and reads) the robot's Git LFS meshes
@pytest.mark.parametrize("robot", _ROBOTS)
def test_no_mesh_is_left_in_the_converters_placeholder_grey(robot: str) -> None:
    """A mesh that binds a material says that material's colour, not (0.5, 0.5, 0.5).

    The converter writes a placeholder displayColor on every visual mesh, and a reader that
    takes displayColor first -- the Controller does -- then draws a black-and-silver robot
    flat grey. `scripts/mjcf_to_usd.py --repaint` puts the bound colour on the mesh; this is
    the layer saying it was run.
    """
    from pxr import Usd, UsdShade

    from dimos.sim2.usd_layer import usd_layer

    stage = Usd.Stage.Open(str(usd_layer(_usd(robot))))
    placeholder = []
    for prim in stage.Traverse():
        if prim.GetTypeName() != "Mesh":
            continue
        bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if not bound or bound.GetPrim().GetAttribute("inputs:color").Get() is None:
            continue
        shown = prim.GetAttribute("primvars:displayColor").Get()
        if shown is not None and all(abs(c - 0.5) < 1e-6 for c in tuple(shown[0])):
            placeholder.append(str(prim.GetPath()))
    assert not placeholder, (
        f"{robot}: {len(placeholder)} mesh(es) still carry the converter's grey, e.g. "
        f"{placeholder[:3]}; run `python scripts/mjcf_to_usd.py --repaint <layer>`"
    )
