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

"""Audit native M20 contacts, inertias and motors after Sim2 USD composition."""

import argparse
from pathlib import Path

import mujoco
import numpy as np

from dimos.experimental.gta_sim2.m20_actor import JOINTS, home_pose
from dimos.robot.assets.model import UsdModel
from dimos.sim2.physics.mujoco import MujocoBackend


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("model", "meshes", "scene"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--profile", choices=("sdk57", "dreamwaq342"), default="sdk57")
    args = parser.parse_args()
    spec = mujoco.MjSpec.from_file(str(args.model.resolve()))
    spec.meshdir = str(args.meshes.resolve())
    native = spec.compile()
    backend = MujocoBackend()
    backend.load(UsdModel.from_file(args.scene.resolve()), 2_000_000)
    model = backend.model
    bodies = {model.body(i).name.split("/")[-1]: i for i in range(1, model.nbody)}
    for i in range(1, native.nbody):
        j = bodies[native.body(i).name]
        np.testing.assert_allclose(native.body_mass[i], model.body_mass[j], rtol=1e-6)
        np.testing.assert_allclose(
            native.body_inertia[i], model.body_inertia[j], rtol=1e-6, atol=1e-8
        )
        source_geoms = np.flatnonzero(
            (native.geom_bodyid == i)
            & ((native.geom_contype != 0) | (native.geom_conaffinity != 0))
        )
        target_geoms = list(
            np.flatnonzero(
                (model.geom_bodyid == j)
                & ((model.geom_contype != 0) | (model.geom_conaffinity != 0))
            )
        )
        assert len(source_geoms) == len(target_geoms), native.body(i).name
        for source in source_geoms:
            matches = [
                target
                for target in target_geoms
                if native.geom_type[source] == model.geom_type[target]
                and np.allclose(native.geom_pos[source], model.geom_pos[target], atol=1e-7)
                and np.allclose(native.geom_size[source], model.geom_size[target], atol=1e-7)
            ]
            assert len(matches) == 1, (native.body(i).name, source, matches)
            target = matches[0]
            target_geoms.remove(target)
            for field in (
                "geom_friction",
                "geom_condim",
                "geom_solref",
                "geom_solimp",
                "geom_contype",
                "geom_conaffinity",
                "geom_margin",
                "geom_gap",
            ):
                np.testing.assert_allclose(
                    getattr(native, field)[source],
                    getattr(model, field)[target],
                    rtol=1e-6,
                    atol=1e-8,
                    err_msg=f"{native.body(i).name}: {field}",
                )
            # Quaternions have equivalent opposite signs.
            np.testing.assert_allclose(
                abs(np.dot(native.geom_quat[source], model.geom_quat[target])), 1, atol=1e-7
            )
    joints = {model.joint(i).name.split("/")[-1]: i for i in range(model.njnt)}
    for name, home in zip(JOINTS, home_pose(args.profile), strict=True):
        joint = joints[name]
        actuator = model.actuator(model.joint(joint).name).id
        np.testing.assert_allclose(model.jnt_axis[joint], native.joint(name).axis)
        for field in ("dof_armature", "dof_damping", "dof_frictionloss"):
            np.testing.assert_allclose(
                getattr(native, field)[native.joint(name).dofadr[0]],
                getattr(model, field)[model.jnt_dofadr[joint]],
                atol=1e-8,
            )
        np.testing.assert_allclose(backend.data.qpos[model.jnt_qposadr[joint]], home, atol=1e-6)
        expected = (
            20
            if args.profile == "dreamwaq342" and "wheel" in name
            else 21.6
            if "wheel" in name
            else 76.4
        )
        np.testing.assert_allclose(model.actuator_forcerange[actuator], [-expected, expected])
        assert model.actuator_forcelimited[actuator]
    # Exported pose must start near contact, not silently reset to the source's 1 m drop.
    assert 0.45 < backend.data.qpos[2] < 0.65, backend.data.qpos[:7]
    print(
        "PASS: native contacts, friction, solver parameters, inertias, joint dynamics, motors and spawn"
    )


if __name__ == "__main__":
    main()
