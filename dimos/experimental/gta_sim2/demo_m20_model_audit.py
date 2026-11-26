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

"""Audit native M20 inertias and motor limits after composition into Sim2 USD."""

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
    joints = {model.joint(i).name.split("/")[-1]: i for i in range(model.njnt)}
    for name, home in zip(JOINTS, home_pose(args.profile), strict=True):
        joint = joints[name]
        actuator = model.actuator(model.joint(joint).name).id
        np.testing.assert_allclose(model.jnt_axis[joint], native.joint(name).axis)
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
    print("PASS: native masses, inertias, axes, home targets, torque limits and spawn height")


if __name__ == "__main__":
    main()
