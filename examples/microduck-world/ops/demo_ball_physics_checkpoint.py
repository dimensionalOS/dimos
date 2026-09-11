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

"""Compare each compiled world ball with an independent, pinned Pollen fixture."""

import json
from pathlib import Path

import mujoco
import numpy as np
from dimos.robot.pollen.microduck.assets_fetch import ensure_assets
from microduck_world.ball_physics import FLOOR_NAMES
from microduck_world.football import BALL_NAMES
from microduck_world.scene import PROJECT_ROOT, load_world
from microduck_world.world_sim import WorldSimModule

CONTACT_FIELDS = ("friction", "solref", "solimp", "condim", "priority", "solmix", "margin", "gap")
OPTION_FIELDS = (
    "timestep",
    "gravity",
    "integrator",
    "solver",
    "cone",
    "iterations",
    "tolerance",
    "impratio",
    "ls_iterations",
    "ls_tolerance",
    "noslip_iterations",
    "disableflags",
    "enableflags",
)


def isolate(world, ball):
    """Retain the compiled sphere and pitch box; translate the flat top to z=0."""
    spec = mujoco.MjSpec()
    for name in OPTION_FIELDS:
        setattr(spec.option, name, getattr(world.opt, name))
    ground = world.geom("football_floor")
    floor = spec.worldbody.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=ground.size,
        pos=[0, 0, -ground.size[2]],
    )
    body = spec.worldbody.add_body(name="ball", pos=[0, 0, 0.05])
    body.add_freejoint(name="ball_freejoint")
    original = world.geom(ball + "_geom")
    sphere = body.add_geom(
        name="ball_geom",
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=original.size,
        mass=float(world.body(ball).mass[0]),
    )
    for source, target in ((ground, floor), (original, sphere)):
        for name in CONTACT_FIELDS:
            value = getattr(source, name)
            setattr(target, name, value.item() if value.size == 1 else value)
    model = spec.compile()
    np.testing.assert_allclose(model.body("ball").inertia, world.body(ball).inertia)
    return model


def trajectory(model, case):
    data = mujoco.MjData(model)
    if case == "rolling":
        data.qvel[0] = 0.5
        data.qvel[4] = 0.5 / model.geom("ball_geom").size[0]
    else:
        data.qpos[2] = 0.5  # Centre height; bottom is 45 cm above the floor.
    samples = []
    contact = None
    for _ in range(round(10 / model.opt.timestep)):
        mujoco.mj_step(model, data)
        samples.append(np.r_[data.time, data.qpos, data.qvel])
        if data.ncon and contact is None:
            c = data.contact[0]
            contact = dict(
                dim=c.dim,
                friction=c.friction.tolist(),
                solref=c.solref.tolist(),
                solimp=c.solimp.tolist(),
                firstContactTime=float(data.time),
            )
    assert contact is not None
    np.testing.assert_allclose(contact["friction"], [1, 1, 0.01, 0.003, 0.003])
    np.testing.assert_allclose(contact["solref"], [0.025, 0.7])
    assert contact["dim"] == 6
    samples = np.asarray(samples)
    speeds = np.linalg.norm(samples[:, 8:11], axis=1)
    if case == "rolling":
        # First point after which horizontal speed stays below 1 cm/s.
        horizontal = np.linalg.norm(samples[:, 8:10], axis=1)
        above = np.flatnonzero(horizontal >= 0.01)
        stopped = int(above[-1] + 1) if len(above) else 0
        assert stopped < len(samples)
        metrics = dict(
            stopTime=float(samples[stopped, 0]),
            stopDistance=float(samples[stopped, 1]),
            finalSpeed=float(speeds[-1]),
        )
    else:
        bounced = np.flatnonzero((samples[:-1, 10] > 0) & (samples[1:, 10] <= 0))
        assert len(bounced)
        peak = int(bounced[0])
        metrics = dict(
            firstReboundCenterHeight=float(samples[peak, 3]),
            firstReboundBottomHeight=float(samples[peak, 3] - 0.05),
            finalCenterHeight=float(samples[-1, 3]),
            maxPenetration=float(max(0, 0.05 - samples[:, 3].min())),
        )
    return samples, dict(**metrics, contact=contact)


def main():
    assets = ensure_assets()
    module = WorldSimModule(
        scene_xml=load_world()[0].mujoco_scene_path,
        robot_mjcf=str(assets.robot_mjcf("default")),
        headless=True,
        auto_stand=False,
    )
    world = module._compose_spec().compile()
    fixture = Path(__file__).parent / "fixtures/pollen_flat_ball.xml"
    reference = mujoco.MjModel.from_xml_path(str(fixture))
    for field in OPTION_FIELDS:
        np.testing.assert_array_equal(getattr(world.opt, field), getattr(reference.opt, field))
    report = dict(
        mujocoVersion=mujoco.__version__,
        upstreamWasmVersion="3.11.0",
        upstreamRevision="e81974b932c7ca1819843b7bb3dcd42e2993e98e",
        floors={},
        balls={},
        comparisons={},
    )
    for name in FLOOR_NAMES:
        geom = world.geom(name)
        report["floors"][name] = {key: getattr(geom, key).tolist() for key in CONTACT_FIELDS}
    for name in BALL_NAMES:
        geom, body = world.geom(name + "_geom"), world.body(name)
        joint = world.joint(name + "_freejoint")
        adr = int(joint.qposadr[0])
        report["balls"][name] = dict(
            radius=float(geom.size[0]),
            mass=float(body.mass[0]),
            inertia=body.inertia.tolist(),
            resetPosition=world.qpos0[adr : adr + 3].tolist(),
            **{key: getattr(geom, key).tolist() for key in CONTACT_FIELDS},
        )
    output = PROJECT_ROOT / "logs/ball-physics-checkpoint"
    output.mkdir(parents=True, exist_ok=True)
    for case in ("rolling", "drop"):
        expected, metrics = trajectory(reference, case)
        report["comparisons"][case] = dict(reference=metrics, candidates={})
        np.save(output / (case + "-reference.npy"), expected)
        for ball in BALL_NAMES:
            actual, measured = trajectory(isolate(world, ball), case)
            error = float(np.max(np.abs(actual - expected)))
            report["comparisons"][case]["candidates"][ball] = dict(
                **measured,
                maxAbsoluteStateDifference=error,
            )
            np.save(output / (case + "-" + ball + ".npy"), actual)
            np.testing.assert_allclose(actual, expected, atol=1e-9, rtol=1e-9)
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: report[k] for k in ("mujocoVersion", "comparisons")}, indent=2))


if __name__ == "__main__":
    main()
