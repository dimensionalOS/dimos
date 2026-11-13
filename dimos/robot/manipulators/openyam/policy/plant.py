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

"""Build the deployable OpenYAM plant from the model the policy was trained in.

The training entry point does not run the compiled MJCF text: it overrides the
actuator limits in memory (no ``ctrlrange`` clamp, force range at the motor peak
torque) and steps at 1/480 s with 16 substeps per 30 Hz policy tick. A simulator
that just loads ``runtime.xml`` would therefore run a slightly different plant.

This tool writes an explicit plant that *is* the trained effective model:

* ``timestep`` 1/480, sixteen substeps per policy tick;
* position actuators keep the trained ``kp``/``kv`` and get
  ``forcerange = ±PEAK`` with ``ctrllimited = false``;
* the two unactuated, mimic-coupled finger joints are welded, so the plant has
  exactly the six joint coordinates the policy addresses (their mass stays on
  the hand link).

Usage:
    python -m dimos.robot.manipulators.openyam.policy.plant \
        --source runtime.xml --out openyam_plant.xml
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re

from dimos.robot.manipulators.openyam.policy.contract import (
    KD,
    KP,
    PEAK,
    PHYSICS_HZ,
    PolicyContractError,
    sha256_file,
)

_JOINT = re.compile(r'<joint name="arm_joint_(\d)"[^>]*/>')
_FINGER_JOINT = re.compile(r'<joint name="hand_finger_joint\d"[^>]*/>')
_EQUALITY = re.compile(r"<equality>.*?</equality>", re.S)
_ACTUATOR = re.compile(r'<position name="arm_joint_(\d)_sim_pd"[^>]*/>')
_ATTR = re.compile(r'(\w+)="([^"]*)"')
_TIMESTEP = re.compile(r'timestep="[^"]*"')


def _attributes(element: str) -> dict[str, str]:
    return dict(_ATTR.findall(element))


def build_plant(
    source: str | Path, out: str | Path, *, meshdir: str | None = None
) -> dict[str, object]:
    """Write the trained plant and return the transformations applied."""
    source_path = Path(source)
    text = source_path.read_text()

    def actuator(match: re.Match[str]) -> str:
        index = int(match.group(1))
        original = _attributes(match.group(0))
        ctrlrange = original.get("ctrlrange", "")
        peak = PEAK[index - 1]
        return (
            f'<position name="arm_joint_{index}_sim_pd" joint="arm_joint_{index}" '
            f'kp="{KP[index - 1]:g}" kv="{KD[index - 1]:g}" '
            f'forcerange="-{peak:g} {peak:g}" ctrlrange="{ctrlrange}" ctrllimited="false" />'
        )

    actuators = len(_ACTUATOR.findall(text))
    if actuators != len(KP):
        raise PolicyContractError(f"source has {actuators} arm actuators, expected {len(KP)}")
    text = _ACTUATOR.sub(actuator, text)
    text, timesteps = _TIMESTEP.subn(f'timestep="{1.0 / PHYSICS_HZ!r}"', text)
    if timesteps != 1:
        raise PolicyContractError("source has no single <option timestep> to rewrite")
    text, removed_fingers = _FINGER_JOINT.subn("", text)
    text, removed_equality = _EQUALITY.subn("", text)
    if removed_fingers != 2 or removed_equality != 1:
        raise PolicyContractError(
            f"expected 2 finger joints and 1 equality block, removed "
            f"{removed_fingers} and {removed_equality}"
        )
    if meshdir is not None:
        text = re.sub(r'meshdir="[^"]*"', f'meshdir="{meshdir}"', text)

    out_path = Path(out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(text)

    report: dict[str, object] = {
        "source": str(source_path.resolve()),
        "source_sha256": sha256_file(source_path),
        "plant": str(out_path.resolve()),
        "plant_sha256": sha256_file(out_path),
        "physics_hz": PHYSICS_HZ,
        "substeps_per_policy_tick": round(PHYSICS_HZ / 30.0),
        "kp": list(KP),
        "kd": list(KD),
        "peak": list(PEAK),
        "welded_finger_joints": ["hand_finger_joint1", "hand_finger_joint2"],
    }
    _verify(out_path)
    return report


def _verify(path: Path) -> None:
    """Load the written plant and check it really carries the trained terms."""
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover - depends on the install extra
        raise RuntimeError("verifying the plant requires mujoco") from exc
    model = mujoco.MjModel.from_xml_path(str(path))
    if model.nu != len(KP):
        raise PolicyContractError(f"plant has {model.nu} actuators, expected {len(KP)}")
    if model.njnt != len(KP):
        raise PolicyContractError(f"plant has {model.njnt} joints, expected {len(KP)}")
    if abs(model.opt.timestep - 1.0 / PHYSICS_HZ) > 1e-12:
        raise PolicyContractError(f"plant timestep is {model.opt.timestep}")
    for index in range(model.nu):
        kp = float(model.actuator_gainprm[index, 0])
        kv = float(-model.actuator_biasprm[index, 2])
        if abs(kp - KP[index]) > 1e-6 or abs(kv - KD[index]) > 1e-6:
            raise PolicyContractError(
                f"actuator {index} gains are kp={kp} kv={kv}, expected {KP[index]}/{KD[index]}"
            )
        if bool(model.actuator_ctrllimited[index]):
            raise PolicyContractError(f"actuator {index} still clips its command to ctrlrange")
        force = [float(value) for value in model.actuator_forcerange[index]]
        if abs(force[0] + PEAK[index]) > 1e-6 or abs(force[1] - PEAK[index]) > 1e-6:
            raise PolicyContractError(f"actuator {index} force range is {force}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, help="trained runtime.xml")
    parser.add_argument("--out", required=True, help="plant XML to write")
    parser.add_argument(
        "--meshdir", help="override the mesh directory (default: keep the source's)"
    )
    options = parser.parse_args()
    print(json.dumps(build_plant(options.source, options.out, meshdir=options.meshdir), indent=2))


if __name__ == "__main__":
    main()
