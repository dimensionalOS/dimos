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

"""Python SDK client for a running manipulation blueprint.

Start ``dimos run xarm-perception-sim``, then run this module.
With no arguments it demonstrates motion. Add ``--object cup --place X Y Z``
to scan, pick, and place at a verified planning-frame position.
"""

from __future__ import annotations

import argparse

from dimos.manipulation.pick_and_place_spec import PickAndPlaceSpec, PickPlaceResult
from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.porcelain.dimos import Dimos
from dimos.sdk.manipulation import Arm


def _require(result: PickPlaceResult) -> None:
    print(result)
    if not result.succeeded:
        raise RuntimeError(str(result))


def run_motion(arm: Arm) -> PlanningGroupID:
    """Demonstrate sequential joint, pose, linear, and gripper commands."""
    print(arm.info)
    print(arm.state())
    initial = arm.joints()
    positions = initial.copy()
    positions[0] += 0.02
    print(arm.move_joints(positions, speed_scale=0.2))

    pose = arm.pose()
    print(arm.move_pose([pose.x, pose.y, pose.z + 0.01], speed_scale=0.2))
    print(arm.move_linear(dz=-0.01, check_collision=True))
    # Restore the original camera/arm pose before the object workflow.
    print(arm.move_joints(initial, speed_scale=0.2))
    print(arm.open_gripper())
    return arm.info.id


def run_pick_place(
    pick_place: PickAndPlaceSpec,
    prompt: str,
    destination: tuple[float, float, float],
    planning_group: PlanningGroupID,
) -> None:
    """Pick the uniquely detected object and place it at a verified position."""
    scan = pick_place.scan_objects([prompt])
    _require(scan)
    if len(scan.objects) != 1:
        raise RuntimeError(f"Use a prompt selecting exactly one object; detected {scan.objects!r}")
    target = scan.objects[0]
    print(pick_place.get_object(target.object_id))
    picked = pick_place.pick_object(target.object_id, planning_group)
    _require(picked)
    print(pick_place.get_grasp_candidates())
    _require(pick_place.place_at(*destination, planning_group=planning_group))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--object", help="Unique object description for the current scene")
    parser.add_argument("--place", nargs=3, type=float, metavar=("X", "Y", "Z"))
    args = parser.parse_args()
    if (args.object is None) != (args.place is None):
        parser.error("--object and --place must be supplied together")

    app = Dimos.connect()
    try:
        arm = Arm.from_app(app)
        group = run_motion(arm)
        if args.object is not None:
            pick_place = app.get_module(PickAndPlaceSpec)
            run_pick_place(pick_place, args.object, tuple(args.place), group)
    finally:
        # Disconnecting does not stop the remote blueprint or cancel its motion.
        app.stop()


if __name__ == "__main__":
    main()
