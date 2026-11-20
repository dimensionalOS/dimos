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

"""RoboLab's evaluation, driven by dimos through DimosClient.

Run with RoboLab's Python, from RoboLab's checkout:

    python.sh run.py --headless --task BananaInBowlTask --listen tcp/0.0.0.0:7448

RoboLab registers its DROID envs as usual, with depth on the one camera dimos sees, and
writes its own results; nothing here reads the task's ground truth.
"""

import argparse
import sys
import traceback

import cv2  # noqa: F401 -- RoboLab: import before isaaclab
from isaaclab.app import AppLauncher  # type: ignore[import-not-found]

parser = argparse.ArgumentParser(description="Evaluate dimos on RoboLab.")
parser.add_argument("--listen", default="tcp/0.0.0.0:7448", help="Zenoh endpoint to listen on.")
parser.add_argument("--connect", default=None, help="Zenoh endpoint to connect to.")
parser.add_argument("--prefix", default="robolab", help="Zenoh key prefix.")
parser.add_argument("--camera", default="over_shoulder_left_camera")
parser.add_argument("--settle-steps", type=int, default=15)
parser.add_argument("--release-settle-steps", type=int, default=30)

from robolab.eval.runner import (  # type: ignore[import-not-found]
    add_common_eval_args,
    run_evaluation,
)

add_common_eval_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()
args_cli.enable_cameras = True
simulation_app = AppLauncher(args_cli).app

from dimos_client import DimosClient, open_session  # type: ignore[import-not-found]
import robolab.constants  # type: ignore[import-not-found]
from robolab.registrations.droid.auto_env_registrations_jointpos import (  # type: ignore[import-not-found]
    auto_register_droid_envs,
)
from robolab.variations.camera import (  # type: ignore[import-not-found]
    OverShoulderLeftCameraCfg,
    with_depth,
)

robolab.constants.ENABLE_SUBTASK_PROGRESS_CHECKING = args_cli.enable_subtask

# Depth renders only when sensors update eagerly in headless mode.
auto_register_droid_envs(
    task_dirs=args_cli.task_dirs,
    task=args_cli.task,
    cameras=[with_depth(OverShoulderLeftCameraCfg)],
    lazy_sensor_update=False,
)

session = open_session(args_cli.listen, args_cli.connect)


def make_client(args: argparse.Namespace) -> DimosClient:
    return DimosClient(
        session,
        prefix=args.prefix,
        camera=args.camera,
        settle_steps=args.settle_steps,
        release_settle_steps=args.release_settle_steps,
    )


def main() -> None:
    run_evaluation(args_cli, policy="dimos", client_factory=make_client)
    session.close()
    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[dimos-robolab] terminated with error: {e}")
        traceback.print_exc()
        simulation_app.close()
        sys.exit(1)
