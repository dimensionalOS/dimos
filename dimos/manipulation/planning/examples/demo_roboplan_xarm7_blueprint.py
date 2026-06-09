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

"""Manual QA blueprint for the optional RoboPlan manipulation backend.

Run from the repository root:

    uv run --extra manipulation python -m dimos.manipulation.planning.examples.demo_roboplan_xarm7_blueprint

Then, in another terminal, use the normal manipulation RPC client:

    uv run --extra manipulation python -i -m dimos.manipulation.planning.examples.manipulation_client

    >>> robots()
    >>> plan([0.1] * 7)
    >>> execute()

This demo intentionally avoids the CLI JSON robot-config path because the current
JSON config loader cannot deserialize PoseStamped robot overrides. It still uses
a real Blueprint and ControlCoordinator, just launched directly from Python.
"""

from pathlib import Path

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.transport import LCMTransport
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.catalog.ufactory import XARM7_FK_MODEL, xarm7

_xarm7_cfg = xarm7(
    name="arm",
    adapter_type="mock",
    add_gripper=False,
    model_path=XARM7_FK_MODEL,
)
_xarm7_model_cfg = _xarm7_cfg.to_robot_model_config()

roboplan_xarm7_planner_coordinator = autoconnect(
    ManipulationModule.blueprint(
        robots=[_xarm7_model_cfg],
        planning_backend="roboplan",
        planning_backend_options={
            "roboplan": {
                "model_path": Path("data/xarm_description/urdf/xarm7/xarm7.urdf"),
                "srdf_path": Path("dimos/manipulation/planning/examples/roboplan_xarm7.srdf"),
                "package_paths": [Path("data")],
                "planning_group": "arm",
                "active_joint_names": _xarm7_cfg.joint_names,
                "base_frame": "link_base",
                "end_effector_frame": "link7",
                "retiming": "dimos",
                "rrt": {
                    "max_nodes": 1000,
                    "max_connection_distance": 3.0,
                    "collision_check_step_size": 0.05,
                    "collision_check_use_bisection": False,
                    "goal_biasing_probability": 0.15,
                    "rrt_connect": True,
                },
                "ik": {
                    "max_iters": 100,
                    "max_time": 0.05,
                    "check_collisions": True,
                },
                "toppra": {
                    "dt": 0.02,
                    "mode": "Hermite",
                    "velocity_scale": 1.0,
                    "acceleration_scale": 1.0,
                },
            }
        },
        planning_timeout=10.0,
        enable_viz=False,
    ),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_xarm7_cfg.to_hardware_component()],
        tasks=[_xarm7_cfg.to_task_config()],
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)


def main() -> None:
    coordinator = ModuleCoordinator.build(roboplan_xarm7_planner_coordinator, {})
    coordinator.start_rpc_service()
    coordinator.loop()


if __name__ == "__main__":
    main()
