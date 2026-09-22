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

"""Plan and inspect generated joint waypoints against a real Drake world."""

from pathlib import Path
from tempfile import TemporaryDirectory

from dimos_generated.sensor_msgs.msg import JointState
import numpy as np

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.planners.rrt_planner import RRTConnectPlanner
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.manipulation.planning.utils.path_utils import interpolate_path
from dimos.manipulation.planning.world.drake_world import DrakeWorld
from dimos.robot.assets.model import RobotModel

_URDF = """<robot name="demo_arm">
<link name="base"/><link name="elbow"/><link name="tool"/>
<joint name="shoulder" type="revolute">
  <parent link="base"/><child link="elbow"/><axis xyz="0 0 1"/>
  <limit lower="-1" upper="1" effort="1" velocity="1"/>
</joint>
<joint name="wrist" type="revolute">
  <parent link="elbow"/><child link="tool"/><origin xyz="1 0 0"/>
  <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
</joint>
</robot>"""


def main() -> None:
    with TemporaryDirectory(prefix="dimos-cdr-planning-") as directory:
        path = Path(directory) / "arm.urdf"
        path.write_text(_URDF)
        config = RobotModelConfig(
            model=RobotModel.from_file(path).with_default_joint_acceleration_limit(2.0),
            joint_names=["shoulder", "wrist"],
            base_link="base",
            planning_groups=[PlanningGroupDefinition("arm", ("shoulder", "wrist"), "base", "tool")],
        )
        world = DrakeWorld()
        world.load_model(prepare_robot_model(config))
        world.finalize()
        start = JointState.decode(
            JointState(name=config.joint_names, position=[-0.4, 0.3]).encode()
        )
        goal = JointState.decode(JointState(name=config.joint_names, position=[0.4, -0.3]).encode())
        result = RRTConnectPlanner().plan_joint_path(world, start, goal)
        assert result.is_success(), result
        path_points = interpolate_path(result.path, resolution=0.2)
        print(f"Drake model joints: {world.get_prepared_model().joint_space.names}")
        print(f"RRT result: {result.status.name}; interpolated waypoints={len(path_points)}")
        for index, point in enumerate(path_points):
            decoded = JointState.decode(point.encode())
            assert decoded == point
            assert world.check_config_collision_free(decoded)
            with world.scratch_context() as context:
                world.set_joint_state(context, decoded)
                np.testing.assert_allclose(
                    list(world.get_joint_state(context).position), list(decoded.position)
                )
            print(
                f"  waypoint {index}: {list(decoded.position)}; CDR round-trip and collision check passed"
            )
        np.testing.assert_allclose(list(path_points[0].position), list(start.position))
        np.testing.assert_allclose(list(path_points[-1].position), list(goal.position))
    print("PASS: generated JointState feeds real Drake and RRT; temporary model removed")


if __name__ == "__main__":
    main()
