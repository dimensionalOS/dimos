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

"""Follow a generated CDR path in the existing base simulator and export an SVG."""

from pathlib import Path as FilePath

from dimos_generated.geometry_msgs.msg import PoseStamped, Twist, Vector3
from dimos_generated.nav_msgs.msg import Path
from dimos_generated.std_msgs.msg import Header

from dimos.control.benchmarking.paths import straight_line, trajectory_to_svg
from dimos.control.benchmarking.plant import (
    FopdtChannelParams,
    TwistBasePlantParams,
    TwistBasePlantSim,
)
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.path_follower_task.path_follower_task import (
    PathFollowerTask,
    PathFollowerTaskConfig,
)
from dimos.core.global_config import GlobalConfig


def main() -> None:
    reference = straight_line(length=2.0)
    reference.header = Header(frame_id="world")
    reference.poses = [
        PoseStamped(header=reference.header, pose=pose.pose) for pose in reference.poses
    ]
    received = Path.decode(reference.encode())
    names = ["base/vx", "base/vy", "base/wz"]
    task = PathFollowerTask(
        "demo", PathFollowerTaskConfig(joint_names=names, speed=0.5), GlobalConfig()
    )
    channel = FopdtChannelParams(K=1.0, tau=0.1, L=0.0)
    plant = TwistBasePlantSim(TwistBasePlantParams(vx=channel, vy=channel, wz=channel))
    plant.reset(0.0, 0.0, 0.0, 0.1)
    assert task.start_path(received, PoseStamped(header=received.header))
    trace: list[tuple[float, float]] = []
    for tick in range(200):
        state = CoordinatorState(
            joints=JointStateSnapshot(
                joint_positions=dict(zip(names, [plant.x, plant.y, plant.yaw], strict=True)),
                joint_velocities=dict.fromkeys(names, 0.0),
            ),
            t_now=tick * 0.1,
            dt=0.1,
        )
        command = task.compute(state)
        trace.append((plant.x, plant.y))
        if not task.is_active():
            break
        assert command is not None
        vx, vy, wz = command.velocities
        wire = Twist(linear=Vector3(x=vx, y=vy), angular=Vector3(z=wz)).encode()
        decoded = Twist.decode(wire)
        plant.step(decoded.linear.x, decoded.linear.y, decoded.angular.z, 0.1)
    assert task.get_state() == "arrived"
    assert abs(plant.x - 2.0) <= 0.2 and abs(plant.y) < 1e-9
    output = FilePath("build/message-codegen/demo/evidence/path-following.svg")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(trajectory_to_svg(received, trace))
    print(f"Input: nav_msgs/msg/Path; {len(received.poses)} CDR-decoded waypoints")
    print(
        f"Result: {task.get_state()}; position=({plant.x:.3f}, {plant.y:.3f}); ticks={len(trace)}"
    )
    print(f"SVG reference and executed path: {output}")
    print("PASS: generated paths and Twist commands drive the base simulator to arrival")


if __name__ == "__main__":
    main()
