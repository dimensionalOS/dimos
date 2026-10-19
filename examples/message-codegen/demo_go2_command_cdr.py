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

"""Inspect generated Go2 command boundaries with mocked streams and driver RPC."""

import time
from unittest.mock import MagicMock, patch

from dimos_generated.geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Vector3
from dimos_generated.std_msgs.msg import Header

from dimos.core.module import Module
from dimos.msgs.time import time_from_nanoseconds
from dimos.teleop.hosted.go2_command import Go2CommandConfig, Go2CommandModule


def main() -> None:
    # This handler harness never builds a blueprint or connects a robot driver.
    with patch.object(Module, "__init__", return_value=None):
        module = Go2CommandModule()
    module.config = Go2CommandConfig()
    module.go2 = MagicMock()
    for port in ("tele_cmd_vel", "goal_request", "cmd_ack"):
        setattr(module, port, MagicMock())
    stamp_ns = time.time_ns() - 100_000_000
    for offset in (0, 1):
        message = TwistStamped(
            header=Header(stamp=time_from_nanoseconds(stamp_ns + offset)),
            twist=Twist(linear=Vector3(x=99), angular=Vector3(z=-50)),
        )
        module._on_cmd_vel_in(TwistStamped.decode(message.encode()))
    assert module.tele_cmd_vel.publish.call_count == 2
    command = Twist.decode(module.tele_cmd_vel.publish.call_args.args[0].encode())
    assert command.linear.x == 1.5 and command.angular.z == -2
    module._handle_nav_goal({"x": 2.5, "y": -1, "nonce": 1})
    goal = PoseStamped.decode(module.goal_request.publish.call_args.args[0].encode())
    assert goal.header.frame_id == "world" and goal.pose.position.x == 2.5
    module._estopped = True
    module._on_cmd_vel_in(message)
    assert module.tele_cmd_vel.publish.call_count == 2
    module.go2.assert_not_called()
    assert module.go2.mock_calls == []
    print(f"CDR drive: two frames 1ns apart, x={command.linear.x}, yaw={command.angular.z}")
    print(f"CDR goal: {goal.header.frame_id} ({goal.pose.position.x}, {goal.pose.position.y})")
    print("PASS: generated command types, limits, exact ordering and E-STOP; no driver calls")


if __name__ == "__main__":
    main()
