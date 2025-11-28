# Copyright 2025 Dimensional Inc.
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


from dimos import spec
from dimos.core import DimosCluster, In, Module, rpc
from dimos.msgs.geometry_msgs import (
    Twist,
    TwistStamped,
)
from dimos.robot.unitree.connection.connection import UnitreeWebRTCConnection


class G1Connection(Module):
    cmd_vel: In[TwistStamped] = None  # type: ignore
    ip: str | None

    connection: UnitreeWebRTCConnection

    def __init__(self, ip: str | None = None, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)

        if ip is None:
            raise ValueError("IP address must be provided for G1")
        self.connection = UnitreeWebRTCConnection(ip)

    @rpc
    def start(self) -> None:
        super().start()
        self.connection.start()
        self._disposables.add(
            self.cmd_vel.subscribe(self.move),  # type: ignore[arg-type]
        )

    @rpc
    def stop(self) -> None:
        self.connection.stop()
        super().stop()

    @rpc
    def move(self, twist_stamped: TwistStamped, duration: float = 0.0) -> None:
        """Send movement command to robot."""
        twist = Twist(linear=twist_stamped.linear, angular=twist_stamped.angular)
        self.connection.move(twist, duration)

    @rpc
    def publish_request(self, topic: str, data: dict):  # type: ignore[no-untyped-def, type-arg]
        """Forward WebRTC publish requests to connection."""
        return self.connection.publish_request(topic, data)


def deploy(dimos: DimosCluster, ip: str, local_planner: spec.LocalPlanner) -> G1Connection:
    connection = dimos.deploy(G1Connection, ip)  # type: ignore[attr-defined]
    connection.cmd_vel.connect(local_planner.cmd_vel)
    connection.start()
    return connection  # type: ignore[no-any-return]
