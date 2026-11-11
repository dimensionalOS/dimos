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

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus
from dimos_lcm.std_msgs import Time
from reactivex import interval

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Int32 import Int32
from dimos.msgs.std_msgs.String import String


class DemoTopicEcho(Module):
    text: Out[String]
    flag: Out[Bool]
    count: Out[Int32]
    goal_status: Out[GoalStatus]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(interval(1.0).subscribe(self._publish))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _publish(self, tick: int) -> None:
        self.text.publish(String(f"hello {tick}"))
        self.flag.publish(Bool(tick % 2 == 0))
        self.count.publish(Int32(tick))
        goal_id = GoalID(stamp=Time(sec=tick, nsec=0), id=f"goal-{tick}")
        self.goal_status.publish(GoalStatus(goal_id=goal_id, status=tick % 4, text="running"))


if __name__ == "__main__":
    ModuleCoordinator.build(DemoTopicEcho.blueprint()).loop()
