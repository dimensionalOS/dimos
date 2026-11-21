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

from typing import Any

from dimos.hosted.run_coordinator import HostedRunCoordinator


class FakeRPC:
    """Answers `<fragment>/<method>` from a table; a missing entry times out like a silent fragment."""

    def __init__(self, answers: dict[str, list[str]]) -> None:
        self.answers = answers

    def call_sync(self, topic: str, args: Any, rpc_timeout: float | None = None) -> tuple[Any, Any]:
        if topic not in self.answers:
            raise TimeoutError(topic)
        return self.answers[topic], None


def test_lists_every_fragments_modules_and_streams_and_skips_a_silent_one() -> None:
    rpc = FakeRPC(
        {
            "runs/r/hosts/aos/coordinator/list_modules": ["controller", "imu"],
            "runs/r/hosts/gos/coordinator/list_modules": ["planner"],
            "runs/r/hosts/gos/coordinator/list_streams": ["odom"],
        }
    )
    names = [f"runs/r/hosts/{host}/coordinator" for host in ("aos", "gos", "nos")]
    coordinator = HostedRunCoordinator(rpc, names)  # type: ignore[arg-type]

    assert coordinator.ping() == "pong"
    assert coordinator.list_modules() == ["controller", "imu", "planner"]
    assert coordinator.list_streams() == ["odom"]
    assert set(coordinator.rpcs) == {"ping", "list_modules", "list_streams"}
