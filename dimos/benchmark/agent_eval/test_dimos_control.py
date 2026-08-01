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

from datetime import UTC, datetime

from dimos.agents.code_policy import CodePolicySessionReceipt
from dimos.benchmark.agent_eval.dimos_control import AttachedDimosControl
from dimos.navigation.base import NavigationState


class _Module:
    def __init__(self) -> None:
        self.interrupted = False
        self.cancelled = False
        self.state = NavigationState.IDLE

    def reset_session(self):
        return CodePolicySessionReceipt(
            session_id="code_policy_session_" + "a" * 32,
            reset_at=datetime.now(UTC),
            previous_session_id=None,
        )

    def interrupt_active(self):
        self.interrupted = True
        return True

    def get_state(self):
        return self.state

    def cancel_goal(self):
        self.cancelled = True


class _App:
    def __init__(self) -> None:
        self.CodePolicyModule = _Module()
        self.ReplanningAStarPlanner = _Module()
        self.stopped = False

    def stop(self):
        self.stopped = True


def test_attached_control_uses_host_only_rpc_paths() -> None:
    app = _App()
    control = AttachedDimosControl(app)  # type: ignore[arg-type]

    receipt = control.reset_session(1.0)
    assert receipt.session_id.startswith("code_policy_session_")
    assert control.interrupt_active(1.0)
    assert not control.motion_active(1.0)
    app.ReplanningAStarPlanner.state = NavigationState.FOLLOWING_PATH
    assert control.motion_active(1.0)
    control.cancel_motion(1.0)
    control.close()

    assert app.ReplanningAStarPlanner.cancelled
    assert app.stopped
