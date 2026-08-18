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

"""Start the isolated LeRobot runtime without loading a checkpoint."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.imitation.policy.lerobot.module import LeRobotPolicyModule


def run_example() -> None:
    coordinator = ModuleCoordinator.build(
        autoconnect(
            LeRobotPolicyModule.blueprint(
                policies={"smoke": {"policy_path": "unused-smoke-checkpoint"}},
                joint_names=["smoke/joint"],
            )
        )
    )
    try:
        module = coordinator.get_instance(LeRobotPolicyModule)
        status = module.policy_status()
        print("LeRobot Python-native runtime started.")
        print("Available policies:", ", ".join(status["available_policies"]))
        print("Observations ready:", str(status["observations_ready"]).lower())
        print("Observation status:", status["observation_error"])
    finally:
        coordinator.stop()
        print("LeRobot Python-native runtime stopped.")


if __name__ == "__main__":
    run_example()
